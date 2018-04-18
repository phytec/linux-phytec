/*	--*- c -*--
 * Copyright (C) 2016 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 and/or (at your option) version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/driver.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "vm050-regs.h"

#define INVALID_FOURCC		(~0u)

/* TODO: remove me in the final driver! */
#define DO_DEBUG		1

#if DO_DEBUG
#  define dtrace_printk(...)	trace_printk(__VA_ARGS__)
#else
#  define dtrace_printk(...)				\
	do {						\
		if (0)					\
			printk(KERN_DEBUG __VA_ARGS__);	\
	} while (0)
#endif

/* bulk io with big endian and regcache is broken :( */
#define REGMAP_BULK_IO_BROKEN	1

#define VM050_NUM_TRACK_TEMP	(4u)

static bool const	compat_compound_ctrl = true;

enum {
	V4L2_CID_X_SNAPSHOT_MODE = V4L2_CID_USER_CLASS + 1,
	V4L2_CID_X_AUTOSCALE_MODE,
	/* TODO: use V4L2_CID_EXPOSURE_METERING? */
	V4L2_CID_X_AUTOSCALE_METERING,
	V4L2_CID_X_AVERAGE,
	V4L2_CID_X_INTERFACE_PERFORMANCE,
	V4L2_CID_X_INTERFACE_ACCURACY,

	/* a control which documents 'V4L2_CID_X_TRACKING_MODE' values */
	V4L2_CID_X_TRACKING_MODE_MENU = V4L2_CID_X_AVERAGE + 19,
	V4L2_CID_X_TRACKING_MODE,
	V4L2_CID_X_TRACKING_PIXNUM,
	V4L2_CID_X_TRACKING_TEMP,
	V4L2_CID_X_TRACKING_FLAG,

	V4L2_CID_X_TEMP_WINDOW_MIN = V4L2_CID_X_TRACKING_FLAG + 20,
	V4L2_CID_X_TEMP_WINDOW_MAX,
	V4L2_CID_X_TEMP_VALUES,
	V4L2_CID_X_TEMP_SCALE,

	V4L2_CID_X_DEAD_PIXELS = V4L2_CID_X_TEMP_SCALE +20,
};

enum vm050_type {
	VM050_TYPE_HTPA32,
	VM050_TYPE_HTPA80,

	VM050_TYPE_MAX_,
};

enum {
	VM050_FLAG_STREAMING,
	VM050_FLAG_REG_ENABLED,
	VM050_FLAG_CLK_ENABLED,
	VM050_FLAG_PROBED,
};

struct vm050 {
	struct v4l2_subdev		subdev;
	struct i2c_client		*i2c;
	struct v4l2_ctrl_handler	ctrls;
	struct media_pad		pad;
	enum vm050_type			type;
	struct regmap *			regmap;
	struct gpio_chip		gc;

	struct clk			*clk;
	unsigned long			sysclk;
	struct regulator		*vdd;

	unsigned int			fmt_code;
	struct v4l2_rect		rect;

	unsigned long			flags;
	struct mutex			lock;

	atomic_t			power_cnt;

	/* shadow some bits of the control registers; the 'enable' and 'reset'
	 * bits will be set depending on the context */
	struct {
		uint16_t		r1[0x5 - 0x1];
		uint16_t		r13[1];
		uint16_t		o[1];
	}				reg_ctrl;

	/* shadow some registers which will be written directly */
	struct {
		uint16_t		r19[0x1a - 0x19];
	}				reg_shadow;

	bool				pixclk_inv:1;
};
#define sd_to_vm050(_sd)	container_of((_sd), struct vm050, subdev)
#define gc_to_vm050(_sd)	container_of((_sd), struct vm050, gc)

static unsigned int const	VM050_MBUS_CODES[] = {
	MEDIA_BUS_FMT_Y8_1X8,
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_Y16_2X8_LE, /* TODO: this needs a patch in generic v4l2 */
	MEDIA_BUS_FMT_Y16_1X16,	  /* HACK: will be translated to Y16_2X8_LE */
};

static struct vm050_sensor_info {
	unsigned int			min_width;
	unsigned int			max_width;

	unsigned int			min_height;
	unsigned int			max_height;
} const					SENSOR_INFO[] = {
	[VM050_TYPE_HTPA32] = { 8, 32,  8, 34 },
	[VM050_TYPE_HTPA80] = { 8, 80,  8, 66 },
};

struct regval {
	uint16_t		val;
	uint8_t			reg;
};

static struct vm050_sensor_info const *vm050_get_info(struct vm050 const *vm050)
{
	if (WARN(vm050->type >= ARRAY_SIZE(SENSOR_INFO),
		 "internal error; bad sensor type %d", vm050->type))
		return NULL;

	return &SENSOR_INFO[vm050->type];
}

static unsigned int  _vm050_reg_to_fourcc(struct vm050 *vm050, uint16_t regval)
{
	switch (regval & VM050_FLD_OUTPUT_CONTRL_DEPTH_msk) {
	case VM050_FLD_OUTPUT_CONTRL_DEPTH_8:
		return MEDIA_BUS_FMT_Y8_1X8;
	case VM050_FLD_OUTPUT_CONTRL_DEPTH_10:
		return MEDIA_BUS_FMT_Y10_1X10;
	case VM050_FLD_OUTPUT_CONTRL_DEPTH_12:
		return MEDIA_BUS_FMT_Y12_1X12;
	case VM050_FLD_OUTPUT_CONTRL_DEPTH_2x8:
		return MEDIA_BUS_FMT_Y16_2X8_LE;
	default:
		v4l2_warn(&vm050->subdev, "invalid depth register value %x\n",
			  regval);
		return INVALID_FOURCC;
	}
}

/* {{{ IO helper functions */
static int vm050_bulk_read(struct vm050 *vm050, unsigned int reg,
			   uint16_t data[], size_t cnt)
{
	int		rc;

	if (WARN_ON(cnt == 0)) {
		rc = -EINVAL;
	} else if (!REGMAP_BULK_IO_BROKEN && cnt > 1) {
		rc = regmap_bulk_read(vm050->regmap, reg, data, cnt);
	} else {
		size_t			i;

		for (i = 0; i < cnt; ++i) {
			unsigned int	tmp;

			rc = regmap_read(vm050->regmap, reg + i, &tmp);
			if (rc < 0)
				break;

			data[i] = tmp;
		}
	}

	if (DO_DEBUG && !rc) {
		size_t			i;

		for (i = 0; i < cnt; ++i)
			dtrace_printk("[%02x] => %04x\n", reg + i, data[i]);
	}

	return rc;
}

static int vm050_bulk_write(struct vm050 *vm050, unsigned int reg,
			    uint16_t const data[], size_t cnt)
{
	int		rc;

	if (WARN_ON(cnt == 0)) {
		rc = -EINVAL;
	} else if (!REGMAP_BULK_IO_BROKEN && cnt > 1) {
		rc = regmap_bulk_write(vm050->regmap, reg, data, cnt);
	} else {
		size_t			i;

		for (i = 0; i < cnt; ++i) {
			rc = regmap_write(vm050->regmap, reg + i, data[i]);
			if (rc < 0)
				break;
		}
	}

	if (DO_DEBUG && !rc) {
		size_t			i;

		for (i = 0; i < cnt; ++i)
			dtrace_printk("[%02x] <= %04x\n", reg + i, data[i]);
	}

	return rc;
}

static int vm050_readv(struct vm050 *vm050, struct regval rv[], size_t cnt)
{
	size_t		pos = 0;
	uint16_t	buf[16];
	int		rc;

	while (pos < cnt) {
		size_t	i;
		size_t	j;

		/* count number of consecutive register addresses */
		for (i = pos; i < cnt && rv[i].reg == rv[pos].reg; ++i)
			;		/* noop */

		/* limit it to the buffer size */
		i = min(i - pos, ARRAY_SIZE(buf));

		rc = vm050_bulk_read(vm050, rv[pos].reg, buf, i);
		if (rc < 0) {
			dev_err(&vm050->i2c->dev,
				"failed to read register %02x+%zu: %d\n",
				rv[pos].reg, i, rc);
			return rc;
		}

		for (j = 0; j < i; ++j)
			rv[pos + j].val = buf[j];

		pos += i;
	}

	return 0;
}

static int vm050_writev(struct vm050 *vm050,
			struct regval const rv[], size_t cnt)
{
	size_t		pos = 0;
	uint16_t	buf[16];
	int		rc;

	while (pos < cnt) {
		size_t	i;
		size_t	j;

		/* count number of consecutive register addresses */
		for (i = pos; i < cnt && rv[i].reg == rv[pos].reg; ++i)
			;		/* noop */

		/* limit it to the buffer size */
		i = min(i - pos, ARRAY_SIZE(buf));
		for (j = 0; j < i; ++j)
			buf[j] = rv[pos + j].val;

		rc = vm050_bulk_write(vm050, rv[pos].reg, buf, i);
		if (rc < 0) {
			dev_err(&vm050->i2c->dev,
				"failed to write register %02x+%zu: %d\n",
				rv[pos].reg, i, rc);
			return rc;
		}

		pos += i;
	}

	return 0;
}

static unsigned int vm050_read(struct vm050 *vm050, uint8_t reg, int *err)
{
	int		rc;
	unsigned int	v;

	if (*err != 0)
		/* noop when previous operations failed */
		return 0;

	rc = regmap_read(vm050->regmap, reg, &v);

	if (DO_DEBUG && !rc)
		dtrace_printk("[%02x] => %04x\n", reg, v);

	if (rc < 0) {
		dev_err(&vm050->i2c->dev, "failed to read register %02x: %d\n",
			reg, rc);
		*err = rc;
	}

	return v;
}

static void vm050_write(struct vm050 *vm050, uint8_t reg, uint16_t v, int *err)
{
	int		rc;

	if (*err != 0)
		/* noop when previous operations failed */
		return;

	rc = regmap_write(vm050->regmap, reg, v);

	if (DO_DEBUG && !rc)
		dtrace_printk("[%02x] <= %04x\n", reg, v);

	if (rc < 0) {
		dev_err(&vm050->i2c->dev,
			"failed to write register %02x with %04x: %d\n",
			reg, v, rc);
		*err = rc;
	}
}

static void vm050_mod(struct vm050 *vm050, uint8_t reg,
		      uint16_t mask, uint16_t set, int *err)
{
	int		rc;

	if (*err != 0)
		/* noop when previous operations failed */
		return;

	rc = regmap_update_bits(vm050->regmap, reg, mask, set);

	if (DO_DEBUG && !rc)
		dtrace_printk("[%02x] <= %04x & %04x\n", reg, set, mask);

	if (rc < 0) {
		dev_err(&vm050->i2c->dev,
			"failed to set register %02x with %04x%04x: %d\n",
			reg, set, mask, rc);
		*err = rc;
	}
}
/* }}} IO helper functions */

static void vm050_mod_cached(struct vm050 *vm050, uint8_t reg,
			     uint16_t mask, uint16_t set, int *err)
{
	uint16_t	*shadow;
	enum {
		OP_NONE,		/* noop; e.g. when not streaming */
		OP_WRITE,		/* write always */
		OP_MOD			/* modify bitmask */
	}		op = OP_MOD;
	bool		delayed;

	WARN_ON(!mutex_is_locked(&vm050->lock));

	if (*err != 0)
		/* noop when previous operations failed */
		return;

	if (WARN_ON(reg == VM050_REG_CONTROL1)) {
		/* REG_CONTROL1 is special; do not handle it here... */
		*err = -EINVAL;
		return;
	} else if (reg >= VM050_REG_CONTROL1 && reg <= VM050_REG_CONTROL5) {
		unsigned int		idx = reg - VM050_REG_CONTROL1;
		BUG_ON(idx >= ARRAY_SIZE(vm050->reg_ctrl.r1));
		shadow  = &vm050->reg_ctrl.r1[idx];
		op      = OP_MOD;
		delayed = false;
	} else if (reg == VM050_REG_CONTROL13) {
		shadow  = &vm050->reg_ctrl.r13[0];
		op      = OP_WRITE;
		delayed	= false;
	} else if (reg == VM050_REG_OUTPUT_CONTRL) {
		shadow  = &vm050->reg_ctrl.o[0];
		op      = OP_WRITE;
		delayed	= false;
	} else {
		shadow  = NULL;
		op      = OP_MOD;
		delayed = false;
	}

	if (shadow) {
		*shadow &= ~mask;
		*shadow |= set;
	}

	if (delayed && !test_bit(VM050_FLAG_STREAMING, &vm050->flags))
		op = OP_NONE;

	switch (op) {
	case OP_WRITE:
		vm050_write(vm050, reg, *shadow, err);
		break;

	case OP_MOD:
		vm050_mod(vm050, reg, mask, set, err);
		break;

	case OP_NONE:
		break;

	default:
		WARN(1, "unexpected op %d\n", op);
		break;
	}
}


static int vm050_flush_cached(struct vm050 *vm050)
{
#define RSET(_addr, _regs) \
	{ (_addr), (_regs), ARRAY_SIZE(_regs) }

	struct {
		unsigned int	addr;
		uint16_t const	*regs;
		size_t		len;
	} const		REGSET[] = {
		RSET(0x19, vm050->reg_shadow.r19),
	};

#undef RSET

	int			rc = 0;
	size_t			i;

	WARN_ON(!mutex_is_locked(&vm050->lock));

	for (i = 0; i < ARRAY_SIZE(REGSET); ++i) {
		rc = vm050_bulk_write(vm050, REGSET[i].addr,
				      REGSET[i].regs, REGSET[i].len);
		if (rc < 0) {
			v4l2_warn(&vm050->subdev,
				  "failed to write %02x+%zu: %d\n",
				  REGSET[i].addr, REGSET[i].len, rc);
			break;
		}
	}

	return rc;
}

static int _vm050_wait_resetbit(struct vm050 *vm050, uint16_t bit,
				unsigned long timeout, uint16_t *regval)
{
	int		rc;

	for (;;) {
		unsigned int		v;

		rc = regmap_read(vm050->regmap, VM050_REG_CONTROL1, &v);

		/* when value could be read and the self clearing reset bits
		 * are unset, abort the loop */
		if (rc == 0 && (v & bit) == 0) {
			if (regval)
				*regval = v;

			break;
		}

		if (time_after(jiffies, timeout)) {
			rc = -ETIMEDOUT;
			break;
		}

		usleep_range(1000, 5000);
	}

	printk("%s(%04x, %lu) -> %d (%04x)\n", __func__, bit, timeout, rc,
	       regval ? *regval : 0xffffu);

	return rc;
}

static int _vm050_reset(struct vm050 *vm050, uint16_t bit,
			unsigned int wait_ms, unsigned int timeout_ms)
{
	int		rc;
	unsigned long	timeout;
	uint16_t	tmp;


	rc = regmap_write(vm050->regmap, VM050_REG_CONTROL1,
			  vm050->reg_ctrl.r1[0] | bit);
	if (rc < 0)
		goto out;

	timeout = jiffies + msecs_to_jiffies(timeout_ms);

	msleep(wait_ms);

	rc = _vm050_wait_resetbit(vm050, bit, timeout, &tmp);

out:
	return rc;
}

static int vm050_reset(struct vm050 *vm050, bool hard_reset)
{
	int		rc;
	bool		sensor_powered = true;
	uint16_t const	reg_ctrl = vm050->reg_ctrl.r1[0];
	uint16_t	cur_ctrl;

	WARN_ON(!mutex_is_locked(&vm050->lock));
	WARN_ON(atomic_read(&vm050->power_cnt) == 0);
	WARN_ON(!(reg_ctrl & VM050_FLD_CONTROL1_POWERON));

	if (hard_reset) {
		rc = _vm050_reset(vm050, VM050_FLD_CONTROL1_MODULE_RESET, 50, 500);
		if (rc < 0)
			goto out;

		sensor_powered = false;
	}

	rc = _vm050_wait_resetbit(vm050, VM050_FLD_CONTROL1_SENSOR_RESET,
				  jiffies + msecs_to_jiffies(300), &cur_ctrl);
	if (rc < 0)
		goto out;

	/* when 'PERFORMANCE' bit is not as wanted, power off the sensor,
	 * change it and power on the sensor again */
	if (hard_reset && ((reg_ctrl ^ cur_ctrl) & VM050_FLD_CONTROL1_PERFORMANCE)) {
		/* power off the sensor; keep PERFORMANCE as-is */
		rc = regmap_write(vm050->regmap, VM050_REG_CONTROL1,
				  cur_ctrl & ~VM050_FLD_CONTROL1_POWERON);
		if (rc < 0)
			goto out;

		msleep(100);

		/* set PERFORMANCE bit to wanted state */
		rc = regmap_write(vm050->regmap, VM050_REG_CONTROL1,
				  reg_ctrl & ~VM050_FLD_CONTROL1_POWERON);
		if (rc < 0)
			goto out;

		/* and power it on (reg_ctrl must have POWERON bit set; see
		 * WARN_ON above) */
		rc = regmap_write(vm050->regmap, VM050_REG_CONTROL1, reg_ctrl);
		if (rc < 0)
			goto out;

		msleep(100);

		/* the power-off -> power-on caused a sensor reset; wait for
		 * it to be finished */
		rc = _vm050_wait_resetbit(vm050, VM050_FLD_CONTROL1_SENSOR_RESET,
					  jiffies + msecs_to_jiffies(300), NULL);
		if (rc < 0)
			goto out;
	}

out:
	if (rc < 0) {
		dev_warn(&vm050->i2c->dev, "failed to %sreset device: %d\n",
			 hard_reset ? "hard-" : "", rc);
		return rc;
	}

	return rc;
}

static void _vm050_power_off(struct vm050 *vm050)
{
	if (test_and_clear_bit(VM050_FLAG_CLK_ENABLED, &vm050->flags))
		clk_disable_unprepare(vm050->clk);

	if (test_and_clear_bit(VM050_FLAG_REG_ENABLED, &vm050->flags))
		regulator_disable(vm050->vdd);
}

static int vm050_power_on(struct vm050 *vm050)
{
	int		rc;

	WARN_ON(!mutex_is_locked(&vm050->lock));

	if (atomic_inc_return(&vm050->power_cnt) > 1) {
		rc = 0;
		goto out;
	}

	if (vm050->clk) {
		rc = clk_set_rate(vm050->clk, vm050->sysclk);
		if (rc < 0) {
			dev_warn(&vm050->i2c->dev,
				 "failed to set clk rate: %d\n", rc);
			goto out;
		}
	}

	if (vm050->vdd) {
		rc = regulator_enable(vm050->vdd);
		if (rc < 0) {
			dev_warn(&vm050->i2c->dev, "failed to enable regulator: %d\n", rc);
			goto out;
		}
		set_bit(VM050_FLAG_REG_ENABLED, &vm050->flags);
	}

	if (vm050->clk) {
		rc = clk_prepare_enable(vm050->clk);
		if (rc)
			goto out;
		set_bit(VM050_FLAG_CLK_ENABLED, &vm050->flags);
	}

	udelay(1);

	rc = vm050_reset(vm050, !test_bit(VM050_FLAG_PROBED, &vm050->flags));
	if (rc < 0)
		goto out;

	set_bit(VM050_FLAG_PROBED, &vm050->flags);

out:
	if (rc < 0) {
		_vm050_power_off(vm050);
		atomic_dec(&vm050->power_cnt);
	}

	return rc;
}

static void vm050_power_off(struct vm050 *vm050)
{
	WARN_ON(!mutex_is_locked(&vm050->lock));

	if (atomic_dec_return(&vm050->power_cnt) > 1)
		return;

	_vm050_power_off(vm050);
}

static int vm050_enum_mbus_code(struct v4l2_subdev *subdev,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > ARRAY_SIZE(VM050_MBUS_CODES))
		return -EINVAL;

	code->code = VM050_MBUS_CODES[code->index];

	return 0;
}

static int _vm050_get_fmt(struct vm050 *vm050,
			  struct v4l2_mbus_framefmt *fmt)
{
	unsigned int	depth;
	unsigned int	width;
	unsigned int	height;
	unsigned int	offs_x;
	unsigned int	offs_y;
	int		rc;
	unsigned int	code;

	/* NOTE: keep them in order so that regmap_bulk_read() can be used! */
	enum {
		O_OUT = 0,
		O_RES_X,
		O_OFS_X,
		O_RES_Y,
		O_OFS_Y,
	};

	struct regval	rv[] = {
		[O_OUT]   = { .reg = VM050_REG_OUTPUT_CONTRL },
		[O_RES_X] = { .reg = VM050_REG_RES_X },
		[O_RES_Y] = { .reg = VM050_REG_RES_Y },
		[O_OFS_X] = { .reg = VM050_REG_OFFSET_X },
		[O_OFS_Y] = { .reg = VM050_REG_OFFSET_Y },
	};

	rc = vm050_readv(vm050, rv, ARRAY_SIZE(rv));
	if (rc < 0)
		return rc;

	depth  = rv[O_OUT].val;
	width  = VM050_FLD_RES_X_get(rv[O_RES_X].val);
	height = VM050_FLD_RES_Y_get(rv[O_RES_Y].val);
	offs_x = VM050_FLD_OFFSET_X_get(rv[O_OFS_X].val);
	offs_y = VM050_FLD_OFFSET_Y_get(rv[O_OFS_Y].val);

	if (width < offs_x) {
		v4l2_warn(&vm050->subdev, "invalid offset %u and width %u\n",
			  offs_x, width);
		rc = -EINVAL;
	}

	if (height < offs_y) {
		v4l2_warn(&vm050->subdev, "invalid offset %u and height %u\n",
			  offs_y, height);
		rc = -EINVAL;
	}

	code = _vm050_reg_to_fourcc(vm050, depth);
	if (code == INVALID_FOURCC)
		return -EINVAL;

	*fmt = (struct v4l2_mbus_framefmt) {
		.width		= width - offs_x,
		.height		= height - offs_y,
		.code		= code,
		.field		= V4L2_FIELD_NONE,
	};

	return 0;
}

static int vm050_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct vm050			*vm050 = sd_to_vm050(sd);
	int				rc;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		format->format =
			*v4l2_subdev_get_try_format(sd, cfg, format->pad);
		rc  = 0;
	} else {
		rc = _vm050_get_fmt(vm050, &format->format);
	}

	return rc;
}

static int _vm050_refresh_format(struct vm050 *vm050)
{
	/* NOTE: keep them in order so that regmap_bulk_read() can be used! */
	enum {
		O_OUT = 0,
		O_RES_X,
		O_OFS_X,
		O_RES_Y,
		O_OFS_Y,
	};

	struct regval	rv[] = {
		[O_OUT]   = { .reg = VM050_REG_OUTPUT_CONTRL },
		[O_RES_X] = { .reg = VM050_REG_RES_X },
		[O_RES_Y] = { .reg = VM050_REG_RES_Y },
		[O_OFS_X] = { .reg = VM050_REG_OFFSET_X },
		[O_OFS_Y] = { .reg = VM050_REG_OFFSET_Y },
	};
	int		rc;
	unsigned int	code;


	WARN_ON(!mutex_is_locked(&vm050->lock));

	rc = vm050_readv(vm050, rv, ARRAY_SIZE(rv));
	if (rc < 0)
		return rc;

	code = _vm050_reg_to_fourcc(vm050, rv[O_OUT].val);
	if (code == INVALID_FOURCC)
		return -EINVAL;

	vm050->fmt_code = code;
	vm050->rect = (struct v4l2_rect){
		.width	= VM050_FLD_RES_X_get(rv[O_RES_X].val),
		.height	= VM050_FLD_RES_X_get(rv[O_RES_Y].val),
		.left	= VM050_FLD_RES_X_get(rv[O_OFS_X].val),
		.top	= VM050_FLD_RES_X_get(rv[O_OFS_Y].val),
	};

	return 0;
}

static int _vm050_set_format(struct vm050 *vm050, unsigned int code,
			     unsigned int left, unsigned int width,
			     unsigned int top, unsigned int height)
{
	struct vm050_sensor_info const	*info = vm050_get_info(vm050);
	struct regval		rv_buf[10];
	struct regval		*rv = rv_buf;
	int			rc;

	WARN_ON(!mutex_is_locked(&vm050->lock));

	if (!info)
		return -EINVAL;

	if (width < info->min_width ||
	    height < info->min_height ||
	    left > info->max_width - info->min_width ||
	    top > info->max_height - info->min_height ||
	    width > info->max_width  - left ||
	    height > info->max_height - top) {
		v4l2_warn(&vm050->subdev,
			  "invalid crop area: [%dx%d+%dx%d] not in [%dx%d..%dx%d]\n",
			  left, top, width, height,
			  info->min_width, info->min_height,
			  info->max_width, info->max_width);

		return -EINVAL;
	}

	vm050->reg_ctrl.o[0] &= ~VM050_FLD_OUTPUT_CONTRL_DEPTH_msk;
	switch (code) {
	case MEDIA_BUS_FMT_Y8_1X8:
		vm050->reg_ctrl.o[0] |= VM050_FLD_OUTPUT_CONTRL_DEPTH_8;
		break;
	case MEDIA_BUS_FMT_Y10_1X10:
		vm050->reg_ctrl.o[0] |= VM050_FLD_OUTPUT_CONTRL_DEPTH_10;
		break;
	case MEDIA_BUS_FMT_Y12_1X12:
		vm050->reg_ctrl.o[0] |= VM050_FLD_OUTPUT_CONTRL_DEPTH_12;
		break;
	case MEDIA_BUS_FMT_Y16_2X8_LE:
		vm050->reg_ctrl.o[0] |= VM050_FLD_OUTPUT_CONTRL_DEPTH_2x8;
		break;
	/* HACK: allow to use common YCrCb formats to select the Y16 mode */
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_2X8:
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		vm050->reg_ctrl.o[0] |= VM050_FLD_OUTPUT_CONTRL_DEPTH_2x8;
		break;
	default:
		v4l2_warn(&vm050->subdev, "invalid format %04x\n", code);
		return -EINVAL;
	}

	/* NOTE: fill 'rv' in order to allow regmap_bulk_write()! */
	*rv++ = (struct regval) {
		.reg = VM050_REG_OUTPUT_CONTRL,
		.val = vm050->reg_ctrl.o[0],
	};

	*rv++ = (struct regval) {
		.reg = VM050_REG_RES_X,
		.val = VM050_FLD_RES_X(width),
	};

	*rv++ = (struct regval) {
		.reg = VM050_REG_OFFSET_X,
		.val = VM050_FLD_OFFSET_X(left),
	};

	*rv++ = (struct regval) {
		.reg = VM050_REG_RES_Y,
		.val = VM050_FLD_RES_Y(height),
	};

	*rv++ = (struct regval) {
		.reg = VM050_REG_OFFSET_Y,
		.val = VM050_FLD_OFFSET_Y(top),
	};

	rc = vm050_writev(vm050, rv_buf, rv - rv_buf);
	if (rc < 0)
		return rc;

	vm050->fmt_code = code;
	vm050->rect = (struct v4l2_rect) {
		.left	= left,
		.top	= top,
		.width	= width,
		.height	= height,
	};

	return 0;
}

static int vm050_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct vm050			*vm050 = sd_to_vm050(sd);
	struct vm050_sensor_info const	*info = vm050_get_info(vm050);
	int				rc;
	unsigned int			width;
	unsigned int			height;
	unsigned int			code;

	if (!info)
		return -EINVAL;

	switch (format->format.code) {
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_Y12_1X12:
	case MEDIA_BUS_FMT_Y16_2X8_LE:
		code = format->format.code;
		break;

	case MEDIA_BUS_FMT_Y16_1X16:	/* TODO: this is broken */
		code = MEDIA_BUS_FMT_Y16_2X8_LE;
		format->format.code = code; /* prevent -EINVAL below */
		break;

	default:
		code = MEDIA_BUS_FMT_Y8_1X8;
		break;
	}

	mutex_lock(&vm050->lock);

	width = clamp(format->format.width, info->min_width,
		      info->max_width - vm050->rect.left);

	height = clamp(format->format.height, info->min_height,
		       info->max_height - vm050->rect.top);

	if (height == info->max_height)
		/* handle embedded-info case */
		width = info->max_width;

	if (format->format.field == V4L2_FIELD_ANY)
		format->format.field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		format->format.width  = width;
		format->format.height = height;
		format->format.code   = code;
		format->format.field  = V4L2_FIELD_NONE;

		*v4l2_subdev_get_try_format(sd, cfg, format->pad) =
			format->format;

		rc = 0;
	} else if (format->format.width != width ||
		   format->format.height != height ||
		   format->format.code != code ||
		   format->format.field != V4L2_FIELD_NONE) {
		v4l2_warn(sd,
			  "format mismatch (%dx%d@%04x/%d) vs. (%dx%d@%04x/%d); offset %dx%d\n",
			  format->format.width, format->format.height,
			  format->format.code, format->format.field,
			  width, height, code, V4L2_FIELD_NONE,
			  vm050->rect.left, vm050->rect.top);

		rc = -EINVAL;
	} else if (test_bit(VM050_FLAG_STREAMING, &vm050->flags)) {
		v4l2_warn(sd, "can not change format while streaming\n");
		rc = -EBUSY;
	} else {
		rc = _vm050_set_format(vm050, code,
				       vm050->rect.left, width,
				       vm050->rect.top, height);
	}

	mutex_unlock(&vm050->lock);

	return rc;
}

static int vm050_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct vm050			*vm050 = sd_to_vm050(sd);
	struct vm050_sensor_info const	*info = vm050_get_info(vm050);
	struct v4l2_rect		*rect;
	int				rc;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect) {
			.left	= 0,
			.width	= info->max_width,
			.top	= 0,
			.height	= info->max_height
		};
		rc = 0;
		break;

	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = (struct v4l2_rect) {
			.left	= 0,
			.width	= info->max_width,
			.top	= 0,
			.height	= info->max_height - 2,
		};
		rc = 0;
		break;

	case V4L2_SEL_TGT_CROP:
		mutex_lock(&vm050->lock);

		switch (sel->which) {
		case V4L2_SUBDEV_FORMAT_TRY:
			rect = v4l2_subdev_get_try_crop(&vm050->subdev, cfg,
							sel->pad);
			break;

		case V4L2_SUBDEV_FORMAT_ACTIVE:
			rc = _vm050_refresh_format(vm050);
			if (rc < 0)
				rect = ERR_PTR(rc);
			else
				rect = &vm050->rect;

			break;

		default:
			rect = ERR_PTR(-EINVAL);
		}

		if (!rect) {
			rc = -EINVAL;
		} else if (IS_ERR(rect)) {
			rc = PTR_ERR(rect);
		} else {
			sel->r = *rect;
			rc = 0;
		}

		mutex_unlock(&vm050->lock);

		break;

	default:
		rc = -EINVAL;
	}

	return rc;
}

static int vm050_set_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct vm050			*vm050 = sd_to_vm050(sd);
	struct vm050_sensor_info const	*info = vm050_get_info(vm050);
	unsigned int			left;
	unsigned int			top;
	unsigned int			width;
	unsigned int			height;
	struct v4l2_rect		*rect;
	int				rc;

	if (!info)
		return -EINVAL;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	left   = clamp_t(unsigned int, sel->r.left,
			0u, info->max_width - info->min_width);
	top    = clamp_t(unsigned int, sel->r.top,
			0u, info->max_height - info->min_height);
	width  = clamp_t(unsigned int, sel->r.width,
			info->min_width, info->max_width - left);
	height = clamp_t(unsigned int, sel->r.height,
			info->min_height, info->max_height - top);

	if (height == info->max_height) {
		/* handle embedded-info case */
		width = info->max_width;
		left  = 0;
	}

	mutex_lock(&vm050->lock);

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		sel->r = (struct v4l2_rect) {
			.left	= left,
			.top	= top,
			.width	= width,
			.height	= height
		};

		rect = v4l2_subdev_get_try_crop(&vm050->subdev, cfg, sel->pad);
		rc = 0;
	} else if (sel->r.left != left || sel->r.width != width ||
		   sel->r.top != top || sel->r.height != height) {
		v4l2_warn(sd, "crop mismatch (%dx%d+%dx%d) vs. (%dx%d+%dx%d)\n",
			  sel->r.left, sel->r.top, sel->r.width, sel->r.height,
			  left, top, width, height);
		rc = -EINVAL;
	} else if (test_bit(VM050_FLAG_STREAMING, &vm050->flags) &&
		   (vm050->rect.width != width ||
		    vm050->rect.height != height)) {
		v4l2_warn(sd, "can not change crop dimension while streaming\n");
		rc = -EBUSY;
	} else {
		rc = _vm050_set_format(vm050, vm050->fmt_code,
				       left, width, top, height);
	}

	mutex_unlock(&vm050->lock);

	return rc;
}


static struct v4l2_subdev_pad_ops const		vm050_pad_ops = {
	.enum_mbus_code		= vm050_enum_mbus_code,
	.get_fmt		= vm050_get_fmt,
	.set_fmt		= vm050_set_fmt,
	.get_selection		= vm050_get_selection,
	.set_selection		= vm050_set_selection,
};

/* {{{ core ops */
#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vm050_g_register(struct v4l2_subdev *sd,
			    struct v4l2_dbg_register *reg)
{
	struct vm050	*vm050 = sd_to_vm050(sd);
	int		rc = 0;

	reg->size = 2;
	reg->val = vm050_read(vm050, reg->reg, &rc);

	return rc;
}

static int vm050_s_register(struct v4l2_subdev *sd,
			    struct v4l2_dbg_register const *reg)
{
	struct vm050	*vm050 = sd_to_vm050(sd);
	int		rc = 0;

	if (reg->size != 2)
		return -EINVAL;

	vm050_write(vm050, reg->reg, reg->val, &rc);

	return rc;
}
#endif

static int vm050_querycap(struct vm050 *vm050, struct v4l2_capability *cap)
{
	strcpy(cap->driver, "phytec-vm050");

	return 0;
}

static long vm050_core_ioctl(struct v4l2_subdev *sd,
			     unsigned int cmd, void *arg)
{
	struct vm050	*vm050 = sd_to_vm050(sd);

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		return vm050_querycap(vm050, arg);

	default:
		return -ENOTTY;
	}
}

static struct v4l2_subdev_core_ops const	vm050_core_ops = {
	.ioctl			= vm050_core_ioctl,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= vm050_g_register,
	.s_register		= vm050_s_register,
#endif
};
/* }}} core ops */

/* {{{ video ops */
static int vm050_video_g_mbus_config(struct v4l2_subdev *sd,
				     struct v4l2_mbus_config *cfg)
{
	struct vm050	*vm050 = sd_to_vm050(sd);

	cfg->flags  = (V4L2_MBUS_MASTER |
		       V4L2_MBUS_HSYNC_ACTIVE_HIGH |
		       V4L2_MBUS_VSYNC_ACTIVE_HIGH |
		       V4L2_MBUS_DATA_ACTIVE_HIGH);

	if (!(vm050->reg_ctrl.o[0] & VM050_FLD_OUTPUT_PCLK_POL) !=
	    !!vm050->pixclk_inv)
		cfg->flags |= V4L2_MBUS_PCLK_SAMPLE_FALLING;
	else
		cfg->flags |= V4L2_MBUS_PCLK_SAMPLE_RISING;

	cfg->type   = V4L2_MBUS_PARALLEL;

	return 0;
}

static int _vm050_video_s_stream_on(struct vm050 *vm050)
{
	/* although we do not modify bits in some registers, we have to write
	 * it here because it can be altered by the control handler (which
	 * writes to shadow registers outside of streaming) */
	struct regval const		rv[] = {
		{
			.reg	= VM050_REG_CONTROL4,
			.val	= vm050->reg_ctrl.r1[3],
		}, {
			.reg	= VM050_REG_CONTROL3,
			.val	= vm050->reg_ctrl.r1[2],
		}, {
			.reg	= VM050_REG_CONTROL2,
			.val	= vm050->reg_ctrl.r1[1],
		}, {
			.reg	= VM050_REG_CONTROL1,
			.val	= (vm050->reg_ctrl.r1[0] |
				   VM050_FLD_CONTROL1_ENABLE),
		},
	};
	int			rc;

	if (test_and_set_bit(VM050_FLAG_STREAMING, &vm050->flags))
		return -EBUSY;

	rc = vm050_flush_cached(vm050);
	if (!rc)
		rc = vm050_writev(vm050, rv, ARRAY_SIZE(rv));

	if (rc < 0)
		goto out;

out:
	if (rc < 0)
		clear_bit(VM050_FLAG_STREAMING, &vm050->flags);

	return rc;
}


static int _vm050_video_s_stream_off(struct vm050 *vm050)
{
	struct regval const		rv[] = {
		{
			.reg	= VM050_REG_CONTROL1,
			.val	= vm050->reg_ctrl.r1[0],
		}
	};
	int			rc;

	if (!test_and_clear_bit(VM050_FLAG_STREAMING, &vm050->flags))
		return -EINVAL;

	rc = vm050_writev(vm050, rv, ARRAY_SIZE(rv));
	if (rc < 0)
		goto out;


out:
	return rc;
}

static int vm050_video_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vm050	*vm050 = sd_to_vm050(sd);
	int		rc;

	mutex_lock(&vm050->lock);

	if (enable)
		rc = _vm050_video_s_stream_on(vm050);
	else
		rc = _vm050_video_s_stream_off(vm050);

	mutex_unlock(&vm050->lock);

	return rc;
}

static struct v4l2_subdev_video_ops const	vm050_subdev_video_ops = {
	.g_mbus_config		= vm050_video_g_mbus_config,
	.s_stream		= vm050_video_s_stream,
};
/* }}} video ops */

static struct v4l2_subdev_ops const		vm050_subdev_ops = {
	.core			= &vm050_core_ops,
	.pad			= &vm050_pad_ops,
	.video			= &vm050_subdev_video_ops,
};

static char const * const			vm050_test_pattern_menu[] = {
	"disabled",
	"incrementation",
	"walking bit",
	"horizontal bars",
	"vertical bars",
	"mosaic",
};

static char const * const			vm050_autoscale_mode_menu[] = {
	"disabled",
	"dynamic",
	"min",
	"max",
	"mean",
	"temp0",
};

static char const * const			vm050_autoscale_metering_menu[] = {
	"reserved",
	"image",
	"center",
	"spot",
};

static char const * const			vm050_average_menu[] = {
	"none",
	"weak",
	"medium",
	"strong",
};

static char const * const			vm050_snapshot_mode_menu[] = {
	"continuous",
	"trigger",
	"manual",
};

static char const * const			vm050_tracking_mode_menu[] = {
	"disabled",
	"eq",
	"le",
	"ge",
};

static int vm050_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vm050		*vm050 = ctrl->priv;
	int			rc = 0;
	size_t			i;

	mutex_lock(&vm050->lock);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		vm050_mod_cached(vm050, VM050_REG_CONTROL3,
				 VM050_FLD_CONTROL3_MIRROR_X,
				 ctrl->val ? VM050_FLD_CONTROL3_MIRROR_X : 0,
				 &rc);
		break;

	case V4L2_CID_VFLIP:
		vm050_mod_cached(vm050, VM050_REG_CONTROL3,
				 VM050_FLD_CONTROL3_MIRROR_Y,
				 ctrl->val ? VM050_FLD_CONTROL3_MIRROR_Y : 0,
				 &rc);
		break;

	case V4L2_CID_FLASH_STROBE:
		vm050_mod_cached(vm050, VM050_REG_CONTROL2,
				 VM050_FLD_CONTROL2_STROBE_msk,
				 ctrl->val ?
				 VM050_FLD_CONTROL2_STROBE_ENABLE :
				 VM050_FLD_CONTROL2_STROBE_DISABLE,
				 &rc);
		break;

	case V4L2_CID_X_AUTOSCALE_MODE:
		vm050_mod_cached(vm050, VM050_REG_CONTROL2,
				 VM050_FLD_CONTROL2_AUTOSCALE_msk,
				 VM050_FLD_CONTROL2_AUTOSCALE(ctrl->val), &rc);
		break;

	case V4L2_CID_X_AUTOSCALE_METERING:
		vm050_mod_cached(vm050, VM050_REG_CONTROL2,
				 VM050_FLD_CONTROL2_METERING_msk,
				 VM050_FLD_CONTROL2_METERING(ctrl->val), &rc);
		break;

	case V4L2_CID_X_AVERAGE:
		vm050_mod_cached(vm050, VM050_REG_CONTROL3,
				 VM050_FLD_CONTROL3_AVG_msk,
				 VM050_FLD_CONTROL3_AVG(ctrl->val), &rc);
		break;

	case V4L2_CID_X_INTERFACE_PERFORMANCE:
		vm050_mod_cached(vm050, VM050_REG_OUTPUT_CONTRL,
				 VM050_FLD_OUTPUT_PERFORMANCE,
				 ctrl->val ? VM050_FLD_OUTPUT_PERFORMANCE : 0,
				 &rc);
		break;

	case V4L2_CID_X_INTERFACE_ACCURACY:
		vm050_mod_cached(vm050, VM050_REG_OUTPUT_CONTRL,
				 VM050_FLD_OUTPUT_ACCURACY,
				 ctrl->val ? VM050_FLD_OUTPUT_ACCURACY : 0,
				 &rc);
		break;

	case V4L2_CID_X_SNAPSHOT_MODE:
		vm050_mod_cached(vm050, VM050_REG_CONTROL2,
				 VM050_FLD_CONTROL2_MODE_msk,
				 VM050_FLD_CONTROL2_MODE(ctrl->val), &rc);

		if (ctrl->val == 0 &&
		    test_bit(VM050_FLAG_STREAMING, &vm050->flags))
			/* set 'enable sensor' bit again when streaming;
			 * snapshot modes might have disabled it */
			vm050_write(vm050, VM050_REG_CONTROL1,
				    vm050->reg_ctrl.r1[0] |
				    VM050_FLD_CONTROL1_ENABLE, &rc);

		break;

	case V4L2_CID_TEST_PATTERN:
		if (WARN_ON(ctrl->val > ARRAY_SIZE(vm050_test_pattern_menu))) {
			rc = -EINVAL;
			goto out;
		}

		vm050_mod_cached(vm050, VM050_REG_CONTROL3,
				 VM050_FLD_CONTROL3_TEST_msk,
				 VM050_FLD_CONTROL3_TEST(ctrl->val),
				 &rc);

		break;

	case V4L2_CID_X_TEMP_WINDOW_MAX:
		vm050_mod_cached(vm050, VM050_REG_TEMP_WINDOW_MAX, 0xffffu,
				 ctrl->val, &rc);
		break;

	case V4L2_CID_X_TEMP_WINDOW_MIN:
		vm050_mod_cached(vm050, VM050_REG_TEMP_WINDOW_MIN, 0xffffu,
				 ctrl->val, &rc);
		break;

	case V4L2_CID_X_TRACKING_TEMP:
		for (i = 0; i < ctrl->dims[0]; ++i) {
			u16	v = ctrl->p_new.p_u16[i];

			vm050_mod_cached(vm050, VM050_REG_TRACK_TEMP(i),
					 0xffffu, v, &rc);
		}
		break;

	case V4L2_CID_X_TRACKING_PIXNUM:
		for (i = 0; i < ctrl->dims[0]; ++i) {
			u16	v = ctrl->p_new.p_u16[i];

			vm050_mod_cached(vm050, VM050_REG_TRACK_TEMP_CONF(i),
					 VM050_FLD_TRACK_TEMP_CONF_PIXNUM_msk,
					 VM050_FLD_TRACK_TEMP_CONF_PIXNUM(v),
					 &rc);
		}
		break;

	case V4L2_CID_X_TRACKING_MODE:
		for (i = 0; i < ctrl->dims[0]; ++i) {
			s32	v = ctrl->p_new.p_s32[i];

			vm050_mod_cached(vm050, VM050_REG_TRACK_TEMP_CONF(i),
					 VM050_FLD_TRACK_TEMP_CONF_MODE_msk,
					 VM050_FLD_TRACK_TEMP_CONF_MODE(v),
					 &rc);
		}
		break;

	default:
		v4l2_warn(&vm050->subdev, "unsupported control %d\n", ctrl->id);
		rc = -EINVAL;
	}

out:

	mutex_unlock(&vm050->lock);

	return rc;
}

static int vm050_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vm050		*vm050 = ctrl->priv;
	union {
		uint16_t	temps[VM050_NUM_TRACK_TEMP];
		uint16_t	scale[2];
		uint16_t	u16;
	}			val;
	int			rc;
	uint32_t		tmp;
	size_t			i;

	mutex_lock(&vm050->lock);

	BUILD_BUG_ON(VM050_REG_TEMP_MAX    != VM050_REG_TEMP_MIN + 1);
	BUILD_BUG_ON(VM050_REG_TEMP_MEAN   != VM050_REG_TEMP_MIN + 2);
	BUILD_BUG_ON(VM050_REG_TEMP_CENTER != VM050_REG_TEMP_MIN + 3);

	BUILD_BUG_ON(VM050_REG_TEMP_WINDOW_SCALE_LO != VM050_REG_TEMP_WINDOW_SCALE_HI + 1);

	switch (ctrl->id) {
	case V4L2_CID_X_TEMP_WINDOW_MIN:
		rc = vm050_bulk_read(vm050, VM050_REG_TEMP_MIN, &val.u16, 1);
		if (rc < 0)
			goto out;

		ctrl->val = val.u16;
		break;

	case V4L2_CID_X_TEMP_WINDOW_MAX:
		rc = vm050_bulk_read(vm050, VM050_REG_TEMP_MAX, &val.u16, 1);
		if (rc < 0)
			goto out;

		ctrl->val = val.u16;
		break;

	case V4L2_CID_X_TEMP_VALUES:
		rc = vm050_bulk_read(vm050, VM050_REG_TEMP_MIN, val.temps,
				     ARRAY_SIZE(val.temps));
		if (rc < 0)
			goto out;

		/* TODO: fix endianess? */
		memcpy(ctrl->p_new.p_u16, val.temps, sizeof val.temps);
		rc = 0;
		break;

	case V4L2_CID_X_TRACKING_FLAG:
		rc = vm050_bulk_read(vm050, VM050_REG_TRACK_TEMP_CONF(0),
				     val.temps, ARRAY_SIZE(val.temps));
		if (rc < 0)
			goto out;

		for (i = 0; i < ARRAY_SIZE(val.temps); ++i)
			ctrl->p_new.p_u32[i] = !!(val.temps[i] &
						  VM050_FLD_TRACK_TEMP_CONF_FLAG);

		break;

	case V4L2_CID_X_TEMP_SCALE:
		rc = vm050_bulk_read(vm050, VM050_REG_TEMP_WINDOW_SCALE_HI,
				     val.scale, ARRAY_SIZE(val.scale));
		if (rc < 0)
			goto out;

		tmp   = val.scale[0];
		tmp <<= 16;
		tmp  |= val.scale[1];

		*ctrl->p_new.p_s64 = tmp;
		break;

	default:
		v4l2_warn(&vm050->subdev, "unsupported control %d\n", ctrl->id);
		rc = -EINVAL;
	}

out:

	mutex_unlock(&vm050->lock);

	return rc;
}

static struct v4l2_ctrl_ops const		vm050_ctrl_ops = {
	.s_ctrl			= vm050_s_ctrl,
	.g_volatile_ctrl	= vm050_g_ctrl,
};

static struct v4l2_ctrl_config const		vm050_ctrls[] = {
	{
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.def		= 1,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_FLASH_STROBE,
		.type		= V4L2_CTRL_TYPE_BUTTON,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_AUTOSCALE_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Autoscale mode",
		.min		= 0,
		.def		= 1,
		.max		= ARRAY_SIZE(vm050_autoscale_mode_menu) - 1,
		.qmenu		= vm050_autoscale_mode_menu,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_AUTOSCALE_METERING,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Autoscale metering",
		.min		= 0,
		.max		= ARRAY_SIZE(vm050_autoscale_metering_menu) - 1,
		.def		= 1,
		.qmenu		= vm050_autoscale_metering_menu,
		.menu_skip_mask = BIT(0),
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_AVERAGE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Average",
		.min		= 0,
		.max		= ARRAY_SIZE(vm050_average_menu) - 1,
		.qmenu		= vm050_average_menu,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_INTERFACE_PERFORMANCE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "interface performance",
		.min		= 0,
		.max		= 1,
		.step		= 1,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_INTERFACE_ACCURACY,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "interface accuracy",
		.min		= 0,
		.max		= 1,
		.step		= 1,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_SNAPSHOT_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Snapshot mode",
		.min		= 0,
		.max		= ARRAY_SIZE(vm050_snapshot_mode_menu) - 1,
		.qmenu		= vm050_snapshot_mode_menu,
		.menu_skip_mask = BIT(2),
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Test Pattern",
		.min		= 0,
		.max		= ARRAY_SIZE(vm050_test_pattern_menu) - 1,
		.qmenu		= vm050_test_pattern_menu,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TEMP_WINDOW_MIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "min temperature",
		.flags		= (V4L2_CTRL_FLAG_EXECUTE_ON_WRITE |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 2500,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TEMP_WINDOW_MAX,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "max temperature",
		.flags		= (V4L2_CTRL_FLAG_EXECUTE_ON_WRITE |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 0xffff,
		.step		= 1,
		.def		= 3500,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TRACKING_PIXNUM,
		.type		= V4L2_CTRL_TYPE_U16,
		.name		= "track-pixel",
		.min		= 0,
		.max		= 0,	/* will be filled manually */
		.step		= 1,
		.dims		= { VM050_NUM_TRACK_TEMP },
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TRACKING_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "tracking mode",
		.min		= 0,
		.max		= ARRAY_SIZE(vm050_tracking_mode_menu) - 1,
		.step		= 1,
		.qmenu		= vm050_tracking_mode_menu,
		.dims		= { VM050_NUM_TRACK_TEMP },
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TRACKING_TEMP,
		.type		= V4L2_CTRL_TYPE_U16,
		.name		= "tracking temperature",
		.min		= 0,
		.max		= 0xffff,
		.step		= 1,
		.dims		= { VM050_NUM_TRACK_TEMP },
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TEMP_VALUES,
		.type		= V4L2_CTRL_TYPE_U16,
		.name		= "temperatures",
		.flags		= (V4L2_CTRL_FLAG_READ_ONLY |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 0xffff,
		.step		= 1,
		.dims		= { 4 },
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TRACKING_FLAG,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "tracking flag",
		.flags		= (V4L2_CTRL_FLAG_READ_ONLY |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.dims		= { VM050_NUM_TRACK_TEMP },
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TRACKING_MODE_MENU,
		.type		= V4L2_CTRL_TYPE_MENU,
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
		.name		= "tracking mode menu",
		.min		= 0,
		.max		= ARRAY_SIZE(vm050_tracking_mode_menu) - 1,
		.qmenu		= vm050_tracking_mode_menu,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_TEMP_SCALE,
		/* TODO: really integeer? */
		.type		= V4L2_CTRL_TYPE_INTEGER64,
		.name		= "temp scaling factor",
		.flags		= (V4L2_CTRL_FLAG_READ_ONLY |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 0xffffffff,
		.step		= 1,
	}, {
		.ops		= &vm050_ctrl_ops,
		.id		= V4L2_CID_X_DEAD_PIXELS,
		.type		= V4L2_CTRL_TYPE_U16,
		.name		= "dead pixels",
		.flags		= V4L2_CTRL_FLAG_READ_ONLY,
		.min		= 0,
		.max		= 0xffff,
		.step		= 1,
	}
};

/* {{{ internal ops */
static int vm050_setup_ctrls(struct vm050 *vm050)
{
	size_t		i;
	int		rc = 0;

	uint16_t	ctrl3;
	uint16_t	dead_pixels[VM050_NUM_DEAD_PIX_MAX];
	unsigned int	num_dead_pixel;

	ctrl3 = vm050_read(vm050, VM050_REG_CONTROL3, &rc);
	if (rc == 0)
		rc = vm050_bulk_read(vm050, VM050_REG_DEAD_PIX_ADDR(0),
				     dead_pixels, ARRAY_SIZE(dead_pixels));

	if (rc < 0) {
		v4l2_err(&vm050->subdev,
			 "failed to read dead pixel information: %d\n", rc);
		goto out;
	}

	num_dead_pixel = VM050_FLD_CONTROL3_DEAD_PIXEL_get(ctrl3);
	vm050->reg_ctrl.r1[2] &= ~VM050_FLD_CONTROL3_DEAD_PIXEL_msk;
	vm050->reg_ctrl.r1[2] |= VM050_FLD_CONTROL3_DEAD_PIXEL(num_dead_pixel);

	for (i = 0; i < ARRAY_SIZE(vm050_ctrls); ++i) {
		struct v4l2_ctrl_config		ctrl = vm050_ctrls[i];
		struct v4l2_ctrl		*cptr;

		switch (ctrl.id) {
		case V4L2_CID_X_TRACKING_PIXNUM:
			switch (vm050->type) {
			case VM050_TYPE_HTPA32:
				ctrl.max = 1023;
				break;

			case VM050_TYPE_HTPA80:
				ctrl.max = 5119;
				break;

			case VM050_TYPE_MAX_:
				BUG();
			}
			break;

		case V4L2_CID_X_DEAD_PIXELS:
			if (num_dead_pixel == 0)
				continue;

			ctrl.dims[0] = num_dead_pixel;
			break;
		}

		if (ctrl.dims[0] > 0 && compat_compound_ctrl) {
			switch (ctrl.type) {
			case V4L2_CTRL_TYPE_BOOLEAN:
			case V4L2_CTRL_TYPE_MENU:
				ctrl.type = V4L2_CTRL_TYPE_U32;
				break;

			default:
				break;
			}
		}

		cptr = v4l2_ctrl_new_custom(&vm050->ctrls, &ctrl, vm050);
		rc = vm050->ctrls.error;
		if (rc < 0) {
			v4l2_warn(&vm050->subdev,
				  "failed to register control#%zu '%s': %d\n",
				  i, ctrl.name, rc);
			goto out;
		}

		switch (ctrl.id) {
		case V4L2_CID_X_DEAD_PIXELS: {
			unsigned int	j;

			for (j = 0; j < num_dead_pixel; ++j)
				cptr->p_cur.p_u16[j] = dead_pixels[j];

			break;
		}

		default:
			break;
		}
	}

out:
	return rc;
}

static int vm050_registered(struct v4l2_subdev *sd)
{
	static char const * const	TABLE_TYPES[] = {
		[0] = "",
		[1] = " precise",
		[2] = "R1",
		[3] = "R1 precise"
	};
	struct vm050	*vm050 = sd_to_vm050(sd);
	struct regval	rv[] = {
		/* NOTE: keep them in order so that regmap_bulk_read() can be
		 * used! */
		[0] = { .reg = VM050_REG_SENSOR_TYPE },
		[1] = { .reg = VM050_REG_FIRMARE },
		[2] = { .reg = VM050_REG_SENSOR_ID_HI },
		[3] = { .reg = VM050_REG_SENSOR_ID_LO},
		[4] = { .reg = VM050_REG_SERIAL_HI },
		[5] = { .reg = VM050_REG_SERIAL_LO},
	};
	char const	*id_str;
	char const	*table_str;
	int		rc;

	mutex_lock(&vm050->lock);

	rc = vm050_power_on(vm050);
	if (rc < 0)
		goto out_nopower;

	rc = vm050_readv(vm050, rv, ARRAY_SIZE(rv));
	if (rc < 0)
		goto out;

	switch ((rv[0].val & VM050_FLD_SENSOR_TYPE_TYPE_msk)) {
	case VM050_FLD_SENSOR_TYPE_TYPE_HTPA32:
		vm050->type = VM050_TYPE_HTPA32;
		id_str = "VM-050";
		break;

	case VM050_FLD_SENSOR_TYPE_TYPE_HTPA80:
		vm050->type = VM050_TYPE_HTPA80;
		id_str = "VM-051";
		break;

	default:
		dev_warn(&vm050->i2c->dev, "unsupported sensor type %04x\n",
			 rv[0].val);
		rc = -ENODEV;
		goto out;
	}

	table_str = TABLE_TYPES[VM050_FLD_SENSOR_TYPE_TABLE_get(rv[0].val) % 4];

	_vm050_refresh_format(vm050);

	/* TODO: this mutex_lock() block is hacky */
	mutex_unlock(&vm050->lock);
	rc = vm050_setup_ctrls(vm050);
	if (rc >= 0)
		rc = v4l2_ctrl_handler_setup(&vm050->ctrls);
	mutex_lock(&vm050->lock);
	if (rc < 0) {
		v4l2_warn(sd, "failed to set controls: %d\n", rc);
		goto out;
	}

	v4l2_info(sd,
		  "%s%s with firmware %d.%03d, device id %04x%04x, serial %04x%04x\n",
		  id_str, table_str,
		  VM050_FLD_FIRMWARE_MAJOR_get(rv[1].val),
		  VM050_FLD_FIRMWARE_MINOR_get(rv[1].val),
		  rv[2].val, rv[3].val, rv[4].val, rv[5].val);


	rc = 0;

out:
	if (rc < 0)
		vm050_power_off(vm050);

out_nopower:
	mutex_unlock(&vm050->lock);

	return rc;
}

static void vm050_unregistered(struct v4l2_subdev *sd)
{
	struct vm050	*vm050 = sd_to_vm050(sd);

	mutex_lock(&vm050->lock);
	vm050_power_off(vm050);
	mutex_unlock(&vm050->lock);
}

static struct v4l2_subdev_internal_ops const	vm050_internal_ops = {
	.registered		= vm050_registered,
	.unregistered		= vm050_unregistered,
};
/* }}} internal ops */

/* {{{ regmap setup */
static struct regmap_range const	vm050_regmap_wr_ranges[] = {
	{ 0x02, 0x05 },
	{ 0x0f, 0x13 },
	{ 0x19, 0x1a },
	{ 0x20, 0x27 },
	{ 0x32, 0x32 },
};

static struct regmap_range const	vm050_regmap_rd_ranges[] = {
	{ 0x00, 0x13 },
	{ 0x17, 0x1e },
	{ 0x20, 0x27 },
	{ 0x2b, 0x32 },
	{ 0x40, 0x46 },
	{ 0x4a, 0x61 },
};

static struct regmap_range const	vm050_regmap_volatile_ranges[] = {
	{ VM050_REG_CONTROL1, VM050_REG_CONTROL1 },
	{ VM050_REG_CONTROL13, VM050_REG_CONTROL13 },
	{ VM050_REG_TRACK_TEMP_CONF(0), VM050_REG_TRACK_TEMP_CONF(3) },
	{ VM050_REG_TEMP_WINDOW_SCALE_HI, VM050_REG_TEMP_WINDOW_SCALE_LO },
	{ VM050_REG_TEMP_MIN, VM050_REG_TEMP_CENTER },
	{ VM050_REG_STATUS1, VM050_REG_STATUS3 },
};

static struct regmap_access_table const	vm050_regmap_wr_table = {
	.yes_ranges	= vm050_regmap_wr_ranges,
	.n_yes_ranges	= ARRAY_SIZE(vm050_regmap_wr_ranges),
};

static struct regmap_access_table const	vm050_regmap_rd_table = {
	.yes_ranges	= vm050_regmap_rd_ranges,
	.n_yes_ranges	= ARRAY_SIZE(vm050_regmap_rd_ranges),
};

static struct regmap_access_table const	vm050_regmap_volatile_table = {
	.yes_ranges	= vm050_regmap_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(vm050_regmap_volatile_ranges),
};

static struct regmap_config const	vm050_regmap_config = {
	.name		= "vm050",
	.reg_bits	= 8,
	.reg_stride	= 1,
	.val_bits	= 16,
	.max_register	= 0x8b,
	.wr_table	= &vm050_regmap_wr_table,
	.rd_table	= &vm050_regmap_rd_table,
	.volatile_table	= &vm050_regmap_volatile_table,
	/* seems to be broken in chip */
	.can_multi_write = false,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};
/* }}} regmap setup */

/* {{{ gpio chip functionality */
static int vm050_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	struct vm050		*vm050 = gc_to_vm050(chip);
	int			rc;

	mutex_lock(&vm050->lock);
	rc = vm050_power_on(vm050);
	mutex_unlock(&vm050->lock);

	return rc;
}

static void vm050_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	struct vm050		*vm050 = gc_to_vm050(chip);

	mutex_lock(&vm050->lock);
	vm050_power_off(vm050);
	mutex_unlock(&vm050->lock);
}

static int vm050_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	struct vm050		*vm050 = gc_to_vm050(chip);
	int			rc = 0;

	mutex_lock(&vm050->lock);
	vm050_mod_cached(vm050, VM050_REG_CONTROL13,
			 VM050_FLD_CONTROL13_GPIO_DIR(offset) |
			 VM050_FLD_CONTROL13_GPIO_LVL(offset),

			 VM050_FLD_CONTROL13_GPIO_DIR(offset) |
			 (value ? VM050_FLD_CONTROL13_GPIO_LVL(offset) : 0),
			 &rc);
	mutex_unlock(&vm050->lock);

	return rc;
}

static int vm050_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct vm050		*vm050 = gc_to_vm050(chip);
	int			rc = 0;

	mutex_lock(&vm050->lock);
	vm050_mod_cached(vm050, VM050_REG_CONTROL13,
			 VM050_FLD_CONTROL13_GPIO_DIR(offset), 0, &rc);
	mutex_unlock(&vm050->lock);

	return rc;
}

static void vm050_gpio_set(struct gpio_chip *chip,
			   unsigned int offset, int value)
{
	struct vm050		*vm050 = gc_to_vm050(chip);
	int			rc = 0;

	mutex_lock(&vm050->lock);
	vm050_mod_cached(vm050, VM050_REG_CONTROL13,
			 VM050_FLD_CONTROL13_GPIO_LVL(offset),
			 (value ? VM050_FLD_CONTROL13_GPIO_LVL(offset) : 0),
			 &rc);
	mutex_unlock(&vm050->lock);

	/* TODO: handle 'rc'? */
}

static int vm050_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct vm050		*vm050 = gc_to_vm050(chip);
	int			rc = 0;
	unsigned int		v;

	mutex_lock(&vm050->lock);
	v = vm050_read(vm050, VM050_REG_CONTROL13, &rc);
	mutex_unlock(&vm050->lock);

	if (rc < 0)
		return rc;

	return v & VM050_FLD_CONTROL13_GPIO_LVL(offset);
}
/* }}} gpio chip functionality */

static int vm050_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	struct device		*dev = &client->dev;
	struct device_node const *np = dev->of_node;
	struct vm050		*vm050;
	struct v4l2_subdev	*sd;
	int			rc;

	vm050 = devm_kzalloc(dev, sizeof *vm050, GFP_KERNEL);
	if (!vm050)
		return -ENOMEM;

	vm050->i2c = client;

	vm050->regmap = devm_regmap_init_i2c(client, &vm050_regmap_config);
	if (IS_ERR(vm050->regmap))
		return PTR_ERR(vm050->regmap);

	vm050->clk = devm_clk_get(&client->dev, NULL);
	rc = PTR_ERR_OR_ZERO(vm050->clk);
	if (rc == -ENOENT) {
		vm050->clk = NULL;
	} else if (rc == -EPROBE_DEFER) {
		return rc;
	} else if (rc < 0) {
		dev_warn(&client->dev, "failed to get clk: %d\n", rc);
		return rc;
	}

	vm050->vdd = devm_regulator_get_optional(&client->dev, "vdd");
	rc = PTR_ERR_OR_ZERO(vm050->vdd);
	if (rc == -ENODEV) {
		vm050->vdd = NULL;
	} else if (rc == -EPROBE_DEFER) {
		return rc;
	} else if (rc < 0) {
		dev_warn(&client->dev, "failed to get vdd: %d\n", rc);
		return rc;
	}

	mutex_init(&vm050->lock);

	/* TODO: verify default settings! */
	vm050->reg_ctrl.r1[0] = (0x0103 & ~(VM050_FLD_CONTROL1_ENABLE |
					    VM050_FLD_CONTROL1_PERFORMANCE));
	vm050->reg_ctrl.r1[1] = 0x1100;
	vm050->reg_ctrl.r1[2] = 0xf100;
	vm050->reg_ctrl.r1[3] = 0x1100;
	vm050->reg_ctrl.o[0]  = 0x0011;

	if (of_property_read_bool(np, "pixclk-falling"))
		vm050->reg_ctrl.o[0] &= ~VM050_FLD_OUTPUT_PCLK_POL;
	else
		vm050->reg_ctrl.o[0] |=  VM050_FLD_OUTPUT_PCLK_POL;

	if (of_property_read_bool(np, "conservative-timing"))
		vm050->reg_ctrl.r1[0] &= ~VM050_FLD_CONTROL1_PERFORMANCE;
	else
		vm050->reg_ctrl.r1[0] |= VM050_FLD_CONTROL1_PERFORMANCE;

	vm050->pixclk_inv = of_property_read_bool(np, "pixclk-inv");

	v4l2_i2c_subdev_init(&vm050->subdev, client, &vm050_subdev_ops);

	vm050->pad.flags |= MEDIA_PAD_FL_SOURCE;

	sd = &vm050->subdev;
	sd->owner  = THIS_MODULE;
	sd->dev    = dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &vm050_internal_ops;
	snprintf(sd->name, sizeof sd->name, "vm050@%02x", client->addr);

	/* create the gpio chip */
	vm050->gc = (struct gpio_chip) {
		.label			= dev_name(&client->dev),
		.owner			= THIS_MODULE,
		.ngpio			= 2,
		.base			= -1,
		.parent			= &client->dev,
		.request		= vm050_gpio_request,
		.free			= vm050_gpio_free,
		.direction_output	= vm050_gpio_direction_output,
		.direction_input	= vm050_gpio_direction_input,
		.set			= vm050_gpio_set,
		.get			= vm050_gpio_get,
		.can_sleep		= true,
	};

	rc = gpiochip_add(&vm050->gc);
	if (rc < 0) {
		dev_err(dev, "failed to register gpio chip: %d\n", rc);
		goto err_v4l2_gpiochip_add;
	}

	/* setup the v4l2 ctrl handler */
	v4l2_ctrl_handler_init(&vm050->ctrls, ARRAY_SIZE(vm050_ctrls));
	sd->ctrl_handler = &vm050->ctrls;

	/* initialize the mediabus entity */
	rc = media_entity_pads_init(&sd->entity, 1, &vm050->pad);
	if (rc < 0) {
		dev_err(dev, "media_entity_init() failed: %d\n", rc);
		goto err_media_entity_init;
	}

	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	rc = v4l2_async_register_subdev(sd);
	if (rc < 0) {
		dev_err(dev, "v4l2_async_register_subdev() failed: %d\n", rc);
		goto err_v4l2_async_register_subdev;
	}

	dev_info(dev, "phytec-vm050 sensor probed\n");

	return 0;

	v4l2_async_unregister_subdev(&vm050->subdev);
err_v4l2_async_register_subdev:

	media_entity_cleanup(&vm050->subdev.entity);
err_media_entity_init:

	v4l2_ctrl_handler_free(&vm050->ctrls);
err_v4l2_gpiochip_add:

	gpiochip_remove(&vm050->gc);

	return rc;
}

static int vm050_remove(struct i2c_client *client)
{
	struct v4l2_subdev	*subdev = i2c_get_clientdata(client);
	struct vm050		*vm050	= sd_to_vm050(subdev);

	v4l2_async_unregister_subdev(&vm050->subdev);
	media_entity_cleanup(&vm050->subdev.entity);
	v4l2_ctrl_handler_free(&vm050->ctrls);
	gpiochip_remove(&vm050->gc);

	return 0;
}

static const struct i2c_device_id vm050_id[] = {
	{ "vm050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vm050_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id vm050_of_match[] = {
	{ .compatible = "phytec,vm050", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vm050_of_match);
#endif

static struct i2c_driver vm050_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(vm050_of_match),
		.name = "phytec-vm050",
	},
	.probe          = vm050_probe,
	.remove         = vm050_remove,
	.id_table       = vm050_id,
};
module_i2c_driver(vm050_i2c_driver);

MODULE_DESCRIPTION("Phytec VM05x Camera driver");
MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
MODULE_LICENSE("GPL");
