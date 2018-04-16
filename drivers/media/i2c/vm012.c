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
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/ftrace_event.h>
#include <asm/unaligned.h>

#include <linux/ftrace_event.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "vm012-regs.h"

#define INVALID_FOURCC		(~0u)

#define VM012_MIN_WIDTH		(16u)
#define VM012_MAX_WIDTH		(1280u)
#define VM012_MIN_HEIGHT	(16u)
#define VM012_MAX_HEIGHT	(1024u)

/* TODO: remove me in the final driver! */
#define DO_DEBUG		0

enum vm012_type {
	VM012_TYPE_NOIV2SE,		/* color */
	VM012_TYPE_NOIV2SN,		/* mono */

	VM012_TYPE_MAX_,
};

enum {
	VM012_FLAG_STREAMING,
	VM012_FLAG_REG_ENABLED,
	VM012_FLAG_CLK_ENABLED,

	VM012_FLAG_8BPP_MODE,
};

enum {
	V4L2_CID_X_ENABLE_SUBSAMPLING	= V4L2_CID_USER_CLASS + 1,
	V4L2_CID_X_ENABLE_BINNING,
};

struct vm012_of_data {
	char const			*id;
	enum vm012_type			type;
	unsigned int			chip_config:2; /* chip_config[0..1] */
	bool				has_binning:1;

	unsigned int const		*mbus_codes;
	size_t				num_mbus_codes;
};

struct vm012_regmap_bus {
	struct i2c_client		*i2c;
	unsigned int			active_page;
	uint16_t			reg_control; /* shadow copy */
	struct mutex			lock;

	union {
		unsigned char		raw[16];

		struct {
			uint8_t		reg;
			__be16		val;
		}			rd;

		struct {
			uint8_t		reg;
			__be16		val;
		} __packed		wr;
	}				xfer_buf;
};

struct vm012 {
	struct v4l2_subdev		subdev;
	struct vm012_regmap_bus		rbus;
	struct vm012_of_data const	*info;

	struct v4l2_ctrl_handler	ctrls;
	struct media_pad		pad;
	enum vm012_type			type;

	struct clk			*clk;
	unsigned long			sysclk;
	struct regulator		*vdd;

	unsigned int			fmt_code;
	struct v4l2_rect		rect;

	unsigned long			flags;
	struct mutex			lock;

	atomic_t			power_cnt;

	bool				pixclk_inv:1;
	bool				enable_subsampling:1;
	bool				enable_binning:1;
	bool				is_cmos:1;
};
#define sd_to_vm012(_sd)	container_of((_sd), struct vm012, subdev)

static unsigned int const	VM012_MBUS_CODES_MONO[] = {
	MEDIA_BUS_FMT_Y8_1X8,
	MEDIA_BUS_FMT_Y10_1X10,
};

static unsigned int const	VM012_MBUS_CODES_COLOR[] = {
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SRGGB10_1X10,
};

struct vm012_of_data const	VM012_OF_INFO[] = {
	[VM012_TYPE_NOIV2SE]	= {	/* color */
		.id		= "NOIV2SE (color)",
		.type		= VM012_TYPE_NOIV2SE,
		.chip_config	= 3,
		.mbus_codes	= VM012_MBUS_CODES_COLOR,
		.num_mbus_codes	= ARRAY_SIZE(VM012_MBUS_CODES_COLOR),
		.has_binning	= false,
	},

	[VM012_TYPE_NOIV2SN]	= {
		.id		= "NOIV2SN (monochrome)",
		.type		= VM012_TYPE_NOIV2SN,
		.chip_config	= 2,
		.mbus_codes	= VM012_MBUS_CODES_MONO,
		.num_mbus_codes	= ARRAY_SIZE(VM012_MBUS_CODES_MONO),
		.has_binning	= true,
	},
};

struct regval {
	uint16_t		val;
	uint16_t		reg;
	/* bits which are to be changed; msk == 0 means msk == 0xffff unless
	 * 'msk_valid' is set */
	uint16_t		msk;
	/* when set, 'msk' is always valid (even when 'msk == 0') */
	bool			msk_valid;
};

static unsigned int  _vm012_reg_to_fourcc(struct vm012 *vm012, uint16_t regval)
{
	bool	is_10bpp;


	if (vm012->is_cmos)
		/* CMOS mode allows only 10bit mode; handle it internally */
		is_10bpp = !test_bit(VM012_FLAG_8BPP_MODE, &vm012->flags);
	else
		is_10bpp = regval & VITA1300_FLD_CLOCK_GENERATOR_ADC_MODE_10;

	switch (vm012->info->type) {
	case VM012_TYPE_NOIV2SE:
		return (is_10bpp
			? MEDIA_BUS_FMT_SRGGB10_1X10
			: MEDIA_BUS_FMT_SRGGB8_1X8);

	case VM012_TYPE_NOIV2SN:
		return (is_10bpp
			? MEDIA_BUS_FMT_Y10_1X10
			: MEDIA_BUS_FMT_Y8_1X8);

	default:
		WARN(1, "bad type %d\n", vm012->info->type);
		return INVALID_FOURCC;
	}
}

static unsigned int _fmt_get_bpp(unsigned int code)
{
	switch (code) {
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return 8;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;

	default:
		WARN(1, "unsupported code %04x", code);
		return 0;
	}
}

/* {{{ IO helper functions */
static inline bool _vm012_reg_is_direct(unsigned int reg)
{
	return (reg == VM012_REG_FIRMWARE ||
		reg == VM012_REG_CONTROL ||
		reg == VM012_REG_STATUS);
}

static int _i2c_read(struct vm012_regmap_bus *bus, uint8_t reg, uint16_t *res)
{
	struct i2c_msg		msg[] = {
		{
			.addr	= bus->i2c->addr,
			.flags	= 0,
			.len	= sizeof bus->xfer_buf.rd.reg,
			.buf	= &bus->xfer_buf.rd.reg
		}, {
			.addr	= bus->i2c->addr,
			.flags	= I2C_M_RD,
			.len	= sizeof bus->xfer_buf.rd.val,
			.buf	= (void *)&bus->xfer_buf.rd.val,
		}
	};
	int		rc;

	WARN_ON(!mutex_is_locked(&bus->lock));

	bus->xfer_buf.rd.reg = reg;

	rc = i2c_transfer(bus->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (rc == ARRAY_SIZE(msg)) {
		uint16_t	tmp = be16_to_cpu(bus->xfer_buf.rd.val);

		if (res)
			*res = tmp;

		rc   = 0;
	} else if (rc >= 0) {
		/* partial read */
		rc   = -EIO;
	}

	return rc;
}

static int _i2c_write(struct vm012_regmap_bus *bus, uint8_t reg, uint16_t val)
{
	int		rc;

	WARN_ON(!mutex_is_locked(&bus->lock));

	bus->xfer_buf.wr.reg = reg;
	bus->xfer_buf.wr.val = cpu_to_be16(val);

	rc = i2c_master_send(bus->i2c, (void const *)&bus->xfer_buf.wr,
			     sizeof bus->xfer_buf.wr);
	if (rc == sizeof bus->xfer_buf.wr)
		rc = 0;
	else if (rc >= 0)
		/* partial write */
		rc = -EIO;

	if (rc < 0)
		return rc;

	if (reg == VM012_REG_CONTROL) {
		/* shadow the control register when written */
		/* remove self-clearing bits and the page-bit*/
		val &= ~VM012_FLD_CONTROL_AUTO_SENSOR_RESET;
		val &= ~VM012_FLD_CONTROL_VM012_RESET;
		val &= ~VM012_FLD_CONTROL_PAGE;

		bus->reg_control = val;
	}

	return 0;
}

static int _i2c_wait(struct vm012_regmap_bus *bus)
{
	uint16_t	tmp;
	unsigned int	retries;
	int		rc;

	WARN_ON(!mutex_is_locked(&bus->lock));

	for (retries = 5; retries > 0; --retries) {
		rc = _i2c_read(bus, VM012_REG_STATUS, &tmp);
		if (rc < 0)
			break;

		tmp = be16_to_cpu(tmp);
		/* this assumes, the i2c controller supports bit
		 * stretching. Else SPI_READ must be observed too. */
		if ((tmp & VM012_FLD_STATUS_SPI_WRITE) == 0)
			break;

		mutex_unlock(&bus->lock);

		usleep_range(50, 150);

		mutex_lock(&bus->lock);

		rc = -ETIMEDOUT;
	}

	return rc;
}

static int _i2c_set_page(struct vm012_regmap_bus *bus, unsigned int page)
{
	int		rc;
	uint16_t	reg_val = bus->reg_control;

	WARN_ON(!mutex_is_locked(&bus->lock));

	if (bus->active_page == page)
		return 0;

	if (page > 1)
		return -EINVAL;

	reg_val &= ~VM012_FLD_CONTROL_PAGE;
	reg_val |= (page == 1) ? VM012_FLD_CONTROL_PAGE : 0;

	rc = _i2c_write(bus, VM012_REG_CONTROL, reg_val);
	if (rc < 0)
		return rc;

	bus->active_page = page;
	return 0;
}


static int vm012_i2c_read(struct vm012_regmap_bus *bus,
			  uint16_t reg, uint16_t *res)
{
	int		rc;
	unsigned int	page = reg >> 8;

	if (page > 1)
		return -EINVAL;

	mutex_lock(&bus->lock);

	rc = _i2c_set_page(bus, page);
	if (rc < 0)
		goto out;

	rc = _i2c_read(bus, reg, res);
	if (rc < 0)
		goto out;

out:
	mutex_unlock(&bus->lock);

	return rc;
}


static int vm012_i2c_write(struct vm012_regmap_bus *bus,
			   uint16_t reg, uint16_t val)
{
	int		rc;
	unsigned int	page = reg >> 8;

	if (page > 1)
		return -EINVAL;

	mutex_lock(&bus->lock);

	if (!_vm012_reg_is_direct(reg)) {
		rc = _i2c_wait(bus);
		if (rc < 0)
			goto out;
	}

	rc = _i2c_set_page(bus, page);
	if (rc < 0)
		goto out;

	rc = _i2c_write(bus, reg, val);
	if (rc < 0)
		goto out;

out:
	mutex_unlock(&bus->lock);

	return rc;
}

static unsigned int vm012_read(struct vm012 *vm012, uint16_t reg, int *err)
{
	int		rc;
	uint16_t	v;

	if (*err != 0)
		/* noop when previous operations failed */
		return 0;

	rc = vm012_i2c_read(&vm012->rbus, reg, &v);
	if (rc < 0) {
		dev_err(&vm012->rbus.i2c->dev,
			"failed to read register %02x: %d\n", reg, rc);
		*err = rc;
	}

	return v;
}

static int vm012_readv(struct vm012 *vm012, struct regval rv[], size_t cnt)
{
	size_t		i;

	for (i = 0; i < cnt; ++i) {
		int		rc = 0;

		rv[i].val = vm012_read(vm012, rv[i].reg, &rc);
		if (rc < 0)
			return rc;
	}

	return 0;
}

static void vm012_mod(struct vm012 *vm012, uint16_t reg,
		      uint16_t mask, uint16_t set, int *err)
{
	int		rc;
	uint16_t	tmp;

	if (*err != 0)
		/* noop when previous operations failed */
		return;

	if (mask != 0xffffu) {
		rc = vm012_i2c_read(&vm012->rbus, reg, &tmp);
	} else {
		tmp = 0;
		rc  = 0;
	}

	if (rc >= 0) {
		tmp &= ~mask;
		tmp |= set;

		rc = vm012_i2c_write(&vm012->rbus, reg, tmp);
	}


	if (rc < 0) {
		dev_err(&vm012->rbus.i2c->dev,
			"failed to set register %02x with %04x & %04x: %d\n",
			reg, set, mask, rc);
		*err = rc;
	}
}

static void vm012_write(struct vm012 *vm012, uint16_t reg, uint16_t v, int *err)
{
	int		rc;

	if (*err != 0)
		/* noop when previous operations failed */
		return;

	rc = vm012_i2c_write(&vm012->rbus, reg, v);
	if (rc < 0) {
		dev_err(&vm012->rbus.i2c->dev,
			"failed to write register %02x with %04x: %d\n",
			reg, v, rc);
		*err = rc;
	}
}

static int vm012_writev(struct vm012 *vm012,
			struct regval const rv[], size_t cnt)
{
	size_t			i;

	for (i = 0; i < cnt; ++i) {
		uint16_t	msk;
		int		rc = 0;

		msk = rv[i].msk;
		if (msk == 0 && !rv[i].msk_valid)
			msk = 0xffffu;

		if (msk == 0xffffu)
			vm012_write(vm012, rv[i].reg, rv[i].val, &rc);
		else
			vm012_mod(vm012, rv[i].reg, msk, rv[i].val, &rc);

		if (rc < 0)
			return rc;
	}

	return 0;
}

/* }}} IO helper functions */

static int _vm012_reset_hard(struct vm012 *vm012)
{
	int		rc;
	unsigned int	v;

	rc = 0;
	vm012_write(vm012, VM012_REG_CONTROL,
		    VM012_FLD_CONTROL_VM012_RESET, &rc);
	if (rc < 0)
		return rc;

	msleep(50);

	vm012_write(vm012, VM012_REG_CONTROL,
		    VM012_FLD_CONTROL_AUTO_SENSOR_RESET, &rc);
	if (rc < 0)
		return rc;

	msleep(250);

	v = vm012_read(vm012, VM012_REG_CONTROL, &rc);
	if (rc < 0)
		return rc;

	if ((v & VM012_FLD_CONTROL_AUTO_SENSOR_RESET)) {
		dev_warn(&vm012->rbus.i2c->dev,
			 "auto-sensor-reset bit still set '%04x'", v);
		rc = -EIO;
	}

	return rc;
}

static int _vm012_reset_soft(struct vm012 *vm012)
{
	struct regval const		SOFT_RESET_SEQ[] = {
		{
			.reg	= VITA1300_SOFT_RESET_PLL,
			.val	= 0x0099
		}, {
			.reg	= VITA1300_SOFT_RESET_CGEN,
			.val	= 0x0009
		}, {
			.reg	= VITA1300_SOFT_RESET_ANALOG,
			.val	= 0x0999
		}
	};

	return vm012_writev(vm012, SOFT_RESET_SEQ, ARRAY_SIZE(SOFT_RESET_SEQ));
}

static int vm012_reset(struct vm012 *vm012, bool hard_reset)
{
	int		rc;

	WARN_ON(!mutex_is_locked(&vm012->lock));
	WARN_ON(atomic_read(&vm012->power_cnt) == 0);

	if (hard_reset)
		rc = _vm012_reset_hard(vm012);
	else
		rc = _vm012_reset_soft(vm012);

	if (rc < 0) {
		dev_warn(&vm012->rbus.i2c->dev,
			 "failed to %sreset device: %d\n",
			 hard_reset ? "hard-" : "", rc);
		return rc;
	}

	return rc;
}

static void _vm012_power_off(struct vm012 *vm012)
{
	struct regval const		POWEROFF_SEQ[] = {
		{
			.reg	= VM012_REG_CONTROL,
			.val	= VM012_FLD_CONTROL_SENSOR_RESET,
		}
	};
	int				rc;

	rc = vm012_writev(vm012, POWEROFF_SEQ, ARRAY_SIZE(POWEROFF_SEQ));
	if (rc < 0) {
		dev_warn(&vm012->rbus.i2c->dev,
			 "failed to poweroff device: %d\n", rc);
	}

	/* give some time to upload registers by SPI */
	msleep(10);

	if (test_and_clear_bit(VM012_FLAG_CLK_ENABLED, &vm012->flags))
		clk_disable_unprepare(vm012->clk);

	if (test_and_clear_bit(VM012_FLAG_REG_ENABLED, &vm012->flags))
		regulator_disable(vm012->vdd);
}

#define REGSET(_r, _v) { .reg = (_r), .val = (_v) }

static int vm012_power_on(struct vm012 *vm012)
{
	/* second register set: write dynamic information  */
	struct regval const		rv1[] = {
		REGSET(VITA1300_CHIP_CONFIG, vm012->info->chip_config)
	};

	/* third register set: write static information */
	static struct regval const	rv2[] = {
		VITA1300_STATIC_INIT_ENABLE_CLOCK_MANAGEMENT_V2,
		VITA1300_STATIC_INIT_REQUIRED_REGISTERS,
		VITA1300_STATIC_INIT_MODE_REGISTERS_V2,

		REGSET(VITA1300_SEQ_CONF, 0x0001),
		REGSET(VITA1300_SEQ_CONF, 0x0000),
	};
	int				rc;

	WARN_ON(!mutex_is_locked(&vm012->lock));

	if (atomic_inc_return(&vm012->power_cnt) > 1) {
		rc = 0;
		goto out;
	}

	if (vm012->clk) {
		rc = clk_set_rate(vm012->clk, vm012->sysclk);
		if (rc < 0) {
			dev_warn(&vm012->rbus.i2c->dev,
				 "failed to set clk rate: %d\n", rc);
			goto out;
		}
	}

	if (vm012->vdd) {
		rc = regulator_enable(vm012->vdd);
		if (rc < 0) {
			dev_warn(&vm012->rbus.i2c->dev,
				 "failed to enable regulator: %d\n", rc);
			goto out;
		}
		set_bit(VM012_FLAG_REG_ENABLED, &vm012->flags);
	}

	if (vm012->clk) {
		rc = clk_prepare_enable(vm012->clk);
		if (rc)
			goto out;
		set_bit(VM012_FLAG_CLK_ENABLED, &vm012->flags);
	}

	udelay(1);

	rc = vm012_reset(vm012, true);
	if (rc < 0)
		goto out;

	rc = vm012_writev(vm012, rv1, ARRAY_SIZE(rv1));
	if (rc < 0)
		goto out;

	rc = vm012_writev(vm012, rv2, ARRAY_SIZE(rv2));
	if (rc < 0)
		goto out;

out:
	if (rc < 0) {
		_vm012_power_off(vm012);
		atomic_dec(&vm012->power_cnt);
	}

	return rc;
}

static void vm012_power_off(struct vm012 *vm012)
{
	WARN_ON(!mutex_is_locked(&vm012->lock));

	if (atomic_dec_return(&vm012->power_cnt) > 1)
		return;

	_vm012_power_off(vm012);
}

static int vm012_enum_mbus_code(struct v4l2_subdev *subdev,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct vm012			*vm012 = sd_to_vm012(subdev);

	if (code->index >= vm012->info->num_mbus_codes)
		return -EINVAL;

	code->code = vm012->info->mbus_codes[code->index];

	return 0;
}

static int _vm012_read_format(struct vm012 *vm012, bool honor_scale,
			      struct v4l2_rect *rect, unsigned int *code)
{
	unsigned int	x_start;
	unsigned int	x_end;
	unsigned int	y_start;
	unsigned int	y_end;

	unsigned int	depth;
	unsigned int	scale;
	unsigned int	tmp_code;
	int		rc = 0;

	struct regval	rv[] = {
		/* NOTE: keep them in order so that regmap_bulk_read() can be
		 * used! */
		[0] = { .reg = VITA1300_REG_ROIx_CFG0(0) },
		[1] = { .reg = VITA1300_REG_ROIx_CFG1(0) },
		[2] = { .reg = VITA1300_REG_ROIx_CFG2(0) },
		[3] = { .reg = VITA1300_SEQ_CONF },
		[4] = { .reg = VITA1300_CLOCK_GENERATOR },
	};

	rc = vm012_readv(vm012, rv, ARRAY_SIZE(rv));
	if (rc < 0)
		return rc;

	x_start = VITA1300_FLD_ROI_CFG0_X_START_get(rv[0].val) * 8;
	x_end   = VITA1300_FLD_ROI_CFG0_X_END_get(rv[0].val) * 8 + 8;
	y_start = VITA1300_FLD_ROI_CFG1_Y_START_get(rv[1].val);
	y_end   = VITA1300_FLD_ROI_CFG2_Y_END_get(rv[2].val) + 1;
	depth   = rv[4].val;

	if (x_end < x_start) {
		v4l2_warn(&vm012->subdev, "invalid x range [%d,%d]\n",
			  x_start, x_end);
		rc = -EINVAL;
	}

	if (y_end < y_start) {
		v4l2_warn(&vm012->subdev, "invalid y range [%d,%d]\n",
			  y_start, y_end);
		rc = -EINVAL;
	}

	if (!honor_scale)
		scale = 1;
	else if (rv[3].val & VITA1300_FLD_SEQ_CONF_ENA_SUBSAMPLING)
		scale = 2;
	else if (vm012->info->has_binning &&
		 (rv[3].val & VITA1300_FLD_SEQ_CONF_ENA_BINNING))
		scale = 2;
	else
		scale = 1;

	tmp_code = _vm012_reg_to_fourcc(vm012, depth);
	if (tmp_code == INVALID_FOURCC)
		rc = -EINVAL;

	if (rc < 0)
		return rc;

	*rect = (struct v4l2_rect) {
		.left	= x_start,
		.top	= y_start,
		.width	= (x_end - x_start) / scale,
		.height	= (y_end - y_start) / scale,
	};

	*code = tmp_code;

	return 0;
}

static int _vm012_get_fmt(struct vm012 *vm012,
			  struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_rect	rect;
	unsigned int		code;
	int			rc;

	rc = _vm012_read_format(vm012, true, &rect, &code);
	if (rc < 0)
		return rc;

	*fmt = (struct v4l2_mbus_framefmt) {
		.width		= rect.width,
		.height		= rect.height,
		.code		= code,
		.field		= V4L2_FIELD_NONE,
	};

	return 0;
}

static unsigned int _vm012_get_scale_sft(struct vm012 *vm012)
{
	return (vm012->enable_binning || vm012->enable_subsampling) ? 1 : 0;
}

static int vm012_get_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct vm012			*vm012 = sd_to_vm012(sd);
	int				rc;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		unsigned int	scale = _vm012_get_scale_sft(vm012);

		format->format =
			*v4l2_subdev_get_try_format(sd, cfg, format->pad);

		format->format.width  >>= scale;
		format->format.height >>= scale;

		rc  = 0;
	} else {
		rc = _vm012_get_fmt(vm012, &format->format);
	}

	return rc;
}

static int _vm012_refresh_format(struct vm012 *vm012)
{
	return _vm012_read_format(vm012, false, &vm012->rect, &vm012->fmt_code);
}

static int _vm012_set_format(struct vm012 *vm012, unsigned int code,
			     unsigned int left, unsigned int width,
			     unsigned int top, unsigned int height)
{
	struct regval		rv_buf[10];
	struct regval		*rv = rv_buf;
	int			rc;

	WARN_ON(!mutex_is_locked(&vm012->lock));

	if (width < VM012_MIN_WIDTH || height < VM012_MIN_HEIGHT ||
	    left > VM012_MAX_WIDTH - VM012_MIN_WIDTH ||
	    top > VM012_MAX_HEIGHT - VM012_MIN_HEIGHT ||
	    width > VM012_MAX_WIDTH  - left ||
	    height > VM012_MAX_HEIGHT - top) {
		v4l2_warn(&vm012->subdev,
			  "invalid crop area: [%dx%d+%dx%d] not in [%dx%d..%dx%d]\n",
			  left, top, width, height,
			  VM012_MIN_WIDTH, VM012_MIN_HEIGHT,
			  VM012_MIN_HEIGHT, VM012_MAX_HEIGHT);

		return -EINVAL;
	}

	/* NOTE: fill 'rv' in order to allow regmap_bulk_write()! */

	/* page 1 */

	*rv++ = (struct regval) {
		.reg = VITA1300_REG_ROIx_CFG0(0),
		.val = (VITA1300_FLD_ROI_CFG0_X_START(left / 8) |
			VITA1300_FLD_ROI_CFG0_X_END((left + width) / 8 - 1)),
	};

	*rv++ = (struct regval) {
		.reg = VITA1300_REG_ROIx_CFG1(0),
		.val = VITA1300_FLD_ROI_CFG1_Y_START(top),
	};

	*rv++ = (struct regval) {
		.reg = VITA1300_REG_ROIx_CFG2(0),
		.val = VITA1300_FLD_ROI_CFG2_Y_END(top + height - 1),
	};

	/* page 0 */

	*rv++ = (struct regval) {
		/* activate only ROI #0 */
		.reg = VITA1300_REG_ROI_ACTIVE,
		.val = BIT(0),
	};

	*rv++ = (struct regval) {
		.reg = VITA1300_CLOCK_GENERATOR,
		.msk = VITA1300_FLD_CLOCK_GENERATOR_ADC_MODE_10,
		.val = ((_fmt_get_bpp(code) == 10 || vm012->is_cmos)
			? VITA1300_FLD_CLOCK_GENERATOR_ADC_MODE_10 : 0u)
	};

	*rv++ = (struct regval) {
		.reg = VITA1300_SEQ_CONF,
		.msk = (VITA1300_FLD_SEQ_CONF_ENA_SUBSAMPLING |
			VITA1300_FLD_SEQ_CONF_ENA_BINNING),
		.val = ((vm012->enable_binning
			 ? VITA1300_FLD_SEQ_CONF_ENA_BINNING : 0u) |
			(vm012->enable_subsampling
			 ? VITA1300_FLD_SEQ_CONF_ENA_SUBSAMPLING : 0u))
	};

	rc = vm012_writev(vm012, rv_buf, rv - rv_buf);
	if (rc < 0)
		return rc;

	/* cache this internally; CMOS variant does nto support 8bpp */
	if (_fmt_get_bpp(code) == 8)
		set_bit(VM012_FLAG_8BPP_MODE, &vm012->flags);
	else
		clear_bit(VM012_FLAG_8BPP_MODE, &vm012->flags);

	vm012->fmt_code = code;
	vm012->rect = (struct v4l2_rect) {
		.left	= left,
		.top	= top,
		.width	= width,
		.height	= height,
	};

	return 0;
}

static int vm012_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_pad_config *cfg,
			 struct v4l2_subdev_format *format)
{
	struct vm012			*vm012 = sd_to_vm012(sd);
	int				rc;
	unsigned int			width;
	unsigned int			height;
	unsigned int			code;
	size_t				i;
	int				scale_sft;

	code = vm012->info->mbus_codes[0];
	for (i = 0; i < vm012->info->num_mbus_codes; ++i) {
		if (vm012->info->mbus_codes[i] == format->format.code) {
			code = vm012->info->mbus_codes[i];
			break;
		}
	}

	mutex_lock(&vm012->lock);

	scale_sft = _vm012_get_scale_sft(vm012);

	width  = clamp(format->format.width,
		       VM012_MIN_WIDTH >> scale_sft,
		       VM012_MAX_WIDTH >> scale_sft) << scale_sft;

	height = clamp(format->format.height,
		       VM012_MIN_HEIGHT >> scale_sft,
		       VM012_MAX_HEIGHT >> scale_sft) << scale_sft;

	width  = round_up(width, 16);
	height = round_up(height, 2);

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
	} else if (format->format.width != (width >> scale_sft) ||
		   format->format.height != (height >> scale_sft) ||
		   format->format.code != code ||
		   format->format.field != V4L2_FIELD_NONE) {
		v4l2_warn(sd, "format mismatch (%dx%d@%04x/%d) vs. (%dx%d@%04x>>%d/%d)\n",
			  format->format.width, format->format.height,
			  format->format.code, format->format.field,
			  width, height, code, scale_sft, V4L2_FIELD_NONE);

		rc = -EINVAL;
	} else if (test_bit(VM012_FLAG_STREAMING, &vm012->flags)) {
		v4l2_warn(sd, "can not change format while streaming\n");
		rc = -EBUSY;
	} else {
		rc = _vm012_set_format(vm012, code,
				       vm012->rect.left, width,
				       vm012->rect.top, height);
	}

	mutex_unlock(&vm012->lock);

	return rc;
}

static int vm012_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct vm012			*vm012 = sd_to_vm012(sd);
	struct v4l2_rect		*rect;
	int				rc;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = (struct v4l2_rect) {
			.left	= 0,
			.width	= VM012_MAX_WIDTH,
			.top	= 0,
			.height	= VM012_MAX_HEIGHT
		};
		rc = 0;
		break;

	case V4L2_SEL_TGT_CROP:
		mutex_lock(&vm012->lock);

		switch (sel->which) {
		case V4L2_SUBDEV_FORMAT_TRY:
			rect = v4l2_subdev_get_try_crop(&vm012->subdev, cfg,
							sel->pad);
			break;

		case V4L2_SUBDEV_FORMAT_ACTIVE:
			rc = _vm012_refresh_format(vm012);
			if (rc < 0)
				rect = ERR_PTR(rc);
			else
				rect = &vm012->rect;

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

		mutex_unlock(&vm012->lock);

		break;

	default:
		rc = -EINVAL;
	}

	return rc;
}

static int vm012_set_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_selection *sel)
{
	struct vm012			*vm012 = sd_to_vm012(sd);
	unsigned int			left;
	unsigned int			top;
	unsigned int			width;
	unsigned int			height;
	struct v4l2_rect		*rect;
	int				rc;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	left   = clamp_t(unsigned int, sel->r.left,
			 0u, VM012_MAX_WIDTH - VM012_MIN_WIDTH);
	top    = clamp_t(unsigned int, sel->r.top,
			 0u, VM012_MAX_HEIGHT - VM012_MIN_HEIGHT);

	left = round_up(left, 16);
	top  = round_up(top, 2);

	width  = clamp_t(unsigned int, sel->r.width,
			 VM012_MIN_WIDTH, VM012_MAX_WIDTH - left);
	height = clamp_t(unsigned int, sel->r.height,
			 VM012_MIN_HEIGHT, VM012_MAX_HEIGHT - top);

	width  = round_up(width, 16);
	height = round_up(height, 2);

	mutex_lock(&vm012->lock);

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		sel->r = (struct v4l2_rect) {
			.left	= left,
			.top	= top,
			.width	= width,
			.height	= height
		};

		rect = v4l2_subdev_get_try_crop(&vm012->subdev, cfg, sel->pad);
		rc = 0;
	} else if (sel->r.left != left || sel->r.width != width ||
		   sel->r.top != top || sel->r.height != height) {
		v4l2_warn(sd, "crop mismatch (%dx%d+%dx%d) vs. (%dx%d+%dx%d)\n",
			  sel->r.left, sel->r.top, sel->r.width, sel->r.height,
			  left, top, width, height);
		rc = -EINVAL;
	} else if (test_bit(VM012_FLAG_STREAMING, &vm012->flags) &&
		   (vm012->rect.width != width ||
		    vm012->rect.height != height)) {
		v4l2_warn(sd, "can not change crop dimension while streaming\n");
		rc = -EBUSY;
	} else {
		rc = _vm012_set_format(vm012, vm012->fmt_code,
				       left, width, top, height);
	}

	mutex_unlock(&vm012->lock);

	return rc;
}


static struct v4l2_subdev_pad_ops const		vm012_pad_ops = {
	.enum_mbus_code		= vm012_enum_mbus_code,
	.get_fmt		= vm012_get_fmt,
	.set_fmt		= vm012_set_fmt,
	.get_selection		= vm012_get_selection,
	.set_selection		= vm012_set_selection,
};

/* {{{ core ops */
#ifdef CONFIG_VIDEO_ADV_DEBUG
static int vm012_g_register(struct v4l2_subdev *sd,
			    struct v4l2_dbg_register *reg)
{
	struct vm012	*vm012 = sd_to_vm012(sd);
	int		rc = 0;

	reg->size = 2;
	reg->val = vm012_read(vm012, reg->reg, &rc);

	return rc;
}

static int vm012_s_register(struct v4l2_subdev *sd,
			    struct v4l2_dbg_register const *reg)
{
	struct vm012	*vm012 = sd_to_vm012(sd);
	int		rc = 0;

	if (reg->size != 2 && reg->size != 0)
		return -EINVAL;

	vm012_write(vm012, reg->reg, reg->val, &rc);

	return rc;
}
#endif

static int vm012_querycap(struct vm012 *vm012, struct v4l2_capability *cap)
{
	strcpy(cap->driver, "phytec-vm012");

	return 0;
}

static long vm012_core_ioctl(struct v4l2_subdev *sd,
			     unsigned int cmd, void *arg)
{
	struct vm012	*vm012 = sd_to_vm012(sd);

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		return vm012_querycap(vm012, arg);

	default:
		return -ENOTTY;
	}
}

static struct v4l2_subdev_core_ops const	vm012_core_ops = {
	.ioctl			= vm012_core_ioctl,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= vm012_g_register,
	.s_register		= vm012_s_register,
#endif
};
/* }}} core ops */

/* {{{ video ops */
static int vm012_video_g_mbus_config(struct v4l2_subdev *sd,
				     struct v4l2_mbus_config *cfg)
{
	struct vm012	*vm012 = sd_to_vm012(sd);

	cfg->flags  = (V4L2_MBUS_SLAVE |
		       V4L2_MBUS_MASTER |
		       V4L2_MBUS_HSYNC_ACTIVE_HIGH |
		       V4L2_MBUS_VSYNC_ACTIVE_HIGH |
		       V4L2_MBUS_DATA_ACTIVE_HIGH);

	if (vm012->pixclk_inv)
		cfg->flags |= V4L2_MBUS_PCLK_SAMPLE_RISING;
	else
		cfg->flags |= V4L2_MBUS_PCLK_SAMPLE_FALLING;

	cfg->type   = V4L2_MBUS_PARALLEL;

	return 0;
}

static int _vm012_video_s_stream_on(struct vm012 *vm012)
{
	struct regval const		rv[] = {
		{
			.reg	= VITA1300_SEQ_CONF,
			.msk	= VITA1300_FLD_SEQ_CONF_ENA_SEQ,
			.val	= VITA1300_FLD_SEQ_CONF_ENA_SEQ,
		},
	};
	int			rc;

	if (test_and_set_bit(VM012_FLAG_STREAMING, &vm012->flags))
		return -EBUSY;

	rc = vm012_writev(vm012, rv, ARRAY_SIZE(rv));
	if (rc < 0)
		goto out;

out:
	if (rc < 0)
		clear_bit(VM012_FLAG_STREAMING, &vm012->flags);

	return rc;
}

static int _vm012_video_s_stream_off(struct vm012 *vm012)
{
	struct regval const		rv[] = {
		{
			.reg	= VITA1300_SEQ_CONF,
			.msk	= VITA1300_FLD_SEQ_CONF_ENA_SEQ,
			.val	= 0,
		},
	};
	int			rc;

	if (!test_and_clear_bit(VM012_FLAG_STREAMING, &vm012->flags))
		return -EINVAL;

	rc = vm012_writev(vm012, rv, ARRAY_SIZE(rv));
	if (rc < 0)
		goto out;


out:
	return rc;
}

static int vm012_video_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vm012	*vm012 = sd_to_vm012(sd);
	int		rc;

	mutex_lock(&vm012->lock);

	if (enable)
		rc = _vm012_video_s_stream_on(vm012);
	else
		rc = _vm012_video_s_stream_off(vm012);

	mutex_unlock(&vm012->lock);

	return rc;
}

static struct v4l2_subdev_video_ops const	vm012_subdev_video_ops = {
	.g_mbus_config		= vm012_video_g_mbus_config,
	.s_stream		= vm012_video_s_stream,
};
/* }}} video ops */

static struct v4l2_subdev_ops const		vm012_subdev_ops = {
	.core			= &vm012_core_ops,
	.pad			= &vm012_pad_ops,
	.video			= &vm012_subdev_video_ops,
};

/* {{{ internal ops */
static int vm012_registered(struct v4l2_subdev *sd)
{
	struct vm012		*vm012 = sd_to_vm012(sd);

	/* first register set: read information */
	struct regval			rv0[] = {
		[0] = { .reg = VITA1300_CHIP_ID },
		[1] = {	.reg = VM012_REG_FIRMWARE },
	};

	int		rc;

	mutex_lock(&vm012->lock);

	rc = vm012_power_on(vm012);
	if (rc < 0)
		goto out_nopower;

	rc = vm012_readv(vm012, rv0, ARRAY_SIZE(rv0));
	if (rc < 0)
		goto out;

	if (rv0[0].val != VITA1300_DTA_CHIP_ID) {
		v4l2_err(sd, "unexpected chip id %04x\n", rv0[0].val);
		rc = -ENOENT;
		goto out;
	}

	_vm012_refresh_format(vm012);

	v4l2_info(sd, "%s with firmware %d.%03d, device id %04x\n",
		  vm012->info->id,
		  VM012_FLD_FIRMWARE_MAJOR_get(rv0[1].val),
		  VM012_FLD_FIRMWARE_MINOR_get(rv0[1].val),
		  rv0[0].val);

	rc = 0;

out:
	if (rc < 0)
		vm012_power_off(vm012);

out_nopower:
	mutex_unlock(&vm012->lock);

	return rc;
}

static void vm012_unregistered(struct v4l2_subdev *sd)
{
	struct vm012	*vm012 = sd_to_vm012(sd);

	mutex_lock(&vm012->lock);
	vm012_power_off(vm012);
	mutex_unlock(&vm012->lock);
}

static struct v4l2_subdev_internal_ops const	vm012_internal_ops = {
	.registered		= vm012_registered,
	.unregistered		= vm012_unregistered,
};
/* }}} internal ops */

static int vm012_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vm012		*vm012 = ctrl->priv;
	int			rc = 0;

	mutex_lock(&vm012->lock);

	switch (ctrl->id) {
	case V4L2_CID_X_ENABLE_SUBSAMPLING:
		/* TODO: actual logic requires to call set_fmt() or set_sel()
		 * to take changes of subsampling or binning an effect */
		if (test_bit(VM012_FLAG_STREAMING, &vm012->flags))
			rc = -EBUSY;
		else
			vm012->enable_subsampling = !!ctrl->val;
		break;

	case V4L2_CID_X_ENABLE_BINNING:
		/* TODO: actual logic requires to call set_fmt() or set_sel()
		 * to take changes of subsampling or binning an effect */
		if (test_bit(VM012_FLAG_STREAMING, &vm012->flags))
			rc = -EBUSY;
		else
			vm012->enable_binning = !!ctrl->val;
		break;

	default:
		v4l2_warn(&vm012->subdev, "unsupported control %d\n", ctrl->id);
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&vm012->lock);

	return rc;
}

static struct v4l2_ctrl_ops const		vm012_ctrl_ops = {
	.s_ctrl			= vm012_s_ctrl,
};

static struct v4l2_ctrl_config const		vm012_ctrls[] = {
	{
		.ops		= &vm012_ctrl_ops,
		.id		= V4L2_CID_X_ENABLE_SUBSAMPLING,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Subsampling",
		.min		= 0,
		.max		= 1,
		.step		= 1,
	}, {
		.ops		= &vm012_ctrl_ops,
		.id		= V4L2_CID_X_ENABLE_BINNING,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Binning",
		.min		= 0,
		.max		= 1,
		.step		= 1,
	}
};

static int vm012_probe(struct i2c_client *client,
		       const struct i2c_device_id *did)
{
	struct device		*dev = &client->dev;
	struct device_node const *np = dev->of_node;
	struct vm012		*vm012;
	struct v4l2_subdev	*sd;
	int			rc;
	size_t			i;
	u32			sysclk;

	vm012 = devm_kzalloc(dev, sizeof *vm012, GFP_KERNEL);
	if (!vm012)
		return -ENOMEM;

	vm012->rbus.i2c = client;
	vm012->rbus.active_page = ~0u;
	mutex_init(&vm012->rbus.lock);

	vm012->info = (void const *)did->driver_data;

	vm012->clk = devm_clk_get(&client->dev, NULL);
	rc = PTR_ERR_OR_ZERO(vm012->clk);
	if (rc == -ENOENT) {
		vm012->clk = NULL;
	} else if (rc == -EPROBE_DEFER) {
		return rc;
	} else if (rc < 0) {
		dev_warn(&client->dev, "failed to get clk: %d\n", rc);
		return rc;
	}

	vm012->vdd = devm_regulator_get_optional(&client->dev, "vdd");
	rc = PTR_ERR_OR_ZERO(vm012->vdd);
	if (rc == -ENODEV) {
		vm012->vdd = NULL;
	} else if (rc == -EPROBE_DEFER) {
		return rc;
	} else if (rc < 0) {
		dev_warn(&client->dev, "failed to get vdd: %d\n", rc);
		return rc;
	}

	mutex_init(&vm012->lock);

	if (of_property_read_u32(np, "sysclk", &sysclk) < 0)
		sysclk = 54000000;

	vm012->sysclk  = sysclk;
	vm012->pixclk_inv = of_property_read_bool(np, "pixclk-inv");
	vm012->is_cmos = true;		/* TODO: read from dtree? */

	v4l2_i2c_subdev_init(&vm012->subdev, client, &vm012_subdev_ops);

	vm012->pad.flags |= MEDIA_PAD_FL_SOURCE;

	sd = &vm012->subdev;
	sd->owner  = THIS_MODULE;
	sd->dev    = dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &vm012_internal_ops;
	snprintf(sd->name, sizeof sd->name, "vm012@%02x", client->addr);

	/* setup the v4l2 ctrl handler */
	v4l2_ctrl_handler_init(&vm012->ctrls, ARRAY_SIZE(vm012_ctrls));

	rc = 0;
	for (i = 0; i < ARRAY_SIZE(vm012_ctrls); ++i) {
		if (vm012_ctrls[i].id == V4L2_CID_X_ENABLE_BINNING &&
		    !vm012->info->has_binning)
			continue;

		v4l2_ctrl_new_custom(&vm012->ctrls, &vm012_ctrls[i], vm012);
		rc = vm012->ctrls.error;
		if (rc < 0) {
			dev_warn(dev, "failed to register control#%zu '%s': %d\n",
				 i, vm012_ctrls[i].name, rc);
			goto err;
		}
	}

	rc = v4l2_ctrl_handler_setup(&vm012->ctrls);
	if (rc < 0) {
		v4l2_warn(&vm012->subdev, "failed to set controls: %d\n", rc);
		goto err;
	}
	sd->ctrl_handler = &vm012->ctrls;

	/* initialize the mediabus entity */
	rc = media_entity_init(&sd->entity, 1, &vm012->pad, 0);
	if (rc < 0) {
		dev_err(dev, "media_entity_init() failed: %d\n", rc);
		goto err_media_entity_init;
	}

	rc = v4l2_async_register_subdev(sd);
	if (rc < 0) {
		dev_err(dev, "v4l2_async_register_subdev() failed: %d\n", rc);
		goto err_v4l2_async_register_subdev;
	}

	dev_info(dev, "phytec-vm012 sensor probed\n");

	return 0;

	v4l2_async_unregister_subdev(&vm012->subdev);
err_v4l2_async_register_subdev:

	media_entity_cleanup(&vm012->subdev.entity);
err_media_entity_init:

	v4l2_ctrl_handler_free(&vm012->ctrls);

err:
	return rc;
}

static int vm012_remove(struct i2c_client *client)
{
	struct v4l2_subdev	*subdev = i2c_get_clientdata(client);
	struct vm012		*vm012	= sd_to_vm012(subdev);

	v4l2_async_unregister_subdev(&vm012->subdev);
	media_entity_cleanup(&vm012->subdev.entity);
	v4l2_ctrl_handler_free(&vm012->ctrls);

	return 0;
}

static const struct i2c_device_id vm012_id[] = {
	{
		.name	= "vm012c",
		.driver_data	= (uintptr_t)&VM012_OF_INFO[VM012_TYPE_NOIV2SE]
	}, {
		.name	= "vm012m",
		.driver_data	= (uintptr_t)&VM012_OF_INFO[VM012_TYPE_NOIV2SN]
	},
	{ }
};
MODULE_DEVICE_TABLE(i2c, vm012_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id vm012_of_match[] = {
	{
		.compatible = "phytec,vm012c",
		.data = &VM012_OF_INFO[VM012_TYPE_NOIV2SE]
	}, {
		.compatible = "phytec,vm012m",
		.data = &VM012_OF_INFO[VM012_TYPE_NOIV2SN],
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vm012_of_match);
#endif

static struct i2c_driver vm012_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(vm012_of_match),
		.name = "phytec-vm012",
	},
	.probe          = vm012_probe,
	.remove         = vm012_remove,
	.id_table       = vm012_id,
};
module_i2c_driver(vm012_i2c_driver);

MODULE_DESCRIPTION("Phytec VM05x Camera driver");
MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
MODULE_LICENSE("GPL");
