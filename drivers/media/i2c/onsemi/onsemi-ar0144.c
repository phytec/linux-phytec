/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * ONSemi AR0144 sensor driver.
 *
 * Copyright (C) 2019 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 */


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/clk.h>

#include <media/v4l2-ctrls.h>

#include "onsemi-core.h"
#include "onsemi-regmap.h"

enum ar0144_shdw_ctrl {
	AR0144_SHDW_CTRL_DIGITAL_GAIN_RED,
	AR0144_SHDW_CTRL_DIGITAL_GAIN_BLUE,
	AR0144_SHDW_CTRL_DIGITAL_GAIN_GREENB,
	AR0144_SHDW_CTRL_DIGITAL_GAIN_GREENR,
};

static unsigned int const		AR0144_SHDW_CTRL_MAP[] = {
	[AR0144_SHDW_CTRL_DIGITAL_GAIN_RED]	= V4L2_CID_X_DIGITAL_GAIN_RED,
	[AR0144_SHDW_CTRL_DIGITAL_GAIN_BLUE]	= V4L2_CID_X_DIGITAL_GAIN_BLUE,
	[AR0144_SHDW_CTRL_DIGITAL_GAIN_GREENR]	= V4L2_CID_X_DIGITAL_GAIN_GREENR,
	[AR0144_SHDW_CTRL_DIGITAL_GAIN_GREENB]	= V4L2_CID_X_DIGITAL_GAIN_GREENB,
};

struct ar0144_v4l_parm {
	struct onsemi_v4l_parm		core;

	/* can be modified while streaming */
	unsigned int			extra_delay;
	bool				hflip;
	bool				vflip;
};

struct ar0144_sensor {
	struct onsemi_core		core;

	struct ar0144_v4l_parm		v4l_parm;
	struct onsemi_pll_cfg		pll_cfg;
	struct onsemi_pll_freq		pll_freq;

	struct onsemi_timing_map	mipi_tm[5];

	struct v4l2_ctrl		*shadow_ctrls[ARRAY_SIZE(AR0144_SHDW_CTRL_MAP)];
};

#define core_to_ar0144(_core) \
	container_of((_core), struct ar0144_sensor, core)

static int ar0144_read_mipi_timings(struct ar0144_sensor *sensor)
{
	size_t			i;
	int			rc;

	for (i = 0; i < ARRAY_SIZE(sensor->mipi_tm); ++i) {
		char	attr[] = "mipi-XX";

		sprintf(attr, "mipi-%zu", i);
		rc = onsemi_timing_pair_read_of(&sensor->core, attr,
						&sensor->mipi_tm[i]);
		if (rc < 0)
			break;
	}

	return rc;
}

static int ar0144_find_mipi_timings(struct ar0144_sensor const *sensor,
				    unsigned long freq,
				    uint16_t tm[])
{
	size_t			i;
	int			rc = 0;

	dev_dbg(sensor->core.dev, "MIPI-TIMINGS for %lu\n", freq);

	for (i = 0; i < ARRAY_SIZE(sensor->mipi_tm); ++i) {
		unsigned int	val;

		rc = onsemi_timing_pair_find(&sensor->mipi_tm[i], freq, &val);
		if (rc < 0) {
			dev_warn(sensor->core.dev,
				 "failed to find mipi-timing-%zu for freq %lu\n",
				 i, freq);
			break;
		}

		dev_dbg(sensor->core.dev, "  MIPI-TM#%zu: %04x\n", i, val);

		tm[i] = val;
	}

	return rc;
}

static int ar0144_probe(struct i2c_client *i2c, struct i2c_device_id const *did)
{
	struct ar0144_sensor		*sensor;
	struct onsemi_dev_cfg const	*cfg = (void const *)did->driver_data;
	int				rc;
	bool				is_powered = false;

	sensor = devm_kzalloc(&i2c->dev, sizeof *sensor, GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->core.v4l_parm = &sensor->v4l_parm.core;
	sensor->core.pll_cfg  = &sensor->pll_cfg;
	sensor->core.pll_freq = &sensor->pll_freq;

	rc = onsemi_core_init(i2c, &sensor->core, cfg);
	if (rc < 0)
		goto out;

	if (onsemi_has_mipi(&sensor->core)) {
		rc = ar0144_read_mipi_timings(sensor);
		if (rc < 0)
			goto out;
	}

	rc = onsemi_power_get(&sensor->core);
	if (rc < 0)
		goto out;

	is_powered = true;

	rc = onsemi_core_hw_init(&sensor->core, cfg);
	if (rc < 0)
		goto out;

	rc = onsemi_core_v4l_init(&sensor->core, &sensor->v4l_parm.core, cfg);
	if (rc < 0)
		goto out;

	pr_debug("onsemi=%p\n", &sensor->core);

	rc = 0;

out:
	if (rc < 0) {
		if (is_powered)
			onsemi_power_put(&sensor->core);
	}

	return rc;
}

static int ar0144_remove(struct i2c_client *i2c)
{
	struct v4l2_subdev	*sd = i2c_get_clientdata(i2c);
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	struct ar0144_sensor	*sensor = core_to_ar0144(onsemi);

	onsemi_core_release(onsemi);
	onsemi_power_put(&sensor->core);

	return 0;
}

static int ar0144_fill_limits(struct onsemi_core *onsemi,
			      struct onsemi_limits *limits,
			      int (*dflt_fn)(struct onsemi_core *onsemi,
					     struct onsemi_limits *limits))
{
	int		rc;

	rc = dflt_fn(onsemi, limits);
	if (rc < 0)
		goto out;

	limits->hblank_min = 208;

	limits->pll_op_sys_clk_div = (struct onsemi_range) {
		.min	= 1,
		.max	= 2,
	};

	limits->pll_op_pix_clk_div = (struct onsemi_range) {
		.min	= 4,
		.max	= 16,
	};

	limits->pre_pll_div = (struct onsemi_range) {
		.min	= 1,
		.max	= 64,
	};

	limits->pre_pll_mul = (struct onsemi_range) {
		.min	= 32,
		.max	= 384,
	};

	limits->ext_clk = (struct onsemi_range) {
		.min	=  6000000,
		.max	= 48000000,
	};

	limits->op_clk = (struct onsemi_range) {
		.min	= 0,
		.max	= 37125000,
	};

	limits->pll_vco = (struct onsemi_range) {
		.min	= 384000000,
		.max	= 768000000,
	};

	limits->pix_clk = (struct onsemi_range) {
		.min	= 0,
		.max	= 74250000,
	};

	limits->f_serial = (struct onsemi_range) {
		.min	= 300000000,
		.max	= 768000000,
	};

	rc = 0;

out:
	return rc;
}

static int ar0144_stream_off(struct onsemi_core *onsemi)
{
	int		rc = 0;

	switch (onsemi->active_bus->bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		onsemi_write(&rc, onsemi, 0x31c6, 0x80);

		/* TODO: prevent going into standby mode  */
		rc = 1;
		break;
	default:
		break;
	}

	return rc;
}

static int ar0144_stream_on(struct onsemi_core *onsemi)
{
	int		rc = 0;
	unsigned int	bpp = onsemi->v4l_parm->bpp;

	switch (onsemi->color_mode) {
	case ONSEMI_COLOR_MONOCHROME:
		onsemi_update_bits(&rc, onsemi, 0x30b0, BIT(7), BIT(7));
		break;
	case ONSEMI_COLOR_BAYER:
		onsemi_update_bits(&rc, onsemi, 0x30b0, BIT(7), 0);
		break;
	default:
		WARN_ON(1);
		rc = -EINVAL;
		break;
	}

	switch (onsemi->active_bus->bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		onsemi_write(&rc, onsemi, 0x31c6, 0x00);
		onsemi_write(&rc, onsemi, 0x3354,
			     bpp ==  8 ? 0x2a :
			     bpp == 10 ? 0x2b :
			     bpp == 12 ? 0x2c :
			     0);
		break;

	default:
		break;
	}

	if (rc < 0) {
		dev_warn(onsemi->dev, "failed to enable AR0144: %d\n", rc);
		goto out;
	}

	rc = 0;

out:
	return rc;
}

static char const * const		ar0144_test_pattern_menu[] = {
	"disabled",
	"solid color",
	"color bar",
	"fade to gray",
	"walking 1 (12 bit)"
};

static char const * const		ar0144_embdata_menu[] = {
	"disabled",
	"stats",
	"data",
	"both",
};

static char const * const		ar0144_binning_menu[] = {
	"none",
	"avg",
	"sum",
};

static char const * const		ar0144_ana_gain_min_menu[] = {
	"1x",
	"2x",
	"4x",
	"8x",
};

static s64 const			ar0144_skip_menu[] = {
	1, 2, 4, 8, 16
};

static bool ar0144_ctrl_get(struct onsemi_core *onsemi,
			    struct v4l2_ctrl *ctrl, int *rc)
{
	switch (ctrl->id) {
	case V4L2_CID_X_AUTO_EXPOSURE_CUR:
		ctrl->val = onsemi_read(rc, onsemi, 0x3164);
		break;
	default:
		return false;
	}

	return true;
}

static bool ar0144_ctrl_setup_pre(struct onsemi_core *onsemi,
				  struct v4l2_ctrl_config *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_X_DIGITAL_GAIN_RED:
	case V4L2_CID_X_DIGITAL_GAIN_GREENR:
	case V4L2_CID_X_DIGITAL_GAIN_BLUE:
	case V4L2_CID_X_DIGITAL_GAIN_GREENB:
		/* filter out color related controls for monochrome sensors */
		if (onsemi->color_mode == ONSEMI_COLOR_MONOCHROME)
			return false;

		break;
	}

	return true;
}

static void ar0144_ctrl_setup_post(struct onsemi_core *onsemi,
				   struct v4l2_ctrl *ctrl)
{
	struct ar0144_sensor	*sensor = core_to_ar0144(onsemi);
	size_t			i;

	for (i = 0; i < ARRAY_SIZE(AR0144_SHDW_CTRL_MAP); ++i) {
		if (AR0144_SHDW_CTRL_MAP[i] == ctrl->id) {
			sensor->shadow_ctrls[i] = ctrl;
			break;
		}
	}
}

static void _ctrl_try_set(struct v4l2_ctrl *ctrl, int v)
{
	pr_debug("%s: %s <- %d\n", __func__, ctrl ? ctrl->name : NULL, v);

	if (ctrl) {
		ctrl->val = v;
		ctrl->cur.val = v;
	}
}

static void _ar0144_set_analogue_gain(int *rc, struct onsemi_core *onsemi,
				      unsigned int v, s32 *res)
{
	unsigned int		coarse;
	unsigned int		fine;

	if (*rc < 0)
		return;

	for (coarse = 0; coarse < 4; ++coarse)
		if (v < (1u << coarse) * 1000000 * 32 / 16500)
			break;

	if (v <= (1u << coarse) * 1000) {
		fine = 0;
	} else {
		fine = 32 - 32 * 1000 * (1u << coarse) / v;
	}

	if (WARN(fine > 15, "invalid 'fine' value %u", fine))
		fine = 15;

	onsemi_update_bits(rc, onsemi, 0x3060, (7u << 4) | (15u <<0),
			   (coarse << 4) | (fine << 0));

	if (*rc < 0)
		return;

	if (res)
		*res = 1000 * (1u << coarse) * 32 / (32 - fine);
}

static void _ar0144_set_digital_gain(int *rc, struct onsemi_core *onsemi,
				     unsigned int cid, unsigned int v)
{
	unsigned int		reg;
	unsigned int		coarse = v / 1000;
	unsigned int		fine = (v % 1000) * 128 / 1000;
	bool			update_all = false;

	if (*rc < 0)
		return;

	switch (cid) {
	case V4L2_CID_DIGITAL_GAIN:
		reg = 0x305e;
		update_all = true;
		break;

	case V4L2_CID_X_DIGITAL_GAIN_RED:
		reg = 0x305a;
		break;

	case V4L2_CID_X_DIGITAL_GAIN_GREENR:
		reg = 0x3056;
		break;

	case V4L2_CID_X_DIGITAL_GAIN_BLUE:
		reg = 0x3058;
		break;

	case V4L2_CID_X_DIGITAL_GAIN_GREENB:
		reg = 0x305c;
		break;

	default:
		WARN(1, "CID %04x unknown", cid);
		*rc = -EINVAL;
		return;
	}

	onsemi_write(rc, onsemi, reg, (coarse << 7) | fine);

	if (*rc < 0)
		return;

	if (update_all) {
		struct ar0144_sensor	*sensor = core_to_ar0144(onsemi);
		struct v4l2_ctrl	**ctrls = sensor->shadow_ctrls;

		_ctrl_try_set(ctrls[AR0144_SHDW_CTRL_DIGITAL_GAIN_RED], v);
		_ctrl_try_set(ctrls[AR0144_SHDW_CTRL_DIGITAL_GAIN_BLUE], v);
		_ctrl_try_set(ctrls[AR0144_SHDW_CTRL_DIGITAL_GAIN_GREENB], v);
		_ctrl_try_set(ctrls[AR0144_SHDW_CTRL_DIGITAL_GAIN_GREENR], v);
	}
}

static bool ar0144_ctrl_set(struct onsemi_core *onsemi,
			    struct v4l2_ctrl *ctrl, int *rc)
{
	unsigned int		tmp;

	if (WARN_ON(rc < 0))
		return false;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		onsemi_update_bits(rc, onsemi, 0x3100, BIT(0),
				   ctrl->val == V4L2_EXPOSURE_AUTO ? BIT(0) : 0);
		break;
	case V4L2_CID_X_AUTO_EXPOSURE_TGT:
		onsemi_write(rc, onsemi, 0x3102, ctrl->val);
		break;
	case V4L2_CID_X_AUTO_EXPOSURE_MIN:
		onsemi_write(rc, onsemi, 0x311e, ctrl->val);
		break;
	case V4L2_CID_X_AUTO_EXPOSURE_MAX:
		onsemi_write(rc, onsemi, 0x311c, ctrl->val);
		break;
	case V4L2_CID_X_EMBEDDED_DATA:
		if (onsemi_is_streaming(onsemi)) {
			*rc = -EBUSY;
			goto out;
		}
		onsemi_update_bits(rc, onsemi, 0x3064, BIT(8) | BIT(7),
				   ctrl->val << 7);
		break;
	case V4L2_CID_X_AUTOGAIN_ANALOGUE:
		onsemi_update_bits(rc, onsemi, 0x3100, BIT(1),
				   ctrl->val ? BIT(1) : 0);
		break;
	case V4L2_CID_X_AUTOGAIN_DIGITAL:
		onsemi_update_bits(rc, onsemi, 0x3100, BIT(4),
				   ctrl->val ? BIT(4) : 0);
		break;
	case V4L2_CID_X_AUTOGAIN_ANALOGUE_MIN:
		onsemi_update_bits(rc, onsemi, 0x3100, 3u << 5, ctrl->val << 5);
		break;
	case V4L2_CID_TEST_PATTERN:
		onsemi_write(rc, onsemi, 0x3070,
			     ctrl->val < 4 ? ctrl->val :
			     ctrl->val - 4 + 256);
		onsemi_update_bits(rc, onsemi, 0x3044, 3u << 4, 0);
		break;
	case V4L2_CID_X_BINNING_COL:
		onsemi_update_bits(rc, onsemi, 0x3040, BIT(13) | BIT(5),
				   ctrl->val == 0 ? 0 :
				   ctrl->val == 1 ? BIT(13) :
				   BIT(13) | BIT(5));
		break;
	case V4L2_CID_X_BINNING_ROW:
		onsemi_update_bits(rc, onsemi, 0x3040, BIT(12),
				   ctrl->val == 0 ? 0 : BIT(12));
		break;
	case V4L2_CID_X_SKIP_COL:
		if (onsemi_is_streaming(onsemi)) {
			*rc = -EBUSY;
			goto out;
		}

		tmp = 1u << ctrl->val;
		onsemi_write(rc, onsemi, 0x30a2, (tmp << 1) - 1u);

		if (*rc == 0) {
			onsemi->v4l_parm->x_skip = tmp;
			onsemi->v4l_parm->x_scale = tmp;
		}

		break;

	case V4L2_CID_X_SKIP_ROW:
		if (onsemi_is_streaming(onsemi)) {
			*rc = -EBUSY;
			goto out;
		}

		tmp = 1u << ctrl->val;
		onsemi_write(rc, onsemi, 0x30a6, (tmp << 1) - 1u);

		if (*rc == 0) {
			onsemi->v4l_parm->y_skip = tmp;
			onsemi->v4l_parm->y_scale = tmp;
		}
		break;

	case V4L2_CID_DIGITAL_GAIN:
	case V4L2_CID_X_DIGITAL_GAIN_RED:
	case V4L2_CID_X_DIGITAL_GAIN_GREENR:
	case V4L2_CID_X_DIGITAL_GAIN_BLUE:
	case V4L2_CID_X_DIGITAL_GAIN_GREENB:
		_ar0144_set_digital_gain(rc, onsemi, ctrl->id, ctrl->val);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		_ar0144_set_analogue_gain(rc, onsemi, ctrl->val, &ctrl->val);
		break;

	case V4L2_CID_X_COMPAND:
		onsemi_update_bits(rc, onsemi, 0x31d0, BIT(0),
				   ctrl->val ? BIT(0) : 0);
		break;

	case V4L2_CID_X_BLACK_LEVEL_AUTO:
		onsemi_update_bits(rc, onsemi, 0x3180, BIT(15),
				   ctrl->val ? BIT(15) : 0);
		break;

	case V4L2_CID_FLASH_LED_MODE:
		onsemi_update_bits(rc, onsemi, 0x3270, BIT(8),
				   ctrl->val == V4L2_FLASH_LED_MODE_FLASH ? BIT(8)
				   : 0);
		break;

	case V4L2_CID_X_FLASH_DELAY:
		onsemi_update_bits(rc, onsemi, 0x3270, 0xffu << 0,
				   ctrl->val & 0xffu);
		break;

	case V4L2_CID_LINK_FREQ:
		break;

	default:
		return false;
	}

out:
	return true;
}

static struct v4l2_ctrl_config const	ar0144_ctrls[] = {
	{
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_AUTO_EXPOSURE_TGT,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "auto exposure tgt",
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 0x5000,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_AUTO_EXPOSURE_MIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "auto exposure min",
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_AUTO_EXPOSURE_MAX,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "auto exposure max",
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 800,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_AUTO_EXPOSURE_CUR,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "auto exposure cur",
		.flags		= (V4L2_CTRL_FLAG_READ_ONLY |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 65535,
		.step		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_MENU,
		.min		= 0,
		.max		= V4L2_EXPOSURE_MANUAL,
		.menu_skip_mask	= ~(BIT(V4L2_EXPOSURE_AUTO) |
				    BIT(V4L2_EXPOSURE_MANUAL)),
		.def		= V4L2_EXPOSURE_AUTO,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_AUTOGAIN_ANALOGUE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "autogain analogue",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_AUTOGAIN_DIGITAL,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "autogain digital",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_test_pattern_menu) - 1,
		.qmenu		= ar0144_test_pattern_menu,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_EMBEDDED_DATA,
		.type		= V4L2_CTRL_TYPE_MENU,
		.flags		= V4L2_CTRL_FLAG_MODIFY_LAYOUT,
		.name		= "embedded data",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_embdata_menu) - 1,
		.menu_skip_mask	= BIT(0) | BIT(2),
		.def		= 1,
		.qmenu		= ar0144_embdata_menu,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_BINNING_COL,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "col binning",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_binning_menu) - 1,
		.qmenu		= ar0144_binning_menu,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_BINNING_ROW,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "row binning",
		.min		= 0,
		/* filter out 'sum' from the menu by omitting last entry */
		.max		= ARRAY_SIZE(ar0144_binning_menu) - 2,
		.qmenu		= ar0144_binning_menu,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_SKIP_COL,
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.flags		= V4L2_CTRL_FLAG_MODIFY_LAYOUT,
		.name		= "horizontal skipping",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_skip_menu) - 1,
		.qmenu_int	= ar0144_skip_menu,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_SKIP_ROW,
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.flags		= V4L2_CTRL_FLAG_MODIFY_LAYOUT,
		.name		= "vertical skipping",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_skip_menu) - 1,
		.qmenu_int	= ar0144_skip_menu,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_ANALOGUE_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		=  1000,
		.step		=     1,
		.max		= 16000,
		.def		=  2000,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_DIGITAL_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.flags		= (V4L2_CTRL_FLAG_EXECUTE_ON_WRITE |
				   V4L2_CTRL_FLAG_UPDATE),
		.min		=  1000,
		.step		=     1,
		.max		= 15999,
		.def		=  1000,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_RED,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "digital gain red",
		.min		=  1000,
		.step		=     1,
		.max		= 15999,
		.def		=  1000,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_GREENR,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "digital gain green (red)",
		.min		=  1000,
		.step		=     1,
		.max		= 15999,
		.def		=  1000,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_BLUE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "digital gain blue",
		.min		=  1000,
		.step		=     1,
		.max		= 15999,
		.def		=  1000,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_GREENB,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "digital gain green (blue)",
		.min		=  1000,
		.step		=     1,
		.max		= 15999,
		.def		=  1000,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_AUTOGAIN_ANALOGUE_MIN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "analogue gain auto min",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0144_ana_gain_min_menu) - 1,
		.qmenu		= ar0144_ana_gain_min_menu,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_COMPAND,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "companding",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_BLACK_LEVEL_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "black level correction",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_FLASH_LED_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.min		= 0,
		.max		= V4L2_FLASH_LED_MODE_FLASH,
		.menu_skip_mask	= BIT(V4L2_FLASH_LED_MODE_TORCH),
		.def		= V4L2_FLASH_LED_MODE_NONE,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_FLASH_DELAY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "flash delay",
		.min		=  -128,
		.step		=     1,
		.max		=   127,
		.def		=     0,
	},
};

static int ar0144_prepare(struct onsemi_core *sensor)
{
	int		rc = 0;

	if (sensor->active_bus &&
	    sensor->active_bus->bus_type == V4L2_MBUS_CSI2_DPHY) {
		onsemi_write(&rc, sensor, 0x31d8, (3 << 8) | (1 << 4));
		onsemi_write(&rc, sensor, 0x31c6, 0x80);
		onsemi_update_bits(&rc, sensor, 0x301a, BIT(2), BIT(2));
	}

	if (sensor->active_bus) {
		unsigned int	slew_rate_dat = sensor->active_bus->slew_rate_dat;
		unsigned int	slew_rate_clk = sensor->active_bus->slew_rate_clk;
		uint32_t	val = 0;
		uint32_t	msk = 0;

		if (slew_rate_dat != ONSEMI_NO_SLEW_RATE) {
			val |= (slew_rate_dat & 0x7u) << 13;
			msk |= 0x7u << 13;
		}

		if (slew_rate_clk != ONSEMI_NO_SLEW_RATE) {
			val |= (slew_rate_clk & 0x7u) << 10;
			msk |= 0x7u << 10;
		}

		if (msk != 0u)
			onsemi_update_bits(&rc, sensor, 0x306e, msk, val);
	}

	return rc;
}

static int ar0144_pll_calculate(struct onsemi_core *onsemi)
{
	struct ar0144_sensor		*sensor = core_to_ar0144(onsemi);
	struct onsemi_v4l_parm const	*parm = onsemi->v4l_parm;
	struct onsemi_businfo const	*bus_info = onsemi->active_bus;

	if (WARN_ON(!bus_info))
		return -EINVAL;

	v4l2_info(&onsemi->subdev,
		  "%s: code=%x, bpp=%d, frame=%dx%d [%dx%d+%dx%d]\n",
		  __func__, parm->code, parm->bpp, parm->frame.width, parm->frame.height,
		  parm->crop.left, parm->crop.top, parm->crop.width, parm->crop.height);

	(void)sensor;

	return 0;
}

static int ar0144_pll_set(struct onsemi_core *onsemi,
			  struct onsemi_pll_cfg const *cfg,
			  struct onsemi_pll_freq const *freq)
{
	struct ar0144_sensor		*sensor = core_to_ar0144(onsemi);
	int		rc;
	uint16_t	mipi_tm[ARRAY_SIZE(sensor->mipi_tm)];

	if (WARN_ON(!onsemi->active_bus))
		return -EPIPE;

	rc = 0;
	onsemi_write(&rc, onsemi, 0x302a, cfg->vt_pix_div);
	onsemi_write(&rc, onsemi, 0x302c, cfg->vt_sys_div);
	onsemi_write(&rc, onsemi, 0x302e, cfg->pre_pll_div);
	onsemi_write(&rc, onsemi, 0x3030, cfg->pre_pll_mul);
	onsemi_write(&rc, onsemi, 0x3036, cfg->op_pix_div);
	onsemi_write(&rc, onsemi, 0x3038, cfg->op_sys_div);

	if (rc < 0)
		goto out;

	switch (onsemi->active_bus->bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		rc = ar0144_find_mipi_timings(sensor, freq->vco, mipi_tm);
		if (rc < 0)
			break;

		onsemi_write(&rc, onsemi, 0x31b4, mipi_tm[0]);
		onsemi_write(&rc, onsemi, 0x31b6, mipi_tm[1]);
		onsemi_write(&rc, onsemi, 0x31b8, mipi_tm[2]);
		onsemi_write(&rc, onsemi, 0x31ba, mipi_tm[3]);
		onsemi_write(&rc, onsemi, 0x31bc, mipi_tm[4]);
		break;

	default:
		rc = 0;
	}

out:
	return rc;
}

static struct onsemi_core_ops const	ar0144_core_ops = {
	.has_smia_cfg		= false,
	.vaa_first		= true,
	.supported_bpp		= BIT(8) | BIT(10) | BIT(12),

	.fill_limits		= ar0144_fill_limits,
	.stream_on		= ar0144_stream_on,
	.stream_off		= ar0144_stream_off,
	.prepare		= ar0144_prepare,
	.pll_set		= ar0144_pll_set,
	.pll_calculate		= ar0144_pll_calculate,

	.ctrl_set		= ar0144_ctrl_set,
	.ctrl_get		= ar0144_ctrl_get,
	.ctrl_setup_pre		= ar0144_ctrl_setup_pre,
	.ctrl_setup_post	= ar0144_ctrl_setup_post,
	.ctrls			= ar0144_ctrls,
	.num_ctrls		= ARRAY_SIZE(ar0144_ctrls),

};

static struct regmap_range const		ar0144_regmap_octet_table_yes[] = {
	{ 0x301c, 0x301d },
	{ 0x3021, 0x3024 },
};

static struct regmap_access_table const		ar0144_regmap_octet_table = {
	.yes_ranges	= ar0144_regmap_octet_table_yes,
	.n_yes_ranges	= ARRAY_SIZE(ar0144_regmap_octet_table_yes),
};

static struct regmap_range const		ar0144_regmap_wr_table_no[] = {
	{ 0x301c, 0x301d },
	{ 0x3021, 0x3024 },

	{ 0x3026, 0x3026 },
	{ 0x303a, 0x303c },

	{ ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3024),
	  ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3024) },
};

static struct regmap_access_table const		ar0144_regmap_wr_table = {
	.no_ranges	= ar0144_regmap_wr_table_no,
	.n_no_ranges	= ARRAY_SIZE(ar0144_regmap_wr_table_no),
};

static struct regmap_range const		ar0144_regmap_rd_table_yes[] = {
	{ 0x1000, 0x1300 },

	{ 0x3000, 0x301a },
	{ 0x301e, 0x301e },
	{ 0x3026, 0x3ffe },

	{ ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x301c),
	  ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x301d) },

	{ ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3021),
	  ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3024) },
};

static struct regmap_access_table const		ar0144_regmap_rd_table = {
	.yes_ranges	= ar0144_regmap_rd_table_yes,
	.n_yes_ranges	= ARRAY_SIZE(ar0144_regmap_rd_table_yes),
};

static struct regmap_range const		ar0144_regmap_vol_table_yes[] = {
	{ 0x301a, 0x301a },
	{ 0x3026, 0x3026 },
	{ 0x303a, 0x303c },
	{ 0x312a, 0x312a },
	{ 0x3150, 0x3164 },
	{ ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3021),
	  ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3021) },
	{ ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3024),
	  ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3024) },
};

static struct regmap_access_table const		ar0144_regmap_vol_table = {
	.yes_ranges	= ar0144_regmap_vol_table_yes,
	.n_yes_ranges	= ARRAY_SIZE(ar0144_regmap_vol_table_yes),
};


static struct regmap_config const	ar0144_regmap_config = {
	.reg_bits	= 16,
	.reg_stride	= 2,
	.val_bits	= 16,
	.max_register	= 0xfffe,

	.wr_table	= &ar0144_regmap_wr_table,
	.rd_table	= &ar0144_regmap_rd_table,
	.volatile_table	= &ar0144_regmap_vol_table,

	.cache_type	= REGCACHE_NONE,
};

static struct onsemi_dev_cfg const	ar0144_dev_cfg = {
	.chip_version	= 0x0356,
	.ops		= &ar0144_core_ops,
	.regmap_config	= &ar0144_regmap_config,
	.is_color	= -1,
};

static struct onsemi_dev_cfg const	ar0144c_dev_cfg = {
	.chip_version	= 0x0356,
	.ops		= &ar0144_core_ops,
	.regmap_config	= &ar0144_regmap_config,
	.is_color	= 1,
};

static struct onsemi_dev_cfg const	ar0144m_dev_cfg = {
	.chip_version	= 0x0356,
	.ops		= &ar0144_core_ops,
	.regmap_config	= &ar0144_regmap_config,
	.is_color	= 0,
};

static struct i2c_device_id const	ar0144_id_table[] = {
	{ "ar0144",  (uintptr_t)&ar0144_dev_cfg },
	{ "ar0144c", (uintptr_t)&ar0144c_dev_cfg },
	{ "ar0144m", (uintptr_t)&ar0144m_dev_cfg },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ar0144_id_table);

static struct of_device_id const	ar0144_of_match[] = {
	{ .compatible = "onsemi,ar0144",  .data = &ar0144_dev_cfg },
	{ .compatible = "onsemi,ar0144c", .data = &ar0144c_dev_cfg },
	{ .compatible = "onsemi,ar0144m", .data = &ar0144m_dev_cfg },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0144_of_match);

static struct dev_pm_ops const		ar0144_pm_ops = {
};

static struct i2c_driver ar0144_i2c_driver = {
	.driver		= {
		.name	= "ar0144",
		.pm	= &ar0144_pm_ops,
		.of_match_table = of_match_ptr(ar0144_of_match),
	},
	.probe		= ar0144_probe,
	.remove		= ar0144_remove,
	.id_table	= ar0144_id_table,
};
module_i2c_driver(ar0144_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
