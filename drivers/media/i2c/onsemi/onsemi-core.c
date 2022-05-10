/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Generic part of ONSemi ARxxx sensor drivers.
 *
 * Copyright (C) 2018 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 */

#include "onsemi-core.h"

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#include <linux/of_graph.h>

#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>

#include "onsemi-regmap.h"
#include "onsemi-reg.h"

#define IS_PWR2(_v)	(((_v) & ((_v) - 1u)) == 0)

static s64 const	S64_ZERO;

static bool is_pwr2(unsigned long v)
{
	return (v & (v - 1lu)) == 0;
}

static int onsemi_get_regulator(struct device *dev, char const *name,
				struct regulator **reg)
{
	struct regulator		*res;
	int				rc;

	res = devm_regulator_get(dev, name);
	if (IS_ERR(reg)) {
		rc = PTR_ERR(reg);
		dev_probe_err(dev, rc, "failed to get %s regulator: %d\n",
			      name, rc);
		goto out;
	}

	*reg = res;
	rc = 0;

out:
	return rc;
}

static int onsemi_core_of_bus_init_ep(struct onsemi_core *onsemi,
				      struct device_node *np,
				      struct onsemi_businfo *info)
{
	struct v4l2_fwnode_endpoint	endpoint = {
						.bus_type = V4L2_MBUS_UNKNOWN
					};
	int				rc;
	int32_t				tmp;

	if (info->is_used) {
		dev_warn(onsemi->dev, "ep already initialized\n");
		return -EINVAL;
	}

	rc = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(np), &endpoint);
	if (rc < 0) {
		dev_warn(onsemi->dev, "failed to parse ep:%d\n", rc);
		return rc;
	}

	/* 'endpoint.bus.parallel.data_shift' can not be used because it is
	 * unsigned but we require signed */
	if (of_property_read_s32(np, "onsemi,data-shift", &tmp) < 0)
		info->dout_sft = 0;
	else
		info->dout_sft = tmp;

	if (of_property_read_s32(np, "onsemi,slew-rate-dat", &tmp) < 0)
		info->slew_rate_dat = ONSEMI_NO_SLEW_RATE;
	else
		info->slew_rate_dat = tmp;

	if (of_property_read_s32(np, "onsemi,slew-rate-clk", &tmp) < 0)
		info->slew_rate_clk = ONSEMI_NO_SLEW_RATE;
	else
		info->slew_rate_clk = tmp;

	info->bus_type = endpoint.bus_type;

	if (endpoint.nr_of_link_frequencies > 0)
		info->max_freq = endpoint.link_frequencies[0];
	else
		info->max_freq = 0;

	switch (endpoint.bus_type) {
	case V4L2_MBUS_PARALLEL:
		info->bus_width = endpoint.bus.parallel.bus_width;
		rc = 0;
		break;

	case V4L2_MBUS_CSI2_DPHY:
		info->bus_width = endpoint.bus.mipi_csi2.num_data_lanes;
		rc = 0;
		break;

	default:
		dev_warn(onsemi->dev, "unsupported v4l bus type %d\n",
			 endpoint.bus_type);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		goto out;

	if (!onsemi->active_bus)
		onsemi->active_bus = info;

	info->is_used  = true;
	rc = 0;

out:
	return rc;
}

static int onsemi_core_of_bus_init(struct device *dev,
				   struct onsemi_core *onsemi)
{
	int			rc = 0;
	struct device_node	*np = NULL;
	bool			missing_reg_prop = false;
	bool			have_reg_prop = false;

	for (;;) {
		uint32_t		reg;

		np = of_graph_get_next_endpoint(dev->of_node, np);
		if (!np)
			break;

		if (of_property_read_u32(np->parent, "reg", &reg) >= 0) {
			have_reg_prop = true;
		} else {
			missing_reg_prop = true;
			reg = 0;
		}

		if (reg >= ARRAY_SIZE(onsemi->bus_info)) {
			dev_warn(dev, "endpoint reg %u out of range\n", reg);
			rc = -EINVAL;
			break;
		}

		rc = onsemi_core_of_bus_init_ep(onsemi, np,
						&onsemi->bus_info[reg]);
		if (rc < 0) {
			dev_warn(dev, "bad ep#%d", reg);
			break;
		}
	}

	of_node_put(np);

	if (missing_reg_prop && have_reg_prop)
		dev_warn(dev, "endpoint without reg property\n");

	return rc;
}

static int onsemi_core_of_init(struct device *dev, struct onsemi_core *onsemi)
{
	struct gpio_desc	*gpio;
	struct clk		*clk;
	int			rc;

	rc = onsemi_core_of_bus_init(dev, onsemi);
	if (rc < 0)
		goto out;

	rc = onsemi_get_regulator(dev, "vdd", &onsemi->reg_vdd);
	if (rc < 0)
		goto out;

	rc = onsemi_get_regulator(dev, "vddio", &onsemi->reg_vddio);
	if (rc < 0)
		goto out;

	rc = onsemi_get_regulator(dev, "vaa", &onsemi->reg_vaa);
	if (rc < 0)
		goto out;

	rc = onsemi_get_regulator(dev, "vaapix", &onsemi->reg_vaapix);
	if (rc < 0)
		goto out;

	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	rc = PTR_ERR_OR_ZERO(gpio);
	if (rc < 0) {
		dev_probe_err(dev, rc, "failed to get rst gpio: %d\n", rc);
		goto out;
	}
	onsemi->rst_gpio = gpio;

	clk = devm_clk_get(dev, "ext");
	rc = PTR_ERR_OR_ZERO(clk);
	if (rc < 0) {
		if (rc != -EPROBE_DEFER)
			dev_probe_err(dev, rc, "failed to get ext clock: %d\n", rc);
		goto out;
	}
	onsemi->extclk = clk;

	/* Support for 'synch' clock is hacky and required only for special
	 * hardware (e.g. Phytec Nunki Board with LVDS deserializer) */
	clk = devm_clk_get(dev, "synch");
	rc = PTR_ERR_OR_ZERO(clk);
	if (rc == -ENOENT) {
		clk = NULL;
	} else if (rc < 0) {
		if (rc != -EPROBE_DEFER)
			dev_probe_err(dev, rc, "failed to get synch clock: %d\n", rc);
		goto out;
	}
	onsemi->synchclk = clk;

	rc = 0;

out:
	return rc;
}

static bool in_range(char const *id, struct onsemi_range const *r, unsigned long v)
{
	if (!r)
		return true;

	if (v < r->min || v > r->max) {
		pr_debug("%s: '%s' (%lu) out of range [%lu, %lu]\n", __func__,
			 id, v, r->min, r->max);
		return false;
	}

	return true;
}

static int onsemi_core_pll_set(struct onsemi_core *onsemi,
			       struct onsemi_pll_cfg const *cfg,
			       struct onsemi_pll_freq const *freq)
{
	int				rc = 0;

	if (onsemi->synchclk &&
	    !test_and_clear_bit(ONSEMI_FLAG_SYNCCLK_ON, &onsemi->flags)) {
		int	rc;

		rc = clk_set_rate(onsemi->synchclk, onsemi->pll_freq->link_freq);
		if (rc < 0) {
			v4l2_warn(&onsemi->subdev,
				  "failed to set sync clk rate to %llu: %d\n",
				  onsemi->pll_freq->link_freq, rc);
			goto syncclk_out;
		}

		rc = clk_prepare_enable(onsemi->synchclk);
		if (rc < 0) {
			v4l2_warn(&onsemi->subdev,
				  "failed to enable sync clk: %d\n", rc);
			goto syncclk_out;
		}

	syncclk_out:
		if (rc < 0)
			clear_bit(ONSEMI_FLAG_SYNCCLK_ON, &onsemi->flags);

		/* ignore errors regarding synchclk setup */
		rc = 0;
	}

	if (onsemi->ops->pll_set)
		rc = onsemi->ops->pll_set(onsemi, cfg, freq);
	else
		rc = 0;

	if (rc < 0) {
		dev_warn(onsemi->dev, "failed to set PLL: %d\n", rc);
		goto out;
	}

	dev_dbg(onsemi->dev,
		"CLK: ext=%lu, vco=%lu, vt=[%lu, %lu, %lu], op=[%lu, %lu, %lu]\n",
		freq->ext,    freq->vco,
		freq->vt_sys, freq->vt_pix, freq->clk_pixel,
		freq->op_sys, freq->op_pix, freq->clk_op);

	rc = 0;


out:
	return rc;
}

static int _onsemi_power_reset(struct onsemi_core *sensor, bool do_assert)
{
	int			rc = 0;

	dbg_trace_pwr("%sasserting reset\n", do_assert ? "" : "de");

	if (sensor->rst_gpio) {
		gpiod_set_value_cansleep(sensor->rst_gpio, do_assert);
	} else if (!do_assert) {
		onsemi_write(&rc, sensor,
			     ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3021), 1);

		if (rc < 0)
			dev_warn(sensor->dev,
				 "failed to initiate software reset: %d\n", rc);
	} else {
		rc = 0;
	}

	rc = 0;

	if (do_assert) {
		/* TODO: this is not really correct; we do not have an
		 * explicit STOP state */
		usleep_range(10, 10000);
	} else {
		msleep(100);

		regcache_mark_dirty(sensor->regmap);
	}

	return rc;
}

static int _onsemi_power_off(struct onsemi_core *sensor)
{
	WARN_ON(sensor->pwr_state != ONSEMI_PWR_RST);

	clk_disable_unprepare(sensor->extclk);

	if (!sensor->ops->vaa_first) {
		regulator_disable(sensor->reg_vaapix);
		regulator_disable(sensor->reg_vaa);

		msleep(20);
	}

	regulator_disable(sensor->reg_vdd);

	msleep(20);

	regulator_disable(sensor->reg_vddio);

	msleep(20);

	if (sensor->ops->vaa_first) {
		regulator_disable(sensor->reg_vaapix);
		regulator_disable(sensor->reg_vaa);

		msleep(20);
	}

	return 0;
}

static int _onsemi_power_on(struct onsemi_core *sensor)
{
	int		rc;

	WARN_ON(sensor->pwr_state != ONSEMI_PWR_OFF);

	if (sensor->ops->vaa_first) {
		rc = regulator_enable(sensor->reg_vaa);
		if (rc < 0)
			goto err_vaa;

		rc = regulator_enable(sensor->reg_vaapix);
		if (rc < 0)
			goto err_vaapix;

		usleep_range(4*1500, 4*10000);
	}

	if (sensor->rst_gpio)
		gpiod_set_value_cansleep(sensor->rst_gpio, 1);

	rc = regulator_enable(sensor->reg_vddio);
	if (rc < 0)
		goto err_vddio;

	usleep_range(4*1500, 4*10000);

	rc = regulator_enable(sensor->reg_vdd);
	if (rc < 0)
		goto err_vdd;

	usleep_range(4*1500, 4*10000);

	if (!sensor->ops->vaa_first) {
		rc = regulator_enable(sensor->reg_vaa);
		if (rc < 0)
			goto err_vaa;

		rc = regulator_enable(sensor->reg_vaapix);
		if (rc < 0)
			goto err_vaapix;

		usleep_range(4*1500, 4*10000);
	}

	rc = clk_prepare_enable(sensor->extclk);
	if (rc < 0)
		goto err_extclk;

	sensor->ext_clk_freq = clk_get_rate(sensor->extclk);

	usleep_range(4*1500, 4*10000);
	regcache_mark_dirty(sensor->regmap);

	return 0;

err_extclk:

	regulator_disable(sensor->reg_vaapix);
err_vaapix:

	regulator_disable(sensor->reg_vaa);
err_vaa:

	regulator_disable(sensor->reg_vdd);
err_vdd:

	regulator_disable(sensor->reg_vddio);
err_vddio:

	return rc;
}

static int onsemi_power_change_unlocked(struct onsemi_core *sensor,
					enum onsemi_pwr_state state)
{
	enum onsemi_state_fn {
		ONSEMI_FN_NONE,
		ONSEMI_FN_ON,
		ONSEMI_FN_OFF,
		ONSEMI_FN_RST_ASSERT,
		ONSEMI_FN_RST_DEASSERT,
	};

	struct onsemi_state_transition {
		enum onsemi_state_fn	fn;
		enum onsemi_pwr_state	next;
	};

	static struct onsemi_state_transition const	STATES[3][3] = {
		[ONSEMI_PWR_OFF] = {
			[ONSEMI_PWR_OFF]	= {
				.fn	= ONSEMI_FN_NONE,
				.next	= ONSEMI_PWR_OFF,
			},
			[ONSEMI_PWR_ON]		= {
				.fn	= ONSEMI_FN_ON,
				.next	= ONSEMI_PWR_RST,
			},
			[ONSEMI_PWR_RST]	= {
				.fn	= ONSEMI_FN_ON,
				.next	= ONSEMI_PWR_RST,
			},
		},

		[ONSEMI_PWR_RST] = {
			[ONSEMI_PWR_OFF]	= {
				.fn	= ONSEMI_FN_OFF,
				.next	= ONSEMI_PWR_OFF,
			},
			[ONSEMI_PWR_ON]		= {
				.fn	= ONSEMI_FN_RST_DEASSERT,
				.next	= ONSEMI_PWR_ON,
			},
			[ONSEMI_PWR_RST]	= {
				.fn	= ONSEMI_FN_NONE,
				.next	= ONSEMI_PWR_RST,
			},
		},

		[ONSEMI_PWR_ON] = {
			[ONSEMI_PWR_OFF]	= {
				.fn	= ONSEMI_FN_RST_ASSERT,
				.next	= ONSEMI_PWR_RST,
			},
			[ONSEMI_PWR_ON]		= {
				.fn	= ONSEMI_FN_NONE,
				.next	= ONSEMI_PWR_ON,
			},
			[ONSEMI_PWR_RST]	= {
				.fn	= ONSEMI_FN_RST_ASSERT,
				.next	= ONSEMI_PWR_RST,
			},
		},
	};


	int		rc;

	dbg_trace_pwr("power-state %d -> %d\n", sensor->pwr_state, state);

	while (state != sensor->pwr_state) {
		struct onsemi_state_transition const	*trans =
			&STATES[sensor->pwr_state][state];

		switch (trans->fn) {
		case ONSEMI_FN_NONE:
			rc = 0;
			break;

		case ONSEMI_FN_ON:
			rc = _onsemi_power_on(sensor);
			break;

		case ONSEMI_FN_OFF:
			rc = _onsemi_power_off(sensor);
			break;

		case ONSEMI_FN_RST_ASSERT:
			rc = _onsemi_power_reset(sensor, true);
			break;

		case ONSEMI_FN_RST_DEASSERT:
			rc = _onsemi_power_reset(sensor, false);
			break;

		default:
			BUG();
			break;
		}

		if (rc < 0)
			break;

		sensor->pwr_state = trans->next;
	}

	if (rc < 0)
		goto out;

	rc = 0;

out:
	if (rc < 0) {
		dev_err(sensor->dev, "failed to change power (%d): %d\n",
			state, rc);
	}

	return rc;
}

static int onsemi_power_change(struct onsemi_core *sensor,
			       enum onsemi_pwr_state state)
{
	int		rc;

	mutex_lock(&sensor->lock);
	rc = onsemi_power_change_unlocked(sensor, state);
	mutex_unlock(&sensor->lock);

	return rc;
}

int onsemi_power_get(struct onsemi_core *sensor)
{
	int	rc;
	int	num_users;

	num_users = atomic_inc_return(&sensor->num_pwr_users);

	dbg_trace_pwr("num_users=%d\n", num_users);

	if (WARN_ON(num_users <= 0))
		rc = -EINVAL;
	else if (num_users == 1)
		rc = onsemi_power_change(sensor, ONSEMI_PWR_ON);
	else
		rc = 0;

	return rc;
}
EXPORT_SYMBOL_GPL(onsemi_power_get);

void onsemi_power_put(struct onsemi_core *sensor)
{
	int	num_users;

	num_users = atomic_dec_return(&sensor->num_pwr_users);

	WARN_ON(num_users < 0);

	if (num_users == 0)
		onsemi_power_change(sensor, ONSEMI_PWR_OFF);
}
EXPORT_SYMBOL_GPL(onsemi_power_put);

#define attribute_typeof(_struct, _attr)	__typeof__(&((_struct *)0)->_attr)
#define attribute_types_compatible(_type, _struct, _attr) \
	(__builtin_types_compatible_p(attribute_typeof(_struct, _attr), _type *))

#define offsetof_t(_type, _struct, _attr)				\
	(BUILD_BUG_ON_ZERO(!attribute_types_compatible(_type, _struct, _attr)) + \
	 offsetof(_struct, _attr))

#define DECL_LIMIT(_reg, _attr)						\
	{								\
		.reg = _reg,						\
		.name	= # _attr,					\
		.ofs	= offsetof_t(unsigned long,			\
				     struct onsemi_limits, _attr),	\
	}

static struct {
	char const	*name;
	uint16_t	reg;
	size_t		ofs;
} const		ONSEMI_LIMITS[] = {
	DECL_LIMIT(0x1120, pll_vt_sys_clk_div.min),
	DECL_LIMIT(0x1122, pll_vt_sys_clk_div.max),
	DECL_LIMIT(0x1134, pll_vt_pix_clk_div.min),
	DECL_LIMIT(0x1136, pll_vt_pix_clk_div.max),

	DECL_LIMIT(0x1140, vlen.min),
	DECL_LIMIT(0x1142, vlen.max),
	DECL_LIMIT(0x1144, hlen.min),
	DECL_LIMIT(0x1146, hlen.max),
	DECL_LIMIT(0x1148, hblank_min),
	DECL_LIMIT(0x114a, vblank_min),

	DECL_LIMIT(0x1180, x.min),
	DECL_LIMIT(0x1182, y.min),
	DECL_LIMIT(0x1184, x.max),
	DECL_LIMIT(0x1186, y.max),

	DECL_LIMIT(0x1160, pll_op_sys_clk_div.min),
	DECL_LIMIT(0x1162, pll_op_sys_clk_div.max),
	DECL_LIMIT(0x116c, pll_op_pix_clk_div.min),
	DECL_LIMIT(0x116e, pll_op_pix_clk_div.max),

	DECL_LIMIT(0x1114, pre_pll_mul.min),
	DECL_LIMIT(0x1116, pre_pll_mul.max),
	DECL_LIMIT(0x1108, pre_pll_div.min),
	DECL_LIMIT(0x110a, pre_pll_div.max),

	DECL_LIMIT(0xffff, pix_clk.min),
	DECL_LIMIT(0xffff, pix_clk.max),

	DECL_LIMIT(0xffff, pix_clk.min),
	DECL_LIMIT(0xffff, pix_clk.max),
};

#undef DECL_LIMIT

static void onsemi_core_debugfs_release(struct device *dev, void *dentry_)
{
	struct dentry	**dentry = dentry_;

	debugfs_remove_recursive(*dentry);
}

static int onsemi_core_debugfs_init(struct onsemi_core *onsemi)
{
	struct dentry	**d_top_ctx;
	struct dentry	*d_top;
	struct dentry	*d_limits;
	struct dentry	*d_freq;
	struct onsemi_pll_freq *freq = onsemi->pll_freq;
	size_t		i;
	int		rc;

	if (WARN_ON(!freq))
		return -EINVAL;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return 0;

	d_top_ctx = devres_alloc(onsemi_core_debugfs_release, sizeof *d_top_ctx,
				 GFP_KERNEL);
	if (!d_top_ctx)
		return -ENOMEM;

	d_top = debugfs_create_dir(dev_name(onsemi->dev), NULL);
	rc = PTR_ERR_OR_ZERO(d_top);
	if (rc < 0) {
		dev_err(onsemi->dev, "failed to create toplevel debugfs: %d\n",
			rc);
		d_top = NULL;
		goto out;
	}

	d_limits = debugfs_create_dir("limits", d_top);
	rc = PTR_ERR_OR_ZERO(d_limits);
	if (rc < 0) {
		dev_err(onsemi->dev, "failed to create 'limits' debugfs: %d\n",
			rc);
		goto out;
	}

	d_freq = debugfs_create_dir("freq", d_top);
	rc = PTR_ERR_OR_ZERO(d_freq);
	if (rc < 0) {
		dev_err(onsemi->dev, "failed to create 'freq' debugfs: %d\n",
			rc);
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(ONSEMI_LIMITS); ++i) {
		uintptr_t	tmp = (uintptr_t)onsemi->limits;

		tmp += ONSEMI_LIMITS[i].ofs;

		debugfs_create_ulong(ONSEMI_LIMITS[i].name, 0444, d_limits,
				     (unsigned long *)tmp);
	}

	debugfs_create_ulong("ext",       0444, d_freq, &freq->ext);
	debugfs_create_ulong("vco",       0444, d_freq, &freq->vco);
	debugfs_create_ulong("clk-pixel", 0444, d_freq, &freq->clk_pixel);
	debugfs_create_ulong("clk-op",    0444, d_freq, &freq->clk_op);
	debugfs_create_ulong("vt-pix",    0444, d_freq, &freq->vt_pix);
	debugfs_create_ulong("vt-sys",    0444, d_freq, &freq->vt_sys);
	debugfs_create_ulong("op-pix",    0444, d_freq, &freq->op_pix);
	debugfs_create_ulong("op-sys",    0444, d_freq, &freq->op_sys);
	debugfs_create_u64  ("link_freq", 0444, d_freq, &freq->link_freq);

	onsemi->debugfs_top    = d_top;
	onsemi->debugfs_limits = d_limits;
	onsemi->debugfs_freq   = d_freq;

	*d_top_ctx = d_top;
	devres_add(onsemi->dev, d_top_ctx);

	rc = 0;

out:
	if (rc < 0) {
		debugfs_remove_recursive(d_top);
		devres_free(d_top_ctx);
	}

	return rc;
}

#undef DECL_LIMIT

void onsemi_core_release(struct onsemi_core *onsemi)
{
	v4l2_async_unregister_subdev(&onsemi->subdev);
	media_entity_cleanup(&onsemi->subdev.entity);
	v4l2_ctrl_handler_free(&onsemi->ctrls);
}
EXPORT_SYMBOL_GPL(onsemi_core_release);

int onsemi_core_init(struct i2c_client *i2c,
		     struct onsemi_core *onsemi,
		     struct onsemi_dev_cfg const *cfg)
{
	int		rc;

	if (WARN_ON(!cfg || !cfg->ops || !cfg->regmap_config)) {
		dev_err(&i2c->dev, "missing/bad device config\n");
		rc = -EINVAL;
		goto out;
	}

	if (WARN_ON(!onsemi->v4l_parm || !onsemi->pll_cfg || !onsemi->pll_freq)) {
		dev_err(&i2c->dev, "missing/bad device attributes\n");
		rc = -EINVAL;
		goto out;
	}

#if 0
	if (WARN_ON(!cfg->ops->get_v4l_parm)) {
		dev_err(&i2c->dev, "bad device ops\n");
		rc = -EINVAL;
		goto out;
	}
#endif

	*onsemi = (struct onsemi_core) {
		.dev		= &i2c->dev,
		.pwr_state	= ONSEMI_PWR_OFF,
		.num_pwr_users	= ATOMIC_INIT(0),
		.num_v4lpwr_users = ATOMIC_INIT(0),
		.ops		= cfg->ops,
		.limits		= cfg->limits,
		.lock		= __MUTEX_INITIALIZER(onsemi->lock),
		.v4l_parm	= onsemi->v4l_parm,
		.pll_cfg	= onsemi->pll_cfg,
		.pll_freq	= onsemi->pll_freq,
	};

	if (!onsemi->limits) {
		struct onsemi_limits	*limits = cfg->limits_buf;

		if (!limits)
			limits = devm_kzalloc(onsemi->dev, sizeof *limits,
					      GFP_KERNEL);

		if (!limits) {
			dev_err(onsemi->dev,
				"failed to allocate buffer for limits\n");
			rc = -ENOMEM;
			goto out;
		}

		onsemi->limits = limits;
		onsemi->dyn_limits = limits;
	}

	rc = onsemi_core_of_init(onsemi->dev, onsemi);
	if (rc < 0) {
		dev_err(onsemi->dev, "bad/missing dtree setup: %d\n", rc);
		goto out;
	}

	onsemi->regmap = devm_regmap_init_onsemi(i2c, cfg->regmap_config);
	rc = PTR_ERR_OR_ZERO(onsemi->regmap);
	if (rc < 0) {
		dev_err(onsemi->dev, "failed to initialized regmap: %d\n", rc);
		goto out;
	}

	rc = onsemi_core_debugfs_init(onsemi);
	if (rc < 0) {
		dev_err(onsemi->dev, "failed to create debugfs: %d\n", rc);
		goto out;
	};

	rc = 0;

out:
	return rc;
}
EXPORT_SYMBOL_GPL(onsemi_core_init);

#define DECL_LIMIT(_reg, _fld)						\
	{								\
		.reg = _reg,						\
		.ofs = offsetof(struct onsemi_limits, _fld),		\
	}

static int onsemi_core_read_limits(struct onsemi_core *onsemi,
				   struct onsemi_limits *limits)
{
	size_t		i;
	int		rc = 0;

	for (i = 0; i < ARRAY_SIZE(ONSEMI_LIMITS); ++i) {
		unsigned long	tmp;
		uintptr_t	dst = (uintptr_t)(limits);

		if (ONSEMI_LIMITS[i].reg == 0xffff)
			continue;

		tmp = onsemi_read(&rc, onsemi, ONSEMI_LIMITS[i].reg);
		if (rc < 0)
			break;

		dst += ONSEMI_LIMITS[i].ofs;
		memcpy((void *)dst, &tmp, sizeof tmp);
	}

	if (rc < 0) {
		dev_warn(onsemi->dev, "failed to read sensor limits: %d\n", rc);
		goto out;
	}

	rc = 0;

out:
	return rc;
}

#undef DECL_LIMIT

static unsigned long onsemi_clk_div_mul(unsigned long freq,
					unsigned int div,
					unsigned int mul)
{
	uint64_t	res;

	if (WARN_ON(div == 0))
		return 0;

	res  = freq;
	res *= mul;
	res  = div_u64(res, div);

	return res;
}

static ssize_t onsemi_core_show_version_chip(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev	*sd = dev_get_drvdata(dev);
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);

	return sprintf(buf, "0x%04x\n", onsemi->version.chip);
}

static ssize_t onsemi_core_show_version_rev(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev	*sd = dev_get_drvdata(dev);
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);

	return sprintf(buf, "0x%04x\n", onsemi->version.rev);
}

static ssize_t onsemi_core_show_version_cust(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev	*sd = dev_get_drvdata(dev);
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);

	return sprintf(buf, "0x%04x\n", onsemi->version.cust);
}


static struct device_attribute const	ONSEMI_CORE_DEV_ATTRS[] = {
	{
		.attr	= {
			.name	= "version-chip",
			.mode	= 0444,
		},
		.show	= onsemi_core_show_version_chip,
	}, {
		.attr	= {
			.name	= "version-rev",
			.mode	= 0444,
		},
		.show	= onsemi_core_show_version_rev,
	}, {
		.attr	= {
			.name	= "version-cust",
			.mode	= 0444,
		},
		.show	= onsemi_core_show_version_cust,
	},
};

static struct attribute const * const	ONSEMI_CORE_ATTRS[] = {
	&ONSEMI_CORE_DEV_ATTRS[0].attr,
	&ONSEMI_CORE_DEV_ATTRS[1].attr,
	&ONSEMI_CORE_DEV_ATTRS[2].attr,

	NULL,
};

static struct attribute_group const	ONSEMI_CORE_ATTR_GRP = {
	.name		= NULL,
	.attrs		= (struct attribute **)(ONSEMI_CORE_ATTRS),
};

static struct onsemi_core_thermal *
onsemi_core_get_thermal(struct onsemi_core *onsemi)
{
#if IS_ENABLED(CONFIG_THERMAL)
	return &onsemi->thermal;
#else
	return NULL;
#endif
}

static int onsemi_core_get_temp(void *onsemi_, int *res)
{
	struct onsemi_core			*onsemi = onsemi_;
	struct onsemi_core_thermal const	*thermal = onsemi_core_get_thermal(onsemi);
	int			rc;
	uint16_t		ctrl_reg;
	uint16_t		val;

	rc = onsemi_power_get(onsemi);
	if (rc < 0)
		return rc;

	mutex_lock(&onsemi->lock);
	ctrl_reg  = onsemi_read(&rc, onsemi, 0x30b4);

	if ((ctrl_reg & BIT(0)) != 0) {
		val = onsemi_read(&rc, onsemi, 0x30b2);
	} else {
		ctrl_reg &= ~(BIT(5) | BIT(4));

		onsemi_write(&rc, onsemi, 0x30b4, ctrl_reg | BIT(0) | BIT(5));
		onsemi_write(&rc, onsemi, 0x30b4, ctrl_reg | BIT(0) | BIT(4));

		udelay(500);
		val = onsemi_read(&rc, onsemi, 0x30b2);

		onsemi_write(&rc, onsemi, 0x30b4, ctrl_reg);
	}

	mutex_unlock(&onsemi->lock);

	dev_dbg(onsemi->dev, "%s: ctrl=%04x, val=%04x\n", __func__,
		ctrl_reg, val);

	if (rc < 0)
		return rc;

	*res = thermal->slope * val + thermal->t0;

	return 0;
}

static struct thermal_zone_of_device_ops const	ONSEMI_CORE_THERMAL_OPS = {
	.get_temp	= onsemi_core_get_temp,
};

static int onsemi_core_thermal_init(struct onsemi_core *onsemi)
{
	struct device_node		*np = onsemi->dev->of_node;
	struct onsemi_core_thermal	*thermal = onsemi_core_get_thermal(onsemi);
	struct thermal_zone_device	*tdev;
	int				rc;
	uint16_t			cal;
	int32_t				slope;
	int32_t				t_ref;
	signed long			t0;

	if (!IS_ENABLED(CONFIG_THERMAL))
		return 0;

	rc = 0;
	cal = onsemi_read(&rc, onsemi, 0x30c6);
	if (rc < 0)
		return rc;

	if (of_property_read_s32(np, "thermal,slope", &slope) < 0)
		slope =  700;

	if (of_property_read_s32(np, "thermal,tref", &t_ref) < 0)
		t_ref = 55000;

	if (slope == 0) {
		dev_err(onsemi->dev, "bad thermal slope parameter\n");
		return -EINVAL;
	}

	tdev = devm_thermal_zone_of_sensor_register(onsemi->dev, 0, onsemi,
						    &ONSEMI_CORE_THERMAL_OPS);
	rc = PTR_ERR_OR_ZERO(tdev);
	if (rc == -ENODEV) {
		dev_dbg(onsemi->dev, "no thermal zone setup; skipping registration\n");
		return 0;
	}

	if (rc < 0) {
		if (rc != -EPROBE_DEFER)
			dev_err(onsemi->dev, "failed to register thermal device: %d\n", rc);
		return rc;
	}

	t0  = t_ref;
	t0 -= cal * slope;

	*thermal = (struct onsemi_core_thermal) {
		.tdev	= tdev,
		.t0	= t0,
		.slope	= slope,
	};

	dev_dbg(onsemi->dev, "thermal: cal=%u, slope=%d, t_ref=%d -> t_0=%ld\n",
		(int)cal, (int)slope, (int)t_ref, t0);

	return 0;
}

int onsemi_core_hw_init(struct onsemi_core *onsemi,
			struct onsemi_dev_cfg const *cfg)
{
	int				rc;
	struct onsemi_core_version	ver;

	rc = onsemi_power_get(onsemi);
	if (rc < 0)
		return rc;

	if (cfg->ops->has_smia_cfg) {
		ver.chip = onsemi_read(&rc, onsemi, 0x0000);
		ver.rev  = onsemi_read(&rc, onsemi,
				       ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x0002));
		ver.manu = onsemi_read(&rc, onsemi,
				       ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x0003));
		ver.smia = onsemi_read(&rc, onsemi,
				       ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x0004));
	} else {
		ver.chip = onsemi_read(&rc, onsemi, 0x3000);
		ver.rev  = onsemi_read(&rc, onsemi, 0x300e);
		ver.manu = 0;
		ver.smia = 0;
	}

	ver.cust = onsemi_read(&rc, onsemi, 0x31fe);

	if (rc < 0) {
		dev_err(onsemi->dev, "failed to read sensor information: %d\n",
			rc);
		goto out;
	}

	if (ver.chip != cfg->chip_version) {
		dev_err(onsemi->dev, "unexpected sensor %04x\n", ver.chip);
		rc = -ENOENT;
		goto out;
	}

	switch (cfg->is_color) {
	case -1:
		if (ver.cust & BIT(4))
			onsemi->color_mode = ONSEMI_COLOR_BAYER;
		else
			onsemi->color_mode = ONSEMI_COLOR_MONOCHROME;

		break;

	case 0:
		onsemi->color_mode = ONSEMI_COLOR_MONOCHROME;
		break;

	case 1:
		onsemi->color_mode = ONSEMI_COLOR_BAYER;
		break;

	default:
		WARN_ON(1);
		rc = -ENOENT;
		goto out;
	}

	if (!onsemi->dyn_limits)
		/* static limits */
		rc = 0;
	else if (onsemi->ops->fill_limits)
		rc = onsemi->ops->fill_limits(onsemi, onsemi->dyn_limits,
					      onsemi_core_read_limits);
	else
		rc = onsemi_core_read_limits(onsemi, onsemi->dyn_limits);

	if (rc < 0) {
		dev_err(onsemi->dev, "failed to read sensor limits\n");
		goto out;
	}

	if (cfg->ops->has_smia_cfg) {
		dev_info(onsemi->dev, "device %04x, rev %d/%d/%d, cust %02x\n",
			 ver.chip, ver.rev, ver.manu, ver.smia, ver.cust);
	} else  {
		dev_info(onsemi->dev, "device %04x, cust %d\n",
			 ver.chip, ver.cust);
	}

	rc = devm_device_add_group(onsemi->dev, &ONSEMI_CORE_ATTR_GRP);
	if (rc < 0)
		goto out;

	rc = onsemi_core_thermal_init(onsemi);
	if (rc < 0)
		goto out;

	onsemi->version = ver;
	rc = 0;

out:
	onsemi_power_put(onsemi);

	return rc;
}
EXPORT_SYMBOL_GPL(onsemi_core_hw_init);

static int onsemi_querycap(struct onsemi_core *onsemi,
			   struct v4l2_capability *cap)
{
	strcpy(cap->driver, "onsemi-core");

	return 0;
}

static long onsemi_core_ioctl(struct v4l2_subdev *sd,
			      unsigned int cmd, void *arg)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		return onsemi_querycap(onsemi, arg);

	default:
		return -ENOTTY;
	}
}

static int __used onsemi_g_register(struct v4l2_subdev *sd,
				    struct v4l2_dbg_register *reg)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	int			rc = 0;
	unsigned int		addr;

	if (reg->size == 1)
		addr = ONSEMI_REGMAP_8BIT_REG_TO_ADDR(reg->reg);
	else
		addr = reg->reg;

	reg->val = onsemi_read(&rc, onsemi, addr);

	return rc;
}

static int __used onsemi_s_register(struct v4l2_subdev *sd,
				    struct v4l2_dbg_register const *reg)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	int			rc = 0;
	unsigned int		addr;

	if (reg->size == 1)
		addr = ONSEMI_REGMAP_8BIT_REG_TO_ADDR(reg->reg);
	else
		addr = reg->reg;

	onsemi_write(&rc, onsemi, addr, reg->val);

	return rc;
}

static int onsemi_s_power(struct v4l2_subdev *sd, int on)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	int			rc;
	int			num_users;

	num_users = atomic_add_return(on ? 1  : -1, &onsemi->num_v4lpwr_users);
	if (WARN_ON(num_users < 0)) {
		rc = -EINVAL;
		goto out;
	}

	if (on &&
	    test_and_set_bit(ONSEMI_FLAG_V4L_POWERED, &onsemi->flags)) {
		v4l2_warn(sd, "s_power(ON) called twice");
		/* noop; powered on through v4l2 api already */
		return 0;
	}

	if (!on &&
	    !test_and_clear_bit(ONSEMI_FLAG_V4L_POWERED, &onsemi->flags)) {
		v4l2_warn(sd, "s_power(OFF) called twice");
		/* noop; already powered down through v4l2 api */
		return 0;
	}

	if (on) {
		rc = onsemi_power_get(onsemi);

		if (rc == 0 && onsemi->ops->prepare)
			rc = onsemi->ops->prepare(onsemi);
		if (rc < 0)
			clear_bit(ONSEMI_FLAG_V4L_POWERED, &onsemi->flags);
	} else {
		onsemi_power_put(onsemi);
		rc = 0;
	}

out:
	if (rc < 0)
		atomic_add(on ? -1 : 1, &onsemi->num_v4lpwr_users);

	return rc;
}

static int onsemi_get_mbus_config(struct v4l2_subdev *sd,
				  unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct onsemi_core *onsemi = sd_to_onsemi(sd);
	struct onsemi_businfo const *bus_info = onsemi->active_bus;

	if (bus_info->bus_type == V4L2_MBUS_PARALLEL) {
		cfg->flags  = (V4L2_MBUS_MASTER |
			       V4L2_MBUS_HSYNC_ACTIVE_HIGH |
			       V4L2_MBUS_VSYNC_ACTIVE_HIGH |
			       V4L2_MBUS_DATA_ACTIVE_HIGH);
	}

	if (bus_info->bus_type == V4L2_MBUS_CSI2_DPHY) {
		if (bus_info->bus_width == 1)
			cfg->flags |= V4L2_MBUS_CSI2_1_LANE;
		else
			cfg->flags |= V4L2_MBUS_CSI2_2_LANE;
	}

	cfg->type = bus_info->bus_type;

	return 0;
}

static void onsemi_get_frame_size(struct onsemi_core const *onsemi,
				  struct onsemi_v4l_parm const *parm,
				  unsigned int *h_size_out,
				  unsigned int *v_size_out)
{
	struct onsemi_limits const	*limits = onsemi->limits;
	unsigned int			h_size;
	unsigned int			v_size;

	if (!parm)
		parm = onsemi->v4l_parm;

	h_size  = parm->crop.width;
	h_size += parm->hblank;
	h_size  = clamp_t(unsigned int, (h_size + 1) / 2 * 2,
			  limits->hlen.min, limits->hlen.max);

	v_size  = parm->crop.height;
	v_size += parm->vblank;
	v_size  = clamp_t(unsigned int, v_size,
			  limits->vlen.min, limits->vlen.max);

	if (h_size_out)
		*h_size_out = h_size;

	if (v_size_out)
		*v_size_out = v_size;
}

static unsigned long onsemi_frame_time_us(struct onsemi_core const *onsemi,
					  struct onsemi_v4l_parm const *parm,
					  struct onsemi_pll_freq const *freq)
{
	unsigned int			h_size;
	unsigned int			v_size;
	unsigned long			clk_pixel;
	uint64_t			res;

	clk_pixel = freq->clk_pixel;
	onsemi_get_frame_size(onsemi, parm, &h_size, &v_size);

	if (WARN_ON(h_size == 0 || v_size == 0 || clk_pixel == 0))
		return 0;

	res     = h_size * v_size;
	res    *= 1000000;

	return div64_ul(res, clk_pixel);
}

static void onsemi_enter_standby(struct onsemi_core *onsemi,
				 bool do_wait, int *err)
{
	unsigned int	timeout = do_wait ? 1000 : 0;
	bool		is_first = true;
	int		rc = err ? *err : 0;
	bool		is_smia;

	is_smia = (onsemi_read(&rc, onsemi, 0x301a) &
		   (ONSEMI_FLD_RESET_REGISTER_SMIA_DIS |
		    ONSEMI_FLD_RESET_REGISTER_PARALLEL_EN)) == 0;

	while (rc >= 0) {
		uint16_t	status;

		status = onsemi_read(&rc, onsemi, ONSEMI_REG_FRAME_STATUS);
		if (rc < 0)
			break;

		if (status & ONSEMI_FLD_FRAME_STATUS_STANDBY)
			break;

		if (timeout == 0) {
			v4l2_warn(&onsemi->subdev, "timeout while trying to enter standby\n");
			rc = -ETIMEDOUT;
			break;
		}

		if (is_first) {
			onsemi_update_bits(&rc, onsemi, 0x301a,
					   ONSEMI_FLD_RESET_REGISTER_STREAM |
					   ONSEMI_FLD_RESET_REGISTER_GPI_EN,
					   0);

			is_first = false;
		}

		--timeout;

		usleep_range(2000, 3000);
	}

	if (is_smia) {
		msleep(onsemi_frame_time_us(onsemi, NULL, onsemi->pll_freq) / 1000);

		onsemi_update_bits(&rc, onsemi, 0x301a,
				   ONSEMI_FLD_RESET_REGISTER_SMIA_DIS,
				   ONSEMI_FLD_RESET_REGISTER_SMIA_DIS);
	}

	if (test_and_clear_bit(ONSEMI_FLAG_SYNCCLK_ON, &onsemi->flags))
		clk_disable_unprepare(onsemi->synchclk);

	if (err)
		*err = rc;
}

static void onsemi_params_group_hold(struct onsemi_core *onsemi, bool ena, int *err)
{
	unsigned int const	addr = ONSEMI_REGMAP_8BIT_REG_TO_ADDR(0x3022);

	onsemi_write(err, onsemi, addr, ena ? 1 : 0);
}

static void onsemi_params_write_frame(struct onsemi_core *onsemi, int *err)
{
	struct onsemi_v4l_parm const	*parm = onsemi->v4l_parm;
	struct onsemi_businfo const	*bus_info = onsemi->active_bus;
	unsigned int			h_size;
	unsigned int			v_size;

	if (*err < 0)
		return;

	onsemi_get_frame_size(onsemi, NULL, &h_size, &v_size);

	if (onsemi->ops->hclk_mul_2 && bus_info) {
		if (bus_info->bus_type == V4L2_MBUS_CSI2_DPHY &&
		    bus_info->bus_width < 4)
			h_size *= 2;
	}

	onsemi_write(err, onsemi, 0x300a, v_size);
	onsemi_write(err, onsemi, 0x300c, h_size);

	if (onsemi->ops->has_smia_cfg) {
		onsemi_write(err, onsemi, 0x034c, parm->frame.width);
		onsemi_write(err, onsemi, 0x034e, parm->frame.height);
	}
}

static void onsemi_params_write_crop(struct onsemi_core *onsemi,
				     struct onsemi_v4l_parm const *parm,
				     bool write_back, int *err)
{
	if (WARN_ON(!parm && write_back))
		return;

	if (*err < 0)
		return;

	if (!parm)
		parm = onsemi->v4l_parm;

	onsemi_write(err, onsemi, 0x3002, parm->crop.top);
	onsemi_write(err, onsemi, 0x3004, parm->crop.left);
	onsemi_write(err, onsemi, 0x3006, parm->crop.top + parm->crop.height - 1);
	onsemi_write(err, onsemi, 0x3008, parm->crop.left + parm->crop.width - 1);

	if (*err < 0)
		return;

	if (write_back)
		onsemi->v4l_parm->crop = parm->crop;
}

static void onsemi_params_write_parallel(struct onsemi_core *onsemi, int *err)
{
	onsemi_write(err, onsemi, 0x31ae, 0x0200);
	onsemi_update_bits(err, onsemi, 0x301a,
			   ONSEMI_FLD_RESET_REGISTER_SMIA_DIS |
			   ONSEMI_FLD_RESET_REGISTER_PARALLEL_EN |
			   ONSEMI_FLD_RESET_REGISTER_DRIVE_PINS,

			   ONSEMI_FLD_RESET_REGISTER_SMIA_DIS |
			   ONSEMI_FLD_RESET_REGISTER_PARALLEL_EN |
			   ONSEMI_FLD_RESET_REGISTER_DRIVE_PINS);

}

static void onsemi_params_write_mipi(struct onsemi_core *onsemi, int *err)
{
	struct onsemi_businfo const	*info = onsemi->active_bus;

	if (WARN_ON(!info || info->bus_width < 1 || info->bus_width >= 16)) {
		*err = -EINVAL;
		return;
	}

	onsemi_write(err, onsemi, 0x31ae, (2 << 8) | info->bus_width);
	onsemi_update_bits(err, onsemi, 0x301a,
			   ONSEMI_FLD_RESET_REGISTER_SMIA_DIS |
			   ONSEMI_FLD_RESET_REGISTER_PARALLEL_EN |
			   ONSEMI_FLD_RESET_REGISTER_DRIVE_PINS,
			   0);
}

static void onsemi_params_write_pll(struct onsemi_core *onsemi, int *err)
{
	unsigned int	bpp = onsemi->v4l_parm->bpp;

	if (*err < 0)
		return;

	if (onsemi->active_bus->bus_type == V4L2_MBUS_PARALLEL)
		bpp = 12;

	onsemi_enter_standby(onsemi, true, err);
	if (*err < 0)
		goto out;

	*err = onsemi_core_pll_set(onsemi, onsemi->pll_cfg, onsemi->pll_freq);
	if (*err < 0)
		goto out;

	if (!onsemi->ops->has_smia_cfg)
		onsemi_write(err, onsemi, 0x31ac, (bpp << 8) | bpp);
	else
		onsemi_write(err, onsemi, 0x0112, (bpp << 8) | bpp);

out:
	if (*err < 0)
		v4l2_err(&onsemi->subdev, "failed to set PLL: %d\n", *err);
}

static int _onsemi_s_stream_on(struct onsemi_core *onsemi)
{
	int			rc;

	rc = onsemi_power_get(onsemi);
	if (rc < 0)
		return rc;

	mutex_lock(&onsemi->lock);

	rc = onsemi->ops->pll_calculate(onsemi);
	if (rc < 0)
		goto out;

	onsemi_params_write_pll(onsemi, &rc);

	switch (onsemi->active_bus->bus_type) {
	case V4L2_MBUS_PARALLEL:
		onsemi_params_write_parallel(onsemi, &rc);
		break;

	case V4L2_MBUS_CSI2_DPHY:
		onsemi_params_write_mipi(onsemi, &rc);
		break;

	default:
		WARN_ON(1);
		rc = -EINVAL;
	}

	if (!IS_ENABLED(CONFIG_THERMAL)) {
		onsemi_update_bits(&rc, onsemi, 0x30b4,
				   BIT(0) | BIT(5) | BIT(4),
				   BIT(0) | BIT(5) | BIT(4));
	}

	onsemi_update_bits(&rc, onsemi, 0x301a,
			   ONSEMI_FLD_RESET_REGISTER_LOCK_REG, 0);
	onsemi_write(&rc, onsemi, 0x301e, onsemi->v4l_parm->data_pedestal);

	onsemi_params_group_hold(onsemi, true, &rc);

	onsemi_params_write_frame(onsemi, &rc);
	onsemi_params_write_crop(onsemi, NULL, false, &rc);

	if (rc >= 0 && onsemi->ops->stream_on)
		rc = onsemi->ops->stream_on(onsemi);

	onsemi_params_group_hold(onsemi, false, &rc);

	onsemi_write(&rc, onsemi, ONSEMI_REG_MASK_CORRUPTED_FRAMES, 1);
	onsemi_write(&rc, onsemi, ONSEMI_REG_MOD_SELECT, 1);

	if (rc < 0)
		goto out;

	rc = 0;

out:
	mutex_unlock(&onsemi->lock);

	if (rc < 0)
		onsemi_power_put(onsemi);

	return rc;
}

static int _onsemi_s_stream_off(struct onsemi_core *onsemi)
{
	int		rc = 0;

	mutex_lock(&onsemi->lock);

	if (!IS_ENABLED(CONFIG_THERMAL)) {
		onsemi_update_bits(&rc, onsemi, 0x30b4,
				   BIT(0) | BIT(5) | BIT(4),
				   BIT(5));
	}

	if (rc >= 0 && onsemi->ops->stream_off)
		rc = onsemi->ops->stream_off(onsemi);

	if (rc == 0)
		onsemi_enter_standby(onsemi, true, &rc);

	/* TODO: improve error handling! */
	rc = 0;

	if (rc < 0)
		goto out;

	/* TODO: ignore errors... */
	rc = 0;

out:
	mutex_unlock(&onsemi->lock);

	onsemi_power_put(onsemi);

	return rc;
}

static int onsemi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	int			rc;

	if (!onsemi->active_bus) {
		dev_dbg(onsemi->dev, "no active bus\n");
		return -EPIPE;
	}

	if (enable &&
	    test_and_set_bit(ONSEMI_FLAG_V4L_STREAMING, &onsemi->flags)) {
		return -EBUSY;
	}

	if (!enable &&
	    !test_and_clear_bit(ONSEMI_FLAG_V4L_STREAMING, &onsemi->flags)) {
		return -EBADF;
	}

	if (enable) {
		rc = _onsemi_s_stream_on(onsemi);
		if (rc < 0)
			clear_bit(ONSEMI_FLAG_V4L_STREAMING, &onsemi->flags);
	} else {
		rc = _onsemi_s_stream_off(onsemi);
		if (rc < 0)
			set_bit(ONSEMI_FLAG_V4L_STREAMING, &onsemi->flags);
	}

	return rc;
}

static int _onsemi_enum_mbus_code_bayer(struct onsemi_core *onsemi,
					struct onsemi_businfo const *info,
					unsigned int idx,
					unsigned int *code)
{
	static unsigned int const	FMT8[] = {
		[0] = MEDIA_BUS_FMT_SBGGR8_1X8,
		[1] = MEDIA_BUS_FMT_SGBRG8_1X8,
		[2] = MEDIA_BUS_FMT_SGRBG8_1X8,
		[3] = MEDIA_BUS_FMT_SRGGB8_1X8,
	};

	static unsigned int const	FMT10[] = {
		[0] = MEDIA_BUS_FMT_SBGGR10_1X10,
		[1] = MEDIA_BUS_FMT_SGBRG10_1X10,
		[2] = MEDIA_BUS_FMT_SGRBG10_1X10,
		[3] = MEDIA_BUS_FMT_SRGGB10_1X10,
	};

	static unsigned int const	FMT12[] = {
		[0] = MEDIA_BUS_FMT_SBGGR12_1X12,
		[1] = MEDIA_BUS_FMT_SGBRG12_1X12,
		[2] = MEDIA_BUS_FMT_SGRBG12_1X12,
		[3] = MEDIA_BUS_FMT_SRGGB12_1X12,
	};

	unsigned long const	KNOWN_FMT = BIT(8) | BIT(10) | BIT(12);

	unsigned long		supported_bpp = onsemi->ops->supported_bpp;
	unsigned int		bit;
	unsigned int		bpp = 0;

	for_each_set_bit(bit, &supported_bpp, 16) {
		unsigned int	tmp;

		if (WARN_ON(abs(info->dout_sft) > bit))
			/* should be checked in _init()... */
			continue;

		tmp = bit + info->dout_sft;

		/* filter out unsupported formats; e.g. when Y10 gets shifted
		 * 4 bits */
		if (!test_bit(tmp, &KNOWN_FMT))
			/* do *not* decrease 'idx' */
			continue;

		if (idx < 4) {
			bpp = tmp;
			break;
		}

		idx -= 4;
	}

	if (bpp == 0) {
		dev_dbg(onsemi->dev, "bayer mbus code not found\n");
		return -EINVAL;
	}

	switch (bpp) {
	case 8:
		*code = FMT8[idx % 4];
		break;
	case 10:
		*code = FMT10[idx % 4];
		break;
	case 12:
		*code = FMT12[idx % 4];
		break;
	default:
		v4l2_warn(&onsemi->subdev, "unsupported bpp %d\n", bpp);
		return -EINVAL;
	}

	return 0;
}

static int _onsemi_enum_mbus_code_mono(struct onsemi_core *onsemi,
				       struct onsemi_businfo const *info,
				       unsigned int idx,
				       unsigned int *code)
{
	unsigned long const	KNOWN_FMT = BIT(8) | BIT(10) | BIT(12) | BIT(16);
	unsigned long		supported_bpp = onsemi->ops->supported_bpp;
	unsigned int		bit;
	unsigned int		bpp = 0;

	for_each_set_bit(bit, &supported_bpp, 16) {
		unsigned int	tmp;

		if (WARN_ON(abs(info->dout_sft) > bit))
			/* should be checked in _init()... */
			continue;

		tmp = bit + info->dout_sft;

		/* filter out unsupported formats; e.g. when Y10 gets shifted
		 * 4 bits */
		if (!test_bit(tmp, &KNOWN_FMT))
			/* do *not* decrease 'idx' */
			continue;

		if (idx == 0) {
			bpp = tmp;
			break;
		}

		--idx;
	}

	if (bpp == 0) {
		dev_dbg(onsemi->dev, "mono mbus code not found\n");
		return -EINVAL;
	}

	switch (bpp) {
	case 8:
		*code = MEDIA_BUS_FMT_Y8_1X8;
		break;
	case 10:
		*code = MEDIA_BUS_FMT_Y10_1X10;
		break;
	case 12:
		*code = MEDIA_BUS_FMT_Y12_1X12;
		break;
#ifdef MEDIA_BUS_FMT_Y16_1X16
	case 16:
		*code = MEDIA_BUS_FMT_Y16_1X16;
		break;
#endif
	default:
		v4l2_warn(&onsemi->subdev, "unsupported bpp %d\n", bpp);
		return -EINVAL;
	}

	return 0;
}

static int onsemi_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct onsemi_core		*onsemi = sd_to_onsemi(sd);
	struct onsemi_businfo const	*info;
	unsigned int			fmt_code;
	int				rc;

	if (onsemi->ops->enum_mbus_code)
		return onsemi->ops->enum_mbus_code(onsemi, sd_state, code);

	info = onsemi_get_businfo(onsemi, code->pad);
	if (!info)
		return -EINVAL;

	switch (onsemi->color_mode) {
	case ONSEMI_COLOR_MONOCHROME:
		rc = _onsemi_enum_mbus_code_mono(onsemi, info, code->index, &fmt_code);
		break;
	case ONSEMI_COLOR_BAYER:
		rc = _onsemi_enum_mbus_code_bayer(onsemi, info, code->index, &fmt_code);
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	if (rc < 0)
		return rc;

	code->code = fmt_code;

	return 0;
}

static int onsemi_mbus_to_bpp(struct onsemi_core const *onsemi,
			      struct onsemi_businfo const *info,
			      unsigned int code, unsigned int *bpp)
{
	int		rc = 0;
	signed int	raw_bpp;

	switch (onsemi->color_mode) {
	case ONSEMI_COLOR_MONOCHROME:
		switch (code) {
		case MEDIA_BUS_FMT_Y8_1X8:
			raw_bpp = 8;
			break;
		case MEDIA_BUS_FMT_Y10_1X10:
			raw_bpp = 10;
			break;
		case MEDIA_BUS_FMT_Y12_1X12:
			raw_bpp = 12;
			break;
#ifdef MEDIA_BUS_FMT_Y16_1X16
		case MEDIA_BUS_FMT_Y16_1X16:
			raw_bpp = 16;
			break;
#endif
		default:
			v4l2_warn(&onsemi->subdev,
				  "unsupported mbus fmt %04x for mono sensor\n",
				  code);
			rc = -EINVAL;
			break;
		}
		break;

	case ONSEMI_COLOR_BAYER:
		switch (code) {
		case MEDIA_BUS_FMT_SBGGR8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
		case MEDIA_BUS_FMT_SGRBG8_1X8:
		case MEDIA_BUS_FMT_SRGGB8_1X8:
			raw_bpp = 8;
			break;
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			raw_bpp = 10;
			break;
		case MEDIA_BUS_FMT_SBGGR12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			raw_bpp = 12;
			break;
		default:
			v4l2_warn(&onsemi->subdev,
				  "unsupported mbus fmt %04x for mono sensor\n",
				  code);
			rc = -EINVAL;
			break;
		}
		break;

	default:
		/* TODO: implement color mode */
		WARN_ON(1);
		rc = -ENOTTY;
	}

	if (rc < 0) {
		v4l2_warn(&onsemi->subdev, "unsupported color format %04x\n",
			  code);
		goto out;
	}

	raw_bpp -= info->dout_sft;
	if (raw_bpp < 0 || raw_bpp > 16) {
		v4l2_warn(&onsemi->subdev, "bad bpp/sft %d\n", raw_bpp);
		rc = -EINVAL;
		goto out;
	}

	if (!test_bit(raw_bpp, &onsemi->ops->supported_bpp)) {
		v4l2_warn(&onsemi->subdev, "bad bpp %d\n", raw_bpp);
		rc = -EINVAL;
		goto out;
	}

	*bpp = raw_bpp;
	rc = 0;

out:
	return rc;
}

static int onsemi_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	struct onsemi_v4l_parm	*parm = onsemi->v4l_parm;
	struct onsemi_businfo const	*info;
	unsigned int		w;
	unsigned int		h;
	unsigned int		bpp;
	unsigned int		max_w;
	unsigned int		max_h;
	int			rc;
	struct onsemi_pll_cfg		*pll_cfg;
	struct onsemi_pll_freq		*pll_freq;
	bool				bad_again = false;

	info = onsemi_get_businfo(onsemi, format->pad);
	if (!info)
		return -EINVAL;

	if (test_bit(ONSEMI_FLAG_V4L_STREAMING, &onsemi->flags) &&
	    format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EBUSY;

again:
	w = format->format.width;
	h = format->format.height;

	if (format->format.field == V4L2_FIELD_ANY)
		format->format.field = V4L2_FIELD_NONE;

	if (format->format.colorspace == V4L2_COLORSPACE_DEFAULT)
		format->format.colorspace = V4L2_COLORSPACE_RAW;

	if (format->format.xfer_func == V4L2_XFER_FUNC_DEFAULT)
		format->format.xfer_func = V4L2_XFER_FUNC_NONE;

	rc = onsemi_mbus_to_bpp(onsemi, info, format->format.code, &bpp);
	if (rc < 0)
		goto bad_parm;

	if (format->format.field != V4L2_FIELD_NONE ||
	    format->format.xfer_func != V4L2_XFER_FUNC_NONE ||
	    format->format.colorspace != V4L2_COLORSPACE_RAW) {
		v4l2_warn(sd, "unsupported params (%d, %d, %d)",
			  format->format.field, format->format.xfer_func,
			  format->format.colorspace);
		goto bad_parm;
	}

	max_w  = onsemi->limits->x.max - onsemi->limits->x.min + 1;
	max_h  = onsemi->limits->y.max - onsemi->limits->y.min + 1;
	max_w /= parm->x_scale;
	max_h /= parm->y_scale;

	if (w > max_w || h > max_h) {
		v4l2_warn(sd, "frame %dx%d too large (max %dx%d)\n",
			  w, h, max_w, max_h);
		goto bad_parm;
	}

	format->format.width  = w;
	format->format.height = h;

	if (onsemi->ops->set_fmt) {
		rc = onsemi->ops->set_fmt(onsemi, sd_state, format, bpp);
		if (rc < 0)
			goto bad_parm;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    info == onsemi->active_bus) {
		pll_cfg  = onsemi->pll_cfg;
		pll_freq = onsemi->pll_freq;
	} else {
		pll_cfg  = NULL;
		pll_freq = NULL;
	}

	rc = onsemi_calculate_pll(onsemi, info, bpp, pll_cfg, pll_freq);
	if (rc < 0)
		goto bad_parm;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		struct onsemi_v4l_parm	*parm = onsemi->v4l_parm;

		parm->frame.height = format->format.height;
		parm->frame.width  = format->format.width;
		parm->code         = format->format.code;
		parm->bpp          = bpp;
	}

	return 0;

bad_parm:
	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE || bad_again)
		return -EINVAL;

	bad_again = true;

	format->format = (struct v4l2_mbus_framefmt) {
		.field		= V4L2_FIELD_NONE,
		.xfer_func	= V4L2_XFER_FUNC_NONE,
		.colorspace	= V4L2_COLORSPACE_RAW,
		.height		= onsemi->v4l_parm->frame.height,
		.width		= onsemi->v4l_parm->frame.width,
		.code		= onsemi->v4l_parm->code,
	};

	goto again;
}

static int onsemi_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	struct onsemi_v4l_parm	*parm = onsemi->v4l_parm;

	format->pad = 0;
	format->format = (struct v4l2_mbus_framefmt) {
		.width		= parm->frame.width,
		.height		= parm->frame.height,
		.code		= parm->code,
		.field		= V4L2_FIELD_NONE,
		.colorspace	= V4L2_COLORSPACE_RAW,
		.xfer_func	= V4L2_XFER_FUNC_NONE,
	};

	return 0;
}

static int onsemi_check_sel_bayer(struct onsemi_core const *onsemi,
				  struct v4l2_subdev_selection const *s)
{
	struct onsemi_v4l_parm const	*parm = onsemi->v4l_parm;
	unsigned int			idx;
	unsigned int			x_mod;
	unsigned int			y_mod;

	switch (parm->code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		idx = 0x10;		/* y+1, x+0 */
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		idx = 0x11;		/* y+1, x+1 */
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		idx = 0x00;		/* y+0, x+0 */
		break;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		idx = 0x01;		/* y+0, x+1 */
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	x_mod = parm->x_skip;
	y_mod = parm->y_skip;

	if (WARN_ON(x_mod == 0) || WARN_ON(y_mod == 0))
		return -EINVAL;

	if ((s->r.left + x_mod - ((idx >> 0) & 0xf)) % x_mod != 0 ||
	    (s->r.top  + y_mod - ((idx >> 4) & 0xf)) % y_mod != 0) {
		v4l2_info(&onsemi->subdev,
			  "oddly aligned crop area (%dx%d %% %dx%d)\n",
			  s->r.left, s->r.top, x_mod, y_mod);
		return -EINVAL;
	}

	return 0;
}

static int onsemi_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *s)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	struct onsemi_v4l_parm	*parm = onsemi->v4l_parm;
	struct onsemi_limits const *limits = onsemi->limits;
	bool			is_try = s->which == V4L2_SUBDEV_FORMAT_TRY;
	bool			is_streaming;
	int			rc;

	if (s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (s->r.left < limits->x.min ||
	    s->r.top < limits->y.min ||
	    s->r.width > limits->x.max - onsemi->limits->x.min + 1 ||
	    s->r.height > limits->y.max - onsemi->limits->y.min + 1) {
		v4l2_warn(sd, "bad selection %dx%d+%dx%d\n",
			  s->r.left, s->r.top, s->r.width, s->r.height);

		/* TODO: implement V4L2_SEL_FLAG_GE/LE handling? */
		return -EINVAL;
	}

	if (is_try)
		return 0;

	mutex_lock(&onsemi->lock);

	switch (onsemi->color_mode) {
	case ONSEMI_COLOR_MONOCHROME:
		break;
	case ONSEMI_COLOR_BAYER:
		rc = onsemi_check_sel_bayer(onsemi, s);
		break;
	default:
		WARN_ON(1);
		rc = -EINVAL;
		break;
	}

	is_streaming = test_bit(ONSEMI_FLAG_V4L_STREAMING, &onsemi->flags);

	if ((s->r.width != parm->crop.width) ||
	    (s->r.height != parm->crop.height)) {
		if (is_streaming) {
			rc = -EBUSY;
			goto out;
		}
	}

	if (!is_streaming) {
		parm->crop = s->r;
		rc = 0;
	} else {
		struct onsemi_v4l_parm	tmp_parm = *parm;

		tmp_parm.crop = s->r;

		rc = 0;

		onsemi_params_group_hold(onsemi, 1, &rc);
		onsemi_params_write_crop(onsemi, &tmp_parm, true, &rc);
		onsemi_params_group_hold(onsemi, 0, &rc);
	}

	if (rc < 0) {
		v4l2_warn(sd, "failed to set crop params: %d\n", rc);
		goto out;
	}

	rc = 0;


out:
	mutex_unlock(&onsemi->lock);

	return rc;
}

static int onsemi_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *s)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	struct onsemi_v4l_parm	*parm = onsemi->v4l_parm;
	struct onsemi_limits const *limits = onsemi->limits;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		mutex_lock(&onsemi->lock);
		s->r = parm->crop;
		mutex_unlock(&onsemi->lock);
		break;

	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r = (struct v4l2_rect) {
			.left	= limits->x.min,
			.top	= limits->y.min,
			.width	= limits->x.max - limits->x.min + 1,
			.height = limits->y.max - limits->y.min + 1,
		};
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int onsemi_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct onsemi_core		*onsemi = sd_to_onsemi(sd);
	struct onsemi_businfo const	*info;
	struct onsemi_pll_freq		freq;
	int				rc;
	unsigned long			tm;

	info = onsemi_get_businfo(onsemi, interval->pad);
	if (!info)
		return -EINVAL;

	mutex_lock(&onsemi->lock);

	rc = onsemi_calculate_pll(onsemi, info, onsemi->v4l_parm->bpp,
				  NULL, &freq);
	if (rc < 0)
		goto out;

	tm = onsemi_frame_time_us(onsemi, NULL, &freq);

	interval->interval = (struct v4l2_fract) {
		.numerator	= tm,
		.denominator	= 1000000,
	};

	rc = 0;

out:
	mutex_unlock(&onsemi->lock);

	return rc;
}

struct onsemi_interval_search_parm {
	struct onsemi_core const	*onsemi;
	struct v4l2_fract const		*interval;
	struct onsemi_pll_freq		freq;
	struct onsemi_v4l_parm		parm;
};

/**
 *  run a binary search of 'val' between 'min' and 'max' until delta is
 *  below 'epsilon'
 *
 *  @parm object with static/constant search parameters
 *  @min  lower border of search
 *  @max  upper border of esearch
 *  @val  value to be searched; the actual input value will be tested first
 *  @epsilon allowed delta in number of pixels
 */
static int _onsemi_interval_search(struct onsemi_interval_search_parm const *parm,
				   unsigned long min, unsigned long max,
				   unsigned int *val,
				   unsigned int epsilon)
{
	/* we will scale 'min', 'max' + '*val' in the binary search to ensure
	 * that the loop terminates **and** finds an element; 'epsilon' must
	 * be large enough (e.g. a whole row) so that we can find the
	 * middle.
	 *
	 * This parameter does NOT increase the number of loops but just
	 * compensates rounding effects */
	static unsigned int const	V_SCALE = 4;

	uint64_t		e = epsilon;
	uint64_t		num = parm->interval->numerator;
	unsigned long		denom = parm->interval->denominator;
	unsigned long		v = *val;

	if (WARN_ON(parm->freq.clk_pixel == 0))
		return -EINVAL;

	/* adjust existing 'val' to limits */
	if (v < min)
		v = min;

	if (v > max)
		v = max;

	/* scale interval because we are working with us internally */
	num *= 1000000;

	/* scale/transform epsilon from number of pixels to time for them in
	 * us */
	e *= 1000000;
	e  = div64_ul(e, parm->freq.clk_pixel);
	e += 5;				/* some arbitrary adjustment... */

	/* at the end we try to satisfy
	 *
	 *    abs(tm - num/denom) < epsilon
	 *
	 * which will be transformed to
	 *
	 *    abs(tm * denom - num) < epsilon * denom
	 *
	 * Right side is an invariant which can be calculated here, outside
	 * the loop
	 */
	e *= denom;

	min *= V_SCALE;
	max *= V_SCALE;
	v   *= V_SCALE;

	pr_debug("%s  : [%lu,%lu]/%lu -> e=%llu\n", __func__, min, max, v, e);

	do {
		unsigned long		tm;
		uint64_t		tm_s;
		uint64_t		delta;

		/* revert scaling and write back the real value so it will be
		 * used in onsemi_frame_time_us() call below */
		*val = v / V_SCALE;

		tm    = onsemi_frame_time_us(parm->onsemi,
					     &parm->parm, &parm->freq);
		tm_s  = tm;
		tm_s *= denom;

		if (tm_s < num) {
			delta = num - tm_s;
			min   = v;
		} else {
			delta = tm_s - num;
			max   = v;
		}

		pr_debug("  %s: [%lu,%lu]/%lu -> tm=%llu, delta=%llu\n",
			 __func__, min, max, v, tm_s, delta);

		if (delta < e)
			return tm;

		v = (min + max) / 2;
	} while (min + V_SCALE - 1 < max);

	return -ERANGE;
}

static int onsemi_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct onsemi_core		*onsemi = sd_to_onsemi(sd);
	struct onsemi_businfo const	*info;
	struct v4l2_ctrl		*vblank_ctrl;
	signed long			rc;

	struct onsemi_interval_search_parm	parm = {
		.onsemi		= onsemi,
		.interval	= &interval->interval,
	};

	/* used to reflect changes in the v4l2 ctrl */
	vblank_ctrl = v4l2_ctrl_find(&onsemi->ctrls, V4L2_CID_VBLANK);

	mutex_lock(&onsemi->lock);

	if (onsemi_is_streaming(onsemi)) {
		rc = -EBUSY;
		goto out;
	}

	info = onsemi_get_businfo(onsemi, interval->pad);
	if (!info) {
		rc = -EINVAL;
		goto out;
	}

	rc = onsemi_calculate_pll(onsemi, info, onsemi->v4l_parm->bpp,
				  NULL, &parm.freq);
	if (rc < 0)
		goto out;

	parm.parm = *onsemi->v4l_parm;

	rc = _onsemi_interval_search(&parm,
				     onsemi->limits->vblank_min,
				     onsemi->limits->vlen.max -
				     parm.parm.crop.height,
				     &parm.parm.vblank,
				     0xffff);
	if (rc < 0) {
		v4l2_warn(sd, "failed to find matching parameters for given interval; assuming limits\n");
		rc = onsemi_frame_time_us(onsemi, &parm.parm, &parm.freq);
	}

	interval->interval = (struct v4l2_fract) {
		.numerator	= rc,
		.denominator	= 1000000,
	};

	*onsemi->v4l_parm = parm.parm;

	if (vblank_ctrl)
		vblank_ctrl->cur.val = onsemi->v4l_parm->vblank;

	rc = 0;

out:
	mutex_unlock(&onsemi->lock);

	return rc;
}

static int onsemi_link_validate(struct v4l2_subdev *sink,
				struct media_link *link,
				struct v4l2_subdev_format *source_fmt,
				struct v4l2_subdev_format *sink_fmt)
{
	return 0;
}

static int onsemi_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct onsemi_core		*onsemi = ctrl->priv;
	struct onsemi_v4l_parm		*parm = onsemi->v4l_parm;
	int				rc = 0;

	mutex_lock(&onsemi->lock);

	if (onsemi->ops->ctrl_set &&
	    onsemi->ops->ctrl_set(onsemi, ctrl, &rc))
		goto out;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		if (onsemi_is_streaming(onsemi)) {
			rc = -EBUSY;
			goto out;
		}
		parm->vblank = ctrl->val;
		break;

	case V4L2_CID_HBLANK:
		if (onsemi_is_streaming(onsemi)) {
			rc = -EBUSY;
			goto out;
		}
		parm->hblank = ctrl->val;
		break;

	case V4L2_CID_HFLIP:
		onsemi_update_bits(&rc, onsemi, 0x3040, BIT(14),
				   ctrl->val ? BIT(14) : 0);
		break;
	case V4L2_CID_VFLIP:
		onsemi_update_bits(&rc, onsemi, 0x3040, BIT(15),
				   ctrl->val ? BIT(15) : 0);
		break;
	case V4L2_CID_EXPOSURE:
		onsemi_write(&rc, onsemi, 0x3012, ctrl->val);
		break;
	case V4L2_CID_X_EXPOSURE_FINE:
		onsemi_write(&rc, onsemi, 0x3014, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		onsemi_write(&rc, onsemi, 0x3072, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		onsemi_write(&rc, onsemi, 0x3074, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		onsemi_write(&rc, onsemi, 0x3076, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		onsemi_write(&rc, onsemi, 0x3078, ctrl->val);
		break;
	default:
		dev_dbg(onsemi->dev, "unhandled control %08x (%s)\n", ctrl->id,
			v4l2_ctrl_get_name(ctrl->id));
		rc = -ENOTTY;
		break;
	}

out:
	mutex_unlock(&onsemi->lock);

	return rc;
}

static int onsemi_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct onsemi_core		*onsemi = ctrl->priv;
	int				rc = 0;

	mutex_lock(&onsemi->lock);

	if (onsemi->ops->ctrl_get &&
	    onsemi->ops->ctrl_get(onsemi, ctrl, &rc))
		goto out;

	switch (ctrl->id) {
	case V4L2_CID_X_HBLANK_EFFECTIVE: {
		unsigned int			h_size;
		struct onsemi_v4l_parm const	*parm = onsemi->v4l_parm;

		if (!parm) {
			rc = -EPIPE;
			goto out;
		}

		onsemi_get_frame_size(onsemi, parm, &h_size, NULL);
		ctrl->val = h_size - parm->crop.width;
		rc = 0;
		break;
	}

	default:
		dev_dbg(onsemi->dev, "unhandled control %08x (%s)\n", ctrl->id,
			v4l2_ctrl_get_name(ctrl->id));
		rc = -ENOTTY;
		break;
	}

out:
	mutex_unlock(&onsemi->lock);

	return rc;
}

struct v4l2_ctrl_ops const		onsemi_core_ctrl_ops = {
	.s_ctrl			= onsemi_s_ctrl,
	.g_volatile_ctrl	= onsemi_g_ctrl,
};
EXPORT_SYMBOL_GPL(onsemi_core_ctrl_ops);

static struct v4l2_ctrl_config const	onsemi_core_ctrls[] = {
	{
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_VBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_HBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_HBLANK_EFFECTIVE,
		.name		= "hblank (effective)",
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.flags		= (V4L2_CTRL_FLAG_READ_ONLY |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 800,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_X_EXPOSURE_FINE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "exposure fine",
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 10,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_RED,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_GREENR,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_GREENB,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_BLUE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
	}, {
		.ops		= &onsemi_core_ctrl_ops,
		.id		= V4L2_CID_LINK_FREQ,
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.flags		= (V4L2_CTRL_FLAG_READ_ONLY |
				   V4L2_CTRL_FLAG_VOLATILE),
		.min		= 0,
		.max		= 0,
	},
};


static struct v4l2_subdev_core_ops const	onsemi_subdev_core_ops = {
	.ioctl			= onsemi_core_ioctl,
	.s_power		= onsemi_s_power,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.s_register		= onsemi_s_register,
	.g_register		= onsemi_g_register,
#endif
};

static struct v4l2_subdev_video_ops const	onsemi_subdev_video_ops = {
	.s_stream		= onsemi_s_stream,
	.g_frame_interval	= onsemi_g_frame_interval,
	.s_frame_interval	= onsemi_s_frame_interval,
};

static struct v4l2_subdev_pad_ops const		onsemi_subdev_pad_ops = {
	.enum_mbus_code		= onsemi_enum_mbus_code,
	.set_fmt		= onsemi_set_fmt,
	.get_fmt		= onsemi_get_fmt,
	.set_selection		= onsemi_set_selection,
	.get_selection		= onsemi_get_selection,
	.link_validate		= onsemi_link_validate,
	.get_mbus_config	= onsemi_get_mbus_config,
};

static struct v4l2_subdev_ops const		onsemi_subdev_ops = {
	.core			= &onsemi_subdev_core_ops,
	.video			= &onsemi_subdev_video_ops,
	.pad			= &onsemi_subdev_pad_ops,
};

static int onsemi_setup_single_ctrl(struct onsemi_core *onsemi,
				    struct v4l2_ctrl_config const *ctrl_tmpl)
{
	struct onsemi_limits const	*limits = onsemi->limits;
	struct v4l2_ctrl_config		ctrl = *ctrl_tmpl;
	struct v4l2_ctrl		*cptr;
	int				rc;

	rc = onsemi->ctrls.error;
	if (WARN_ON_ONCE(rc < 0))
		return rc;

	switch (ctrl.id) {
	case V4L2_CID_HBLANK:
		ctrl.min = limits->hblank_min;
		ctrl.def = ctrl.min;
		break;
	case V4L2_CID_VBLANK:
		ctrl.min = limits->vblank_min;
		ctrl.def = ctrl.min;
		break;
	case V4L2_CID_LINK_FREQ:
		ctrl.qmenu_int = &onsemi->pll_freq->link_freq;
		break;
	default:
		break;
	}

	if (onsemi->ops->ctrl_setup_pre &&
	    !onsemi->ops->ctrl_setup_pre(onsemi, &ctrl))
		/* control filtered out */
		return 0;

	dev_dbg(onsemi->dev, "%s: adding '%s'\n", __func__, ctrl.name);

	cptr = v4l2_ctrl_new_custom(&onsemi->ctrls, &ctrl, onsemi);
	rc = onsemi->ctrls.error;
	if (rc < 0) {
		v4l2_warn(&onsemi->subdev,
			  "failed to register control '%s' (%x): %d\n",
			  ctrl.name ? ctrl.name : v4l2_ctrl_get_name(ctrl.id),
			  ctrl.id, rc);
		goto out;
	}

	if (onsemi->ops->ctrl_setup_post)
		onsemi->ops->ctrl_setup_post(onsemi, cptr);

	rc = 0;

out:
	return rc;
}

static int onsemi_setup_ctrls(struct onsemi_core *onsemi)
{
	size_t			i;
	int			rc = 0;

	for (i = 0; i < ARRAY_SIZE(onsemi_core_ctrls) && !(rc < 0); ++i)
		rc = onsemi_setup_single_ctrl(onsemi, &onsemi_core_ctrls[i]);

	for (i = 0; i < onsemi->ops->num_ctrls && !(rc < 0); ++i)
		rc = onsemi_setup_single_ctrl(onsemi, &onsemi->ops->ctrls[i]);

	return rc;
}

static int onsemi_subdev_registered(struct v4l2_subdev *sd)
{
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	int			rc;

	rc = v4l2_ctrl_handler_setup(&onsemi->ctrls);
	if (rc < 0) {
		v4l2_err(sd, "failed to setup controls: %d\n", rc);
		goto out;
	}

	rc = 0;

out:
	return rc;
}

static struct v4l2_subdev_internal_ops const	onsemi_subdev_internal_ops = {
	.registered		= onsemi_subdev_registered,
};

static int onsemi_fill_v4l_parm(struct onsemi_core *onsemi,
				 struct onsemi_v4l_parm *parm)
{
	int			rc;
	unsigned int		x[2];
	unsigned int		y[2];
	size_t			i;
	struct v4l2_subdev_mbus_code_enum	mbus_code = {
		.index	= 0,
	};

	for (i = 0; i < ARRAY_SIZE(onsemi->bus_info); ++i) {
		if (onsemi->bus_info[i].is_used) {
			mbus_code.pad = i;
			break;
		}
	}

	rc = 0;
	y[0] = onsemi_read(&rc, onsemi, 0x3002);
	x[0] = onsemi_read(&rc, onsemi, 0x3004);
	y[1] = onsemi_read(&rc, onsemi, 0x3006);
	x[1] = onsemi_read(&rc, onsemi, 0x3008);

	if (rc < 0) {
		dev_err(onsemi->dev, "failed to read active window: %d\n", rc);
		goto out;
	}

	if (x[0] > x[1] || y[0] > y[1] ||
	    x[1] - x[0]  <  onsemi->limits->x.min ||
	    y[1] - y[0]  <  onsemi->limits->y.min ||
	    x[1] - x[0] >=  onsemi->limits->x.max ||
	    y[1] - y[0] >=  onsemi->limits->y.max) {
		dev_warn(onsemi->dev, "unexpected window (%d,%d)-(%d, %d)\n",
			 x[0], y[0], x[1], y[1]);
		rc = -EINVAL;
		goto out;
	}

	parm->frame = (struct v4l2_frmsize_discrete) {
		.width  = x[1] - x[0] + 1,
		.height = y[1] - y[0] + 1,
	};

	parm->crop = (struct v4l2_rect) {
		.left	= x[0],
		.top	= y[0],
		.width  = x[1] - x[0] + 1,
		.height = y[1] - y[0] + 1,
	};

	parm->bpp     = 8;
	parm->x_scale = 1;
	parm->y_scale = 1;
	parm->hblank = onsemi->limits->hblank_min;
	parm->vblank = onsemi->limits->vblank_min;
	parm->x_skip = 1;
	parm->y_skip = 1;
	parm->data_pedestal = onsemi_read(&rc, onsemi, 0x301e);

	rc = onsemi_enum_mbus_code(&onsemi->subdev, NULL, &mbus_code);
	if (rc < 0)
		goto out;

	parm->code = mbus_code.code;

	rc = 0;

out:
	return rc;
}

static int onsemi_subdev_link_validate(struct media_link *link)
{
	return 0;
}

static int onsemi_subdev_link_setup(struct media_entity *entity,
				    struct media_pad const *local,
				    struct media_pad const *remote,
				    u32 flags)
{
	struct v4l2_subdev	*sd =
		media_entity_to_v4l2_subdev(entity);
	struct onsemi_core	*onsemi = sd_to_onsemi(sd);
	struct onsemi_businfo const	*bus_info =
		onsemi_get_businfo(onsemi, local->index);
	int				rc = 0;

	if (WARN_ON(!bus_info || !onsemi->v4l_parm))
		return -EINVAL;

	if (onsemi_is_streaming(onsemi)) {
		dev_dbg(onsemi->dev, "device is streaming; can not setup links\n");
		return -EBUSY;
	}

	if (WARN_ON(test_bit(ONSEMI_FLAG_V4L_POWERED, &onsemi->flags)))
		return -EBUSY;

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (onsemi->active_bus && onsemi->active_bus != bus_info) {
			dev_dbg(onsemi->dev, "another link is already active\n");
			return -EBUSY;
		}

		rc = onsemi_calculate_pll(onsemi, bus_info, onsemi->v4l_parm->bpp,
					  onsemi->pll_cfg, onsemi->pll_freq);
		if (rc < 0) {
			dev_dbg(onsemi->dev, "bad PLL setup: %d\n", rc);
			goto out;
		}

		onsemi->active_bus = bus_info;
	} else {
		if (onsemi->active_bus == bus_info) {
			onsemi->active_bus = NULL;
			onsemi->pll_freq->link_freq = 0;
		}
	}

out:
	return rc;
}

static struct media_entity_operations const	onsemi_entity_ops = {
	.link_validate	= onsemi_subdev_link_validate,
	.link_setup	= onsemi_subdev_link_setup,
};

int onsemi_core_v4l_init(struct onsemi_core *onsemi,
			 struct onsemi_v4l_parm *parm,
			 struct onsemi_dev_cfg const *cfg)
{
	/* HACK: we need this below in v4l2_i2c_subdev_init() although rest of
	 * code uses regmap */
	struct i2c_client	*i2c = to_i2c_client(onsemi->dev);
	struct v4l2_subdev	*sd = &onsemi->subdev;
	int			rc;
	size_t			i;
	unsigned int		pad_cnt;

	rc = onsemi_power_get(onsemi);
	if (rc < 0)
		return rc;

	v4l2_i2c_subdev_init(sd, i2c, &onsemi_subdev_ops);

	rc = onsemi_fill_v4l_parm(onsemi, parm);
	if (rc < 0)
		goto out;

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &onsemi_subdev_internal_ops;

	pad_cnt = 0;
	for (i = 0; i < ARRAY_SIZE(onsemi->bus_info); ++i) {
		if (onsemi->bus_info[i].is_used) {
			onsemi->pad[i].flags = MEDIA_PAD_FL_SOURCE;
			pad_cnt = i + 1;
		}
	}

	rc = media_entity_pads_init(&sd->entity, pad_cnt, onsemi->pad);
	if (rc < 0)
		goto out;

	sd->entity.ops = &onsemi_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	rc = v4l2_ctrl_handler_init(&onsemi->ctrls,
				    ARRAY_SIZE(onsemi_core_ctrls) +
				    onsemi->ops->num_ctrls);
	if (rc < 0) {
		media_entity_cleanup(&sd->entity);
		dev_err(onsemi->dev,
			"failed to initialize v4l ctrl handle: %d\n", rc);
		goto out;
	}
	sd->ctrl_handler = &onsemi->ctrls;

	rc = onsemi_setup_ctrls(onsemi);
	if (rc < 0)
		goto out;

	rc = v4l2_async_register_subdev(sd);
	if (rc < 0) {
		media_entity_cleanup(&sd->entity);
		dev_err(onsemi->dev, "failed to register v4l2 subdev: %d\n", rc);
		goto out;
	}

	rc = 0;

out:
	onsemi_power_put(onsemi);

	return rc;
}
EXPORT_SYMBOL_GPL(onsemi_core_v4l_init);

int onsemi_timing_pair_read_of(struct onsemi_core const *onsemi,
			       char const *name,
			       struct onsemi_timing_map *map)
{
	struct device_node	*np;
	int			cnt;
	uint32_t		data[16 * 2];
	int			i;
	unsigned long		last_freq = 0;

	if (WARN_ON(map->tm != NULL))
		return -EINVAL;

	np = of_get_child_by_name(onsemi->dev->of_node, "timings");
	if (!np) {
		dev_warn(onsemi->dev, "missing 'timings' node\n");
		return -ENOENT;
	}

	cnt = of_property_count_elems_of_size(np, name, sizeof data[0]);
	if (cnt < 0) {
		dev_warn(onsemi->dev, "bad/missing 'timings/%s' property\n", name);
		return -ENOENT;
	}

	if (cnt == 0 || cnt % 2 || cnt > ARRAY_SIZE(data)) {
		dev_warn(onsemi->dev, "odd/bad 'timings/%s' property\n", name);
		return -ENOENT;
	}

	cnt /= 2;
	map->tm = devm_kcalloc(onsemi->dev, cnt, sizeof (map->tm)[0],
			       GFP_KERNEL);
	if (!map->tm)
		return -ENOMEM;

	of_property_read_u32_array(np, name, data, cnt * 2);
	of_node_put(np);

	for (i = 0; i < cnt; ++i) {
		unsigned long	freq = data[i * 2];
		unsigned long	val  = data[i * 2 + 1];

		dev_dbg(onsemi->dev, "%s#%d: %lu -> %lu\n", name, i, freq, val);

		if (freq <= last_freq) {
			dev_warn(onsemi->dev,
				 "bad 'timings/%s#%d' order (%lu <= %lu)\n",
				 name, i, freq, last_freq);
			devm_kfree(onsemi->dev, map->tm);
			map->tm = NULL;
			return -EINVAL;
		}

		map->tm[i] = (struct onsemi_timing) {
			.freq	= freq,
			.val	= val,
		};
	}

	map->num = cnt;

	return 0;
}
EXPORT_SYMBOL_GPL(onsemi_timing_pair_read_of);

int onsemi_timing_pair_find(struct onsemi_timing_map const *map,
			    unsigned long freq, unsigned int *val)
{
	size_t				i;
	struct onsemi_timing const	*tm = NULL;

	for (i = 0; i < map->num && !tm; ++i) {
		if (map->tm[i].freq >= freq)
			tm = &map->tm[i];
	}

	if (!tm)
		return -EINVAL;

	*val = tm->val;

	return 0;
}
EXPORT_SYMBOL_GPL(onsemi_timing_pair_find);

static int onsemi_calculate_vco(unsigned int *pll_div,
				unsigned int *pll_mul,
				struct onsemi_limits const *limits,
				unsigned long ext_hz,
				unsigned long vco_hz)
{
	struct pll_setup {
		unsigned int	div;
		unsigned int	mul;
		unsigned long	freq;
		unsigned long	delta;
	};
	struct pll_setup	best = {
		.div	= *pll_div,
		.mul	= *pll_mul,
	};

	unsigned int		div;
	unsigned int		mul;

	if (!in_range("VCO", &limits->pll_vco, vco_hz)) {
		pr_warn("%s: requested VCO %lu out of range [%lu..%lu]\n",
			__func__, vco_hz,
			limits->pll_vco.min, limits->pll_vco.max);
		return -EINVAL;
	}


	if (best.div == 0)
		best.freq = 0;
	else
		best.freq  = onsemi_clk_div_mul(ext_hz, best.div, best.div);

	if (best.freq < vco_hz)
		best.delta = vco_hz - best.freq;
	else
		best.delta = best.freq - vco_hz;

	for (div = limits->pre_pll_div.min;
	     div < limits->pre_pll_div.max && best.freq != vco_hz;
	     ++div) {
		unsigned long	freq = 0;

		for (mul = round_up(limits->pre_pll_mul.min, 2);
		     mul < limits->pre_pll_mul.max &&
			     freq < limits->pll_vco.max &&
			     best.freq != vco_hz;
		     mul += 2) {
			uint64_t	delta;

			freq = onsemi_clk_div_mul(ext_hz, div, mul);

			if (freq < limits->pll_vco.min ||
			    freq > limits->pll_vco.max)
				continue;

			if (freq < vco_hz)
				delta = vco_hz - freq;
			else
				delta = freq - vco_hz;

			if (delta < best.delta) {
				best.div = div;
				best.mul = mul;
				best.freq = freq;
				best.delta = delta;
			}
		}
	}

	pr_debug("%s: %lu / %u * %u = %llu (%lu)\n", __func__,
		 ext_hz, best.div, best.mul,
		 (unsigned long long)best.freq, vco_hz);

	if (!in_range("VCO", &limits->pll_vco, best.freq)) {
		pr_warn("%s: failed to find PLL settings for (%lu -> %lu)\n",
			__func__, ext_hz, vco_hz);
		return -EINVAL;
	}

	*pll_div = best.div;
	*pll_mul = best.mul;

	return 0;
}

static unsigned long _onsemi_estimate_vco(struct onsemi_core const *onsemi,
					  struct onsemi_businfo const *bus_info,
					  unsigned int bpp)
{
	struct onsemi_limits const	*limits = onsemi->limits;
	unsigned long			bus_freq;
	unsigned long			vco_freq;

	bus_freq = bus_info->max_freq;

	switch (bus_info->bus_type) {
	case V4L2_MBUS_PARALLEL: {
		unsigned long	max_bus = limits->pix_clk.max;

		if (bus_freq == 0 || bus_freq > max_bus)
			bus_freq = max_bus;

		vco_freq = bus_freq * 12;
		break;
	}

	case V4L2_MBUS_CSI2_DPHY: {
		unsigned long	max_vco = limits->pix_clk.max * bpp;
		unsigned int	bus_width = bus_info->bus_width;

		if (bus_freq == 0 || bus_freq >= max_vco / bus_width)
			vco_freq = max_vco;
		else
			vco_freq = bus_freq * bus_width;
		break;
	}

	default:
		WARN_ON(1);
		return 0;
	}

	while (vco_freq > limits->pll_vco.max)
		vco_freq /= 2;

	return vco_freq;
}

static void _onsemi_calculate_div(struct onsemi_core const *onsemi,
				  struct onsemi_businfo const *bus_info,
				  unsigned long freq_vco,
				  unsigned int bpp,
				  struct onsemi_pll_cfg *cfg)
{
	struct onsemi_limits const	*limits = onsemi->limits;
	uint64_t			max_f;

	switch (bus_info->bus_type) {
	case V4L2_MBUS_PARALLEL:
		cfg->vt_sys_div = 1;
		cfg->vt_pix_div = 12 / 2;

		max_f  = limits->pix_clk.max;
		max_f *= cfg->vt_pix_div;

		while (freq_vco / cfg->vt_sys_div > max_f)
			cfg->vt_sys_div *= 2;

		/* not used */
		cfg->op_sys_div = 0;
		cfg->op_pix_div = 0;
		break;

	case V4L2_MBUS_CSI2_DPHY:
		/* limit serial output clock */
		max_f  = limits->f_serial.max;
		if (bus_info->max_freq != 0)
			max_f = min_t(uint64_t, max_f, bus_info->max_freq);
		max_f *= bus_info->bus_width;

		cfg->op_sys_div = 1;
		while (freq_vco / cfg->op_sys_div > max_f)
			cfg->op_sys_div *= 2;

		dev_dbg(onsemi->dev, "  vco=%lu, max_op=%llu -> div=%u\n",
			freq_vco, max_f, cfg->op_sys_div);

		/* limit pixel clock */
		max_f  = limits->pix_clk.max;
		max_f *= bpp / 2;

		cfg->vt_sys_div = cfg->op_sys_div;
		while (freq_vco / cfg->vt_sys_div > max_f) {
			cfg->op_sys_div *= 2;
			cfg->vt_sys_div *= 2;
		}

		cfg->vt_pix_div = bpp / 2;
		cfg->op_pix_div = bpp;

		dev_dbg(onsemi->dev, "  vco=%lu, max_vt=%llu -> vt_div=%u*%u, op_div=%u*%u\n",
			freq_vco, max_f,
			cfg->vt_sys_div, cfg->vt_pix_div,
			cfg->op_sys_div, cfg->op_pix_div);

		break;

	default:
		BUG();
	}
}

static unsigned long _ul_mul_div(unsigned long a, unsigned long b, unsigned long c)
{
	uint64_t	res;

	res  = a;
	res *= b;
	res  = div64_ul(res, c);

	WARN_ON(res != (unsigned long)res);

	return res;
}

int onsemi_calculate_pll(struct onsemi_core const *onsemi,
			 struct onsemi_businfo const *bus_info,
			 unsigned int bpp,
			 struct onsemi_pll_cfg *cfg_out,
			 struct onsemi_pll_freq *freq_out)
{
	struct onsemi_limits const	*limits = onsemi->limits;
	int				rc = 0;
	struct onsemi_pll_freq		freq;
	bool				use_op_clk;
	struct onsemi_pll_cfg		cfg = {
		.pc_speed	= 1,
		.op_speed	= 1,
	};
	unsigned long			vco_freq;

	if (WARN_ON(!bus_info))
		return -EPIPE;

	if (WARN_ON(onsemi->ext_clk_freq == 0))
		return -EPIPE;

	if (WARN_ON(bpp == 0))
		return -EINVAL;

	dev_dbg(onsemi->dev, "%s(%p (%d, %lu, %u), %u)\n", __func__,
		bus_info,
		bus_info->bus_type, bus_info->max_freq, bus_info->bus_width,
		bpp);

	vco_freq = _onsemi_estimate_vco(onsemi, bus_info, bpp);

	rc = onsemi_calculate_vco(&cfg.pre_pll_div, &cfg.pre_pll_mul,
				  limits, onsemi->ext_clk_freq, vco_freq);
	if (rc < 0)
		return rc;

	freq.ext = onsemi->ext_clk_freq;
	freq.vco = _ul_mul_div(freq.ext, cfg.pre_pll_mul, cfg.pre_pll_div);

	_onsemi_calculate_div(onsemi, bus_info, freq.vco, bpp, &cfg);

	switch (bus_info->bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		use_op_clk     = true;
		/* TODO: where is this '/ 2' is coming from?  It has been
		 * added because csi2_dphy_init() in the imx6 csi driver
		 * expects this scaling. */
		freq.link_freq = freq.vco / cfg.op_sys_div / 2;

		if (bus_info->bus_width > 2)
			freq.link_freq *= bus_info->bus_width / 2;

		break;

	case V4L2_MBUS_PARALLEL:
		use_op_clk = false;
		freq.link_freq = freq.vco / cfg.vt_sys_div / cfg.vt_pix_div;
		break;

	default:
		BUG();
	}

	dbg_trace_v4l("bpp=%u, width=%u\n", bpp, bus_info->bus_width);

	if (!in_range("pre_div", &limits->pre_pll_div, cfg.pre_pll_div) ||
	    !in_range("pre_mul", &limits->pre_pll_mul, cfg.pre_pll_mul) ||
	    !in_range("vt_sys", &limits->pll_vt_sys_clk_div, cfg.vt_sys_div) ||
	    !in_range("vt_pix", &limits->pll_vt_pix_clk_div, cfg.vt_pix_div))
		return -EINVAL;

	if (use_op_clk &&
	    (!in_range("op_sys", &limits->pll_op_sys_clk_div, cfg.op_sys_div) ||
	     !in_range("op_pix", &limits->pll_op_pix_clk_div, cfg.op_pix_div)))
		return -EINVAL;

	if (use_op_clk &&
	    (cfg.op_speed == 0 || !is_pwr2(cfg.op_speed))) {
		pr_debug("invalid op_speed %d\n", cfg.op_speed);
		return -EINVAL;
	}

	freq.vt_sys = freq.vco / cfg.vt_sys_div;
	freq.vt_pix = freq.vt_sys / cfg.vt_pix_div;
	freq.clk_pixel = freq.vt_pix / cfg.pc_speed;

	if (use_op_clk) {
		freq.op_sys = freq.vco / cfg.op_sys_div;
		freq.op_pix = freq.op_sys / cfg.op_pix_div;
		freq.clk_op = freq.op_pix / cfg.op_speed;
	} else {
		/* used in debug messages only */
		freq.op_sys = 0;
		freq.op_pix = 0;
		freq.clk_op = 0;
	}

	dev_dbg(onsemi->dev,
		"? CLK: ext=%lu, vco=%lu, vt=[%lu, %lu, %lu], op=[%lu, %lu, %lu], link=%llu\n",
		freq.ext, freq.vco,
		freq.vt_sys, freq.vt_pix, freq.clk_pixel,
		freq.op_sys, freq.op_pix, freq.clk_op,
		(unsigned long long)freq.link_freq);

	if (!in_range("ext_clk",  &limits->ext_clk, freq.ext) ||
	    !in_range("vco",      &limits->pll_vco, freq.vco) ||
	    !in_range("pix_clk",  &limits->pix_clk, freq.clk_pixel))
		return -EINVAL;

	if (use_op_clk &&
	    !in_range("op_clk",   use_op_clk ? &limits->op_clk : NULL,  freq.clk_op))
		return -EINVAL;

	/* copy results here before sensor specific pll_validate() so that
	 * this method can access derived (--> container_of()) variants of
	 * pll_cfg resp. freq */
	if (freq_out)
		*freq_out = freq;

	if (cfg_out)
		*cfg_out = cfg;

	if (onsemi->ops->pll_validate) {
		rc = onsemi->ops->pll_validate(onsemi, cfg_out, freq_out);
		if (rc < 0) {
			dev_warn(onsemi->dev,
				 "extended PLL verification failed: %d\n", rc);
			return rc;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(onsemi_calculate_pll);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
