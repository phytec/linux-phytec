/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Generic part of ONSemi ARxxx sensor drivers.
 *
 * Copyright (C) 2018 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 */

#ifndef H_LINUX_MEDIA_I2C_ONSEMI_CORE_H
#define H_LINUX_MEDIA_I2C_ONSEMI_CORE_H

#include <linux/types.h>
#include <linux/mutex.h>

#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define ONSEMI_NO_SLEW_RATE	(~0u)

enum {
	V4L2_CID_USER_BASE_ONSEMI	= V4L2_CID_USER_BASE + 0x2500,
	V4L2_CID_X_EXPOSURE_FINE,
	V4L2_CID_X_AUTO_EXPOSURE_MIN,
	V4L2_CID_X_AUTO_EXPOSURE_MAX,
	V4L2_CID_X_AUTO_EXPOSURE_TGT,
	V4L2_CID_X_AUTO_EXPOSURE_CUR,
	V4L2_CID_X_EMBEDDED_DATA,

	V4L2_CID_X_AUTOGAIN_ANALOGUE,
	V4L2_CID_X_AUTOGAIN_DIGITAL,

	V4L2_CID_X_AUTOGAIN_ANALOGUE_MIN,

	V4L2_CID_X_BINNING_ROW,
	V4L2_CID_X_BINNING_COL,
	V4L2_CID_X_SUMMING_COL,
	V4L2_CID_X_SUMMING_ROW,

	V4L2_CID_X_SKIP_COL,
	V4L2_CID_X_SKIP_ROW,

	V4L2_CID_X_TRIGGER_MDOE,

	V4L2_CID_X_DIGITAL_GAIN_RED,
	V4L2_CID_X_DIGITAL_GAIN_GREENR,
	V4L2_CID_X_DIGITAL_GAIN_BLUE,
	V4L2_CID_X_DIGITAL_GAIN_GREENB,

	V4L2_CID_X_COMPAND,
	V4L2_CID_X_BLACK_LEVEL_AUTO,

	V4L2_CID_X_FLASH_DELAY,
	V4L2_CID_X_HBLANK_EFFECTIVE,
};

enum onsemi_pwr_state {
	ONSEMI_PWR_OFF,
	ONSEMI_PWR_ON,
	ONSEMI_PWR_RST,
};

struct onsemi_range {
	unsigned long			min;
	unsigned long			max;
};

struct onsemi_limits {
	struct onsemi_range		x;
	struct onsemi_range		y;

	unsigned long			hblank_min;
	unsigned long			vblank_min;

	struct onsemi_range		hlen;
	struct onsemi_range		vlen;

	struct onsemi_range		pre_pll_div;
	struct onsemi_range		pre_pll_mul;

	struct onsemi_range		pll_vt_sys_clk_div;
	struct onsemi_range		pll_vt_pix_clk_div;

	struct onsemi_range		pll_op_sys_clk_div;
	struct onsemi_range		pll_op_pix_clk_div;

	struct onsemi_range		ext_clk;
	struct onsemi_range		pll_vco;
	struct onsemi_range		pix_clk;
	struct onsemi_range		op_clk;

	struct onsemi_range		f_serial;
};

struct onsemi_pll_cfg {
	unsigned int		pre_pll_div;
	unsigned int		pre_pll_mul;

	unsigned int		vt_sys_div;
	unsigned int		vt_pix_div;

	unsigned int		op_sys_div;
	unsigned int		op_pix_div;

	unsigned int		op_speed;
	unsigned int		pc_speed;
};

struct onsemi_pll_freq {
	unsigned long		ext;
	unsigned long		vco;
	unsigned long		clk_pixel;
	unsigned long		clk_op;
	unsigned long		vt_pix;
	unsigned long		vt_sys;
	unsigned long		op_pix;
	unsigned long		op_sys;

	s64			link_freq;
};

struct onsemi_v4l_parm {
	unsigned int		code;
	unsigned int		bpp;
	struct v4l2_frmsize_discrete	frame;

	struct v4l2_rect	crop;

	unsigned int		x_scale;
	unsigned int		y_scale;

	unsigned int		hblank;
	unsigned int		vblank;

	unsigned int		x_skip;
	unsigned int		y_skip;

	unsigned int		data_pedestal;
};

struct onsemi_core;

struct onsemi_core_ops {
	bool		has_smia_cfg:1;
	bool		hclk_mul_2:1;

	unsigned long	supported_bpp;

	bool		vaa_first:1;

	struct v4l2_ctrl_config const	*ctrls;
	size_t				num_ctrls;

	int		(*pll_calculate)(struct onsemi_core *onsemi);

	int		(*pll_validate)(struct onsemi_core const *onsemi,
					struct onsemi_pll_cfg const *cfg,
					struct onsemi_pll_freq *freq);
	int		(*pll_set)(struct onsemi_core *onsemi,
				   struct onsemi_pll_cfg const *cfg,
				   struct onsemi_pll_freq const *freq);

	int		(*fill_limits)(struct onsemi_core *onsemi,
				       struct onsemi_limits *limits,
				       int (*dflt_fn)(struct onsemi_core *onsemi,
						      struct onsemi_limits *limits));

	/* mandatory */
	int		(*enum_mbus_code)(struct onsemi_core *onsemi,
					  struct v4l2_subdev_state *sd_state,
					  struct v4l2_subdev_mbus_code_enum *code);

	int		(*stream_on)(struct onsemi_core *onsemi);
	int		(*stream_off)(struct onsemi_core *onsemi);

	int		(*prepare)(struct onsemi_core *onsemi);

	/* optionally */
	int		(*set_fmt)(struct onsemi_core *onsemi,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *format,
				   unsigned int bpp);

	/* returns 'false' when ctrl should not be added */
	bool		(*ctrl_setup_pre)(struct onsemi_core *onsemi,
					  struct v4l2_ctrl_config *ctrl);

	void		(*ctrl_setup_post)(struct onsemi_core *onsemi,
					   struct v4l2_ctrl *ctrl);

	/* reutrns 'true' when ctrl has been set completely by specialized
	 * driver */
	bool		(*ctrl_set)(struct onsemi_core *onsemi,
				    struct v4l2_ctrl *ctrl, int *rc);
	bool		(*ctrl_get)(struct onsemi_core *onsemi,
				    struct v4l2_ctrl *ctrl, int *rc);
};

enum {
	ONSEMI_FLAG_V4L_POWERED,
	ONSEMI_FLAG_V4L_STREAMING,
	ONSEMI_FLAG_SYNCCLK_ON,
};

enum onsemi_color_mode {
	ONSEMI_COLOR_MONOCHROME,
	ONSEMI_COLOR_BAYER,
	ONSEMI_COLOR_RGB,
};

struct onsemi_timing {
	unsigned long		freq;
	unsigned int		val;
};

struct onsemi_timing_map {
	struct onsemi_timing	*tm;
	size_t			num;
};

struct onsemi_businfo {
	enum v4l2_mbus_type		bus_type;
	unsigned int			bus_width;
	signed int			dout_sft;
	unsigned long			max_freq;
	bool				is_used;
	unsigned int			slew_rate_dat;
	unsigned int			slew_rate_clk;
};

struct onsemi_core_version {
	unsigned int	chip;
	unsigned int	rev;
	unsigned int	manu;
	unsigned int	smia;
	unsigned int	cust;
};;

struct onsemi_core_thermal {
	struct thermal_zone_device	*tdev;
	signed long			t0;
	signed long			slope;
};

struct onsemi_core {
	struct v4l2_subdev		subdev;
	struct media_pad		pad[2];
	struct v4l2_ctrl_handler	ctrls;
	struct onsemi_businfo		bus_info[2];
	unsigned int			num_bus;
	struct onsemi_core_version	version;

	struct device			*dev;
	struct onsemi_limits const	*limits;
	struct regmap			*regmap;

	struct regulator		*reg_vddio;
	struct regulator		*reg_vdd;
	struct regulator		*reg_vaa;
	struct regulator		*reg_vaapix;

	struct gpio_desc		*rst_gpio;

	struct clk			*extclk;
	struct clk			*synchclk;

	struct onsemi_core_ops const	*ops;

	struct mutex			lock;
	enum onsemi_pwr_state		pwr_state;
	atomic_t			num_pwr_users;
	atomic_t			num_v4lpwr_users;

	struct dentry			*debugfs_top;
	struct dentry			*debugfs_limits;
	struct dentry			*debugfs_freq;

	struct onsemi_limits		*dyn_limits;


	unsigned long			flags;
	enum onsemi_color_mode		color_mode;

	struct onsemi_v4l_parm		*v4l_parm;
	struct onsemi_pll_cfg		*pll_cfg;
	struct onsemi_pll_freq		*pll_freq;
	unsigned int			ext_clk_freq;

#if IS_ENABLED(CONFIG_THERMAL)
	struct onsemi_core_thermal	thermal;
#endif

	struct onsemi_businfo const	*active_bus;
};

#define sd_to_onsemi(_sd) \
	container_of(_sd, struct onsemi_core, subdev)


struct onsemi_dev_cfg {
	uint16_t				chip_version;
	int					is_color;

	/**
	 *  Static sensor limits; can be NULL.
	 *
	 *  When NULL, limits will be read from sensor
	 */
	struct onsemi_limits const		*limits;
	/**
	 *  Buffer for dynamic sensor limits; can be NULL
	 *
	 *  When NULL, a buffer will be allocated by devm_kzalloc()
	 */
	struct onsemi_limits			*limits_buf;
	struct onsemi_core_ops const		*ops;
	struct regmap_config const		*regmap_config;
};

#define dev_probe_err(_dev, _rc, _fmt, ...) do {		\
		if ((_rc) == -EPROBE_DEFER)			\
			dev_dbg(_dev, _fmt, __VA_ARGS__);	\
		else						\
			dev_err(_dev, _fmt, __VA_ARGS__);	\
	} while (0)

int onsemi_power_get(struct onsemi_core *onsemi);
void onsemi_power_put(struct onsemi_core *onsemi);

struct i2c_client;

struct v4l2_ctrl_ops;
extern struct v4l2_ctrl_ops const	onsemi_core_ctrl_ops;

int onsemi_core_init(struct i2c_client *i2c,
		     struct onsemi_core *,
		     struct onsemi_dev_cfg const *cfg);

void onsemi_core_release(struct onsemi_core *);

int onsemi_core_hw_init(struct onsemi_core *,
			struct onsemi_dev_cfg const *cfg);

int onsemi_core_v4l_init(struct onsemi_core *,
			 struct onsemi_v4l_parm *parm,
			 struct onsemi_dev_cfg const *cfg);

int onsemi_timing_pair_read_of(struct onsemi_core const *,
			       char const *name,
			       struct onsemi_timing_map *map);

int onsemi_timing_pair_find(struct onsemi_timing_map const *map,
			    unsigned long freq, unsigned int *val);

int onsemi_calculate_pll(struct onsemi_core const *onsemi,
			 struct onsemi_businfo const *bus_info,
			 unsigned int bpp,
			 struct onsemi_pll_cfg *cfg_out,
			 struct onsemi_pll_freq *freq_out);

inline static struct onsemi_businfo const *
onsemi_get_businfo(struct onsemi_core const *onsemi, unsigned int pad)
{
	if (pad >= ARRAY_SIZE(onsemi->bus_info))
		return NULL;

	if (!onsemi->bus_info[pad].is_used)
		return NULL;

	return &onsemi->bus_info[pad];
}

inline static bool onsemi_is_streaming(struct onsemi_core const *onsemi)
{
	return test_bit(ONSEMI_FLAG_V4L_STREAMING, &onsemi->flags);
};

inline static bool
onsemi_has_mipi(struct onsemi_core const *onsemi)
{
	size_t		i;

	for (i = 0; i < ARRAY_SIZE(onsemi->bus_info); ++i) {
		if (onsemi->bus_info[i].is_used &&
		    onsemi->bus_info[i].bus_type == V4L2_MBUS_CSI2_DPHY)
			return true;
	}

	return false;
}

#define ONSEMI_COND(_rc, _op) do {		\
		if (*(_rc) < 0) {		\
			/* noop */		\
		} else {			\
			*(_rc) = _op;		\
		}				\
	} while (0)

#define onsemi_update_bits(_rc, _onsemi, _reg, _mask, _val) \
	ONSEMI_COND(_rc, regmap_update_bits((_onsemi)->regmap, _reg, _mask, _val))

#define onsemi_write(_rc, _onsemi, _reg, _val) \
	ONSEMI_COND(_rc, regmap_write((_onsemi)->regmap, _reg, _val))

#define onsemi_read(_rc, _onsemi, _reg)					\
	({								\
		unsigned int _tmp_val = 0;				\
		ONSEMI_COND(_rc, regmap_read((_onsemi)->regmap, _reg, &_tmp_val)); \
		_tmp_val;						\
	})


#ifdef CONFIG_VIDEO_ONSEMI_ENABLE_DEBUG

#  define dbg_trace_pwr(_fmt, ...)		\
	trace_printk(_fmt, ##__VA_ARGS__)
#  define dbg_trace_v4l(_fmt, ...) \
	trace_printk(_fmt, ##__VA_ARGS__)

#else

#  define dbg_trace_pwr(_fmt, ...)			\
	do {						\
		if (0)					\
			printk(_fmt, ##__VA_ARGS__);	\
	} while (0)
#  define dbg_trace_v4l(_fmt, ...)			\
	do {						\
		if (0)					\
			printk(_fmt, ##__VA_ARGS__);	\
	} while (0)

#endif	/* CONFIG_VIDEO_ONSEMI_ENABLE_DEBUG */

#endif	/* H_LINUX_MEDIA_I2C_ONSEMI_CORE_H */
