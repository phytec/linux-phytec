/*
 * Driver for MT9M001 CMOS Image Sensor from Aptina
 * Copyright (C) 2016, Christian Hemp <c.hemp@phytec.de>
 *
 * Based on soc_camera mt9m001 driver from Guennadi Liakhovetski.
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>

#include <media/media-entity.h>
#include <media/i2c/mt9m001.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-of.h>
#include <linux/v4l2-mediabus.h>
#include <linux/mutex.h>


#define MT9M001_PIXEL_ARRAY_HEIGHT		1048
#define MT9M001_PIXEL_ARRAY_WIDTH		1312

#define MT9M001_CHIP_VERSION			0x00
#define MT9M001_ROW_START			0x01
#define		MT9M001_ROW_START_MIN		12
#define		MT9M001_ROW_START_DEF		12
#define		MT9M001_ROW_START_MAX		1048
#define MT9M001_COLUMN_START			0x02
#define		MT9M001_COLUMN_START_MIN	20
#define		MT9M001_COLUMN_START_DEF	20
#define		MT9M001_COLUMN_START_MAX	1312
#define MT9M001_WINDOW_HEIGHT			0x03
#define		MT9M001_WINDOW_HEIGHT_MIN	2
#define		MT9M001_WINDOW_HEIGHT_DEF	1024
#define		MT9M001_WINDOW_HEIGHT_MAX	1024
#define MT9M001_WINDOW_WIDTH			0x04
#define		MT9M001_WINDOW_WIDTH_MIN	3
#define		MT9M001_WINDOW_WIDTH_DEF	1280
#define		MT9M001_WINDOW_WIDTH_MAX	1280
#define MT9M001_HORIZONTAL_BLANKING		0x05
#define		MT9M001_HORIZONTAL_BLANKING_MIN	19
#define		MT9M001_HORIZONTAL_BLANKING_DEF	19
#define		MT9M001_HORIZONTAL_BLANKING_MAX	1000
#define MT9M001_VERTICAL_BLANKING		0x06
#define		MT9M001_VERTICAL_BLANKING_DEF	25
#define MT9M001_OUTPUT_CONTROL			0x07
#define MT9M001_SHUTTER_WIDTH			0x09
#define		MT9M001_SHUTTER_WIDTH_DEF	1049
#define MT9M001_FRAME_RESTART			0x0b
#define MT9M001_SHUTTER_DELAY			0x0c
#define		MT9M001_SHUTTER_DELAY_DEF	0
#define MT9M001_RESET				0x0d
#define MT9M001_READ_OPTIONS1			0x1e
#define MT9M001_READ_OPTIONS2			0x20
#define MT9M001_GLOBAL_GAIN			0x35
#define MT9M001_CHIP_ENABLE			0xF1

enum mt9m001_model {
	MT9M001_MODEL_COLOR,
	MT9M001_MODEL_MONOCHROME,
};

/* MT9M001 has only one fixed colorspace per pixelcode */
struct mt9m001_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

/* Find a data format by a pixel code in an array */
static const struct mt9m001_datafmt *mt9m001_find_datafmt(
	u32 code, const struct mt9m001_datafmt *fmt,
	int n)
{
	int i;

	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;

	return NULL;
}

static const struct mt9m001_datafmt mt9m001_colour_fmts[] = {
	/*
	 * Order important: first natively supported,
	 * second supported with a GPIO extender
	 */
	{MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_COLORSPACE_SRGB},
	{MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB},
};

static const struct mt9m001_datafmt mt9m001_monochrome_fmts[] = {
	/* Order important - see above */
	{MEDIA_BUS_FMT_Y10_1X10, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_Y8_1X8, V4L2_COLORSPACE_JPEG},
};

struct mt9m001 {
	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_mbus_framefmt format;
	struct v4l2_rect crop;

	struct v4l2_ctrl_handler hdl;
	struct {
		/* exposure/auto-exposure cluster */
		struct v4l2_ctrl *autoexposure;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *link_freq;
		struct v4l2_ctrl *pixel_rate;
	};

	enum mt9m001_model model;
	struct mt9m001_platform_data *pdata;
	struct clk *clk;
	u32 sysclk;

	struct mutex power_lock; /* lock to protect power_count */
	int power_count;

	const struct mt9m001_datafmt *fmt;
	const struct mt9m001_datafmt *fmts;
	int num_fmts;
	unsigned int total_h;
};

static struct mt9m001 *to_mt9m001(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m001, subdev);
}

static int reg_read(struct i2c_client *client, const u8 reg)
{
	return i2c_smbus_read_word_swapped(client, reg);
}

static int reg_write(struct i2c_client *client, const u8 reg,
		     const u16 data)
{
	return i2c_smbus_write_word_swapped(client, reg, data);
}

static int reg_set(struct i2c_client *client, const u8 reg,
		   const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret | data);
}

static int reg_clear(struct i2c_client *client, const u8 reg,
		     const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret & ~data);
}

static int mt9m001_reset(struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);

	/*
	 * We don't know, whether platform provides reset, issue a soft reset
	 * too. This returns all registers to their default values.
	 */
	ret = reg_write(client, MT9M001_RESET, 1);
	if (!ret)
		ret = reg_write(client, MT9M001_RESET, 0);

	/* Disable chip, synchronous option update */
	if (!ret)
		ret = reg_write(client, MT9M001_OUTPUT_CONTROL, 0);

	return ret;
}

static int mt9m001_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m001 *mt9m001 = container_of(ctrl->handler,
					       struct mt9m001, hdl);
	s32 min, max;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		min = mt9m001->exposure->minimum;
		max = mt9m001->exposure->maximum;
		mt9m001->exposure->val =
			(524 + (mt9m001->total_h - 1) * (max - min)) / 1048 + min;
		break;
	}
	return 0;
}

static int mt9m001_power_on(struct mt9m001 *mt9m001)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9m001->subdev);
	int ret;

	ret = clk_set_rate(mt9m001->clk, mt9m001->sysclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(mt9m001->clk);
	if (ret < 0)
		return ret;

	ret = reg_write(client, MT9M001_CHIP_ENABLE, 1);

	return ret;
}

static void mt9m001_power_off(struct mt9m001 *mt9m001)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9m001->subdev);

	reg_write(client, MT9M001_CHIP_ENABLE, 0);

	clk_disable_unprepare(mt9m001->clk);
}

static int __mt9m001_set_power(struct mt9m001 *mt9m001, bool on)
{
	int ret;

	if (!on) {
		mt9m001_power_off(mt9m001);
		return 0;
	}

	ret = mt9m001_power_on(mt9m001);
	if (ret < 0)
		return ret;

	return v4l2_ctrl_handler_setup(&mt9m001->hdl);
}

static int mt9m001_enum_frame_size(struct v4l2_subdev *subdev,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);

	if (fse->index >= 3 || fse->code != mt9m001->format.code)
		return -EINVAL;

	fse->min_width = MT9M001_WINDOW_WIDTH_DEF / fse->index;
	fse->max_width = fse->min_width;
	fse->min_height = MT9M001_WINDOW_HEIGHT_DEF / fse->index;
	fse->max_height = fse->min_height;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

static int mt9m001_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9m001 *mt9m001 = to_mt9m001(sd);
	const struct v4l2_rect *crop = &mt9m001->crop;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = reg_write(client, MT9M001_HORIZONTAL_BLANKING,
			MT9M001_HORIZONTAL_BLANKING_DEF);
	if (!ret)
		ret = reg_write(client, MT9M001_VERTICAL_BLANKING,
				MT9M001_VERTICAL_BLANKING_DEF);

	if (!ret)
		ret = reg_write(client, MT9M001_COLUMN_START, crop->left);
	if (!ret)
		ret = reg_write(client, MT9M001_ROW_START, crop->top);
	if (!ret)
		ret = reg_write(client, MT9M001_WINDOW_WIDTH, crop->width - 1);
	if (!ret)
		ret = reg_write(client, MT9M001_WINDOW_HEIGHT,
				crop->height - 1);
	if (!ret &&
	    v4l2_ctrl_g_ctrl(mt9m001->autoexposure) == V4L2_EXPOSURE_AUTO)
		ret = reg_write(client, MT9M001_SHUTTER_WIDTH,
				mt9m001->total_h);

	/* Switch to master "normal" mode or stop sensor readout */
	if (!ret && reg_write(client,
			      MT9M001_OUTPUT_CONTROL, enable ? 2 : 0) < 0)
		return -EIO;

	return 0;
}

static int mt9m001_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_PARALLEL;
	cfg->flags = V4L2_MBUS_MASTER;

	return 0;
}

static int mt9m001_enum_mbus_code(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);

	if (code->pad || code->index  > mt9m001->num_fmts)
		return -EINVAL;

	code->code = mt9m001->fmts[code->index].code;

	return 0;
}

static struct v4l2_mbus_framefmt *
__mt9m001_get_pad_format(struct mt9m001 *mt9m001, struct v4l2_subdev_pad_config *cfg,
			 unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&mt9m001->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mt9m001->format;
	default:
		return NULL;
	}

}

static struct v4l2_rect *
__mt9m001_get_pad_crop(struct mt9m001 *mt9m001, struct v4l2_subdev_pad_config *cfg,
			 unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&mt9m001->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mt9m001->crop;
	default:
		return NULL;
}
}

static int mt9m001_get_format(struct v4l2_subdev *subdev,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_format *fmt)
{
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);

	fmt->format = *__mt9m001_get_pad_format(mt9m001, cfg, fmt->pad,
						fmt->which);
	return 0;
}


static int mt9m001_set_format(struct v4l2_subdev *subdev,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *format)
{
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);
	struct v4l2_mbus_framefmt *__format;
	const struct mt9m001_datafmt *fmt;
	struct v4l2_rect *__crop;
	unsigned int width;
	unsigned int height;
	unsigned int hratio;
	unsigned int vratio;

	__crop = __mt9m001_get_pad_crop(mt9m001, cfg, format->pad,
					format->which);

	/* Clamp the width and height to avoid dividing by zero. */
	width = clamp(ALIGN(format->format.width, 2),
		      max_t(unsigned int, __crop->width / 4,
			    MT9M001_WINDOW_WIDTH_MIN),
		      __crop->width);
	height = clamp(ALIGN(format->format.height, 2),
		       max_t(unsigned int, __crop->height / 4,
			     MT9M001_WINDOW_HEIGHT_MIN),
		       __crop->height);

	mt9m001->total_h = __crop->height + MT9M001_VERTICAL_BLANKING_DEF;

	hratio = DIV_ROUND_CLOSEST(__crop->width, width);
	vratio = DIV_ROUND_CLOSEST(__crop->height, height);

	__format = __mt9m001_get_pad_format(mt9m001, cfg, format->pad,
					    format->which);
	__format->width = __crop->width / hratio;
	__format->height = __crop->height / vratio;

	fmt = mt9m001_find_datafmt(format->format.code, mt9m001->fmts,
				   mt9m001->num_fmts);
	/* set colorspace */
	if (fmt) {
		__format->code = fmt->code;
		__format->colorspace = fmt->colorspace;
	}

	format->format = *__format;

	return 0;
}

static int mt9m001_set_selection(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	struct v4l2_rect rect;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	/* Clamp the crop rectangle boundaries and align them to a multiple of 2
	 * pixels.
	 */
	rect.left = clamp(ALIGN(sel->r.left, 2),
			  MT9M001_COLUMN_START_MIN,
			  MT9M001_COLUMN_START_MAX);
	rect.top = clamp(ALIGN(sel->r.top, 2),
			 MT9M001_ROW_START_MIN,
			 MT9M001_ROW_START_MAX);
	rect.width = clamp_t(unsigned int, ALIGN(sel->r.width, 2),
			     MT9M001_WINDOW_WIDTH_MIN + 1,
			     MT9M001_WINDOW_WIDTH_MAX + 1);
	rect.height = clamp_t(unsigned int, ALIGN(sel->r.height, 2),
			      MT9M001_WINDOW_HEIGHT_MIN + 1,
			      MT9M001_WINDOW_HEIGHT_MAX + 1);

	rect.width = min_t(unsigned int, rect.width,
			   MT9M001_PIXEL_ARRAY_WIDTH - rect.left);
	rect.height = min_t(unsigned int, rect.height,
			    MT9M001_PIXEL_ARRAY_HEIGHT - rect.top);

	__crop = __mt9m001_get_pad_crop(mt9m001, cfg, sel->pad, sel->which);

	if (rect.width != __crop->width || rect.height != __crop->height) {
		/* Reset the output image size if the crop rectangle size has
		 * been modified.
		 */
		__format = __mt9m001_get_pad_format(mt9m001, cfg, sel->pad,
						    sel->which);
		__format->width = rect.width;
		__format->height = rect.height;
	}

	*__crop = rect;
	sel->r = rect;

	return 0;
}

static int mt9m001_get_selection(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_selection *sel)
{
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__mt9m001_get_pad_crop(mt9m001, cfg, sel->pad, sel->which);
		return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */

static int mt9m001_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m001 *mt9m001 = container_of(ctrl->handler,
					       struct mt9m001, hdl);
	struct v4l2_subdev *sd = &mt9m001->subdev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_ctrl *exp = mt9m001->exposure;
	u32 freq;
	int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			data = reg_set(client, MT9M001_READ_OPTIONS2, 0x8000);
		else
			data = reg_clear(client, MT9M001_READ_OPTIONS2, 0x8000);
		if (data < 0)
			return -EIO;
		return 0;

	case V4L2_CID_GAIN:
		/* See Datasheet Table 7, Gain settings. */
		if (ctrl->val <= ctrl->default_value) {
			/* Pack it into 0..1 step 0.125, register values 0..8 */
			unsigned long range = ctrl->default_value -
					      ctrl->minimum;
			data = ((ctrl->val - (s32)ctrl->minimum) * 8 + range /
				 2) / range;

			dev_dbg(&client->dev, "Setting gain %d\n", data);
			data = reg_write(client, MT9M001_GLOBAL_GAIN, data);
			if (data < 0)
				return -EIO;
		} else {
			/*
			 * Pack it into 1.125..15 variable step, register
			 * values 9..67
			 * We assume
			 * qctrl->maximum - qctrl->default_value - 1 > 0
			 */
			unsigned long range = ctrl->maximum -
					      ctrl->default_value - 1;
			unsigned long gain = ((ctrl->val -
					      (s32)ctrl->default_value - 1) *
					      111 + range / 2) / range + 9;

			if (gain <= 32)
				data = gain;
			else if (gain <= 64)
				data = ((gain - 32) * 16 + 16) / 32 + 80;
			else
				data = ((gain - 64) * 7 + 28) / 56 + 96;

			dev_dbg(&client->dev, "Setting gain from %d to %d\n",
				 reg_read(client, MT9M001_GLOBAL_GAIN), data);
			data = reg_write(client, MT9M001_GLOBAL_GAIN, data);
			if (data < 0)
				return -EIO;
		}
		return 0;

	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->val == V4L2_EXPOSURE_MANUAL) {
			unsigned long range = exp->maximum - exp->minimum;
			unsigned long shutter = ((exp->val - (s32)exp->minimum)
						  * 1048 + range / 2) /
						  range + 1;

			dev_dbg(&client->dev,
				"Setting shutter width from %d to %lu\n",
				reg_read(client, MT9M001_SHUTTER_WIDTH),
					 shutter);
			if (reg_write(client, MT9M001_SHUTTER_WIDTH,
				      shutter) < 0)
				return -EIO;
		} else {
			mt9m001->total_h = mt9m001->crop.height +
					   MT9M001_VERTICAL_BLANKING_DEF;
			if (reg_write(client, MT9M001_SHUTTER_WIDTH,
				      mt9m001->total_h) < 0)
				return -EIO;
		}
		return 0;
	case V4L2_CID_LINK_FREQ:
		if (mt9m001->link_freq == NULL)
			break;

		freq = mt9m001->pdata->link_freqs[mt9m001->link_freq->val];
		mt9m001->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9m001_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff)
		return -EINVAL;

	reg->size = 2;
	reg->val = reg_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int mt9m001_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff)
		return -EINVAL;

	if (reg_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}
#endif

static int mt9m001_set_power(struct v4l2_subdev *subdev, int on)
{
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);
	int ret = 0;

	mutex_lock(&mt9m001->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (mt9m001->power_count == !on) {
		ret = __mt9m001_set_power(mt9m001, !!on);
		if (ret < 0)
			goto done;
	}

	/* Update the power count. */
	mt9m001->power_count += on ? 1 : -1;
	WARN_ON(mt9m001->power_count < 0);
done:
	mutex_unlock(&mt9m001->power_lock);
	return ret;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static int mt9m001_registered(struct v4l2_subdev *subdev)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);
	s32 data;
	int ret;

	ret = mt9m001_power_on(mt9m001);
	if (ret < 0) {
		dev_err(&client->dev, "MT9M001 power up failed\n");
		return ret;
	}

	/* Read out the chip version register */
	data = reg_read(client, MT9M001_CHIP_VERSION);

	/* must be 0x8411 or 0x8421 for colour sensor and 8431 for bw */
	switch (data) {
	case 0x8411:
	case 0x8421:
		mt9m001->fmts = mt9m001_colour_fmts;
		mt9m001->num_fmts = ARRAY_SIZE(mt9m001_colour_fmts);
		mt9m001->model = MT9M001_MODEL_COLOR;
		break;
	case 0x8431:
		mt9m001->fmts = mt9m001_monochrome_fmts;
		mt9m001->num_fmts = ARRAY_SIZE(mt9m001_monochrome_fmts);
		mt9m001->model = MT9M001_MODEL_MONOCHROME;
		break;
	default:
		dev_err(&client->dev,
			"No MT9M001 chip detected, register read %x\n", data);
		return -ENODEV;
	}

	dev_info(&client->dev, "Detected a MT9M001 chip ID %x (%s)\n", data,
		 data == 0x8431 ? "C12STM" : "C12ST");

	ret = mt9m001_reset(client);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to initialise the camera\n");
		return ret;
	}
	/*
	 * mt9m001_reset() has reset the chip, returning registers to
	 * defaults
	 */
	ret = v4l2_ctrl_handler_setup(&mt9m001->hdl);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to setup ctrl handler\n");
		return ret;
	}

	mt9m001_power_off(mt9m001);
	return ret;
}

static int mt9m001_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	crop = v4l2_subdev_get_try_crop(subdev, fh->pad, 0);
	crop->left = MT9M001_COLUMN_START_DEF;
	crop->top = MT9M001_ROW_START_DEF;
	crop->width = MT9M001_WINDOW_WIDTH_DEF;
	crop->height = MT9M001_WINDOW_HEIGHT_DEF;

	format = v4l2_subdev_get_try_format(subdev, fh->pad, 0);

	if (mt9m001->model == MT9M001_MODEL_COLOR) {
		format->code = MEDIA_BUS_FMT_SGRBG10_1X10;
		format->colorspace = V4L2_COLORSPACE_SRGB;
	} else {
		format->code = MEDIA_BUS_FMT_Y10_1X10;
		format->colorspace = V4L2_COLORSPACE_JPEG;
	}

	format->width = MT9M001_WINDOW_WIDTH_DEF;
	format->height = MT9M001_WINDOW_HEIGHT_DEF;
	format->field = V4L2_FIELD_NONE;

	return mt9m001_set_power(subdev, 1);
}

static int mt9m001_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	return mt9m001_set_power(subdev, 0);
}

static const struct v4l2_ctrl_ops mt9m001_ctrl_ops = {
	.g_volatile_ctrl = mt9m001_g_volatile_ctrl,
	.s_ctrl = mt9m001_s_ctrl,
};

static struct v4l2_subdev_core_ops mt9m001_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= mt9m001_g_register,
	.s_register	= mt9m001_s_register,
#endif
	.s_power	= mt9m001_set_power,
};

static struct v4l2_subdev_video_ops mt9m001_subdev_video_ops = {
	.g_mbus_config	= mt9m001_g_mbus_config,
	.s_stream	= mt9m001_s_stream,
};

static struct v4l2_subdev_pad_ops mt9m001_subdev_pad_ops = {
	.enum_mbus_code = mt9m001_enum_mbus_code,
	.enum_frame_size = mt9m001_enum_frame_size,
	.get_fmt = mt9m001_get_format,
	.set_fmt = mt9m001_set_format,
	.get_selection = mt9m001_get_selection,
	.set_selection = mt9m001_set_selection,
};

static struct v4l2_subdev_ops mt9m001_subdev_ops = {
	.core	= &mt9m001_subdev_core_ops,
	.video	= &mt9m001_subdev_video_ops,
	.pad	= &mt9m001_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops mt9m001_subdev_internal_ops = {
	.registered = mt9m001_registered,
	.open = mt9m001_open,
	.close = mt9m001_close,
};

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */

static struct mt9m001_platform_data *
mt9m001_get_pdata(struct i2c_client *client)
{
	struct mt9m001_platform_data *pdata;
	struct v4l2_of_endpoint endpoint;
	struct device_node *np;
	struct property *prop;

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return client->dev.platform_data;

	np = of_graph_get_next_endpoint(client->dev.of_node, NULL);
	if (!np)
		return NULL;

	if (v4l2_of_parse_endpoint(np, &endpoint) < 0)
		goto done;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto done;

	prop = of_find_property(np, "link-frequencies", NULL);
	if (prop) {
		u64 *link_freqs;
		size_t size = prop->length / sizeof(*link_freqs);

		link_freqs = devm_kcalloc(&client->dev, size,
					  sizeof(*link_freqs), GFP_KERNEL);
		if (!link_freqs)
			goto done;

		if (of_property_read_u64_array(np, "link-frequencies",
					       link_freqs, size) < 0)
			goto done;

		pdata->link_freqs = link_freqs;
		pdata->link_def_freq = link_freqs[0];
	}


done:
	of_node_put(np);
	return pdata;
}

static int mt9m001_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct mt9m001_platform_data *pdata = mt9m001_get_pdata(client);
	struct mt9m001 *mt9m001;
	unsigned int i;
	int ret;

	if (pdata == NULL) {
		dev_err(&client->dev, "No platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&client->adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	mt9m001 = devm_kzalloc(&client->dev, sizeof(struct mt9m001),
			       GFP_KERNEL);
	if (!mt9m001)
		return -ENOMEM;

	mt9m001->clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(mt9m001->clk)) {
		dev_err(&client->dev, "Unable to get clock\n");
		return PTR_ERR(&mt9m001->clk);
	}

	mutex_init(&mt9m001->power_lock);
	mt9m001->pdata = pdata;

	v4l2_i2c_subdev_init(&mt9m001->subdev, client, &mt9m001_subdev_ops);
	mt9m001->subdev.internal_ops = &mt9m001_subdev_internal_ops;
	v4l2_ctrl_handler_init(&mt9m001->hdl, 10);
	v4l2_ctrl_new_std(&mt9m001->hdl, &mt9m001_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&mt9m001->hdl, &mt9m001_ctrl_ops,
			V4L2_CID_GAIN, 0, 127, 1, 64);
	mt9m001->exposure = v4l2_ctrl_new_std(&mt9m001->hdl, &mt9m001_ctrl_ops,
			V4L2_CID_EXPOSURE, 1, 255, 1, 255);
	/*
	 * Simulated autoexposure. If enabled, we calculate shutter width
	 * ourselves in the driver based on vertical blanking and frame width
	 */
	mt9m001->autoexposure = v4l2_ctrl_new_std_menu(&mt9m001->hdl,
			&mt9m001_ctrl_ops, V4L2_CID_EXPOSURE_AUTO, 1, 0,
			V4L2_EXPOSURE_AUTO);

	if (pdata && pdata->link_freqs) {
		unsigned int def = 0;

		for (i = 0; pdata->link_freqs[i]; ++i) {
			if (pdata->link_freqs[i] == pdata->link_def_freq)
				def = i;
		}

		mt9m001->link_freq =
			v4l2_ctrl_new_int_menu(&mt9m001->hdl,
					       &mt9m001_ctrl_ops,
					       V4L2_CID_LINK_FREQ, i - 1, def,
					       pdata->link_freqs);
		v4l2_ctrl_cluster(1, &mt9m001->link_freq);

	}

	mt9m001->subdev.ctrl_handler = &mt9m001->hdl;
	if (mt9m001->hdl.error) {
		ret = mt9m001->hdl.error;
		goto done;
	}

	v4l2_ctrl_auto_cluster(2, &mt9m001->autoexposure,
					V4L2_EXPOSURE_MANUAL, true);

	mt9m001->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	mt9m001->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&mt9m001->subdev.entity, 1, &mt9m001->pad, 0);
	if (ret < 0)
		goto done;

	/* Second stage probe - when a capture adapter is there */
	mt9m001->crop.left	= MT9M001_COLUMN_START_DEF;
	mt9m001->crop.top	= MT9M001_ROW_START_DEF;
	mt9m001->crop.width	= MT9M001_WINDOW_WIDTH_DEF;
	mt9m001->crop.height	= MT9M001_WINDOW_HEIGHT_DEF;

	mt9m001->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	mt9m001->format.width = MT9M001_WINDOW_WIDTH_DEF;
	mt9m001->format.height = MT9M001_WINDOW_HEIGHT_DEF;
	mt9m001->format.field = V4L2_FIELD_NONE;
	mt9m001->format.colorspace = V4L2_COLORSPACE_SRGB;
	mt9m001->sysclk = 26600000;

	mt9m001->subdev.dev = &client->dev;
	ret = v4l2_async_register_subdev(&mt9m001->subdev);

done:
	if (ret < 0) {
		v4l2_ctrl_handler_free(&mt9m001->hdl);
		media_entity_cleanup(&mt9m001->subdev.entity);
		mutex_destroy(&mt9m001->power_lock);
	}

	return ret;
}

static int mt9m001_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct mt9m001 *mt9m001 = to_mt9m001(subdev);

	v4l2_ctrl_handler_free(&mt9m001->hdl);
	v4l2_async_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	mutex_destroy(&mt9m001->power_lock);

	return 0;
}

static const struct of_device_id mt9m001_of_match[] = {
	{ .compatible = "aptina,mt9m001" },
	{ .compatible = "aptina,mt9m001m" },
	{},
};
MODULE_DEVICE_TABLE(of, mt9m001_of_match);

static const struct i2c_device_id mt9m001_id[] = {
	{ "mt9m001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9m001_id);

static struct i2c_driver mt9m001_i2c_driver = {
	.driver = {
		.name = "mt9m001",
		.of_match_table = of_match_ptr(mt9m001_of_match),
	},
	.probe		= mt9m001_probe,
	.remove		= mt9m001_remove,
	.id_table	= mt9m001_id,
};

module_i2c_driver(mt9m001_i2c_driver);

MODULE_DESCRIPTION("Aptina MT9M001 Camera driver");
MODULE_AUTHOR("Christian Hemp <c.hemp@phytec.de");
MODULE_LICENSE("GPL");
