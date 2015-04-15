/*	--*- c -*--
 * Copyright (C) 2014 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
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

#define DEBUG 1

#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-of.h>

enum cshift_type {
	CSHIFT_BITS2,
	CSHIFT_BITS4
};

enum {
	CSHIFT_PAD_SINK,
	CSHIFT_PAD_SOURCE
};

enum cshift_role {
	CSHIFT_ROLE_SINK,
	CSHIFT_ROLE_SOURCE,
};

enum {
	CSHIFT_STATE_PENDING,
	CSHIFT_STATE_REGISTER,
	CSHIFT_STATE_REGISTERED,
	CSHIFT_STATE_UNREGISTER,
};

struct cshift_device {
	struct device			*dev;
	struct v4l2_device		*v4l2;
	struct v4l2_subdev		subdev;
	struct media_pad		pads[2];

	enum cshift_type		type;
	struct v4l2_dbg_match		dbg_match;
	unsigned long			dbg_reg_last;
	unsigned int			dflt_mbus;
	struct v4l2_mbus_config		dflt_mbus_cfg;
	unsigned int			reg_sz;

	struct v4l2_subdev		*sensor;
	struct mutex			sensor_lock;

	struct work_struct		register_work;
	unsigned long			flags;

	struct {
		struct device_node		*sensor_np;
		struct v4l2_async_subdev	devs[1];
		struct v4l2_async_subdev	*dev_ptr[1];
		struct v4l2_async_notifier	n;
	}			notify;
};

#define sd_to_cshift(_sd) \
	container_of((_sd), struct cshift_device, subdev)

static int debug;
module_param(debug, int, 0644);

static bool cshift_fixup_code_in(struct cshift_device const *cshift,
				 enum cshift_role role,
				 uint32_t *code)
{
	trace_printk("role=%d, code=%08x\n", role, *code);

	switch (role) {
	case CSHIFT_ROLE_SINK:
		if (*code == 0) {
			*code = cshift->dflt_mbus;
			goto out;
		}

		switch (cshift->type) {
		case CSHIFT_BITS2:
			switch (*code) {
			case MEDIA_BUS_FMT_SGRBG10_1X10:
				*code = MEDIA_BUS_FMT_SGRBG8_1X8;
				break;
			case MEDIA_BUS_FMT_Y10_1X10:
				*code = MEDIA_BUS_FMT_Y8_1X8;
				break;
			default:
				dev_dbg(cshift->dev, "%s: invalid code %08x\n",
					__func__, *code);
				return false;
			}
			break;
		case CSHIFT_BITS4:
			switch (*code) {
			case MEDIA_BUS_FMT_SGRBG12_1X12:
				*code = MEDIA_BUS_FMT_SGRBG8_1X8;
				break;
			case MEDIA_BUS_FMT_Y12_1X12:
				*code = MEDIA_BUS_FMT_Y8_1X8;
				break;
			default:
				dev_dbg(cshift->dev, "%s: invalid code %08x\n",
					__func__, *code);
				return false;
			}
			break;

		default:
			dev_dbg(cshift->dev, "%s: mode %u not implemented\n",
				__func__, cshift->type);
			return false;
		}
		break;

	case CSHIFT_ROLE_SOURCE:
		break;

	default:
		WARN_ON(1);
		return false;
	}

out:
	return true;
}

static bool cshift_fixup_code_out(struct cshift_device const *cshift,
				  enum cshift_role role,
				  uint32_t *code)
{
	trace_printk("role=%d, code=%08x\n", role, *code);

	switch (role) {
	case CSHIFT_ROLE_SINK:
		break;

	case CSHIFT_ROLE_SOURCE:
		switch (cshift->type) {
		case CSHIFT_BITS2:
			switch (*code) {
			case MEDIA_BUS_FMT_SGRBG10_1X10:
				*code = MEDIA_BUS_FMT_SGRBG8_1X8;
				break;
			case MEDIA_BUS_FMT_Y10_1X10:
				*code = MEDIA_BUS_FMT_Y8_1X8;
				break;
			default:
				dev_dbg(cshift->dev, "%s: invalid code %08x\n",
					 __func__, *code);
				return false;
			}
			break;
		case CSHIFT_BITS4:
			switch (*code) {
			case MEDIA_BUS_FMT_Y12_1X12:
				*code = MEDIA_BUS_FMT_Y8_1X8;
				break;
			case MEDIA_BUS_FMT_SGRBG12_1X12:
				*code = MEDIA_BUS_FMT_SGRBG8_1X8;
				break;
			default:
				dev_dbg(cshift->dev, "%s: invalid code %08x\n",
					__func__, *code);
				return false;
			}
			break;

		default:
			dev_dbg(cshift->dev, "%s: mode %u not implemented\n",
				__func__, cshift->type);
			return false;
		}
		break;

	default:
		WARN_ON(1);
		return false;
	}

	return true;
}

#define cshift_subdev_call(_sd, _fn, _op, _args...)			\
	({								\
		struct cshift_device	*cshift = sd_to_cshift(sd);	\
		int			rc;				\
									\
		mutex_lock(&cshift->sensor_lock);			\
		rc = v4l2_subdev_call(cshift->sensor, _fn, _op, _args);	\
		mutex_unlock(&cshift->sensor_lock);			\
									\
		rc;							\
	})

#define _cshift_pad_call(_is_set, _sd, _op, _fh, _info, _code_)		\
	struct cshift_device		*_cshift = sd_to_cshift(_sd); \
	struct cshift_remote		remote;				\
	int				_rc = -ENOENT;			\
									\
	if (cshift_find_remote(_cshift, CSHIFT_PAD_SINK, &remote)) {	\
		uint32_t * const	_code = (_code_);		\
		unsigned int		old_pad = (_info)->pad;		\
		uint32_t		old_cod = (_code) ? *(_code) : 0; \
		__typeof__(*(_info))	tmp;				\
									\
		_rc = 0;						\
		if ((_is_set) && (_code) &&				\
		    !cshift_fixup_code_in(_cshift, old_pad, _code))	\
			_rc = -EINVAL;					\
									\
		if (_rc == 0) {						\
			tmp = *(_info);					\
			tmp.pad = remote.pad->index;			\
			_rc = v4l2_subdev_call(remote.subdev, pad,	\
					       _op, _fh, &tmp);		\
		}							\
									\
		if (_rc >= 0) {						\
			tmp.pad = old_pad;				\
			*(_info) = tmp;					\
		}							\
									\
		if (_rc >= 0 &&						\
		    (_code) != NULL &&					\
		    !cshift_fixup_code_out(_cshift, old_pad, _code))	\
			_rc = -EINVAL;					\
									\
		if (_rc < 0 && (_code) != NULL)				\
			*(_code) = old_cod;				\
	}								\
	_rc

#define cshift_pad_call(args...) ({ _cshift_pad_call(args); })

/* {{{ video ops */

#define _cshift_xlate_call(_is_set, _code_, _sd, _fngrp, _op, _args...)	\
	struct cshift_device	*_cshift = sd_to_cshift(sd);		\
	uint32_t * const	_code = (_code_);			\
	uint32_t		_old_code = (_code) ? *(_code) : 0;	\
	int			_rc;					\
									\
	_rc = 0;							\
	if ((_is_set) && (_code) &&					\
	    !cshift_fixup_code_in(_cshift, CSHIFT_ROLE_SINK, _code))	\
		_rc = -EINVAL;						\
									\
	if (_rc == 0) {							\
		mutex_lock(&_cshift->sensor_lock);			\
		_rc = v4l2_subdev_call(_cshift->sensor, _fngrp, _op, _args); \
		mutex_unlock(&_cshift->sensor_lock);			\
	}								\
									\
	if (_rc >= 0 &&							\
	    (_code) != NULL &&						\
	    !cshift_fixup_code_out(_cshift, CSHIFT_ROLE_SOURCE, _code))	\
		_rc = -EINVAL;						\
									\
	if (_rc < 0 && (_code) != NULL)					\
		*(_code) = _old_code;					\
									\
	trace_printk("rc=%d, code=%04x->%04x\n",			\
		     _rc, _old_code, _code ? *_code : 0);		\
									\
	_rc

#define cshift_xlate_call(args...) ({ _cshift_xlate_call(args); })

static int cshift_video_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	int	rc;

	rc = cshift_subdev_call(sd, video, g_crop, crop);
	return rc;
}

static int cshift_video_s_crop(struct v4l2_subdev *sd, struct v4l2_crop const *crop)
{
	int	rc;

	rc = cshift_subdev_call(sd, video, s_crop, crop);
	return rc;
}

static int cshift_video_g_mbus_fmt(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt)
{
	int	rc;

	rc = cshift_xlate_call(false, &fmt->code, sd, video, g_mbus_fmt, fmt);
	trace_printk("%s -> %d|%dx%d, %04x\n", __func__, rc, fmt->width,
		     fmt->height, fmt->code);

	return rc;
}

static int cshift_video_s_mbus_fmt(struct v4l2_subdev *sd,
				   struct v4l2_mbus_framefmt *fmt)
{
	int	rc;

	rc = cshift_xlate_call(true, &fmt->code, sd, video, s_mbus_fmt, fmt);
	trace_printk("%s -> %d|%dx%d, %d\n", __func__, rc, fmt->width,
		     fmt->height, fmt->code);

	return rc;
}

static int cshift_video_try_mbus_fmt(struct v4l2_subdev *sd,
				     struct v4l2_mbus_framefmt *fmt)
{
	int	rc;

	rc = cshift_xlate_call(true, &fmt->code, sd, video, s_mbus_fmt, fmt);
	trace_printk("%s -> %d|%dx%d, %d\n", __func__, rc, fmt->width,
		     fmt->height, fmt->code);

	return rc;
}

static int cshift_video_g_mbus_config(struct v4l2_subdev *sd,
				      struct v4l2_mbus_config *cfg)
{
	struct cshift_device	*cshift = sd_to_cshift(sd);
	int			rc;

	rc = cshift_subdev_call(sd, video, g_mbus_config, cfg);
	if (rc == -ENOIOCTLCMD) {
		*cfg = cshift->dflt_mbus_cfg;
		rc = 0;
	} else if (rc < 0) {
		/* noop */
	} else {
		cfg->flags &= ~(V4L2_MBUS_DATA_EN_ACTIVE_HIGH |
				V4L2_MBUS_DATA_EN_ACTIVE_LOW);
		cfg->flags |= (cshift->dflt_mbus_cfg.flags &
			       (V4L2_MBUS_DATA_EN_ACTIVE_HIGH|
				V4L2_MBUS_DATA_EN_ACTIVE_LOW));
	}

	return rc;
}

static int cshift_video_querystd(struct v4l2_subdev *sd, v4l2_std_id *id)
{
	return cshift_subdev_call(sd, video, querystd, id);
}

static int cshift_video_s_stream(struct v4l2_subdev *sd, int enable)
{
	trace_printk("%s(%p, %d)\n", __func__, sd, enable);
	return cshift_subdev_call(sd, video, s_stream, enable);
}

static struct v4l2_subdev_video_ops const	cshift_camera_video_ops = {
	.g_crop		= cshift_video_g_crop,
	.s_crop		= cshift_video_s_crop,
	.g_mbus_fmt	= cshift_video_g_mbus_fmt,
	.s_mbus_fmt	= cshift_video_s_mbus_fmt,
	.g_mbus_config	= cshift_video_g_mbus_config,
	.querystd	= cshift_video_querystd,
	.try_mbus_fmt	= cshift_video_try_mbus_fmt,
	.s_stream	= cshift_video_s_stream,

};

/* }}} video ops */

/* {{{ core ops */
static int cshift_core_s_power(struct v4l2_subdev *sd, int on)
{
	return cshift_subdev_call(sd, core, s_power, on);
}

static int cshift_core_g_register(struct v4l2_subdev *sd,
				  struct v4l2_dbg_register *reg)
{
#ifdef CONFIG_VIDEO_ADV_DEBUG
	return cshift_subdev_call(sd, core, g_register, reg);
#else
	return -ENOIOCTLCMD;
#endif
}

static int cshift_core_s_register(struct v4l2_subdev *sd,
				  struct v4l2_dbg_register const *reg)
{
#ifdef CONFIG_VIDEO_ADV_DEBUG
	return cshift_subdev_call(sd, core, s_register, reg);
#else
	return -ENOIOCTLCMD;
#endif
}

static struct v4l2_subdev_core_ops		cshift_camera_core_ops = {
	.s_power	= cshift_core_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= cshift_core_g_register,
	.s_register	= cshift_core_s_register,
#endif
};
/* }}} core ops */


/* {{{ pad ops */
struct cshift_remote {
	struct v4l2_subdev	*subdev;
	struct media_pad	*pad;
};

static bool cshift_find_remote(struct cshift_device *cshift,
			       int pad_num,
			       struct cshift_remote *remote)
{
	if (pad_num >= ARRAY_SIZE(cshift->pads))
		return false;

	remote->pad = media_entity_remote_pad(&cshift->pads[pad_num]);
	if (!remote->pad)
		return false;

	remote->subdev = media_entity_to_v4l2_subdev(remote->pad->entity);
	return true;
}

static int cshift_pad_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_fh *fh,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	return cshift_pad_call(0, sd, enum_mbus_code, fh, code, &code->code);
}

static int cshift_pad_enum_frame_size(struct v4l2_subdev *sd,
				      struct v4l2_subdev_fh *fh,
				      struct v4l2_subdev_frame_size_enum *fse)
{
	return cshift_pad_call(0, sd, enum_frame_size, fh, fse, &fse->code);
}

static int cshift_pad_get_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *fmt)
{
	return cshift_pad_call(0, sd, get_fmt, fh, fmt, &fmt->format.code);
}

static int cshift_pad_get_crop(struct v4l2_subdev *sd,
			       struct v4l2_subdev_fh *fh,
			       struct v4l2_subdev_crop *crop)
{
	return cshift_pad_call(0, sd, get_crop, fh, crop, NULL);
}

static int cshift_pad_set_fmt(struct v4l2_subdev *sd,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_format *fmt)
{
	return cshift_pad_call(1, sd, set_fmt, fh, fmt, &fmt->format.code);
}

static int cshift_pad_set_crop(struct v4l2_subdev *sd,
			      struct v4l2_subdev_fh *fh,
			      struct v4l2_subdev_crop *crop)
{
	return cshift_pad_call(1, sd, set_crop, fh, crop, NULL);
}

static struct v4l2_subdev_pad_ops		cshift_camera_pad_ops = {
	.link_validate		= v4l2_subdev_link_validate_default,

	.enum_mbus_code		= cshift_pad_enum_mbus_code,
	.enum_frame_size	= cshift_pad_enum_frame_size,
	.get_fmt		= cshift_pad_get_fmt,
	.get_crop		= cshift_pad_get_crop,
	.set_fmt		= cshift_pad_set_fmt,
	.set_crop		= cshift_pad_set_crop,
};
/* }}} pad ops */

static struct v4l2_subdev_ops const		cshift_camera_ops = {
	.video		= &cshift_camera_video_ops,
	.core		= &cshift_camera_core_ops,
	.pad		= &cshift_camera_pad_ops,
};

static ssize_t cshift_reg_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct v4l2_subdev		*sd = dev_get_drvdata(dev);
	struct cshift_device		*cshift = sd_to_cshift(sd);
	struct v4l2_dbg_register	reg = {
		.match	= cshift->dbg_match,
		.size	= cshift->reg_sz,
		.reg	= cshift->dbg_reg_last,
	};
	int				rc;

	rc = cshift_core_g_register(sd, &reg);

	if (rc < 0)
		return rc;

	return sprintf(buf, "[%04lx]=%lx\n",
		       (unsigned long)reg.reg, (unsigned long)reg.val);
}

static ssize_t cshift_reg_store(struct device *dev,
				struct device_attribute *attr,
				char const *buf, size_t cnt)
{
	struct v4l2_subdev		*sd = dev_get_drvdata(dev);
	struct cshift_device		*cshift = sd_to_cshift(sd);
	unsigned long			addr;
	unsigned long			val;
	int				rc;

	switch (sscanf(buf, "%li %li", &addr, &val)) {
	case 1:
		rc = 0;
		break;

	case 2: {
		struct v4l2_dbg_register const	reg = {
			.match	= cshift->dbg_match,
			.size	= cshift->reg_sz,
			.reg	= addr,
			.val	= val,
		};

		rc = cshift_core_s_register(sd, &reg);
	}
	default:
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		return rc;

	cshift->dbg_reg_last = addr;

	return cnt;
}
DEVICE_ATTR(reg, 0644, cshift_reg_show, cshift_reg_store);

struct attribute		*cshift_attributes[] = {
	&dev_attr_reg.attr,
	NULL
};

struct attribute_group const	cshift_attribute_group = {
	.attrs = cshift_attributes
};

struct cshift_of_endpoint {
	struct v4l2_of_endpoint	ep;
	struct device_node	*np;
	struct device_node	*remote;
	enum cshift_role	role;
};

static int cshift_parse_dt(struct cshift_device *cshift,
			   struct cshift_of_endpoint endpoints[],
			   size_t num_ep)
{
	struct device			*dev = cshift->dev;
	struct device_node const	*np = dev->of_node;
	int				rc;
	uint32_t			val;
	struct device_node		*epnode = NULL;
	size_t				i;
	size_t				num_sink = 0;
	size_t				num_source = 0;
	unsigned int			dflt_mbus = 0;
	struct v4l2_mbus_config		dflt_mbus_cfg = {
		.type = V4L2_MBUS_PARALLEL
	};
	unsigned int			reg_sz = 0;

	rc = of_property_read_u32(np, "phytec,shift", &val);
	if (rc < 0) {
		dev_warn(dev, "failed to get 'phytec,shift' property: %d\n",
			 rc);
		return rc;
	}

	switch (val) {
	case 2:
		cshift->type = CSHIFT_BITS2;
		break;
	case 4:
		cshift->type = CSHIFT_BITS4;
		break;
	default:
		dev_warn(dev, "unsupported 'phytec,shift' value %d\n", val);
		return -EINVAL;
	}

	memset(endpoints, 0, sizeof endpoints[0] * num_ep);

	for (;;) {
		struct v4l2_of_endpoint		ep;
		struct cshift_of_endpoint	*dst;
		struct device_node		*remote;
		size_t				idx;
		enum cshift_role		role;

		epnode = of_graph_get_next_endpoint(np, epnode);
		if (!epnode)
			break;

		rc = v4l2_of_parse_endpoint(epnode, &ep);
		if (rc < 0) {
			dev_warn(dev, "failed to parse endpoint setup: %d\n",
				 rc);
			break;
		}

		idx = ep.base.port;
		rc  = -EINVAL;

		if (of_property_match_string(epnode, "phytec,role",
					     "sink") == 0) {
			role = CSHIFT_ROLE_SINK;
			++num_sink;
		} else if (of_property_match_string(epnode, "phytec,role",
						    "source") == 0) {
			role = CSHIFT_ROLE_SOURCE;
			++num_source;
		} else {
			dev_warn(dev,
				 "bad or missing 'phytec,role' property in ep #%zu\n",
				 idx);
			break;
		}

		remote = of_graph_get_remote_port_parent(epnode);
		if (!remote && role == CSHIFT_ROLE_SINK) {
			dev_warn(dev,
				 "failed to get remote endpoint of #%zu: %d\n",
				 idx, rc);
			break;
		}

		dst = NULL;

		if (idx >= num_ep)
			dev_warn(dev, "unsupported endpoint port %d\n", idx);
		else if (endpoints[idx].np)
			dev_warn(dev, "endpoint %d already set\n", idx);
		else
			dst = &endpoints[idx];

		if (!dst) {
			of_node_put(remote);
			break;
		}

		if (role == CSHIFT_ROLE_SINK) {
			if (of_property_read_u32(epnode,
						 "phytec,default-mbus-code",
						 &val) >= 0)
				dflt_mbus = val;

			if (of_property_read_u32(epnode,
						 "phytec,regsize",
						 &val) >= 0)
				reg_sz = val;

			if (of_property_read_u32(epnode, "phytec,data-en-pol",
						 &val) >= 0)
				dflt_mbus_cfg.flags |=
					(val ?
					 V4L2_MBUS_DATA_EN_ACTIVE_HIGH :
					 V4L2_MBUS_DATA_EN_ACTIVE_LOW);
		}

		dst->ep = ep;
		dst->np = epnode;
		dst->remote = remote;
		dst->role = role;
		rc      = 0;

		dev_dbg(dev, "read ep%zu ep from dtree; role=%d, remote=%s\n",
			idx, dst->role, of_node_full_name(dst->remote));
	}

	of_node_put(epnode);
	if (rc < 0)
		goto out;

	rc = 0;
	if (num_source < 0) {
		dev_warn(dev, "no source endpoint configured\n");
		rc = -EINVAL;
	}

	if (num_sink < 0) {
		dev_warn(dev, "no sink endpoint configured\n");
		rc = -EINVAL;
	}

	if (rc < 0)
		goto out;

	for (i = 0; i < num_ep; ++i) {
		if (!endpoints[i].np) {
			dev_warn(dev, "endpoint #%d not configured\n", i);
			rc = -EINVAL;
			goto out;
		}
	}

	cshift->dflt_mbus     = dflt_mbus;
	cshift->dflt_mbus_cfg = dflt_mbus_cfg;
	cshift->reg_sz        = reg_sz;

	rc = 0;

out:
	if (rc < 0) {
		for (i = num_ep; i > 0; --i) {
			of_node_put(endpoints[i-1].remote);
			of_node_put(endpoints[i-1].np);
		}
	}

	return rc;
}

static int cshift_async_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_subdev *asd)
{
	struct cshift_device	*cshift =
		container_of(notifier, struct cshift_device, notify.n);
	int			rc;

	mutex_lock(&cshift->sensor_lock);
	if (cshift->sensor) {
		v4l2_warn(&cshift->subdev, "tried to register a second sensor\n");
		rc = -EBUSY;
	} else {
		rc = media_entity_create_link(&subdev->entity, 0,
					      &cshift->subdev.entity,
					      CSHIFT_PAD_SINK,
					      MEDIA_LNK_FL_IMMUTABLE |
					      MEDIA_LNK_FL_ENABLED);
		if (rc < 0) {
			v4l2_warn(&cshift->subdev,
				  "failed to link with sensor: %d\n", rc);
		} else {
			cshift->sensor = subdev;
			cshift->dbg_match.type = V4L2_CHIP_MATCH_SUBDEV;
			cshift->dbg_match.addr = 0;
			cshift->dbg_reg_last = 0;
			rc = 0;
		}
	}
	mutex_unlock(&cshift->sensor_lock);

	if (rc == 0)
		v4l2_info(&cshift->subdev, "bound and linked sensor '%s'\n",
			  subdev->name);

	return rc;
}

static void cshift_async_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct cshift_device	*cshift =
		container_of(notifier, struct cshift_device, notify.n);

	mutex_lock(&cshift->sensor_lock);

	if (cshift->sensor == subdev)
		cshift->sensor = NULL;

	/* TODO: is it really correct that we remove all links from the
	 * sensor?  API does not seem to provide a way just to delete the link
	 * created in cshift_async_bound() */
//	media_entity_remove_links(&subdev->entity);

	mutex_unlock(&cshift->sensor_lock);
}

static int cshift_async_complete(struct v4l2_async_notifier *notifier)
{
	struct cshift_device	*cshift =
		container_of(notifier, struct cshift_device, notify.n);
	int			rc;

	mutex_lock(&cshift->sensor_lock);
	rc = v4l2_device_register_subdev_nodes(cshift->v4l2);
	mutex_unlock(&cshift->sensor_lock);

	return rc;
}

static int cshift_add_sink(struct cshift_device *cshift,
			   struct cshift_of_endpoint const *endpoint)
{
	struct v4l2_async_subdev	*sd;

	BUG_ON(cshift->notify.sensor_np);
	BUG_ON(cshift->notify.n.num_subdevs >= ARRAY_SIZE(cshift->notify.devs));

	sd = &cshift->notify.devs[cshift->notify.n.num_subdevs];
	memset(sd, 0, sizeof *sd);

	cshift->notify.sensor_np = of_node_get(endpoint->remote);
	sd->match_type    = V4L2_ASYNC_MATCH_OF;
	sd->match.of.node = cshift->notify.sensor_np;

	cshift->notify.dev_ptr[cshift->notify.n.num_subdevs] = sd;

	++cshift->notify.n.num_subdevs;

	return 0;
}

static int cshift_handle_endpoints(struct cshift_device *cshift,
				   struct cshift_of_endpoint endpoints[],
				   size_t num_ep)
{
	size_t		i;
	int		rc;

	cshift->notify.n.num_subdevs = 0;
	cshift->notify.n.subdevs  = cshift->notify.dev_ptr;
	cshift->notify.n.bound    = cshift_async_bound;
	cshift->notify.n.unbind   = cshift_async_unbind;
	cshift->notify.n.complete = cshift_async_complete;

	rc = 0;
	for (i = 0; i < num_ep && rc == 0; ++i) {
		struct cshift_of_endpoint const	*ep = &endpoints[i];

		dev_dbg(cshift->dev, "registering ep#%zu (%s): type %d\n",
			i, ep->remote ? ep->remote->name : NULL, ep->role);

		switch (ep->role) {
		case CSHIFT_ROLE_SOURCE:
			rc = 0;		/* noop */
			break;
		case CSHIFT_ROLE_SINK:
			rc = cshift_add_sink(cshift, ep);
			break;
		default:
			BUG();
		}
	}

	if (rc < 0)
		dev_warn(cshift->dev, "failed to register endpoints: %d\n", rc);

	if (rc < 0) {
		of_node_put(cshift->notify.sensor_np);
		cshift->notify.sensor_np = NULL;
	}

	for (i = num_ep; i > 0; --i) {
		of_node_put(endpoints[i-1].remote);
		of_node_put(endpoints[i-1].np);
	}

	if (rc < 0)
		goto out;

out:
	return rc;
}

static int cshift_entity_link_setup(struct media_entity *entity,
				    const struct media_pad *local,
				    const struct media_pad *remote, u32 flags)
{
	struct cshift_device	*cshift =
		container_of(entity, struct cshift_device, subdev.entity);

	dev_info(cshift->dev, "%s: %s -> %s (%04x)\n", __func__,
		 local->entity->name, remote->entity->name, flags);

	return 0;
}

struct media_entity_operations			cshift_entity_ops = {
	.link_validate	= v4l2_subdev_link_validate,
	.link_setup	= cshift_entity_link_setup,
};

static void cshift_v4l2_release(struct v4l2_device *v4l2_dev)
{
	kfree(v4l2_dev);
}

static void cshift_registered_work(struct work_struct *work)
{
	struct cshift_device	*cshift =
		container_of(work, struct cshift_device, register_work);
	int			rc;

	if (test_and_clear_bit(CSHIFT_STATE_UNREGISTER, &cshift->flags)) {
		clear_bit(CSHIFT_STATE_REGISTER, &cshift->flags);
		v4l2_async_notifier_unregister(&cshift->notify.n);

		clear_bit(CSHIFT_STATE_REGISTERED, &cshift->flags);
		v4l2_info(cshift->v4l2, "unregistered notifier\n");
	}

	if (test_and_clear_bit(CSHIFT_STATE_REGISTER, &cshift->flags)) {
		rc = v4l2_async_notifier_register(cshift->v4l2,
						  &cshift->notify.n);
		if (rc < 0) {
			v4l2_err(cshift->v4l2,
				 "v4l2_async_notifier_register() failed: %d\n", rc);
			return;
		}

		set_bit(CSHIFT_STATE_REGISTERED, &cshift->flags);
		v4l2_info(cshift->v4l2, "registered notifier\n");
	}

	clear_bit(CSHIFT_STATE_PENDING, &cshift->flags);
}

static int cshift_registered(struct v4l2_subdev *sd)
{
	struct cshift_device	*cshift =
		container_of(sd, struct cshift_device, subdev);
	int			rc;
	struct v4l2_device	*vdev = NULL;

	if (WARN_ON(test_and_set_bit(CSHIFT_STATE_PENDING, &cshift->flags)))
		return -EBUSY;

	if (WARN_ON(test_bit(CSHIFT_STATE_REGISTERED, &cshift->flags))) {
		rc = -EBUSY;
		goto out;
	}

	vdev = kzalloc(sizeof *vdev, GFP_KERNEL);
	if (!vdev) {
		rc = -ENOMEM;
		goto out;
	}

	vdev->release = cshift_v4l2_release;
	vdev->mdev = sd->v4l2_dev->mdev;

	rc = v4l2_device_register(cshift->dev, vdev);
	if (rc < 0) {
		v4l2_err(&cshift->subdev,
			 "failed to register v4l2 device: %d\n", rc);
		goto out;
	}

	mutex_lock(&cshift->sensor_lock);
	BUG_ON(cshift->v4l2 != NULL);
	cshift->v4l2 = vdev;
	mutex_unlock(&cshift->sensor_lock);

	/* ownership of 'vdev' has been transmitted to 'cshift' */
	vdev = NULL;

	set_bit(CSHIFT_STATE_REGISTER, &cshift->flags);
	schedule_work(&cshift->register_work);

	rc = 0;

out:
	kfree(vdev);

	if (rc < 0)
		clear_bit(CSHIFT_STATE_PENDING, &cshift->flags);

	return rc;
}

static void cshift_unregistered(struct v4l2_subdev *sd)
{
	struct cshift_device	*cshift =
		container_of(sd, struct cshift_device, subdev);
	struct v4l2_device	*v4l2;

	mutex_lock(&cshift->sensor_lock);

	v4l2 = cshift->v4l2;
	cshift->v4l2 = NULL;

	mutex_unlock(&cshift->sensor_lock);

	if (v4l2)
		v4l2_device_put(v4l2);

	if (test_and_set_bit(CSHIFT_STATE_PENDING, &cshift->flags) ||
	    test_bit(CSHIFT_STATE_REGISTERED, &cshift->flags)) {
		set_bit(CSHIFT_STATE_UNREGISTER, &cshift->flags);
		schedule_work(&cshift->register_work);
	} else {
		v4l2_dbg(1, debug, sd,
			 "unregistered() called on unregistered device\n");
		clear_bit(CSHIFT_STATE_PENDING, &cshift->flags);
	}
}

static struct v4l2_subdev_internal_ops const	cshift_internal_ops = {
	.registered	= cshift_registered,
	.unregistered	= cshift_unregistered,
};

static int cshift_drv_probe(struct platform_device *pdev)
{
	struct cshift_device		*cshift;
	struct cshift_of_endpoint	endpoints[2];
	int				rc;
	struct v4l2_subdev		*sd;

	cshift = devm_kzalloc(&pdev->dev, sizeof *cshift, GFP_KERNEL);
	if (!cshift)
		return -ENOMEM;

	cshift->dev = &pdev->dev;

	rc = cshift_parse_dt(cshift, endpoints, ARRAY_SIZE(endpoints));
	if (rc < 0)
		return rc;

	rc = cshift_handle_endpoints(cshift, endpoints, ARRAY_SIZE(endpoints));
	if (rc < 0)
		return rc;

	v4l2_subdev_init(&cshift->subdev, &cshift_camera_ops);
	cshift->pads[CSHIFT_PAD_SINK].flags   |= MEDIA_PAD_FL_SINK;
	cshift->pads[CSHIFT_PAD_SOURCE].flags |= MEDIA_PAD_FL_SOURCE;

	sd = &cshift->subdev;
	sd->owner  = THIS_MODULE;
	sd->dev    = cshift->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &cshift_internal_ops;
	sd->entity.ops = &cshift_entity_ops;
	snprintf(sd->name, sizeof sd->name, "color-shifter.%d", pdev->dev.id);

	rc = media_entity_init(&sd->entity, ARRAY_SIZE(cshift->pads),
			       cshift->pads, 0);
	if (rc < 0) {
		dev_err(&pdev->dev, "media_entity_init() failed: %d\n", rc);
		goto err_media_entity_init;
	}

	mutex_init(&cshift->sensor_lock);
	INIT_WORK(&cshift->register_work, cshift_registered_work);
	platform_set_drvdata(pdev, sd);

	rc = sysfs_create_group(&pdev->dev.kobj, &cshift_attribute_group);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to create sysfs group: %d\n", rc);
		goto err_sysfs_create_group;
	}

	rc = v4l2_async_register_subdev(sd);
	if (rc < 0) {
		dev_err(&pdev->dev, "v4l2_async_register_subdev() failed: %d\n",
			rc);
		goto err_v4l2_async_register_subdev;
	}

	dev_info(&pdev->dev, "color shift device registered; type=%d, dflt-mbus=%04x\n",
		 cshift->type, cshift->dflt_mbus);

	return 0;

	sysfs_remove_group(&pdev->dev.kobj, &cshift_attribute_group);
err_sysfs_create_group:

	mutex_destroy(&cshift->sensor_lock);
	v4l2_async_unregister_subdev(&cshift->subdev);
err_v4l2_async_register_subdev:

	media_entity_cleanup(&cshift->subdev.entity);
err_media_entity_init:

	of_node_put(cshift->notify.sensor_np);

	return rc;
}

static int cshift_drv_remove(struct platform_device *pdev)
{
	struct v4l2_subdev	*sd = platform_get_drvdata(pdev);
	struct cshift_device	*cshift = sd_to_cshift(sd);

	cshift_unregistered(&cshift->subdev);
	flush_work(&cshift->register_work);

	media_entity_remove_links(&cshift->subdev.entity);
	sysfs_remove_group(&pdev->dev.kobj, &cshift_attribute_group);
	v4l2_async_unregister_subdev(&cshift->subdev);
	media_entity_cleanup(&cshift->subdev.entity);

	mutex_destroy(&cshift->sensor_lock);

	of_node_put(cshift->notify.sensor_np);

	return 0;
}

static const struct of_device_id cshift_dt_ids[] = {
	{ .compatible = "phytec,color-shifter", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cshift_dt_ids);

static struct platform_device_id cshift_id_table[] = {
	{ "phytec-color-shifter", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, cshift_id_table);

static struct platform_driver	cshift_driver = {
	.probe	= cshift_drv_probe,
	.remove	= cshift_drv_remove,
	.id_table	= cshift_id_table,
	.driver		= {
		.of_match_table = cshift_dt_ids,
		.owner	= THIS_MODULE,
		.name	= "phytec-color-shifter",
	},
};
module_platform_driver(cshift_driver);

MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
MODULE_LICENSE("GPL");
