/*
 * V4L2 Driver for i.MXL/i.MXL camera (CSI) host
 *
 * Copyright (C) 2008, Paulius Zaleckas <paulius.zaleckas@teltonika.lt>
 * Copyright (C) 2009, Darius Augulis <augulis.darius@gmail.com>
 *
 * Based on PXA SoC camera driver
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>

#include <video/imx-ipu-v3.h>
#include "imx-ipu.h"

#include <media/imx.h>
#include <linux/of_graph.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-of.h>

#define DRIVER_NAME "imx-ipuv3-camera"

#define V4L2_EVENT_SYNC_LOCK	(V4L2_EVENT_PRIVATE_START | 0x200)

struct v4l2_event_sync_lock {
	__u8 lock;
} __attribute__ ((packed));

struct ipucsi_format {
	const char *name;
	u32 mbus_code;
	u32 fourcc;
	u32 bytes_per_pixel; /* memory */
	int bytes_per_sample; /* mbus */
	unsigned rgb:1;
	unsigned yuv:1;
	unsigned raw:1;
};

int v4l2_media_subdev_s_stream(struct media_entity *entity, int enable);

static struct ipucsi_format const	ipucsi_formats_bt1120[] = {
	{
		.name = "UYVV 1x16 bit",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 1,
		.yuv = 1,
	}, {
		.name = "YUYV 1x16 bit",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 1,
		.yuv = 1,
	}, {
		.name = "UYVV 1x20 bit",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = MEDIA_BUS_FMT_UYVY10_1X20,
		.bytes_per_pixel = 4,
		.bytes_per_sample = 2,
		.yuv = 1,
	}, {
		.name = "YUYV 1x20 bit",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = MEDIA_BUS_FMT_YUYV10_1X20,
		.bytes_per_pixel = 4,
		.bytes_per_sample = 2,
		.yuv = 1,
	},
	{ /* end-of-array sentinel */ },
};

#define BAYER_FMT(_order, _bits) {					\
	.name = # _order # _bits,					\
	.fourcc = V4L2_PIX_FMT_ ## _order ## _bits,			\
	.mbus_code = MEDIA_BUS_FMT_ ## _order ## _bits ## _1X ## _bits, \
	.bytes_per_pixel = ((_bits) + 7) /  8,				\
	.bytes_per_sample = ((_bits) + 7) /  8,				\
	.raw = 1,							\
	}

static struct ipucsi_format const ipucsi_formats_raw[] = {
	{
		.name = "Monochrome 8 bit",
		.fourcc = V4L2_PIX_FMT_GREY,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.bytes_per_pixel = 1,
		.bytes_per_sample = 1,
		.raw = 1,
	}, {
		.name = "Monochrome 10 bit",
		.fourcc = V4L2_PIX_FMT_Y10,
		.mbus_code = MEDIA_BUS_FMT_Y10_1X10,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "Monochrome 12 bit",
		.fourcc = V4L2_PIX_FMT_Y16,
		.mbus_code = MEDIA_BUS_FMT_Y12_1X12,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "UYUV 2x8 bit",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 1,
		.yuv = 1,
	}, {
		.name = "YUYV 2x8 bit",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 1,
		.yuv = 1,
	}, {
		.name = "UYUV 1x16 bit",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = MEDIA_BUS_FMT_UYVY8_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "YUYV 1x16 bit",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	},
	BAYER_FMT(SBGGR,  8),
	BAYER_FMT(SBGGR, 10),
	BAYER_FMT(SBGGR, 12),
	BAYER_FMT(SGBRG,  8),
	BAYER_FMT(SGBRG, 10),
	BAYER_FMT(SGBRG, 12),
	BAYER_FMT(SGRBG,  8),
	BAYER_FMT(SGRBG, 10),
	BAYER_FMT(SGRBG, 12),
	BAYER_FMT(SRGGB,  8),
	BAYER_FMT(SRGGB, 10),
	BAYER_FMT(SRGGB, 12),
	{
		.name = "Gray 8 bit",
		.fourcc = V4L2_PIX_FMT_GREY,
		.mbus_code = MEDIA_BUS_FMT_Y8_1X8,
		.bytes_per_pixel = 1,
		.bytes_per_sample = 1,
		.raw = 1,
	}, {
		.name = "Gray 10 bit",
		.fourcc = V4L2_PIX_FMT_Y10,
		.mbus_code = MEDIA_BUS_FMT_Y10_1X10,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "Gray 12 bit",
		.fourcc = V4L2_PIX_FMT_Y12,
		.mbus_code = MEDIA_BUS_FMT_Y12_1X12,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "Gray 16 bit",
		.fourcc = V4L2_PIX_FMT_Y16,
		.mbus_code = MEDIA_BUS_FMT_Y16_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "Generic 8 bit",
		.fourcc = V4L2_PIX_FMT_IPU_GENERIC_8,
		.mbus_code = MEDIA_BUS_FMT_GENERIC_8,
		.bytes_per_pixel = 1,
		.bytes_per_sample = 1,
		.raw = 1,
	}, {
		.name = "Generic 16 bit",
		.fourcc = V4L2_PIX_FMT_IPU_GENERIC_16,
		.mbus_code = MEDIA_BUS_FMT_GENERIC_16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	},
	{ /* end-of-array sentinel */ },
};

static struct ipucsi_format const	ipucsi_format_testpattern = {
	.name = "RGB888 32bit",
	.fourcc = V4L2_PIX_FMT_RGB32,
	.mbus_code = MEDIA_BUS_FMT_FIXED,
	.bytes_per_pixel = 4,
	.bytes_per_sample = 4,
	.rgb = 1,
};

/* buffer for one video frame */
struct ipucsi_buffer {
	struct vb2_buffer		vb;
	struct list_head		queue;
};

struct ipucsi {
	struct device			*dev;
	struct v4l2_device		*v4l2_dev;
	struct video_device		vdev;
	struct completion		vdev_released;
	struct media_pad		pad;
	/* The currently active buffer, set by NFACK and cleared by EOF interrupt */
	struct ipucsi_buffer		*active;
	struct list_head		capture;
	int				ilo;

	char				name[sizeof("IPU-CSI") + 32];

	struct vb2_queue		vb2_vidq;

	struct ipu_csi			*csi;
	int				csi_id;
	struct ipu_smfc			*smfc;

	spinlock_t			lock; /* locks CSI register access */
	struct mutex			mutex;
	struct vb2_alloc_ctx		*alloc_ctx;
	enum v4l2_field			field;
	int				sequence;
	struct ipuv3_channel		*ipuch;
	struct ipu_soc			*ipu;
	struct v4l2_of_endpoint		endpoint;

	struct v4l2_format		format;
	struct ipucsi_format		ipucsifmt;
	struct v4l2_ctrl_handler	ctrls;
	struct v4l2_ctrl_handler	ctrls_vdev;
	struct v4l2_ctrl		*ctrl_test_pattern;
	struct media_pad		media_pad;
	struct media_pipeline		pipe;

	struct v4l2_subdev		subdev;
	struct media_pad		subdev_pad[2];
	struct v4l2_mbus_framefmt	format_mbus[2];
	struct ipu_media_link		*link;
	struct v4l2_fh			fh;
	bool				paused;
};

static struct ipucsi_buffer *to_ipucsi_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct ipucsi_buffer, vb);
}

static int ipu_csi_get_mbus_config(struct ipucsi *ipucsi,
				   struct v4l2_mbus_config *config)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	int ret;

	/*
	 * Retrieve media bus configuration from the entity connected directly
	 * to the CSI subdev sink pad.
	 */
	pad = media_entity_remote_pad(&ipucsi->subdev_pad[0]);
	sd = media_entity_to_v4l2_subdev(pad->entity);
	ret = v4l2_subdev_call(sd, video, g_mbus_config, config);
	if (ret == -ENOIOCTLCMD) {
		/* Fall back to static mbus configuration from device tree */
		config->type = ipucsi->endpoint.bus_type;
		config->flags = ipucsi->endpoint.bus.parallel.flags;
		ret = 0;
	}

	return ret;
}

static struct ipucsi_format const *ipu_csi_get_formats(struct ipucsi *ipucsi,
						       size_t *cnt)
{
	struct v4l2_mbus_config mbus_config;
	int			rc;

	rc = ipu_csi_get_mbus_config(ipucsi, &mbus_config);
	if (rc) {
		dev_warn(ipucsi->dev,
			 "failed to get mbus configuration: %d\n", rc);
		return NULL;
	}

	switch (mbus_config.type) {
	case V4L2_MBUS_PARALLEL:
		if (cnt)
			*cnt = ARRAY_SIZE(ipucsi_formats_raw) - 1u;

		return ipucsi_formats_raw;

	case V4L2_MBUS_BT656:
	case V4L2_MBUS_BT1120_SDR:
	case V4L2_MBUS_BT1120_DDR:
		if (cnt)
			*cnt = ARRAY_SIZE(ipucsi_formats_bt1120) - 1u;

		return ipucsi_formats_bt1120;

	default:
		WARN_ON(1);
		return NULL;
	}
}

static int ipu_csi_init_interface_local(struct ipucsi *ipucsi)
{
	struct v4l2_mbus_config		mbus_config;
	int				rc;

	rc = ipu_csi_get_mbus_config(ipucsi, &mbus_config);
	if (rc) {
		dev_warn(ipucsi->dev, "failed to get mbus configuration: %d\n",
			 rc);
		goto out;
	}

	rc = ipu_csi_init_interface(ipucsi->csi, &mbus_config,
				    &ipucsi->format_mbus[0]);
	if (rc) {
		dev_warn(ipucsi->dev, "failed to initialize iface: %d\n",
			 rc);
		goto out;
	}

	rc = ipu_csi_set_dest(ipucsi->csi, IPU_CSI_DEST_IDMAC);
	if (rc) {
		dev_warn(ipucsi->dev, "failed to set IDMAC destination: %d\n",
			 rc);
		goto out;
	}

	rc = 0;

out:
	return rc;
}

static inline void ipucsi_set_inactive_buffer(struct ipucsi *ipucsi,
					      struct vb2_buffer *vb)
{
	int bufptr = !ipu_idmac_get_current_buffer(ipucsi->ipuch);
	dma_addr_t eba = vb2_dma_contig_plane_dma_addr(vb, 0);

	if (ipucsi->ilo < 0)
		/* TODO: really? */
		eba -= ipucsi->ilo;

	ipu_cpmem_set_buffer(ipucsi->ipuch, bufptr, eba);
	ipu_idmac_select_buffer(ipucsi->ipuch, bufptr);
}

int ipucsi_resume_stream(struct ipucsi *ipucsi)
{
	struct ipucsi_buffer *buf;
	struct vb2_buffer *vb;
	unsigned long flags;
	dma_addr_t eba;

	if (!ipucsi->paused)
		return 0;

	spin_lock_irqsave(&ipucsi->lock, flags);

	if (list_empty(&ipucsi->capture)) {
		spin_unlock_irqrestore(&ipucsi->lock, flags);
		return -EAGAIN;
	}

	buf = list_first_entry(&ipucsi->capture, struct ipucsi_buffer, queue);
	vb = &buf->vb;

	ipu_idmac_set_double_buffer(ipucsi->ipuch, 1);

	eba = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (ipucsi->ilo < 0)
		eba -= ipucsi->ilo;

	ipu_cpmem_set_buffer(ipucsi->ipuch, 0, eba);

	ipu_idmac_select_buffer(ipucsi->ipuch, 0);

	/*
	 * Point the inactive buffer address to the next queued buffer,
	 * if available. Otherwise, prepare to reuse the currently active
	 * buffer, unless ipucsi_videobuf_queue gets called in time.
	 */
	if (!list_is_singular(&ipucsi->capture)) {
		buf = list_entry(ipucsi->capture.next->next,
				 struct ipucsi_buffer, queue);
		vb = &buf->vb;
	}

	eba = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (ipucsi->ilo < 0)
		eba -= ipucsi->ilo;

	ipu_cpmem_set_buffer(ipucsi->ipuch, 1, eba);

	ipu_idmac_select_buffer(ipucsi->ipuch, 1);

	spin_unlock_irqrestore(&ipucsi->lock, flags);

	ipu_smfc_enable(ipucsi->smfc);
	ipu_idmac_enable_channel(ipucsi->ipuch);
	ipu_csi_enable(ipucsi->csi);

	ipucsi->active = buf;
	ipucsi->paused = false;

	return 0;
}

static void ipucsi_clear_buffer(struct vb2_buffer *vb, u32 fourcc)
{
	u32 black, *p, *end;

	switch (fourcc) {
	case V4L2_PIX_FMT_UYVY:
		black = 0x00800080;
		break;
	case V4L2_PIX_FMT_YUYV:
		black = 0x80008000;
		break;
	default:
		black = 0x00000000;
		break;
	}

	p = vb2_plane_vaddr(vb, 0);
	end = p + vb2_plane_size(vb, 0) / 4;

	while (p < end)
		*p++ = black;
}

int ipucsi_pause_stream(struct ipucsi *ipucsi)
{
	unsigned long flags;

	if (ipucsi->paused)
		return 0;

	ipu_csi_disable(ipucsi->csi);
	ipu_idmac_disable_channel(ipucsi->ipuch);
	ipu_smfc_disable(ipucsi->smfc);

	ipucsi->paused = true;

	/*
	 * If there is a previously active frame, clear it to black and mark
	 * it as done to hand it off to userspace, unless the list is singular
	 */

	spin_lock_irqsave(&ipucsi->lock, flags);

	if (ipucsi->active && !list_is_singular(&ipucsi->capture)) {
		struct ipucsi_buffer *buf = ipucsi->active;

		ipucsi->active = NULL;
		list_del_init(&buf->queue);

		spin_unlock_irqrestore(&ipucsi->lock, flags);

		ipucsi_clear_buffer(&buf->vb,
				    ipucsi->format.fmt.pix.pixelformat);

		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);

		spin_lock_irqsave(&ipucsi->lock, flags);
	}

	spin_unlock_irqrestore(&ipucsi->lock, flags);

	return 0;
}

static void ipucsi_v4l2_dev_notify(struct v4l2_subdev *sd,
				   unsigned int notification, void *arg)
{
	if (sd == NULL)
		return;

	if (notification == V4L2_SUBDEV_SYNC_LOCK_NOTIFY) {
		struct media_entity_graph graph;
		struct media_entity *entity;
		struct v4l2_event event;
		struct ipucsi *ipucsi;
		bool lock = *(bool *)arg;

		/* Find the CSI (first subdevice entity of the graph) */
		media_entity_graph_walk_start(&graph, &sd->entity);
		while ((entity = media_entity_graph_walk_next(&graph)) &&
		       media_entity_type(entity) != MEDIA_ENT_T_V4L2_SUBDEV);
		if (!entity)
			return;
		sd = media_entity_to_v4l2_subdev(entity);
		ipucsi = container_of(sd, struct ipucsi, subdev);

		if (lock)
			ipucsi_resume_stream(ipucsi);
		else
			ipucsi_pause_stream(ipucsi);

		memset(&event, 0, sizeof(event));
		event.type = V4L2_EVENT_SYNC_LOCK;
		((struct v4l2_event_sync_lock *)event.u.data)->lock = lock;
		v4l2_event_queue(&ipucsi->vdev, &event);
	}
}

static irqreturn_t ipucsi_new_frame_handler(int irq, void *context)
{
	struct ipucsi *ipucsi = context;
	struct ipucsi_buffer *buf;
	struct vb2_buffer *vb;
	unsigned long flags;

	/* The IDMAC just started to write pixel data into the current buffer */

	spin_lock_irqsave(&ipucsi->lock, flags);

	/*
	 * If there is a previously active frame, mark it as done to hand it off
	 * to userspace. Or, if there are no further frames queued, hold on to it.
	 */
	if (ipucsi->active) {
		vb = &ipucsi->active->vb;
		buf = to_ipucsi_vb(vb);

		if (vb2_is_streaming(vb->vb2_queue) && list_is_singular(&ipucsi->capture)) {
			pr_debug("%s: reusing 0x%08x\n", __func__,
				vb2_dma_contig_plane_dma_addr(vb, 0));
			/* DEBUG: check if buf == EBA(active) */
		} else {
			/* Otherwise, mark buffer as finished */
			list_del_init(&buf->queue);

			vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		}
	}

	if (list_empty(&ipucsi->capture))
		goto out;

	ipucsi->active = list_first_entry(&ipucsi->capture,
					   struct ipucsi_buffer, queue);
	vb = &ipucsi->active->vb;
	do_gettimeofday(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.field = ipucsi->format.fmt.pix.field;
	vb->v4l2_buf.sequence = ipucsi->sequence++;

	/*
	 * Point the inactive buffer address to the next queued buffer,
	 * if available. Otherwise, prepare to reuse the currently active
	 * buffer, unless ipucsi_videobuf_queue gets called in time.
	 */
	if (!list_is_singular(&ipucsi->capture)) {
		buf = list_entry(ipucsi->capture.next->next,
				 struct ipucsi_buffer, queue);
		vb = &buf->vb;
	}
	ipucsi_set_inactive_buffer(ipucsi, vb);
out:
	spin_unlock_irqrestore(&ipucsi->lock, flags);

	return IRQ_HANDLED;
}

static struct ipucsi_format const *ipucsi_current_format(struct ipucsi *ipucsi)
{
	if (ipucsi->ctrl_test_pattern->val)
		return &ipucsi_format_testpattern;
	else
		return &ipucsi->ipucsifmt;
}

/*
 *  Videobuf operations
 */
static int ipucsi_videobuf_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		unsigned int *count, unsigned int *num_planes,
		unsigned int sizes[], void *alloc_ctxs[])
{
	struct ipucsi *ipucsi = vq->drv_priv;
	int bytes_per_line;
	struct ipucsi_format const *ipucsifmt = ipucsi_current_format(ipucsi);

	if (!fmt)
		fmt = &ipucsi->format;

	bytes_per_line = fmt->fmt.pix.width * ipucsifmt->bytes_per_pixel;

	dev_dbg(ipucsi->dev, "bytes: %d x: %d y: %d",
			bytes_per_line, fmt->fmt.pix.width, fmt->fmt.pix.height);

	*num_planes = 1;

	ipucsi->sequence = 0;
	sizes[0] = bytes_per_line * fmt->fmt.pix.height;
	alloc_ctxs[0] = ipucsi->alloc_ctx;

	if (!*count)
		*count = 32;

	return 0;
}

static int ipucsi_videobuf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct ipucsi *ipucsi = vq->drv_priv;
	size_t new_size;
	struct ipucsi_buffer *buf;
	struct v4l2_pix_format *pix = &ipucsi->format.fmt.pix;
	struct ipucsi_format const *ipucsifmt = ipucsi_current_format(ipucsi);

	buf = to_ipucsi_vb(vb);

	new_size = pix->width * pix->height * ipucsifmt->bytes_per_pixel;

	if (vb2_plane_size(vb, 0) < new_size)
		return -ENOBUFS;

	vb2_set_plane_payload(vb, 0, new_size);

	return 0;
}

static void ipucsi_videobuf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct ipucsi *ipucsi = vq->drv_priv;
	struct ipucsi_buffer *buf = to_ipucsi_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&ipucsi->lock, flags);

	/*
	 * If there is no next buffer queued, point the inactive buffer
	 * address to the incoming buffer
	 */
	if (vb2_is_streaming(vb->vb2_queue) && list_is_singular(&ipucsi->capture))
		ipucsi_set_inactive_buffer(ipucsi, vb);

	list_add_tail(&buf->queue, &ipucsi->capture);

	spin_unlock_irqrestore(&ipucsi->lock, flags);
}

static void ipucsi_videobuf_release(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct ipucsi *ipucsi = vq->drv_priv;
	struct ipucsi_buffer *buf = to_ipucsi_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&ipucsi->lock, flags);

	if (ipucsi->active == buf)
		ipucsi->active = NULL;

	if (!list_empty(&buf->queue))
		list_del_init(&buf->queue);

	spin_unlock_irqrestore(&ipucsi->lock, flags);
}

static int ipucsi_videobuf_init(struct vb2_buffer *vb)
{
	struct ipucsi_buffer *buf = to_ipucsi_vb(vb);

	/* This is for locking debugging only */
	INIT_LIST_HEAD(&buf->queue);

	return 0;
}

#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
	((x) >> 24) & 0xff

static int ipucsi_videobuf_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ipucsi *ipucsi = vq->drv_priv;
	struct ipuv3_channel *ipuch = ipucsi->ipuch;
	struct ipucsi_format const *ipucsifmt = ipucsi_current_format(ipucsi);
	int xres = ipucsi->format.fmt.pix.width;
	int yres = ipucsi->format.fmt.pix.height;
	struct device *dev = ipucsi->dev;
	int burstsize;
	struct vb2_buffer *vb;
	struct ipucsi_buffer *buf;
	int nfack_irq;
	int ret;
	struct v4l2_rect w = {
		.width	= xres,
		.height	= yres,
	};

	ipu_cpmem_zero(ipuch);

	nfack_irq = ipu_idmac_channel_irq(ipucsi->ipu, ipucsi->ipuch,
			IPU_IRQ_NFACK);
	ret = request_threaded_irq(nfack_irq, NULL, ipucsi_new_frame_handler, IRQF_ONESHOT,
			"ipucsi-nfack", ipucsi);
	if (ret) {
		dev_err(dev, "Failed to request NFACK interrupt: %d\n", nfack_irq);
		return ret;
	}

	dev_dbg(dev, "width: %d height: %d, %c%c%c%c\n",
			xres, yres, pixfmtstr(ipucsi->format.fmt.pix.pixelformat));

	ipu_cpmem_set_resolution(ipuch, xres, yres);

	if (ipucsifmt->raw) {
		/*
		 * raw formats. We can only pass them through to memory
		 */
		ipu_cpmem_set_stride(ipuch, xres * ipucsifmt->bytes_per_pixel);
		ret = ipu_cpmem_set_format_passthrough(
			ipuch, ipucsifmt->bytes_per_sample * 8);
		if (ret) {
			dev_err(dev,
				"failed to configure passthrough mode for %dBps\n",
				ipucsifmt->bytes_per_sample);
			return ret;
		}
	} else {
		/*
		 * formats we understand, we can write it in any format not requiring
		 * colorspace conversion.
		 */
		u32 fourcc = ipucsi->format.fmt.pix.pixelformat;
		ret = 0;

		switch (fourcc) {
		case V4L2_PIX_FMT_RGB32:
			ipu_cpmem_set_stride(ipuch, xres * 4);
			ret = ipu_cpmem_set_fmt(ipuch, fourcc);
			break;
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUYV:
			ipu_cpmem_set_stride(ipuch, xres * 2);
			ipu_cpmem_set_yuv_interleaved(ipuch, fourcc);
			break;
		case V4L2_PIX_FMT_YUV420:
			ipu_cpmem_set_stride(ipuch, xres);
			ipu_cpmem_set_yuv_planar(ipuch, V4L2_PIX_FMT_YUV420, xres, yres);
			break;
		default:
			ret = -EINVAL;
			break;
		}

		if (ret) {
			dev_warn(dev,
				 "failed to set %08x fourcc mode: %d\n",
				 fourcc, ret);
			goto free_irq;
		}
	}

	if (ipucsi->ilo) {
		int stride;

		if (ipucsi->ilo < 0)
			stride = (0x100000 - (ipucsi->ilo/8)) * 8;
		else
			stride = 0;

		ipu_cpmem_interlaced_scan(ipuch, stride);
	}

	/*
	 * Some random value. The reference manual tells us that the burstsize
	 * is a function of the IDMACs PFS, BPP and NPB settings. Unfortunately
	 * it doesn't tell us which function this is.
	 */
	burstsize = 8;

	ipu_smfc_set_burstsize(ipucsi->smfc, burstsize - 1);
	ipu_smfc_map_channel(ipucsi->smfc, ipucsi->csi_id, 0);

	ipu_cpmem_set_high_priority(ipucsi->ipuch);

	ipu_csi_set_window(ipucsi->csi, &w);

	ret = media_entity_pipeline_start(&ipucsi->subdev.entity, &ipucsi->pipe);
	if (ret) {
		v4l2_err(&ipucsi->subdev, "failed to start pipeline: %d\n", ret);
		goto free_irq;
	}

	ret = ipu_csi_init_interface_local(ipucsi);
	if (ret) {
		v4l2_err(&ipucsi->subdev, "failed to initialized interface: %d\n", ret);
		goto free_irq;
	}

	ipu_idmac_set_double_buffer(ipucsi->ipuch, 1);

	if (list_empty(&ipucsi->capture)) {
		v4l2_err(&ipucsi->subdev, "No capture buffers\n");
		ret = -ENOMEM;
		goto free_irq;
	}

	ipucsi->active = NULL;

	/* Point the inactive buffer address to the first buffer */
	buf = list_first_entry(&ipucsi->capture, struct ipucsi_buffer, queue);
	vb = &buf->vb;
	ipucsi_set_inactive_buffer(ipucsi, vb);

	ipu_idmac_enable_channel(ipucsi->ipuch);
	ipu_smfc_enable(ipucsi->smfc);
	ipu_csi_enable(ipucsi->csi);

	ipucsi->paused = false;

	ret = v4l2_media_subdev_s_stream(&ipucsi->subdev.entity, 1);
	if (ret)
		goto free_irq;

	return 0;

free_irq:
	free_irq(nfack_irq, ipucsi);
	return ret;
}

static void ipucsi_videobuf_stop_streaming(struct vb2_queue *vq)
{
	struct ipucsi *ipucsi = vq->drv_priv;
	unsigned long flags;
	int nfack_irq = ipu_idmac_channel_irq(ipucsi->ipu, ipucsi->ipuch,
				IPU_IRQ_NFACK);

	free_irq(nfack_irq, ipucsi);
	ipu_csi_disable(ipucsi->csi);
	ipu_idmac_disable_channel(ipucsi->ipuch);
	ipu_smfc_disable(ipucsi->smfc);

	ipucsi->paused = false;

	spin_lock_irqsave(&ipucsi->lock, flags);
	while (!list_empty(&ipucsi->capture)) {
		struct ipucsi_buffer *buf = list_entry(ipucsi->capture.next,
						 struct ipucsi_buffer, queue);
		list_del_init(ipucsi->capture.next);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&ipucsi->lock, flags);

	media_entity_pipeline_stop(&ipucsi->subdev.entity);

	v4l2_media_subdev_s_stream(&ipucsi->subdev.entity, 0);

	return;
}

static void ipucsi_lock(struct vb2_queue *vq)
{
	struct ipucsi *ipucsi = vq->drv_priv;
	mutex_lock(&ipucsi->mutex);
}

static void ipucsi_unlock(struct vb2_queue *vq)
{
	struct ipucsi *ipucsi = vq->drv_priv;
	mutex_unlock(&ipucsi->mutex);
}

static struct vb2_ops ipucsi_videobuf_ops = {
	.queue_setup		= ipucsi_videobuf_setup,
	.buf_prepare		= ipucsi_videobuf_prepare,
	.buf_queue		= ipucsi_videobuf_queue,
	.buf_cleanup		= ipucsi_videobuf_release,
	.buf_init		= ipucsi_videobuf_init,
	.start_streaming	= ipucsi_videobuf_start_streaming,
	.stop_streaming		= ipucsi_videobuf_stop_streaming,
	.wait_prepare		= ipucsi_unlock,
	.wait_finish		= ipucsi_lock,
};

static int ipucsi_querycap(struct file *file, void *priv,
					struct v4l2_capability *cap)
{
	strlcpy(cap->driver, "imx-ipuv3-csi", sizeof(cap->driver));
	/* cap->name is set by the friendly caller:-> */
	strlcpy(cap->card, "imx-ipuv3-camera", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:imx-ipuv3-csi", sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int ipucsi_try_fmt(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipucsi *ipucsi = video_drvdata(file);
	struct ipucsi_format const *ipucsifmt = ipucsi_current_format(ipucsi);
	enum v4l2_field in = ipucsi->format_mbus[1].field;
	enum v4l2_field out = f->fmt.pix.field;
	struct ipu_fmt *fmt = NULL;
	int bytes_per_pixel;

	if (ipucsifmt->rgb)
		fmt = ipu_find_fmt_rgb(f->fmt.pix.pixelformat);
	else if (ipucsifmt->yuv)
		fmt = ipu_find_fmt_yuv(f->fmt.pix.pixelformat);
	else if (ipucsifmt->raw)
		fmt = ipu_find_fmt_raw(f->fmt.pix.pixelformat);

	if (ipucsifmt->raw) {
		f->fmt.pix.pixelformat = ipucsifmt->fourcc;
		bytes_per_pixel = ipucsifmt->bytes_per_pixel;
	} else {
		if (!fmt)
			return -EINVAL;
		bytes_per_pixel = fmt->bytes_per_pixel;
	}

	v4l_bound_align_image(&f->fmt.pix.width, 128,
			      ipucsi->format_mbus[1].width, 3,
			      &f->fmt.pix.height, 128,
			      ipucsi->format_mbus[1].height, 1, 0);

	f->fmt.pix.bytesperline = f->fmt.pix.width * bytes_per_pixel;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;

	if ((in == V4L2_FIELD_SEQ_TB && out == V4L2_FIELD_INTERLACED_TB) ||
	    (in == V4L2_FIELD_INTERLACED_TB && out == V4L2_FIELD_SEQ_TB) ||
	    (in == V4L2_FIELD_SEQ_BT && out == V4L2_FIELD_INTERLACED_BT) ||
	    (in == V4L2_FIELD_INTERLACED_BT && out == V4L2_FIELD_SEQ_BT)) {
		/*
		 * IDMAC scan order can be used for translation between
		 * interlaced and sequential field formats.
		 */
	} else if (out == V4L2_FIELD_NONE || out == V4L2_FIELD_INTERLACED) {
		/*
		 * If userspace requests progressive or interlaced frames,
		 * interlace sequential fields as closest approximation.
		 */
		if (in == V4L2_FIELD_SEQ_TB)
			out = V4L2_FIELD_INTERLACED_TB;
		else if (in == V4L2_FIELD_SEQ_BT)
			out = V4L2_FIELD_INTERLACED_BT;
		else
			out = in;
	} else {
		/* Translation impossible or userspace doesn't care */
		out = in;
	}
	f->fmt.pix.field = out;

	return 0;
}

static int ipucsi_s_fmt(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipucsi *ipucsi = video_drvdata(file);
	enum v4l2_field in, out;
	int ret;

	ret = ipucsi_try_fmt(file, fh, f);
	if (ret)
		return ret;

	ipucsi->format = *f;

	/*
	 * Set IDMAC scan order interlace offset (ILO) for translation between
	 * interlaced and sequential field formats.
	 */
	in = ipucsi->format_mbus[1].field;
	out = f->fmt.pix.field;
	if ((in == V4L2_FIELD_SEQ_TB && out == V4L2_FIELD_INTERLACED_TB) ||
	    (in == V4L2_FIELD_INTERLACED_TB && out == V4L2_FIELD_SEQ_TB))
		ipucsi->ilo = f->fmt.pix.bytesperline;
	else if ((in == V4L2_FIELD_SEQ_BT && out == V4L2_FIELD_INTERLACED_BT) ||
		 (in == V4L2_FIELD_INTERLACED_BT && out == V4L2_FIELD_SEQ_BT))
		ipucsi->ilo = -f->fmt.pix.bytesperline;
	else
		ipucsi->ilo = 0;

	return 0;
}

static int ipucsi_g_fmt(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipucsi *ipucsi = video_drvdata(file);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	*f = ipucsi->format;
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int ipucsi_enum_fmt(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct ipucsi *ipucsi = video_drvdata(file);

	if (ipucsi->ctrl_test_pattern->val) {
		if (f->index)
			return -EINVAL;
		strlcpy(f->description, ipucsi_format_testpattern.name,
				sizeof(f->description));
		f->pixelformat = ipucsi_format_testpattern.fourcc;

		return 0;
	}

	if (ipucsi->ipucsifmt.rgb)
		return ipu_enum_fmt_rgb(file, priv, f);
	else if (ipucsi->ipucsifmt.yuv)
		return ipu_enum_fmt_yuv(file, priv, f);
	else if (ipucsi->ipucsifmt.raw)
		return ipu_enum_fmt_raw(file, priv, f);

	if (f->index)
		return -EINVAL;

	if (!ipucsi->ipucsifmt.name)
		return -EINVAL;

	strlcpy(f->description, ipucsi->ipucsifmt.name, sizeof(f->description));
	f->pixelformat = ipucsi->ipucsifmt.fourcc;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ipucsi_get_pad_format(struct ipucsi *ipucsi,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *sdformat)
{
	switch (sdformat->which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&ipucsi->subdev, cfg,
						  sdformat->pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ipucsi->format_mbus[sdformat->pad];
	default:
		return NULL;
	}
}

static int ipucsi_subdev_get_format(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_format *sdformat)
{
	struct ipucsi *ipucsi = container_of(subdev, struct ipucsi, subdev);

	sdformat->format = *__ipucsi_get_pad_format(ipucsi, cfg, sdformat);
	return 0;
}

static struct ipucsi_format const *ipucsi_find_subdev_format(
	struct ipucsi *ipucsi, const u32 *fourcc, const u32 *mbus_code)
{
	struct ipucsi_format const *ipucsifmt;
	int i;

	ipucsifmt = ipu_csi_get_formats(ipucsi, NULL);
	if (!ipucsifmt)
		return NULL;

	for (i = 0; ipucsifmt[i].name != NULL; i++) {
		if (fourcc && *fourcc == ipucsifmt[i].fourcc)
			return &ipucsifmt[i];
		if (mbus_code && *mbus_code == ipucsifmt[i].mbus_code)
			return &ipucsifmt[i];
	}

	dev_warn(ipucsi->dev, "no format found for %04x(%p)/%04x(%p) in %p\n",
		 fourcc ? *fourcc : 0, fourcc,
		 mbus_code ? *mbus_code : 0, mbus_code,
		 ipucsifmt);

	return NULL;
}

static int ipucsi_subdev_set_format(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_format *sdformat)
{
	struct ipucsi *ipucsi = container_of(subdev, struct ipucsi, subdev);
	struct v4l2_mbus_framefmt *mbusformat;
	struct ipucsi_format const *ipucsiformat;
	unsigned int width, height;

	ipucsiformat = ipucsi_find_subdev_format(ipucsi, NULL,
						 &sdformat->format.code);
	if (!ipucsiformat)
		return -EINVAL;

	width = clamp_t(unsigned int, sdformat->format.width, 16, 8192);
	height = clamp_t(unsigned int, sdformat->format.height, 16, 4096);

	mbusformat = __ipucsi_get_pad_format(ipucsi, cfg, sdformat);
	mbusformat->width = width;
	mbusformat->height = height;
	mbusformat->code = ipucsiformat->mbus_code;
	mbusformat->field = sdformat->format.field;

	if (mbusformat->field == V4L2_FIELD_SEQ_TB &&
	    mbusformat->width == 720 && mbusformat->height == 480 &&
	    ipucsi->endpoint.bus_type == V4L2_MBUS_BT656) {
		/* We capture NTSC bottom field first */
		mbusformat->field = V4L2_FIELD_SEQ_BT;
	} else if (mbusformat->field == V4L2_FIELD_ANY &&
		   sdformat->pad == 0 &&
		   ipucsi->format_mbus[1].field == V4L2_FIELD_ANY) {
		struct media_pad *pad;

		pad = media_entity_remote_pad(&ipucsi->subdev_pad[0]);
		if (pad) {
			int rc;
			struct v4l2_subdev *sd;
			struct v4l2_subdev_format sens_fmt = {
				.which	= V4L2_SUBDEV_FORMAT_ACTIVE,
				.format	= sdformat->format,
				.pad	= pad->index,
			};
			sd = media_entity_to_v4l2_subdev(pad->entity);
			sens_fmt.pad = pad->index;

			rc = v4l2_subdev_call(sd, pad, get_fmt, cfg, &sens_fmt);
			if (!rc)
				mbusformat->field = sens_fmt.format.field;
		}
	} else if (mbusformat->field == V4L2_FIELD_ANY) {
		mbusformat->field = ipucsi->format_mbus[!sdformat->pad].field;
	}

	sdformat->format = *mbusformat;

	ipucsi->ipucsifmt = *ipucsiformat;

	return 0;
}

static struct v4l2_subdev_pad_ops ipucsi_subdev_pad_ops = {
	.get_fmt = ipucsi_subdev_get_format,
	.set_fmt = ipucsi_subdev_set_format,
};

static const struct v4l2_subdev_ops ipucsi_subdev_ops = {
	.pad    = &ipucsi_subdev_pad_ops,
};

struct media_entity_operations ipucsi_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

int v4l2_media_subdev_s_stream(struct media_entity *entity, int enable)
{
	struct media_entity_graph graph;
	struct media_entity *first;
	struct v4l2_subdev *sd;
	int ret = 0;

	first = entity;

	media_entity_graph_walk_start(&graph, entity);
	if (!enable) {
		first = NULL;
		goto disable;
	}

	while (!ret && (entity = media_entity_graph_walk_next(&graph))) {
		if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(entity);
			ret = v4l2_subdev_call(sd, video, s_stream, 1);
			if (ret == -ENOIOCTLCMD)
				ret = 0;
		}
	}

	if (!ret)
		return 0;

	media_entity_graph_walk_start(&graph, first);

disable:
	while ((entity = media_entity_graph_walk_next(&graph)) && first != entity) {
		if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(entity);
			v4l2_subdev_call(sd, video, s_stream, 0);
		}
	}

	return ret;
}

int v4l2_media_subdev_s_power(struct ipucsi *ipucsi, int enable)
{
	struct media_entity *entity = &ipucsi->subdev.entity;
	struct media_entity_graph graph;
	struct media_entity *first;
	struct v4l2_subdev *sd;
	int ret = 0;

	first = entity;

	media_entity_graph_walk_start(&graph, entity);
	if (!enable) {
		first = NULL;
		goto disable;
	}

	v4l2_ctrl_handler_init(&ipucsi->ctrls_vdev, 1);

	while (!ret && (entity = media_entity_graph_walk_next(&graph))) {
		if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(entity);
			ret = v4l2_subdev_call(sd, core, s_power, 1);
			if (ret == -ENOIOCTLCMD)
				ret = 0;

			ret = v4l2_ctrl_add_handler(&ipucsi->ctrls_vdev,
						    sd->ctrl_handler, NULL);
			if (ret)
				return ret;
		}
	}

	if (!ret)
		return 0;

	media_entity_graph_walk_start(&graph, first);

disable:
	while ((entity = media_entity_graph_walk_next(&graph)) && first != entity) {
		if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(entity);
			v4l2_subdev_call(sd, core, s_power, 0);
		}
	}

	return ret;
}

static int ipucsi_open(struct file *file)
{
	struct ipucsi *ipucsi = video_drvdata(file);
	int ret;

	mutex_lock(&ipucsi->mutex);
	ret = v4l2_fh_open(file);
	if (ret)
		goto out;

	if (v4l2_fh_is_singular_file(file))
		ret = v4l2_media_subdev_s_power(ipucsi, 1);

out:
	mutex_unlock(&ipucsi->mutex);
	return ret;
}

static int ipucsi_release(struct file *file)
{
	struct ipucsi *ipucsi = video_drvdata(file);

	mutex_lock(&ipucsi->mutex);
	mutex_unlock(&ipucsi->mutex);

	if (v4l2_fh_is_singular_file(file)) {
		v4l2_media_subdev_s_power(ipucsi, 0);

		v4l2_ctrl_handler_free(&ipucsi->ctrls_vdev);

		vb2_fop_release(file);
	} else {
		v4l2_fh_release(file);
	}

	return 0;
}

static u64 camera_mask = DMA_BIT_MASK(32);

static const struct v4l2_file_operations ipucsi_capture_fops = {
	.owner		= THIS_MODULE,
	.open		= ipucsi_open,
	.release	= ipucsi_release,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
};

static int ipucsi_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *fsize)
{
	struct ipucsi *ipucsi = video_drvdata(file);
	struct ipucsi_format const *ipucsifmt = ipucsi_current_format(ipucsi);
	struct v4l2_mbus_config mbus_config;
	struct ipu_fmt *fmt = NULL;
	int ret;

	ret = ipu_csi_get_mbus_config(ipucsi, &mbus_config);
	if (ret)
		return ret;

	if (((fsize->index != 0) &&
	     (mbus_config.type != V4L2_MBUS_BT656)) ||
	    (fsize->index > 1))
		return -EINVAL;

	if (ipucsifmt->rgb)
		fmt = ipu_find_fmt_rgb(fsize->pixel_format);
	else if (ipucsifmt->yuv)
		fmt = ipu_find_fmt_yuv(fsize->pixel_format);
	else if (ipucsifmt->raw)
		fmt = ipu_find_fmt_raw(fsize->pixel_format);

	if (!fmt)
		return -EINVAL;

	if (mbus_config.type == V4L2_MBUS_BT656) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = 720;
		fsize->discrete.height = fsize->index ? 576 : 480;
	} else {
		fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
		fsize->stepwise.min_width = 1;
		fsize->stepwise.min_height = 1;
		fsize->stepwise.max_width = ipucsi->format_mbus[1].width;
		fsize->stepwise.max_height = ipucsi->format_mbus[1].height;
		fsize->stepwise.step_width = 1;
		fsize->stepwise.step_height = 1;
	}

	return 0;
}

static int ipucsi_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	if (sub->type == V4L2_EVENT_SYNC_LOCK)
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	return -EINVAL;
}

static int ipucsi_enum_input(struct file *file, void *fh,
			     struct v4l2_input *inp)
{
	if (inp->index > 0)
		return -EINVAL;

	strcpy(inp->name, "IPUv3-CSI");
	inp->type         = V4L2_INPUT_TYPE_CAMERA;
	inp->std          = V4L2_STD_UNKNOWN;
	inp->audioset     = 0;
	inp->tuner        = 0;
	inp->status       = 0;
	inp->capabilities = 0;

	return 0;
}

static int ipucsi_s_input(struct file *file, void *fh, unsigned int i)
{
	if (i != 0)
		return -EINVAL;

	return 0;
}

static int ipucsi_g_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;

	return 0;
}

static const struct v4l2_ioctl_ops ipucsi_capture_ioctl_ops = {
	.vidioc_querycap		= ipucsi_querycap,

	.vidioc_enum_fmt_vid_cap	= ipucsi_enum_fmt,
	.vidioc_try_fmt_vid_cap		= ipucsi_try_fmt,
	.vidioc_s_fmt_vid_cap		= ipucsi_s_fmt,
	.vidioc_g_fmt_vid_cap		= ipucsi_g_fmt,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,

	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,

	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_enum_framesizes		= ipucsi_enum_framesizes,

	.vidioc_subscribe_event		= ipucsi_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

	.vidioc_enum_input		= ipucsi_enum_input,
	.vidioc_s_input			= ipucsi_s_input,
	.vidioc_g_input			= ipucsi_g_input,
};

static int ipucsi_subdev_s_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ipucsi_subdev_ctrl_ops = {
	.s_ctrl = ipucsi_subdev_s_ctrl,
};

static const char * const ipucsi_test_pattern_menu[] = {
	"Disabled",
	"Checkerboard-red",
	"Checkerboard-green",
	"Checkerboard-blue",
};

static int ipucsi_create_controls(struct ipucsi *ipucsi)
{
	struct v4l2_ctrl_handler *handler = &ipucsi->ctrls;

	v4l2_ctrl_handler_init(handler, 1);

	ipucsi->ctrl_test_pattern = v4l2_ctrl_new_std_menu_items(handler,
			&ipucsi_subdev_ctrl_ops, V4L2_CID_TEST_PATTERN,
			ARRAY_SIZE(ipucsi_test_pattern_menu) - 1, 0, 0,
			ipucsi_test_pattern_menu);

	return ipucsi->ctrl_test_pattern ? 0 : -ENOMEM;
}

static int ipucsi_subdev_init(struct ipucsi *ipucsi, struct device_node *node)
{
	struct device_node *endpoint;
	int ret;

	v4l2_subdev_init(&ipucsi->subdev, &ipucsi_subdev_ops);

	ipucsi->subdev.dev = ipucsi->dev;
	ipucsi->subdev.ctrl_handler = &ipucsi->ctrls;

	snprintf(ipucsi->subdev.name, sizeof(ipucsi->subdev.name), "%s-sd",
		 ipucsi->name);

	endpoint = of_get_next_child(node, NULL);
	if (endpoint)
		v4l2_of_parse_endpoint(endpoint, &ipucsi->endpoint);
	of_node_put(endpoint);

	ipucsi->subdev.entity.ops = &ipucsi_entity_ops;

	ipucsi->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ipucsi->subdev_pad[0].flags = MEDIA_PAD_FL_SINK;
	ipucsi->subdev_pad[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(&ipucsi->subdev.entity, 2, ipucsi->subdev_pad, 0);
	if (ret < 0)
		return ret;

	ret = v4l2_device_register_subdev(ipucsi->v4l2_dev, &ipucsi->subdev);
	if (ret < 0) {
		media_entity_cleanup(&ipucsi->subdev.entity);
		return ret;
	}

	return 0;
}

static void ipucsi_video_device_release(struct video_device *vdev)
{
	struct ipucsi	*ipu = container_of(vdev, struct ipucsi, vdev);

	complete(&ipu->vdev_released);
}

static int ipucsi_video_device_init(struct platform_device *pdev,
		struct ipucsi *ipucsi)
{
	struct video_device *vdev = &ipucsi->vdev;
	int ret;

	snprintf(vdev->name, sizeof(vdev->name), "%s-video", ipucsi->name);
	vdev->release	= ipucsi_video_device_release;
	vdev->fops	= &ipucsi_capture_fops;
	vdev->ioctl_ops	= &ipucsi_capture_ioctl_ops;
	vdev->v4l2_dev	= ipucsi->v4l2_dev;
	vdev->dev_parent = &pdev->dev;
	vdev->minor	= -1;
	vdev->lock	= &ipucsi->mutex;
	vdev->ctrl_handler = &ipucsi->ctrls_vdev;
	vdev->queue	= &ipucsi->vb2_vidq;

	video_set_drvdata(vdev, ipucsi);

	ipucsi->media_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&vdev->entity, 1, &ipucsi->media_pad, 0);
	if (ret < 0)
		video_device_release_empty(vdev);

	return ret;
}

static int ipucsi_vb2_init(struct ipucsi *ipucsi)
{
	struct vb2_queue *q;
	int ret;

	ipucsi->alloc_ctx = vb2_dma_contig_init_ctx(ipucsi->dev);
	if (IS_ERR(ipucsi->alloc_ctx))
		return PTR_ERR(ipucsi->alloc_ctx);

	q = &ipucsi->vb2_vidq;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = ipucsi;
	q->ops = &ipucsi_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct ipucsi_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	return 0;
}

static int ipucsi_async_init(struct ipucsi *ipucsi, struct device_node *node)
{
	struct device_node *rp;
	int rc;

	rp = of_get_next_child(node, NULL);
	if (!rp)
		return 0;

	ipucsi->link = ipu_media_entity_create_link(&ipucsi->subdev, 0, rp,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

	if (IS_ERR(ipucsi->link)) {
		rc = PTR_ERR(ipucsi->link);
		ipucsi->link = NULL;
		return rc;
	}
	return 0;
}

static struct device_node *ipucsi_get_port(struct device_node *node, int id)
{
	struct device_node *port;
	int reg;

	for_each_child_of_node(node, port) {
		if (!of_property_read_u32(port, "reg", &reg) && reg == id)
			return of_node_get(port);
	}

	return NULL;
}

static int ipucsi_probe(struct platform_device *pdev)
{
	struct ipu_client_platformdata *pdata = pdev->dev.platform_data;
	struct ipu_soc *ipu = dev_get_drvdata(pdev->dev.parent);
	struct ipucsi *ipucsi;
	struct resource *res;
	int ret;
	struct device_node *node = NULL;

	pdev->dev.dma_mask = &camera_mask,
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32),

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (!pdata)
		return -EINVAL;

	ipucsi = devm_kzalloc(&pdev->dev, sizeof(*ipucsi), GFP_KERNEL);
	if (!ipucsi)
		return -ENOMEM;

	INIT_LIST_HEAD(&ipucsi->capture);
	spin_lock_init(&ipucsi->lock);
	mutex_init(&ipucsi->mutex);

	init_completion(&ipucsi->vdev_released);

	ipucsi->csi_id = pdata->csi;
	ipucsi->csi = ipu_csi_get(ipu, pdata->csi);
	if (IS_ERR(ipucsi->csi)) {
		ret = PTR_ERR(ipucsi->csi);
		dev_warn(&pdev->dev, "failed to get CSI#%d: %d\n",
			 pdata->csi, ret);
		goto failed;
	}

	ipucsi->smfc = ipu_smfc_get(ipu, pdata->dma[0]);
	if (IS_ERR(ipucsi->smfc)) {
		ret = PTR_ERR(ipucsi->smfc);
		dev_warn(&pdev->dev, "failed to get SMFC#%d: %d\n",
			 pdata->dma[0], ret);
		goto failed;
	}

	ipucsi->ipu = ipu;
	ipucsi->dev = &pdev->dev;
	ipucsi->v4l2_dev = ipu_media_get_v4l2_dev();

	if (!ipucsi->v4l2_dev) {
		ret = -EPROBE_DEFER;
		goto failed;
	}

	ipucsi->v4l2_dev->notify = ipucsi_v4l2_dev_notify;

	node = ipucsi_get_port(pdev->dev.parent->of_node, pdata->csi);
	if (!node) {
		dev_err(&pdev->dev, "cannot find node port@%d\n", pdata->csi);
		ret = -ENODEV;
		goto failed;
	}

	ret = of_alias_get_id(node->parent, "ipu");
	if (ret < 0) {
		dev_err(ipucsi->dev, "missing alias for parent ipu\n");
		ret = -EINVAL;
		goto failed;
	}

	snprintf(ipucsi->name, sizeof ipucsi->name, "ipu%u-csi%u",
		 ret, ipucsi->csi_id);

	ipucsi->ipuch = ipu_idmac_get(ipu, pdata->dma[0]);
	if (!ipucsi->ipuch) {
		ret = -EBUSY;
		goto failed;
	}

	ret = ipucsi_video_device_init(pdev, ipucsi);
	if (ret)
		goto failed;

	ret = ipucsi_create_controls(ipucsi);
	if (ret)
		goto failed;

	ret = ipucsi_vb2_init(ipucsi);
	if (ret)
		goto failed;

	ret = ipucsi_subdev_init(ipucsi, node);
	if (ret)
		goto failed;

	ret = ipucsi_async_init(ipucsi, node);
	if (ret)
		goto failed;

	platform_set_drvdata(pdev, ipucsi);

	ret = video_register_device(&ipucsi->vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto failed;

	ret = media_entity_create_link(&ipucsi->subdev.entity, 1,
			&ipucsi->vdev.entity, 0,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0)
		goto failed_video;

	of_node_put(node);
	dev_info(&pdev->dev, "loaded\n");

	return 0;

	/* TODO: remove subdev->vdev link */
failed_video:

	video_unregister_device(&ipucsi->vdev);
failed:
	media_entity_remove_links(&ipucsi->subdev.entity);
	ipu_media_put_v4l2_dev(ipucsi->v4l2_dev);
	of_node_put(node);
	v4l2_ctrl_handler_free(&ipucsi->ctrls);
	if (ipucsi->link)
		ipu_media_entity_remove_link(ipucsi->link);
	if (ipucsi->vdev.entity.links)
		media_entity_cleanup(&ipucsi->vdev.entity);
	if (ipucsi->alloc_ctx)
		vb2_dma_contig_cleanup_ctx(ipucsi->alloc_ctx);
	if (ipucsi->ipuch)
		ipu_idmac_put(ipucsi->ipuch);

	if (!IS_ERR_OR_NULL(ipucsi->smfc))
		ipu_smfc_put(ipucsi->smfc);

	if (!IS_ERR_OR_NULL(ipucsi->csi))
		ipu_csi_put(ipucsi->csi);

	return ret;
}

static int ipucsi_remove(struct platform_device *pdev)
{
	struct ipucsi *ipucsi = platform_get_drvdata(pdev);
	int rc;

	if (ipucsi->link)
		/* can be NULL for unused CSI devices */
		ipu_media_entity_remove_link(ipucsi->link);
	v4l2_device_unregister_subdev(&ipucsi->subdev);
	media_entity_cleanup(&ipucsi->subdev.entity);

	media_entity_remove_links(&ipucsi->vdev.entity);
	media_entity_cleanup(&ipucsi->vdev.entity);
	media_device_unregister_entity(&ipucsi->vdev.entity);
	video_unregister_device(&ipucsi->vdev);

	vb2_queue_release(&ipucsi->vb2_vidq);
	vb2_dma_contig_cleanup_ctx(ipucsi->alloc_ctx);
	ipu_idmac_put(ipucsi->ipuch);
	v4l2_ctrl_handler_free(&ipucsi->ctrls);

	ipu_media_put_v4l2_dev(ipucsi->v4l2_dev);
	ipu_smfc_put(ipucsi->smfc);
	ipu_csi_put(ipucsi->csi);

	mutex_destroy(&ipucsi->mutex);

	rc = wait_for_completion_killable(&ipucsi->vdev_released);

	return rc;
}

static struct platform_driver ipucsi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
	.probe = ipucsi_probe,
	.remove = ipucsi_remove,
};

static int __init ipucsi_init(void)
{
	return platform_driver_register(&ipucsi_driver);
}

static void __exit ipucsi_exit(void)
{
	return platform_driver_unregister(&ipucsi_driver);
}

subsys_initcall(ipucsi_init);
module_exit(ipucsi_exit);

MODULE_DESCRIPTION("i.MX51/53 IPUv3 Camera interface driver");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
