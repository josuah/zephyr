/*
 * SPDX-FileCopyrightText: Copyright Panoramix Labs
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_dvp2axi

#include <zephyr/drivers/otp.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/irq.h>
#include <zephyr/video/video.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bflb_dvp2axi, CONFIG_VIDEO_LOG_LEVEL);

#include <bouffalolab/common/cam_reg.h>
#include <bflb_soc.h>
#include <glb_reg.h>

#include "video_common.h"

#define CAM_REG_DVP_DATA_MODE_NONE		0
#define CAM_REG_DVP_DATA_MODE_888_TO_565	2
#define CAM_REG_DVP_DATA_MODE_888_TO_888X	3
#define CAM_REG_DVP_DATA_MODE_YUV		4

struct bflb_dvp2axi_config {
	uintptr_t base;
	const struct device *source_dev;
	void (*irq_config_func)(const struct device *dev);
	uint8_t axi_burst_length;
	uint8_t axi_data_width;
	uint8_t hsync_active : 1;
	uint8_t vsync_active : 1;
};

struct bflb_dvp2axi_data {
	struct video_format_cap fmts[CONFIG_VIDEO_BFLB_DVP2AXI_MAX_FORMATS];
	size_t num_fmts;
	struct video_format fmt;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct video_buffer *active_vbuf;
	bool is_streaming;
};

static void bflb_dvp2axi_add_format_cap(const struct device *dev,
					  const struct video_format_cap *fmt_cap)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;

	LOG_DBG("%s's format %s [%ux%u - %ux%u] not supported, skipping",
		config->source_dev->name,
		VIDEO_FOURCC_TO_STR(fmt_cap->pixelformat),
		fmt_cap->width_min, fmt_cap->height_min,
		fmt_cap->width_max, fmt_cap->height_max);

	if (data->num_fmts + 1 >= CONFIG_VIDEO_BFLB_DVP2AXI_MAX_FORMATS) {
		LOG_WRN("CONFIG_VIDEO_BFLB_DVP2AXI_MAX_FORMATS too small, raise above %u",
			CONFIG_VIDEO_BFLB_DVP2AXI_MAX_FORMATS);
		return;
	}

	data->fmts[data->num_fmts] = *fmt_cap;
	data->num_fmts++;
}

static int bflb_dvp2axi_get_caps(const struct device *dev, struct video_caps *caps)
{
	struct bflb_dvp2axi_data *data = dev->data;

	caps->format_caps = data->fmts;
	caps->min_vbuf_count = 1;
	caps->buf_align = 16;

	return 0;
}

static int bflb_dvp2axi_set_format(const struct device *dev, struct video_format *fmt)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;
	size_t fmt_idx;
	int ret;

	ret = video_format_caps_index(data->fmts, fmt, &fmt_idx);
	if (ret < 0) {
		LOG_ERR("Format %s %ux%u unsupported by %s",
			VIDEO_FOURCC_TO_STR(fmt->pixelformat), fmt->width, fmt->height, dev->name);
		return ret;
	}

	ret = video_set_format(config->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		return ret;
	}

	data->fmt = *fmt;

	return 0;
}

static int bflb_dvp2axi_get_format(const struct device *dev, struct video_format *fmt)
{
	const struct bflb_dvp2axi_config *config = dev->config;

	return video_get_format(config->source_dev, fmt);
}

static int bflb_dvp2axi_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct bflb_dvp2axi_config *config = dev->config;

	return video_set_frmival(config->source_dev, frmival);
}

static int bflb_dvp2axi_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct bflb_dvp2axi_config *config = dev->config;

	return video_get_frmival(config->source_dev, frmival);
}

static int bflb_dvp2axi_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	const struct bflb_dvp2axi_config *config = dev->config;

	return video_enum_frmival(config->source_dev, fie);
}

static void bflb_dvp2axi_trigger(const struct device *dev)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;
	uint32_t tmp;
	int key;

	key = k_irq_lock();

	if (data->fmt.pixelformat == 0 || data->fmt.width == 0 || data->fmt.height == 0)  {
		LOG_ERR("Format not conifgured");
		return;
	}

	if (data->active_vbuf != NULL) {
		LOG_DBG("Already busy with %p, skipping", data->active_vbuf->buffer);
		goto end;
	}

	data->active_vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
	if (data->active_vbuf == NULL) {
		LOG_DBG("No buffer submitted yet");
		goto end;
	}

	LOG_DBG("Submitting new buffer %p, size %u",
		data->active_vbuf->buffer, data->active_vbuf->size);

	tmp = (uintptr_t)data->active_vbuf->buffer;
	sys_write32(tmp, config->base + CAM_DVP2AXI_ADDR_START_OFFSET);

	tmp = data->active_vbuf->size
		>> (config->axi_burst_length / (config->axi_data_width / BITS_PER_BYTE));
	sys_write32(tmp, config->base + CAM_DVP2AXI_MEM_BCNT_OFFSET);

end:
	k_irq_unlock(key);
}

static void bflb_dvp2axi_detrigger(const struct device *dev)
{
	/* Trigger cancelation, error irq pass buffer to completion queue */
}

static int bflb_dvp2axi_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct bflb_dvp2axi_data *data = dev->data;

	k_fifo_put(&data->fifo_in, vbuf);

	bflb_dvp2axi_trigger(dev);

	return 0;
}

static int bflb_dvp2axi_dequeue(const struct device *dev, struct video_buffer **vbuf,
				  k_timeout_t timeout)
{
	struct bflb_dvp2axi_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		LOG_ERR("Failed to retreive a buffer from %s FIFO", dev->name);
		return -ETIMEDOUT;
	}

	return 0;
}

static int bflb_dvp2axi_flush(const struct device *dev, bool cancel)
{
	return 0;
}

static int bflb_dvp2axi_apply_format(const struct device *dev)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;
	struct video_format source_fmt = {.type = VIDEO_BUF_TYPE_OUTPUT};
	struct video_caps caps = {.type = VIDEO_BUF_TYPE_OUTPUT};
	uint32_t data_mode;
	size_t fmt_idx;
	uint32_t tmp;
	int ret;

	sys_write32((data->fmt.height << 16) | (data->fmt.width << 0),
		     config->base + CAM_DVP2AXI_FRAM_EXM_OFFSET);

	ret = video_get_format(config->source_dev, &source_fmt);
	if (ret < 0) {
		LOG_ERR("Faield to get source %s format", config->source_dev->name);
		return ret;
	}

	/* first try without conversion */
	ret = video_get_caps(config->source_dev, &caps);
	if (ret < 0) {
		LOG_ERR("Failed to get %s capabilities", config->source_dev->name);
		return ret;
	}

	ret = video_format_caps_index(data->fmts, &data->fmt, &fmt_idx);
	if (ret < 0) {
		/* Format not found, try to convert */

		switch (data->fmt.pixelformat) {

		case VIDEO_PIX_FMT_RGB565:
			source_fmt.pixelformat = VIDEO_PIX_FMT_RGB24;
			data_mode = CAM_REG_DVP_DATA_MODE_888_TO_565;

			tmp = sys_read32(config->base + CAM_DVP2AXI_MISC_OFFSET);
			tmp &= ~CAM_REG_FORMAT_565_MASK;
			tmp |= 5 << CAM_REG_FORMAT_565_SHIFT;
			sys_write32(tmp, config->base + CAM_DVP2AXI_MISC_OFFSET);
			break;
#if 0
		case VIDEO_PIX_FMT_BGR565:
			source_fmt.pixelformat = VIDEO_PIX_FMT_RGB24;
			data_mode = CAM_REG_DVP_DATA_MODE_888_TO_565;

			tmp = sys_read32(config->base + CAM_DVP2AXI_MISC_OFFSET);
			tmp &= ~CAM_REG_FORMAT_565_MASK;
			sys_write32(tmp, config->base + CAM_DVP2AXI_MISC_OFFSET);
			break;
#endif
		case VIDEO_PIX_FMT_RGBA32:
			source_fmt.pixelformat = VIDEO_PIX_FMT_RGB24;
			data_mode = CAM_REG_DVP_DATA_MODE_888_TO_888X;
			break;

		case VIDEO_PIX_FMT_GREY:
			source_fmt.pixelformat = VIDEO_PIX_FMT_YVYU;
			data_mode = CAM_REG_DVP_DATA_MODE_YUV;
			break;

		/* TODO: VIDEO_PIX_FMT_UV */
			break;

		default:
			LOG_WRN("Unsupported conversion, usingunmodified input format %s",
				VIDEO_FOURCC_TO_STR(data->fmt.pixelformat));
			source_fmt.pixelformat = data->fmt.pixelformat;
			data_mode = CAM_REG_DVP_DATA_MODE_NONE;
			break;
		}
	} else {
		LOG_INF("No conversion needed, using %s directly",
			VIDEO_FOURCC_TO_STR(data->fmt.pixelformat));
		data_mode = CAM_REG_DVP_DATA_MODE_NONE;
	}

	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp &= ~CAM_REG_DVP_DATA_MODE_MASK;
	tmp |= data_mode << CAM_REG_DVP_DATA_MODE_SHIFT;
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	ret = video_set_format(dev, &source_fmt);
	if (ret < 0) {
		LOG_ERR("Failed to setup %s to format %s %ux%u",
			config->source_dev->name, VIDEO_FOURCC_TO_STR(source_fmt.pixelformat),
			source_fmt.width, source_fmt.height);
		return ret;
	}

	return 0;
}

static int bflb_dvp2axi_set_stream(const struct device *dev, bool stream, enum video_buf_type type)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;
	int ret;

	if (type != VIDEO_BUF_TYPE_OUTPUT) {
		LOG_ERR("Supporting output buffer type only");
		return -EINVAL;
	}

	if (data->is_streaming == stream) {
		return 0;
	}

	if (stream) {
		ret = bflb_dvp2axi_apply_format(dev);
		if (ret < 0) {
			return ret;
		}

		ret = video_stream_start(config->source_dev, type);
		if (ret < 0) {
			LOG_ERR("Failed to start source device %s", config->source_dev->name);
			return ret;
		}

		bflb_dvp2axi_trigger(dev);
	} else {
		ret = video_stream_stop(config->source_dev, type);
		if (ret < 0) {
			LOG_ERR("Failed to start source device %s", config->source_dev->name);
			return ret;
		}

		bflb_dvp2axi_detrigger(dev);
	}

	data->is_streaming = stream;

	return 0;
}

static int bflb_dvp2axi_set_selection(const struct device *dev, struct video_selection *sel)
{
	return 0;
}

static int bflb_dvp2axi_get_selection(const struct device *dev, struct video_selection *sel)
{
	return 0;
}

static const void bflb_dvp2axi_isr(const void *p)
{
	const struct device *dev = p;
	struct bflb_dvp2axi_data *data = dev->data;

	if (data->active_vbuf == NULL) {
		LOG_ERR("%s got ISR without active buffer", dev->name);
		return;
	}

	k_fifo_put(&data->fifo_out, data->active_vbuf);
	data->active_vbuf = NULL;

	bflb_dvp2axi_trigger(dev);
}

static int bflb_dvp2axi_init(const struct device *dev)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;
	struct video_format fmt = {};
	struct video_caps caps = {.type = VIDEO_BUF_TYPE_OUTPUT};
	uint32_t tmp;
	int ret;

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);

	/* axi-burst-length */
	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp &= ~CAM_REG_XLEN_MASK;
	switch (config->axi_burst_length) {
	case 1:
		tmp |= 0 << CAM_REG_XLEN_SHIFT;
		break;
	case 4:
		tmp |= 1 << CAM_REG_XLEN_SHIFT;
		break;
	case 8:
		tmp |= 2 << CAM_REG_XLEN_SHIFT;
		break;
	case 16:
		tmp |= 3 << CAM_REG_XLEN_SHIFT;
		break;
	case 32:
		tmp |= 5 << CAM_REG_XLEN_SHIFT;
		break;
	case 64:
		tmp |= 6 << CAM_REG_XLEN_SHIFT;
		break;
	default:
		LOG_ERR("Invalid AXI burst length size for %s", dev->name);
		return -EINVAL;
	}
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	/* vsync-active high/low */
	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	if (config->vsync_active) {
		tmp |= CAM_REG_FRAM_VLD_POL;
	} else {
		tmp &= ~CAM_REG_FRAM_VLD_POL;
	}
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	/* hsync-active high/low */
	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	if (config->hsync_active) {
		tmp |= CAM_REG_LINE_VLD_POL;
	} else {
		tmp &= ~CAM_REG_LINE_VLD_POL;
	}
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	/* generate format caps */
	ret = video_get_caps(config->source_dev, &caps);
	if (ret != 0) {
		LOG_ERR("Failed to get %s capabilities", config->source_dev->name);
		return ret;
	}
	for (size_t i = 0; caps.format_caps[i].pixelformat != 0; i++) {
		bflb_dvp2axi_add_format_cap(dev, &caps.format_caps[i]);
	}

	/* set default format */
	fmt.pixelformat = caps.format_caps[0].pixelformat,
	fmt.width = caps.format_caps[0].width_min,
	fmt.height = caps.format_caps[0].height_min,
	ret = video_set_format(dev, &fmt);
	if (ret != 0) {
		LOG_ERR("Failed to set default format to %s %ux%u",
			VIDEO_FOURCC_TO_STR(fmt.pixelformat), fmt.width, fmt.height);
		return ret;
	}

	/* enable */
	config->irq_config_func(dev);

	return 0;
}

#if CONFIG_DEVICE_DEINIT_SUPPORT
static const int bflb_dvp2axi_deinit(const struct device *dev)
{
	return 0;
}
#endif

static DEVICE_API(video, bflb_dvp2axi_api) = {
	.set_format = bflb_dvp2axi_set_format,
	.get_format = bflb_dvp2axi_get_format,
	.set_stream = bflb_dvp2axi_set_stream,
	.get_caps = bflb_dvp2axi_get_caps,
	.enqueue = bflb_dvp2axi_enqueue,
	.dequeue = bflb_dvp2axi_dequeue,
	.flush = bflb_dvp2axi_flush,
	.set_frmival = bflb_dvp2axi_set_frmival,
	.get_frmival = bflb_dvp2axi_get_frmival,
	.enum_frmival = bflb_dvp2axi_enum_frmival,
	.set_selection = bflb_dvp2axi_set_selection,
	.get_selection = bflb_dvp2axi_get_selection,
};

#define SOURCE_NODE(n) \
	DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(n, 0, 0))

#define SOURCE_LINK_PROP(n, prop) \
	DT_PROP(DT_CHILD(DT_CHILD(SOURCE_NODE(n), port), endpoint_0), prop)

#define BFLB_DVP2AXI_INIT(n)									\
	static void bflb_dvp2axi_irq_config_##n(const struct device *dev)			\
	{											\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),				\
				bflb_dvp2axi_isr, DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));							\
	}											\
												\
	struct bflb_dvp2axi_data bflb_dvp2axi_data_##n = {					\
	};											\
												\
	const struct bflb_dvp2axi_config bflb_dvp2axi_config_##n = {				\
		.base = DT_INST_REG_ADDR(n),							\
		.irq_config_func = &bflb_dvp2axi_irq_config_##n,				\
		.source_dev = DEVICE_DT_GET(SOURCE_NODE(n)),					\
		.axi_burst_length = DT_INST_PROP(n, axi_burst_length),				\
		.axi_data_width = DT_INST_PROP(n, axi_data_width),				\
		.hsync_active = SOURCE_LINK_PROP(n, hsync_active),				\
		.vsync_active = SOURCE_LINK_PROP(n, vsync_active),				\
	};											\
												\
	DEVICE_DT_INST_DEINIT_DEFINE(n, &bflb_dvp2axi_init, &bflb_dvp2axi_deinit, NULL,		\
					 &bflb_dvp2axi_data_##n, &bflb_dvp2axi_config_##n,	\
					 POST_KERNEL, CONFIG_VIDEO_BFLB_DVP2AXI_INIT_PRIORITY,	\
					 &bflb_dvp2axi_api);					\
												\
	VIDEO_DEVICE_DEFINE(dvp2axi##n, DEVICE_DT_INST_GET(n), DEVICE_DT_GET(SOURCE_NODE(n)));

DT_INST_FOREACH_STATUS_OKAY(BFLB_DVP2AXI_INIT)

#if 0
	case CAM_CMD_SET_RGBA8888_ALPHA:
		/* Set alpha value of RGBA8888 output, arg is alpha */
		tmp = sys_read32(config->base + CAM_DVP2AXI_MISC_OFFSET);
		tmp &= ~CAM_REG_ALPHA_MASK;
		tmp |= arg & CAM_REG_ALPHA_MASK;
		sys_write32(tmp, config->base + CAM_DVP2AXI_MISC_OFFSET);
		break;

	case CAM_CMD_COUNT_TRIGGER_NORMAL_INT:
		/* Set frame count to trigger normal interrupt, arg is frame count */
		tmp = sys_read32(config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
		tmp &= ~CAM_REG_FRAME_CNT_TRGR_INT_MASK;
		tmp |= arg & CAM_REG_FRAME_CNT_TRGR_INT_MASK;
		sys_write32(tmp, config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
		break;

	case CAM_CMD_FRAME_FILTER:
		/* For example: frame_count is 4, frame_valid is 0x14 (10100b).
		 * Third/fifth frame will be retained,First/second/fourth frame will be dropped in every (4 + 1) frames
		 */
		struct bflb_cam_frame_filter_config_s *config = (struct bflb_cam_frame_filter_config_s *)arg;
		sys_write32(config->frame_count, config->base + CAM_DVP2AXI_FRAME_PERIOD_OFFSET);
		sys_write32(config->frame_valid, config->base + CAM_DVP2AXI_FRAME_VLD_OFFSET);
		break;

	bflb_cam_int_mask(dev, CAM_INTMASK_FIFO_OVERWRITE, true);
#endif
