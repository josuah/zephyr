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

#define CAM_INTCLR_NORMAL			(1 << 4)
#define CAM_INTCLR_MEMORY_OVERWRITE		(1 << 5)
#define CAM_INTCLR_FRAME_OVERWRITE		(1 << 6)
#define CAM_INTCLR_FIFO_OVERWRITEK		(1 << 7)
#define CAM_INTCLR_HSYNC_MISMATCH		(1 << 8)
#define CAM_INTCLR_VSYNC_MISMATCH		(1 << 9)

#define CAM_INTMASK_HSYNC_MISMATCH		(1 << 6)
#define CAM_INTMASK_VSYNC_MISMATCH		(1 << 7)
#define CAM_INTMASK_NORMAL			(1 << 8)
#define CAM_INTMASK_MEMORY_OVERWRITE		(1 << 9)
#define CAM_INTMASK_FRAME_OVERWRITE 		(1 << 10)
#define CAM_INTMASK_FIFO_OVERWRITE		(1 << 11)

#define CAM_POLARITY_ACTIVE_LOW			0
#define CAM_POLARITY_ACTIVE_HIGH		1

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
	caps->buf_align = 32;

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
		LOG_DBG("Already busy with %p, skipping", (void *)data->active_vbuf->buffer);
		goto end;
	}

	data->active_vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
	if (data->active_vbuf == NULL) {
		LOG_DBG("No buffer submitted yet");
		goto end;
	}

	LOG_DBG("Submitting new buffer %p, size %u, format %s %ux%u",
		data->active_vbuf->buffer, data->active_vbuf->size,
		VIDEO_FOURCC_TO_STR(data->fmt.pixelformat), data->fmt.width, data->fmt.height);

	tmp = (uintptr_t)data->active_vbuf->buffer;
	sys_write32(tmp, config->base + CAM_DVP2AXI_ADDR_START_OFFSET);

	/* BCNT is AXI burst count */
	tmp = (data->active_vbuf->size >> config->axi_burst_length)
		/ (config->axi_data_width / BITS_PER_BYTE);
	sys_write32(tmp, config->base + CAM_DVP2AXI_MEM_BCNT_OFFSET);

	/* BCNT is byte count */
	tmp = data->active_vbuf->size;
	sys_write32(tmp, config->base + CAM_DVP2AXI_FRAME_BCNT_OFFSET);

	/* Start the DVP engine now that it is fully configured */
	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp |= CAM_REG_DVP_ENABLE;
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
end:
	k_irq_unlock(key);
}

static void bflb_dvp2axi_detrigger(const struct device *dev)
{
	const struct dvp2axi_config *config = dev->config;
	uint32_t tmp;

	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp &= ~CAM_REG_DVP_ENABLE;
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
}

static int bflb_dvp2axi_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct bflb_dvp2axi_data *data = dev->data;

	memset(vbuf->buffer, 0x00, vbuf->size);

	k_fifo_put(&data->fifo_in, vbuf);

	if (data->is_streaming) {
		bflb_dvp2axi_trigger(dev);
	}

	return 0;
}

static uint8_t dvp2axi_frame_count(const struct device *dev)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	uint32_t tmp;

	tmp = sys_read32(config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
	return (tmp & CAM_FRAME_VALID_CNT_MASK) >> CAM_FRAME_VALID_CNT_SHIFT;
}

static int bflb_dvp2axi_dequeue(const struct device *dev, struct video_buffer **vbuf,
				  k_timeout_t timeout)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;
	uint32_t tmp;

	k_sleep(K_MSEC(500));

	tmp = sys_read32(config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);

	LOG_DBG("Waiting a completed buffer: status/error 0x%08x, frame_count %u",
		tmp, dvp2axi_frame_count(dev));

	LOG_DBG("Dumping %p of size %u",
		(void *)data->active_vbuf->buffer, data->active_vbuf->size);
	LOG_HEXDUMP_DBG(data->active_vbuf->buffer, 128, "active_vbuf");

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		LOG_ERR("Failed to retreive a buffer from %s FIFO", dev->name);
		return -ETIMEDOUT;
	}

	LOG_DBG("Buffer completed");

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

	sys_write32(data->fmt.height << 16 | data->fmt.width << 0,
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
		//case VIDEO_PIX_FMT_BGR565:
			source_fmt.pixelformat = VIDEO_PIX_FMT_RGB24;
			data_mode = CAM_REG_DVP_DATA_MODE_888_TO_565;

			tmp = sys_read32(config->base + CAM_DVP2AXI_MISC_OFFSET);
			tmp &= ~CAM_REG_FORMAT_565_MASK;
			if (data->fmt.pixelformat == VIDEO_PIX_FMT_RGB565) {
				tmp |= 5 << CAM_REG_FORMAT_565_SHIFT;
			}
			sys_write32(tmp, config->base + CAM_DVP2AXI_MISC_OFFSET);
			break;

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

	ret = video_set_format(dev, &source_fmt);
	if (ret < 0) {
		LOG_ERR("Failed to setup %s to format %s %ux%u",
			config->source_dev->name, VIDEO_FOURCC_TO_STR(source_fmt.pixelformat),
			source_fmt.width, source_fmt.height);
		return ret;
	}

	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp &= ~CAM_REG_DVP_DATA_MODE_MASK;
	tmp |= data_mode << CAM_REG_DVP_DATA_MODE_SHIFT;
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	return 0;
}

static int bflb_dvp2axi_set_stream(const struct device *dev, bool stream, enum video_buf_type type)
{
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;\
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

		bflb_dvp2axi_trigger(dev);

		ret = video_stream_start(config->source_dev, type);
		if (ret < 0) {
			LOG_ERR("Failed to start source device %s", config->source_dev->name);
			return ret;
		}
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
	const struct bflb_dvp2axi_config *config = dev->config;
	struct bflb_dvp2axi_data *data = dev->data;

	bflb_dvp2axi_detrigger(dev);

	LOG_INF("ISR");

	if (data->active_vbuf == NULL) {
		LOG_ERR("%s got ISR without active buffer", dev->name);
		return;
	}

	sys_write32(CAM_INTCLR_NORMAL, config->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);

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

	LOG_DBG("Initializing %s", dev->name);

	if (!device_is_ready(config->source_dev)) {
		LOG_ERR("%s source %s is not ready", dev->name, config->source_dev->name);
		return -ENODEV;
	}

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);

	/* generate caps */
	ret = video_get_caps(config->source_dev, &caps);
	if (ret != 0) {
		LOG_ERR("Failed to get %s capabilities", config->source_dev->name);
		return ret;
	}
	for (size_t i = 0; caps.format_caps[i].pixelformat != 0; i++) {
		bflb_dvp2axi_add_format_cap(dev, &caps.format_caps[i]);
	}

	/* default format */
	fmt.pixelformat = caps.format_caps[0].pixelformat,
	fmt.width = caps.format_caps[0].width_min,
	fmt.height = caps.format_caps[0].height_min,
	ret = video_set_format(dev, &fmt);
	if (ret != 0) {
		LOG_ERR("Failed to set default format to %s %ux%u",
			VIDEO_FOURCC_TO_STR(fmt.pixelformat), fmt.width, fmt.height);
		return ret;
	}

	sys_write32(0, config->base + CAM_DVP_DEBUG_OFFSET);

	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp &= ~(CAM_REG_DROP_EN | CAM_REG_DROP_EVEN | CAM_REG_DVP_DATA_MODE_MASK
		| CAM_REG_DVP_DATA_BSEL | CAM_REG_V_SUBSAMPLE_EN | CAM_REG_V_SUBSAMPLE_POL
 		); // | CAM_REG_XLEN_MASK | CAM_REG_LINE_VLD_POL | CAM_REG_FRAM_VLD_POL
	tmp |= CAM_REG_SW_MODE; /* TODO: use wrap/continuous mode for JPEG? */
#if 0
	tmp |=	(config->axi_burst_length == 1)  ? (0 << CAM_REG_XLEN_SHIFT) :
		(config->axi_burst_length == 4)  ? (1 << CAM_REG_XLEN_SHIFT) :
		(config->axi_burst_length == 8)  ? (2 << CAM_REG_XLEN_SHIFT) :
		(config->axi_burst_length == 16) ? (3 << CAM_REG_XLEN_SHIFT) :
		(config->axi_burst_length == 32) ? (5 << CAM_REG_XLEN_SHIFT) :
		(config->axi_burst_length == 64) ? (6 << CAM_REG_XLEN_SHIFT) : 0;
#endif
	//tmp |= config->vsync_active ? CAM_REG_FRAM_VLD_POL : 0;
	//tmp |= config->hsync_active ? CAM_REG_LINE_VLD_POL : 0;
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	tmp = sys_read32(config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
	//tmp &= ~CAM_REG_FRAME_CNT_TRGR_INT_MASK;
	//tmp |= num & CAM_REG_FRAME_CNT_TRGR_INT_MASK;
	//tmp |= CAM_REG_INT_HCNT_EN;
	tmp |= CAM_INTMASK_NORMAL;
	tmp |= CAM_INTMASK_MEMORY_OVERWRITE;
	tmp |= CAM_INTMASK_FRAME_OVERWRITE;
	tmp &= ~CAM_INTMASK_FIFO_OVERWRITE;
	tmp |= CAM_INTMASK_HSYNC_MISMATCH;
	tmp |= CAM_INTMASK_VSYNC_MISMATCH;
	sys_write32(tmp, config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);

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
void bflb_cam_init(struct bflb_device_s *dev, const struct bflb_cam_config_s *config)
{
	const struct dvp2axi_config *config = dev->config;
	uint8_t data_mode = 0;
	uint16_t resolution_x, resolution_y;
	uint32_t frame_size;
	uint32_t tmp;

	sys_write32(config->output_bufaddr, config->base + CAM_DVP2AXI_ADDR_START_OFFSET);
	sys_write32(config->resolution_y << 16 | config->resolution_x, config->base + CAM_DVP2AXI_FRAM_EXM_OFFSET);
	sys_write32(data_mode, config->base + CAM_DVP_DEBUG_OFFSET);

	tmp = bflb_clk_get_peripheral_clock(BFLB_DEVICE_TYPE_CAMERA, 0) / 1000000;
	if (tmp == 0) {
		tmp = 80;
	}

	/* Set output format */
	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp |= CAM_REG_SW_MODE;
	tmp &= ~(CAM_REG_DROP_EN | CAM_REG_DROP_EVEN | CAM_REG_DVP_DATA_MODE_MASK |
		CAM_REG_DVP_DATA_BSEL | CAM_REG_V_SUBSAMPLE_EN | CAM_REG_V_SUBSAMPLE_POL);
	frame_size = resolution_x * resolution_y * 2;
	sys_write32(frame_size, config->base + CAM_DVP2AXI_FRAME_BCNT_OFFSET);

	/* Input RGB565/BGR565, output AUTO */
	tmp |= 0 << CAM_REG_DVP_DATA_MODE_SHIFT;
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);

	/* Set output buffer burst count */
	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp = (tmp & CAM_REG_XLEN_MASK) >> CAM_REG_XLEN_SHIFT;
	switch (tmp) {
		case CAM_BURST_INCR1:
			tmp = config->output_bufsize >> 3;
			break;
		case CAM_BURST_INCR4:
			tmp = config->output_bufsize >> 5;
			break;
		case CAM_BURST_INCR8:
			tmp = config->output_bufsize >> 6;
			break;
		case CAM_BURST_INCR16:
			tmp = config->output_bufsize >> 7;
			break;
		case CAM_BURST_INCR32:
			tmp = config->output_bufsize >> 8;
			break;
		case CAM_BURST_INCR64:
			tmp = config->output_bufsize >> 9;
			break;
		default:
			tmp = config->output_bufsize >> 7;
			break;
	}
	sys_write32(tmp, config->base + CAM_DVP2AXI_MEM_BCNT_OFFSET);

	bflb_cam_int_mask(dev, CAM_INTMASK_FIFO_OVERWRITE, true);
}

void bflb_cam_start(struct bflb_device_s *dev)
{
	const struct dvp2axi_config *config = dev->config;
	uint32_t tmp;

	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp |= CAM_REG_DVP_ENABLE;
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
}

void bflb_cam_stop(struct bflb_device_s *dev)
{
	const struct dvp2axi_config *config = dev->config;
	uint32_t tmp;

	tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
	tmp &= ~CAM_REG_DVP_ENABLE;
	sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
}

void bflb_cam_int_mask(struct bflb_device_s *dev, uint32_t int_type, bool mask)
{
	const struct dvp2axi_config *config = dev->config;
	uint32_t tmp;

	tmp = sys_read32(config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
	if (mask) {
		tmp &= ~int_type;
	} else {
		tmp |= int_type;
	}
	sys_write32(tmp, config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
}

void bflb_cam_int_clear(struct bflb_device_s *dev, uint32_t int_type)
{
	sys_write32(int_type, dev->config->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);
}

void bflb_cam_crop_vsync(struct bflb_device_s *dev, uint16_t start_line, uint16_t end_line)
{
	/* Get start_line ~ (end_line - 1), not include end_line */
	sys_write32(start_line << 16 | end_line, dev->config->base + CAM_DVP2AXI_VSYNC_CROP_OFFSET);
}

void bflb_cam_crop_hsync(struct bflb_device_s *dev, uint16_t start_pixel, uint16_t end_pixel)
{
	/* Get start_pixel ~ (end_pixel - 1), not include end_pixel */
	sys_write32(start_pixel << 16 | end_pixel, dev->config->base + CAM_DVP2AXI_HSYNC_CROP_OFFSET);
}

void bflb_cam_pop_one_frame(struct bflb_device_s *dev)
{
	sys_write32(1, dev->config->base + CAM_DVP_FRAME_FIFO_POP_OFFSET);
}

uint8_t bflb_cam_get_frame_count(struct bflb_device_s *dev)
{
	const struct dvp2axi_config *config = dev->config;
	uint32_t tmp;

	tmp = sys_read32(config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
	tmp &= CAM_FRAME_VALID_CNT_MASK;
	return (tmp >> CAM_FRAME_VALID_CNT_SHIFT);
}

uint32_t bflb_cam_get_frame_info(struct bflb_device_s *dev, uint8_t **pic)
{
	const struct dvp2axi_config *config = dev->config;

	*pic = (uint8_t *)(uintptr_t)sys_read32(config->base + CAM_FRAME_START_ADDR0_OFFSET);
	return (sys_read32(config->base + CAM_DVP2AXI_FRAME_BCNT_OFFSET));
}

uint32_t bflb_cam_get_intstatus(struct bflb_device_s *dev)
{
	return (sys_read32(dev->config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET));
}

int bflb_cam_feature_control(struct bflb_device_s *dev, int cmd, size_t arg)
{
	int ret = 0;
	const struct dvp2axi_config *config = dev->config;
	uint32_t tmp;

	switch (cmd) {
		case CAM_CMD_SET_VSYNC_POLARITY:
			/* Set vsync polarity, arg use @ref CAM_POLARITY */
			tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
			if (arg) {
				tmp |= CAM_REG_FRAM_VLD_POL;
			} else {
				tmp &= ~CAM_REG_FRAM_VLD_POL;
			}
			sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
			break;

		case CAM_CMD_SET_HSYNC_POLARITY:
			/* Set hsync polarity, arg use @ref CAM_POLARITY */
			tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
			if (arg) {
				tmp |= CAM_REG_LINE_VLD_POL;
			} else {
				tmp &= ~CAM_REG_LINE_VLD_POL;
			}
			sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
			break;

		case CAM_CMD_SET_BURST:
			/* Set burst length, arg use @ref CAM_BURST */
			tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
			tmp &= ~CAM_REG_XLEN_MASK;
			tmp |= (arg << CAM_REG_XLEN_SHIFT) & CAM_REG_XLEN_MASK;
			sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
			break;

		case CAM_CMD_GET_FRAME_ID:
			/* Get frame id */
			*(uint16_t *)arg = sys_read32(config->base + CAM_FRAME_ID_STS01_OFFSET) & 0xffff;
			break;

		case CAM_CMD_WRAP_MODE:
			/* Wrap to output buffer start address, only effective in mjpeg mode, arg use ENABLE or DISABLE */
			tmp = sys_read32(config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
			if (arg) {
				tmp |= CAM_REG_HW_MODE_FWRAP;
			} else {
				tmp &= ~CAM_REG_HW_MODE_FWRAP;
			}
			sys_write32(tmp, config->base + CAM_DVP2AXI_CONFIGUE_OFFSET);
			break;

		case CAM_CMD_COUNT_TRIGGER_NORMAL_INT:
			/* Set frame count to trigger normal interrupt, arg is frame count */
			tmp = sys_read32(config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
			tmp &= ~CAM_REG_FRAME_CNT_TRGR_INT_MASK;
			tmp |= arg & CAM_REG_FRAME_CNT_TRGR_INT_MASK;
			sys_write32(tmp, config->base + CAM_DVP_STATUS_AND_ERROR_OFFSET);
			break;

		case CAM_CMD_SET_OUTPUT_ADDR:
			sys_write32(arg, config->base + CAM_DVP2AXI_ADDR_START_OFFSET);
			break;

		default:
			ret = -EPERM;
			break;
	}
	return ret;
}
#endif
