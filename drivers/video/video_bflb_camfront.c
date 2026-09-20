/*
 * SPDX-FileCopyrightText: Copyright Panoramix Labs
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_camfront

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/otp.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/dt-bindings/clock/bflb_bl61x_clock.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/video/video.h>

LOG_MODULE_REGISTER(bflb_camfront, CONFIG_VIDEO_LOG_LEVEL);

#include <bouffalolab/common/cam_front_reg.h>
#include <bouffalolab/common/cam_reg.h>

#include "video_common.h"

#include <bflb_soc.h>
#include <glb_reg.h>

#define GLB_CAM_CLK_XCLK		0
#define GLB_CAM_CLK_WIFIPLL_96M		1
#define GLB_CAM_CLK_TOP_AUPLL_DIV5	2

/* for GLB_CAM_CLK_TOP_AUPLL */
#define CAMFRONT_CAM_REF_CLK_DIV	5

struct bflb_camfront_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;
	const struct device *source_dev;
	void (*irq_config_func)(const struct device *dev);
	uint8_t bus_width;
	uint8_t clock_divider;
};

struct bflb_camfront_data {
	struct video_format_cap fmts[CONFIG_VIDEO_BFLB_CAMFRONT_MAX_FORMATS];
	size_t num_fmts;
	struct video_format fmt;
	bool is_streaming;
};

static void bflb_camfront_add_format_cap(const struct device *dev,
				         const struct video_format_cap *fmt_cap)
{
	struct bflb_camfront_data *data = dev->data;

	if (data->num_fmts + 1 >= CONFIG_VIDEO_BFLB_CAMFRONT_MAX_FORMATS) {
		LOG_WRN("CONFIG_VIDEO_BFLB_CAMFRONT_MAX_FORMATS too small, raise above %u",
			CONFIG_VIDEO_BFLB_CAMFRONT_MAX_FORMATS);
		return;
	}

	data->fmts[data->num_fmts] = *fmt_cap;
	data->num_fmts++;
}

static uint32_t bflb_camfront_convert_pixfmt(uint32_t pixfmt)
{
	switch (pixfmt) {
	case VIDEO_PIX_FMT_YUYV:
		return VIDEO_PIX_FMT_UYVY;
	case VIDEO_PIX_FMT_YVYU:
		return VIDEO_PIX_FMT_VYUY;
	case VIDEO_PIX_FMT_UYVY:
		return VIDEO_PIX_FMT_YUYV;
	case VIDEO_PIX_FMT_VYUY:
		return VIDEO_PIX_FMT_YVYU;
	case VIDEO_PIX_FMT_RGB565:
		return VIDEO_PIX_FMT_RGB565X;
	case VIDEO_PIX_FMT_RGB565X:
		return VIDEO_PIX_FMT_RGB565;
	default:
		return pixfmt;
	}
}

static int bflb_camfront_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct bflb_camfront_config *config = dev->config;
	struct bflb_camfront_data *data = dev->data;
	struct video_caps source_caps = {.type = VIDEO_BUF_TYPE_OUTPUT};
	int ret;

	caps->format_caps = data->fmts;
	caps->min_vbuf_count = 1;
	//caps->buf_align = 16; /* TODO */

	if (data->num_fmts > 0) {
		return 0;
	}

	ret = video_get_caps(config->source_dev, &source_caps);
	if (ret != 0) {
		LOG_ERR("Failed to get %s capabilities", config->source_dev->name);
		return ret;
	}

	for (size_t i = 0; source_caps.format_caps[i].pixelformat != 0; i++) {
		uint32_t conv;

		bflb_camfront_add_format_cap(dev, &source_caps.format_caps[i]);

		conv = bflb_camfront_convert_pixfmt(source_caps.format_caps[i].pixelformat);
		if (source_caps.format_caps[i].pixelformat != conv) {
			struct video_format_cap new_format_cap = source_caps.format_caps[i];

			new_format_cap.pixelformat = conv;
			bflb_camfront_add_format_cap(dev, &new_format_cap);
		}
	}

	return 0;
}

static int bflb_camfront_set_format(const struct device *dev, struct video_format *fmt)
{
	const struct bflb_camfront_config *config = dev->config;
	struct bflb_camfront_data *data = dev->data;
	size_t fmt_idx;
	int ret;

	ret = video_format_caps_index(data->fmts, fmt, &fmt_idx);
	if (ret < 0) {
		LOG_ERR("Format %s %ux%u unsupported by %s",
			VIDEO_FOURCC_TO_STR(fmt->pixelformat), fmt->width, fmt->height, dev->name);
		return ret;
	}

	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		return ret;
	}

	ret = video_set_format(config->source_dev, fmt);
	if (ret < 0) {
		LOG_ERR("Failed to set %s format to %s %ux%u",
			config->source_dev->name, VIDEO_FOURCC_TO_STR(data->fmt.pixelformat),
			data->fmt.width, data->fmt.height);
		return ret;
	}

	data->fmt = *fmt;

	return 0;
}

static int bflb_camfront_get_format(const struct device *dev, struct video_format *fmt)
{
	struct bflb_camfront_data *data = dev->data;

	*fmt = data->fmt;

	return 0;
}

static int bflb_camfront_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct bflb_camfront_config *config = dev->config;

	return video_set_frmival(config->source_dev, frmival);
}

static int bflb_camfront_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct bflb_camfront_config *config = dev->config;

	return video_get_frmival(config->source_dev, frmival);
}

static int bflb_camfront_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	const struct bflb_camfront_config *config = dev->config;

	return video_enum_frmival(config->source_dev, fie);
}

#if 0
static int bflb_camfront_apply_config(const struct device *dev)
{
	const struct bflb_camfront_config *config = dev->config;
	struct bflb_camfront_data *data = dev->data;
	const struct device *clock_dev = DEVICE_DT_GET_ANY(bflb_clock_controller);
	uint32_t threshold;
	uint32_t src_clk_hz;
	uint32_t cam_ref_clk_hz;
	int64_t pix_clk_hz;
	uint32_t tmp;
	int ret;

	/* source clock */
	ret = clock_control_get_rate(clock_dev, (void *)BL61X_CLKID_CLK_AUPLL, &src_clk_hz);
	if (ret < 0) {
		LOG_ERR("Failed to query BL61X_CLKID_CLK_XCLK speed");
		return ret;
	}

	/* cam-ref-clk */
	cam_ref_clk_hz = src_clk_hz / CAMFRONT_CAM_REF_CLK_DIV / config->clock_divider;

	/* camera's pclk signal */
	pix_clk_hz = video_get_dvp_link_freq(
		dev, video_bits_per_pixel(data->fmt.pixelformat), config->bus_width);
	if (pix_clk_hz < 0) {
		LOG_ERR("%s needs the link frequency to configure the theshold", dev->name);
		return -ENOTSUP;
	}

	/* TODO get from sensor control */
	pix_clk_hz = 48000000;

	/* Set the line-buffer threshold past the half of the width, depending on the ratio between
	 * this peripheral clock (processing speed) and the pixel clock (input data speed).
	 *
	 * This is likely the trigger for starting to flush the line to the next core (DVP2AXI)
	 * in the video pipeline.
	 */
	threshold = data->fmt.width - data->fmt.width * pix_clk_hz / cam_ref_clk_hz / 2 + 10;
	threshold = CLAMP(threshold, 2, data->fmt.width);
	threshold = CLAMP(threshold, 2, 1024);
	threshold = 900; /* TODO experimental */

	LOG_DBG("setting %s x threshold to %u (format %s %ux%u, pix clk %llu hz, cam clk %u hz)",
		config->source_dev->name, threshold, VIDEO_FOURCC_TO_STR(data->fmt.pixelformat),
		data->fmt.width, data->fmt.height, pix_clk_hz, cam_ref_clk_hz);

	tmp = sys_read32(config->base + CAM_FRONT_CONFIG_OFFSET);
	tmp &= ~CAM_FRONT_RG_DVPAS_FIFO_TH_MASK;
	tmp |= threshold << CAM_FRONT_RG_DVPAS_FIFO_TH_SHIFT;
	sys_write32(tmp, config->base + CAM_FRONT_CONFIG_OFFSET);

	/* TODO handle pixel format conversion at camfront level */
	/* 16-bit endianess swap in the front-end */
	/* If image sensor output format is YUYV, it will be changed to UYVY */
	tmp = sys_read32(config->base + CAM_FRONT_CONFIG_OFFSET);
	if (arg) {
		tmp |= CAM_FRONT_RG_DVPAS_DA_ORDER;
	} else {
		tmp &= ~CAM_FRONT_RG_DVPAS_DA_ORDER;
	}
	sys_write32(tmp, config->base + CAM_FRONT_CONFIG_OFFSET);

	return 0;
}
#endif

static int bflb_camfront_set_stream(const struct device *dev, bool stream, enum video_buf_type type)
{
	const struct bflb_camfront_config *config = dev->config;
	struct bflb_camfront_data *data = dev->data;
	int ret;

	if (type != VIDEO_BUF_TYPE_OUTPUT) {
		LOG_ERR("Supporting output buffer type only");
		return -EINVAL;
	}

	if (data->is_streaming == stream) {
		return 0;
	}

	if (stream) {
#if 0
		ret = bflb_camfront_apply_config(dev);
		if (ret < 0) {
			return ret;
		}
#endif

		ret = video_stream_start(config->source_dev, type);
		if (ret < 0) {
			LOG_ERR("Failed to start source device %s", config->source_dev->name);
			return ret;
		}
	} else {
		ret = video_stream_stop(config->source_dev, type);
		if (ret < 0) {
			LOG_ERR("Failed to stop source device %s", config->source_dev->name);
			return ret;
		}
	}

	data->is_streaming = stream;

	return 0;
}

static DEVICE_API(video, bflb_camfront_api) = {
	.set_format = bflb_camfront_set_format,
	.get_format = bflb_camfront_get_format,
	.set_stream = bflb_camfront_set_stream,
	.get_caps = bflb_camfront_get_caps,
	.set_frmival = bflb_camfront_set_frmival,
	.get_frmival = bflb_camfront_get_frmival,
	.enum_frmival = bflb_camfront_enum_frmival,
};

static void bflb_camfront_init_clock(const struct device *dev)
{
	const struct bflb_camfront_config *config = dev->config;
	uint32_t tmp;

	/* clock off */
	tmp = sys_read32(GLB_BASE + GLB_CAM_CFG0_OFFSET);
	tmp &= ~GLB_REG_CAM_REF_CLK_EN_MSK;
	sys_write32(tmp, GLB_BASE + GLB_CAM_CFG0_OFFSET);

	tmp = sys_read32(GLB_BASE + GLB_CAM_CFG0_OFFSET);
	tmp &= ~GLB_REG_CAM_REF_CLK_SRC_SEL_MSK;
	tmp |= GLB_CAM_CLK_WIFIPLL_96M<< GLB_REG_CAM_REF_CLK_SRC_SEL_POS;
	tmp &= ~GLB_REG_CAM_REF_CLK_DIV_MSK;
	tmp |= (config->clock_divider - 1) << GLB_REG_CAM_REF_CLK_DIV_POS;
	sys_write32(tmp, GLB_BASE + GLB_CAM_CFG0_OFFSET);

	/* clock on */
	tmp = sys_read32(GLB_BASE + GLB_CAM_CFG0_OFFSET);
	tmp |= GLB_REG_CAM_REF_CLK_EN_MSK;
	sys_write32(tmp, GLB_BASE + GLB_CAM_CFG0_OFFSET);
}

static int bflb_camfront_init(const struct device *dev)
{
	const struct bflb_camfront_config *config = dev->config;
	int ret;

	LOG_DBG("Initializing %s", dev->name);

	bflb_camfront_init_clock(dev);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Failed to apply %s default pin config", dev->name);
		return ret;
	}

	/* TODO: loop over sinks and convifure them one by one */
	/* Select the sink to which related config applies */
	sys_write32(0, config->base + CAM_FRONT_DVP2BUS_SRC_SEL_1_OFFSET);

	/* Default format is set by dvp2axi */

	return 0;
}

#if CONFIG_DEVICE_DEINIT_SUPPORT
static const int bflb_camfront_deinit(const struct device *dev)
{
	return 0;
}
#endif

#if 0
#include <zephyr/drivers/uart.h>

static uint64_t __debug_num_isr;
static uint32_t __debug_last_mcause;

void __debug_probe(int n) {
	//uint32_t mcause_val;

	//__asm__ volatile ("csrr %0, mcause" : "=r"(mcause_val) : : );
	__debug_num_isr++;
	//__debug_last_mcause = mcause_val;
}

static void __debug_report(void *p0, void *p1, void *p2) {
	k_sleep(K_SECONDS(1));
	while (true) {
		printk("%%");
		//uart_poll_out(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)), '%');
		//printk("num_isr: %llu\n", __debug_num_isr);
		//printk("last_mcause: %u\n", __debug_last_mcause);
		k_sleep(K_SECONDS(1));
	}
}
K_THREAD_DEFINE(debug_report, 1024,
		__debug_report, NULL, NULL, NULL,
		0, K_ESSENTIAL, 0);
#endif

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(n, 0, 0)))

#define VIDEO_BFLB_CAMFRONT_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);								\
												\
	const struct bflb_camfront_config bflb_camfront_config_##n = {				\
		.base = DT_INST_REG_ADDR(n),							\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		.clock_divider= DT_INST_PROP(n, clock_divider),					\
		.source_dev = SOURCE_DEV(n),							\
		.bus_width = DT_PROP(DT_INST_ENDPOINT_BY_ID(n, 0, 0), bus_width),		\
	};											\
												\
	struct bflb_camfront_data bflb_camfront_data_##n = {					\
	};											\
												\
	DEVICE_DT_INST_DEINIT_DEFINE(n, &bflb_camfront_init, &bflb_camfront_deinit, NULL,	\
				     &bflb_camfront_data_##n, &bflb_camfront_config_##n,	\
				     POST_KERNEL, CONFIG_VIDEO_BFLB_CAMFRONT_INIT_PRIORITY,	\
				     &bflb_camfront_api);					\
												\
	VIDEO_DEVICE_DEFINE(camfront_##n, DEVICE_DT_INST_GET(n), SOURCE_DEV(n));

DT_INST_FOREACH_STATUS_OKAY(VIDEO_BFLB_CAMFRONT_INIT)


#if 0
static void bflb_cam_init(const struct device *dev)
{
	const struct camfront_config *config = dev->config;
	uint32_t threshold;
	uint64_t cam_ref_clk;
	uint32_t tmp;

	cam_ref_clk = bflb_clk_get_peripheral_clock(BFLB_DEVICE_TYPE_CAMERA, 0) / 1000;
	threshold = fmt->width - fmt->width * config->pixel_clock / cam_ref_clk / 2 + 10;
	if (threshold > (fmt->width - 1)) {
		threshold = fmt->width - 1;
	}
	if (threshold < 2) {
		threshold = 2;
	}
	if (threshold > 1024) {
		threshold = 1024;
	}

	tmp = sys_read32(config->base + CAM_FRONT_CONFIG_OFFSET);
	tmp &= ~CAM_FRONT_RG_DVPAS_FIFO_TH_MASK;
	tmp |= threshold << CAM_FRONT_RG_DVPAS_FIFO_TH_SHIFT;
	sys_write32(tmp, config->base + CAM_FRONT_CONFIG_OFFSET);
}
#endif
