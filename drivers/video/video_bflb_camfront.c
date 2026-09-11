/*
 * SPDX-FileCopyrightText: Copyright Panoramix Labs
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_camfront

#include <zephyr/drivers/otp.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/irq.h>
#include <zephyr/video/video.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bflb_camfront, CONFIG_VIDEO_LOG_LEVEL);

#include <bouffalolab/common/cam_reg.h>
#include <bouffalolab/common/cam_front_reg.h>
#include <bflb_soc.h>
#include <glb_reg.h>

#define BFLB_CAMFRONT_REF_CLK_DIV 3

struct bflb_camfront_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;
	const struct device *source_dev;
	void (*irq_config_func)(const struct device *dev);
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
	//caps->buf_align = 16;

	if (data->num_fmts == 0) {
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
	struct bflb_camfront_data *data = dev->data;
	int ret;

	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
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

static int bflb_camfront_apply_format(const struct device *dev)
{
	const struct bflb_camfront_config *config = dev->config;
	struct bflb_camfront_data *data = dev->data;
	const struct device *clock_dev = DEVICE_DT_GET_ANY(bflb_clock_controller);
	struct video_control pix_rate_rate = {.id = VIDEO_CID_PIXEL_RATE};
	struct video_frmival frmival = {};
	uint32_t threshold_x;
	uint64_t cam_ref_clk_hz;
	uint32_t tmp;
	int ret;

	cam_ref_clk_hz = clock_control_get_rate(clock_dev, (void *)BL61X_CLKID_CLK_XCLK)
		/ BFLB_CAMFRONT_REF_CLK_DIV;

	ret = video_ctrl_get(config->source_dev, &pix_rate_ctrl);
	if (ret < 0) {
		LOG_WRN("The camfront driver relies on knowing the pixel clock of %s",
			config->source_dev->name);
		LOG_WRN("Using less reliable estimation using frame rate instead");

		/* We do not know VBLANK/HBLANK, neither if these are reporeted precisely */

		ret = video_get_frmival(dev, &frmival);
		if (ret < 0) {
			LOG_ERR("Cannot estimate %s threshold value from either pixel or frame rate",
				config->source_dev->name);
		}
	}

	threshold_x =
		data->fmt.width
		- (data->fmt.width * pixel_clock / cam_ref_clk_hz) / 2 + 10;
	threshold_x = CLAMP(threshold_x, 2, config->resolution_x);
	threshold_x = CLAMP(threshold_x, 2, 1024);

	LOG_ERR("threshold_x %u");

	tmp = sys_read32(config->base + CAM_FRONT_CONFIG_OFFSET);
	tmp &= CAM_FRONT_RG_DVPAS_FIFO_TH_UMSK;
	tmp |= threshold_x << CAM_FRONT_RG_DVPAS_FIFO_TH_SHIFT;
	sys_write32(tmp, config->base + CAM_FRONT_CONFIG_OFFSET);

	/* If image sensor output format is YUYV, it will be changed to UYVY */
	tmp = sys_read32(config->base + CAM_FRONT_CONFIG_OFFSET);
	if (arg) {
		tmp |= CAM_FRONT_RG_DVPAS_DA_ORDER;
	} else {
		tmp &= ~CAM_FRONT_RG_DVPAS_DA_ORDER;
	}
	sys_write32(tmp, config->base + CAM_FRONT_CONFIG_OFFSET);

	ret = video_set_format(config->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	if (config->output_format >= CAM_OUTPUT_FORMAT_RGB888_OR_BGR888 &&
		config->output_format <= CAM_OUTPUT_FORMAT_RGB888_TO_RGBA8888) {
		tmp = sys_read32(config->base + CAM_FRONT_DVP_MUX_SEL_REG_OFFSET);
		tmp |= (4 << (dev->idx == 0 ? CAM_FRONT_REG_D2XA_IN_SEL_SHIFT : CAM_FRONT_REG_D2XB_IN_SEL_SHIFT));
		sys_write32(tmp, config->base + CAM_FRONT_DVP_MUX_SEL_REG_OFFSET);
		sys_write32(CAM_FRONT_MM_MISC_CR_ISP_Y2R_EN, config->base + CAM_FRONT_MM_MISC_ISP_Y2R_CONFIG_0_OFFSET);
	}

	/* either of these depending on the sink to configure/enable/feed/? */
	sys_write32(0, config->base + CAM_FRONT_DVP2BUS_SRC_SEL_1_OFFSET);
	sys_write32(1, config->base + CAM_FRONT_DVP2BUS_SRC_SEL_1_OFFSET);

	return 0;
}

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
		ret = bflb_camfront_apply_format(dev);
		if (ret < 0) {
			return ret;
		}

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

static void bflb_camfront_init_clock(const struct device *dev)
{
	uint32_t tmp;

	/* disable clock routing */
	tmp = sys_read32(GLB_BASE + GLB_CAM_CFG0_OFFSET);
	tmp &= GLB_REG_CAM_REF_CLK_EN_UMSK;
	sys_write32(tmp, GLB_BASE + GLB_CAM_CFG0_OFFSET);

	/* src=xclk, div=3 */
	tmp = sys_read32(GLB_BASE + GLB_CAM_CFG0_OFFSET);
	tmp &= GLB_REG_CAM_REF_CLK_SRC_SEL_UMSK;
	tmp |= 3 << GLB_REG_CAM_REF_CLK_SRC_SEL_POS;
	tmp &= GLB_REG_CAM_REF_CLK_DIV_UMSK;
	tmp |= BFLB_CAMFRONT_REF_CLK_DIV << GLB_REG_CAM_REF_CLK_DIV_POS;
	sys_write32(tmp, GLB_BASE + GLB_CAM_CFG0_OFFSET);

	/* enable clock routing */
	tmp = sys_read32(GLB_BASE + GLB_CAM_CFG0_OFFSET);
	tmp |= GLB_REG_CAM_REF_CLK_EN_MSK;
	sys_write32(tmp, GLB_BASE + GLB_CAM_CFG0_OFFSET);
}

static int bflb_camfront_init(const struct device *dev)
{
	const struct bflb_camfront_config *config = dev->config;
	uint32_t tmp;
	int ret;

	bflb_camfront_init_clock(dev);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Failed to apply %s default pin config", dev->name);
		return ret;
	}

	sys_write32(0, config->base + CAM_FRONT_DVP2BUS_SRC_SEL_1_OFFSET);

	/* TODO: put at the end of format config? */
	tmp = sys_read32(config->base + CAM_FRONT_CONFIG_OFFSET);
	tmp |= CAM_FRONT_RG_DVPAS_ENABLE;
	sys_write32(tmp, config->base + CAM_FRONT_CONFIG_OFFSET);

	return 0;
}

#if CONFIG_DEVICE_DEINIT_SUPPORT
static const int bflb_camfront_deinit(const struct device *dev)
{
	return 0;
}
#endif

static DEVICE_API(video, bflb_camfront_api) = {
	.set_format = bflb_camfront_set_format,
	.get_format = bflb_camfront_get_format,
	.set_stream = bflb_camfront_set_stream,
	.get_caps = bflb_camfront_get_caps,
	.set_frmival = bflb_camfront_set_frmival,
	.get_frmival = bflb_camfront_get_frmival,
	.enum_frmival = bflb_camfront_enum_frmival,
};

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(n, 0, 0)))

#define VIDEO_BFLB_CAMFRONT_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);								\
												\
	const struct bflb_camfront_config bflb_camfront_config_##n = {				\
		.base = DT_INST_REG_ADDR(n),							\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		.source_dev = SOURCE_DEV(n),							\
	};											\
												\
	struct bflb_camfront_data bflb_camfront_data_##n = {					\
	};											\
												\
	DEVICE_DT_INST_DEINIT_DEFINE(n, &bflb_camfront_init, &bflb_camfront_deinit, NULL,	\
				     &bflb_camfront_data_##n, &bflb_camfront_config_##n,	\
				     POST_KERNEL, CONFIG_VIDEO_BFLB_DVP_INIT_PRIORITY,		\
				     &bflb_camfront_api);					\
												\
	VIDEO_DEVICE_DEFINE(camfront##n, DEVICE_INST_DT_GET(n), SOURCE_DEV(n));

DT_INST_FOREACH_STATUS_OKAY(VIDEO_BFLB_CAMFRONT_INIT)
