/*
 * SPDX-FileCopyrightText: Copyright Panoramix Labs
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_dvp

#include <zephyr/drivers/otp.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/irq.h>
#include <zephyr/video/video.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(video_bflb, CONFIG_VIDEO_LOG_LEVEL);

#include <bouffalolab/common/cam_reg.h>
#include <bflb_soc.h>
#include <glb_reg.h>

struct video_bflb_config {
	const struct pinctrl_dev_config *pcfg;
	const struct device *source_dev;
	void (*irq_config_func)(const struct device *dev);
};

struct video_bflb_data {
	;
};

struct video_format_cap video_bflb_fmts[] = {
	{0},
};

static int stm32_dcmipp_get_caps(const struct device *dev, struct video_caps *caps)
{
	struct stm32_dcmipp_pipe_data *pipe = dev->data;

	caps->format_caps = video_bflb_fmts;
	caps->min_vbuf_count = 1;
	//caps->buf_align = 16;

	return 0;
}

static const void video_bflb_isr(const void *p)
{
	;
}

static void video_bflb_init_clock(const struct device *dev)
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
	tmp |= 3 << GLB_REG_CAM_REF_CLK_DIV_POS;
	sys_write32(tmp, GLB_BASE + GLB_CAM_CFG0_OFFSET);

	/* enable clock routing */
	tmp = sys_read32(GLB_BASE + GLB_CAM_CFG0_OFFSET);
	tmp |= GLB_REG_CAM_REF_CLK_EN_MSK;
	sys_write32(tmp, GLB_BASE + GLB_CAM_CFG0_OFFSET);
}

static int video_bflb_init(const struct device *dev)
{
	const struct video_bflb_config *config = dev->config;
	int ret;

	video_bflb_init_clock(dev);

	config->irq_config_func(dev);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Failed to apply %s default pin config", dev->name);
		return ret;
	}

	return 0;
}

#if CONFIG_DEVICE_DEINIT_SUPPORT
static const int video_bflb_deinit(const struct device *dev)
{
	return 0;
}
#endif

static DEVICE_API(video, video_bflb_api) = {
};

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_NODE_REMOTE_DEVICE(DT_INST_ENDPOINT_BY_ID(n, 0, 0)))

#define VIDEO_BFLB_DVP2AXI_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);								\
												\
	static void video_bflb_irq_config_##n(const struct device *dev)				\
	{											\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),				\
			    video_bflb_isr, DEVICE_DT_INST_GET(n), 0);				\
		irq_enable(DT_INST_IRQN(n));							\
	}											\
												\
	struct video_bflb_data video_bflb_data_##n = {						\
	};											\
												\
	const struct video_bflb_config video_bflb_config_##n = {				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		.irq_config_func = &video_bflb_irq_config_##n,					\
		.source_dev = SOURCE_DEV(n),							\
	};											\
												\
	DEVICE_DT_INST_DEINIT_DEFINE(n, &video_bflb_init, &video_bflb_deinit, NULL,		\
				     &video_bflb_data_##n, &video_bflb_config_##n,		\
				     POST_KERNEL, CONFIG_VIDEO_BFLB_DVP_INIT_PRIORITY,		\
				     &video_bflb_api);						\
												\

	//VIDEO_DEVICE_DEFINE(dvp##n, DEVICE_INST_DT_GET(n), SOURCE_DEV(n));

DT_INST_FOREACH_STATUS_OKAY(VIDEO_BFLB_DVP2AXI_INIT)
