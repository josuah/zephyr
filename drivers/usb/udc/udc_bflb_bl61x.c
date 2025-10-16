/*
 * Copyright (c) 2024-2026 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The Bouffalo Lab USB V2 controller features:
 *  - Dedicated CX (Control Exchange) engine for EP0
 *  - 8 IN + 8 OUT data endpoints (separate)
 *  - Shared FIFO pool (F0-F3 in base regs, F4-F7 in ext regs)
 *  - Built-in DMA engine for FIFO data transfer
 *  - PHY controlled via PDS registers
 *  - Grouped interrupt architecture (G0=CX, G1=FIFO, G2=device)
 */

#include "udc_common.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/cache.h>
#include <zephyr/sys/clock.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_bflb_bl61x, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include <soc.h>
#include <bflb_soc.h>
#include <glb_reg.h>
#include <pds_reg.h>
#include <bouffalolab/common/usb_v2_reg.h>

#define DT_DRV_COMPAT bflb_bl61x_udc

#define USB_BL61X_SPEED_LOW	1
#define USB_BL61X_SPEED_FULL	0
#define USB_BL61X_SPEED_HIGH	2

#define USB_BL61X_FX_X_MASK	0x3F
#define USB_BL61X_FX_X_OFFSET	8

#define USB_BL61X_XPS_X_OFFSET	4

#define USB_BL61X_HSFIFOCAP	512

#define USB_BL61X_NUM_FIFO	4

#define USB_BL61X_EP_DIR_IN	0
#define USB_BL61X_EP_DIR_OUT	1
#define USB_BL61X_FIFO_DIR_OUT	0
#define USB_BL61X_FIFO_DIR_IN	1
#define USB_BL61X_FIFO_DIR_BID	2
#define USB_BL61X_FIFO_EP_NONE	15

#define USB_BL61X_TIMER_AFTER_RESET_HS	(0x44C)
#define USB_BL61X_TIMER_AFTER_RESET_FS	(0x2710)
#define USB_BL61X_TIMER_AFTER_RESET_T	K_MSEC(100)

#define USB_MCX_COMEND_INT (1 << 3)

#define USB_FX_OUT_INT(n)	((1U << ((n) * 2)) + 0)
#define USB_FX_SPK_INT(n)	((1U << ((n) * 2)) + 1)
#define USB_FX_IN_INT(n)	(1U << (16 + (n)))
#define USB_VDMA_CMPLT_FX(n)	(1U << ((n) + 1))

#define USB_VDMA_START		USB_VDMA_START_CXF
#define USB_VDMA_LEN_MASK	USB_VDMA_LEN_CXF_MASK
#define USB_VDMA_IO		USB_VDMA_IO_CXF
#define USB_VDMA_TYPE		USB_VDMA_TYPE_CXF
#define USB_VDMA_LEN_SHIFT	USB_VDMA_LEN_CXF_SHIFT

struct udc_bflb_bl61x_fifo {
	struct udc_ep_config *ep_cfg;
	const struct device *dev;
	struct k_work work;
	bool xfer_end;
	uint8_t idx;
};

struct udc_bflb_bl61x_config {
	uint32_t base;
	size_t num_of_eps;
	void (*irq_enable_func)(const struct device *const dev);
	void (*irq_disable_func)(const struct device *const dev);
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	int speed_idx;
};

struct udc_bflb_bl61x_data {
	struct usb_setup_packet setup __aligned(8);
	k_timepoint_t reset_expiration;
	struct udc_bflb_bl61x_fifo ctrl_fifo;
	struct udc_bflb_bl61x_fifo ep_fifo[USB_BL61X_NUM_FIFO];
};

static enum udc_bus_speed udc_bflb_bl61x_device_speed(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_bflb_bl61x_data *const priv = udc_get_private(dev);
	uint32_t speed;

	/* Reset or init ongoing, result would be incorrect */
	k_sleep(sys_timepoint_timeout(priv->reset_expiration));

	speed = sys_read32(cfg->base + USB_OTG_CSR_OFFSET);
	speed &= USB_SPD_TYP_HOV_POV_MASK;
	speed = speed >> USB_SPD_TYP_HOV_POV_SHIFT;

	if (speed == USB_BL61X_SPEED_FULL) {
		return UDC_BUS_SPEED_FS;
	} else if (speed == USB_BL61X_SPEED_HIGH) {
		return UDC_BUS_SPEED_HS;
	} else {
		return UDC_BUS_UNKNOWN;
	}

	return UDC_BUS_UNKNOWN;
}

static void udc_bflb_bl61x_ctrl_zlp(const struct device *const dev)
{
	uint32_t tmp;
	const struct udc_bflb_bl61x_config *const cfg = dev->config;

	tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
	tmp |= USB_CX_DONE;
	sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
}

static void udc_bflb_bl61x_ep_tx_zlp(const struct device *const dev, const uint8_t ep_idx)
{
	uint32_t tmp;
	const struct udc_bflb_bl61x_config *const cfg = dev->config;

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
	tmp |= USB_TX0BYTE_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
}

static void udc_bflb_bl61x_ep_rx_zlp(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;

	/* Acknowledge all EPs at once */
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_RXZ_OFFSET);
}

static void udc_bflb_bl61x_fifo_configure(const struct device *const dev,
					  const struct udc_bflb_bl61x_fifo *const fifo,
					  const uint8_t block_num,
					  const bool enabled)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	const uint8_t ep_type = fifo->ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK;
	const uint8_t fifo_idx = fifo->idx;
	uint32_t tmp;

	__ASSERT_NO_MSG(fifo_idx < USB_BL61X_NUM_FIFO);

	tmp = sys_read32(cfg->base + USB_DEV_FCFG_OFFSET);
	tmp &= ~(USB_BL61X_FX_X_MASK << (fifo_idx * USB_BL61X_FX_X_OFFSET));
	tmp |= (ep_type << (fifo_idx * USB_BL61X_FX_X_OFFSET + USB_BLK_TYP_F0_SHIFT));
	tmp |= ((block_num - 1) << (fifo_idx * USB_BL61X_FX_X_OFFSET + USB_BLKNO_F0_SHIFT));
	if (fifo->ep_cfg->mps > USB_BL61X_HSFIFOCAP) {
		tmp |= (1U << (fifo_idx * USB_BL61X_FX_X_OFFSET + USB_BLKSZ_F0));
	}
	if (enabled) {
		tmp |= (1U << (fifo_idx * USB_BL61X_FX_X_OFFSET + USB_EN_F0));
	} else {
		tmp &= ~(1U << (fifo_idx * USB_BL61X_FX_X_OFFSET + USB_EN_F0));
	}
	sys_write32(tmp, cfg->base + USB_DEV_FCFG_OFFSET);
}

static void udc_bflb_bl61x_ep_set_out_mps(const struct device *const dev,
					  const uint8_t ep_idx,
					  const uint16_t ep_mps)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
	tmp |= USB_RSTG_OEP1;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
	tmp &= ~USB_RSTG_OEP1;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
	tmp &= ~USB_MAXPS_OEP1_MASK;
	tmp |= ep_mps;
	sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
}

static void udc_bflb_bl61x_ep_set_in_mps(const struct device *const dev,
					  const uint8_t ep_idx,
					  const uint16_t ep_mps)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
	tmp |= USB_RSTG_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
	tmp &= ~USB_RSTG_IEP1;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
	tmp &= ~USB_MAXPS_IEP1_MASK;
	tmp |= ep_mps;
	tmp &= ~USB_TX_NUM_HBW_IEP1_MASK;
	sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
		+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
}

static void udc_bflb_bl61x_fifo_reset_ctrl(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
	tmp |= USB_CX_CLR;
	sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
}

static void udc_bflb_bl61x_fifo_reset(const struct device *const dev, const uint8_t fifo_idx)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_FIBC0_OFFSET + 4 * fifo_idx);
	tmp |= USB_FFRST0_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_FIBC0_OFFSET + 4 * fifo_idx);
}

/* fifo_idx : 1-4, ep_idx: 1-4
 * ep_direction: 0 in 1 out
 */
static void udc_bflb_bl61x_ep_setfifo(const struct device *const dev,
				      const uint8_t ep_idx,
				      const uint8_t ep_dir,
				      const uint8_t fifo_idx)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;
	const uint8_t ep_dir_bit = ep_dir * 4;

	if (ep_idx < 5) {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP0_OFFSET);
		tmp &= ~(0xf << ((ep_idx - 1) * 8 + ep_dir_bit));
		tmp |= (fifo_idx << ((ep_idx - 1) * 8 + ep_dir_bit));
		sys_write32(tmp, cfg->base + USB_DEV_EPMAP0_OFFSET);
	} else {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP1_OFFSET);
		tmp &= ~(0xf << ((ep_idx - 5) * 8 + ep_dir_bit));
		tmp |= (fifo_idx << ((ep_idx - 5) * 8 + ep_dir_bit));
		sys_write32(tmp, cfg->base + USB_DEV_EPMAP1_OFFSET);
	}
}

static struct udc_bflb_bl61x_fifo *udc_bflb_bl61x_ep_getfifo(const struct device *const dev,
							     const uint8_t ep_addr)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_bflb_bl61x_data *const priv = udc_get_private(dev);
	const uint8_t ep_dir = USB_EP_DIR_IS_IN(ep_addr) ? USB_BL61X_EP_DIR_IN : USB_BL61X_EP_DIR_OUT;
	const uint8_t ep_dir_bit = ep_dir * 4;
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_addr);
	uint8_t fifo_idx;
	uint32_t tmp;

	if (ep_idx == 0) {
		return &priv->ctrl_fifo;
	}

	if (ep_idx < 5) {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP0_OFFSET);
		fifo_idx = (tmp >> ((ep_idx - 1) * 8 + ep_dir_bit)) & 0xf;
	} else {
		tmp = sys_read32(cfg->base + USB_DEV_EPMAP1_OFFSET);
		fifo_idx = (tmp >> ((ep_idx - 5) * 8 + ep_dir_bit)) & 0xf;
	}

	__ASSERT_NO_MSG(fifo_idx <= ARRAY_SIZE(priv->ep_fifo));

	return &priv->ep_fifo[fifo_idx];
}

static void udc_bflb_bl61x_fifo_setep(const struct device *const dev,
				      const uint8_t ep_idx,
				      const uint8_t fifo_idx,
				      const uint8_t fifo_dir)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	__ASSERT_NO_MSG(ep_idx < 16);
	__ASSERT_NO_MSG(fifo_idx < USB_BL61X_NUM_FIFO);
	__ASSERT_NO_MSG(fifo_dir <= 2);

	tmp = sys_read32(cfg->base + USB_DEV_FMAP_OFFSET);
	tmp &= ~(USB_BL61X_FX_X_MASK << (fifo_idx * USB_BL61X_FX_X_OFFSET));
	tmp |= (ep_idx << (fifo_idx * USB_BL61X_FX_X_OFFSET));
	tmp |= (fifo_dir << (fifo_idx * USB_BL61X_FX_X_OFFSET + USB_DIR_FIFO0_SHIFT));
	sys_write32(tmp, cfg->base + USB_DEV_FMAP_OFFSET);
}

/* bl61x cannot use cpu read/write for USB */
static void udc_bflb_bl61x_vdma_startread(const struct device *const dev,
					  const uint8_t fifo_idx,
					  uint8_t *const data,
					  const uint32_t len)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	LOG_INF("Starting DMA read, FIFO %u, data %p, len %u", fifo_idx, (void *)data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);
	tmp &= ~USB_VDMA_LEN_MASK;
	tmp &= ~USB_VDMA_IO;
	tmp &= ~USB_VDMA_TYPE;
	tmp |= (len << USB_VDMA_LEN_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);

	sys_write32((uint32_t)data, cfg->base + USB_VDMA_F0PS2_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);

	sys_cache_data_flush_and_invd_range(data, len);
	compiler_barrier();

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);
	tmp |= USB_VDMA_START;
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);

	compiler_barrier();
	sys_cache_data_flush_and_invd_range(data, len);
}

static void udc_bflb_bl61x_vdma_startwrite(const struct device *const dev,
					   const uint8_t fifo_idx, uint8_t *data,
					   const uint32_t len)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	LOG_INF("Starting DMA write, FIFO %u, buf %p, len %u", fifo_idx, (void *)data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);
	tmp &= ~USB_VDMA_LEN_MASK;
	tmp &= ~USB_VDMA_IO;
	tmp |= USB_VDMA_TYPE;
	tmp |= (len << USB_VDMA_LEN_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);

	sys_write32((uint32_t)data, cfg->base + USB_VDMA_F0PS2_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);

	sys_cache_data_flush_and_invd_range(data, len);
	compiler_barrier();

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);
	tmp |= USB_VDMA_START;
	sys_write32(tmp, cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);

	compiler_barrier();
	sys_cache_data_flush_and_invd_range(data, len);
}

static void udc_bflb_bl61x_vdma_startread_ctrl(const struct device *const dev,
					       uint8_t *data,
					       uint32_t len)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	LOG_DBG("Starting DMA read, FIFO CTRL 0x00, data %p, len %u", (void *)data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= ~USB_VDMA_LEN_MASK;
	tmp &= ~USB_VDMA_IO;
	tmp &= ~USB_VDMA_TYPE;
	tmp |= (len << USB_VDMA_LEN_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);

	sys_write32((uint32_t)data, cfg->base + USB_VDMA_CXFPS2_OFFSET);

	sys_cache_data_flush_and_invd_range(data, len);
	compiler_barrier();

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp |= USB_VDMA_START;
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);
}

static void udc_bflb_bl61x_vdma_startwrite_ctrl(const struct device *const dev,
						uint8_t *data, const uint32_t len)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	LOG_DBG("Starting DMA write, FIFO CTRL 0x80, data %p, len %u", (void *)data, len);

	sys_cache_data_flush_and_invd_range(data, len);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= ~USB_VDMA_LEN_MASK;
	tmp &= ~USB_VDMA_IO;
	tmp |= USB_VDMA_TYPE;
	tmp |= (len << USB_VDMA_LEN_SHIFT);
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);

	sys_write32((uint32_t)data, cfg->base + USB_VDMA_CXFPS2_OFFSET);

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp |= USB_VDMA_START;
	sys_write32(tmp, cfg->base + USB_VDMA_CXFPS1_OFFSET);
}

static int udc_bflb_bl61x_set_address(const struct device *const dev, const uint8_t addr)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp = 0;

	if ((sys_read32(cfg->base + USB_DEV_ADR_OFFSET) & USB_DEVADR_MASK) != addr) {
		LOG_INF("Set new address %u for %p", addr, dev);
		tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
		tmp &= ~USB_DEVADR_MASK;
		tmp |= addr;
		sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);
	} else {
		LOG_INF("New address %u for %p already set.", addr, dev);
	}

	/* Accept non-control transfer only when address is non-zero */
	tmp = sys_read32(cfg->base + USB_DEV_ADR_OFFSET);
	if (addr == 0) {
		tmp &= ~USB_AFT_CONF;
	} else {
		tmp |= USB_AFT_CONF;
	}
	sys_write32(tmp, cfg->base + USB_DEV_ADR_OFFSET);

	return 0;
}

static uint32_t udc_bflb_bl61x_ctrl_remain(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_VDMA_CXFPS1_OFFSET);
	tmp &= USB_VDMA_LEN_MASK;

	return (tmp >> USB_VDMA_LEN_SHIFT);
}

static void udc_bflb_bl61x_ctrl_setup_start(const struct device *const dev)
{
	struct udc_bflb_bl61x_data *const priv = udc_get_private(dev);
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_bflb_bl61x_fifo *const fifo = &priv->ctrl_fifo;
	struct udc_ep_config *ep_cfg = &cfg->ep_cfg_out[0];
	uint32_t tmp;

	LOG_DBG("Setup packet received");

	/* Mask SETUP interrupt until this one is processed */
	tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
	tmp |= USB_MCX_SETUP_INT;
	sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

	/* Direction of setup is output */
	LOG_DBG("Setting FIFO %u to endpoint 0x%02x", fifo->idx, ep_cfg->addr);
	fifo->ep_cfg = ep_cfg;

	udc_ep_set_busy(&cfg->ep_cfg_in[0], true);
	udc_ep_set_busy(&cfg->ep_cfg_out[0], true);
	udc_bflb_bl61x_vdma_startread_ctrl(dev, (uint8_t *)&priv->setup, sizeof(priv->setup));
}

static void udc_bflb_bl61x_ctrl_setup_end(const struct device *const dev)
{
	struct udc_bflb_bl61x_data *const priv = udc_get_private(dev);
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	sys_cache_data_flush_and_invd_range(&priv->setup, sizeof(priv->setup));

	/* Latency optimization to update the address early enough for next packet */
	if (priv->setup.bRequest == USB_SREQ_SET_ADDRESS) {
		LOG_DBG("Host requested a new address %u, setting immediately", priv->setup.wValue);
		udc_bflb_bl61x_set_address(dev, priv->setup.wValue);
	}

	if (USB_REQTYPE_GET_DIR(priv->setup.bmRequestType) == USB_REQTYPE_DIR_TO_HOST
		|| priv->setup.wLength == 0) {
		priv->ctrl_fifo.ep_cfg = &cfg->ep_cfg_in[0];
	} else {
		priv->ctrl_fifo.ep_cfg = &cfg->ep_cfg_out[0];
	}

	LOG_DBG("Setup packet complete on 0x%02x, next endpoint 0x%02x",
		USB_CONTROL_EP_OUT, priv->ctrl_fifo.ep_cfg->addr);

	udc_setup_received(dev, &priv->setup);

	/* Unmask SETUP interrupt to let it be triggered */
	tmp = sys_read32(cfg->base + USB_DEV_MISG0_OFFSET);
	tmp &= ~USB_MCX_SETUP_INT;
	sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);
}

static uint32_t udc_bflb_bl61x_ep_remain(const struct device *const dev, const uint8_t fifo_idx)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_VDMA_F0PS1_OFFSET
		+ fifo_idx * USB_BL61X_FX_X_OFFSET);
	tmp &= USB_VDMA_LEN_MASK;

	return (tmp >> USB_VDMA_LEN_SHIFT);
}

static void udc_bflb_bl61x_on_xfer_end(const struct device *const dev,
				       struct udc_bflb_bl61x_fifo *fifo)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_ep_config *const ep_cfg = fifo->ep_cfg;
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	struct net_buf *buf;
	int32_t remain;

	if (!udc_ep_is_busy(ep_cfg)) {
		LOG_DBG("Completion for FIFO %u, endpoint 0x%02x while it was not busy",
			fifo->idx, ep_cfg->addr);
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("Missing buffer for transfer 0x%02x", ep_cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return;
	}

	LOG_INF("Buffer complete %u|%u|%u, ep 0x%02x, FIFO %u, size %u",
		udc_get_buf_info(buf)->setup,
		udc_get_buf_info(buf)->data,
		udc_get_buf_info(buf)->status, ep_cfg->addr, fifo->idx,
		buf->size);

	if (udc_get_buf_info(buf)->setup) {
		udc_bflb_bl61x_ctrl_setup_end(dev);
		goto end;
	}

	udc_buf_get(ep_cfg);

	if (ep_idx == 0) {
		remain = udc_bflb_bl61x_ctrl_remain(dev);
	} else {
		remain = udc_bflb_bl61x_ep_remain(dev, fifo->idx);
	}

	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		LOG_DBG("%d bytes received out of %d, %d bytes remaining",
			buf->size - remain, buf->size, remain);
		net_buf_add(buf, ep_cfg->mps - remain);
	} else {
		LOG_DBG("%d bytes sent out of %d, %d unsent",
			buf->len - remain, buf->size, remain);
		net_buf_pull(buf, buf->len);
	}

	if (ep_idx == 0 && udc_get_buf_info(buf)->data) {
		LOG_DBG("Swapping direction for status stage: %u -> %u",
			USB_EP_DIR_IS_IN(ep_cfg->addr), !USB_EP_DIR_IS_IN(ep_cfg->addr));
		if (USB_EP_DIR_IS_IN(ep_cfg->addr)) {
			fifo->ep_cfg = &cfg->ep_cfg_out[0];
		} else {
			fifo->ep_cfg = &cfg->ep_cfg_in[0];
		}
	}

	sys_cache_data_flush_and_invd_range(buf->data, buf->size);
	udc_submit_ep_event(dev, buf, 0);

end:
	/* Transfer done, another next? */
	fifo->xfer_end = false;
	udc_ep_set_busy(ep_cfg, false);
	udc_ep_set_busy(fifo->ep_cfg, false);
	k_work_submit_to_queue(udc_get_work_q(), &fifo->work);
}

static void udc_bflb_bl61x_on_xfer_ready(const struct device *dev,
					 struct udc_bflb_bl61x_fifo *const fifo)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_ep_config *const ep_cfg = fifo->ep_cfg;
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	struct net_buf *buf;
	struct udc_buf_info *bi;

	if (udc_ep_is_busy(ep_cfg)) {
		LOG_DBG("Endpoint 0x%02x busy", ep_cfg->addr);
		return;
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		return;
	}

	bi = udc_get_buf_info(buf);

	if (ep_idx == 0) {
		udc_ep_set_busy(&cfg->ep_cfg_in[0], true);
		udc_ep_set_busy(&cfg->ep_cfg_out[0], true);

		if (bi->setup) {
			/* Handled in interrupt */
		} else if (bi->data && USB_EP_DIR_IS_IN(ep_cfg->addr)) {
			udc_bflb_bl61x_vdma_startwrite_ctrl(dev, buf->data, buf->len);
		} else if (bi->data && USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
			udc_bflb_bl61x_vdma_startread_ctrl(dev, buf->data, buf->size);
		} else if (bi->status) {
			LOG_DBG("Status stage, control transaction complete");
			udc_bflb_bl61x_ctrl_zlp(dev);
			udc_bflb_bl61x_on_xfer_end(dev, fifo);
		} else {
			LOG_ERR("Invalid transfer");
			udc_buf_get(ep_cfg);
			udc_ep_set_busy(ep_cfg, false);
			udc_submit_ep_event(dev, buf, -EINVAL);
		}
	} else {
		udc_ep_set_busy(ep_cfg, true);

		if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
			udc_bflb_bl61x_vdma_startread(dev, fifo->idx, buf->data, buf->size);
		} else if (buf->len > 0) {
			udc_bflb_bl61x_vdma_startwrite(dev, fifo->idx, buf->data, buf->len);
		} else {
			LOG_DBG("IN: empty length, immediately return");
			udc_bflb_bl61x_ep_tx_zlp(dev, ep_idx);
			udc_bflb_bl61x_on_xfer_end(dev, fifo);
		}
	}
}

static void udc_bflb_bl61x_lock(const struct device *const dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_bflb_bl61x_unlock(const struct device *const dev)
{
	udc_unlock_internal(dev);
}

static void udc_bflb_bl61x_fifo_worker(struct k_work *work)
{
	struct udc_bflb_bl61x_fifo *const fifo =
		CONTAINER_OF(work, struct udc_bflb_bl61x_fifo, work);
	const struct device *const dev = fifo->dev;

	udc_bflb_bl61x_lock(dev);

	if (fifo->xfer_end) {
		udc_bflb_bl61x_on_xfer_end(dev, fifo);
	} else {
		udc_bflb_bl61x_on_xfer_ready(dev, fifo);
	}

	udc_bflb_bl61x_unlock(dev);
}

static int udc_bflb_bl61x_ep_enqueue(const struct device *const dev,
				     struct udc_ep_config *const ep_cfg,
				     struct net_buf *buf)
{
	struct udc_bflb_bl61x_fifo *fifo = udc_bflb_bl61x_ep_getfifo(dev, ep_cfg->addr);
	struct udc_buf_info *bi = udc_get_buf_info(buf);

	LOG_DBG("enqueue %p for ep 0x%02x, %u|%u|%u, size %u, len %u, fifo %p %u for 0x%02x, q %u",
		(void *)buf,
		ep_cfg->addr,
		bi->setup, bi->data, bi->status,
		buf->size,
		buf->len,
		(void *)fifo, fifo->idx,
		fifo->ep_cfg->addr,
		sys_sflist_len(&ep_cfg->fifo._queue.data_q));

	udc_buf_put(ep_cfg, buf);

	if (ep_cfg->stat.halted) {
		LOG_DBG("ep 0x%02x halted", ep_cfg->addr);
		return 0;
	}

	k_work_submit_to_queue(udc_get_work_q(), &fifo->work);

	return 0;
}

static int udc_bflb_bl61x_ep_dequeue(const struct device *const dev,
				   struct udc_ep_config *const ep_cfg)
{
	unsigned int lock_key;

	lock_key = irq_lock();

	/* TODO: wait/cancel all ongoing transfers first */
	udc_ep_cancel_queued(dev, ep_cfg);

	irq_unlock(lock_key);

	return 0;
}

static int udc_bflb_bl61x_ep_enable(const struct device *const dev,
				    struct udc_ep_config *const ep_cfg)
{
	struct udc_bflb_bl61x_data *const priv = udc_get_private(dev);
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	struct udc_bflb_bl61x_fifo *fifo = NULL;
	uint8_t num_fifos;
	uint8_t num_blocks;

	if (ep_idx == 0) {
		LOG_DBG("Enabling ep 0x%02x, FIFO CTRL", ep_cfg->addr);
		return 0;
	}

	/* FIFO allocation */
	for (int i = 0; i < USB_BL61X_NUM_FIFO; i++) {
		if (priv->ep_fifo[i].ep_cfg == NULL) {
			fifo = &priv->ep_fifo[i];
			break;
		}
	}
	if (fifo == NULL) {
		LOG_ERR("Not enough FIFOs for endpoint 0x%02x", ep_cfg->addr);
		return -ENOBUFS;
	}

	LOG_DBG("Setting FIFO %u to endpoint 0x%02x", fifo->idx, ep_cfg->addr);
	fifo->ep_cfg = ep_cfg;

	LOG_DBG("Enabling ep 0x%02x, FIFO %u", ep_cfg->addr, fifo->idx);

	/* ISO (not supported yet) needs 1024 bytes (2 FIFOs), everything else is 512 (1 FIFO) */
	num_fifos = 1;
	num_blocks = 1;

	LOG_INF("configuring ep 0x%02x", ep_cfg->addr);

	/* EP config */
	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		udc_bflb_bl61x_ep_set_out_mps(dev, ep_idx, ep_cfg->mps);
		udc_bflb_bl61x_ep_setfifo(dev, ep_idx, USB_BL61X_EP_DIR_OUT, fifo->idx);
	} else {
		udc_bflb_bl61x_ep_set_in_mps(dev, ep_idx, ep_cfg->mps);
		udc_bflb_bl61x_ep_setfifo(dev, ep_idx, USB_BL61X_EP_DIR_IN, fifo->idx);
	}

	/* FIFO config */
	udc_bflb_bl61x_fifo_setep(dev, ep_idx, fifo->idx, USB_BL61X_FIFO_DIR_BID);
	udc_bflb_bl61x_fifo_configure(dev, fifo, num_blocks, true);

	return 0;
}

/* Can't disable */
static int udc_bflb_bl61x_ep_disable(const struct device *const dev,
				     struct udc_ep_config *const ep_cfg)
{
	struct udc_bflb_bl61x_fifo *fifo;

	/* TODO: cancel all ongoing activity on this FIFO */

	fifo = udc_bflb_bl61x_ep_getfifo(dev, ep_cfg->addr);
	if (fifo == NULL) {
		LOG_ERR("No FIFO associated with endpoint 0x%02x", ep_cfg->addr);
		return -EALREADY;
	}

	LOG_DBG("Disable ep 0x%02x", ep_cfg->addr);

	fifo->ep_cfg = NULL;

	return 0;
}

static int udc_bflb_bl61x_ep_set_halt(const struct device *const dev,
				      struct udc_ep_config *const ep_cfg)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	uint32_t tmp;

	LOG_DBG("Set halt ep 0x%02x", ep_cfg->addr);

	if (ep_idx == 0) {
		tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
		tmp |= USB_CX_STL;
		sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
	} else {
		if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
			tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
			tmp |= USB_STL_OEP1;
			sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
			tmp |= USB_STL_IEP1;
			sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
		}
		ep_cfg->stat.halted = true;
	}

	return 0;
}

static int udc_bflb_bl61x_ep_clear_halt(const struct device *const dev,
				        struct udc_ep_config *const ep_cfg)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_bflb_bl61x_fifo *fifo = udc_bflb_bl61x_ep_getfifo(dev, ep_cfg->addr);
	const uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->addr);
	uint32_t tmp;

	LOG_DBG("Clear halt ep 0x%02x", ep_cfg->addr);

	if (ep_idx == 0) {
		tmp = sys_read32(cfg->base + USB_DEV_CXCFE_OFFSET);
		tmp &= ~USB_CX_STL;
		sys_write32(tmp, cfg->base + USB_DEV_CXCFE_OFFSET);
	} else {
		if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
			tmp = sys_read32(cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
			tmp &= ~USB_STL_OEP1;
			sys_write32(tmp, cfg->base + USB_DEV_OUTMPS1_OFFSET
				+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
		} else {
			tmp = sys_read32(cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
			tmp &= ~USB_STL_IEP1;
			sys_write32(tmp, cfg->base + USB_DEV_INMPS1_OFFSET
				+ (ep_idx - 1) * USB_BL61X_XPS_X_OFFSET);
		}

		k_work_submit_to_queue(udc_get_work_q(), &fifo->work);
	}

	ep_cfg->stat.halted = false;

	return 0;
}

static int udc_bflb_bl61x_host_wakeup(const struct device *const dev)
{
	LOG_DBG("Remote wakeup from %p", dev);

	return -ENOTSUP;
}

static int udc_bflb_bl61x_enable(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_bflb_bl61x_data *const priv = udc_get_private(dev);
	uint32_t tmp;

	LOG_DBG("Enable device %s", dev->name);

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_IDDIG_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	/* 'disable global irq' */
	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* force unplug signal */
	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp |= USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_CAP_RMWAKUP;
	tmp |= USB_CHIP_EN_HOV;
	if (cfg->speed_idx < UDC_BUS_SPEED_HS) {
		tmp |= USB_FORCE_FS;
	} else {
		tmp &= ~USB_FORCE_FS;
	}
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp |= USB_SFRST_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* wait for soft reset */
	while ((sys_read32(cfg->base + USB_DEV_CTL_OFFSET)
		& USB_SFRST_HOV) != 0) {
		k_msleep(1);
	}

	tmp = sys_read32(cfg->base + USB_DEV_SMT_OFFSET);
	tmp &= ~USB_SOFMT_MASK;
	if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
		tmp |= USB_BL61X_TIMER_AFTER_RESET_HS;
	} else {
		tmp |= USB_BL61X_TIMER_AFTER_RESET_FS;
	}
	sys_write32(tmp, cfg->base + USB_DEV_SMT_OFFSET);

	/* 'MISGx': Mask Interrupts Source Group x (set is block interrupt)
	 * 'ISGx' : Interrupts Source Group x (set is clear interrupt, read is status)
	 */

	/* clear IRQs */
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_ISG0_OFFSET);
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_ISG1_OFFSET);
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_ISG2_OFFSET);
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_ISG3_OFFSET);

	/* enable IRQs in group 0 for setup */
	tmp = 0xFFFFFFFFU;
	tmp &= ~USB_MCX_SETUP_INT;
	sys_write32(tmp, cfg->base + USB_DEV_MISG0_OFFSET);

	/* disable IRQs in group 1 (fifo interrupts) */
	tmp = 0xFFFFFFFFU;
	sys_write32(tmp, cfg->base + USB_DEV_MISG1_OFFSET);

	/* enable some group 2 interrupts */
	tmp = 0xFFFFFFFFU;
	tmp &= ~USB_SUSP_INT;
	tmp &= ~USB_RESM_INT;
	tmp &= ~USBRST_INT;
	tmp &= ~USB_MRX0BYTE_INT;
	tmp &= ~USB_ISO_SEQ_ERR_INT;
	tmp &= ~USB_ISO_SEQ_ABORT_INT;
	sys_write32(tmp, cfg->base + USB_DEV_MISG2_OFFSET);

	/* enable some group 3 interrupts (DMA completion interrupts, errors) */
	tmp = 0xFFFFFFFFU;
	tmp &= ~USB_MVDMA_CMPLT_CXF;
	tmp &= ~USB_MVDMA_CMPLT_F0;
	tmp &= ~USB_MVDMA_CMPLT_F1;
	tmp &= ~USB_MVDMA_CMPLT_F2;
	tmp &= ~USB_MVDMA_CMPLT_F3;
	tmp &= ~USB_MVDMA_ERROR_CXF;
	tmp &= ~USB_MVDMA_ERROR_F0;
	tmp &= ~USB_MVDMA_ERROR_F1;
	tmp &= ~USB_MVDMA_ERROR_F2;
	tmp &= ~USB_MVDMA_ERROR_F3;
	sys_write32(tmp, cfg->base + USB_DEV_MISG3_OFFSET);

	/* enable group irqs */
	tmp = sys_read32(cfg->base + USB_DEV_MIGR_OFFSET);
	tmp &= ~USB_MINT_G0;
	tmp &= ~USB_MINT_G1;
	tmp &= ~USB_MINT_G2;
	tmp &= ~USB_MINT_G3;
	tmp &= ~USB_MINT_G4;
	sys_write32(tmp, cfg->base + USB_DEV_MIGR_OFFSET);

	/* Select device mode, not OTG (host) mode */
	tmp = sys_read32(cfg->base + USB_GLB_INT_OFFSET);
	tmp |= USB_MHC_INT;
	tmp |= USB_MOTG_INT;
	tmp &= ~USB_MDEV_INT;
	sys_write32(tmp, cfg->base + USB_GLB_INT_OFFSET);

	/* enable 'vdma' (virtual dma) */
	tmp = sys_read32(cfg->base + USB_VDMA_CTRL_OFFSET);
	tmp |= USB_VDMA_EN;
	sys_write32(tmp, cfg->base + USB_VDMA_CTRL_OFFSET);

	/* disable force unplug signal */
	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp &= ~USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	/* 'enable global irq' */
	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp |= USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	/* Give it time to complete the High-Speed detection handshake */
	priv->reset_expiration = sys_timepoint_calc(USB_BL61X_TIMER_AFTER_RESET_T);

	return 0;
}

static int udc_bflb_bl61x_disable(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	tmp = sys_read32(cfg->base + USB_DEV_CTL_OFFSET);
	tmp &= ~USB_GLINT_EN_HOV;
	sys_write32(tmp, cfg->base + USB_DEV_CTL_OFFSET);

	tmp = sys_read32(cfg->base + USB_PHY_TST_OFFSET);
	tmp |= USB_UNPLUG;
	sys_write32(tmp, cfg->base + USB_PHY_TST_OFFSET);

	return 0;
}

static void udc_bflb_bl61x_clock_init(const struct device *const dev)
{
	uint32_t tmp;

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_PU_USBPLL_MMDIV_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp &= ~GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);

	k_usleep(5);

	tmp = sys_read32(GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
	tmp |= GLB_USBPLL_RSTB_MSK;
	sys_write32(tmp, GLB_BASE + GLB_WIFI_PLL_CFG10_OFFSET);
}

static void udc_bflb_bl61x_phy_init(const struct device *const dev)
{
	uint32_t tmp;

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_XTLSEL_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp |= PDS_REG_PU_USB20_PSW_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp |= PDS_REG_USB_PHY_PONRST_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	k_usleep(1);

	/* enable reset */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp &= ~PDS_REG_USB_SW_RST_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_usleep(1);

	/* unsuspend */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_EXT_SUSP_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_msleep(5);

	/* disable reset */
	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp |= PDS_REG_USB_SW_RST_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	k_msleep(5);
}

static int udc_bflb_bl61x_init(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	int ret;

	udc_bflb_bl61x_clock_init(dev);
	udc_bflb_bl61x_phy_init(dev);

	/* set endpoints and fifo mappings to disabled
	 * we have 5 total fifos (4 regular, one control)
	 * disabled is 0xf value
	 * see usb_v2_reg.h for format
	 */
	sys_write32(0xFFFFFFFFU, cfg->base + USB_DEV_EPMAP0_OFFSET);

	udc_bflb_bl61x_fifo_setep(dev, USB_BL61X_FIFO_EP_NONE, 0, USB_BL61X_FIFO_DIR_OUT);
	udc_bflb_bl61x_fifo_setep(dev, USB_BL61X_FIFO_EP_NONE, 1, USB_BL61X_FIFO_DIR_OUT);
	udc_bflb_bl61x_fifo_setep(dev, USB_BL61X_FIFO_EP_NONE, 2, USB_BL61X_FIFO_DIR_OUT);
	udc_bflb_bl61x_fifo_setep(dev, USB_BL61X_FIFO_EP_NONE, 3, USB_BL61X_FIFO_DIR_OUT);

	udc_bflb_bl61x_fifo_reset_ctrl(dev);
	udc_bflb_bl61x_fifo_reset(dev, 0);
	udc_bflb_bl61x_fifo_reset(dev, 1);
	udc_bflb_bl61x_fifo_reset(dev, 2);
	udc_bflb_bl61x_fifo_reset(dev, 3);

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 64, 0);
	if (ret < 0) {
		LOG_ERR("Failed to enable control endpoint");
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 64, 0);
	if (ret < 0) {
		LOG_ERR("Failed to enable control endpoint");
		return ret;
	}

	cfg->irq_enable_func(dev);

	LOG_INF("Initialized %s", dev->name);

	return 0;
}

/* Shut down the controller completely */
static int udc_bflb_bl61x_shutdown(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	uint32_t tmp;

	cfg->irq_disable_func(dev);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_XTLSEL_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_PU_USB20_PSW_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
	tmp &= ~PDS_REG_USB_PHY_PONRST_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

	tmp = sys_read32(PDS_BASE + PDS_USB_CTL_OFFSET);
	tmp &= ~PDS_REG_USB_EXT_SUSP_N_MSK;
	sys_write32(tmp, PDS_BASE + PDS_USB_CTL_OFFSET);

	return 0;
}

static int udc_bflb_bl61x_driver_preinit(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_bflb_bl61x_data *const priv = udc_get_private(dev);
	struct udc_data *const data = dev->data;
	int err;

	k_mutex_init(&data->mutex);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;

	if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
		data->caps.hs = true;
	}

	for (int i = 0; i < cfg->num_of_eps; i++) {
		cfg->ep_cfg_out[i].caps.out = 1;
		cfg->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;

		if (USB_EP_GET_IDX(cfg->ep_cfg_out[i].addr) == 0) {
			cfg->ep_cfg_out[i].caps.control = 1;
			cfg->ep_cfg_out[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_out[i].caps.bulk = 1;
			cfg->ep_cfg_out[i].caps.interrupt = 1;
			cfg->ep_cfg_out[i].caps.iso = 1;
			/* TODO support 2 FIFO/ep to reach 1024 */
			cfg->ep_cfg_out[i].caps.mps = 512;
		}

		err = udc_register_ep(dev, &cfg->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < cfg->num_of_eps; i++) {
		cfg->ep_cfg_in[i].caps.in = 1;
		cfg->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;

		if (USB_EP_GET_IDX(cfg->ep_cfg_in[i].addr) == 0) {
			cfg->ep_cfg_in[i].caps.control = 1;
			cfg->ep_cfg_in[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_in[i].caps.bulk = 1;
			cfg->ep_cfg_in[i].caps.interrupt = 1;
			cfg->ep_cfg_in[i].caps.iso = 1;
			/* TODO support 2 FIFO/ep to reach 1024 */
			cfg->ep_cfg_in[i].caps.mps = 512;
		}

		err = udc_register_ep(dev, &cfg->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	priv->ctrl_fifo.ep_cfg = &cfg->ep_cfg_out[0];
	priv->ctrl_fifo.dev = dev;
	priv->ctrl_fifo.idx = 0xFF;
	priv->ctrl_fifo.xfer_end = false;
	k_work_init(&priv->ctrl_fifo.work, &udc_bflb_bl61x_fifo_worker);

	for (int i = 0; i < USB_BL61X_NUM_FIFO; i++) {
		priv->ep_fifo[i].ep_cfg = NULL;
		priv->ep_fifo[i].dev = dev;
		priv->ep_fifo[i].idx = i;
		priv->ep_fifo[i].xfer_end = false;
		k_work_init(&priv->ep_fifo[i].work, &udc_bflb_bl61x_fifo_worker);
	}

	LOG_INF("Device %s (max. speed %d)", dev->name, cfg->speed_idx);

	return 0;
}

static void udc_bflb_bl61x_isr(const struct device *const dev)
{
	const struct udc_bflb_bl61x_config *const cfg = dev->config;
	struct udc_bflb_bl61x_data *const priv = udc_get_private(dev);
	struct udc_bflb_bl61x_fifo *fifo;
	uint32_t glb_intstatus;
	uint32_t dev_intstatus;
	uint32_t group_intstatus;
	uint32_t tmp;

	glb_intstatus = sys_read32(cfg->base + USB_GLB_ISR_OFFSET);
	sys_write32(glb_intstatus, cfg->base + USB_GLB_ISR_OFFSET);

	if (glb_intstatus & USB_DEV_INT) {
		dev_intstatus = sys_read32(cfg->base + USB_DEV_IGR_OFFSET);

		/* Control endpoint interrupts */
		if (dev_intstatus & USB_INT_G0) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG0_OFFSET);
			sys_write32(group_intstatus, cfg->base + USB_DEV_ISG0_OFFSET);

			if (group_intstatus & USB_CX_COMABT_INT) {
				udc_submit_event(dev, UDC_EVT_ERROR, -ECANCELED);
				LOG_ERR("Control command abort");
			}
			if (group_intstatus & USB_CX_SETUP_INT) {
				udc_bflb_bl61x_ctrl_setup_start(dev);
			}
			if (group_intstatus & USB_CX_COMFAIL_INT) {
				udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
				LOG_ERR("Control command Fail");
			}
		}

		/* FIFO interrupts */
		if (dev_intstatus & USB_INT_G1) {
			/* Nothing we care about in group 1 */
		}

		/* Link interrupts */
		if (dev_intstatus & USB_INT_G2) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG2_OFFSET);
			sys_write32(group_intstatus, cfg->base + USB_DEV_ISG2_OFFSET);

			/* suspended */
			if (group_intstatus & USB_SUSP_INT) {
				udc_bflb_bl61x_fifo_reset_ctrl(dev);
				udc_bflb_bl61x_fifo_reset(dev, 0);
				udc_bflb_bl61x_fifo_reset(dev, 1);
				udc_bflb_bl61x_fifo_reset(dev, 2);
				udc_bflb_bl61x_fifo_reset(dev, 3);

				udc_set_suspended(dev, true);
				udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			}

			/* resumed */
			if (group_intstatus & USB_RESM_INT) {
				udc_set_suspended(dev, false);
				udc_submit_event(dev, UDC_EVT_RESUME, 0);
			}

			if (group_intstatus & USBRST_INT) {
				udc_bflb_bl61x_fifo_reset_ctrl(dev);
				udc_bflb_bl61x_fifo_reset(dev, 0);
				udc_bflb_bl61x_fifo_reset(dev, 1);
				udc_bflb_bl61x_fifo_reset(dev, 2);
				udc_bflb_bl61x_fifo_reset(dev, 3);

				tmp = sys_read32(cfg->base + USB_DEV_SMT_OFFSET);
				tmp &= ~USB_SOFMT_MASK;
				if (cfg->speed_idx == UDC_BUS_SPEED_HS) {
					tmp |= USB_BL61X_TIMER_AFTER_RESET_HS;
				} else {
					tmp |= USB_BL61X_TIMER_AFTER_RESET_FS;
				}
				sys_write32(tmp, cfg->base + USB_DEV_SMT_OFFSET);

				/* Give it time to complete the High-Speed detection handshake */
				priv->reset_expiration =
					sys_timepoint_calc(USB_BL61X_TIMER_AFTER_RESET_T);

				udc_submit_event(dev, UDC_EVT_RESET, 0);
			}

			if (group_intstatus & USB_RX0BYTE_INT) {
				udc_bflb_bl61x_ep_rx_zlp(dev);
			}

			if (group_intstatus & USB_ISO_SEQ_ERR_INT) {
				udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
				LOG_ERR("Isosynchronous sequence error");
			}

			if (group_intstatus & USB_ISO_SEQ_ABORT_INT) {
				udc_submit_event(dev, UDC_EVT_ERROR, -ECANCELED);
				LOG_ERR("Isosynchronous sequence aborted");
			}
		}

		/* DMA interrupts */
		if (dev_intstatus & USB_INT_G3) {
			group_intstatus = sys_read32(cfg->base + USB_DEV_ISG3_OFFSET);
			sys_write32(group_intstatus, cfg->base + USB_DEV_ISG3_OFFSET);

			if (group_intstatus & USB_VDMA_CMPLT_CXF) {
				fifo = &priv->ctrl_fifo;

				fifo->xfer_end = true;
				k_work_submit_to_queue(udc_get_work_q(), &fifo->work);
			}

			for (int i = 0; i < USB_BL61X_NUM_FIFO; i++) {
				if (group_intstatus & USB_VDMA_CMPLT_FX(i)) {
					fifo = &priv->ep_fifo[i];

					fifo->xfer_end = true;
					k_work_submit_to_queue(udc_get_work_q(), &fifo->work);
				}
			}
		}

		if (dev_intstatus & USB_INT_G4) {
			/* Nothing we care about in group 4 */
		}
	}
}

static const struct udc_api udc_bflb_bl61x_api = {
	.lock = udc_bflb_bl61x_lock,
	.unlock = udc_bflb_bl61x_unlock,
	.device_speed = udc_bflb_bl61x_device_speed,
	.init = udc_bflb_bl61x_init,
	.enable = udc_bflb_bl61x_enable,
	.disable = udc_bflb_bl61x_disable,
	.shutdown = udc_bflb_bl61x_shutdown,
	.set_address = udc_bflb_bl61x_set_address,
	.host_wakeup = udc_bflb_bl61x_host_wakeup,
	.ep_enable = udc_bflb_bl61x_ep_enable,
	.ep_disable = udc_bflb_bl61x_ep_disable,
	.ep_set_halt = udc_bflb_bl61x_ep_set_halt,
	.ep_clear_halt = udc_bflb_bl61x_ep_clear_halt,
	.ep_enqueue = udc_bflb_bl61x_ep_enqueue,
	.ep_dequeue = udc_bflb_bl61x_ep_dequeue,
};

#define UDC_BFLB_BL61X_DEVICE_DEFINE(n)						\
	static void udc_irq_enable_func##n(const struct device *const dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    udc_bflb_bl61x_isr,					\
			    DEVICE_DT_INST_GET(n), 0);				\
										\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void udc_irq_disable_func##n(const struct device *const dev)	\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
										\
	static struct udc_ep_config						\
		ep_cfg_out[DT_INST_PROP(n, num_bidir_endpoints)];		\
										\
	static struct udc_ep_config						\
		ep_cfg_in[DT_INST_PROP(n, num_bidir_endpoints)];		\
										\
	static const struct udc_bflb_bl61x_config udc_bflb_bl61x_config_##n = {	\
		.base = DT_INST_REG_ADDR(n),					\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),		\
		.ep_cfg_in = ep_cfg_in,						\
		.ep_cfg_out = ep_cfg_out,					\
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),	\
		.irq_enable_func = udc_irq_enable_func##n,			\
		.irq_disable_func = udc_irq_disable_func##n,			\
	};									\
										\
	static struct udc_bflb_bl61x_data udc_priv_##n;				\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, udc_bflb_bl61x_driver_preinit, NULL,		\
			      &udc_data_##n, &udc_bflb_bl61x_config_##n,	\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &udc_bflb_bl61x_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_BFLB_BL61X_DEVICE_DEFINE)
