/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dwc2

#include "uhc_common.h"
#include "uhc_dwc2.h"

#include <string.h>
#include <stdio.h>

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/usb/usb_ch9.h>
#include <usb_dwc2_hw.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_dwc2, CONFIG_UHC_DRIVER_LOG_LEVEL);

enum dwc2_drv_event_type {
	/* USB connection speed determined after bus reset */
	DWC2_DRV_EVT_ENUM_DONE,
	/* Trigger next transfer, must not be used for control OUT */
	DWC2_DRV_EVT_XFER,
	/* Setup packet received */
	DWC2_DRV_EVT_SETUP,
	/* Transaction on endpoint is finished */
	DWC2_DRV_EVT_EP_FINISHED,
	/* Remote Wakeup should be initiated */
	DWC2_DRV_EVT_REMOTE_WAKEUP,
	/* Core should enter hibernation */
	DWC2_DRV_EVT_ENTER_HIBERNATION,
	/* Core should exit hibernation due to bus reset */
	DWC2_DRV_EVT_HIBERNATION_EXIT_BUS_RESET,
	/* Core should exit hibernation due to host resume */
	DWC2_DRV_EVT_HIBERNATION_EXIT_HOST_RESUME,
};

/* Minimum RX FIFO size in 32-bit words considering the largest used OUT packet
 * of 512 bytes. The value must be adjusted according to the number of OUT
 * endpoints.
 */
#define UHC_DWC2_GRXFSIZ_FS_DEFAULT	(15U + 512U/4U)
/* Default Rx FIFO size in 32-bit words calculated to support High-Speed with:
 *   * 1 control endpoint in Completer/Buffer DMA mode: 13 locations
 *   * Global OUT NAK: 1 location
 *   * Space for 3 * 1024 packets: ((1024/4) + 1) * 3 = 774 locations
 * Driver adds 2 locations for each OUT endpoint to this value.
 */
#define UHC_DWC2_GRXFSIZ_HS_DEFAULT	(13 + 1 + 774)

/* TX FIFO0 depth in 32-bit words (used by control IN endpoint)
 * Try 2 * bMaxPacketSize0 to allow simultaneous operation with a fallback to
 * whatever is available when 2 * bMaxPacketSize0 is not possible.
 */
#define UHC_DWC2_FIFO0_DEPTH		(2 * 16U)

/* Get Data FIFO access register */
#define UHC_DWC2_EP_FIFO(base, idx)	((mem_addr_t)base + 0x1000 * (idx + 1))

enum dwc2_suspend_type {
	DWC2_SUSPEND_NO_POWER_SAVING,
	DWC2_SUSPEND_HIBERNATION,
};

/* Registers that have to be stored before Partial Power Down or Hibernation */
struct dwc2_reg_backup {
	uint32_t gotgctl;
	uint32_t gahbcfg;
	uint32_t gusbcfg;
	uint32_t gintmsk;
	uint32_t grxfsiz;
	uint32_t gnptxfsiz;
	uint32_t gi2cctl;
	uint32_t glpmcfg;
	uint32_t gdfifocfg;
	union {
		uint32_t dptxfsiz[15];
		uint32_t dieptxf[15];
	};
	uint32_t dcfg;
	uint32_t dctl;
	uint32_t diepmsk;
	uint32_t doepmsk;
	uint32_t daintmsk;
	uint32_t diepctl[16];
	uint32_t dieptsiz[16];
	uint32_t diepdma[16];
	uint32_t doepctl[16];
	uint32_t doeptsiz[16];
	uint32_t doepdma[16];
	uint32_t pcgcctl;
};

/* Driver private data per instance */
struct uhc_dwc2_data {
	struct k_spinlock lock;
	struct k_thread thread_data;
	/* Main events the driver thread waits for */
	struct k_event drv_evt;
	/* Endpoint is considered disabled when there is no active transfer */
	struct k_event ep_disabled;
	/* Transfer triggers (IN on bits 0-15, OUT on bits 16-31) */
	atomic_t xfer_new;
	/* Finished transactions (IN on bits 0-15, OUT on bits 16-31) */
	atomic_t xfer_finished;
	struct dwc2_reg_backup backup;
	uint32_t ghwcfg1;
	uint32_t max_xfersize;
	uint32_t max_pktcnt;
	uint32_t tx_len[16];
	uint32_t rx_siz[16];
	/* Isochronous endpoint enabled (IN on bits 0-15, OUT on bits 16-31) */
	uint32_t iso_enabled;
	uint16_t iso_in_rearm;
	uint16_t ep_out_disable;
	uint16_t ep_out_stall;
	uint16_t txf_set;
	uint16_t pending_tx_flush;
	uint16_t dfifodepth;
	uint16_t rxfifo_depth;
	uint16_t max_txfifo_depth[16];
	uint16_t sof_num;
	/* Configuration flags */
	unsigned int dynfifosizing : 1;
	unsigned int bufferdma : 1;
	unsigned int syncrst : 1;
	/* Defect workarounds */
	unsigned int wa_essregrestored : 1;
	/* Runtime state flags */
	unsigned int hibernated : 1;
	unsigned int enumdone : 1;
	unsigned int enumspd : 2;
	unsigned int pending_dout_feed : 1;
	enum dwc2_suspend_type suspend_type;
	/* Number of endpoints including control endpoint */
	uint8_t numdeveps;
	/* Number of IN endpoints including control endpoint */
	uint8_t ineps;
	/* Number of OUT endpoints including control endpoint */
	uint8_t outeps;
	uint8_t setup[8];
};

#if defined(CONFIG_PINCTRL)
#include <zephyr/drivers/pinctrl.h>

static int dwc2_init_pinctrl(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	const struct pinctrl_dev_config *const pcfg = config->pcfg;
	int ret = 0;

	if (pcfg == NULL) {
		LOG_INF("Skip pinctrl configuration");
		return 0;
	}

	ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to apply default pinctrl state (%d)", ret);
	}

	LOG_DBG("Apply pinctrl");

	return ret;
}
#else
static int dwc2_init_pinctrl(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}
#endif

static inline struct usb_dwc2_reg *dwc2_get_base(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;

	return config->base;
}

static void dwc2_wait_for_bit(const struct device *dev,
			      mem_addr_t addr, uint32_t bit)
{
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(100));

	/* This could potentially be converted to use proper synchronization
	 * primitives instead of busy looping, but the number of interrupt bits
	 * this function can be waiting for is rather high.
	 *
	 * Busy looping is most likely fine unless profiling shows otherwise.
	 */
	while (!(sys_read32(addr) & bit)) {
		if (dwc2_quirk_is_phy_clk_off(dev)) {
			/* No point in waiting, because the bit can only be set
			 * when the PHY is actively clocked.
			 */
			return;
		}

		if (sys_timepoint_expired(timeout)) {
			LOG_ERR("Timeout waiting for bit 0x%08X at 0x%08X",
				bit, (uint32_t)addr);
			return;
		}
	}
}

static void dwc2_backup_registers(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct dwc2_reg_backup *backup = &priv->backup;

	backup->gotgctl = sys_read32((mem_addr_t)&base->gotgctl);
	backup->gahbcfg = sys_read32((mem_addr_t)&base->gahbcfg);
	backup->gusbcfg = sys_read32((mem_addr_t)&base->gusbcfg);
	backup->gintmsk = sys_read32((mem_addr_t)&base->gintmsk);
	backup->grxfsiz = sys_read32((mem_addr_t)&base->grxfsiz);
	backup->gnptxfsiz = sys_read32((mem_addr_t)&base->gnptxfsiz);
	backup->gi2cctl = sys_read32((mem_addr_t)&base->gi2cctl);
	backup->glpmcfg = sys_read32((mem_addr_t)&base->glpmcfg);
	backup->gdfifocfg = sys_read32((mem_addr_t)&base->gdfifocfg);

	for (uint8_t i = 1U; i < priv->ineps; i++) {
		backup->dieptxf[i - 1] = sys_read32((mem_addr_t)&base->dieptxf[i - 1]);
	}

	backup->dcfg = sys_read32((mem_addr_t)&base->dcfg);
	backup->dctl = sys_read32((mem_addr_t)&base->dctl);
	backup->diepmsk = sys_read32((mem_addr_t)&base->diepmsk);
	backup->doepmsk = sys_read32((mem_addr_t)&base->doepmsk);
	backup->daintmsk = sys_read32((mem_addr_t)&base->daintmsk);

	for (uint8_t i = 0U; i < 16; i++) {
		uint32_t epdir = usb_dwc2_get_ghwcfg1_epdir(priv->ghwcfg1, i);

		if (epdir == USB_DWC2_GHWCFG1_EPDIR_IN || epdir == USB_DWC2_GHWCFG1_EPDIR_BDIR) {
			backup->diepctl[i] = sys_read32((mem_addr_t)&base->in_ep[i].diepctl);
			if (backup->diepctl[i] & USB_DWC2_DEPCTL_DPID) {
				backup->diepctl[i] |= USB_DWC2_DEPCTL_SETD1PID;
			} else {
				backup->diepctl[i] |= USB_DWC2_DEPCTL_SETD0PID;
			}
			backup->dieptsiz[i] = sys_read32((mem_addr_t)&base->in_ep[i].dieptsiz);
			backup->diepdma[i] = sys_read32((mem_addr_t)&base->in_ep[i].diepdma);
		}

		if (epdir == USB_DWC2_GHWCFG1_EPDIR_OUT || epdir == USB_DWC2_GHWCFG1_EPDIR_BDIR) {
			backup->doepctl[i] = sys_read32((mem_addr_t)&base->out_ep[i].doepctl);
			if (backup->doepctl[i] & USB_DWC2_DEPCTL_DPID) {
				backup->doepctl[i] |= USB_DWC2_DEPCTL_SETD1PID;
			} else {
				backup->doepctl[i] |= USB_DWC2_DEPCTL_SETD0PID;
			}
			backup->doeptsiz[i] = sys_read32((mem_addr_t)&base->out_ep[i].doeptsiz);
			backup->doepdma[i] = sys_read32((mem_addr_t)&base->out_ep[i].doepdma);
		}
	}

	backup->pcgcctl = sys_read32((mem_addr_t)&base->pcgcctl);
}

static void dwc2_restore_essential_registers(const struct device *dev,
					     bool rwup, bool bus_reset)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct dwc2_reg_backup *backup = &priv->backup;
	uint32_t pcgcctl = backup->pcgcctl & USB_DWC2_PCGCCTL_RESTOREVALUE_MASK;

	if (usb_dwc2_get_pcgcctl_p2hd_dev_enum_spd(pcgcctl) ==
	    USB_DWC2_PCGCCTL_P2HD_DEV_ENUM_SPD_HS) {
		pcgcctl |= BIT(17);
	}

	sys_write32(backup->glpmcfg, (mem_addr_t)&base->glpmcfg);
	sys_write32(backup->gi2cctl, (mem_addr_t)&base->gi2cctl);
	sys_write32(pcgcctl, (mem_addr_t)&base->pcgcctl);

	sys_write32(backup->gahbcfg | USB_DWC2_GAHBCFG_GLBINTRMASK,
		    (mem_addr_t)&base->gahbcfg);

	sys_write32(0xFFFFFFFFUL, (mem_addr_t)&base->gintsts);
	sys_write32(USB_DWC2_GINTSTS_RSTRDONEINT, (mem_addr_t)&base->gintmsk);

	sys_write32(backup->gusbcfg, (mem_addr_t)&base->gusbcfg);
	sys_write32(backup->dcfg, (mem_addr_t)&base->dcfg);

	if (bus_reset) {
		sys_write32(backup->dcfg, (mem_addr_t)&base->dcfg);
	}

	if (!rwup) {
		pcgcctl |= USB_DWC2_PCGCCTL_RESTOREMODE | USB_DWC2_PCGCCTL_RSTPDWNMODULE;
	}
	sys_write32(pcgcctl, (mem_addr_t)&base->pcgcctl);
	k_busy_wait(1);

	pcgcctl |= USB_DWC2_PCGCCTL_ESSREGRESTORED;
	sys_write32(pcgcctl, (mem_addr_t)&base->pcgcctl);

	/* Note: in Remote Wakeup case 15 ms max signaling time starts now */

	/* Wait for Restore Done Interrupt */
	dwc2_wait_for_bit(dev, (mem_addr_t)&base->gintsts, USB_DWC2_GINTSTS_RSTRDONEINT);

	if (priv->wa_essregrestored) {
		pcgcctl &= ~USB_DWC2_PCGCCTL_ESSREGRESTORED;
		sys_write32(pcgcctl, (mem_addr_t)&base->pcgcctl);
		k_busy_wait(1);
	}

	if (!bus_reset) {
		sys_write32(0xFFFFFFFFUL, (mem_addr_t)&base->gintsts);
	}
}

static void dwc2_restore_device_registers(const struct device *dev, bool rwup)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct dwc2_reg_backup *backup = &priv->backup;

	sys_write32(backup->gotgctl, (mem_addr_t)&base->gotgctl);
	sys_write32(backup->gahbcfg, (mem_addr_t)&base->gahbcfg);
	sys_write32(backup->gusbcfg, (mem_addr_t)&base->gusbcfg);
	sys_write32(backup->gintmsk, (mem_addr_t)&base->gintmsk);
	sys_write32(backup->grxfsiz, (mem_addr_t)&base->grxfsiz);
	sys_write32(backup->gnptxfsiz, (mem_addr_t)&base->gnptxfsiz);
	sys_write32(backup->gdfifocfg, (mem_addr_t)&base->gdfifocfg);

	for (uint8_t i = 1U; i < priv->ineps; i++) {
		sys_write32(backup->dieptxf[i - 1], (mem_addr_t)&base->dieptxf[i - 1]);
	}

	if (!rwup) {
		sys_write32(backup->dctl, (mem_addr_t)&base->dctl);
	}

	sys_write32(backup->diepmsk, (mem_addr_t)&base->diepmsk);
	sys_write32(backup->doepmsk, (mem_addr_t)&base->doepmsk);
	sys_write32(backup->daintmsk, (mem_addr_t)&base->daintmsk);

	for (uint8_t i = 0U; i < 16; i++) {
		uint32_t epdir = usb_dwc2_get_ghwcfg1_epdir(priv->ghwcfg1, i);

		if (epdir == USB_DWC2_GHWCFG1_EPDIR_IN || epdir == USB_DWC2_GHWCFG1_EPDIR_BDIR) {
			sys_write32(backup->dieptsiz[i], (mem_addr_t)&base->in_ep[i].dieptsiz);
			sys_write32(backup->diepdma[i], (mem_addr_t)&base->in_ep[i].diepdma);
			sys_write32(backup->diepctl[i], (mem_addr_t)&base->in_ep[i].diepctl);
		}

		if (epdir == USB_DWC2_GHWCFG1_EPDIR_OUT || epdir == USB_DWC2_GHWCFG1_EPDIR_BDIR) {
			sys_write32(backup->doeptsiz[i], (mem_addr_t)&base->out_ep[i].doeptsiz);
			sys_write32(backup->doepdma[i], (mem_addr_t)&base->out_ep[i].doepdma);
			sys_write32(backup->doepctl[i], (mem_addr_t)&base->out_ep[i].doepctl);
		}
	}
}

static void dwc2_unset_unused_fifo(const struct device *dev)
{
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct uhc_ep_config *tmp;

	for (uint8_t i = priv->ineps - 1U; i > 0; i--) {
		tmp = uhc_get_ep_cfg(dev, i | USB_EP_DIR_IN);

		if (tmp->stat.enabled && (priv->txf_set & BIT(i))) {
			return;
		}

		if (!tmp->stat.enabled && (priv->txf_set & BIT(i))) {
			priv->txf_set &= ~BIT(i);
		}
	}
}

static int dwc2_core_soft_reset(const struct device *dev)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	mem_addr_t grstctl_reg = (mem_addr_t)&base->grstctl;
	const unsigned int csr_timeout_us = 10000UL;
	uint32_t cnt = 0UL;

	/* Check AHB master idle state */
	while (!(sys_read32(grstctl_reg) & USB_DWC2_GRSTCTL_AHBIDLE)) {
		k_busy_wait(1);

		if (++cnt > csr_timeout_us) {
			LOG_ERR("Wait for AHB idle timeout, GRSTCTL 0x%08x",
				sys_read32(grstctl_reg));
			return -EIO;
		}
	}

	/* Apply Core Soft Reset */
	sys_write32(USB_DWC2_GRSTCTL_CSFTRST, grstctl_reg);

	cnt = 0UL;
	do {
		if (++cnt > csr_timeout_us) {
			LOG_ERR("Wait for CSR done timeout, GRSTCTL 0x%08x",
				sys_read32(grstctl_reg));
			return -EIO;
		}

		k_busy_wait(1);

		if (dwc2_quirk_is_phy_clk_off(dev)) {
			/* Software reset won't finish without PHY clock */
			return -EIO;
		}
	} while (sys_read32(grstctl_reg) & USB_DWC2_GRSTCTL_CSFTRST &&
		 !(sys_read32(grstctl_reg) & USB_DWC2_GRSTCTL_CSFTRSTDONE));

	/* CSFTRSTDONE is W1C so the write must have the bit set to clear it */
	sys_clear_bits(grstctl_reg, USB_DWC2_GRSTCTL_CSFTRST);

	return 0;
}

static int uhc_dwc2_init_controller(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const base = config->base;
	mem_addr_t grxfsiz_reg = (mem_addr_t)&base->grxfsiz;
	mem_addr_t gahbcfg_reg = (mem_addr_t)&base->gahbcfg;
	mem_addr_t gusbcfg_reg = (mem_addr_t)&base->gusbcfg;
	mem_addr_t dcfg_reg = (mem_addr_t)&base->dcfg;
	uint32_t gsnpsid;
	uint32_t dcfg;
	uint32_t gusbcfg;
	uint32_t gahbcfg;
	uint32_t ghwcfg2;
	uint32_t ghwcfg3;
	uint32_t ghwcfg4;
	uint32_t val;
	int ret;
	bool hs_phy;

	ret = dwc2_core_soft_reset(dev);
	if (ret) {
		return ret;
	}

	/* Enable RTL workarounds based on controller revision */
	gsnpsid = sys_read32((mem_addr_t)&base->gsnpsid);
	priv->wa_essregrestored = gsnpsid < USB_DWC2_GSNPSID_REV_5_00A;

	priv->ghwcfg1 = sys_read32((mem_addr_t)&base->ghwcfg1);
	ghwcfg2 = sys_read32((mem_addr_t)&base->ghwcfg2);
	ghwcfg3 = sys_read32((mem_addr_t)&base->ghwcfg3);
	ghwcfg4 = sys_read32((mem_addr_t)&base->ghwcfg4);

	if (!(ghwcfg4 & USB_DWC2_GHWCFG4_DEDFIFOMODE)) {
		LOG_ERR("Only dedicated TX FIFO mode is supported");
		return -ENOTSUP;
	}

	/*
	 * Force device mode as we do no support role changes.
	 * Wait 25ms for the change to take effect.
	 */
	gusbcfg = USB_DWC2_GUSBCFG_FORCEDEVMODE;
	sys_write32(gusbcfg, gusbcfg_reg);
	k_msleep(25);

	/* Buffer DMA is always supported in Internal DMA mode.
	 * TODO: check and support descriptor DMA if available
	 */
	priv->bufferdma = (usb_dwc2_get_ghwcfg2_otgarch(ghwcfg2) ==
			   USB_DWC2_GHWCFG2_OTGARCH_INTERNALDMA);

	if (!IS_ENABLED(CONFIG_UHC_DWC2_DMA)) {
		priv->bufferdma = 0;
	} else if (priv->bufferdma) {
		LOG_WRN("Experimental DMA enabled");
	}

	if (ghwcfg2 & USB_DWC2_GHWCFG2_DYNFIFOSIZING) {
		LOG_DBG("Dynamic FIFO Sizing is enabled");
		priv->dynfifosizing = true;
	}

	if (IS_ENABLED(CONFIG_UHC_DWC2_HIBERNATION) &&
	    ghwcfg4 & USB_DWC2_GHWCFG4_HIBERNATION) {
		LOG_INF("Hibernation enabled");
		priv->suspend_type = DWC2_SUSPEND_HIBERNATION;
	} else {
		priv->suspend_type = DWC2_SUSPEND_NO_POWER_SAVING;
	}

	/* Get the number or endpoints and IN endpoints we can use later */
	priv->numdeveps = usb_dwc2_get_ghwcfg2_numdeveps(ghwcfg2) + 1U;
	priv->ineps = usb_dwc2_get_ghwcfg4_ineps(ghwcfg4) + 1U;
	LOG_DBG("Number of endpoints (NUMDEVEPS + 1) %u", priv->numdeveps);
	LOG_DBG("Number of IN endpoints (INEPS + 1) %u", priv->ineps);

	LOG_DBG("Number of periodic IN endpoints (NUMDEVPERIOEPS) %u",
		usb_dwc2_get_ghwcfg4_numdevperioeps(ghwcfg4));
	LOG_DBG("Number of additional control endpoints (NUMCTLEPS) %u",
		usb_dwc2_get_ghwcfg4_numctleps(ghwcfg4));

	LOG_DBG("OTG architecture (OTGARCH) %u, mode (OTGMODE) %u",
		usb_dwc2_get_ghwcfg2_otgarch(ghwcfg2),
		usb_dwc2_get_ghwcfg2_otgmode(ghwcfg2));

	priv->dfifodepth = usb_dwc2_get_ghwcfg3_dfifodepth(ghwcfg3);
	LOG_DBG("DFIFO depth (DFIFODEPTH) %u bytes", priv->dfifodepth * 4);

	priv->max_pktcnt = GHWCFG3_PKTCOUNT(usb_dwc2_get_ghwcfg3_pktsizewidth(ghwcfg3));
	priv->max_xfersize = GHWCFG3_XFERSIZE(usb_dwc2_get_ghwcfg3_xfersizewidth(ghwcfg3));
	LOG_DBG("Max packet count %u, Max transfer size %u",
		priv->max_pktcnt, priv->max_xfersize);

	LOG_DBG("Vendor Control interface support enabled: %s",
		(ghwcfg3 & USB_DWC2_GHWCFG3_VNDCTLSUPT) ? "true" : "false");

	LOG_DBG("PHY interface type: FSPHYTYPE %u, HSPHYTYPE %u, DATAWIDTH %u",
		usb_dwc2_get_ghwcfg2_fsphytype(ghwcfg2),
		usb_dwc2_get_ghwcfg2_hsphytype(ghwcfg2),
		usb_dwc2_get_ghwcfg4_phydatawidth(ghwcfg4));

	LOG_DBG("LPM mode is %s",
		(ghwcfg3 & USB_DWC2_GHWCFG3_LPMMODE) ? "enabled" : "disabled");

	if (ghwcfg3 & USB_DWC2_GHWCFG3_RSTTYPE) {
		priv->syncrst = 1;
	}

	/* Configure AHB, select Completer or DMA mode */
	gahbcfg = sys_read32(gahbcfg_reg);

	if (priv->bufferdma) {
		gahbcfg |= USB_DWC2_GAHBCFG_DMAEN;
	} else {
		gahbcfg &= ~USB_DWC2_GAHBCFG_DMAEN;
	}

	sys_write32(gahbcfg, gahbcfg_reg);

	dcfg = sys_read32(dcfg_reg);

	dcfg &= ~USB_DWC2_DCFG_DESCDMA;

	/* Configure PHY and device speed */
	dcfg &= ~USB_DWC2_DCFG_DEVSPD_MASK;
	switch (usb_dwc2_get_ghwcfg2_hsphytype(ghwcfg2)) {
	case USB_DWC2_GHWCFG2_HSPHYTYPE_UTMIPLUSULPI:
		__fallthrough;
	case USB_DWC2_GHWCFG2_HSPHYTYPE_ULPI:
		gusbcfg |= USB_DWC2_GUSBCFG_PHYSEL_USB20 |
			   USB_DWC2_GUSBCFG_ULPI_UTMI_SEL_ULPI;
		if (IS_ENABLED(CONFIG_UHC_DRIVER_HIGH_SPEED_SUPPORT_ENABLED)) {
			dcfg |= usb_dwc2_set_dcfg_devspd(USB_DWC2_DCFG_DEVSPD_USBHS20);
		} else {
			dcfg |= usb_dwc2_set_dcfg_devspd(USB_DWC2_DCFG_DEVSPD_USBFS20);
		}
		hs_phy = true;
		break;
	case USB_DWC2_GHWCFG2_HSPHYTYPE_UTMIPLUS:
		gusbcfg |= USB_DWC2_GUSBCFG_PHYSEL_USB20 |
			   USB_DWC2_GUSBCFG_ULPI_UTMI_SEL_UTMI;
		if (IS_ENABLED(CONFIG_UHC_DRIVER_HIGH_SPEED_SUPPORT_ENABLED)) {
			dcfg |= usb_dwc2_set_dcfg_devspd(USB_DWC2_DCFG_DEVSPD_USBHS20);
		} else {
			dcfg |= usb_dwc2_set_dcfg_devspd(USB_DWC2_DCFG_DEVSPD_USBFS20);
		}
		hs_phy = true;
		break;
	case USB_DWC2_GHWCFG2_HSPHYTYPE_NO_HS:
		__fallthrough;
	default:
		if (usb_dwc2_get_ghwcfg2_fsphytype(ghwcfg2) !=
		    USB_DWC2_GHWCFG2_FSPHYTYPE_NO_FS) {
			gusbcfg |= USB_DWC2_GUSBCFG_PHYSEL_USB11;
		}

		dcfg |= usb_dwc2_set_dcfg_devspd(USB_DWC2_DCFG_DEVSPD_USBFS1148);
		hs_phy = false;
	}

	if (usb_dwc2_get_ghwcfg4_phydatawidth(ghwcfg4)) {
		gusbcfg |= USB_DWC2_GUSBCFG_PHYIF_16_BIT;
	}

	/* Update PHY configuration */
	sys_write32(gusbcfg, gusbcfg_reg);
	sys_write32(dcfg, dcfg_reg);

	priv->outeps = 0U;
	for (uint8_t i = 0U; i < priv->numdeveps; i++) {
		uint32_t epdir = usb_dwc2_get_ghwcfg1_epdir(priv->ghwcfg1, i);

		if (epdir == USB_DWC2_GHWCFG1_EPDIR_OUT ||
		    epdir == USB_DWC2_GHWCFG1_EPDIR_BDIR) {
			mem_addr_t doepctl_reg = dwc2_get_dxepctl_reg(dev, i);

			sys_write32(USB_DWC2_DEPCTL_SNAK, doepctl_reg);
			priv->outeps++;
		}
	}

	LOG_DBG("Number of OUT endpoints %u", priv->outeps);

	/* Read and store all TxFIFO depths because Programmed FIFO Depths must
	 * not exceed the power-on values.
	 */
	val = sys_read32((mem_addr_t)&base->gnptxfsiz);
	priv->max_txfifo_depth[0] = usb_dwc2_get_gnptxfsiz_nptxfdep(val);
	for (uint8_t i = 1; i < priv->ineps; i++) {
		priv->max_txfifo_depth[i] = dwc2_get_txfdep(dev, i - 1);
	}

	priv->rxfifo_depth = usb_dwc2_get_grxfsiz(sys_read32(grxfsiz_reg));

	if (priv->dynfifosizing) {
		uint32_t gnptxfsiz;
		uint32_t default_depth;

		/* TODO: For proper runtime FIFO sizing UHC driver would have to
		 * have prior knowledge of the USB configurations. Only with the
		 * prior knowledge, the driver will be able to fairly distribute
		 * available resources. For the time being just use different
		 * defaults based on maximum configured PHY speed, but this has
		 * to be revised if e.g. thresholding support would be necessary
		 * on some target.
		 */
		if (hs_phy) {
			default_depth = UHC_DWC2_GRXFSIZ_HS_DEFAULT;
		} else {
			default_depth = UHC_DWC2_GRXFSIZ_FS_DEFAULT;
		}
		default_depth += priv->outeps * 2U;

		/* Driver does not dynamically resize RxFIFO so there is no need
		 * to store reset value. Read the reset value and make sure that
		 * the programmed value is not greater than what driver sets.
		 */
		priv->rxfifo_depth = MIN(priv->rxfifo_depth, default_depth);
		sys_write32(usb_dwc2_set_grxfsiz(priv->rxfifo_depth), grxfsiz_reg);

		/* Set TxFIFO 0 depth */
		val = MIN(UHC_DWC2_FIFO0_DEPTH, priv->max_txfifo_depth[0]);
		gnptxfsiz = usb_dwc2_set_gnptxfsiz_nptxfdep(val) |
			    usb_dwc2_set_gnptxfsiz_nptxfstaddr(priv->rxfifo_depth);

		sys_write32(gnptxfsiz, (mem_addr_t)&base->gnptxfsiz);
	}

	LOG_DBG("RX FIFO size %u bytes", priv->rxfifo_depth * 4);
	for (uint8_t i = 1U; i < priv->ineps; i++) {
		LOG_DBG("TX FIFO%u depth %u addr %u",
			i, priv->max_txfifo_depth[i], dwc2_get_txfaddr(dev, i));
	}

	if (uhc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	if (uhc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	/* Unmask interrupts */
	sys_write32(IF_ENABLED(CONFIG_UHC_ENABLE_SOF, (USB_DWC2_GINTSTS_SOF |
						       USB_DWC2_GINTSTS_INCOMPISOOUT |
						       USB_DWC2_GINTSTS_INCOMPISOIN |))
		    USB_DWC2_GINTSTS_OEPINT | USB_DWC2_GINTSTS_IEPINT |
		    USB_DWC2_GINTSTS_ENUMDONE | USB_DWC2_GINTSTS_USBRST |
		    USB_DWC2_GINTSTS_WKUPINT | USB_DWC2_GINTSTS_USBSUSP |
		    USB_DWC2_GINTSTS_GOUTNAKEFF,
		    (mem_addr_t)&base->gintmsk);

	return 0;
}

static int uhc_dwc2_enable(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	int err;

	err = dwc2_quirk_pre_enable(dev);
	if (err) {
		LOG_ERR("Quirk pre enable failed %d", err);
		return err;
	}

	err = uhc_dwc2_init_controller(dev);
	if (err) {
		return err;
	}

	/* Enable global interrupt */
	sys_set_bits((mem_addr_t)&base->gahbcfg, USB_DWC2_GAHBCFG_GLBINTRMASK);
	config->irq_enable_func(dev);

	/* Disable soft disconnect */
	sys_clear_bits((mem_addr_t)&base->dctl, USB_DWC2_DCTL_SFTDISCON);
	LOG_DBG("Enable device %p", base);

	err = dwc2_quirk_post_enable(dev);
	if (err) {
		LOG_ERR("Quirk post enable failed %d", err);
		return err;
	}

	return 0;
}

static int uhc_dwc2_disable(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	mem_addr_t dctl_reg = (mem_addr_t)&base->dctl;
	struct net_buf *buf;
	int err;

	LOG_DBG("Disable device %p", dev);

	if (uhc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_DBG("Failed to disable control endpoint");
		return -EIO;
	}

	if (uhc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_DBG("Failed to disable control endpoint");
		return -EIO;
	}

	config->irq_disable_func(dev);

	sys_clear_bits((mem_addr_t)&base->gahbcfg, USB_DWC2_GAHBCFG_GLBINTRMASK);

	/* Enable soft disconnect */
	sys_set_bits(dctl_reg, USB_DWC2_DCTL_SFTDISCON);

	/* OUT endpoint 0 cannot be disabled by software. The buffer allocated
	 * in dwc2_ctrl_feed_dout() can only be freed after core reset if the
	 * core was in Buffer DMA mode.
	 *
	 * Soft Reset does timeout if PHY clock is not running. However, just
	 * triggering Soft Reset seems to be enough on shutdown clean up.
	 */
	dwc2_core_soft_reset(dev);
	buf = uhc_buf_get_all(uhc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
	if (buf) {
		net_buf_unref(buf);
	}

	err = dwc2_quirk_disable(dev);
	if (err) {
		LOG_ERR("Quirk disable failed %d", err);
		return err;
	}

	return 0;
}

static int uhc_dwc2_init(const struct device *dev)
{
	int ret;

	ret = dwc2_quirk_init(dev);
	if (ret) {
		LOG_ERR("Quirk init failed %d", ret);
		return ret;
	}

	return dwc2_init_pinctrl(dev);
}

static int uhc_dwc2_shutdown(const struct device *dev)
{
	int ret;

	ret = dwc2_quirk_shutdown(dev);
	if (ret) {
		LOG_ERR("Quirk shutdown failed %d", ret);
		return ret;
	}

	return 0;
}

static int dwc2_driver_preinit(const struct device *dev)
{
	const struct uhc_dwc2_config *config = dev->config;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct uhc_data *data = dev->data;
	uint16_t mps = 1023;
	uint32_t numdeveps;
	uint32_t ineps;

	k_mutex_init(&data->mutex);

	k_event_init(&priv->drv_evt);
	atomic_clear(&priv->xfer_new);
	atomic_clear(&priv->xfer_finished);
	k_event_init(&priv->ep_disabled);
	k_event_set(&priv->ep_disabled, UINT32_MAX);
	priv->ep_out_disable = 0;
	priv->ep_out_stall = 0;
	priv->iso_enabled = 0;

	/*
	 * At this point, we cannot or do not want to access the hardware
	 * registers to get GHWCFGn values. For now, we will use devicetree to
	 * get GHWCFGn values and use them to determine the number and type of
	 * configured endpoints in the hardware. This can be considered a
	 * workaround, and we may change the upper layer internals to avoid it
	 * in the future.
	 */
	ineps = usb_dwc2_get_ghwcfg4_ineps(config->ghwcfg4) + 1U;
	numdeveps = usb_dwc2_get_ghwcfg2_numdeveps(config->ghwcfg2) + 1U;
	LOG_DBG("Number of endpoints (NUMDEVEPS + 1) %u", numdeveps);
	LOG_DBG("Number of IN endpoints (INEPS + 1) %u", ineps);

	return 0;
}

static void uhc_dwc2_lock(const struct device *dev)
{
	k_sched_lock();
	uhc_lock_internal(dev, K_FOREVER);
}

static void uhc_dwc2_unlock(const struct device *dev)
{
	uhc_unlock_internal(dev);
	k_sched_unlock();
}

static void dwc2_on_bus_reset(const struct device *dev)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	uint32_t doepmsk;
	uint32_t diepmsk;

	/* Set the NAK bit for all OUT endpoints */
	for (uint8_t i = 0U; i < priv->numdeveps; i++) {
		uint32_t epdir = usb_dwc2_get_ghwcfg1_epdir(priv->ghwcfg1, i);
		mem_addr_t doepctl_reg;

		LOG_DBG("ep 0x%02x EPDIR %u", i, epdir);
		if (epdir == USB_DWC2_GHWCFG1_EPDIR_OUT ||
		    epdir == USB_DWC2_GHWCFG1_EPDIR_BDIR) {
			doepctl_reg = dwc2_get_dxepctl_reg(dev, i);
			sys_write32(USB_DWC2_DEPCTL_SNAK, doepctl_reg);
		}
	}

	doepmsk = USB_DWC2_DOEPINT_SETUP | USB_DWC2_DOEPINT_EPDISBLD | USB_DWC2_DOEPINT_XFERCOMPL;
	if (dwc2_in_buffer_dma_mode(dev)) {
		doepmsk |= USB_DWC2_DOEPINT_STSPHSERCVD;
	}

	sys_write32(doepmsk, (mem_addr_t)&base->doepmsk);

	diepmsk = USB_DWC2_DIEPINT_INEPNAKEFF | USB_DWC2_DIEPINT_EPDISBLD |
		  USB_DWC2_DIEPINT_XFERCOMPL;
	sys_write32(diepmsk, (mem_addr_t)&base->diepmsk);

	/* Software has to handle RxFLvl interrupt only in Completer mode */
	if (dwc2_in_completer_mode(dev)) {
		sys_set_bits((mem_addr_t)&base->gintmsk,
			     USB_DWC2_GINTSTS_RXFLVL);
	}

	/* Clear device address during reset. */
	sys_clear_bits((mem_addr_t)&base->dcfg, USB_DWC2_DCFG_DEVADDR_MASK);

	/* Speed enumeration must happen after reset. */
	priv->enumdone = 0;
}

static void uhc_dwc2_isr_handler(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	mem_addr_t gintsts_reg = (mem_addr_t)&base->gintsts;
	uint32_t int_status;
	uint32_t gintmsk;

	if (priv->hibernated) {
		uint32_t gpwrdn = sys_read32((mem_addr_t)&base->gpwrdn);
		bool reset, resume = false;

		/* Clear interrupts */
		sys_write32(gpwrdn, (mem_addr_t)&base->gpwrdn);

		if (gpwrdn & USB_DWC2_GPWRDN_LNSTSCHNG) {
			resume = usb_dwc2_get_gpwrdn_linestate(gpwrdn) ==
				 USB_DWC2_GPWRDN_LINESTATE_DM1DP0;
		}

		reset = gpwrdn & USB_DWC2_GPWRDN_RESETDETECTED;

		if (resume) {
			k_event_post(&priv->drv_evt,
				     BIT(DWC2_DRV_EVT_HIBERNATION_EXIT_HOST_RESUME));
		}

		if (reset) {
			k_event_post(&priv->drv_evt, BIT(DWC2_DRV_EVT_HIBERNATION_EXIT_BUS_RESET));
		}

		(void)dwc2_quirk_irq_clear(dev);
		return;
	}

	gintmsk = sys_read32((mem_addr_t)&base->gintmsk);

	/*  Read and handle interrupt status register */
	while ((int_status = sys_read32(gintsts_reg) & gintmsk)) {
		LOG_DBG("GINTSTS 0x%x", int_status);
`	}

	(void)dwc2_quirk_irq_clear(dev);
}

static const struct uhc_api uhc_dwc2_api = {
	.lock = uhc_dwc2_lock,
	.unlock = uhc_dwc2_unlock,
	.init = uhc_dwc2_init,
	.enable = uhc_dwc2_enable,
	.disable = uhc_dwc2_disable,
	.shutdown = uhc_dwc2_shutdown,
};

#define UHC_DWC2_DT_INST_REG_ADDR(n)						\
	COND_CODE_1(DT_NUM_REGS(DT_DRV_INST(n)), (DT_INST_REG_ADDR(n)),		\
		    (DT_INST_REG_ADDR_BY_NAME(n, core)))

#define UHC_DWC2_PINCTRL_DT_INST_DEFINE(n)					\
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default),			\
		    (PINCTRL_DT_INST_DEFINE(n)), ())

#define UHC_DWC2_PINCTRL_DT_INST_DEV_CONFIG_GET(n)				\
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default),			\
		    ((void *)PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL))

#define UHC_DWC2_IRQ_FLAGS_TYPE0(n)	0
#define UHC_DWC2_IRQ_FLAGS_TYPE1(n)	DT_INST_IRQ(n, type)
#define DW_IRQ_FLAGS(n) \
	_CONCAT(UHC_DWC2_IRQ_FLAGS_TYPE, DT_INST_IRQ_HAS_CELL(n, type))(n)

/*
 * A UHC driver should always be implemented as a multi-instance
 * driver, even if your platform does not require it.
 */
#define UHC_DWC2_DEVICE_DEFINE(n)						\
	UHC_DWC2_PINCTRL_DT_INST_DEFINE(n);					\
										\
	static void uhc_dwc2_irq_enable_func_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    uhc_dwc2_isr_handler,				\
			    DEVICE_DT_INST_GET(n),				\
			    DW_IRQ_FLAGS(n));					\
										\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void uhc_dwc2_irq_disable_func_##n(const struct device *dev)	\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
										\
	static const struct uhc_dwc2_config uhc_dwc2_config_##n = {		\
		.num_out_eps = DT_INST_PROP(n, num_out_eps),			\
		.num_in_eps = DT_INST_PROP(n, num_in_eps),			\
		.base = (struct usb_dwc2_reg *)UHC_DWC2_DT_INST_REG_ADDR(n),	\
		.pcfg = UHC_DWC2_PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.irq_enable_func = uhc_dwc2_irq_enable_func_##n,		\
		.irq_disable_func = uhc_dwc2_irq_disable_func_##n,		\
		.quirks = UHC_DWC2_VENDOR_QUIRK_GET(n),				\
		.ghwcfg1 = DT_INST_PROP(n, ghwcfg1),				\
		.ghwcfg2 = DT_INST_PROP(n, ghwcfg2),				\
		.ghwcfg4 = DT_INST_PROP(n, ghwcfg4),				\
	};									\
										\
	static struct uhc_dwc2_data uhc_priv_##n = {				\
	};									\
										\
	static struct uhc_data uhc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(uhc_data_##n.mutex),		\
		.priv = &uhc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, dwc2_driver_preinit, NULL,			\
			      &uhc_data_##n, &uhc_dwc2_config_##n,		\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &uhc_dwc2_api);

DT_INST_FOREACH_STATUS_OKAY(UHC_DWC2_DEVICE_DEFINE)
