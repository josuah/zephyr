/*
 * Copyright (c) 2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dwc2

#include "uhc_common.h"
#include "uhc_dwc2.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/usb/usb_ch9.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_dwc2, CONFIG_UHC_DRIVER_LOG_LEVEL);

#define DEBOUNCE_DELAY_MS CONFIG_UHC_DWC2_PORT_DEBOUNCE_DELAY_MS
#define RESET_HOLD_MS     CONFIG_UHC_DWC2_PORT_RESET_HOLD_MS
#define RESET_RECOVERY_MS CONFIG_UHC_DWC2_PORT_RESET_RECOVERY_MS
#define SET_ADDR_DELAY_MS CONFIG_UHC_DWC2_PORT_SET_ADDR_DELAY_MS

#define CTRL_EP_MAX_MPS_LS   8U
#define CTRL_EP_MAX_MPS_HSFS 64U

enum uhc_dwc2_event {
	/* Root port event */
	UHC_DWC2_EVENT_PORT,
	/* Root chan event */
	UHC_DWC2_EVENT_CHAN,
};

enum uhc_dwc2_speed {
	UHC_DWC2_SPEED_HIGH = 0,
	UHC_DWC2_SPEED_FULL = 1,
	UHC_DWC2_SPEED_LOW = 2,
};

enum uhc_dwc2_xfer_type {
	UHC_DWC2_XFER_TYPE_CTRL = 0,
	UHC_DWC2_XFER_TYPE_ISOCHRONOUS = 1,
	UHC_DWC2_XFER_TYPE_BULK = 2,
	UHC_DWC2_XFER_TYPE_INTR = 3,
};

enum uhc_port_event {
	/* No event has occurred or the event is no longer valid */
	UHC_PORT_EVENT_NONE,
	/* A device has been connected to the port */
	UHC_PORT_EVENT_CONNECTION,
	/* Device has completed reset and enabled on the port */
	UHC_PORT_EVENT_ENABLED,
	/* A device disconnection has been detected */
	UHC_PORT_EVENT_DISCONNECTION,
	/* Port error detected. Port is now UHC_PORT_STATE_RECOVERY */
	UHC_PORT_EVENT_ERROR,
	/* Overcurrent detected. Port is now UHC_PORT_STATE_RECOVERY */
	UHC_PORT_EVENT_OVERCURRENT,
};

enum uhc_port_state {
	/* The port is not powered */
	UHC_PORT_STATE_NOT_POWERED,
	/* The port is powered but no device is connected */
	UHC_PORT_STATE_DISCONNECTED,
	/* A device is connected to the port but has not been reset. */
	/* SOF/keep alive aren't being sent */
	UHC_PORT_STATE_DISABLED,
	/* The port is issuing a reset condition */
	UHC_PORT_STATE_RESETTING,
	/* The port has been suspended. */
	UHC_PORT_STATE_SUSPENDED,
	/* The port is issuing a resume condition */
	UHC_PORT_STATE_RESUMING,
	/* The port has been enabled. SOF/keep alive are being sent */
	UHC_PORT_STATE_ENABLED,
	/* Port needs to be recovered from a fatal error (error, overcurrent, or disconnection) */
	UHC_PORT_STATE_RECOVERY,
};

enum uhc_dwc2_core_event {
	/* No event occurred, or could not decode interrupt */
	UHC_DWC2_CORE_EVENT_NONE,
	/* A channel event has occurred. Call the channel event handler instead */
	UHC_DWC2_CORE_EVENT_CHAN,
	/* The host port has detected a connection */
	UHC_DWC2_CORE_EVENT_CONN,
	/* The host port has detected a disconnection */
	UHC_DWC2_CORE_EVENT_DISCONN,
	/* The host port has been enabled (i.e., connected device has been reset. Send SOFs) */
	UHC_DWC2_CORE_EVENT_ENABLED,
	/* The host port has been disabled (no more SOFs)  */
	UHC_DWC2_CORE_EVENT_DISABLED,
	/* The host port has encountered an overcurrent condition */
	UHC_DWC2_CORE_EVENT_OVRCUR,
	/* The host port has been cleared of the overcurrent condition */
	UHC_DWC2_CORE_EVENT_OVRCUR_CLR,
};

enum uhc_dwc2_chan_event {
	/* The channel has completed execution of a transfer. Channel is now halted */
	DWC2_CHAN_EVENT_CPLT,
	/* The channel has encountered an error. Channel is now halted */
	DWC2_CHAN_EVENT_ERROR,
	/* A halt has been requested on the channel */
	DWC2_CHAN_EVENT_HALT_REQ,
	/* No event (interrupt ran for internal processing) */
	DWC2_CHAN_EVENT_NONE,
};

enum uhc_dwc2_chan_state {
	/* Pipe is active */
	UHC_CHAN_STATE_ACTIVE,
	/* Pipe is halted */
	UHC_CHAN_STATE_HALTED,
};

enum uhc_dwc2_ctrl_stage {
	CTRL_STAGE_DATA0 = 0,
	CTRL_STAGE_DATA2 = 1,
	CTRL_STAGE_DATA1 = 2,
	CTRL_STAGE_SETUP = 3,
};

struct uhc_dwc2_chan_config {
	/* Speed of the device */
	enum uhc_dwc2_speed dev_speed;
	/* Device address */
	uint8_t dev_addr;
};

struct uhc_dwc2_chan {
	/* XFER queuing related */
	sys_dlist_t xfer_pending_list;
	/* TODO: Lists of pending and done? */
	int num_xfer_pending;
	int num_xfer_done;
	/* Pointer to the transfer associated with the buffer */
	struct uhc_transfer *xfer;
	/* Interval in frames (FS) or microframes (HS) */
	unsigned int interval;
	/* Offset in the periodic scheduler */
	uint32_t offset;
	/* The chan event type */
	enum uhc_dwc2_chan_event chan_event;
	/* Type of endpoint */
	enum uhc_dwc2_xfer_type type;
	/* Pipe status/state/events related */
	enum uhc_dwc2_chan_state state;
	enum uhc_dwc2_chan_event last_event;
	/* Maximum Packet Size */
	uint16_t mps;
	/* Endpoint address */
	uint8_t bEndpointAddress;
	/* Device Address */
	uint8_t dev_addr;
	/* Stage index */
	uint8_t cur_stg;
	/* New address */
	uint8_t new_addr;
	/* The index of the channel */
	uint8_t chan_idx;
	/* Set address request */
	uint8_t set_addr: 1;
	/* Data stage is IN */
	uint8_t data_stg_in: 1;
	/* Has no data stage */
	uint8_t data_stg_skip: 1;
	/* This DMA buffer is currently being executed */
	uint8_t executing: 1;
	/* THis DMA buffer was canceled before completion  */
	uint8_t was_canceled: 1;
	/* High-speed flag */
	uint8_t is_hs: 1;
	/* Support for Low-Speed is via a Full-Speed HUB */
	uint8_t ls_via_fs_hub: 1;
	uint8_t waiting_halt: 1;
	uint8_t chan_cmd_processing: 1;
	/* XFER: pending, in-flight or done */
	uint8_t has_xfer: 1;
	/* Pipe event is pending */
	uint8_t event_pending: 1;
	/* Is channel enabled */
	uint8_t active: 1;
	/* Halt has been requested */
	uint8_t halt_requested: 1;
	/* TODO: Add channel error? */
};

struct uhc_dwc2_data {
	struct k_sem irq_sem;
	/* TODO: spinlock? */
	struct k_thread thread_data;
	/* Mutex for port access */
	struct k_mutex mutex;
	/* Main events the driver thread waits for */
	struct k_event drv_evt;
	struct uhc_dwc2_chan chan;
	struct uhc_dwc2_chan *ctrl_chan;
	/* Handles of each channel */
	struct uhc_dwc2_chan **channels;
	/* Number of channels currently allocated */
	size_t num_channels;
	/* Bit mask of channels with pending interrupts */
	uint32_t pending_channel_intrs_msk;
	/* Data, that is used in multiple threads */
	enum uhc_port_event last_event;
	enum uhc_port_state port_state;
	/* Number of available channels */
	size_t dwc2_numchannels;
	/* FIFO */
	uint16_t fifo_top;
	uint16_t fifo_nptxfsiz;
	uint16_t fifo_rxfsiz;
	uint16_t fifo_ptxfsiz;
	/* FIFO depth in WORDs */
	uint16_t dwc2_fifodepth;
	/* High-speed PHY type */
	uint8_t dwc2_hsphytype;
	/* Full-speed PHY type */
	uint8_t dwc2_fsphytype;
	/* PHY data width */
	uint8_t dwc2_phydatawidth;
	/* Buffer DMA mode flag */
	bool dwc2_bufferdma;
	/* Number of idle chans */
	uint8_t num_chans_idle;
	/* Number of chans queued for processing */
	uint8_t num_chans_queued;
	/* Debounce lock */
	uint8_t lock_enabled: 1;
	/* Port event is pending */
	uint8_t event_pending: 1;
	/* Device is connected */
	uint8_t conn_dev_ena: 1;
	/* Waiting to be disabled */
	uint8_t waiting_disable: 1;
	/* TODO: Port context and callback? */
	/* TODO: Dynamic chan allocation on enqueue? */
	/* TODO: FRAME LIST? */
	/* TODO: Pipes/channels LIST? */
};

/* Host channel registers address */
#define UHC_DWC2_CHAN_REG(base, chan_idx)                                                          \
	((struct usb_dwc2_host_chan *)(((mem_addr_t)(base)) + 0x500UL + ((chan_idx) * 0x20UL)))

/*
 * DWC2 FIFO Management
 */

/* Programming Guide 2.1.2 FIFO RAM allocation
 * RX
 * - Largest-EPsize/4 + 2 (status info). recommended x2 if high bandwidth or multiple ISO are used.
 * - 2 for transfer complete and channel halted status
 * - 1 for each Control/Bulk out endpoint to Handle NAK/NYET (i.e max is number of host channel)
 *
 * TX non-periodic (NPTX)
 * - At least largest-EPsize/4, recommended x2
 *
 * TX periodic (PTX)
 * - At least largest-EPsize*MulCount/4 (MulCount up to 3 for high-bandwidth ISO/interrupt)
 */
enum {
	EPSIZE_BULK_FS = 64,
	EPSIZE_BULK_HS = 512,

	EPSIZE_ISO_FS_MAX = 1023,
	EPSIZE_ISO_HS_MAX = 1024,
};

static inline void uhc_dwc2_config_fifo_fixed_dma(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	LOG_DBG("Configuring FIFO sizes");
	priv->fifo_top = priv->dwc2_fifodepth;
	priv->fifo_top -= priv->dwc2_numchannels;

	/* TODO: support HS */

	uint32_t nptx_largest = EPSIZE_BULK_FS / 4;
	uint32_t ptx_largest = 256 / 4;

	priv->fifo_nptxfsiz = 2 * nptx_largest;
	priv->fifo_rxfsiz = 2 * (ptx_largest + 2) + priv->dwc2_numchannels;
	priv->fifo_ptxfsiz = priv->fifo_top - (priv->fifo_nptxfsiz + priv->fifo_rxfsiz);

	/* TODO: verify ptxfsiz is overflowed */

	LOG_DBG("FIFO sizes calculated");
	LOG_DBG("\ttop=%u, nptx=%u, rx=%u, ptx=%u", priv->fifo_top * 4, priv->fifo_nptxfsiz * 4,
		priv->fifo_rxfsiz * 4, priv->fifo_ptxfsiz * 4);
}

/*
 * DWC2 low-level Functions,
 */

void dwc2_hal_flush_rx_fifo(struct usb_dwc2_reg *const dwc2)
{
	mem_addr_t grstctl_reg = (mem_addr_t)&dwc2->grstctl;

	sys_write32(USB_DWC2_GRSTCTL_RXFFLSH, grstctl_reg);
	while (sys_read32(grstctl_reg) & USB_DWC2_GRSTCTL_RXFFLSH) {
	}
}

void dwc2_hal_flush_tx_fifo(struct usb_dwc2_reg *const dwc2, const uint8_t fnum)
{
	mem_addr_t grstctl_reg = (mem_addr_t)&dwc2->grstctl;
	uint32_t grstctl;

	grstctl = usb_dwc2_set_grstctl_txfnum(fnum) | USB_DWC2_GRSTCTL_TXFFLSH;

	sys_write32(grstctl, grstctl_reg);
	while (sys_read32(grstctl_reg) & USB_DWC2_GRSTCTL_TXFFLSH) {
	}
}

static inline void dwc2_hal_set_frame_list(struct usb_dwc2_reg *const dwc2, void *frame_list)
{
	LOG_WRN("Setting frame list not implemented yet");
}

static inline void dwc2_hal_periodic_enable(struct usb_dwc2_reg *const dwc2)
{
	LOG_WRN("Enabling periodic scheduling not implemented yet");
}

static inline void dwc2_hal_port_init(struct usb_dwc2_reg *const dwc2)
{
	sys_clear_bits((mem_addr_t)&dwc2->haintmsk, 0xFFFFFFFFUL);
	sys_set_bits((mem_addr_t)&dwc2->gintmsk, USB_DWC2_GINTSTS_PRTINT | USB_DWC2_GINTSTS_HCHINT);
}

#define USB_DWC2_HPRT_W1C_MSK                                                                      \
	(USB_DWC2_HPRT_PRTENA | USB_DWC2_HPRT_PRTCONNDET | USB_DWC2_HPRT_PRTENCHNG |               \
	 USB_DWC2_HPRT_PRTOVRCURRCHNG)

static inline void dwc2_hal_toggle_reset(struct usb_dwc2_reg *const dwc2, bool reset_on)
{
	uint32_t hprt = sys_read32((mem_addr_t)&dwc2->hprt);

	if (reset_on) {
		hprt |= USB_DWC2_HPRT_PRTRST;
	} else {
		hprt &= ~USB_DWC2_HPRT_PRTRST;
	}

	sys_write32(hprt & (~USB_DWC2_HPRT_W1C_MSK), (mem_addr_t)&dwc2->hprt);
}

static inline void dwc2_hal_toggle_power(struct usb_dwc2_reg *const dwc2, bool power_on)
{
	uint32_t hprt = sys_read32((mem_addr_t)&dwc2->hprt);

	if (power_on) {
		hprt |= USB_DWC2_HPRT_PRTPWR;
	} else {
		hprt &= ~USB_DWC2_HPRT_PRTPWR;
	}

	sys_write32(hprt & (~USB_DWC2_HPRT_W1C_MSK), (mem_addr_t)&dwc2->hprt);
}

int dwc2_core_reset(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;

	mem_addr_t grstctl_reg = (mem_addr_t)&dwc2->grstctl;
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
		if (uhc_dwc2_quirk_is_phy_clk_off(dev)) {
			/* Software reset won't finish without PHY clock */
			return -EIO;
		}
	} while (sys_read32(grstctl_reg) & USB_DWC2_GRSTCTL_CSFTRST &&
		 !(sys_read32(grstctl_reg) & USB_DWC2_GRSTCTL_CSFTRSTDONE));

	/* CSFTRSTDONE is W1C so the write must have the bit set to clear it */
	sys_clear_bits(grstctl_reg, USB_DWC2_GRSTCTL_CSFTRST);

	LOG_DBG("DWC2 core reset done");

	return 0;
}

static inline int dwc2_get_config(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	uint32_t gsnpsid = sys_read32((mem_addr_t)&dwc2->gsnpsid);
	uint32_t ghwcfg2 = sys_read32((mem_addr_t)&dwc2->ghwcfg2);
	uint32_t ghwcfg3 = sys_read32((mem_addr_t)&dwc2->ghwcfg3);
	uint32_t ghwcfg4 = sys_read32((mem_addr_t)&dwc2->ghwcfg4);

	LOG_DBG("GSNPSID=%08Xh, GHWCFG2=%08Xh, GHWCFG3=%08Xh, GHWCFG4=%08Xh", gsnpsid, ghwcfg2,
		ghwcfg3, ghwcfg4);

	/* Check Synopsis ID register, failed if controller clock/power is not enabled */
	__ASSERT((gsnpsid == USB_DWC2_GSNPSID_REV_5_00A),
		 "DWC2 core ID is not compatible with the driver, GSNPSID: 0x%08x", gsnpsid);

	if (gsnpsid == 0) {
		LOG_ERR("Unable to read DWC2 Core ID, core is not powered on");
		return -ENODEV;
	}

	if (!(ghwcfg4 & USB_DWC2_GHWCFG4_DEDFIFOMODE)) {
		LOG_ERR("Only dedicated TX FIFO mode is supported");
		return -ENOTSUP;
	}

	/* Buffer DMA is always supported in Internal DMA mode.
	 * TODO: check and support descriptor DMA if available
	 */
	priv->dwc2_bufferdma =
		(usb_dwc2_get_ghwcfg2_otgarch(ghwcfg2) == USB_DWC2_GHWCFG2_OTGARCH_INTERNALDMA);

	if (IS_ENABLED(CONFIG_UHC_DWC2_DMA)) {
		if (priv->dwc2_bufferdma) {
			LOG_DBG("Buffer DMA enabled");
		}
	} else {
		priv->dwc2_bufferdma = 0;
	}

	if (ghwcfg2 & USB_DWC2_GHWCFG2_DYNFIFOSIZING) {
		LOG_DBG("Dynamic FIFO Sizing is enabled");
		/* TODO: support FIFO dynamic sizing */
	}

	/* TODO: Support hybernation */

	LOG_DBG("OTG architecture (OTGARCH) %u, mode (OTGMODE) %u",
		usb_dwc2_get_ghwcfg2_otgarch(ghwcfg2), usb_dwc2_get_ghwcfg2_otgmode(ghwcfg2));

	priv->dwc2_fifodepth = usb_dwc2_get_ghwcfg3_dfifodepth(ghwcfg3);
	LOG_DBG("DFIFO depth (DFIFODEPTH) %u bytes", priv->dwc2_fifodepth * 4);

	/* TODO: Support vendor control interface */
	LOG_DBG("Vendor Control interface support enabled: %s",
		(ghwcfg3 & USB_DWC2_GHWCFG3_VNDCTLSUPT) ? "true" : "false");

	LOG_DBG("PHY interface type: FSPHYTYPE %u, HSPHYTYPE %u, DATAWIDTH %u",
		usb_dwc2_get_ghwcfg2_fsphytype(ghwcfg2), usb_dwc2_get_ghwcfg2_hsphytype(ghwcfg2),
		usb_dwc2_get_ghwcfg4_phydatawidth(ghwcfg4));

	/* TODO: Support LPM */
	LOG_DBG("LPM mode is %s", (ghwcfg3 & USB_DWC2_GHWCFG3_LPMMODE) ? "enabled" : "disabled");

	if (ghwcfg3 & USB_DWC2_GHWCFG3_RSTTYPE) {
		/* TODO: Support sync reset */
	}

	/* TODO: Support dedicated FIFO mode */

	priv->dwc2_hsphytype = usb_dwc2_get_ghwcfg2_hsphytype(ghwcfg2);
	priv->dwc2_fsphytype = usb_dwc2_get_ghwcfg2_fsphytype(ghwcfg2);
	priv->dwc2_phydatawidth = usb_dwc2_get_ghwcfg4_phydatawidth(ghwcfg4);
	priv->dwc2_numchannels = usb_dwc2_get_ghwcfg2_numhstchnl(ghwcfg2) + 1U;

	LOG_DBG("PHY interface type: FSPHYTYPE %u, HSPHYTYPE %u, DATAWIDTH %u", priv->dwc2_fsphytype,
		priv->dwc2_hsphytype, priv->dwc2_phydatawidth);

	LOG_DBG("Number of host channels (NUMHSTCHNL + 1) %u", priv->dwc2_numchannels);

	return 0;
}

static void dwc2_channel_configure(const struct device *dev, struct uhc_dwc2_chan *chan)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	const struct usb_dwc2_host_chan *chan_regs = UHC_DWC2_CHAN_REG(dwc2, chan->chan_idx);

	__ASSERT(!chan->active && !chan->halt_requested,
		 "Cannot change endpoint characteristics while channel is active or halted");

	mem_addr_t hcchar_reg = (mem_addr_t)&chan_regs->hcchar;

	uint32_t hcchar = ((uint32_t)chan->mps << USB_DWC2_HCCHAR0_MPS_POS) |
			  ((uint32_t)USB_EP_GET_IDX(chan->bEndpointAddress)
			   << USB_DWC2_HCCHAR0_EPNUM_POS) |
			  ((uint32_t)chan->type << USB_DWC2_HCCHAR0_EPTYPE_POS) |
			  ((uint32_t)1UL /* TODO: chan->mult */ << USB_DWC2_HCCHAR0_EC_POS) |
			  ((uint32_t)chan->dev_addr << USB_DWC2_HCCHAR0_DEVADDR_POS);

	if (USB_EP_DIR_IS_IN(chan->bEndpointAddress)) {
		hcchar |= USB_DWC2_HCCHAR0_EPDIR;
	}

	/* TODO: LS device plugged to HUB */
	if (false) {
		hcchar |= USB_DWC2_HCCHAR0_LSPDDEV;
	}

	if (chan->type == UHC_DWC2_XFER_TYPE_INTR) {
		hcchar |= USB_DWC2_HCCHAR0_ODDFRM;
	}

	sys_write32(hcchar, hcchar_reg);

	if (chan->type == UHC_DWC2_XFER_TYPE_ISOCHRONOUS ||
	    chan->type == UHC_DWC2_XFER_TYPE_INTR) {
		LOG_WRN("ISOC and INTR channels are note supported yet");
	}
}

static inline enum uhc_dwc2_speed dwc2_hal_get_port_speed(struct usb_dwc2_reg *const dwc2)
{
	uint32_t hprt = sys_read32((mem_addr_t)&dwc2->hprt);
	return (hprt & USB_DWC2_HPRT_PRTSPD_MASK) >> USB_DWC2_HPRT_PRTSPD_POS;
}

/*
 * DWC2 Port Management
 */

/* Host Port Control and Status Register */
#define USB_DWC2_HPRT_PRTENCHNG      BIT(3)
#define USB_DWC2_HPRT_PRTOVRCURRCHNG BIT(5)
#define USB_DWC2_HPRT_PRTCONNDET     BIT(1)

#define CORE_INTRS_EN_MSK (USB_DWC2_GINTSTS_DISCONNINT)

/* Interrupts that pertain to core events */
#define CORE_EVENTS_INTRS_MSK (USB_DWC2_GINTSTS_DISCONNINT | USB_DWC2_GINTSTS_HCHINT)

/* Interrupt that pertain to host port events */
#define PORT_EVENTS_INTRS_MSK                                                                      \
	(USB_DWC2_HPRT_PRTCONNDET | USB_DWC2_HPRT_PRTENCHNG | USB_DWC2_HPRT_PRTOVRCURRCHNG)

static inline int uhc_dwc2_get_port_speed(const struct device *dev, enum uhc_dwc2_speed *speed)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;

	if (priv->port_state != UHC_PORT_STATE_ENABLED) {
		LOG_ERR("Port is not enabled, cannot get speed");
		return -ENODEV;
	}

	*speed = dwc2_hal_get_port_speed(dwc2);

	return 0;
}

static void uhc_dwc2_lock_enable(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	/* Disable Connection and disconnection interrupts to prevent spurious events */
	sys_clear_bits((mem_addr_t)&dwc2->gintmsk,
		       USB_DWC2_GINTSTS_PRTINT | USB_DWC2_GINTSTS_DISCONNINT);
	priv->lock_enabled = 1;
}

static inline void uhc_dwc2_lock_disable(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	priv->lock_enabled = 0;
	/* Clear Connection and disconnection interrupt in case it triggered again */
	sys_set_bits((mem_addr_t)&dwc2->gintsts, USB_DWC2_GINTSTS_DISCONNINT);
	/* Clear the PRTCONNDET interrupt by writing 1 to the corresponding bit (W1C logic) */
	sys_set_bits((mem_addr_t)&dwc2->hprt, USB_DWC2_HPRT_PRTCONNDET);
	/* Re-enable the HPRT (connection) and disconnection interrupts */
	sys_set_bits((mem_addr_t)&dwc2->gintmsk,
		     USB_DWC2_GINTSTS_PRTINT | USB_DWC2_GINTSTS_DISCONNINT);
}

static inline void dwc2_port_enable(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	enum uhc_dwc2_speed speed = dwc2_hal_get_port_speed(dwc2);
	uint32_t hcfg = sys_read32((mem_addr_t)&dwc2->hcfg);
	uint32_t hfir = sys_read32((mem_addr_t)&dwc2->hfir);

	/* We can select Buffer DMA of Scatter-Gather DMA mode here: Buffer DMA by default */
	hcfg &= ~USB_DWC2_HCFG_DESCDMA;

	/* Disable periodic scheduling, will enable later */
	hcfg &= ~USB_DWC2_HCFG_PERSCHEDENA;

	if (priv->dwc2_hsphytype == 0) {
		/*
		Indicate to the OTG core what speed the PHY clock is at
		Note: FSLS PHY has an implicit 8 divider applied when in LS mode,
		so the values of FSLSPclkSel and FrInt have to be adjusted accordingly.
		*/
		uint8_t fslspclksel = (speed == UHC_DWC2_SPEED_FULL) ? 1 : 2;
		hcfg &= ~USB_DWC2_HCFG_FSLSPCLKSEL_MASK;
		hcfg |= (fslspclksel << USB_DWC2_HCFG_FSLSPCLKSEL_POS);

		/* Disable dynamic loading */
		hfir &= ~USB_DWC2_HFIR_HFIRRLDCTRL;
		/*
		Set frame interval to be equal to 1ms
		Note: FSLS PHY has an implicit 8 divider applied when in LS mode,
			so the values of FSLSPclkSel and FrInt have to be adjusted accordingly.
		*/
		uint16_t frint = (speed == UHC_DWC2_SPEED_FULL) ? 48000 : 6000;
		hfir &= ~USB_DWC2_HFIR_FRINT_MASK;
		hfir |= (frint << USB_DWC2_HFIR_FRINT_POS);

		sys_write32(hcfg, (mem_addr_t)&dwc2->hcfg);
		sys_write32(hfir, (mem_addr_t)&dwc2->hfir);
	} else {
		LOG_ERR("Configuring clocks for HS PHY is not implemented");
	}
}

static int uhc_dwc2_power_on(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;

	/* Port can only be powered on if it's currently unpowered */
	if (priv->port_state == UHC_PORT_STATE_NOT_POWERED) {
		priv->port_state = UHC_PORT_STATE_DISCONNECTED;
		dwc2_hal_port_init(dwc2);
		dwc2_hal_toggle_power(dwc2, true);
		return 0;
	}

	return -EINVAL;
}

static inline int uhc_dwc2_config_phy(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;

	/* Init PHY based on the speed */
	int ret;

	if (priv->dwc2_hsphytype != 0) {
		uint32_t gusbcfg = sys_read32((mem_addr_t)&dwc2->gusbcfg);

		/* De-select FS PHY */
		gusbcfg &= ~USB_DWC2_GUSBCFG_PHYSEL_USB11;

		if (priv->dwc2_hsphytype == USB_DWC2_GHWCFG2_HSPHYTYPE_ULPI) {
			LOG_WRN("Highspeed ULPI PHY init");
			/* Select ULPI PHY (external) */
			gusbcfg |= USB_DWC2_GUSBCFG_ULPI_UTMI_SEL_ULPI;
			/* ULPI is always 8-bit interface */
			gusbcfg &= ~USB_DWC2_GUSBCFG_PHYIF_16_BIT;
			/* ULPI select single data rate */
			gusbcfg &= ~USB_DWC2_GUSBCFG_DDR_DOUBLE;
			/* Default internal VBUS Indicator and Drive */
			gusbcfg &= ~(USB_DWC2_GUSBCFG_ULPIEVBUSD | USB_DWC2_GUSBCFG_ULPIEVBUSI);
			/* Disable FS/LS ULPI and Supend mode */
			gusbcfg &= ~(USB_DWC2_GUSBCFG_ULPIFSLS | USB_DWC2_GUSBCFG_ULPICLK_SUSM);
		} else {
			LOG_WRN("Highspeed UTMI+ PHY init");
			/* Select UTMI+ PHY (internal) */
			gusbcfg &= ~USB_DWC2_GUSBCFG_ULPI_UTMI_SEL_ULPI;
			/* Set 16-bit interface if supported */
			if (priv->dwc2_phydatawidth) {
				gusbcfg |= USB_DWC2_GUSBCFG_PHYIF_16_BIT;
			} else {
				gusbcfg &= ~USB_DWC2_GUSBCFG_PHYIF_16_BIT;
			}
		}
		sys_write32(gusbcfg, (mem_addr_t)&dwc2->gusbcfg);

		ret = uhc_dwc2_quirk_phy_pre_select(dev);
		if (ret) {
			LOG_ERR("Quirk PHY pre select failed %d", ret);
			return ret;
		}

		/* Reset core after selecting PHY */
		ret = dwc2_core_reset(dev);
		if (ret) {
			LOG_ERR("DWC2 core reset failed after PHY init: %d", ret);
			return ret;
		}

		ret = uhc_dwc2_quirk_phy_post_select(dev);
		if (ret) {
			LOG_ERR("Quirk PHY post select failed %d", ret);
			return ret;
		}
	} else {
		sys_set_bits((mem_addr_t)&dwc2->gusbcfg, USB_DWC2_GUSBCFG_PHYSEL_USB11);

		ret = uhc_dwc2_quirk_phy_pre_select(dev);
		if (ret) {
			LOG_ERR("Quirk PHY pre select failed %d", ret);
			return ret;
		}

		/* Reset core after selecting PHY */
		ret = dwc2_core_reset(dev);
		if (ret) {
			LOG_ERR("DWC2 core reset failed after PHY init: %d", ret);
			return ret;
		}

		ret = uhc_dwc2_quirk_phy_post_select(dev);
		if (ret) {
			LOG_ERR("Quirk PHY post select failed %d", ret);
			return ret;
		}
	}

	return ret;
}

static inline void uhc_dwc2_set_defaults(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	/* Disable Global Interrupt */
	sys_clear_bits((mem_addr_t)&dwc2->gahbcfg, USB_DWC2_GAHBCFG_GLBINTRMASK);

	/* Enable Host mode */
	sys_set_bits((mem_addr_t)&dwc2->gusbcfg, USB_DWC2_GUSBCFG_FORCEHSTMODE);
	/* Wait until core is in host mode */
	while ((sys_read32((mem_addr_t)&dwc2->gintsts) & USB_DWC2_GINTSTS_CURMOD) != 1) {
	}

	/* TODO: Set AHB burst mode for some ECO only for ESP32S2 */
	/* Make config quirk? */

	/* TODO: Disable HNP and SRP capabilities */
	/* Also move to quirk? */

	sys_clear_bits((mem_addr_t)&dwc2->gintmsk, 0xFFFFFFFFUL);

	sys_set_bits((mem_addr_t)&dwc2->gintmsk, CORE_INTRS_EN_MSK);

	/* Clear status */
	uint32_t core_intrs = sys_read32((mem_addr_t)&dwc2->gintsts);
	sys_write32(core_intrs, (mem_addr_t)&dwc2->gintsts);

	/* Configure AHB */
	uint32_t gahbcfg = sys_read32((mem_addr_t)&dwc2->gahbcfg);
	gahbcfg |= USB_DWC2_GAHBCFG_NPTXFEMPLVL;
	gahbcfg &= ~USB_DWC2_GAHBCFG_HBSTLEN_MASK;
	gahbcfg |= (USB_DWC2_GAHBCFG_HBSTLEN_INCR16 << USB_DWC2_GAHBCFG_HBSTLEN_POS);
	sys_write32(gahbcfg, (mem_addr_t)&dwc2->gahbcfg);

	if (priv->dwc2_bufferdma) {
		sys_set_bits((mem_addr_t)&dwc2->gahbcfg, USB_DWC2_GAHBCFG_DMAEN);
	}

	/* Enable Global Interrupt */
	sys_set_bits((mem_addr_t)&dwc2->gahbcfg, USB_DWC2_GAHBCFG_GLBINTRMASK);
}

static int uhc_dwc2_init_controller(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	int ret;

	/* Get hardware configuration */
	ret = dwc2_get_config(dev);
	if (ret) {
		LOG_ERR("Failed to get DWC2 core parameters: %d", ret);
		return ret;
	}

	/* Pre-calculate FIFO settings */
	uhc_dwc2_config_fifo_fixed_dma(dev);

	/* Config PHY */
	ret = uhc_dwc2_config_phy(dev);
	if (ret) {
		LOG_ERR("Failed to configure DWC2 PHY: %d", ret);
		return ret;
	}

	/* Set defaults */
	uhc_dwc2_set_defaults(dev);

	/* Update the port state and flags */
	priv->port_state = UHC_PORT_STATE_NOT_POWERED;
	priv->last_event = UHC_PORT_EVENT_NONE;

	/* TODO: Clear all the flags and channels */
	priv->num_channels = 0;
	priv->pending_channel_intrs_msk = 0;
	if (priv->channels) {
		for (int i = 0; i < priv->dwc2_numchannels; i++) {
			priv->channels[i] = NULL;
		}
	}

	return ret;
}

static enum uhc_port_event uhc_dwc2_decode_hprt(const struct device *dev,
						enum uhc_dwc2_core_event core_event)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	enum uhc_port_event port_event = UHC_PORT_EVENT_NONE;
	switch (core_event) {
	case UHC_DWC2_CORE_EVENT_CONN: {
		port_event = UHC_PORT_EVENT_CONNECTION;
		break;
	}
	case UHC_DWC2_CORE_EVENT_DISCONN: {
		/* TODO: priv->port_state = UHC_PORT_STATE_RECOVERY */
		port_event = UHC_PORT_EVENT_DISCONNECTION;
		priv->conn_dev_ena = 0;
		break;
	}
	case UHC_DWC2_CORE_EVENT_ENABLED: {
		/* Initialize remaining host port registers */
		dwc2_port_enable(dev);
		port_event = UHC_PORT_EVENT_ENABLED;
		priv->conn_dev_ena = 1;
		break;
	}
	case UHC_DWC2_CORE_EVENT_DISABLED: {
		priv->conn_dev_ena = 0;
		/* Could be due to a disable request or reset request, or due to a port error */
		/* Ignore the disable event if it's due to a reset request */
		if (priv->port_state != UHC_PORT_STATE_RESETTING) {
			if (priv->waiting_disable) {
				/* Disabled by request (i.e. by port command). Generate an internal
				 * event */
				priv->port_state = UHC_PORT_STATE_DISABLED;
				priv->waiting_disable = 0;
				/* TODO: Notify the port event from ISR */
				LOG_ERR("Port disabled by request, not implemented yet");
			} else {
				/* Disabled due to a port error */
				LOG_ERR("Port disabled due to an error, changing state to "
					"recovery");
				priv->port_state = UHC_PORT_STATE_RECOVERY;
				port_event = UHC_PORT_EVENT_ERROR;
			}
		}
		break;
	}
	case UHC_DWC2_CORE_EVENT_OVRCUR:
	case UHC_DWC2_CORE_EVENT_OVRCUR_CLR: {
		/* TODO: Handle overcurrent event */

		/*
		 * If port state powered, we need to power it off to protect it
		 * change port state to recovery
		 * generate port event UHC_PORT_EVENT_OVERCURRENT
		 * disable the flag conn_dev_ena
		 */

		LOG_ERR("Overcurrent detected on port, not implemented yet");
		break;
	}
	default: {
		__ASSERT(false, "uhc_dwc2_decode_hprt: Unexpected core event %d", core_event);
		break;
	}
	}
	return port_event;
}

static inline enum uhc_dwc2_core_event uhc_dwc2_decode_intr(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;
	mem_addr_t hprt_reg = (mem_addr_t)&dwc2->hprt;

	enum uhc_dwc2_core_event core_event = UHC_DWC2_CORE_EVENT_NONE;
	/* Read and clear core interrupt status */
	uint32_t core_intrs = sys_read32((mem_addr_t)&dwc2->gintsts);
	sys_write32(core_intrs, (mem_addr_t)&dwc2->gintsts);

	uint32_t port_intrs = 0;

	if (core_intrs & USB_DWC2_GINTSTS_PRTINT) {
		port_intrs = sys_read32(hprt_reg);
		/* Clear the interrupt status by writing 1 to the W1C bits, except the PRTENA bit */
		sys_write32(port_intrs & (~USB_DWC2_HPRT_PRTENA), hprt_reg);
	}

	LOG_DBG("GINTSTS=%08Xh, HPRT=%08Xh", core_intrs, port_intrs);

	/*
	 * Note:
	 * ENABLED < DISABLED < CONN < DISCONN < OVRCUR
	 * The order of events is important, as some events take precedence over others.
	 * Do not change order of checks. Regressing events (e.g. enable -> disabled,
	 * connected -> connected) always take precedence.
	 */
	if ((core_intrs & CORE_EVENTS_INTRS_MSK) || (port_intrs & PORT_EVENTS_INTRS_MSK)) {
		if (core_intrs & USB_DWC2_GINTSTS_DISCONNINT) {
			/* Disconnect event */
			core_event = UHC_DWC2_CORE_EVENT_DISCONN;
			/* Debounce lock */
			uhc_dwc2_lock_enable(dev);
		} else {
			/* Port still connected, check port event */
			if (port_intrs & USB_DWC2_HPRT_PRTOVRCURRCHNG) {
				/* Check if this is an overcurrent or an overcurrent cleared */
				if (port_intrs & USB_DWC2_HPRT_PRTOVRCURRACT) {
					/* TODO: Verify handling logic during overcurrent */
					core_event = UHC_DWC2_CORE_EVENT_OVRCUR;
				} else {
					core_event = UHC_DWC2_CORE_EVENT_OVRCUR_CLR;
				}
			} else if (port_intrs & USB_DWC2_HPRT_PRTENCHNG) {
				if (port_intrs & USB_DWC2_HPRT_PRTENA) {
					/* Host port was enabled */
					core_event = UHC_DWC2_CORE_EVENT_ENABLED;
				} else {
					/* Host port has been disabled */
					core_event = UHC_DWC2_CORE_EVENT_DISABLED;
				}
			} else if (port_intrs & USB_DWC2_HPRT_PRTCONNDET &&
				   !priv->lock_enabled) {
				core_event = UHC_DWC2_CORE_EVENT_CONN;
				/* Debounce lock */
				uhc_dwc2_lock_enable(dev);
			} else {
				/* Should never happened, as port event masked with
				 * PORT_EVENTS_INTRS_MSK */
				__ASSERT(false,
					 "uhc_dwc2_decode_intr: Unknown port interrupt, HPRT=%08Xh",
					 port_intrs);
			}
		}
	}
	/* Port events always take precedence over channel events */
	if (core_event == UHC_DWC2_CORE_EVENT_NONE && (core_intrs & USB_DWC2_GINTSTS_HCHINT)) {
		/* One or more channels have pending interrupts. Store the mask of those channels */
		priv->pending_channel_intrs_msk = sys_read32((mem_addr_t)&dwc2->haint);
		core_event = UHC_DWC2_CORE_EVENT_CHAN;
	}

	return core_event;
}

enum uhc_dwc2_chan_event uhc_dwc2_hal_chan_decode_intr(const struct device *dev,
						       struct uhc_dwc2_chan *chan)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	const struct usb_dwc2_host_chan *chan_regs = UHC_DWC2_CHAN_REG(dwc2, chan->chan_idx);
	mem_addr_t hcint_reg = (mem_addr_t)&chan_regs->hcint;

	uint32_t hcint = sys_read32(hcint_reg);
	/* Clear the interrupt bits by writing them back */
	sys_write32(hcint, hcint_reg);

	enum uhc_dwc2_chan_event chan_event;

	/*
	 * Note:
	 * We don't assert on (chan->active) here as it could have been already cleared
	 * by usb_dwc_hal_chan_request_halt()
	 */

	/*
	 * Note:
	 * Do not change order of checks as some events take precedence over others.
	 * Errors > Channel Halt Request > Transfer completed
	 */
	if (hcint & (USB_DWC2_HCINT_STALL | USB_DWC2_HCINT_BBLERR | USB_DWC2_HCINT_XACTERR)) {
		__ASSERT(hcint & USB_DWC2_HCINT_CHHLTD, "uhc_dwc2_hal_chan_decode_intr: Channel "
							"error without channel halted interrupt");

		LOG_ERR("Channel %d error: 0x%08x", chan->chan_idx, hcint);
		/* TODO: Store the error in hal context */
		chan_event = DWC2_CHAN_EVENT_ERROR;
	} else if (hcint & USB_DWC2_HCINT_CHHLTD) {
		if (chan->halt_requested) {
			chan->halt_requested = 0;
			chan_event = DWC2_CHAN_EVENT_HALT_REQ;
		} else {
			chan_event = DWC2_CHAN_EVENT_CPLT;
		}
		chan->active = 0;
	} else if (hcint & USB_DWC2_HCINT_XFERCOMPL) {
		/* Note:
		 * The channel isn't halted yet, so we need to halt it manually to stop the
		 * execution of the next packet. Relevant only for Scatter-Gather DMA and never
		 * occurs oin Buffer DMA.
		 */
		sys_set_bits((mem_addr_t)&chan_regs->hcchar, USB_DWC2_HCCHAR0_CHDIS);

		/*
		 * After setting the halt bit, this will generate another channel halted interrupt.
		 * We treat this interrupt as a NONE event, then cycle back with the channel halted
		 * interrupt to handle the CPLT event.
		 */
		chan_event = DWC2_CHAN_EVENT_NONE;
	} else {
		__ASSERT(false, "Unknown channel interrupt, HCINT=%08Xh", hcint);
		chan_event = DWC2_CHAN_EVENT_NONE;
	}
	return chan_event;
}

struct uhc_dwc2_chan *uhc_dwc2_get_chan_pending_intr(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	if (priv->channels == NULL) {
		LOG_WRN("uhc_dwc2_get_chan_pending_intr: No channels allocated");
		return NULL;
	}

	int chan_num = __builtin_ffs(priv->pending_channel_intrs_msk);
	if (chan_num) {
		/* Clear the pending bit for that channel */
		priv->pending_channel_intrs_msk &= ~(1 << (chan_num - 1));
		return priv->channels[chan_num - 1];
	} else {
		return NULL;
	}
}

struct uhc_transfer *chan_get_next_xfer(struct uhc_dwc2_chan *chan)
{
	struct uhc_transfer *xfer;
	sys_dnode_t *node = sys_dlist_peek_head(&chan->xfer_pending_list);
	return (node == NULL) ? NULL : SYS_DLIST_CONTAINER(node, xfer, node);
}

static inline uint16_t calc_packet_count(const uint16_t size, const uint8_t mps)
{
	if (size == 0) {
		return 1; /* in Buffer DMA mode Zero Length Packet still counts as 1 packet */
	} else {
		return DIV_ROUND_UP(size, mps);
	}
}

static inline bool _buffer_check_done(struct uhc_dwc2_chan *chan)
{
	/* Only control transfers need to be continued */
	if (chan->type != UHC_DWC2_XFER_TYPE_CTRL) {
		return true;
	}

	return (chan->cur_stg == 2);
}

static inline void _buffer_fill_ctrl(struct uhc_dwc2_chan *chan, struct uhc_transfer *const xfer)
{
	/* Get information about the control transfer by analyzing the setup packet */
	const struct usb_setup_packet *setup_pkt = (const struct usb_setup_packet *)xfer->setup_pkt;

	chan->cur_stg = 0;
	chan->data_stg_in = usb_reqtype_is_to_host(setup_pkt);
	chan->data_stg_skip = (setup_pkt->wLength == 0);
	chan->set_addr = 0;

	if (setup_pkt->bRequest == USB_SREQ_SET_ADDRESS) {
		chan->set_addr = 1;
		chan->new_addr = setup_pkt->wValue & 0x7F;
		LOG_DBG("Set address request, new address %d", chan->new_addr);
	}

	LOG_DBG("data_stg_in: %d, data_stg_skip: %d", chan->data_stg_in, chan->data_stg_skip);

	/* Save the xfer pointer in the buffer */
	chan->xfer = xfer;

	/* TODO Sync data from cache to memory. For OUT and CTRL transfers */
}

static void IRAM_ATTR _buffer_fill(struct uhc_dwc2_chan *chan)
{
	struct uhc_transfer *xfer = chan_get_next_xfer(chan);
	chan->num_xfer_pending--;

	/* TODO: Double buffering scheme? */

	switch (chan->type) {
	case UHC_DWC2_XFER_TYPE_CTRL: {
		_buffer_fill_ctrl(chan, xfer);
		break;
	}
	default: {
		LOG_ERR("Unsupported transfer type %d", chan->type);
		break;
	}
	}
	/* TODO: sync CACHE */
}

static inline enum uhc_dwc2_ctrl_stage cal_next_pid(enum uhc_dwc2_ctrl_stage pid, uint8_t pkt_count)
{
	if (pkt_count & 0x01) {
		/* Toggle DATA0 and DATA1 */
		return pid ^ 0x02;
	} else {
		return pid;
	}
}

static void IRAM_ATTR _buffer_exec_proceed(const struct device *dev, struct uhc_dwc2_chan *chan)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_transfer *const xfer = chan->xfer;
	const struct usb_dwc2_host_chan *chan_regs = UHC_DWC2_CHAN_REG(dwc2, chan->chan_idx);

	__ASSERT(xfer != NULL, "_buffer_exec_proceed: No transfer assigned to buffer");
	__ASSERT(chan->cur_stg != 2, "_buffer_exec: Invalid control stage: %d",
		 chan->cur_stg);

	bool next_dir_is_in;
	enum uhc_dwc2_ctrl_stage next_pid;
	uint16_t size = 0;
	uint8_t *dma_addr = NULL;

	if (chan->cur_stg == 0) { /* Just finished control stage */
		if (chan->data_stg_skip) {
			/* No data stage. Go straight to status stage */
			next_dir_is_in = true; /* With no data stage, status stage must be IN */
			next_pid = CTRL_STAGE_DATA1; /* Status stage always has a PID of DATA1 */
			chan->cur_stg = 2; /* Skip over */
		} else {
			/* Go to data stage */
			next_dir_is_in = chan->data_stg_in;
			next_pid =
				CTRL_STAGE_DATA1; /* Data stage always starts with a PID of DATA1 */
			chan->cur_stg = 1;

			/* NOTE:
			 * For OUT - number of bytes host sends to device
			 * For IN - number of bytes host reserves to receive
			 */
			size = xfer->buf->size;

			/* TODO: Toggle PID? */

			/* TODO: Check if the buffer is large enough for the next transfer? */

			/* TODO: Check that the buffer is DMA and CACHE aligned and compatible with
			 * the DMA */
			/* (better to do this on enqueue) */

			if (xfer->buf != NULL) {
				/* Get the tail of the buffer to append data */
				dma_addr = net_buf_tail(xfer->buf);
				/* TODO: Ensure the buffer has enough space? */
				net_buf_add(xfer->buf, size);
			}
		}
	} else { /* cur_stg == 1. Just finished data stage. Go to status stage */
		/* Status stage is always the opposite direction of data stage */
		next_dir_is_in = !chan->data_stg_in;
		next_pid = CTRL_STAGE_DATA1; /* Status stage always has a PID of DATA1 */
		chan->cur_stg = 2;
	}

	/* Calculate new packet count */
	const uint16_t pkt_cnt = calc_packet_count(size, chan->mps);

	if (next_dir_is_in) {
		sys_set_bits((mem_addr_t)&chan_regs->hcchar, USB_DWC2_HCCHAR0_EPDIR);
	} else {
		sys_clear_bits((mem_addr_t)&chan_regs->hcchar, USB_DWC2_HCCHAR0_EPDIR);
	}

	uint32_t hctsiz = ((next_pid << USB_DWC2_HCTSIZ_PID_POS) & USB_DWC2_HCTSIZ_PID_MASK) |
			  ((pkt_cnt << USB_DWC2_HCTSIZ_PKTCNT_POS) & USB_DWC2_HCTSIZ_PKTCNT_MASK) |
			  ((size << USB_DWC2_HCTSIZ_XFERSIZE_POS) & USB_DWC2_HCTSIZ_XFERSIZE_MASK);
	sys_write32(hctsiz, (mem_addr_t)&chan_regs->hctsiz);
	sys_write32((uint32_t)dma_addr, (mem_addr_t)&chan_regs->hcdma);

	/* TODO: Configure split transaction if needed */

	/* TODO: sync CACHE */
	uint32_t hcchar = sys_read32((mem_addr_t)&chan_regs->hcchar);
	hcchar |= USB_DWC2_HCCHAR0_CHENA;
	hcchar &= ~USB_DWC2_HCCHAR0_CHDIS;
	sys_write32(hcchar, (mem_addr_t)&chan_regs->hcchar);
}

static inline void _buffer_done(const struct device *dev, struct uhc_dwc2_chan *chan,
				enum uhc_dwc2_chan_event chan_event, bool canceled)
{
	chan->executing = 0;
	chan->was_canceled = canceled;
	chan->chan_event = chan_event;
}

static inline bool _buffer_can_fill(struct uhc_dwc2_chan *chan)
{
	/* We can only fill if there are pending XFRs and at least one unfilled buffer */
	return (chan->num_xfer_pending > 0);

	/* TODO: Double buffering scheme? */
}

static inline bool _buffer_can_exec(struct uhc_dwc2_chan *chan)
{
	/* TODO: Double buffering scheme? */
	/* For one buffer we can execute it always */
	return true;
}

/*
 * Decode a channel interrupt and take appropriate action
 * Interrupt context.
 */
static enum uhc_dwc2_chan_event uhc_dwc2_decode_chan(const struct device *dev,
						     struct uhc_dwc2_chan *chan)
{
	enum uhc_dwc2_chan_event chan_event = uhc_dwc2_hal_chan_decode_intr(dev, chan);
	enum uhc_dwc2_chan_event ret = DWC2_CHAN_EVENT_NONE;

	LOG_DBG("Channel event: %d", chan_event);

	switch (chan_event) {
	case DWC2_CHAN_EVENT_NONE: {
		/* No event, nothing to do */
		break;
	}
	case DWC2_CHAN_EVENT_CPLT: {
		if (!_buffer_check_done(chan)) {
			_buffer_exec_proceed(dev, chan);
			break;
		}
		chan->last_event = DWC2_CHAN_EVENT_CPLT;
		ret = chan->last_event;
		_buffer_done(dev, chan, chan->last_event, false);
		break;
	}
	case DWC2_CHAN_EVENT_ERROR: {
		LOG_ERR("Channel error handling not implemented yet");
		/* TODO: get channel error, halt the chan */
		break;
	}
	case DWC2_CHAN_EVENT_HALT_REQ: {
		LOG_ERR("Channel halt request handling not implemented yet");

		__ASSERT(chan->waiting_halt,
			 "uhc_dwc2_decode_chan: Pipe is not watiting to be halted");

		/* TODO: Implement halting the ongoing transfer */

		/* Hint:
		 * We've halted a transfer, so we need to trigger the chan callback
		 * chan->last_event = DWC2_CHAN_EVENT_CPLT;
		 * ret = chan->last_event;
		 * Halt request event is triggered when packet is successful completed.
		 * But just treat all halted transfers as errors
		 * chan->state = UHC_CHAN_STATE_HALTED;
		 * Notify the task waiting for the chan halt or halt it right away
		 * _internal_chan_event_notify(chan, true);
		 */
		break;
	}
	default:
		/* Should never happen */
		LOG_WRN("Unknown channel event %d", chan_event);
		break;
	}

	return ret;
}

static IRAM_ATTR void _buffer_exec(const struct device *dev, struct uhc_dwc2_chan *chan)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_transfer *const xfer = (struct uhc_transfer *)chan->xfer;
	const struct usb_dwc2_host_chan *chan_regs = UHC_DWC2_CHAN_REG(dwc2, chan->chan_idx);

	LOG_DBG("ep=%02X, mps=%d", xfer->ep, chan->mps);

	if (USB_EP_GET_IDX(xfer->ep) == 0) {
		/* Control stage is always OUT */
		sys_clear_bits((mem_addr_t)&chan_regs->hcchar, USB_DWC2_HCCHAR0_EPDIR);
	}

	if (xfer->interval != 0) {
		LOG_ERR("Periodic transfer is not supported");
	}

	const uint16_t pkt_cnt =
		calc_packet_count(sizeof(struct usb_setup_packet), chan->mps);
	int next_pid = CTRL_STAGE_SETUP;
	uint16_t size = sizeof(struct usb_setup_packet);

	uint32_t hctsiz = ((next_pid << USB_DWC2_HCTSIZ_PID_POS) & USB_DWC2_HCTSIZ_PID_MASK) |
			  ((pkt_cnt << USB_DWC2_HCTSIZ_PKTCNT_POS) & USB_DWC2_HCTSIZ_PKTCNT_MASK) |
			  ((size << USB_DWC2_HCTSIZ_XFERSIZE_POS) & USB_DWC2_HCTSIZ_XFERSIZE_MASK);
	sys_write32(hctsiz, (mem_addr_t)&chan_regs->hctsiz);

	sys_write32((uint32_t)xfer->setup_pkt, (mem_addr_t)&chan_regs->hcdma);

	/* TODO: Configure split transaction if needed */

	uint32_t hcint = sys_read32((mem_addr_t)&chan_regs->hcint);
	sys_write32(hcint, (mem_addr_t)&chan_regs->hcint);

	/* TODO: sync CACHE */
	uint32_t hcchar = sys_read32((mem_addr_t)&chan_regs->hcchar);
	hcchar |= USB_DWC2_HCCHAR0_CHENA;
	hcchar &= ~USB_DWC2_HCCHAR0_CHDIS;
	sys_write32(hcchar, (mem_addr_t)&chan_regs->hcchar);

	chan->executing = 1;
}

static void uhc_dwc2_isr_handler(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	unsigned int key = irq_lock();

	enum uhc_dwc2_core_event core_event = uhc_dwc2_decode_intr(dev);
	if (core_event == UHC_DWC2_CORE_EVENT_CHAN) {
		/* Channel event. Cycle through each pending channel */
		struct uhc_dwc2_chan *chan = uhc_dwc2_get_chan_pending_intr(dev);

		while (chan != NULL) {
			enum uhc_dwc2_chan_event chan_event = uhc_dwc2_decode_chan(dev, chan);
			if (chan_event != DWC2_CHAN_EVENT_NONE) {
				chan->last_event = chan_event;
				chan->event_pending = 1;
				k_event_post(&priv->drv_evt, BIT(UHC_DWC2_EVENT_CHAN));
			}
			/* Check for more channels with pending interrupts. Returns NULL if there
			 * are no more */
			chan = uhc_dwc2_get_chan_pending_intr(dev);
		}
	} else {
		if (core_event != UHC_DWC2_CORE_EVENT_NONE) {
			/* Port event */
			enum uhc_port_event port_event = uhc_dwc2_decode_hprt(dev, core_event);
			if (port_event != UHC_PORT_EVENT_NONE) {
				priv->last_event = port_event;
				priv->event_pending = 1;
				k_event_post(&priv->drv_evt, BIT(UHC_DWC2_EVENT_PORT));
			}
		} else {
			/* No core event, nothing to do. Should never occur */
			__ASSERT(false, "uhc_dwc2_isr_handler: No core event detected");
		}
	}

	irq_unlock(key);

	(void)uhc_dwc2_quirk_irq_clear(dev);
}

/* TODO: critical section */
static inline bool uhc_dwc2_port_debounce(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;

	/* TODO: exit critical section */
	k_msleep(DEBOUNCE_DELAY_MS);
	/* TODO: enter critical section */

	/* Check the post-debounce state (i.e., whether it's actually connected/disconnected) */
	bool is_connected = ((sys_read32((mem_addr_t)&dwc2->hprt) & USB_DWC2_HPRT_PRTCONNSTS) != 0);
	if (is_connected) {
		priv->port_state = UHC_PORT_STATE_DISABLED;
	} else {
		priv->port_state = UHC_PORT_STATE_DISCONNECTED;
	}
	/* Disable debounce lock */
	uhc_dwc2_lock_disable(dev);
	return is_connected;
}

static inline enum uhc_port_event uhc_dwc2_get_port_event(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	enum uhc_port_event ret = UHC_PORT_EVENT_NONE;
	/* TODO: enter critial section */
	if (priv->event_pending) {
		priv->event_pending = 0;
		ret = priv->last_event;
		switch (ret) {
		case UHC_PORT_EVENT_CONNECTION: {
			/* Don't update state immediately, we still need to debounce. */
			if (uhc_dwc2_port_debounce(dev)) {
				ret = UHC_PORT_EVENT_CONNECTION;
			} else {
				LOG_ERR("Port is not connected after debounce");
				/* TODO: Simulate and/or verify */
				LOG_WRN("Port debounce error handling is not implemented yet");
			}
			break;
		}
		case UHC_PORT_EVENT_DISCONNECTION:
		case UHC_PORT_EVENT_ERROR:
		case UHC_PORT_EVENT_OVERCURRENT: {
			break;
		}
		default: {
			break;
		}
		}
	} else {
		ret = UHC_PORT_EVENT_NONE;
	}
	/* TODO: exit critical section */
	return ret;
}

static inline void uhc_dwc2_flush_chans(const struct device *dev)
{
	/* TODO: For each chan, reinitialize the channel with EP characteristics */
	/* Flush the channel EP characteristics */
	/* TODO: Sync CACHE */
}

static inline void dwc2_apply_fifo_config(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	/* Get FIFO top from config */
	uint16_t fifo_available = priv->fifo_top;

	sys_write32(fifo_available << USB_DWC2_GDFIFOCFG_EPINFOBASEADDR_POS | fifo_available,
		    (mem_addr_t)&dwc2->gdfifocfg);

	fifo_available -= priv->fifo_rxfsiz;

	sys_write32(priv->fifo_rxfsiz << USB_DWC2_GRXFSIZ_RXFDEP_POS, (mem_addr_t)&dwc2->grxfsiz);

	fifo_available -= priv->fifo_nptxfsiz;

	sys_write32(priv->fifo_nptxfsiz << USB_DWC2_GNPTXFSIZ_NPTXFDEP_POS | fifo_available,
		    (mem_addr_t)&dwc2->gnptxfsiz);

	fifo_available -= priv->fifo_ptxfsiz;

	sys_write32(priv->fifo_ptxfsiz << USB_DWC2_HPTXFSIZ_PTXFSIZE_POS | fifo_available,
		    (mem_addr_t)&dwc2->hptxfsiz);

	dwc2_hal_flush_tx_fifo(dwc2, 0x10UL);
	dwc2_hal_flush_rx_fifo(dwc2);

	LOG_DBG("FIFO configuration applied");
	LOG_DBG("\tnptx=%u, rx=%u, ptx=%u", priv->fifo_nptxfsiz * 4, priv->fifo_rxfsiz * 4,
		priv->fifo_ptxfsiz * 4);
}

static inline int uhc_dwc2_port_reset(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;

	int ret;

	/* Enter critical section */
	unsigned int key = irq_lock();

	/* TODO: implement port checks */

	/*
	 * Hint:
	 * Port can only a reset when it is in the enabled or disabled (in the case of a new
	 * connection) states. priv->port_state == UHC_PORT_STATE_ENABLED;
	 * priv->port_state == UHC_PORT_STATE_DISABLED;
	 * priv->num_channels_chans_queued == 0
	 */

	/*
	Proceed to resetting the bus:
	- Update the port's state variable
	- Hold the bus in the reset state for RESET_HOLD_MS.
	- Return the bus to the idle state for RESET_RECOVERY_MS
	During this reset the port state should be set to RESETTING and do not change.
	*/
	priv->port_state = UHC_PORT_STATE_RESETTING;
	dwc2_hal_toggle_reset(dwc2, true);

	/* Exit critical section */
	irq_unlock(key);

	/* Hold the bus in the reset state */
	k_msleep(RESET_HOLD_MS);

	/* Enter critical section */
	key = irq_lock();

	if (priv->port_state != UHC_PORT_STATE_RESETTING) {
		/* The port state has unexpectedly changed */
		LOG_ERR("Port state changed during reset");
		ret = -EIO;
		goto bailout;
	}

	/* Return the bus to the idle state. Port enabled event should occur */
	dwc2_hal_toggle_reset(dwc2, false);

	/* Exit critical section */
	irq_unlock(key);

	/* Give the port time to recover */
	k_msleep(RESET_RECOVERY_MS);

	/* TODO: enter critical section */
	if (priv->port_state != UHC_PORT_STATE_RESETTING ||
	    !priv->conn_dev_ena) {
		/* The port state has unexpectedly changed */
		LOG_ERR("Port state changed during reset");
		ret = -EIO;
		goto bailout;
	}

	dwc2_apply_fifo_config(dev);
	dwc2_hal_set_frame_list(dwc2, NULL /* priv->frame_list , FRAME_LIST_LEN */);
	dwc2_hal_periodic_enable(dwc2);
	ret = 0;
bailout:
	uhc_dwc2_flush_chans(dev);
	return ret;
}

/*
 * Port recovery is necessary when the port is in an error state and needs to be reset.
 */
static inline int uhc_dwc2_port_recovery(const struct device *dev)
{
	int ret;

	/* TODO: Implement port checks */
	/* Port should be in recovery state and no ongoing transfers */
	/* Port flags should be 0 */

	/* TODO: enter critical section */
	ret = uhc_dwc2_quirk_irq_disable_func(dev);
	if (ret) {
		LOG_ERR("Quirk IRQ disable failed %d", ret);
		return ret;
	}

	/* Init controller */
	ret = uhc_dwc2_init_controller(dev);
	if (ret) {
		LOG_ERR("Failed to init controller: %d", ret);
		return ret;
	}

	ret = uhc_dwc2_quirk_irq_enable_func(dev);
	if (ret) {
		LOG_ERR("Quirk IRQ enable failed %d", ret);
		return ret;
	}
	/* TODO: exit critical section */

	ret = uhc_dwc2_power_on(dev);
	if (ret) {
		LOG_ERR("Failed to power on root port: %d", ret);
		return ret;
	}

	return ret;
}

/*
 * Submit a new device connected event to the higher logic.
 */
static inline void uhc_dwc2_submit_new_device(const struct device *dev, enum uhc_dwc2_speed speed)
{
	const char *uhc_dwc2_speed_str[] = { "High", "Full", "Low" };
	enum uhc_event_type type;

	LOG_WRN("New dev, %s Speed", uhc_dwc2_speed_str[speed]);

	switch (speed) {
	case UHC_DWC2_SPEED_LOW:
		type = UHC_EVT_DEV_CONNECTED_LS;
		break;
	case UHC_DWC2_SPEED_FULL:
		type = UHC_EVT_DEV_CONNECTED_FS;
		break;
	case UHC_DWC2_SPEED_HIGH:
		type = UHC_EVT_DEV_CONNECTED_HS;
		break;
	default:
		LOG_ERR("Unsupported speed %d", speed);
		return;
	}
	uhc_submit_event(dev, type, 0);
}

/*
 * Submit a device gone event to the higher logic.
 */
static inline void uhc_dwc2_submit_dev_gone(const struct device *dev)
{
	LOG_WRN("Dev gone");
	uhc_submit_event(dev, UHC_EVT_DEV_REMOVED, 0);
}

/*
 * Fills the endpoint characteristics for a chan.
 */
static void uhc_dwc2_chan_set_ep_char(const struct uhc_dwc2_chan_config *chan_config,
				      enum uhc_dwc2_xfer_type type, bool is_ctrl_chan, int chan_idx,
				      enum uhc_dwc2_speed port_speed, struct uhc_dwc2_chan *chan)
{
	enum uhc_dwc2_xfer_type dw2_ll_xfer_type;

	if (type == UHC_DWC2_XFER_TYPE_CTRL) {
		dw2_ll_xfer_type = UHC_DWC2_XFER_TYPE_CTRL;
	} else {
		LOG_ERR("Unsupported transfer type %d", type);
		return;
	}

	chan->type = dw2_ll_xfer_type;

	if (is_ctrl_chan) {
		chan->bEndpointAddress = 0;
		/* Set the default chan's MPS to the worst case MPS for the device's speed */
		chan->mps = (chan_config->dev_speed == UHC_DWC2_SPEED_LOW)
				       ? CTRL_EP_MAX_MPS_LS
				       : CTRL_EP_MAX_MPS_HSFS;
	} else {
		/* TODO: Implement for non-control chans */
		LOG_WRN("Setting up chan characteristics for non-control chan has not implemented "
			"yet");
		return;
	}

	chan->dev_addr = chan_config->dev_addr;
	chan->ls_via_fs_hub = 0;
	chan->interval = 0;
	chan->offset = 0;
}

/*
 * Adds the channel object to the channel list and initializes it.
 */
static inline bool uhc_dwc2_chan_init(const struct device *dev, struct uhc_dwc2_chan *chan)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;
	const struct usb_dwc2_host_chan *chan_regs = UHC_DWC2_CHAN_REG(dwc2, chan->chan_idx);

	__ASSERT(priv->channels, "uhc_dwc2_chan_alloc: Channel handles list not allocated");

	/* TODO: FIFO sizes should be set before attempting to allocate a channel */

	if (priv->num_channels == priv->dwc2_numchannels) {
		return false;
	}

	uint8_t chan_idx = 0xff;
	for (uint8_t i = 0; i < priv->dwc2_numchannels; i++) {
		if (priv->channels[i] == NULL) {
			priv->channels[i] = chan;
			chan_idx = i;
			priv->num_channels++;
			break;
		}
	}

	__ASSERT(chan_idx != 0xff, "No free channels available, num_channels=%d, numchannels=%d",
		 priv->num_channels, priv->dwc2_numchannels);

	/* Initialize channel object */
	LOG_DBG("Allocating channel %d", chan_idx);

	chan->chan_idx = chan_idx;

	/* Init underlying channel registers */

	/* Clear the interrupt bits by writing them back */
	uint32_t hcint = sys_read32((mem_addr_t)&chan_regs->hcint);
	sys_write32(hcint, (mem_addr_t)&chan_regs->hcint);

	/* Enable channel interrupts in the core */
	sys_set_bits((mem_addr_t)&dwc2->haintmsk, (1 << chan_idx));

	/* Enable transfer complete and channel halted interrupts */
	sys_set_bits((mem_addr_t)&chan_regs->hcintmsk,
		     USB_DWC2_HCINT_XFERCOMPL | USB_DWC2_HCINT_CHHLTD);
	return true;
}

/*
 * Allocate a chan holding the underlying channel object and the DMA buffer for transfer purposes.
 */
static inline int uhc_dwc2_chan_alloc(const struct device *dev,
				      const struct uhc_dwc2_chan_config *chan_config,
				      struct uhc_dwc2_chan **chan_ptr)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	int ret;

	/* TODO: Allocate the chan and it's resources */
	struct uhc_dwc2_chan *chan = &priv->chan;

	/* TODO: Double buffering scheme? */

	enum uhc_dwc2_xfer_type type = UHC_DWC2_XFER_TYPE_CTRL;
	/* TODO: Refactor to get port speed, static for now */
	enum uhc_dwc2_speed port_speed = UHC_DWC2_SPEED_FULL;
	bool is_default = true;
	int chan_idx = 0;

	uhc_dwc2_chan_set_ep_char(chan_config, type, is_default, chan_idx, port_speed, chan);
	chan->state = UHC_CHAN_STATE_ACTIVE;

	/* TODO: enter critical section */
	if (!priv->conn_dev_ena) {
		/* TODO: exit critical section */
		LOG_ERR("Port is not enabled, cannot allocate channel");
		ret = -ENODEV;
		goto err;
	}

	bool chan_allocated = uhc_dwc2_chan_init(dev, chan);
	if (!chan_allocated) {
		/* TODO: exit critical section */
		LOG_ERR("No more free channels available");
		ret = -ENOMEM;
		goto err;
	}

	dwc2_channel_configure(dev, chan);

	/* TODO: sync CACHE */

	/* TODO: Add the chan to the list of idle chans in the port object */
	sys_dlist_init(&chan->xfer_pending_list);
	priv->num_chans_idle++;

	/* TODO: exit critical section */

	*chan_ptr = chan;
	return 0;
err:
	return ret;
}

/*
 * Free the chan and its resources.
 */
static inline int uhc_dwc2_chan_free(const struct device *dev, struct uhc_dwc2_chan *chan)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	if (chan->num_xfer_pending && chan->has_xfer) {
		LOG_ERR("Unable to free chan with pending XFERs");
		return -EBUSY;
	}


	__ASSERT(priv->channels, "uhc_dwc2_chan_alloc: Channel handles list not allocated");

	if (chan->type == UHC_DWC2_XFER_TYPE_INTR ||
	    chan->type == UHC_DWC2_XFER_TYPE_ISOCHRONOUS) {
		/* TODO: Unschedule this channel */
		LOG_WRN("uhc_dwc2_chan_free: Cannot free interrupt or isochronous channels yet");
	}

	__ASSERT(!chan->active,
		 "uhc_dwc2_chan_free: Cannot free channel %d, it is still active",
		 chan->chan_idx);

	sys_clear_bits((mem_addr_t)&dwc2->haintmsk, (1 << chan->chan_idx));

	priv->channels[chan->chan_idx] = NULL;
	priv->num_channels--;

	LOG_DBG("Freeing channel %d, num_channels=%d", chan->chan_idx,
		priv->num_channels);

	__ASSERT(priv->num_channels >= 0,
		 "uhc_dwc2_chan_free: Number of allocated channels is negative: %d",
		 priv->num_channels);

	/* TODO: Remove the chan from the list of idle chans in the port object */
	priv->num_chans_idle--;

	return 0;
}

static inline void uhc_dwc2_handle_port_events(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	enum uhc_port_event port_event = uhc_dwc2_get_port_event(dev);
	int ret;

	LOG_DBG("Port event: %d", port_event);

	switch (port_event) {
	case UHC_PORT_EVENT_NONE:
		/* No event, nothing to do */
		break;
	case UHC_PORT_EVENT_CONNECTION: {
		uhc_dwc2_port_reset(dev);
		break;
	}

	case UHC_PORT_EVENT_ENABLED: {
		/* TODO: enter critical section */
		priv->port_state = UHC_PORT_STATE_ENABLED;
		/* TODO: exit critical section */

		enum uhc_dwc2_speed speed;

		ret = uhc_dwc2_get_port_speed(dev, &speed);
		if (ret) {
			LOG_ERR("Failed to get port speed");
			break;
		}

		/* Allocate the Pipe for the EP0 Control Endpoint */
		struct uhc_dwc2_chan *ctrl_chan;
		struct uhc_dwc2_chan_config chan_config = {
			.dev_speed = speed,
			.dev_addr = 0,
		};

		ret = uhc_dwc2_chan_alloc(dev, &chan_config, &ctrl_chan);
		if (ret) {
			LOG_ERR("Failed to initialize channels: %d", ret);
			break;
		}
		/* Save the control chan handle in the private data */
		priv->ctrl_chan = ctrl_chan;
		/* Notify the higher logic about the new device */
		uhc_dwc2_submit_new_device(dev, speed);
		break;
	}
	case UHC_PORT_EVENT_DISCONNECTION:
	case UHC_PORT_EVENT_ERROR:
	case UHC_PORT_EVENT_OVERCURRENT: {
		bool port_has_device = false;

		/* TODO: enter critical section */
		switch (priv->port_state) {
		case UHC_PORT_STATE_DISABLED:
			break;
		case UHC_PORT_STATE_NOT_POWERED:
		case UHC_PORT_STATE_ENABLED:
			port_has_device = true;
			break;
		default:
			LOG_ERR("Unexpected port state %d", priv->port_state);
			break;
		}
		/* TODO: exit critical section */

		if (port_has_device) {
			uhc_dwc2_chan_free(dev, priv->ctrl_chan);
			priv->ctrl_chan = NULL;
			uhc_dwc2_submit_dev_gone(dev);
		}
		/* Recover the port */
		uhc_dwc2_port_recovery(dev);
		break;
	}
	default:
		break;
	}
}

static inline void uhc_dwc2_handle_chan_events(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	/* TODO: support more than CTRL chan */
	struct uhc_dwc2_chan *chan = &priv->chan;
	const struct usb_dwc2_host_chan *chan_regs = UHC_DWC2_CHAN_REG(dwc2, chan->chan_idx);

	LOG_DBG("Pipe event: %d", chan->last_event);

	if (chan->last_event == DWC2_CHAN_EVENT_CPLT) {
		/* XFER transfer is done, process the transfer and release the chan buffer */
		struct uhc_transfer *const xfer = (struct uhc_transfer *)chan->xfer;

		if (xfer->buf != NULL && xfer->buf->len) {
			LOG_HEXDUMP_WRN(xfer->buf->data, xfer->buf->len, "data");
		}

		/* TODO: Refactor the address setting logic. */
		/* To configure the channel, we need to get the dev addr from higher logic */
		if (chan->set_addr) {
			chan->set_addr = 0;
			chan->dev_addr = chan->new_addr;
			/* Set the new device address in the channel */
			sys_set_bits((mem_addr_t)&chan_regs->hcchar,
				     (chan->dev_addr << USB_DWC2_HCCHAR0_DEVADDR_POS));
			k_msleep(SET_ADDR_DELAY_MS);
		}

		/* TODO: Refactor chan resources release */
		chan->has_xfer = 0;
		priv->num_chans_idle++;
		priv->num_chans_queued--;

		uhc_xfer_return(dev, xfer, 0);

	} else {
		/* TODO: Handle the rest chan events */
		LOG_ERR("Unhandled chan event %d", chan->last_event);
	}
}

/*
 * Thread that processes USB events from the DWC2 controller: Port, Pipe.
 */
static inline void uhc_dwc2_thread_handler(void *const arg)
{
	const struct device *dev = (const struct device *)arg;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);

	uint32_t evt = k_event_wait(&priv->drv_evt, UINT32_MAX, false, K_FOREVER);

	/* The role of this line is only to shift the .text area or insert delay, either
	 * of which temporarily fixes a crash which I am currently debugging step-by-step.
	 */
	//LOG_DBG("%d", __LINE__);
	uhc_lock_internal(dev, K_FOREVER);

	if (evt & BIT(UHC_DWC2_EVENT_PORT)) {
		k_event_clear(&priv->drv_evt, BIT(UHC_DWC2_EVENT_PORT));
		uhc_dwc2_handle_port_events(dev);
	}

	if (evt & BIT(UHC_DWC2_EVENT_CHAN)) {
		k_event_clear(&priv->drv_evt, BIT(UHC_DWC2_EVENT_CHAN));
		uhc_dwc2_handle_chan_events(dev);
	}

	uhc_unlock_internal(dev);
}

static inline int uhc_dwc2_submit_ctrl_xfer(const struct device *dev, struct uhc_dwc2_chan *chan,
					    struct uhc_transfer *const xfer)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	LOG_HEXDUMP_WRN(xfer->setup_pkt, 8, "setup");

	LOG_DBG("endpoint=%02Xh, mps=%d, interval=%d, start_frame=%d, stage=%d, no_status=%d",
		xfer->ep, xfer->mps, xfer->interval, xfer->start_frame, xfer->stage,
		xfer->no_status);

	/* TODO: Check that XFER has not already been enqueued? */

	/* TODO: setup packet must be aligned 4 bytes? */
	if (((uintptr_t)xfer->setup_pkt % 4)) {
		LOG_WRN("Setup packet address %p is not 4-byte aligned", xfer->setup_pkt);
	}

	/* TODO: Buffer addr that will used as dma addr also should be aligned */
	if ((xfer->buf != NULL) && ((uintptr_t)net_buf_tail(xfer->buf) % 4)) {
		LOG_WRN("XFER buffer address %08lXh is not 4-byte aligned",
			(uintptr_t)net_buf_tail(xfer->buf));
	}

	sys_dlist_append(&chan->xfer_pending_list, &xfer->node);
	chan->num_xfer_pending++;

	unsigned int key = irq_lock();

	if (_buffer_can_fill(chan)) {
		_buffer_fill(chan);
	}
	if (_buffer_can_exec(chan)) {
		_buffer_exec(dev, chan);
	}

	if (!chan->has_xfer) {
		/* This is the first XFER to be enqueued into the chan. */
		/* TODO: remove chan from idle chans list */
		/* TODO: add chan to active chans list */
		priv->num_chans_idle--;
		priv->num_chans_queued++;
		chan->has_xfer = 1;
	}

	irq_unlock(key);

	return 0;
}

/*
 * UHC DWC2 Driver API
 */

static int uhc_dwc2_lock(const struct device *dev)
{
	struct uhc_data *data = dev->data;

	return k_mutex_lock(&data->mutex, K_FOREVER);
}

static int uhc_dwc2_unlock(const struct device *dev)
{
	struct uhc_data *data = dev->data;

	return k_mutex_unlock(&data->mutex);
}

static int uhc_dwc2_sof_enable(const struct device *dev)
{
	LOG_ERR("uhc_dwc2_sof_enable is not implemented yet");
	return -ENOSYS;
}

static int uhc_dwc2_bus_suspend(const struct device *dev)
{
	LOG_ERR("uhc_dwc2_bus_suspend is not implemented yet");
	return -ENOSYS;
}

static int uhc_dwc2_bus_reset(const struct device *dev)
{
	/* TODO: move the reset logic here */

	/* Hint: First reset is done by the uhc dwc2 driver, so we don't need to do anything here */
	uhc_submit_event(dev, UHC_EVT_RESETED, 0);
	return 0;
}

static int uhc_dwc2_bus_resume(const struct device *dev)
{
	LOG_ERR("uhc_dwc2_bus_resume is not implemented yet");
	return -ENOSYS;
}

static int uhc_dwc2_enqueue(const struct device *dev, struct uhc_transfer *const xfer)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	int ret;
	if (USB_EP_GET_IDX(xfer->ep) == 0) {
		ret = uhc_dwc2_submit_ctrl_xfer(dev, priv->ctrl_chan, xfer);
		if (ret) {
			LOG_ERR("Failed to submit xfer: %d", ret);
			return ret;
		}
	} else {
		LOG_ERR("Non-control endpoint enqueue not implemented yet");
		return -ENOSYS;
	}

	return 0;
}

static int uhc_dwc2_dequeue(const struct device *dev, struct uhc_transfer *const xfer)
{
	LOG_ERR("uhc_dwc2_dequeue is not implemented yet");
	return -ENOSYS;
}

static int uhc_dwc2_preinit(const struct device *dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct uhc_data *data= dev->data;

	/* Initialize the private data structure */
	memset(priv, 0, sizeof(struct uhc_dwc2_data));
	priv->ctrl_chan = NULL;
	k_mutex_init(&data->mutex);
	k_mutex_init(&priv->mutex);
	k_event_init(&priv->drv_evt);

	/* TODO: Overwrite the DWC2 register values with the devicetree values? */

	(void)uhc_dwc2_quirk_caps(dev);

	config->make_thread(dev);
	return 0;
}

static int uhc_dwc2_init(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	int ret;

	ret = uhc_dwc2_quirk_init(dev);
	if (ret) {
		LOG_ERR("Quirk init failed %d", ret);
		return ret;
	}

	ret = uhc_dwc2_init_controller(dev);
	if (ret) {
		return ret;
	}

	/* Allocate memory for the channel objects */
	priv->channels = k_malloc(priv->dwc2_numchannels * sizeof(struct uhc_dwc2_chan *));
	if (priv->channels == NULL) {
		LOG_ERR("Failed to allocate channel handles");
		return -ENOMEM;
	}

	for (uint8_t i = 0; i < priv->dwc2_numchannels; i++) {
		priv->channels[i] = NULL;
	}

	return 0;
}

static int uhc_dwc2_enable(const struct device *dev)
{
	int ret;

	ret = uhc_dwc2_quirk_pre_enable(dev);
	if (ret) {
		LOG_ERR("Quirk pre enable failed %d", ret);
		return ret;
	}

	ret = uhc_dwc2_quirk_irq_enable_func(dev);
	if (ret) {
		LOG_ERR("Quirk IRQ enable failed %d", ret);
		return ret;
	}

	ret = uhc_dwc2_power_on(dev);
	if (ret) {
		LOG_ERR("Failed to power on port: %d", ret);
		return ret;
	}

	return 0;
}

static int uhc_dwc2_disable(const struct device *dev)
{
	int ret;

	LOG_WRN("uhc_dwc2_disable has not been fully implemented yet");

	ret = uhc_dwc2_quirk_disable(dev);
	if (ret) {
		LOG_ERR("Quirk disable failed %d", ret);
		return ret;
	}

	return -ENOSYS;
}

static int uhc_dwc2_shutdown(const struct device *dev)
{
	int ret;

	LOG_WRN("uhc_dwc2_shutdown has not been fully implemented yet");

	/* TODO: Release memory for channel handles */
	/* Hint : k_free(priv->channels); */

	ret = uhc_dwc2_quirk_shutdown(dev);
	if (ret) {
		LOG_ERR("Quirk shutdown failed %d", ret);
		return ret;
	}

	return -ENOSYS;
}

/*
 * Device Definition and Initialization
 */

K_THREAD_STACK_DEFINE(uhc_dwc2_stack, CONFIG_UHC_DWC2_STACK_SIZE);

static void uhc_dwc2_thread(void *arg1, void *arg2, void *arg3)
{
	while (true) {
		uhc_dwc2_thread_handler(arg1);
	}
}

static void uhc_dwc2_make_thread(const struct device *dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);

	k_thread_create(&priv->thread_data, uhc_dwc2_stack, K_THREAD_STACK_SIZEOF(uhc_dwc2_stack),
			uhc_dwc2_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_UHC_DWC2_THREAD_PRIORITY), K_ESSENTIAL, K_NO_WAIT);
	k_thread_name_set(&priv->thread_data, dev->name);
}

static const struct uhc_api uhc_dwc2_api = {
	/* Common */
	.lock = uhc_dwc2_lock,
	.unlock = uhc_dwc2_unlock,
	.init = uhc_dwc2_init,
	.enable = uhc_dwc2_enable,
	.disable = uhc_dwc2_disable,
	.shutdown = uhc_dwc2_shutdown,
	/* Bus related */
	.bus_reset = uhc_dwc2_bus_reset,
	.sof_enable = uhc_dwc2_sof_enable,
	.bus_suspend = uhc_dwc2_bus_suspend,
	.bus_resume = uhc_dwc2_bus_resume,
	/* EP related */
	.ep_enqueue = uhc_dwc2_enqueue,
	.ep_dequeue = uhc_dwc2_dequeue,
};

#define UHC_DWC2_DT_INST_REG_ADDR(n)                                                               \
	COND_CODE_1(DT_NUM_REGS(DT_DRV_INST(n)),                                                   \
		    (DT_INST_REG_ADDR(n)),                                                         \
		    (DT_INST_REG_ADDR_BY_NAME(n, core)))

static struct uhc_dwc2_data uhc_dwc2_data = {
	.irq_sem = Z_SEM_INITIALIZER(uhc_dwc2_data.irq_sem, 0, 1),
};

static const struct uhc_dwc2_config uhc_dwc2_config_host = {
	.base = (struct usb_dwc2_reg *)UHC_DWC2_DT_INST_REG_ADDR(0),
	.make_thread = uhc_dwc2_make_thread,
	.quirks = UHC_DWC2_VENDOR_QUIRK_GET(0),
};

static struct uhc_data uhc_dwc2_priv_data = {
	.priv = &uhc_dwc2_data,
};

DEVICE_DT_INST_DEFINE(0, uhc_dwc2_preinit, NULL, &uhc_dwc2_priv_data, &uhc_dwc2_config_host,
		      POST_KERNEL, 99, &uhc_dwc2_api);
