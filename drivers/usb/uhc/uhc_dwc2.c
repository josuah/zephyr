/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Roman Leonov <jam_roma@yahoo.com>
 * SPDX-FileCopyrightText: Copyright Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dwc2

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_ch9.h>

LOG_MODULE_REGISTER(uhc_dwc2, CONFIG_UHC_DRIVER_LOG_LEVEL);

#include <usb_dwc2_hw.h>

#include "uhc_common.h"

/* Mask to clear HPRT register */
#define USB_DWC2_HPRT_W1C_MSK	(USB_DWC2_HPRT_PRTENA |				\
				 USB_DWC2_HPRT_PRTCONNDET |			\
				 USB_DWC2_HPRT_PRTENCHNG |			\
				 USB_DWC2_HPRT_PRTOVRCURRCHNG)

#define EPSIZE_BULK_FS		64
#define EPSIZE_BULK_HS		512
#define EPSIZE_ISO_FS_MAX	1023
#define EPSIZE_ISO_HS_MAX	1024

/* TODO: Configurable? */
#define CONFIG_UHC_DWC2_MAX_CHANNELS		16

#define UHC_DWC2_QUIRK_CONFIG(dev)						\
	(((const struct uhc_dwc2_config *)dev->config)->quirk_config)

#define UHC_DWC2_QUIRK_DATA(dev)						\
	(((const struct uhc_dwc2_config *)dev->config)->quirk_data)

enum uhc_dwc2_event {
	/* No event has occurred or the event is no longer valid */
	UHC_DWC2_EVENT_NONE,
	/* A device has been connected to the port */
	UHC_DWC2_EVENT_PORT_CONNECTION,
	/* A device is connected to the port but has not been reset. */
	UHC_DWC2_EVENT_PORT_DISABLED,
	/* A device disconnection has been detected */
	UHC_DWC2_EVENT_PORT_DISCONNECTION,
	/* Port error detected */
	UHC_DWC2_EVENT_PORT_ERROR,
	/* Overcurrent detected */
	UHC_DWC2_EVENT_PORT_OVERCURRENT,
	/* Port has pending channel event, one bit per channel */
	UHC_DWC2_EVENT_PORT_PEND_CHANNEL
};

enum uhc_dwc2_channel_event {
	/* The channel has completed a transfer. Channel is now halted */
	UHC_DWC2_CHANNEL_EVENT_CPLT,
	/* The channel has completed a transfer. Request STALLed. Channel is now halted */
	UHC_DWC2_CHANNEL_EVENT_STALL,
	/* The channel has encountered an error. Channel is now halted */
	UHC_DWC2_CHANNEL_EVENT_ERROR,
	/* Need to release the channel for the next transfer */
	UHC_DWC2_CHANNEL_DO_RELEASE,
};

enum uhc_dwc2_xfer_type {
	UHC_DWC2_XFER_TYPE_CTRL,
	UHC_DWC2_XFER_TYPE_ISOCHRONOUS,
	UHC_DWC2_XFER_TYPE_BULK,
	UHC_DWC2_XFER_TYPE_INTR,
};

enum uhc_dwc2_channel_pid {
	UHC_DWC2_PID_DATA0 = 0,
	UHC_DWC2_PID_DATA2 = 1,
	UHC_DWC2_PID_DATA1 = 2,
	UHC_DWC2_PID_MDATA_SETUP = 3,
};

#define UHC_DWC2_CHANNEL_REGS(base, chan_idx)					\
	((struct usb_dwc2_host_chan *)(((mem_addr_t)(base)) + USB_DWC2_HCCHAR(chan_idx)))

#define UHC_DWC2_FIFODEPTH(config)						\
	((uint32_t)(((config)->ghwcfg3 & USB_DWC2_GHWCFG3_DFIFODEPTH_MASK) >>	\
		    USB_DWC2_GHWCFG3_DFIFODEPTH_POS))

#define UHC_DWC2_NUMHSTCHNL(config)						\
	((uint32_t)(((config)->ghwcfg2 & USB_DWC2_GHWCFG2_NUMHSTCHNL_MASK) >>	\
		    USB_DWC2_GHWCFG2_NUMHSTCHNL_POS))


/* Interrupts that pertain to core events */
#define CORE_EVENTS_INTRS_MSK	(USB_DWC2_GINTSTS_DISCONNINT |			\
				USB_DWC2_GINTSTS_HCHINT)

/* Interrupt that pertain to host port events */
#define PORT_EVENTS_INTRS_MSK	(USB_DWC2_HPRT_PRTCONNDET |			\
				USB_DWC2_HPRT_PRTENCHNG |			\
				USB_DWC2_HPRT_PRTOVRCURRCHNG)

struct uhc_dwc2_vendor_quirks {
	int (*preinit)(const struct device *const dev);
	int (*init)(const struct device *const dev);
	int (*pre_enable)(const struct device *const dev);
	int (*post_enable)(const struct device *const dev);
	int (*disable)(const struct device *const dev);
	int (*shutdown)(const struct device *const dev);
	int (*irq_clear)(const struct device *const dev);
	int (*phy_pre_select)(const struct device *const dev);
	int (*phy_post_select)(const struct device *const dev);
	int (*is_phy_clk_off)(const struct device *const dev);
	int (*get_phy_clk)(const struct device *const dev);
	int (*post_hibernation_entry)(const struct device *const dev);
	int (*pre_hibernation_exit)(const struct device *const dev);
};

struct uhc_dwc2_config {
	/* Pointer to base address of DWC2 registers */
	struct usb_dwc2_reg *const base;
	/* Thread stack */
	k_thread_stack_t *stack;
	size_t stack_size;
	/* Vendor-specific implementation */
	const struct uhc_dwc2_vendor_quirks *quirk_api;
	void *quirk_data;
	const void *quirk_config;
	/* IRQ configuration */
	void (*irq_enable_func)(const struct device *const dev);
	void (*irq_disable_func)(const struct device *const dev);
	/* Hardware configuration registers */
	uint32_t gsnpsid;
	uint32_t ghwcfg1;
	uint32_t ghwcfg2;
	uint32_t ghwcfg3;
	uint32_t ghwcfg4;
};

struct uhc_dwc2_channel {
	/* Pointer to base address of a channel registers */
	struct usb_dwc2_host_chan *base;
	/* Pointer to the transfer */
	struct uhc_transfer *xfer;
	/* Channel events */
	atomic_t events;
	/* Index of the channel */
	uint8_t index;
	/* Control transfer stage */
	enum uhc_control_stage ctrl_stg;
	/* Associated endpoint characteristics */
	/* Type of endpoint */
	enum uhc_dwc2_xfer_type type;
	/* Address */
	uint8_t ep_addr;
	/* Device Address */
	uint8_t dev_addr;
	/* Maximum Packet Size */
	uint16_t ep_mps;
	/* Channel flags */
	/* Channel is claimed and has associated endpoint */
	uint8_t claimed: 1;
	/* Transfer stage is IN */
	uint8_t data_stg_in: 1;
	/* Transfer has no data stage */
	uint8_t data_stg_skip: 1;
	/* Transfer will change the device address */
	uint8_t set_address : 1;
	/* Channel executing transfer */
	uint8_t executing: 1;
};

struct uhc_dwc2_data {
	struct k_thread thread;
	/* Event bitmask the driver thread waits for */
	struct k_event event;
	/* Semaphore used in a different thread */
	struct k_sem sem_port_en;
	/* Port channels */
	struct uhc_dwc2_channel channel[CONFIG_UHC_DWC2_MAX_CHANNELS];
	/* Bit mask of channels with pending interrupts */
	uint32_t pending_channels_msk;
	/* Port FIFO configuration */
	uint16_t fifo_top;
	uint16_t fifo_nptxfsiz;
	uint16_t fifo_rxfsiz;
	uint16_t fifo_ptxfsiz;
	/* Root Port flags */
	uint8_t debouncing : 1;
	uint8_t has_device : 1;
};

static int uhc_dwc2_soft_reset(const struct device *const dev);
static void uhc_dwc2_isr_handler(const struct device *const dev);

/*
 * Vendor quirks handling
 *
 * Definition of vendor-specific functions that can be overwritten on a per-SoC basis.
 */

#if DT_HAS_COMPAT_STATUS_OKAY(espressif_esp32_usb_otg)
#include "uhc_dwc2_esp32_usb_otg.h"
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_usbhs_nrf54l)
#include "uhc_dwc2_nrf_usbhs_nrf54l.h"
#endif

/* Wrapper functions that fallback to returning 0 if no quirk is needed */
#define DWC2_QUIRK_FUNC_DEFINE(fn)						\
	static int uhc_dwc2_quirk_##fn(const struct device *const dev)		\
	{									\
		const struct uhc_dwc2_config *const config = dev->config;	\
										\
		if (config->quirk_api->fn != NULL) {				\
			return config->quirk_api->fn(dev);			\
		}								\
										\
		return 0;							\
	}

DWC2_QUIRK_FUNC_DEFINE(init);
DWC2_QUIRK_FUNC_DEFINE(pre_enable);
DWC2_QUIRK_FUNC_DEFINE(post_enable);
DWC2_QUIRK_FUNC_DEFINE(disable);
DWC2_QUIRK_FUNC_DEFINE(shutdown);
DWC2_QUIRK_FUNC_DEFINE(irq_clear);
DWC2_QUIRK_FUNC_DEFINE(preinit);
DWC2_QUIRK_FUNC_DEFINE(phy_pre_select);
DWC2_QUIRK_FUNC_DEFINE(phy_post_select);
DWC2_QUIRK_FUNC_DEFINE(is_phy_clk_off);

/* TODO: search in case of present helper function like this */
static uint16_t calc_packet_count(const uint16_t size, const uint8_t mps)
{
	if (size == 0) {
		return 1; /* in Buffer DMA mode Zero Length Packet still counts as 1 packet */
	} else {
		return DIV_ROUND_UP(size, mps);
	}
}

/*
 * Hardware Abstraction Layer
 *
 * Snippets of code repeated across the driver to set multiple registers.
 * Only low-level register I/O is done, using (struct usb_dwc2_reg) and other
 * parameters but never dev or dev->data/config.
 */

static uint32_t uhc_dwc2_hal_get_speed(struct usb_dwc2_reg *const dwc2)
{
	uint32_t hprt = sys_read32((mem_addr_t)&dwc2->hprt);

	return usb_dwc2_get_hprt_prtspd(hprt);
}

static void uhc_dwc2_hal_init_hfir(struct usb_dwc2_reg *const dwc2)
{
	uint32_t hfir = sys_read32((mem_addr_t)&dwc2->hfir);

	/* Disable dynamic loading */
	hfir &= ~USB_DWC2_HFIR_HFIRRLDCTRL;

	/* Set frame interval to be equal to 1ms
	 * Note: FSLS PHY has an implicit 8 divider applied when in LS mode,
	 * so the values of FSLSPclkSel and FrInt have to be adjusted accordingly.
	 */
	hfir &= ~USB_DWC2_HFIR_FRINT_MASK;
	if (uhc_dwc2_hal_get_speed(dwc2) == USB_DWC2_HPRT_PRTSPD_FULL) {
		hfir |= 48000 << USB_DWC2_HFIR_FRINT_POS;
	} else {
		hfir |= 6000 << USB_DWC2_HFIR_FRINT_POS;
	}

	sys_write32(hfir, (mem_addr_t)&dwc2->hfir);
}

static void uhc_dwc2_hal_init_hcfg(struct usb_dwc2_reg *const dwc2)
{
	uint32_t hcfg = sys_read32((mem_addr_t)&dwc2->hcfg);

	/* We can select Buffer DMA of Scatter-Gather DMA mode here: Buffer DMA for now */
	hcfg &= ~USB_DWC2_HCFG_DESCDMA;

	/* Disable periodic scheduling, will enable later */
	hcfg &= ~USB_DWC2_HCFG_PERSCHEDENA;

	/* Configure the PHY clock speed depending on max supported DWC2 speed */
	hcfg &= ~USB_DWC2_HCFG_FSLSPCLKSEL_MASK;
	if (IS_ENABLED(CONFIG_UDC_DRIVER_HIGH_SPEED_SUPPORT_ENABLED)) {
		hcfg |= USB_DWC2_HCFG_FSLSPCLKSEL_CLK3060 << USB_DWC2_HCFG_FSLSPCLKSEL_POS;
	} else {
		hcfg |= USB_DWC2_HCFG_FSLSPCLKSEL_CLK48 << USB_DWC2_HCFG_FSLSPCLKSEL_POS;
	}

	sys_write32(hcfg, (mem_addr_t)&dwc2->hcfg);
}

static void uhc_dwc2_hal_init_gusbcfg(struct usb_dwc2_reg *const dwc2)
{
	uint32_t gusbcfg = sys_read32((mem_addr_t)&dwc2->gusbcfg);
	uint32_t ghwcfg2 = sys_read32((mem_addr_t)&dwc2->ghwcfg2);
	uint32_t ghwcfg4 = sys_read32((mem_addr_t)&dwc2->ghwcfg4);

	/* Enable Host mode */
	sys_set_bits((mem_addr_t)&dwc2->gusbcfg, USB_DWC2_GUSBCFG_FORCEHSTMODE);
	/* Wait until core is in host mode */
	while ((sys_read32((mem_addr_t)&dwc2->gintsts) & USB_DWC2_GINTSTS_CURMOD) != 1) {
	}

	if (usb_dwc2_get_ghwcfg2_hsphytype(ghwcfg2) != 0) {
		/* De-select FS PHY */
		gusbcfg &= ~USB_DWC2_GUSBCFG_PHYSEL_USB11;

		if (usb_dwc2_get_ghwcfg2_hsphytype(ghwcfg2) == USB_DWC2_GHWCFG2_HSPHYTYPE_ULPI) {
			LOG_DBG("Highspeed ULPI PHY init");
			/* Select ULPI PHY (external) */
			gusbcfg |= USB_DWC2_GUSBCFG_ULPI_UTMI_SEL_ULPI;
			/* ULPI is always 8-bit interface */
			gusbcfg &= ~USB_DWC2_GUSBCFG_PHYIF_16_BIT;
			/* ULPI select single data rate */
			gusbcfg &= ~USB_DWC2_GUSBCFG_DDR_DOUBLE;
			/* Default internal VBUS Indicator and Drive */
			gusbcfg &= ~(USB_DWC2_GUSBCFG_ULPIEVBUSD | USB_DWC2_GUSBCFG_ULPIEVBUSI);
			/* Disable FS/LS ULPI and Suspend mode */
			gusbcfg &= ~(USB_DWC2_GUSBCFG_ULPIFSLS | USB_DWC2_GUSBCFG_ULPICLK_SUSM);
		} else {
			LOG_DBG("Highspeed UTMI+ PHY init");
			/* Select UTMI+ PHY (internal) */
			gusbcfg &= ~USB_DWC2_GUSBCFG_ULPI_UTMI_SEL_ULPI;
			/* Set 16-bit interface if supported */
			if (usb_dwc2_get_ghwcfg4_phydatawidth(ghwcfg4)) {
				gusbcfg |= USB_DWC2_GUSBCFG_PHYIF_16_BIT;
			} else {
				gusbcfg &= ~USB_DWC2_GUSBCFG_PHYIF_16_BIT;
			}
		}
		sys_write32(gusbcfg, (mem_addr_t)&dwc2->gusbcfg);
	} else {
		LOG_DBG("Fullspeed PHY init");
		sys_set_bits((mem_addr_t)&dwc2->gusbcfg, USB_DWC2_GUSBCFG_PHYSEL_USB11);
	}
}
static int uhc_dwc2_hal_init_gahbcfg(struct usb_dwc2_reg *const dwc2)

{
	uint32_t ghwcfg2 = sys_read32((mem_addr_t)&dwc2->ghwcfg2);
	uint32_t gahbcfg;

	/* Disable Global Interrupt */
	sys_clear_bits((mem_addr_t)&dwc2->gahbcfg, USB_DWC2_GAHBCFG_GLBINTRMASK);

	/* TODO: Set AHB burst mode for some ECO only for some devices */
	/* TODO: Disable HNP and SRP capabilities */

	/* Configure AHB */
	gahbcfg = USB_DWC2_GAHBCFG_NPTXFEMPLVL |
		  usb_dwc2_set_gahbcfg_hbstlen(USB_DWC2_GAHBCFG_HBSTLEN_INCR16);
	sys_write32(gahbcfg, (mem_addr_t)&dwc2->gahbcfg);

	if (usb_dwc2_get_ghwcfg2_otgarch(ghwcfg2) == USB_DWC2_GHWCFG2_OTGARCH_INTERNALDMA) {
		sys_set_bits((mem_addr_t)&dwc2->gahbcfg, USB_DWC2_GAHBCFG_DMAEN);
	} else {
		/* TODO: Implement Slave mode */
		LOG_WRN("DMA is not supported and Slave Mode is not implemented");
		return -ENXIO;
	}

	/* Enable Global Interrupt */
	sys_set_bits((mem_addr_t)&dwc2->gahbcfg, USB_DWC2_GAHBCFG_GLBINTRMASK);

	return 0;
}

static int uhc_dwc2_hal_core_reset(struct usb_dwc2_reg *const dwc2, const k_timeout_t timeout)
{
	const k_timepoint_t timepoint = sys_timepoint_calc(timeout);

	/* Check AHB master idle state */
	while ((sys_read32((mem_addr_t)&dwc2->grstctl) & USB_DWC2_GRSTCTL_AHBIDLE) == 0) {
		if (sys_timepoint_expired(timepoint)) {
			LOG_ERR("Wait for AHB idle timeout, GRSTCTL 0x%08X",
				sys_read32((mem_addr_t)&dwc2->grstctl));
			return -EIO;
		}

		k_busy_wait(1);
	}

	/* Apply Core Soft Reset */
	sys_write32(USB_DWC2_GRSTCTL_CSFTRST, (mem_addr_t)&dwc2->grstctl);

	/* Wait for reset to complete */
	while ((sys_read32((mem_addr_t)&dwc2->grstctl) & USB_DWC2_GRSTCTL_CSFTRST) != 0 &&
	       (sys_read32((mem_addr_t)&dwc2->grstctl) & USB_DWC2_GRSTCTL_CSFTRSTDONE) == 0) {
		if (sys_timepoint_expired(timepoint)) {
			LOG_ERR("Wait for CSR done timeout, GRSTCTL 0x%08X",
				sys_read32((mem_addr_t)&dwc2->grstctl));
			return -EIO;
		}

		k_busy_wait(1);
	}

	/* CSFTRSTDONE is W1C so the write must have the bit set to clear it */
	sys_clear_bits((mem_addr_t)&dwc2->grstctl, USB_DWC2_GRSTCTL_CSFTRST);

	LOG_DBG("DWC2 core reset done");

	return 0;
}

static void uhc_dwc2_hal_port_set_power(struct usb_dwc2_reg *const dwc2, const bool power)
{
	uint32_t hprt;

	hprt = sys_read32((mem_addr_t)&dwc2->hprt);
	if (power) {
		hprt |= USB_DWC2_HPRT_PRTPWR;
	} else {
		hprt &= ~USB_DWC2_HPRT_PRTPWR;
	}
	sys_write32(hprt & (~USB_DWC2_HPRT_W1C_MSK), (mem_addr_t)&dwc2->hprt);
}

static void uhc_dwc2_hal_port_set_bus_reset(struct usb_dwc2_reg *const dwc2, const bool bus_reset)
{
	uint32_t hprt;

	hprt = sys_read32((mem_addr_t)&dwc2->hprt);
	if (bus_reset) {
		hprt |= USB_DWC2_HPRT_PRTRST;
	} else {
		hprt &= ~USB_DWC2_HPRT_PRTRST;
	}
	sys_write32(hprt & (~USB_DWC2_HPRT_W1C_MSK), (mem_addr_t)&dwc2->hprt);
}

static void uhc_dwc2_hal_set_fifo_sizes(struct usb_dwc2_reg *const dwc2, uint32_t ghwcfg2, uint32_t ghwcfg3)
{
	const uint32_t nptx_largest = EPSIZE_BULK_FS / 4;
	const uint32_t ptx_largest = 256 / 4;
	const uint32_t dfifodepth = FIELD_GET(USB_DWC2_GHWCFG3_DFIFODEPTH_MASK, ghwcfg3);
	const uint32_t numhstchnl = FIELD_GET(USB_DWC2_GHWCFG2_NUMHSTCHNL_MASK, ghwcfg2);
	uint32_t fifo_available = dfifodepth - (numhstchnl + 1);
	/* TODO: support HS */
	const uint16_t fifo_nptxfdep = 2 * nptx_largest;
	const uint16_t fifo_rxfsiz = 2 * (ptx_largest + 2) + numhstchnl + 1;
	const uint16_t fifo_ptxfsiz = fifo_available - (fifo_nptxfdep + fifo_rxfsiz);
	uint32_t gdfifocfg;
	uint32_t grxfsiz;
	uint32_t gnptxfsiz;
	uint32_t hptxfsiz;
	uint32_t grstctl;

	LOG_DBG("Setting FIFO sizes: top=%u, nptx=%u, rx=%u, ptx=%u",
		fifo_available * 4, fifo_nptxfdep * 4, fifo_rxfsiz * 4, fifo_ptxfsiz * 4);

	/* FIFO config */
	gdfifocfg = usb_dwc2_set_gdfifocfg_epinfobaseaddr(fifo_available) |
		    usb_dwc2_set_gdfifocfg_gdfifocfg(fifo_available);
	sys_write32(gdfifocfg, (mem_addr_t)&dwc2->gdfifocfg);

	/* RX FIFO size */
	fifo_available -= fifo_rxfsiz;
	grxfsiz = usb_dwc2_set_grxfsiz(fifo_rxfsiz);
	sys_write32(grxfsiz, (mem_addr_t)&dwc2->grxfsiz);

	/* Non-periodic TX FIFO size */
	fifo_available -= fifo_nptxfdep;
	gnptxfsiz = usb_dwc2_set_gnptxfsiz_nptxfdep(fifo_nptxfdep);
	gnptxfsiz |= usb_dwc2_set_gnptxfsiz_nptxfstaddr(fifo_available);
	sys_write32(gnptxfsiz, (mem_addr_t)&dwc2->gnptxfsiz);

	/* Periodic TX FIFO size */
	fifo_available -= fifo_ptxfsiz;
	hptxfsiz = usb_dwc2_set_hptxfsiz_ptxfsize(fifo_ptxfsiz);
	hptxfsiz |= fifo_available;
	sys_write32(hptxfsiz, (mem_addr_t)&dwc2->hptxfsiz);

	/* Flush TX FIFO and set number of TX FIFO */
	grstctl = usb_dwc2_set_grstctl_txfnum(CONFIG_UHC_DWC2_MAX_CHANNELS) |
		  USB_DWC2_GRSTCTL_TXFFLSH;
	sys_write32(grstctl, (mem_addr_t)&dwc2->grstctl);
	while (sys_read32((mem_addr_t)&dwc2->grstctl) & USB_DWC2_GRSTCTL_TXFFLSH) {
		continue;
	}

	/* Flush RX FIFO */
	sys_write32(USB_DWC2_GRSTCTL_RXFFLSH, (mem_addr_t)&dwc2->grstctl);
	while (sys_read32((mem_addr_t)&dwc2->grstctl) & USB_DWC2_GRSTCTL_RXFFLSH) {
		continue;
	}
}

static int uhc_dwc2_hal_init_host(struct usb_dwc2_reg *const dwc2)
{
	int ret;

	/* Init GUSBCFG */
	uhc_dwc2_hal_init_gusbcfg(dwc2);

	/* Init GAHBCFG */
	ret = uhc_dwc2_hal_init_gahbcfg(dwc2);

	if (ret != 0) {
		/* TODO: Implement Slave Mode */
		LOG_WRN("DMA isn't supported, but Slave Mode isn't implemented yet");
		return ret;
	}

	/* Clear interrupts */
	sys_clear_bits((mem_addr_t)&dwc2->gintmsk, 0xFFFFFFFFUL);
	sys_set_bits((mem_addr_t)&dwc2->gintmsk, USB_DWC2_GINTSTS_DISCONNINT);

	/* Clear status */
	uint32_t core_intrs = sys_read32((mem_addr_t)&dwc2->gintsts);

	sys_write32(core_intrs, (mem_addr_t)&dwc2->gintsts);

	return ret;
}

/*
 * Port
 *
 * Event handling and debounce logic for DWC2 port
 */

static void uhc_dwc2_port_debounce_lock(const struct device *const dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);

	/* Disable connection and disconnection interrupts to prevent spurious events */
	sys_clear_bits((mem_addr_t)&dwc2->gintmsk, USB_DWC2_GINTSTS_PRTINT |
						   USB_DWC2_GINTSTS_DISCONNINT);
	/* Set the debounce lock flag */
	priv->debouncing = 1;
}

static void uhc_dwc2_port_debounce_unlock(const struct device *const dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);

	/* Clear the flag */
	priv->debouncing = 0;

	/* Clear Connection and disconnection interrupt in case it triggered again */
	sys_set_bits((mem_addr_t)&dwc2->gintsts, USB_DWC2_GINTSTS_DISCONNINT);

	/* Clear the PRTCONNDET interrupt by writing 1 to the corresponding bit (W1C logic) */
	sys_set_bits((mem_addr_t)&dwc2->hprt, USB_DWC2_HPRT_PRTCONNDET);

	/* Re-enable the HPRT (connection) and disconnection interrupts */
	sys_set_bits((mem_addr_t)&dwc2->gintmsk, USB_DWC2_GINTSTS_PRTINT |
						 USB_DWC2_GINTSTS_DISCONNINT);
}

static bool uhc_dwc2_channel_xfer_is_done(struct uhc_dwc2_channel *channel)
{
	/* Only control transfers need to be handled in stages */
	if (channel->type != UHC_DWC2_XFER_TYPE_CTRL) {
		return true;
	}

	/* Is done when we finished the UHC_CONTROL_STAGE_STATUS */
	return (channel->ctrl_stg == UHC_CONTROL_STAGE_STATUS);
}

static void uhc_dwc2_channel_process_ctrl(struct uhc_dwc2_channel *channel)
{
	struct uhc_transfer *const xfer = channel->xfer;
	bool next_dir_is_in;
	enum uhc_dwc2_channel_pid next_pid;
	uint16_t size = 0;
	uint8_t *dma_addr = NULL;
	uint32_t hctsiz;

	if (channel->ctrl_stg == UHC_CONTROL_STAGE_SETUP) {
		/* Just finished UHC_CONTROL_STAGE_SETUP */
		if (channel->data_stg_skip) {
			/* No data stage. Go strait to status */
			next_dir_is_in = true;
			next_pid = UHC_DWC2_PID_DATA1;
			channel->ctrl_stg = UHC_CONTROL_STAGE_STATUS;
		} else {
			/* Data stage is present, go to data stage */
			next_dir_is_in = channel->data_stg_in;
			next_pid = UHC_DWC2_PID_DATA1;
			channel->ctrl_stg = UHC_CONTROL_STAGE_DATA;

			/*
			 * NOTE:
			 * For OUT - number of bytes host sends to device
			 * For IN - number of bytes host reserves to receive
			 */
			if (!next_dir_is_in) {
				size = xfer->buf->len;
			} else {
				/* TODO: Check the buffer is large enough for the next transfer? */
				size = net_buf_tailroom(xfer->buf);
			}

			/* TODO: Toggle PID? */

			if (xfer->buf != NULL) {
				/* Get the tail of the buffer to append data */
				dma_addr = net_buf_tail(xfer->buf);
			}
		}
	} else {
		/* Just finished UHC_CONTROL_STAGE_DATA */
		hctsiz = sys_read32((mem_addr_t)&channel->base->hctsiz);
		/* Actual is requested minus remaining */
		size_t actual = xfer->buf->size - usb_dwc2_get_hctsiz_xfersize(hctsiz);
		/* Increase the net_buf for the actual transferred len */
		net_buf_add(xfer->buf, actual);
		/* Status stage is always the opposite direction of data stage */
		next_dir_is_in = !channel->data_stg_in;
		/* Status stage always has a PID of DATA1 */
		next_pid = UHC_DWC2_PID_DATA1;
		channel->ctrl_stg = UHC_CONTROL_STAGE_STATUS;
	}

	/* Calculate new packet count */
	const uint16_t pkt_cnt = calc_packet_count(size, channel->ep_mps);

	if (next_dir_is_in) {
		sys_set_bits((mem_addr_t)&channel->base->hcchar, USB_DWC2_HCCHAR_EPDIR);
	} else {
		sys_clear_bits((mem_addr_t)&channel->base->hcchar, USB_DWC2_HCCHAR_EPDIR);
	}

	hctsiz = usb_dwc2_set_hctsiz_pid(next_pid) |
		usb_dwc2_set_hctsiz_pktcnt(pkt_cnt) |
		usb_dwc2_set_hctsiz_xfersize(size);

	sys_write32(hctsiz, (mem_addr_t)&channel->base->hctsiz);
	sys_write32((uint32_t)dma_addr, (mem_addr_t)&channel->base->hcdma);

	/* TODO: Configure split transaction if needed */

	/* TODO: sync CACHE */

	uint32_t hcchar = sys_read32((mem_addr_t)&channel->base->hcchar);

	hcchar |= USB_DWC2_HCCHAR_CHENA;
	hcchar &= ~USB_DWC2_HCCHAR_CHDIS;
	sys_write32(hcchar, (mem_addr_t)&channel->base->hcchar);
}

static uint32_t uhc_dwc2_channel_irq_handle_events(struct uhc_dwc2_channel *channel)
{
	uint32_t hcint = sys_read32((mem_addr_t)&channel->base->hcint);
	uint32_t channel_events = 0;

	/* Clear the interrupt bits by writing them back */
	sys_write32(hcint, (mem_addr_t)&channel->base->hcint);

	/* TODO: Read hcchar and split paths for EP IN and OUT? */

	/* TODO: Read hcchar and split paths for BULK/CTRL/ISOC/INTR? */

	if (hcint & USB_DWC2_HCINT_CHHLTD) {
		if (hcint & USB_DWC2_HCINT_XFERCOMPL) {
			/* TODO: channel error count = 0 */

			if (!uhc_dwc2_channel_xfer_is_done(channel)) {
				/* Control transfer isn't finished yet - continue in ISR */
				uhc_dwc2_channel_process_ctrl(channel);
			} else {
				/* Transfer finished, notify thread */
				channel_events |= BIT(UHC_DWC2_CHANNEL_DO_RELEASE);
				channel_events |= BIT(UHC_DWC2_CHANNEL_EVENT_CPLT);
			}
		} else if (hcint & USB_DWC2_HCINT_STALL) {
			/* TODO: channel error count = 0 */

			/* Expecting ACK interrupt next */
			sys_set_bits((mem_addr_t)&channel->base->hcintmsk, USB_DWC2_HCINT_ACK);

			/* Notify thread */
			channel_events |= BIT(UHC_DWC2_CHANNEL_DO_RELEASE);
			channel_events |= BIT(UHC_DWC2_CHANNEL_EVENT_STALL);
		}
	} else if (hcint & USB_DWC2_HCINT_ACK) {
		/* TODO: channel error count = 1 */

		/* Not expecting ACK interrupt anymore */
		sys_clear_bits((mem_addr_t)&channel->base->hcintmsk, USB_DWC2_HCINT_ACK);
	} else {
		LOG_WRN("Channel has not been handled: HCINT=0x%08x", hcint);
	}

	return channel_events;
}

static bool uhc_dwc2_port_debounce(const struct device *const dev,
				   enum uhc_dwc2_event event)
{
	const struct uhc_dwc2_config *config = dev->config;
	struct usb_dwc2_reg *dwc2 = config->base;
	bool connected;
	bool want_connected;

	/**
	 * HINT: Do the debounce delay outside of the global lock
	 */
	uhc_unlock_internal(dev);

	k_msleep(CONFIG_UHC_DWC2_DEBOUNCE_DELAY_MS);

	uhc_lock_internal(dev, K_FOREVER);

	connected = ((sys_read32((mem_addr_t)&dwc2->hprt) & USB_DWC2_HPRT_PRTCONNSTS) != 0);
	want_connected = (event == UHC_DWC2_EVENT_PORT_CONNECTION);

	uhc_dwc2_port_debounce_unlock(dev);

	/* True if stable state matches the event */
	return connected == want_connected;
}

static void uhc_dwc2_port_handle_events(const struct device *const dev, uint32_t event_mask)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	const uint32_t speed = uhc_dwc2_hal_get_speed(dwc2);

	LOG_DBG("Port events: 0x%08X", event_mask);

	if (event_mask & BIT(UHC_DWC2_EVENT_PORT_CONNECTION)) {
		/* Port connected */
		LOG_DBG("Port connected");

		/* Debounce port connection */
		if (uhc_dwc2_port_debounce(dev, UHC_DWC2_EVENT_PORT_CONNECTION)) {
			/* Notify the higher logic about the new device */
			switch (speed) {
			case USB_DWC2_HPRT_PRTSPD_LOW:
				LOG_INF("New %s-speed device", "low");
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_LS, 0);
				break;
			case USB_DWC2_HPRT_PRTSPD_FULL:
				LOG_INF("New %s-speed device", "full");
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_FS, 0);
				break;
			case USB_DWC2_HPRT_PRTSPD_HIGH:
				LOG_INF("New %s-speed device", "high");
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_HS, 0);
				break;
			default:
				LOG_ERR("New device with unsupported speed %d", speed);
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_LS, -ENOTSUP);
				break;
			}

			/* Program the Host Frame Interval Register according to speed */
			uhc_dwc2_hal_init_hfir(dwc2);

			/* Mark that port has a device */
			priv->has_device = 1;
		} else {
			/* TODO: Implement handling */
			LOG_ERR("Port changed during debouncing connect");
		}

		/* Reset of the initialization in uhc_dwc2_bus_reset() */
	}

	if (event_mask & BIT(UHC_DWC2_EVENT_PORT_DISCONNECTION)) {
		/* Port disconnected */
		/* Debounce port disconnection */
		if (uhc_dwc2_port_debounce(dev, UHC_DWC2_EVENT_PORT_DISCONNECTION)) {
			LOG_DBG("Port disconnected");

			/* If port has a device - notify upper layer */
			if (priv->has_device) {
				uhc_submit_event(dev, UHC_EVT_DEV_REMOVED, 0);
				/* Unmark that port has a device */
				priv->has_device = 0;
			}
			/* Reset the controller to handle new connection */
			uhc_dwc2_soft_reset(dev);
		} else {
			/* TODO: Implement handling */
			LOG_ERR("Port changed during debouncing disconnect");
		}
	}

	if (event_mask & BIT(UHC_DWC2_EVENT_PORT_ERROR)) {
		LOG_DBG("Port error");

		if (priv->has_device) {
			uhc_submit_event(dev, UHC_EVT_DEV_REMOVED, 0);
			/* Unmark that port has a device */
			priv->has_device = 0;
		}

		/* TODO: recover from the error */

		/* Reset the controller to handle new connection */
		uhc_dwc2_soft_reset(dev);
	}

	if (event_mask & BIT(UHC_DWC2_EVENT_PORT_OVERCURRENT)) {
		LOG_ERR("Port overcurrent");
		/* TODO: Handle overcurrent */
		LOG_WRN("Handle overcurrent is not implemented");
		/* Just power off the port via registers */
	}
}

static void uhc_dwc2_port_power_on(const struct device *const dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;

	sys_clear_bits((mem_addr_t)&dwc2->haintmsk, 0xFFFFFFFFUL);
	sys_set_bits((mem_addr_t)&dwc2->gintmsk, USB_DWC2_GINTSTS_PRTINT | USB_DWC2_GINTSTS_HCHINT);
	uhc_dwc2_hal_port_set_power(dwc2, true);
}

static void uhc_dwc2_port_fifo_precalc_dma(const struct device *const dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);

	const uint32_t nptx_largest = EPSIZE_BULK_FS / 4;
	const uint32_t ptx_largest = 256 / 4;

	LOG_DBG("Init FIFO sizes");

	/* TODO: support HS */

	priv->fifo_top = UHC_DWC2_FIFODEPTH(config) - (UHC_DWC2_NUMHSTCHNL(config) + 1);
	priv->fifo_nptxfsiz = 2 * nptx_largest;
	priv->fifo_rxfsiz = 2 * (ptx_largest + 2) + UHC_DWC2_NUMHSTCHNL(config) + 1;
	priv->fifo_ptxfsiz = priv->fifo_top - (priv->fifo_nptxfsiz + priv->fifo_rxfsiz);

	/* TODO: verify ptxfsiz is overflowed */

	LOG_DBG("FIFO sizes: top=%u, nptx=%u, rx=%u, ptx=%u", priv->fifo_top * 4,
		priv->fifo_nptxfsiz * 4, priv->fifo_rxfsiz * 4, priv->fifo_ptxfsiz * 4);
}

/*
 * Channel
 *
 * Event decoding and channel management
 */

static int uhc_dwc2_channel_claim(const struct device *const dev,
				  struct uhc_transfer *const xfer,
				  struct uhc_dwc2_channel **channel_hdl)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;
	const struct usb_device *udev = xfer->udev;

	/* TODO: select non-claimed channel, use channel 0 for now */
	uint8_t idx = 0;

	struct uhc_dwc2_channel *channel = &priv->channel[idx];

	switch (xfer->type) {
	case USB_EP_TYPE_CONTROL:
		channel->type = UHC_DWC2_XFER_TYPE_CTRL;
		break;
	case USB_EP_TYPE_BULK:
		channel->type = UHC_DWC2_XFER_TYPE_BULK;
		break;
	case USB_EP_TYPE_INTERRUPT:
		channel->type = UHC_DWC2_XFER_TYPE_INTR;
		break;
	case USB_EP_TYPE_ISO:
		channel->type = UHC_DWC2_XFER_TYPE_ISOCHRONOUS;
		break;
	default:
		return -EINVAL;
	}

	/* Save channel characteristics underlying channel */
	channel->xfer = xfer;
	channel->ep_addr = xfer->ep;
	channel->dev_addr = udev->addr;
	channel->ep_mps = xfer->mps;

	/* Claim channel */
	channel->claimed = 1;

	/* Init underlying channel registers */

	/* Clear the interrupt bits by writing them back */
	uint32_t hcint = sys_read32((mem_addr_t)&channel->base->hcint);

	sys_write32(hcint, (mem_addr_t)&channel->base->hcint);

	/* Enable channel interrupt in the core */
	sys_set_bits((mem_addr_t)&dwc2->haintmsk, (1 << idx));

	/* Enable transfer complete and channel halted interrupts */
	sys_set_bits((mem_addr_t)&channel->base->hcintmsk, USB_DWC2_HCINT_XFERCOMPL |
							USB_DWC2_HCINT_CHHLTD);

	uint32_t hcchar = usb_dwc2_set_hcchar_mps(channel->ep_mps) |
			usb_dwc2_set_hcchar_epnum(USB_EP_GET_IDX(channel->ep_addr)) |
			usb_dwc2_set_hcchar_eptype(channel->type) |
			usb_dwc2_set_hcchar_ec(1UL /* TODO: ep_config->mult */) |
			usb_dwc2_set_hcchar_devaddr(channel->dev_addr);

	if (USB_EP_DIR_IS_IN(channel->ep_addr)) {
		hcchar |= USB_DWC2_HCCHAR_EPDIR;
	}

	if (false /* TODO: Support Hubs channel->ls_via_fs_hub */) {
		hcchar |= USB_DWC2_HCCHAR_LSPDDEV;
	}

	if (channel->type == UHC_DWC2_XFER_TYPE_INTR) {
		hcchar |= USB_DWC2_HCCHAR_ODDFRM;
	}

	sys_write32(hcchar, (mem_addr_t)&channel->base->hcchar);

	LOG_DBG("Claimed channel%d", idx);

	*channel_hdl = channel;
	return 0;
}

static int uhc_dwc2_channel_release(const struct device *const dev,
				struct uhc_dwc2_channel *channel)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;

	sys_clear_bits((mem_addr_t)&dwc2->haintmsk, (1 << channel->index));

	/* Release channel */
	channel->xfer = NULL;
	channel->claimed = 0;

	LOG_DBG("Released channel%d", channel->index);

	return 0;
}

static int uhc_dwc2_channel_start_transfer_ctrl(struct uhc_dwc2_channel *channel)
{
	/* Get information about the control transfer by analyzing the setup packet */
	struct uhc_transfer *const xfer = channel->xfer;
	const struct usb_setup_packet *setup_pkt = (const struct usb_setup_packet *)xfer->setup_pkt;

	LOG_HEXDUMP_DBG(xfer->setup_pkt, 8, "setup");

	channel->ctrl_stg = UHC_CONTROL_STAGE_SETUP;
	channel->data_stg_in = usb_reqtype_is_to_host(setup_pkt);
	channel->data_stg_skip = (setup_pkt->wLength == 0);
	channel->set_address = (setup_pkt->bRequest == USB_SREQ_SET_ADDRESS) ? 1 : 0;

	LOG_DBG("data_in: %s, data_skip: %s",
			channel->data_stg_in ? "true" : "false",
			channel->data_stg_skip ? "true" : "false");

	if (USB_EP_GET_IDX(xfer->ep) == 0) {
		/* Control stage is always OUT */
		sys_clear_bits((mem_addr_t)&channel->base->hcchar, USB_DWC2_HCCHAR_EPDIR);
	}

	if (xfer->interval != 0) {
		LOG_WRN("Periodic transfer is not supported");
	}

	const uint16_t pkt_cnt = calc_packet_count(sizeof(struct usb_setup_packet),
						channel->ep_mps);

	uint32_t hctsiz = usb_dwc2_set_hctsiz_pid(UHC_DWC2_PID_MDATA_SETUP) |
		usb_dwc2_set_hctsiz_pktcnt(pkt_cnt) |
		usb_dwc2_set_hctsiz_xfersize(sizeof(struct usb_setup_packet));

	sys_write32(hctsiz, (mem_addr_t)&channel->base->hctsiz);
	sys_write32((uint32_t)xfer->setup_pkt, (mem_addr_t)&channel->base->hcdma);

	/* TODO: Configure split transaction if needed */

	uint32_t hcint = sys_read32((mem_addr_t)&channel->base->hcint);

	sys_write32(hcint, (mem_addr_t)&channel->base->hcint);

	/* TODO: sync CACHE */

	uint32_t hcchar = sys_read32((mem_addr_t)&channel->base->hcchar);

	hcchar |= USB_DWC2_HCCHAR_CHENA;
	hcchar &= ~USB_DWC2_HCCHAR_CHDIS;
	sys_write32(hcchar, (mem_addr_t)&channel->base->hcchar);

	channel->executing = 1;

	return 0;
}

struct uhc_dwc2_channel *uhc_dwc2_channel_get_pending(const struct device *const dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	int chan_num = __builtin_ffs(priv->pending_channels_msk);

	if (chan_num) {
		/* Clear the pending bit for that channel */
		priv->pending_channels_msk &= ~(1 << (chan_num - 1));
		return &priv->channel[chan_num - 1];
	} else {
		return NULL;
	}
}

/*
 * Transfers
 *
 * Channels are allocated and assigned a transfer until it completes.
 */

static int uhc_dwc2_submit_xfer(const struct device *const dev, struct uhc_transfer *const xfer)
{
	struct uhc_dwc2_channel *channel = NULL;
	int ret;

	LOG_DBG("ep=%02Xh, mps=%d, int=%d, start_frame=%d, stage=%d, no_status=%d",
		xfer->ep, xfer->mps, xfer->interval,
		xfer->start_frame, xfer->stage, xfer->no_status);

	/* TODO: dma requirement, setup packet must be aligned 4 bytes */
	if (((uintptr_t)xfer->setup_pkt % 4)) {
		LOG_WRN("Setup packet address %p is not 4-byte aligned",
					xfer->setup_pkt);
	}

	/* TODO: dma requirement, buffer addr that will used as dma addr also should be aligned */
	if ((xfer->buf != NULL) && ((uintptr_t)net_buf_tail(xfer->buf) % 4)) {
		LOG_WRN("Buffer address %08lXh is not 4-byte aligned",
					(uintptr_t)net_buf_tail(xfer->buf));
	}

	ret = uhc_dwc2_channel_claim(dev, xfer, &channel);
	if (ret != 0) {
		LOG_ERR("Failed to claim channel: %d", ret);
		return ret;
	}

	switch (xfer->type) {
	case USB_EP_TYPE_CONTROL:
		return uhc_dwc2_channel_start_transfer_ctrl(channel);
	case USB_EP_TYPE_BULK:
	case USB_EP_TYPE_INTERRUPT:
	case USB_EP_TYPE_ISO:
	default:
		LOG_WRN("Channel type isn't supported yet");
		uhc_dwc2_channel_release(dev, channel);
		return -EINVAL;
	}

	return ret;
}

static void uhc_dwc2_channel_handle_events(const struct device *const dev,
						struct uhc_dwc2_channel *channel)
{
	uint32_t events = (uint32_t)atomic_set(&channel->events, 0);
	struct uhc_transfer *const xfer = channel->xfer;
	int err;

	LOG_DBG("Thread channel%d events: 0x%08x", channel->index, events);

	channel->executing = 0;

	if (events & BIT(UHC_DWC2_CHANNEL_EVENT_CPLT)) {
		/**
		 * HINT: When device is processing SetAddress(), delay should be applied
		 */
		if (channel->set_address) {
			k_msleep(CONFIG_UHC_DWC2_SET_ADDR_DELAY_MS);
		}
		/* No error */
		err = 0;
	}

	if (events & BIT(UHC_DWC2_CHANNEL_EVENT_STALL)) {
		/* Request STALLed */
		err = -EPIPE;
	}

	if (events & BIT(UHC_DWC2_CHANNEL_EVENT_ERROR)) {
		LOG_ERR("Channel%d error", channel->index);
		/* TODO: Channel has an error */
		LOG_WRN("Channel error handing has not been implemented yet");
	}

	if (events & BIT(UHC_DWC2_CHANNEL_DO_RELEASE)) {
		uhc_dwc2_channel_release(dev, channel);
	}

	/* Notify the upper logic */
	uhc_xfer_return(dev, xfer, err);
}

/*
 * Events
 *
 * Event decoding, ISR handler, and event loop thread.
 */

static enum uhc_dwc2_event uhc_dwc2_port_get_event(const struct device *const dev)
{
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;

	/* Read and clear core interrupt status */
	uint32_t core_intrs = sys_read32((mem_addr_t)&dwc2->gintsts);

	sys_write32(core_intrs, (mem_addr_t)&dwc2->gintsts);

	uint32_t port_intrs = 0;

	if (core_intrs & USB_DWC2_GINTSTS_PRTINT) {
		/* Port interrupt occurred -> read and clear selected HPRT W1C bits */
		port_intrs = sys_read32((mem_addr_t)&dwc2->hprt);
		/* W1C changed bits except the PRTENA */
		sys_write32(port_intrs & ~USB_DWC2_HPRT_PRTENA, (mem_addr_t)&dwc2->hprt);
	}

	LOG_DBG("GINTSTS=0x%08x, HPRT=0x%08x", core_intrs, port_intrs);

	/* Handle disconnect first */
	if (core_intrs & USB_DWC2_GINTSTS_DISCONNINT) {
		/* Port disconnected */
		uhc_dwc2_port_debounce_lock(dev);
		return UHC_DWC2_EVENT_PORT_DISCONNECTION;
	}

	/* To have better throughput, handle channels right after disconnection */
	if (core_intrs & USB_DWC2_GINTSTS_HCHINT) {
		priv->pending_channels_msk = sys_read32((mem_addr_t)&dwc2->haint);
		return UHC_DWC2_EVENT_PORT_PEND_CHANNEL;
	}

	/* Handle port overcurrent as it is a failure state */
	if (port_intrs & USB_DWC2_HPRT_PRTOVRCURRCHNG) {
		/* TODO: Overcurrent or overcurrent clear? */
		return UHC_DWC2_EVENT_PORT_OVERCURRENT;
	}

	/* Handle port change bits */
	if (port_intrs & USB_DWC2_HPRT_PRTENCHNG) {
		if (port_intrs & USB_DWC2_HPRT_PRTENA) {
			/* Host port was enabled */
			k_sem_give(&priv->sem_port_en);
			return UHC_DWC2_EVENT_NONE;
		}
		/* Host port has been disabled */
		return UHC_DWC2_EVENT_PORT_DISABLED;
	}

	/* Handle port connection */
	if (port_intrs & USB_DWC2_HPRT_PRTCONNDET && !priv->debouncing) {
		/* Port connected */
		uhc_dwc2_port_debounce_lock(dev);
		return UHC_DWC2_EVENT_PORT_CONNECTION;
	}

	return UHC_DWC2_EVENT_NONE;
}

static void uhc_dwc2_isr_handler(const struct device *const dev)
{
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct uhc_dwc2_channel *channel = NULL;
	enum uhc_dwc2_event port_event = uhc_dwc2_port_get_event(dev);
	uint32_t channel_events = 0;

	/**
	 * TODO: port_event serialization might be simplified by
	 * obsoleting uhc_dwc2_port_get_event() but after the implementation of all channels type.
	 */

	switch (port_event) {
	case UHC_DWC2_EVENT_NONE: {
		/* Port event occurred but should not be handled in higher logic */
		break;
	}
	case UHC_DWC2_EVENT_PORT_PEND_CHANNEL: {
		/* Handle pending channel event  */
		channel = uhc_dwc2_channel_get_pending(dev);
		while (channel != NULL) {
			channel_events = uhc_dwc2_channel_irq_handle_events(channel);
			if (channel_events) {
				/**
				 * The rest channel events be handled by the thread
				 */
				atomic_or(&channel->events, channel_events);
				k_event_set(&priv->event,
					BIT(UHC_DWC2_EVENT_PORT_PEND_CHANNEL + channel->index));
			}
			channel = uhc_dwc2_channel_get_pending(dev);
		}
		break;
	}
	default:
		/* Notify thread about port event */
		k_event_set(&priv->event, BIT(port_event));
		break;
	}

	(void) uhc_dwc2_quirk_irq_clear(dev);
}

static void uhc_dwc2_thread(void *arg0, void *arg1, void *arg2)
{
	const struct device *const dev = (const struct device *)arg0;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	uint32_t event_mask;

	while (true) {
		event_mask = k_event_wait_safe(&priv->event, UINT32_MAX, false, K_FOREVER);

		uhc_lock_internal(dev, K_FOREVER);

		/* Handle port events */
		uhc_dwc2_port_handle_events(dev, event_mask);

		/* Interate channels events */
		for (uint32_t index = 0; index < CONFIG_UHC_DWC2_MAX_CHANNELS; index++) {
			if (event_mask & BIT(UHC_DWC2_EVENT_PORT_PEND_CHANNEL + index)) {
				uhc_dwc2_channel_handle_events(dev, &priv->channel[index]);
			}
		}

		uhc_unlock_internal(dev);
	}
}

/*
 * Device driver API
 */

static int uhc_dwc2_lock(const struct device *const dev)
{
	struct uhc_data *data = dev->data;

	return k_mutex_lock(&data->mutex, K_FOREVER);
}

static int uhc_dwc2_unlock(const struct device *const dev)
{
	struct uhc_data *data = dev->data;

	return k_mutex_unlock(&data->mutex);
}

static int uhc_dwc2_sof_enable(const struct device *const dev)
{
	LOG_WRN("%s has not been implemented", __func__);

	return -ENOSYS;
}

static int uhc_dwc2_bus_suspend(const struct device *const dev)
{
	LOG_WRN("%s has not been implemented", __func__);

	return -ENOSYS;
}

static int uhc_dwc2_bus_reset(const struct device *const dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct usb_dwc2_reg *const dwc2 = config->base;

	LOG_DBG("Applying a bus reset");

	/* Reset the port */
	uhc_dwc2_hal_port_set_bus_reset(dwc2, true);

	/* Hold the bus in the reset state */
	k_msleep(CONFIG_UHC_DWC2_RESET_HOLD_MS);

	/* Return the bus to the idle state. Port enabled event should occur */
	uhc_dwc2_hal_port_set_bus_reset(dwc2, false);

	/* Wait the port to become enabled again */
	k_sem_take(&priv->sem_port_en, K_FOREVER);

	/* Finish the port config for the appeared device */
	uhc_dwc2_hal_set_fifo_sizes(dwc2, config->ghwcfg2, config->ghwcfg3);
	/* TODO: set frame list for the ISOC/INTR xfer */
	/* TODO: enable periodic transfer */

	return 0;
}

static int uhc_dwc2_bus_resume(const struct device *const dev)
{
	LOG_WRN("%s has not been implemented", __func__);

	return -ENOSYS;
}

static int uhc_dwc2_enqueue(const struct device *const dev, struct uhc_transfer *const xfer)
{
	int ret;

	(void)uhc_xfer_append(dev, xfer);

	uhc_lock_internal(dev, K_FOREVER);

	ret = uhc_dwc2_submit_xfer(dev, xfer);

	uhc_unlock_internal(dev);

	return ret;
}

static int uhc_dwc2_dequeue(const struct device *const dev, struct uhc_transfer *const xfer)
{
	LOG_WRN("%s has not been implemented", __func__);

	return -ENOSYS;
}

static int uhc_dwc2_init(const struct device *const dev)
{
	struct uhc_dwc2_data *priv = uhc_get_private(dev);
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	uint32_t val;
	int ret;

	ret = uhc_dwc2_quirk_init(dev);
	if (ret != 0) {
		LOG_ERR("Quirk init failed %d", ret);
		return ret;
	}

	/* Read hardware configuration registers */

	val = sys_read32((mem_addr_t)&dwc2->gsnpsid);
	if (val != config->gsnpsid) {
		LOG_ERR("Unexpected GSNPSID 0x%08x instead of 0x%08x", val, config->gsnpsid);
		return -ENOTSUP;
	}

	val = sys_read32((mem_addr_t)&dwc2->ghwcfg1);
	if (val != config->ghwcfg1) {
		LOG_ERR("Unexpected GHWCFG1 0x%08x instead of 0x%08x", val, config->ghwcfg1);
		return -ENOTSUP;
	}

	val = sys_read32((mem_addr_t)&dwc2->ghwcfg2);
	if (val != config->ghwcfg2) {
		LOG_ERR("Unexpected GHWCFG2 0x%08x instead of 0x%08x", val, config->ghwcfg2);
		return -ENOTSUP;
	}

	val = sys_read32((mem_addr_t)&dwc2->ghwcfg3);
	if (val != config->ghwcfg3) {
		LOG_ERR("Unexpected GHWCFG3 0x%08x instead of 0x%08x", val, config->ghwcfg3);
		return -ENOTSUP;
	}

	val = sys_read32((mem_addr_t)&dwc2->ghwcfg4);
	if (val != config->ghwcfg4) {
		LOG_ERR("Unexpected GHWCFG4 0x%08x instead of 0x%08x", val, config->ghwcfg4);
		return -ENOTSUP;
	}

	/* 3. Select PHY */

	ret = uhc_dwc2_quirk_phy_pre_select(dev);
	if (ret != 0) {
		LOG_ERR("Quirk PHY pre select failed %d", ret);
		return ret;
	}

	if (uhc_dwc2_quirk_is_phy_clk_off(dev)) {
		LOG_ERR("PHY clock is  turned off, cannot reset");
		return -EIO;
	}

	ret = uhc_dwc2_hal_core_reset(dwc2, K_MSEC(10));
	if (ret != 0) {
		LOG_ERR("DWC2 core reset failed after PHY init: %d", ret);
		return ret;
	}

	ret = uhc_dwc2_quirk_phy_post_select(dev);
	if (ret != 0) {
		LOG_ERR("Quirk PHY post select failed %d", ret);
		return ret;
	}

	/* 4. Pre-calculate FIFO settings */
	uhc_dwc2_port_fifo_precalc_dma(dev);

	/* 5. Init DWC2 controller as a host */

	ret = uhc_dwc2_hal_init_host(dwc2);
	if (ret != 0) {
		return ret;
	}

	/* 6. Init channels list */
	for (uint32_t idx = 0; idx < CONFIG_UHC_DWC2_MAX_CHANNELS; idx++) {
		priv->channel[idx].base = UHC_DWC2_CHANNEL_REGS(dwc2, idx);
		priv->channel[idx].index = idx;
	}

	return 0;
}

static int uhc_dwc2_enable(const struct device *const dev)
{
	const struct uhc_dwc2_config *const config = dev->config;

	int ret;

	ret = uhc_dwc2_quirk_pre_enable(dev);
	if (ret != 0) {
		LOG_ERR("Quirk pre enable failed %d", ret);
		return ret;
	}

	/* 1. Flush root port config */

	/* TODO: Pre-calculate FIFO configuration */

	/* TODO: Flush channels */

	/* 2. Enable IRQ */
	config->irq_enable_func(dev);

	/* 3. Prepare for device connection */
	uhc_dwc2_port_power_on(dev);

	ret = uhc_dwc2_quirk_post_enable(dev);
	if (ret != 0) {
		LOG_ERR("Quirk post enable failed %d", ret);
		return ret;
	}

	return 0;
}

static int uhc_dwc2_disable(const struct device *const dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const dwc2 = config->base;
	int ret;

	return 0;

	/* TODO: Check ongoing transfer? */

	config->irq_disable_func(dev);
	sys_clear_bits((mem_addr_t)&dwc2->gahbcfg, USB_DWC2_GAHBCFG_GLBINTRMASK);

	/* Power down root port */
	uhc_dwc2_hal_port_set_power(dwc2, false);

	ret = uhc_dwc2_quirk_disable(dev);
	if (ret != 0) {
		LOG_ERR("Quirk init failed: %d", ret);
		return ret;
	}

	return 0;
}

static int uhc_dwc2_shutdown(const struct device *const dev)
{
	int ret;

	return 0;

	ret = uhc_dwc2_quirk_shutdown(dev);
	if (ret != 0) {
		LOG_ERR("Quirk shutdown failed: %d", ret);
		return ret;
	}

	return 0;
}

static int uhc_dwc2_soft_reset(const struct device *const dev)
{
	int ret;

	LOG_DBG("Applying soft reset");

	ret = uhc_dwc2_disable(dev);
	if (ret != 0) {
		return ret;
	}

	ret = uhc_dwc2_shutdown(dev);
	if (ret != 0) {
		return ret;
	}

	ret = uhc_dwc2_init(dev);
	if (ret != 0) {
		return ret;
	}

	ret = uhc_dwc2_enable(dev);
	if (ret != 0) {
		return ret;
	}

	return ret;
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

static int uhc_dwc2_preinit(const struct device *const dev)
{
	const struct uhc_dwc2_config *const config = dev->config;
	struct uhc_dwc2_data *const priv = uhc_get_private(dev);
	struct uhc_data *const data = dev->data;

	k_mutex_init(&data->mutex);
	k_event_init(&priv->event);
	k_sem_init(&priv->sem_port_en, 0, 1);

	uhc_dwc2_quirk_preinit(dev);

	k_thread_create(&priv->thread,
			config->stack,
			config->stack_size,
			uhc_dwc2_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_UHC_DWC2_THREAD_PRIORITY),
			K_ESSENTIAL,
			K_NO_WAIT);

	k_thread_name_set(&priv->thread, dev->name);

	return 0;
}

#ifndef UHC_DWC2_IRQ_DT_INST_DEFINE
#define UHC_DWC2_IRQ_DT_INST_DEFINE(n)						\
	static void uhc_dwc2_irq_enable_func_##n(const struct device *const dev)\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    uhc_dwc2_isr_handler,				\
			    DEVICE_DT_INST_GET(n),				\
			    0);							\
										\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void uhc_dwc2_irq_disable_func_##n(const struct device *const dev)\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}
#endif

#define UHC_DWC2_DEVICE_DEFINE(n)						\
	K_THREAD_STACK_DEFINE(uhc_dwc2_stack_##n, CONFIG_UHC_DWC2_STACK_SIZE);	\
										\
	UHC_DWC2_IRQ_DT_INST_DEFINE(n)						\
										\
	static struct uhc_dwc2_data uhc_dwc2_priv_##n = {			\
		.event = Z_EVENT_INITIALIZER(uhc_dwc2_priv_##n.event),		\
		.sem_port_en =							\
			Z_SEM_INITIALIZER(uhc_dwc2_priv_##n.sem_port_en, 0, 1),	\
	};									\
										\
	static struct uhc_data uhc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(uhc_data_##n.mutex),		\
		.priv = &uhc_dwc2_priv_##n,					\
	};									\
										\
	static const struct uhc_dwc2_config uhc_dwc2_config_##n = {		\
		.base = (struct usb_dwc2_reg *)DT_INST_REG_ADDR(n),		\
		.stack = uhc_dwc2_stack_##n,					\
		.stack_size = K_THREAD_STACK_SIZEOF(uhc_dwc2_stack_##n),	\
		.quirk_api = &uhc_dwc2_vendor_quirks_##n,			\
		.quirk_config = &uhc_dwc2_quirk_config_##n,			\
		.quirk_data = &uhc_dwc2_quirk_data_##n,				\
		.irq_enable_func = uhc_dwc2_irq_enable_func_##n,		\
		.irq_disable_func = uhc_dwc2_irq_disable_func_##n,		\
		.gsnpsid = DT_INST_PROP(n, gsnpsid),				\
		.ghwcfg1 = DT_INST_PROP(n, ghwcfg1),				\
		.ghwcfg2 = DT_INST_PROP(n, ghwcfg2),				\
		.ghwcfg3 = DT_INST_PROP(n, ghwcfg3),				\
		.ghwcfg4 = DT_INST_PROP(n, ghwcfg4),				\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, uhc_dwc2_preinit, NULL,			\
		&uhc_data_##n, &uhc_dwc2_config_##n,				\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
		&uhc_dwc2_api);

DT_INST_FOREACH_STATUS_OKAY(UHC_DWC2_DEVICE_DEFINE)
