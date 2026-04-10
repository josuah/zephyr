/*
 * Copyright (c) 2026 Roman Leonov <jam_roma@yahoo.com>
 * SPDX-FileCopyrightText: Copyright Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UHC_DWC2_NRF_USBHS_NRF54L
#define ZEPHYR_DRIVERS_USB_UHC_DWC2_NRF_USBHS_NRF54L

#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#define USBHS_DT_WRAPPER_REG_ADDR(n) UINT_TO_POINTER(DT_REG_ADDR(DT_INST_PARENT(n)))

#define NRF_DEFAULT_IRQ_PRIORITY 1

/*
 * On USBHS, we cannot access the DWC2 register until VBUS is detected and
 * valid. If the user tries to force usbd_enable() and the corresponding
 * uhc_enable() without a "VBUS ready" notification, the event wait will block.
 */
static K_EVENT_DEFINE(usbhs_events);
#define USBHS_VBUS_READY   BIT(0)
#define USBHS_VBUS_REMOVED BIT(1)

static struct onoff_manager *pclk24m_mgr;
static struct onoff_client pclk24m_cli;

static void vregusb_isr(const void *arg)
{
	if (NRF_VREGUSB->EVENTS_VBUSDETECTED) {
		NRF_VREGUSB->EVENTS_VBUSDETECTED = 0;
		k_event_post(&usbhs_events, USBHS_VBUS_READY);
	}

	if (NRF_VREGUSB->EVENTS_VBUSREMOVED) {
		NRF_VREGUSB->EVENTS_VBUSREMOVED = 0;
		k_event_set_masked(&usbhs_events, 0, USBHS_VBUS_REMOVED);
	}
}

static int usbhs_enable_core(const struct device *dev)
{
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);
	int err;

	if (!k_event_wait(&usbhs_events, USBHS_VBUS_READY, false, K_NO_WAIT)) {
		LOG_WRN("VBUS is not ready, block uhc_enable()");
		if (!k_event_wait(&usbhs_events, USBHS_VBUS_READY, false, K_FOREVER)) {
			return -ETIMEDOUT;
		}
	}

	/* Request PCLK24M using clock control driver */
	sys_notify_init_spinwait(&pclk24m_cli.notify);
	err = onoff_request(pclk24m_mgr, &pclk24m_cli);
	if (err < 0) {
		LOG_ERR("Failed to start PCLK24M %d", err);
		return err;
	}

	/* Power up peripheral */
	wrapper->ENABLE = USBHS_ENABLE_CORE_Msk;

	/* Set ID to Host and disable D+ pull-up */
	wrapper->PHY.OVERRIDEVALUES = (1 << 24) | (1 << 23);
	wrapper->PHY.INPUTOVERRIDE = (1 << 31) | (1 << 30) | (1 << 24) | (1 << 23);

	/* Release PHY power-on reset */
	wrapper->ENABLE = USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk;

	/* Wait for PHY clock to start */
	k_busy_wait(45);

	/* Release DWC2 reset */
	wrapper->TASKS_START = 1UL;

	/* Wait for clock to start to avoid hang on too early register read */
	k_busy_wait(1);

	return 0;
}

static int usbhs_disable_core(const struct device *dev)
{
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);
	int err;

	/* Set ID to Device and forcefully disable D+ pull-up */
	wrapper->PHY.OVERRIDEVALUES = (1 << 31);
	wrapper->PHY.INPUTOVERRIDE = (1 << 31) | USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk;

	wrapper->ENABLE = 0UL;

	/* Release PCLK24M using clock control driver */
	err = onoff_cancel_or_release(pclk24m_mgr, &pclk24m_cli);
	if (err < 0) {
		LOG_ERR("Failed to stop PCLK24M %d", err);
		return err;
	}

	return 0;
}

static int usbhs_init_vreg_and_clock_and_core(const struct device *dev)
{
	IRQ_CONNECT(VREGUSB_IRQn, NRF_DEFAULT_IRQ_PRIORITY, vregusb_isr, DEVICE_DT_INST_GET(0), 0);

	/* Init VREG */
	NRF_VREGUSB->INTEN = VREGUSB_INTEN_VBUSDETECTED_Msk | VREGUSB_INTEN_VBUSREMOVED_Msk;
	NRF_VREGUSB->TASKS_START = 1;

	/* TODO: Determine conditions when VBUSDETECTED is not generated */
	if (sys_read32((mem_addr_t)NRF_VREGUSB + 0x400) & BIT(2)) {
		k_event_post(&usbhs_events, USBHS_VBUS_READY);
	}

	irq_enable(VREGUSB_IRQn);

	/* Init the clock */
	pclk24m_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF24M);

	/* Enable the core */
	return usbhs_enable_core(dev);

	/* It is now possible to access the configuration registers */
}

static int usbhs_disable_vreg(const struct device *dev)
{
	NRF_VREGUSB->INTEN = 0;
	NRF_VREGUSB->TASKS_STOP = 1;

	return 0;
}

static int usbhs_init_caps(const struct device *dev)
{
	struct uhc_data *data = dev->data;

	data->caps.hs = true;

	return 0;
}

#define QUIRK_NRF_USBHS_DEFINE(n)						\
	struct uhc_dwc2_vendor_quirks uhc_dwc2_vendor_quirks_##n = {		\
		.init = usbhs_init_vreg_and_clock_and_core,			\
		.pre_enable = usbhs_enable_core,				\
		.disable = usbhs_disable_core,					\
		.shutdown = usbhs_disable_vreg,					\
		.caps = usbhs_init_caps,					\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_NRF_USBHS_DEFINE)

#endif /* ZEPHYR_DRIVERS_USB_UHC_DWC2_NRF_USBHS_NRF54L */
