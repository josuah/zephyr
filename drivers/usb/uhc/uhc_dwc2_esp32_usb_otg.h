/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Roman Leonov <jam_roma@yahoo.com>
 * SPDX-FileCopyrightText: Copyright Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_UHC_DWC2_ESP32_USB_OTG_H
#define ZEPHYR_DRIVERS_UHC_DWC2_ESP32_USB_OTG_H

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/drivers/clock_control.h>

#include <esp_err.h>
#include <esp_private/usb_phy.h>
#include <esp_rom_gpio.h>
#include <hal/gpio_ll.h>
#include <hal/usb_wrap_hal.h>
#include <soc/gpio_sig_map.h>
#include <soc/usb_periph.h>
#include <soc/usb_pins.h>

struct usb_dw_esp32_config {
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	int irq_source;
	int irq_priority;
	int irq_flags;
	/* PHY context */
	usb_phy_target_t phy_target;
	usb_phy_controller_t phy_controller;
	usb_phy_status_t phy_status;
	usb_otg_mode_t phy_otg_mode;
	usb_phy_speed_t phy_otg_speed;
};

struct usb_dw_esp32_data {
	struct intr_handle_data_t *int_handle;
	/* PHY context */
	usb_wrap_hal_context_t phy_wrap_hal;
};

static inline int esp32_usb_otg_init(const struct device *dev)
{
	const struct uhc_dwc2_config *const cfg = dev->config;
	const struct usb_dw_esp32_config *const quirk_cfg = cfg->quirk_config;
	struct usb_dw_esp32_data *const quirk_data = cfg->quirk_data;
	int ret;

	if (!device_is_ready(quirk_cfg->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(quirk_cfg->clock_dev, quirk_cfg->clock_subsys);
	if (ret != 0) {
		return ret;
	}

	/* pinout config to work in USB_OTG_MODE_HOST */
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT,
				       USB_OTG_IDDIG_IN_IDX,
				       false);
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT,
				       USB_SRP_BVALID_IN_IDX,
				       false);
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
				       USB_OTG_VBUSVALID_IN_IDX,
				       false);
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT,
				       USB_OTG_AVALID_IN_IDX,
				       false);

	if (quirk_cfg->phy_target == USB_PHY_TARGET_INT) {
		gpio_ll_set_drive_capability(GPIO_LL_GET_HW(0), USBPHY_DM_NUM, GPIO_DRIVE_CAP_3);
		gpio_ll_set_drive_capability(GPIO_LL_GET_HW(0), USBPHY_DP_NUM, GPIO_DRIVE_CAP_3);
	}

	/* allocate interrupt but keep it disabled to avoid
	 * spurious suspend/resume event at enumeration phase
	 */
	ret = esp_intr_alloc(
		quirk_cfg->irq_source,
		ESP_INTR_FLAG_INTRDISABLED | ESP_PRIO_TO_FLAGS(quirk_cfg->irq_priority) |
		ESP_INT_FLAGS_CHECK(quirk_cfg->irq_flags),
		(intr_handler_t)uhc_dwc2_isr_handler,
		(void *)dev,
		&quirk_data->int_handle);

	return ret;
}

static inline int esp32_usb_otg_enable_phy(const struct device *const dev)
{
	const struct uhc_dwc2_config *const cfg = dev->config;
	const struct usb_dw_esp32_config *const quirk_cfg = cfg->quirk_config;
	struct usb_dw_esp32_data *const quirk_data = cfg->quirk_data;

	usb_wrap_ll_enable_bus_clock(true);
	usb_wrap_hal_init(&quirk_data->phy_wrap_hal);

#if USB_WRAP_LL_EXT_PHY_SUPPORTED
	usb_wrap_hal_phy_set_external(&quirk_data->phy_wrap_hal,
			(quirk_cfg->phy_target == USB_PHY_TARGET_EXT));
#endif
	if (quirk_cfg->phy_target == USB_PHY_TARGET_INT) {
		/* Configure pull resistors for host */
		usb_wrap_pull_override_vals_t vals = {
			.dp_pu = false,
			.dm_pu = false,
			.dp_pd = true,
			.dm_pd = true,
		};
		usb_wrap_hal_phy_enable_pull_override(&quirk_data->phy_wrap_hal, &vals);
	}

	LOG_DBG("PHY enabled");

	return 0;
}

static inline int esp32_usb_otg_disable_phy(const struct device *const dev)
{
	const struct uhc_dwc2_config *const cfg = dev->config;
	struct usb_dw_esp32_data *const quirk_data = cfg->quirk_data;

	usb_wrap_ll_enable_bus_clock(false);
	usb_wrap_ll_phy_enable_pad(quirk_data->phy_wrap_hal.dev, false);

	LOG_DBG("PHY disabled");

	return 0;
}

static void esp32_usb_otg_irq_enable_func(const struct device *const dev)
{
	const struct uhc_dwc2_config *const cfg = dev->config;
	struct usb_dw_esp32_data *const quirk_data = cfg->quirk_data;

	esp_intr_enable(quirk_data->int_handle);
}

static void esp32_usb_otg_irq_disable_func(const struct device *const dev)
{
	const struct uhc_dwc2_config *const cfg = dev->config;
	struct usb_dw_esp32_data *const quirk_data = cfg->quirk_data;

	esp_intr_disable(quirk_data->int_handle);
}

#define UHC_DWC2_IRQ_DEFINE(n)							\
	static void uhc_dwc2_irq_enable_func_##n(const struct device *const dev)\
	{									\
		esp32_usb_otg_irq_enable_func(dev);				\
	}									\
										\
	static void uhc_dwc2_irq_disable_func_##n(const struct device *const dev)\
	{									\
		esp32_usb_otg_irq_disable_func(dev);				\
	}									\

#define ESP32_USB_OTG_DEFINE(n)							\
	static const struct uhc_dwc2_vendor_quirks uhc_dwc2_vendor_quirks_##n = {\
		.init = esp32_usb_otg_init,					\
		.pre_enable = esp32_usb_otg_enable_phy,				\
		.disable = esp32_usb_otg_disable_phy,				\
	};									\
										\
	static const struct usb_dw_esp32_config uhc_dwc2_quirk_config_##n = {	\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
		.clock_subsys = (clock_control_subsys_t)			\
			DT_INST_CLOCKS_CELL(n, offset),				\
		.irq_source = DT_INST_IRQ_BY_IDX(n, 0, irq),			\
		.irq_priority = DT_INST_IRQ_BY_IDX(n, 0, priority),		\
		.irq_flags = DT_INST_IRQ_BY_IDX(n, 0, flags),			\
		.phy_target = USB_PHY_TARGET_INT,				\
		.phy_controller = USB_PHY_CTRL_OTG,				\
		.phy_otg_mode = USB_OTG_MODE_HOST,				\
		.phy_otg_speed = USB_PHY_SPEED_UNDEFINED,			\
	};									\
										\
	static struct usb_dw_esp32_data uhc_dwc2_quirk_data_##n;

DT_INST_FOREACH_STATUS_OKAY(ESP32_USB_OTG_DEFINE)

#endif /* ZEPHYR_DRIVERS_UHC_DWC2_ESP32_USB_OTG_H */
