/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC3_VENDOR_QUIRKS_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC3_VENDOR_QUIRKS_H

#if DT_HAS_COMPAT_STATUS_OKAY(snps_dwc3 /* <- replace with your more specific compatible */)

struct udc_dwc3_quirk_data {
	uint32_t custom_content;
};

#define UDC_DWC3_HAS_QUIRK_INIT
int udc_dwc3_quirk_init(const struct device *const dev)
{
	const struct udc_dwc3_config *const config = dev->config;
	struct udc_dwc3_quirk_data *const quirk_data = config->quirk_data;

	LOG_DBG("Example of quirk for init(): %x", quirk_data->custom_content);

	return 0;
}

#define UDC_DWC3_QUIRK_DEFINE(n)						\
static struct udc_dwc3_quirk_data udc_dwc3_quirk_data_##n = {		\
	.custom_content = 0x1234					\
};

#endif /* DT_HAS_COMPAT_STATUS_OKAY(your_compatible_string_here) */

/* Fallback no-op implementations */
#ifndef UDC_DWC3_HAS_QUIRK_PREINIT
int udc_dwc3_quirk_preinit(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UDC_DWC3_HAS_QUIRK_INIT
int udc_dwc3_quirk_init(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UDC_DWC3_HAS_QUIRK_ENABLE
int udc_dwc3_quirk_enable(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UDC_DWC3_HAS_QUIRK_DISABLE
int udc_dwc3_quirk_disable(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UDC_DWC3_HAS_QUIRK_SHUTDOWN
int udc_dwc3_quirk_shutdown(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC3_VENDOR_QUIRKS_H */
