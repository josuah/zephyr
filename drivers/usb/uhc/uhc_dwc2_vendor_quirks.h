/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Roman Leonov <jam_roma@yahoo.com>
 * SPDX-FileCopyrightText: Copyright Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Vendor quirks handling
 *
 * Definition of vendor-specific functions that can be overwritten on a per-SoC basis
 * by defining the associated macro to inhibit the default no-op alias.
 */

#if DT_HAS_COMPAT_STATUS_OKAY(espressif_esp32_usb_otg)

static void uhc_dwc2_isr_handler(const struct device *const dev);

#define UHC_DWC2_IRQ_DT_INST_DEFINE(n)						\
	static void uhc_dwc2_irq_enable_func_##n(const struct device *const dev)\
	{									\
		(void)uhc_dwc2_isr_handler;					\
		/* TODO: esp_intr_enable */					\
	}									\
										\
	static void uhc_dwc2_irq_disable_func_##n(const struct device *const dev)\
	{									\
		/* TODO: esp_intr_enable */					\
	}

#endif /* DT_HAS_COMPAT_STATUS_OKAY(espressif_esp32_usb_otg) */

/* Add next vendor quirks definition above this line */

/* Default data potentially stored in config->quirk_data */
#ifndef UHC_DWC2_QUIRK_DATA
#define UHC_DWC2_QUIRK_DATA(n) static int uhc_dwc2_quirk_data_##n;
#endif

/* Default IRQ enable/disable functions */
#ifndef UHC_DWC2_IRQ_DT_INST_DEFINE
#define UHC_DWC2_IRQ_DT_INST_DEFINE(n)						\
	static void uhc_dwc2_irq_enable_func_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			uhc_dwc2_isr_handler, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void uhc_dwc2_irq_disable_func_##n(const struct device *dev)	\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}
#endif

/* Fallback no-op implementations */
#ifndef UHC_DWC2_HAS_QUIRK_INIT
static int uhc_dwc2_quirk_init(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UHC_DWC2_HAS_QUIRK_PRE_ENABLE
static int uhc_dwc2_quirk_pre_enable(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UHC_DWC2_HAS_QUIRK_POST_ENABLE
static int uhc_dwc2_quirk_post_enable(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UHC_DWC2_HAS_QUIRK_DISABLE
static int uhc_dwc2_quirk_disable(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UHC_DWC2_HAS_QUIRK_SHUTDOWN
static int uhc_dwc2_quirk_shutdown(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UHC_DWC2_HAS_QUIRK_IRQ_CLEAR
static int uhc_dwc2_quirk_irq_clear(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
#ifndef UHC_DWC2_HAS_QUIRK_PREINIT
static int uhc_dwc2_quirk_preinit(const struct device *const dev)
{
	LOG_DBG("Fallback quirk called for %s", __func__);
	return 0;
}
#endif
