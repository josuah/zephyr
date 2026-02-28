/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC3_VENDOR_QUIRKS_TI_AM62_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC3_VENDOR_QUIRKS_TI_AM62_H

#include <zephyr/drivers/syscon.h>
#include <zephyr/drivers/clock_control/tisci_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/device_mmio.h>

struct dwc3_ti_phy_regs {
	uint8_t RESERVED[0x130]; /**< Reserved, offset: 0x0 - 0x130 */
	volatile uint32_t REG12; /**< PHY2 PLL REG12, offset: 0x130 */
};

struct dwc3_ti_usbss_regs {
	uint8_t RESERVED_1[0x8];        /**< Reserved, offset: 0x0 - 0x8 */
	volatile uint32_t PHY_CONFIG;   /**< Static PHY configuration, offset: 0x8 */
	uint8_t RESERVED_2[0x10];       /**< Reserved, offset: 0x0C - 0x1C */
	volatile uint32_t MODE_CONTROL; /**< DRD role determination , offset: 0x1C */
};

#define DEV_CFG(dev)   ((const struct dwc3_ti_config *)(dev)->config)
#define DEV_DATA(dev)  ((struct dwc3_ti_data *)(dev)->data)
#define DEV_USBSS(dev) ((struct dwc3_ti_usbss_regs *)DEVICE_MMIO_NAMED_GET(dev, usbss))
#define DEV_PHY(dev)   ((struct dwc3_ti_phy_regs *)DEVICE_MMIO_NAMED_GET(dev, phy))

/* USB syscon module registers */
#define DWC3_TI_PHY_PLL_REFCLK_MASK GENMASK(3, 0)

/* USB PHY2 PLL registers */
#define DWC3_TI_PHY_PLL_LDO_REF_EN    BIT(5)
#define DWC3_TI_PHY_PLL_LDO_REF_EN_EN BIT(4)

/* USB CFG registers */
#define DWC3_TI_CFG_PHY_CONFIG_VBUS_SEL     BIT(1)
#define DWC3_TI_CFG_MODE_CONTROL_MODE_VALID BIT(0)

struct dwc3_ti_config {
	DEVICE_MMIO_NAMED_ROM(usbss);
	DEVICE_MMIO_NAMED_ROM(phy);

	const struct device *syscon_pll_refclk;
	mem_addr_t offset_pll_refclk;
	const struct device *clock_controller;
	clock_control_subsys_t clock_subsys;
	bool vbus_divider;
};

struct dwc3_ti_data {
	DEVICE_MMIO_NAMED_RAM(usbss);
	DEVICE_MMIO_NAMED_RAM(phy);
};

static int dwc3_ti_phy_syscon_pll_refclk(struct udc_dwc3_quirk_data *qdev)
{
	const int dwc3_ti_rate_table[] = {
		9600,  10000, 12000, 19200, 20000, 24000, 25000,
		26000, 38400, 40000, 58000, 50000, 52000,
	};
	const struct dwc3_ti_config *cfg = DEV_CFG(qdev);
	uint32_t rate;
	int rv = 0;

	rv = clock_control_get_rate(cfg->clock_controller, cfg->clock_subsys, &rate);
	if (rv != 0) {
		LOG_ERR("failed to get the clock rate");
	}

	if (cfg->syscon_pll_refclk == NULL) {
		LOG_ERR("no syscon device for ti,syscon-phy-pll-refclk");
		return -EINVAL;
	}

	ARRAY_FOR_EACH(dwc3_ti_rate_table, code) {
		if (dwc3_ti_rate_table[code] == (rate / 1000)) {
			uint32_t reg = 0;

			rv = syscon_read_reg(cfg->syscon_pll_refclk, cfg->offset_pll_refclk, &reg);
			if (rv != 0) {
				LOG_ERR("failed to read syscon register");
				return rv;
			}

			reg &= ~DWC3_TI_PHY_PLL_REFCLK_MASK;
			reg |= FIELD_PREP(DWC3_TI_PHY_PLL_REFCLK_MASK, code);

			rv = syscon_write_reg(cfg->syscon_pll_refclk, cfg->offset_pll_refclk, reg);
			if (rv != 0) {
				LOG_ERR("failed to write syscon register");
				return rv;
			}

			return 0;
		}
	}

	LOG_ERR("failed to find correct rate code for the refclk frequency");
	return -EINVAL;
}

static void dwc3_ti_configure_usbss(struct udc_dwc3_quirk_data *qdev)
{
	const struct dwc3_ti_config *cfg = DEV_CFG(qdev);
	struct dwc3_ti_usbss_regs *usbss = DEV_USBSS(qdev);

	if (cfg->vbus_divider) {
		usbss->PHY_CONFIG |= DWC3_TI_CFG_PHY_CONFIG_VBUS_SEL;
	}
	usbss->MODE_CONTROL |= DWC3_TI_CFG_MODE_CONTROL_MODE_VALID;
}

static void dwc3_ti_configure_phy2_pll(struct udc_dwc3_quirk_data *qdev)
{
	struct dwc3_ti_phy_regs *phy = DEV_PHY(qdev);

	phy->REG12 |= (DWC3_TI_PHY_PLL_LDO_REF_EN | DWC3_TI_PHY_PLL_LDO_REF_EN_EN);
}

#define UDC_DWC3_HAS_QUIRK_PREINIT
int udc_dwc3_quirk_preinit(const struct device *dev)
{
	const struct udc_dwc3_config *config = dev->config;
	struct udc_dwc3_quirk_data *qdev = config->quirk_data;
	int rv = 0;

	DEVICE_MMIO_NAMED_MAP(qdev, usbss, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(qdev, phy, K_MEM_CACHE_NONE);

	rv = dwc3_ti_phy_syscon_pll_refclk(qdev);
	if (rv != 0) {
		return rv;
	}

	dwc3_ti_configure_phy2_pll(qdev);
	dwc3_ti_configure_usbss(qdev);

	return 0;
}

#define DWC3_TI_DEFINE_CLOCK_SUBSYS(n)						\
	COND_CODE_1(CONFIG_CLOCK_CONTROL_TISCI, (				\
		static const struct tisci_clock_config tisci_refclk_##n =	\
			TISCI_GET_CLOCK_DETAILS_BY_INST(n);			\
	), ())

#define DWC3_TI_CLOCK_CONTROL(n)						\
	COND_CODE_1(CONFIG_CLOCK_CONTROL_TISCI, (				\
		.clock_controller = TISCI_GET_CLOCK_BY_INST(n),			\
		.clock_subsys = (clock_control_subsys_t) &tisci_refclk_##n,	\
	), ())

#define DWC3_TI_PHY_SYSCON_PLL_REFCLK_INIT(n)					\
	.syscon_pll_refclk =							\
		DEVICE_DT_GET_OR_NULL(DT_PARENT(DT_INST_PHANDLE(n, ti_syscon_phy_pll_refclk))),\
	.offset_pll_refclk = DT_REG_ADDR(DT_INST_PHANDLE(n, ti_syscon_phy_pll_refclk))

#define UDC_DWC3_QUIRK_DEFINE(n)						\
	DWC3_TI_DEFINE_CLOCK_SUBSYS(n)						\
	static const struct dwc3_ti_config udc_dwc3_quirk_config_##n = {	\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(usbss, DT_DRV_INST(n)),	\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(phy, DT_DRV_INST(n)),	\
		.vbus_divider = DT_INST_PROP(n, ti_vbus_divider),		\
		DWC3_TI_PHY_SYSCON_PLL_REFCLK_INIT(n),				\
		DWC3_TI_CLOCK_CONTROL(n),					\
	};									\
	static struct dwc3_ti_data udc_dwc3_quirk_data_##n;

#undef DEV_CFG
#undef DEV_DATA
#undef DEV_USBSS
#undef DEV_PHY

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC3_VENDOR_QUIRKS_TI_AM62_H */
