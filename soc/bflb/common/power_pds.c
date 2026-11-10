/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Panoramix Labs
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bflb_power_pds, CONFIG_KERNEL_LOG_LEVEL);

#include <bflb_soc.h>
#include <hbn_reg.h>
#include <aon_reg.h>

/* PDS interrupt type definition */
#define PDS_INT_WAKEUP           (0) /* PDS wakeup interrupt(assert bit while wakeup, include PDS_Timer/...) */
#define PDS_INT_RF_DONE          (1) /* PDS RF done interrupt */
#define PDS_INT_WIFI_TBTT_SLEEP  (2) /* PDS wifi tbtt sleep interrupt */
#define PDS_INT_WIFI_TBTT_WAKEUP (3) /* PDS wifi tbtt wakeup interrupt */
#define PDS_INT_MAX              (4) /* PDS int max number */

/* PDS configuration type definition */
#define PDS_CTL1_OFFSET				0U
/* PDS Start */
#define PDS_CTL1_PDS_START_MASK			BIT(0)
/* PDS sleep forever */
#define PDS_CTL1_SLEEP_FOREVER_MASK		BIT(1)
/* Power off xtal force */
#define PDS_CTL1_XTALFORCE_OFF_MASK		BIT(2)
/* Save WIFI State Before Enter PDS */
#define PDS_CTL1_SAVE_WIFI_STATE_MASK		BIT(3)
/* power down ldo11 during PDS */
#define PDS_CTL1_LDO11_OFF_MASK			BIT(4)
/* power down bg_sys during PDS */
#define PDS_CTL1_BG_SYS_OFF_MASK		BIT(5)
/* PDS control the GPIO IE/PU/PD at Sleep Mode */
#define PDS_CTL1_CTRL_GPIO_IE_PU_PD_MASK	BIT(6)
/* power down dcdc18 during PDS */
#define PDS_CTL1_DCDC18_OFF_MASK		BIT(7)
/* gate clock during PDS (each pwr domain has its own control) */
#define PDS_CTL1_CLK_OFF_MASK			BIT(8)
/* mem_stby during PDS (each power domain can has its own control) */
#define PDS_CTL1_MEM_STBY_MASK			BIT(9)
/* avoid glb_reg reset by any reset */
#define PDS_CTL1_GLB_RST_PROTECT_MASK		BIT(10)
/* Isolation during PDS (each power domain can has its own control) */
#define PDS_CTL1_ISOLATION_MASK			BIT(11)
/* wait XTAL Ready during before PDS Interrupt */
#define PDS_CTL1_WAIT_XTAL_RDY_MASK		BIT(12)
/* Power off during PDS (each power domain can has its own control) */
#define PDS_CTL1_PDS_PWR_OFF_MASK		BIT(13)
/* xtal power down during PDS */
#define PDS_CTL1_XTAL_OFF_MASK			BIT(14)
/* pds_soc_enb always active */
#define PDS_CTL1_SOC_ENB_FORCE_ON_MASK		BIT(15)
/* pds_rst controlled by PDS */
#define PDS_CTL1_PDS_RST_SOC_EN_MASK		BIT(16)
/* RC32M always on or RC32M on/off controlled by PDS state */
#define PDS_CTL1_PDS_RC32M_ON_MASK		BIT(17)
/* PDS "SLEEP" control DCDC11 voltage enable */
#define PDS_CTL1_PDS_DCDC11V_SEL_EN_MASK	BIT(18)
/* PDS Control USB PLL off When pds_pwr_off */
#define PDS_CTL1_USB_PLL_OFF_MASK		BIT(19)
/* PDS control aupll power off */
#define PDS_CTL1_AUPLL_OFF_MASK			BIT(20)
/* PDS control wifipll power off */
#define PDS_CTL1_WIFIPLL_OFF_MASK		BIT(22)
/* DCDC11 voltage selection in PDS mode */
#define PDS_CTL1_PDS_DCDC11_VSEL_MASK		GENMASK(27, 23)
/* select the way RF controlled by PDS */
#define PDS_CTL1_PDS_CTL_RF_SEL_MASK		GENMASK(29, 28)
/* PDS Auto Sleep When wifi_tbtt_sleep_irq detect to 1 */
#define PDS_CTL1_PDS_USE_TBTT_SLP_MASK		BIT(30)
/* HW Keep GPIO at PDS Mode */
#define PDS_CTL1_PDS_GPIO_ISO_MOD_MASK		BIT(31)

/* PDS force configuration type definition */
#define PDS_CTL2_OFSET				4U
/* manual force NP power off */
#define PDS_CTL2_FORCE_CPU_PWR_OFF_MASK		BIT(0)
/* manual force WB power off */
#define PDS_CTL2_FORCE_WB_PWR_OFF_MASK		BIT(2)
/* manual force USB power off */
#define PDS_CTL2_FORCE_USB_PWR_OFF_MASK		BIT(3)
/* manual force NP isolation */
#define PDS_CTL2_FORCE_CPU_ISO_MASK		BIT(4)
/* manual force WB isolation */
#define PDS_CTL2_FORCE_WB_ISO_MASK		BIT(6)
/* manual force USB isolation */
#define PDS_CTL2_FORCE_USB_ISO_MASK		BIT(7)
/* manual force NP pds reset */
#define PDS_CTL2_FORCE_CPU_PDS_RST_MASK		BIT(8)
/* manual force WB pds reset */
#define PDS_CTL2_FORCE_WB_PDS_RST_MASK		BIT(10)
/* manual force USB pds reset */
#define PDS_CTL2_FORCE_USB_PD_SRST_MASK		BIT(11)
/* manual force NP memory sleep */
#define PDS_CTL2_FORCE_CPU_MEM_STBY_MASK	BIT(12)
/* manual force WB memory sleep */
#define PDS_CTL2_FORCE_WB_MEM_STBY_MASK		BIT(14)
/* manual force USB memory sleep */
#define PDS_CTL2_FORCE_USB_MEM_STBY_MASK	BIT(15)
/* manual force NP clock gated */
#define PDS_CTL2_FORCE_CPU_GATE_CLK_MASK	BIT(16)
/* manual force WB clock gated */
#define PDS_CTL2_FORCE_WB_GATE_CLK_MASK		BIT(18)
/* manual force USB clock gated */
#define PDS_CTL2_FORCE_USB_GATE_CLK_MASK	BIT(19)

/* PDS force configuration type definition */
#define PDS_CTL3_OFFSET				8U
/* manual force MISC pwr_off */
#define PDS_CTL3_FORCE_MISC_PWR_OFF_MASK	BIT(1)
/* manual force MISC iso_en */
#define PDS_CTL3_FORCE_MISC_ISO_EN_MASK		BIT(4)
/* manual force MISC pds_rst */
#define PDS_CTL3_FORCE_MISC_PDS_RST_MASK	BIT(7)
/* manual force MISC mem_stby */
#define PDS_CTL3_FORCE_MISC_MEM_STBY_MASK	BIT(10)
/* manual force MISC gate_clk */
#define PDS_CTL3_FORCE_MISC_GATE_CLK_MASK	BIT(13)
/* make CPU isolated at PDS Sleep state */
#define PDS_CTL3_CPU_ISO_EN_MASK		BIT(24)
/* make WB isolated at PDS Sleep state */
#define PDS_CTL3_WB_ISO_EN_MASK			BIT(27)
/* make USB isolated at PDS Sleep state */
#define PDS_CTL3_USB_ISO_EN_MASK		BIT(29)
/* make misc isolated at PDS Sleep state */
#define PDS_CTL3_MISC_ISO_EN_MASK		BIT(30)

/* PDS force configuration type definition */
#define PDS_CTL4_OFFSET				12U
#define PDS_CTL4_CPU_PWR_OFF_MASK		BIT(0)
#define PDS_CTL4_CPU_RST_MASK			BIT(1)
#define PDS_CTL4_CPU_MEM_STBY_MASK		BIT(2)
#define PDS_CTL4_CPU_GATE_CLK_MASK		BIT(3)
#define PDS_CTL4_WB_PWR_OFF_MASK		BIT(12)
#define PDS_CTL4_WB_RST_MASK			BIT(13)
#define PDS_CTL4_WB_MEM_STBY_MASK		BIT(14)
#define PDS_CTL4_WB_GATE_CLK_MASK		BIT(15)
#define PDS_CTL4_USB_PWR_OFF_MASK		BIT(20)
#define PDS_CTL4_USB_RST_MASK			BIT(21)
#define PDS_CTL4_USB_MEM_STBY_MASK		BIT(22)
#define PDS_CTL4_USB_GATE_CLK_MASK		BIT(23)
#define PDS_CTL4_MISC_PWR_OFF_MASK		BIT(24)
#define PDS_CTL4_MISC_RST_MASK			BIT(25)
#define PDS_CTL4_MISC_MEM_STBY_MASK		BIT(26)
#define PDS_CTL4_MISC_GATE_CLK_MASK		BIT(27)

/* PDS force configuration type definition */
#define PDS_CTL5_OFFSET				16U
/* pds start condition mask np_wfi */
#define PDS_CTL5_CPU_WFI_MASK_MASK		BIT(0)
/* GPIO21/22/28/29 5V Tolerant PAD Open Drain Enable */
#define PDS_CTL5_PDS_PAD_OD_EN_MASK		BIT(1)
/* HW control turn on/off USB 3.3V @USB1.1V Power On/OFF */
#define PDS_CTL5_CTRL_USB_33_MASK		BIT(8)
/* power down ldo18io during PDS */
#define PDS_CTL5_LDO_18_IO_OFF_MASK		BIT(9)
/* enable or disable keep function */
#define PDS_CTL5_PDS_GPIO_KEEP_MASK		GENMASK(18, 16)

/* `substate_id` encodes levels such as Level1, Level2, Level3, Level7, Level15, which are masks of
 * which power domain should be enabled.
 *
 * Levels<n> do not match the table with PDS<n> definition, both in the SDK's bl616_pm.c. This
 * driver uses the Level<n> naming convention as used by the actual data structures.
 */
#define POWER_DOMAIN_CPU_OFF_MASK		BIT(0)
#define POWER_DOMAIN_WIRELESS_OFF_MASK		BIT(1)
#define POWER_DOMAIN_USB_OFF_MASK		BIT(2)
#define POWER_DOMAIN_MISC_OFF_MASK		BIT(3)

/**
 * Data collected from the SDK for BL616 using awk:
 *
 * pdsCtl
 *
 * pdsStart          1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * sleepForever      0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * xtalForceOff      0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * saveWiFiState     0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * ldo11Off          1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * bgSysOff          1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * ctrlGpioIePuPd    0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * dcdc18Off         1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * clkOff            1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * memStby           1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * glbRstProtect     0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * isolation         1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * waitXtalRdy       0, (1)    0, (2)    1, (3)    1, (7)    1, (15)
 * pdsPwrOff         1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * xtalOff           1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * socEnbForceOn     0, (1)    0, (2)    0, (3)    0, (7)    1, (15)
 * pdsRstSocEn       0, (1)    0, (2)    0, (3)    0, (7)    1, (15)
 * pdsRC32mOn        0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * pdsDcdc11VselEn   0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * usbpllOff         1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * aupllOff          1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * wifipllOff        1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * pdsDcdc11Vsel     8, (1)    8, (2)    8, (3)    8, (7)    8, (15)
 * pdsCtlRfSel       3, (1)    3, (2)    3, (3)    3, (7)    3, (15)
 * pdsUseTbttSlp     0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * pdsGpioIsoMod     0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 *
 * pdsCtl2
 *
 * forceCpuPwrOff    0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceWbPwrOff     0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceUsbPwrOff    0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceCpuIso       0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceWbIso        0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceUsbIso       0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceCpuPdsRst    0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceWbPdsRst     0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceUsbPdsRst    0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceCpuMemStby   0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceWbMemStby    0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceUsbMemStby   0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceCpuGateClk   0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceWbGateClk    0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceUsbGateClk   0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 *
 * pdsCtl3
 *
 * forceMiscPwrOff   0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceMiscIsoEn    0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceMiscPdsRst   0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceMiscMemStby  0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * forceMiscGateClk  0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * cpuIsoEn          0, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * wbIsoEn           1, (1)    0, (2)    1, (3)    1, (7)    1, (15)
 * usbIsoEn          0, (1)    0, (2)    0, (3)    1, (7)    1, (15)
 * miscIsoEn         0, (1)    0, (2)    0, (3)    0, (7)    1, (15)
 *
 * pdsCtl4
 *
 * cpuPwrOff         0, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * cpuRst            0, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * cpuMemStby        0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * cpuGateClk        0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * wbPwrOff          1, (1)    0, (2)    1, (3)    1, (7)    1, (15)
 * wbRst             1, (1)    0, (2)    1, (3)    1, (7)    1, (15)
 * wbMemStby         0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * wbGateClk         0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * usbPwrOff         0, (1)    0, (2)    0, (3)    1, (7)    1, (15)
 * usbRst            0, (1)    0, (2)    0, (3)    1, (7)    1, (15)
 * usbMemStby        0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * usbGateClk        0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * miscPwrOff        0, (1)    0, (2)    0, (3)    0, (7)    1, (15)
 * miscRst           0, (1)    0, (2)    0, (3)    0, (7)    1, (15)
 * miscMemStby       0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * miscGateClk       0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 *
 * pdsCtl5
 *
 * cpuWfiMask        0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * pdsPadOdEn        0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 * ctrlUsb33         0, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * ldo18ioOff        1, (1)    1, (2)    1, (3)    1, (7)    1, (15)
 * pdsGpioKeep       0, (1)    0, (2)    0, (3)    0, (7)    0, (15)
 */

int pm_state_set(enum pm_state state, uint8_t substate_id)
{
	uint32_t tmp;
	uint32_t ctl1;

	if (state != PM_STATE_SOFT_OFF) {
		return -ENOTSUP;
	}

	/* CTL1 */

	tmp = 0;
	tmp |= PDS_CTL1_LDO11_OFF_MASK;
	tmp |= PDS_CTL1_BG_SYS_OFF_MASK;
	tmp |= PDS_CTL1_DCDC18_OFF_MASK;
	tmp |= PDS_CTL1_CLK_OFF_MASK;
	tmp |= PDS_CTL1_MEM_STBY_MASK;
	tmp |= PDS_CTL1_ISOLATION_MASK;
	tmp |= PDS_CTL1_PDS_PWR_OFF_MASK;
	tmp |= PDS_CTL1_XTAL_OFF_MASK;
	tmp |= PDS_CTL1_USB_PLL_OFF_MASK;
	tmp |= PDS_CTL1_AUPLL_OFF_MASK;
	tmp |= PDS_CTL1_WIFIPLL_OFF_MASK;
	tmp |= FIELD_PREP(PDS_CTL1_PDS_DCDC11_VSEL_MASK, 8);
	tmp |= FIELD_PREP(PDS_CTL1_PDS_CTL_RF_SEL_MASK, 3);

	ctl1 = tmp;
	sys_write32(tmp, PDS_BASE + PDS_CTL1_OFFSET);

	/* CTL2 */

	tmp = 0;

	sys_write32(tmp, PDS_BASE + PDS_CTL1_OFFSET);

	/* CTL3 */

	tmp = 0;
	/* Enable isolation cells signals by default */
	tmp |= PDS_CTL3_CPU_ISO_EN_MASK;
	tmp |= PDS_CTL3_WB_ISO_EN_MASK;
	tmp |= PDS_CTL3_USB_ISO_EN_MASK;
	tmp |= PDS_CTL3_MISC_ISO_EN_MASK;

	/* Disable incoming isolation cells for power-domains that are not used: would bring power
	 * to powered-off silicon: short circuit (not confirmed from SDK or doc).
	 */
	if ((substate_id & POWER_DOMAIN_CPU_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL3_CPU_ISO_EN_MASK;
	}
	if ((substate_id & POWER_DOMAIN_WIRELESS_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL3_WB_ISO_EN_MASK;
	}
	if ((substate_id & POWER_DOMAIN_USB_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL3_USB_ISO_EN_MASK;
	}
	if ((substate_id & POWER_DOMAIN_MISC_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL3_MISC_ISO_EN_MASK;
	}

	sys_write32(tmp, PDS_BASE + PDS_CTL3_OFFSET);

	/* CTL4 */

	tmp = 0;

	/* Turn off the selected power domains */
	if ((substate_id & POWER_DOMAIN_CPU_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL4_CPU_PWR_OFF_MASK;
		tmp &= ~PDS_CTL4_CPU_RST_MASK;
	}
	if ((substate_id & POWER_DOMAIN_WIRELESS_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL4_WB_PWR_OFF_MASK;
		tmp &= ~PDS_CTL4_WB_RST_MASK;
	}
	if ((substate_id & POWER_DOMAIN_USB_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL4_USB_PWR_OFF_MASK;
		tmp &= ~PDS_CTL4_USB_RST_MASK;
	}
	if ((substate_id & POWER_DOMAIN_MISC_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL4_MISC_PWR_OFF_MASK;
		tmp &= ~PDS_CTL4_MISC_RST_MASK;
	}

	sys_write32(tmp, PDS_BASE + PDS_CTL4_OFFSET);

	/* CTL5 */

	tmp = 0;
	tmp |= PDS_CTL5_LDO_18_IO_OFF_MASK;

	/* Turn off USB 3.3V when the *CPU* (not USB?!) is off, which could be a bug/typo or not
	 * (i.e. dependency between power domains for electrical reasons).
	 */
	if ((substate_id & POWER_DOMAIN_CPU_OFF_MASK) != 0) {
		tmp &= ~PDS_CTL5_CTRL_USB_33_MASK;
	}

	sys_write32(tmp, PDS_BASE + PDS_CTL5_OFFSET);

	/* Enter low-power state */

	tmp = ctl1 | PDS_CTL1_PDS_START_MASK;

	sys_write32(tmp, PDS_BASE + PDS_CTL5_OFFSET);

	return 0;
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	/* Do nothing */
}
