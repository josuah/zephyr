/* SPDX-License-Identifier: MIT
 * Copyright (c) 2023 tinyVision.ai
 * Author: Josuah Demangeon <me@josuah.net>
 *
 * Hardware registers used by this driver.
 */

#include <zephyr/sys/util.h>

/*
 * Hardware format representing one DMA transaction request passed from the 
 * CPU to the DWC3 core.
 */
struct dwc3_trb {
	/* uint64_t address at which DWC3 should perform the DMA request */
	uint32_t addr_lo;
	uint32_t addr_hi;

	/* Status of the DMA transaction to perform, featuring the BUFSIZ field */
	uint32_t status;
#define DWC3_TRB_STATUS_TRBSTS_MASK		GENMASK(31, 28)	
#define DWC3_TRB_STATUS_TRBSTS_OK		(0x0 << 28)	
#define DWC3_TRB_STATUS_TRBSTS_MISSEDISOC	(0x1 << 28)
#define DWC3_TRB_STATUS_TRBSTS_SETUPPENDING	(0x2 << 28)
#define DWC3_TRB_STATUS_PCM1_MASK		GENMASK(25, 24)
#define DWC3_TRB_STATUS_PCM1_1PKT		(0x0 << 24)
#define DWC3_TRB_STATUS_PCM1_2PKT		(0x1 << 24)
#define DWC3_TRB_STATUS_PCM1_3PKT		(0x2 << 24)
#define DWC3_TRB_STATUS_PCM1_4PKT		(0x3 << 24)
#define DWC3_TRB_STATUS_BUFSIZ_MASK		GENMASK(23, 0)
#define DWC3_TRB_STATUS_BUFSIZ_SHIFT		0

	/* Control parameters: type of operation, direction, extra flags */
	uint32_t ctrl;
#define DWC3_TRB_CTRL_HWO			BIT(0)
#define DWC3_TRB_CTRL_LST			BIT(1)
#define DWC3_TRB_CTRL_CHN			BIT(2)
#define DWC3_TRB_CTRL_CSP			BIT(3)
#define DWC3_TRB_CTRL_TRBCTL_MASK		GENMASK(9, 4)
#define DWC3_TRB_CTRL_TRBCTL_SHIFT		4
#define DWC3_TRB_CTRL_ISP_IMI			BIT(10)
#define DWC3_TRB_CTRL_IOC			BIT(11)
#define DWC3_TRB_CTRL_SIDSOFN_MASK		GENMASK(29, 14)
#define DWC3_TRB_CTRL_SIDSOFN_SHIFT		14

} __packed;

/* Device Endpoint Command  */
#define DWC3_DEPCMD_BASE(n)			(0xc800 + (n) * 0x10)
#define DWC3_DEPCMDPAR2(n)			(DWC3_DEPCMD_BASE(n) + 0x00)
#define DWC3_DEPCMDPAR1(n)			(DWC3_DEPCMD_BASE(n) + 0x04)
#define DWC3_DEPCMDPAR0(n)			(DWC3_DEPCMD_BASE(n) + 0x08)
#define DWC3_DEPCMD(n)				(DWC3_DEPCMD_BASE(n) + 0x0c)
/* Written values */
#define DWC3_DEPCMD_DEPCFG			1
#define DWC3_DEPCMD_DEPXFERCFG			2
#define DWC3_DEPCMD_DEPGETDSEQ			3
#define DWC3_DEPCMD_DEPSETSTALL			4
#define DWC3_DEPCMD_DEPCSTALL			5
#define DWC3_DEPCMD_DEPSTRTXFER			6
#define DWC3_DEPCMD_DEPUPDXFER			7
#define DWC3_DEPCMD_DEPENDXFER			8
#define DWC3_DEPCMD_DEPSTARTCFG			9
/* Read values */
#define DWC3_DEPCMD_CMDACT			BIT(10)
