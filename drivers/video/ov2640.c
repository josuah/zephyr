/*
 * Copyright (c) 2021 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ovti_ov2640

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/video/video.h>
#include <zephyr/sys/util.h>

#include "video_common.h"

LOG_MODULE_REGISTER(video_ov2640, CONFIG_VIDEO_LOG_LEVEL);

/* DSP register bank FF=0x00*/
#define OV2640_REG_QS     0x44
#define OV2640_REG_HSIZE  0x51
#define OV2640_REG_VSIZE  0x52
#define OV2640_REG_XOFFL  0x53
#define OV2640_REG_YOFFL  0x54
#define OV2640_REG_VHYX   0x55
#define OV2640_REG_TEST   0x57
#define OV2640_REG_ZMOW   0x5A
#define OV2640_REG_ZMOH   0x5B
#define OV2640_REG_ZMHH   0x5C
#define OV2640_REG_BPADDR 0x7C
#define OV2640_REG_BPDATA 0x7D
#define OV2640_REG_SIZEL  0x8C
#define OV2640_REG_HSIZE8 0xC0
#define OV2640_REG_VSIZE8 0xC1
#define OV2640_REG_CTRL1  0xC3

#define OV2640_REG_CTRLI       0x50
#define OV2640_REG_CTRLI_LP_DP 0x80

#define OV2640_REG_CTRL0        0xC2
#define OV2640_REG_CTRL0_YUV422 0x08
#define OV2640_REG_CTRL0_YUV_EN 0x04
#define OV2640_REG_CTRL0_RGB_EN 0x02

#define OV2640_REG_CTRL2           0x86
#define OV2640_REG_CTRL2_DCW_EN    0x20
#define OV2640_REG_CTRL2_SDE_EN    0x10
#define OV2640_REG_CTRL2_UV_ADJ_EN 0x08
#define OV2640_REG_CTRL2_UV_AVG_EN 0x04
#define OV2640_REG_CTRL2_CMX_EN    0x01

#define OV2640_REG_CTRL3              0x87
#define OV2640_REG_CTRL3_BPC_EN       0x80
#define OV2640_REG_CTRL3_WPC_EN       0x40
#define OV2640_REG_R_DVP_SP           0xD3
#define OV2640_REG_R_DVP_SP_AUTO_MODE 0x80

#define OV2640_REG_R_BYPASS           0x05
#define OV2640_REG_R_BYPASS_DSP_EN    0x00
#define OV2640_REG_R_BYPASS_DSP_BYPAS 0x01

#define OV2640_REG_IMAGE_MODE         0xDA
#define OV2640_REG_IMAGE_MODE_JPEG_EN 0x10
#define OV2640_REG_IMAGE_MODE_RGB565  0x08

#define OV2640_REG_RESET      0xE0
#define OV2640_REG_RESET_JPEG 0x10
#define OV2640_REG_RESET_DVP  0x04

#define OV2640_REG_MC_BIST              0xF9
#define OV2640_REG_MC_BIST_RESET        0x80
#define OV2640_REG_MC_BIST_BOOT_ROM_SEL 0x40

#define OV2640_REG_BANK_SEL        0xFF
#define OV2640_REG_BANK_SEL_DSP    0x00
#define OV2640_REG_BANK_SEL_SENSOR 0x01

/* Sensor register bank FF=0x01*/
#define OV2640_REG_COM1        0x03
#define OV2640_REG__PID     0x0A
#define OV2640_REG__PID_VAL 0x26
#define OV2640_REG__VER     0x0B
#define OV2640_REG__VER_VAL 0x42
#define OV2640_REG_AEC         0x10
#define OV2640_REG_CLKRC       0x11
#define OV2640_REG_COM10       0x15
#define OV2640_REG_HSTART      0x17
#define OV2640_REG_HSTOP       0x18
#define OV2640_REG_VSTART      0x19
#define OV2640_REG_VSTOP       0x1A
#define OV2640_REG_AEW         0x24
#define OV2640_REG_AEB         0x25
#define OV2640_REG_ARCOM2      0x34
#define OV2640_REG_FLL         0x46
#define OV2640_REG_FLH         0x47
#define OV2640_REG_COM19       0x48
#define OV2640_REG_ZOOMS       0x49
#define OV2640_REG_BD50        0x4F
#define OV2640_REG_BD60        0x50
#define OV2640_REG_5D       0x5D
#define OV2640_REG_5E       0x5E
#define OV2640_REG_5F       0x5F
#define OV2640_REG_60       0x60
#define OV2640_REG_HISTO_LOW   0x61
#define OV2640_REG_HISTO_HIGH  0x62

#define OV2640_REG_04           0x04
#define OV2640_REG_04_DEFAULT   0x28
#define OV2640_REG_04_HFLIP_IMG 0x80
#define OV2640_REG_04_VFLIP_IMG 0x40
#define OV2640_REG_04_VREF_EN   0x10
#define OV2640_REG_04_HREF_EN   0x08

#define OV2640_REG_COM2              0x09
#define OV2640_REG_COM2_OUT_DRIVE_X2 0x00

#define OV2640_REG_COM3             0x0C
#define OV2640_REG_COM3_DEFAULT     0x38
#define OV2640_REG_COM3_BAND_AUTO   0x02

#define OV2640_REG_COM7           0x12
#define OV2640_REG_COM7_SRST      0x80
#define OV2640_REG_COM7_RES_UXGA  0x00 /* UXGA */
#define OV2640_REG_COM7_ZOOM_EN   0x04 /* Enable Zoom */
#define OV2640_REG_COM7_COLOR_BAR 0x02 /* Enable Color Bar Test */

#define OV2640_REG_COM8         0x13
#define OV2640_REG_COM8_DEFAULT 0xC0
#define OV2640_REG_COM8_BNDF_EN 0x20 /* Enable Banding filter */
#define OV2640_REG_COM8_AGC_EN  0x04 /* AGC Auto/Manual control selection */
#define OV2640_REG_COM8_AEC_EN  0x01 /* Auto/Manual Exposure control */

#define OV2640_REG_COM9             0x14 /* AGC gain ceiling */
#define OV2640_REG_COM9_DEFAULT     0x08
#define OV2640_REG_COM9_AGC_GAIN_8X (0x02 << 5)

#define OV2640_REG_COM10 0x15

#define OV2640_REG_CTRL1_AWB 0x08 /* Enable AWB */

#define OV2640_REG_VV                  0x26
#define OV2640_REG_VV_AGC_TH(h, l) ((h << 4) | (l & 0x0F))

#define OV2640_REG_32      0x32
#define OV2640_REG_32_UXGA 0x36

#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define HD_720_WIDTH	1280
#define HD_720_HEIGHT	720
#define HD_1080_WIDTH	1920
#define HD_1080_HEIGHT	1080
#define QCIF_WIDTH	176
#define QCIF_HEIGHT	144
#define QQCIF_WIDTH	88
#define QQCIF_HEIGHT	72
#define QQVGA_WIDTH	160
#define QQVGA_HEIGHT	120
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define SVGA_WIDTH	800
#define SVGA_HEIGHT	600
#define SXGA_WIDTH	1280
#define SXGA_HEIGHT	1024
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define UXGA_WIDTH	1600
#define UXGA_HEIGHT	1200
#define XGA_WIDTH	1024
#define XGA_HEIGHT	768

struct ov2640_reg {
	uint8_t addr;
	uint8_t value;
};

struct ov2640_win_size {
	uint32_t width;
	uint32_t height;
	const struct ov2640_reg *regs;
	uint32_t regs_size;
};

#define OV2640_ZOOM_CONFIG(x, y, v_div, h_div, pclk_div)					\
	{OV2640_REG_CTRLI,									\
	 OV2640_REG_CTRLI_LP_DP |								\
	 FIELD_PREP(GENMASK(5, 3), v_div) |							\
	 FIELD_PREP(GENMASK(2, 0), h_div)},							\
	{OV2640_REG_ZMOW, FIELD_PREP(GENMASK(7, 0), (x) >> 2)},					\
	{OV2640_REG_ZMOH, FIELD_PREP(GENMASK(7, 0), (y) >> 2)},					\
	{OV2640_REG_ZMHH, FIELD_PREP(GENMASK(1, 0), (x) >> (8 + 2)) |				\
			  FIELD_PREP(GENMASK(2, 2), (y) >> (8 + 2))},				\
	{OV2640_REG_R_DVP_SP, pclk_div},							\
	{OV2640_REG_RESET, 0x00}

const int64_t ov2640_link_freq[] = {
	MHZ(12), // MHZ(48)
};

static const struct ov2640_reg ov2640_qqvga_regs[] = {
	OV2640_ZOOM_CONFIG(QQVGA_WIDTH, QQVGA_HEIGHT, 3, 3, 8),
};

static const struct ov2640_reg ov2640_qcif_regs[] = {
	OV2640_ZOOM_CONFIG(QCIF_WIDTH, QCIF_HEIGHT, 3, 3, 4),
};

static const struct ov2640_reg ov2640_240x240_regs[] = {
	OV2640_ZOOM_CONFIG(240, 240, 2, 2, 4),
};

static const struct ov2640_reg ov2640_qvga_regs[] = {
	OV2640_ZOOM_CONFIG(QVGA_WIDTH, QVGA_HEIGHT, 2, 2, 4),
};

static const struct ov2640_reg ov2640_cif_regs[] = {
	OV2640_ZOOM_CONFIG(CIF_WIDTH, CIF_HEIGHT, 2, 2, 8),
};

static const struct ov2640_reg ov2640_vga_regs[] = {
	OV2640_ZOOM_CONFIG(VGA_WIDTH, VGA_HEIGHT, 0, 0, 2),
};

static const struct ov2640_reg ov2640_svga_regs[] = {
	OV2640_ZOOM_CONFIG(SVGA_WIDTH, SVGA_HEIGHT, 1, 1, 2),
};

static const struct ov2640_reg ov2640_xga_regs[] = {
	OV2640_ZOOM_CONFIG(XGA_WIDTH, XGA_HEIGHT, 0, 0, 2),
	{OV2640_REG_CTRLI, 0x00},
};

static const struct ov2640_reg ov2640_sxga_regs[] = {
	OV2640_ZOOM_CONFIG(SXGA_WIDTH, SXGA_HEIGHT, 0, 0, 2),
	{OV2640_REG_CTRLI, 0x00},
	{OV2640_REG_R_DVP_SP, 2 | OV2640_REG_R_DVP_SP_AUTO_MODE},
};

static const struct ov2640_reg ov2640_uxga_regs[] = {
	OV2640_ZOOM_CONFIG(UXGA_WIDTH, UXGA_HEIGHT, 0, 0, 0),
	{OV2640_REG_CTRLI, 0x00},
	{OV2640_REG_R_DVP_SP, 0 | OV2640_REG_R_DVP_SP_AUTO_MODE},
};

#define OV2640_SIZE(w, h, r) {.width = w, .height = h, .regs = r, .regs_size = ARRAY_SIZE(r)}

static const struct ov2640_win_size ov2640_supported_win_sizes[] = {
	OV2640_SIZE(QQVGA_WIDTH, QQVGA_HEIGHT, ov2640_qqvga_regs),
	OV2640_SIZE(QCIF_WIDTH, QCIF_HEIGHT, ov2640_qcif_regs),
	OV2640_SIZE(240, 240, ov2640_240x240_regs),
	OV2640_SIZE(QVGA_WIDTH, QVGA_HEIGHT, ov2640_qvga_regs),
	OV2640_SIZE(CIF_WIDTH, CIF_HEIGHT, ov2640_cif_regs),
	OV2640_SIZE(VGA_WIDTH, VGA_HEIGHT, ov2640_vga_regs),
	OV2640_SIZE(SVGA_WIDTH, SVGA_HEIGHT, ov2640_svga_regs),
	OV2640_SIZE(XGA_WIDTH, XGA_HEIGHT, ov2640_xga_regs),
	OV2640_SIZE(SXGA_WIDTH, SXGA_HEIGHT, ov2640_sxga_regs),
	OV2640_SIZE(UXGA_WIDTH, UXGA_HEIGHT, ov2640_uxga_regs),
};

static const struct ov2640_reg default_regs[] = {
	{OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_DSP},
	{0x2c, 0xff},
	{0x2e, 0xdf},
	{OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR},
	{0x3c, 0x32},
	{OV2640_REG_CLKRC, 0x80},             /* Set PCLK divider */
	{OV2640_REG_COM2, OV2640_REG_COM2_OUT_DRIVE_X2}, /* Output drive x2 */
	{OV2640_REG_04,
		OV2640_REG_04_DEFAULT |
		OV2640_REG_04_HREF_EN},
	{OV2640_REG_COM8,
		OV2640_REG_COM8_DEFAULT |
		OV2640_REG_COM8_BNDF_EN |
		OV2640_REG_COM8_AGC_EN |
		OV2640_REG_COM8_AEC_EN},
	{OV2640_REG_COM9,
		OV2640_REG_COM9_DEFAULT |
		OV2640_REG_COM9_AGC_GAIN_8X},
	{OV2640_REG_COM10, 0x00}, /* Invert VSYNC */
	{0x2c, 0x0c},
	{0x33, 0x78},
	{0x3a, 0x33},
	{0x3b, 0xfb},
	{0x3e, 0x00},
	{0x43, 0x11},
	{0x16, 0x10},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x23, 0x00},
	{OV2640_REG_ARCOM2, 0xa0},
	{0x06, 0x02},
	{0x06, 0x88},
	{0x07, 0xc0},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	{0x4c, 0x00},
	{0x4a, 0x81},
	{0x21, 0x99},
	{OV2640_REG_AEW, 0x40},
	{OV2640_REG_AEB, 0x38},
	/* AGC/OV2640_REG_AEC fast mode operating region */
	{OV2640_REG_VV, OV2640_REG_VV_AGC_TH(0x08, 0x02)},
	{OV2640_REG_COM19, 0x00}, /* Zoom control 2 LSBs */
	{OV2640_REG_ZOOMS, 0x00}, /* Zoom control 8 MSBs */
	{0x5c, 0x00},
	{0x63, 0x00},
	{OV2640_REG_FLL, 0x00},
	{OV2640_REG_FLH, 0x00},

	/* Set banding filter */
	{OV2640_REG_COM3,
		OV2640_REG_COM3_DEFAULT |
		OV2640_REG_COM3_BAND_AUTO},
	{OV2640_REG_5D, 0x55},
	{OV2640_REG_5E, 0x7d},
	{OV2640_REG_5F, 0x7d},
	{OV2640_REG_60, 0x55},
	{OV2640_REG_HISTO_LOW, 0x70},
	{OV2640_REG_HISTO_HIGH, 0x80},
	{0x7c, 0x05},
	{0x20, 0x80},
	{0x28, 0x30},
	{0x6c, 0x00},
	{0x6d, 0x80},
	{0x6e, 0x00},
	{0x70, 0x02},
	{0x71, 0x94},
	{0x73, 0xc1},
	{0x3d, 0x34},
	/* { OV2640_REG_COM7, OV2640_REG_COM7_RES_UXGA | OV2640_REG_COM7_ZOOM_EN }, */
	{0x5a, 0x57},
	{OV2640_REG_BD50, 0xbb},
	{OV2640_REG_BD60, 0x9c},

	{OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_DSP},
	{0xe5, 0x7f},
	{OV2640_REG_MC_BIST,
		OV2640_REG_MC_BIST_RESET |
		OV2640_REG_MC_BIST_BOOT_ROM_SEL},
	{0x41, 0x24},
	{OV2640_REG_RESET,
		OV2640_REG_RESET_JPEG |
		OV2640_REG_RESET_DVP},
	{0x76, 0xff},
	{0x33, 0xa0},
	{0x42, 0x20},
	{0x43, 0x18},
	{0x4c, 0x00},
	{OV2640_REG_CTRL3,
		OV2640_REG_CTRL3_BPC_EN |
		OV2640_REG_CTRL3_WPC_EN |
		0x10},
	{0x88, 0x3f},
	{0xd7, 0x03},
	{0xd9, 0x10},
	{OV2640_REG_R_DVP_SP,
		OV2640_REG_R_DVP_SP_AUTO_MODE |
		0x2},
	{0xc8, 0x08},
	{0xc9, 0x80},
	{OV2640_REG_BPADDR, 0x00},
	{OV2640_REG_BPDATA, 0x00},
	{OV2640_REG_BPADDR, 0x03},
	{OV2640_REG_BPDATA, 0x48},
	{OV2640_REG_BPDATA, 0x48},
	{OV2640_REG_BPADDR, 0x08},
	{OV2640_REG_BPDATA, 0x20},
	{OV2640_REG_BPDATA, 0x10},
	{OV2640_REG_BPDATA, 0x0e},
	{0x90, 0x00},
	{0x91, 0x0e},
	{0x91, 0x1a},
	{0x91, 0x31},
	{0x91, 0x5a},
	{0x91, 0x69},
	{0x91, 0x75},
	{0x91, 0x7e},
	{0x91, 0x88},
	{0x91, 0x8f},
	{0x91, 0x96},
	{0x91, 0xa3},
	{0x91, 0xaf},
	{0x91, 0xc4},
	{0x91, 0xd7},
	{0x91, 0xe8},
	{0x91, 0x20},
	{0x92, 0x00},
	{0x93, 0x06},
	{0x93, 0xe3},
	{0x93, 0x03},
	{0x93, 0x03},
	{0x93, 0x00},
	{0x93, 0x02},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x96, 0x00},
	{0x97, 0x08},
	{0x97, 0x19},
	{0x97, 0x02},
	{0x97, 0x0c},
	{0x97, 0x24},
	{0x97, 0x30},
	{0x97, 0x28},
	{0x97, 0x26},
	{0x97, 0x02},
	{0x97, 0x98},
	{0x97, 0x80},
	{0x97, 0x00},
	{0x97, 0x00},
	{0xa4, 0x00},
	{0xa8, 0x00},
	{0xc5, 0x11},
	{0xc6, 0x51},
	{0xbf, 0x80},
	{0xc7, 0x10},
	{0xb6, 0x66},
	{0xb8, 0xA5},
	{0xb7, 0x64},
	{0xb9, 0x7C},
	{0xb3, 0xaf},
	{0xb4, 0x97},
	{0xb5, 0xFF},
	{0xb0, 0xC5},
	{0xb1, 0x94},
	{0xb2, 0x0f},
	{0xc4, 0x5c},
	{0xa6, 0x00},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x1b},
	{0xa7, 0x31},
	{0xa7, 0x00},
	{0xa7, 0x18},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x19},
	{0xa7, 0x31},
	{0xa7, 0x00},
	{0xa7, 0x18},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x19},
	{0xa7, 0x31},
	{0xa7, 0x00},
	{0xa7, 0x18},
	{0x7f, 0x00},
	{0xe5, 0x1f},
	{0xe1, 0x77},
	{0xdd, 0x7f},
	{OV2640_REG_CTRL0,
		OV2640_REG_CTRL0_YUV422 |
		OV2640_REG_CTRL0_YUV_EN |
		OV2640_REG_CTRL0_RGB_EN},
	{0x00, 0x00},
};

static const struct ov2640_reg uxga_regs[] = {
	{OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR},
	/* DSP input image resolution and window size control */
	{OV2640_REG_COM7, OV2640_REG_COM7_RES_UXGA},
	{OV2640_REG_COM1, 0x0F},        /* UXGA=0x0F, SVGA=0x0A, CIF=0x06 */
	{OV2640_REG_32, OV2640_REG_32_UXGA}, /* UXGA=0x36, SVGA/CIF=0x09 */

	{OV2640_REG_HSTART, 0x11}, /* UXGA=0x11, SVGA/CIF=0x11 */
	{OV2640_REG_HSTOP, 0x75},  /* UXGA=0x75, SVGA/CIF=0x43 */

	{OV2640_REG_VSTART, 0x01}, /* UXGA=0x01, SVGA/CIF=0x00 */
	{OV2640_REG_VSTOP, 0x97},  /* UXGA=0x97, SVGA/CIF=0x4b */
	{0x3d, 0x34},   /* UXGA=0x34, SVGA/CIF=0x38 */

	{0x35, 0x88},
	{0x22, 0x0a},
	{0x37, 0x40},
	{0x34, 0xa0},
	{0x06, 0x02},
	{0x0d, 0xb7},
	{0x0e, 0x01},
	{0x42, 0x83},

	/*
	 * Set DSP input image size and offset.
	 * The sensor output image can be scaled with OUTW/OUTH
	 */
	{OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_DSP},
	{OV2640_REG_R_BYPASS, OV2640_REG_R_BYPASS_DSP_BYPAS},

	{OV2640_REG_RESET, OV2640_REG_RESET_DVP},
	{OV2640_REG_HSIZE8, (UXGA_WIDTH >> 3)},  /* Image Horizontal Size OV2640_REG_HSIZE[10:3] */
	{OV2640_REG_VSIZE8, (UXGA_HEIGHT >> 3)}, /* Image Vertical Size OV2640_REG_VSIZE[10:3] */

	/* {OV2640_REG_HSIZE[11], HSIZE[2:0], OV2640_REG_VSIZE[2:0]} */
	{OV2640_REG_SIZEL,
		((UXGA_WIDTH >> 6) & 0x40) |
		((UXGA_WIDTH & 0x7) << 3) |
		(UXGA_HEIGHT & 0x7)},

	{OV2640_REG_XOFFL, 0x00},                       /* OFFSET_X[7:0] */
	{OV2640_REG_YOFFL, 0x00},                       /* OFFSET_Y[7:0] */
	{OV2640_REG_HSIZE, ((UXGA_WIDTH >> 2) & 0xFF)},  /* H_SIZE[7:0] real/4 */
	{OV2640_REG_VSIZE, ((UXGA_HEIGHT >> 2) & 0xFF)}, /* V_SIZE[7:0] real/4 */

	/* V_SIZE[8]/OFFSET_Y[10:8]/H_SIZE[8]/OFFSET_X[10:8] */
	{OV2640_REG_VHYX,
		((UXGA_HEIGHT >> 3) & 0x80) |
		((UXGA_WIDTH >> 7) & 0x08)},
	{OV2640_REG_TEST, (UXGA_WIDTH >> 4) & 0x80}, /* H_SIZE[9] */

	{OV2640_REG_CTRL2,
		OV2640_REG_CTRL2_DCW_EN |
		OV2640_REG_CTRL2_SDE_EN |
		OV2640_REG_CTRL2_UV_AVG_EN |
		OV2640_REG_CTRL2_CMX_EN |
		OV2640_REG_CTRL2_UV_ADJ_EN},

	/* H_DIVIDER/V_DIVIDER */
	{OV2640_REG_CTRLI,
		OV2640_REG_CTRLI_LP_DP |
		0x00},
	/* DVP prescaler */
	{OV2640_REG_R_DVP_SP,
		OV2640_REG_R_DVP_SP_AUTO_MODE |
		0x04},

	{OV2640_REG_R_BYPASS, OV2640_REG_R_BYPASS_DSP_EN},
	{OV2640_REG_RESET, 0x00},
	{0, 0},
};

#define NUM_BRIGHTNESS_LEVELS (5)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS + 1][5] = {
	{OV2640_REG_BPADDR,OV2640_REG_BPDATA,
	 OV2640_REG_BPADDR,OV2640_REG_BPDATA, OV2640_REG_BPDATA},
	{0x00, 0x04, 0x09, 0x00, 0x00},           /* -2 */
	{0x00, 0x04, 0x09, 0x10, 0x00},           /* -1 */
	{0x00, 0x04, 0x09, 0x20, 0x00},           /*  0 */
	{0x00, 0x04, 0x09, 0x30, 0x00},           /* +1 */
	{0x00, 0x04, 0x09, 0x40, 0x00},           /* +2 */
};

#define NUM_CONTRAST_LEVELS (5)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS + 1][7] = {
	{OV2640_REG_BPADDR, OV2640_REG_BPDATA,
	 OV2640_REG_BPADDR, OV2640_REG_BPDATA, OV2640_REG_BPDATA, OV2640_REG_BPDATA,
	 OV2640_REG_BPDATA},
	{0x00, 0x04, 0x07, 0x20, 0x18, 0x34, 0x06},               /* -2 */
	{0x00, 0x04, 0x07, 0x20, 0x1c, 0x2a, 0x06},               /* -1 */
	{0x00, 0x04, 0x07, 0x20, 0x20, 0x20, 0x06},               /*  0 */
	{0x00, 0x04, 0x07, 0x20, 0x24, 0x16, 0x06},               /* +1 */
	{0x00, 0x04, 0x07, 0x20, 0x28, 0x0c, 0x06},               /* +2 */
};

#define NUM_SATURATION_LEVELS (5)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS + 1][5] = {
	{OV2640_REG_BPADDR, OV2640_REG_BPDATA,
	 OV2640_REG_BPADDR, OV2640_REG_BPDATA, OV2640_REG_BPDATA},
	{0x00, 0x02, 0x03, 0x28, 0x28},           /* -2 */
	{0x00, 0x02, 0x03, 0x38, 0x38},           /* -1 */
	{0x00, 0x02, 0x03, 0x48, 0x48},           /*  0 */
	{0x00, 0x02, 0x03, 0x58, 0x58},           /* +1 */
	{0x00, 0x02, 0x03, 0x58, 0x58},           /* +2 */
};

struct ov2640_config {
	struct i2c_dt_spec i2c;
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	struct gpio_dt_spec reset_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios)
	struct gpio_dt_spec pwdn_gpio;
#endif
	uint8_t clock_rate_control;
};

struct ov2640_ctrls {
	struct video_ctrl hflip;
	struct video_ctrl vflip;
	struct video_ctrl ae;
	struct video_ctrl awb;
	struct video_ctrl gain;
	struct video_ctrl contrast;
	struct video_ctrl brightness;
	struct video_ctrl saturation;
	struct video_ctrl jpeg;
	struct video_ctrl test_pattern;
	struct video_ctrl link_freq;
};

struct ov2640_data {
	struct ov2640_ctrls ctrls;
	struct video_format fmt;
};

#define OV2640_VIDEO_FORMAT_CAP(width, height, format)                                             \
	{                                                                                          \
		.pixelformat = (format),                                                           \
		.width_min = (width),                                                              \
		.width_max = (width),                                                              \
		.height_min = (height),                                                            \
		.height_max = (height),                                                            \
		.width_step = 0,                                                                   \
		.height_step = 0,                                                                  \
	}

static const struct video_format_cap fmts[] = {
	OV2640_VIDEO_FORMAT_CAP(QQVGA_WIDTH, QQVGA_HEIGHT,
				VIDEO_PIX_FMT_RGB565), /* 160 x 120 QQVGA */
	OV2640_VIDEO_FORMAT_CAP(QCIF_WIDTH, QCIF_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 176 x 144 QCIF */
	OV2640_VIDEO_FORMAT_CAP(240, 240, VIDEO_PIX_FMT_RGB565),
	OV2640_VIDEO_FORMAT_CAP(QVGA_WIDTH, QVGA_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 320 x 240 QVGA */
	OV2640_VIDEO_FORMAT_CAP(CIF_WIDTH, CIF_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 352 x 288 CIF   */
	OV2640_VIDEO_FORMAT_CAP(VGA_WIDTH, VGA_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 640 x 480 VGA   */
	OV2640_VIDEO_FORMAT_CAP(SVGA_WIDTH, SVGA_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 800 x 600 SVGA */
	OV2640_VIDEO_FORMAT_CAP(XGA_WIDTH, XGA_HEIGHT, VIDEO_PIX_FMT_RGB565), /* 1024 x 768 XVGA  */
	OV2640_VIDEO_FORMAT_CAP(SXGA_WIDTH, SXGA_HEIGHT,
				VIDEO_PIX_FMT_RGB565), /* 1280 x 1024 SXGA  */
	OV2640_VIDEO_FORMAT_CAP(UXGA_WIDTH, UXGA_HEIGHT,
				VIDEO_PIX_FMT_RGB565), /* 1600 x 1200 UXGA  */
	OV2640_VIDEO_FORMAT_CAP(QQVGA_WIDTH, QQVGA_HEIGHT,
				VIDEO_PIX_FMT_JPEG),                          /* 160 x 120 QQVGA */
	OV2640_VIDEO_FORMAT_CAP(QCIF_WIDTH, QCIF_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 176 x 144 QCIF  */
	OV2640_VIDEO_FORMAT_CAP(CIF_WIDTH, CIF_HEIGHT, VIDEO_PIX_FMT_JPEG),   /* 352 x 288 CIF   */
	OV2640_VIDEO_FORMAT_CAP(240, 240, VIDEO_PIX_FMT_JPEG),
	OV2640_VIDEO_FORMAT_CAP(QVGA_WIDTH, QVGA_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 320 x 240 QVGA */
	OV2640_VIDEO_FORMAT_CAP(VGA_WIDTH, VGA_HEIGHT, VIDEO_PIX_FMT_JPEG),   /* 640 x 480 VGA   */
	OV2640_VIDEO_FORMAT_CAP(SVGA_WIDTH, SVGA_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 800 x 600 SVGA  */
	OV2640_VIDEO_FORMAT_CAP(XGA_WIDTH, XGA_HEIGHT, VIDEO_PIX_FMT_JPEG),   /* 1024 x 768 XVGA  */
	OV2640_VIDEO_FORMAT_CAP(SXGA_WIDTH, SXGA_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 1280 x 1024 SXGA */
	OV2640_VIDEO_FORMAT_CAP(UXGA_WIDTH, UXGA_HEIGHT, VIDEO_PIX_FMT_JPEG), /* 1600 x 1200 UXGA */
	{0}};

static int ov2640_write_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr, uint8_t value)
{
	int ret;

	/**
	 * It rarely happens that the camera does not respond with ACK signal.
	 * In that case it usually responds on 2nd try but there is a 3rd one
	 * just to be sure that the connection error is not caused by driver
	 * itself.
	 */
	for (int tries = 3; tries > 0; tries--) {
		ret = i2c_reg_write_byte_dt(spec, reg_addr, value);
		if (ret == 0) {
			return 0;
		}

		k_msleep(5);
	}

	LOG_ERR("failed to write 0x%x to 0x%x", value, reg_addr);

	return ret;
}

static int ov2640_read_reg(const struct i2c_dt_spec *spec, uint8_t reg_addr)
{
	uint8_t value;
	int ret;

	/**
	 * It rarely happens that the camera does not respond with ACK signal.
	 * In that case it usually responds on 2nd try but there is a 3rd one
	 * just to be sure that the connection error is not caused by driver
	 * itself.
	 */
	for (int tries = 3; tries > 0; tries--) {
		ret = i2c_reg_read_byte_dt(spec, reg_addr, &value);
		if (ret == 0) {
			return value;
		}

		k_msleep(5);
	}

	LOG_ERR("Failed to read register 0x%x", reg_addr);

	return ret;
}

static int ov2640_write_all(const struct device *dev, const struct ov2640_reg *regs,
			    uint16_t reg_num)
{
	const struct ov2640_config *cfg = dev->config;
	int ret;

	for (int i = 0; i < reg_num; i++) {
		ret = ov2640_write_reg(&cfg->i2c, regs[i].addr, regs[i].value);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int ov2640_soft_reset(const struct device *dev)
{
	const struct ov2640_config *cfg = dev->config;
	int ret;

	/* Switch to DSP register bank */
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}

	/* Initiate system reset */
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_COM7, OV2640_REG_COM7_SRST);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ov2640_set_level(const struct device *dev, int level, int max_level, int cols,
			    const uint8_t regs[][cols])
{
	const struct ov2640_config *cfg = dev->config;
	int ret;

	level += max_level / 2 + 1;

	/* Switch to DSP register bank */
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_DSP);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < (ARRAY_SIZE(regs[0]) / sizeof(regs[0][0])); i++) {
		ret = ov2640_write_reg(&cfg->i2c, regs[0][i], regs[level][i]);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int ov2640_set_output_format(const struct device *dev, int output_format)
{
	const struct ov2640_config *cfg = dev->config;
	int ret;

	/* Switch to DSP register bank */
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_DSP);
	if (ret < 0) {
		return ret;
	}

	if (output_format == VIDEO_PIX_FMT_JPEG) {
		/* Enable JPEG compression */
		ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_IMAGE_MODE, OV2640_REG_IMAGE_MODE_JPEG_EN);
		if (ret < 0) {
			return ret;
		}
	} else if (output_format == VIDEO_PIX_FMT_RGB565) {
		/* Disable JPEG compression and set output to RGB565 */
		ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_IMAGE_MODE, OV2640_REG_IMAGE_MODE_RGB565);
		if (ret < 0) {
			return ret;
		}
	} else {
		LOG_ERR("Image format not supported");
		return -ENOTSUP;
	}

	k_msleep(30);

	return ret;
}

static int ov2640_set_quality(const struct device *dev, int qs)
{
	const struct ov2640_config *cfg = dev->config;
	int ret;

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_DSP);
	if (ret < 0) {
		return ret;
	}

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_QS, qs);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ov2640_set_colorbar(const struct device *dev, uint8_t enable)
{
	const struct ov2640_config *cfg = dev->config;
	uint8_t reg;
	int ret;

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}

	reg = ov2640_read_reg(&cfg->i2c, OV2640_REG_COM7);
	if (enable) {
		reg |= OV2640_REG_COM7_COLOR_BAR;
	} else {
		reg &= ~OV2640_REG_COM7_COLOR_BAR;
	}

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_COM7, reg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ov2640_set_white_bal(const struct device *dev, int enable)
{
	const struct ov2640_config *cfg = dev->config;
	uint8_t reg;
	int ret;

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}

	reg = ov2640_read_reg(&cfg->i2c, OV2640_REG_CTRL1);
	if (enable) {
		reg |= OV2640_REG_CTRL1_AWB;
	} else {
		reg &= ~OV2640_REG_CTRL1_AWB;
	}
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_CTRL1, reg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ov2640_set_gain_ctrl(const struct device *dev, int enable)
{
	const struct ov2640_config *cfg = dev->config;
	uint8_t reg;
	int ret;

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}

	reg = ov2640_read_reg(&cfg->i2c, OV2640_REG_COM8);
	if (enable) {
		reg |= OV2640_REG_COM8_AGC_EN;
	} else {
		reg &= ~OV2640_REG_COM8_AGC_EN;
	}

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_COM8, reg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ov2640_set_exposure_ctrl(const struct device *dev, int enable)
{
	const struct ov2640_config *cfg = dev->config;
	int ret;

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_COM8, enable ? OV2640_REG_COM8_AEC_EN : 0);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ov2640_set_horizontal_mirror(const struct device *dev, int enable)
{
	const struct ov2640_config *cfg = dev->config;
	uint8_t reg;
	int ret;

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}

	reg = ov2640_read_reg(&cfg->i2c, OV2640_REG_04);
	if (enable) {
		reg |= OV2640_REG_04_HFLIP_IMG;
	} else {
		reg &= ~OV2640_REG_04_HFLIP_IMG;
	}

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_04, reg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ov2640_set_vertical_flip(const struct device *dev, int enable)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint8_t reg;

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}

	reg = ov2640_read_reg(&cfg->i2c, OV2640_REG_04);
	if (enable) {
		reg |= OV2640_REG_04_VFLIP_IMG | OV2640_REG_04_VREF_EN;
	} else {
		reg &= ~(OV2640_REG_04_VFLIP_IMG | OV2640_REG_04_VREF_EN);
	}

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_04, reg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static const struct ov2640_win_size *ov2640_select_win(uint32_t width, uint32_t height)
{
	for (int i = 0; i < ARRAY_SIZE(ov2640_supported_win_sizes); i++) {
		if (ov2640_supported_win_sizes[i].width == width &&
		    ov2640_supported_win_sizes[i].height == height) {
			return &ov2640_supported_win_sizes[i];
		}
	}

	return NULL;
}

static int ov2640_set_resolution(const struct device *dev, uint16_t img_width, uint16_t img_height)
{
	int ret = 0;
	const struct ov2640_config *cfg = dev->config;

	uint16_t w = img_width;
	uint16_t h = img_height;

	const struct ov2640_win_size *win = ov2640_select_win(w, h);

	if (win == NULL) {
		LOG_ERR("Couldn't find window size for desired resolution setting");
		return -EINVAL;
	}
	LOG_DBG("Selected resolution %ux%u", win->width, win->height);

	/* Write DSP input registers */
	ret = ov2640_write_all(dev, uxga_regs, ARRAY_SIZE(uxga_regs));
	if (ret < 0) {
		return ret;
	}

	/* Disable DSP */
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_DSP);
	if (ret < 0) {
		return ret;
	}
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_R_BYPASS, OV2640_REG_R_BYPASS_DSP_BYPAS);
	if (ret < 0) {
		return ret;
	}

	/* Select window size */
	ret = ov2640_write_all(dev, win->regs, win->regs_size);
	if (ret < 0) {
		return ret;
	}

	/* Set OV2640_REG_CLKRC */
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_CLKRC, cfg->clock_rate_control);
	if (ret < 0) {
		return ret;
	}

	/* Enable DSP */
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_DSP);
	if (ret < 0) {
		return ret;
	}
	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_R_BYPASS, OV2640_REG_R_BYPASS_DSP_EN);
	if (ret < 0) {
		return ret;
	}
	k_msleep(30);

	return 0;
}

uint8_t ov2640_check_connection(const struct device *dev)
{
	const struct ov2640_config *cfg = dev->config;
	int ret;

	uint8_t reg_pid_val, reg_ver_val;

	ret = ov2640_write_reg(&cfg->i2c, OV2640_REG_BANK_SEL, OV2640_REG_BANK_SEL_SENSOR);
	if (ret < 0) {
		return ret;
	}

	reg_pid_val = ov2640_read_reg(&cfg->i2c, OV2640_REG__PID);
	reg_ver_val = ov2640_read_reg(&cfg->i2c, OV2640_REG__VER);

	if (OV2640_REG__PID_VAL != reg_pid_val || OV2640_REG__VER_VAL != reg_ver_val) {
		LOG_ERR("OV2640 not detected, expected 0x%02x%02x got 0x%02x%02x",
			OV2640_REG__PID_VAL, OV2640_REG__VER_VAL, reg_pid_val, reg_ver_val);
		return -ENODEV;
	}

	return 0;
}

static int ov2640_set_fmt(const struct device *dev, struct video_format *fmt)
{
	struct ov2640_data *drv_data = dev->data;
	uint16_t width, height;
	int ret = 0;
	int i = 0;

	/* We only support RGB565 and JPEG pixel formats */
	if (fmt->pixelformat != VIDEO_PIX_FMT_RGB565 && fmt->pixelformat != VIDEO_PIX_FMT_JPEG) {
		LOG_ERR("ov2640 camera supports only RGB565 and JPG pixelformats!");
		return -ENOTSUP;
	}

	width = fmt->width;
	height = fmt->height;

	if (memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt)) == 0) {
		/* nothing to do */
		return 0;
	}

	drv_data->fmt = *fmt;

	ret = ov2640_set_output_format(dev, fmt->pixelformat);
	if (ret < 0) {
		return ret;
	}

	/* Check if camera is capable of handling given format */
	while (fmts[i].pixelformat) {
		if (fmts[i].width_min == width && fmts[i].height_min == height &&
		    fmts[i].pixelformat == fmt->pixelformat) {
			/* Set window size */
			ret = ov2640_set_resolution(dev, fmt->width, fmt->height);
			if (ret < 0) {
				return ret;
			}

			return 0;
		}
		i++;
	}

	/* Camera is not capable of handling given format */
	LOG_ERR("Image format not supported\n");
	return -ENOTSUP;
}

static int ov2640_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct ov2640_data *drv_data = dev->data;

	*fmt = drv_data->fmt;

	return 0;
}

static int ov2640_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
	return 0;
}

static int ov2640_get_caps(const struct device *dev, struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int ov2640_set_ctrl(const struct device *dev, uint32_t id)
{
	struct ov2640_data *drv_data = dev->data;
	struct ov2640_ctrls *ctrls = &drv_data->ctrls;

	switch (id) {
	case VIDEO_CID_HFLIP:
		return ov2640_set_horizontal_mirror(dev, ctrls->hflip.val);
	case VIDEO_CID_VFLIP:
		return ov2640_set_vertical_flip(dev, ctrls->vflip.val);
	case VIDEO_CID_EXPOSURE:
		return ov2640_set_exposure_ctrl(dev, ctrls->ae.val);
	case VIDEO_CID_WHITE_BALANCE_TEMPERATURE:
		return ov2640_set_white_bal(dev, ctrls->awb.val);
	case VIDEO_CID_GAIN:
		return ov2640_set_gain_ctrl(dev, ctrls->gain.val);
	case VIDEO_CID_BRIGHTNESS:
		return ov2640_set_level(dev, ctrls->brightness.val, NUM_BRIGHTNESS_LEVELS,
					ARRAY_SIZE(brightness_regs[0]), brightness_regs);
	case VIDEO_CID_CONTRAST:
		return ov2640_set_level(dev, ctrls->contrast.val, NUM_CONTRAST_LEVELS,
					ARRAY_SIZE(contrast_regs[0]), contrast_regs);
	case VIDEO_CID_SATURATION:
		return ov2640_set_level(dev, ctrls->saturation.val, NUM_SATURATION_LEVELS,
					ARRAY_SIZE(saturation_regs[0]), saturation_regs);
	case VIDEO_CID_JPEG_COMPRESSION_QUALITY:
		return ov2640_set_quality(dev, ctrls->jpeg.val);
	case VIDEO_CID_TEST_PATTERN:
		return ov2640_set_colorbar(dev, ctrls->test_pattern.val);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(video, ov2640_driver_api) = {
	.set_format = ov2640_set_fmt,
	.get_format = ov2640_get_fmt,
	.get_caps = ov2640_get_caps,
	.set_stream = ov2640_set_stream,
	.set_ctrl = ov2640_set_ctrl,
};

static int ov2640_init_controls(const struct device *dev)
{
	struct ov2640_data *drv_data = dev->data;
	struct ov2640_ctrls *ctrls = &drv_data->ctrls;
	int ret;

	ret = video_init_ctrl(&ctrls->hflip, dev, VIDEO_CID_HFLIP,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 0});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->vflip, dev, VIDEO_CID_VFLIP,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 0});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->ae, dev, VIDEO_CID_EXPOSURE,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 1});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->awb, dev, VIDEO_CID_WHITE_BALANCE_TEMPERATURE,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 1});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->gain, dev, VIDEO_CID_GAIN,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 1});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->brightness, dev, VIDEO_CID_BRIGHTNESS,
			      (struct video_ctrl_range){.min = -2, .max = 2, .step = 1, .def = 0});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->contrast, dev, VIDEO_CID_CONTRAST,
			      (struct video_ctrl_range){.min = -2, .max = 2, .step = 1, .def = 0});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->saturation, dev, VIDEO_CID_SATURATION,
			      (struct video_ctrl_range){.min = -2, .max = 2, .step = 1, .def = 0});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(
		&ctrls->jpeg, dev, VIDEO_CID_JPEG_COMPRESSION_QUALITY,
		(struct video_ctrl_range){.min = 5, .max = 100, .step = 1, .def = 50});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_ctrl(&ctrls->test_pattern, dev, VIDEO_CID_TEST_PATTERN,
			      (struct video_ctrl_range){.min = 0, .max = 1, .step = 1, .def = 0});
	if (ret < 0) {
		return ret;
	}

	ret = video_init_int_menu_ctrl(&ctrls->link_freq, dev, VIDEO_CID_LINK_FREQ,
				       0, ov2640_link_freq, ARRAY_SIZE(ov2640_link_freq));
	if (ret < 0) {
		return ret;
	}
	ctrls->link_freq.flags |= VIDEO_CTRL_FLAG_READ_ONLY;

	return 0;
}

static int ov2640_init(const struct device *dev)
{

#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios) || DT_INST_NODE_HAS_PROP(0, reset_gpios)
	const struct ov2640_config *cfg = dev->config;
#endif

	/* set default/init format SVGA RGB565 */
	struct video_format fmt = {
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width = SVGA_WIDTH,
		.height = SVGA_HEIGHT,
	};
	int ret;

#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios)
	ret = gpio_pin_configure_dt(&cfg->pwdn_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	k_sleep(K_MSEC(1));
#endif

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&cfg->reset_gpio, 0);
	k_sleep(K_MSEC(1));
#endif

	ret = ov2640_check_connection(dev);
	if (ret < 0) {
		return ret;
	}

	ov2640_soft_reset(dev);
	k_msleep(300);

	ov2640_write_all(dev, default_regs, ARRAY_SIZE(default_regs));

	ret = ov2640_set_fmt(dev, &fmt);
	if (ret < 0) {
		LOG_ERR("Unable to configure default format");
		return ret;
	}

	ret = ov2640_set_exposure_ctrl(dev, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ov2640_set_white_bal(dev, 1);
	if (ret < 0) {
		return ret;
	}

	/* Initialize controls */
	return ov2640_init_controls(dev);
}

/* Unique Instance */
static const struct ov2640_config ov2640_cfg_0 = {
	.i2c = I2C_DT_SPEC_INST_GET(0),
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	.reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#endif
#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios)
	.pwdn_gpio = GPIO_DT_SPEC_INST_GET(0, pwdn_gpios),
#endif
	.clock_rate_control = DT_INST_PROP(0, clock_rate_control),
};
static struct ov2640_data ov2640_data_0;

static int ov2640_init_0(const struct device *dev)
{
	const struct ov2640_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
		LOG_ERR("%s: device %s is not ready", dev->name, cfg->reset_gpio.port->name);
		return -ENODEV;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, pwdn_gpios)
	if (!gpio_is_ready_dt(&cfg->pwdn_gpio)) {
		LOG_ERR("%s: device %s is not ready", dev->name, cfg->pwdn_gpio.port->name);
		return -ENODEV;
	}
#endif

	if (i2c_configure(cfg->i2c.bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD))) {
		LOG_ERR("Failed to configure ov2640 i2c interface.");
	}

	return ov2640_init(dev);
}

DEVICE_DT_INST_DEFINE(0, &ov2640_init_0, NULL, &ov2640_data_0, &ov2640_cfg_0, POST_KERNEL,
		      CONFIG_VIDEO_INIT_PRIORITY, &ov2640_driver_api);

VIDEO_DEVICE_DEFINE(ov2640, DEVICE_DT_INST_GET(0), NULL);
