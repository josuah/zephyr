/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_USB_COMMON_UVC_H
#define ZEPHYR_INCLUDE_USB_COMMON_UVC_H

#include <zephyr/usb/usb_ch9.h>

enum uvc_control_type {
	UVC_CONTROL_SIGNED,
	UVC_CONTROL_UNSIGNED,
};

/* Mapping between UVC controls and Video controls */
struct uvc_control_map {
	/* Video CID to use for this control */
	uint32_t cid;
	/* Size to write out */
	uint8_t size;
	/* Bit position in the UVC control */
	uint8_t bit;
	/* UVC selector identifying this control */
	uint8_t selector;
	/* Whether the UVC value is signed, always false for bitmaps and boolean */
	enum uvc_control_type type;
};

extern const struct uvc_control_map uvc_control_map_ct[];
extern const size_t uvc_control_map_ct_len;
extern const struct uvc_control_map uvc_control_map_pu[];
extern const size_t uvc_control_map_pu_len;
extern const struct uvc_control_map uvc_control_map_su[];
extern const size_t uvc_control_map_su_len;
extern const struct uvc_control_map uvc_control_map_xu[];
extern const size_t uvc_control_map_xu_len;

/*
 * Get the UVC GUID code from the FourCC format code
 */
void uvc_fourcc_to_guid(uint8_t guid[16], const uint32_t fourcc);

/*
 * Get the FourCC format code from the UVC GUID code
 */
uint32_t uvc_guid_to_fourcc(const uint8_t guid[16]);

#endif /* ZEPHYR_INCLUDE_USB_COMMON_CLASS_UVC_H */
