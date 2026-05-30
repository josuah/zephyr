/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>

#include "usbh_device.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbh_ch9, CONFIG_USBH_LOG_LEVEL);

/*
 * For now we set it to the upper limit defined in Chapter
 * "9.2.6.4 Standard Device Requests"
 * This will need to be revised and set depending on the request.
 */
#define SETUP_REQ_TIMEOUT	5000U

K_SEM_DEFINE(ch9_req_sync, 0, 1);
static bool ctrl_req_no_status;

static int ch9_req_cb(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	LOG_DBG("Request finished %p, err %d", xfer, xfer->err);
	if (xfer->err == -ECONNRESET) {
		LOG_INF("Transfer %p cancelled", (void *)xfer);
		usbh_xfer_free(udev, xfer);

		return 0;
	}

	k_sem_give(&ch9_req_sync);

	return 0;
}

void usbh_req_omit_status(const bool omit)
{
	ctrl_req_no_status = omit;
}

int usbh_req_setup(struct usb_device *const udev,
		   const uint8_t bmRequestType,
		   const uint8_t bRequest,
		   const uint16_t wValue,
		   const uint16_t wIndex,
		   const uint16_t wLength,
		   void *data)
{
	struct usb_setup_packet req = {
		.bmRequestType = bmRequestType,
		.bRequest = bRequest,
		.wValue = sys_cpu_to_le16(wValue),
		.wIndex = sys_cpu_to_le16(wIndex),
		.wLength = sys_cpu_to_le16(wLength),
	};
	struct uhc_transfer *xfer;
	struct usbh_context *const ctx = udev->ctx;
	uint8_t ep = usb_reqtype_is_to_device(&req) ? 0x00 : 0x80;
	int ret;

	xfer = uhc_xfer_alloc_with_buf(ctx->dev, ep, udev, ch9_req_cb, NULL, wLength);
	if (!xfer) {
		LOG_ERR("Failed to allocate the UHC transfer with a buffer, wLength: %d", wLength);
		return -ENOMEM;
	}

	memcpy(xfer->setup_pkt, &req, sizeof(req));

	__ASSERT(IS_ENABLED(CONFIG_ZTEST) ||
		 (xfer->buf != NULL && wLength) || (xfer->buf == NULL && !wLength),
		 "Unresolved conditions for data stage");

	if (usb_reqtype_is_to_device(&req) && data != NULL) {
		net_buf_add_mem(xfer->buf, data, MIN(xfer->buf->size, wLength));
	}

	if (IS_ENABLED(CONFIG_ZTEST) && ctrl_req_no_status) {
		xfer->no_status = true;
	}

	ret = usbh_xfer_enqueue(udev, xfer);
	if (ret) {
		LOG_ERR("Failed to enqueue buffer");
		goto xfer_alloc_err;
	}

	if (k_sem_take(&ch9_req_sync, K_MSEC(SETUP_REQ_TIMEOUT)) != 0) {
		ret = usbh_xfer_dequeue(udev, xfer);
		if (ret != 0) {
			LOG_ERR("Failed to cancel transfer");
			goto xfer_alloc_err;
		}

		LOG_ERR("Timeout");
		ret = -ETIMEDOUT;
		goto xfer_alloc_err;
	}

	if (xfer->err != 0) {
		ret = xfer->err;
		goto xfer_alloc_err;
	}

	if (xfer->buf->len != wLength) {
		LOG_WRN("Expected %u bytes, got %zu bytes", wLength, xfer->buf->len);
	}

	if (usb_reqtype_is_to_host(&req) && data != NULL) {
		memcpy(data, xfer->buf->data, MIN(xfer->buf->size, wLength));
	}

xfer_alloc_err:
	net_buf_unref(xfer->buf);
	usbh_xfer_free(udev, xfer);

	return ret;
}

int usbh_req_desc(struct usb_device *const udev,
		  const uint8_t type, const uint8_t index,
		  const uint16_t id,
		  const uint16_t len,
		  void *data)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_HOST << 7;
	const uint8_t bRequest = USB_SREQ_GET_DESCRIPTOR;
	const uint16_t wValue = (type << 8) | index;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, wValue, id, len,
			      data);
}

int usbh_req_desc_dev(struct usb_device *const udev,
		      const uint16_t len,
		      struct usb_device_descriptor *const desc)
{
	const uint8_t type = USB_DESC_DEVICE;
	const uint16_t wLength = MIN(len, sizeof(struct usb_device_descriptor));
	int ret;

	ret = usbh_req_desc(udev, type, 0, 0, wLength, desc);
	if (ret == 0) {
		desc->bcdUSB = sys_le16_to_cpu(desc->bcdUSB);
		desc->idVendor = sys_le16_to_cpu(desc->idVendor);
		desc->idProduct = sys_le16_to_cpu(desc->idProduct);
		desc->bcdDevice = sys_le16_to_cpu(desc->bcdDevice);
	}

	return ret;
}

int usbh_req_desc_cfg(struct usb_device *const udev,
		      const uint8_t index,
		      const uint16_t len,
		      struct usb_cfg_descriptor *const desc)
{
	const uint8_t type = USB_DESC_CONFIGURATION;
	const uint16_t wLength = len;
	int ret;

	ret = usbh_req_desc(udev, type, index, 0, wLength, desc);
	if (ret != 0) {
		return ret;
	}

	desc->wTotalLength = sys_le16_to_cpu(desc->wTotalLength);

	return 0;
}

int usbh_req_set_address(struct usb_device *const udev,
			 const uint8_t addr)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_DEVICE << 7;
	const uint8_t bRequest = USB_SREQ_SET_ADDRESS;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, addr, 0, 0,
			      NULL);
}

int usbh_req_set_cfg(struct usb_device *const udev,
		     const uint8_t cfg)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_DEVICE << 7;
	const uint8_t bRequest = USB_SREQ_SET_CONFIGURATION;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, cfg, 0, 0,
			      NULL);
}

int usbh_req_get_cfg(struct usb_device *const udev,
		     uint8_t *const cfg)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_HOST << 7;
	const uint8_t bRequest = USB_SREQ_GET_CONFIGURATION;
	const uint16_t wLength = 1;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, 0, 0, wLength,
			      cfg);
}

int usbh_req_set_alt(struct usb_device *const udev,
		     const uint8_t iface, const uint8_t alt)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_DEVICE << 7 |
				      USB_REQTYPE_RECIPIENT_INTERFACE;
	const uint8_t bRequest = USB_SREQ_SET_INTERFACE;
	const uint16_t wValue = alt;
	const uint16_t wIndex = iface;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, wValue, wIndex, 0,
			      NULL);
}

int usbh_req_set_sfs_rwup(struct usb_device *const udev)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_DEVICE << 7;
	const uint8_t bRequest = USB_SREQ_SET_FEATURE;
	const uint16_t wValue = USB_SFS_REMOTE_WAKEUP;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, wValue, 0, 0,
			      NULL);
}

int usbh_req_clear_sfs_rwup(struct usb_device *const udev)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_DEVICE << 7;
	const uint8_t bRequest = USB_SREQ_CLEAR_FEATURE;
	const uint16_t wValue = USB_SFS_REMOTE_WAKEUP;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, wValue, 0, 0,
			      NULL);
}

int usbh_req_set_sfs_halt(struct usb_device *const udev, const uint8_t ep)
{
	const uint8_t bmRequestType = USB_REQTYPE_RECIPIENT_ENDPOINT;
	const uint8_t bRequest = USB_SREQ_SET_FEATURE;
	const uint16_t wValue = USB_SFS_ENDPOINT_HALT;
	const uint16_t wIndex = ep;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, wValue, wIndex, 0,
			      NULL);
}

int usbh_req_clear_sfs_halt(struct usb_device *const udev, const uint8_t ep)
{
	const uint8_t bmRequestType = USB_REQTYPE_RECIPIENT_ENDPOINT;
	const uint8_t bRequest = USB_SREQ_CLEAR_FEATURE;
	const uint16_t wValue = USB_SFS_ENDPOINT_HALT;
	const uint16_t wIndex = ep;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, wValue, wIndex, 0,
			      NULL);
}

int usbh_req_set_hcfs_ppwr(struct usb_device *const udev,
			   const uint8_t port)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_DEVICE << 7 |
				      USB_REQTYPE_TYPE_CLASS << 5 |
				      USB_REQTYPE_RECIPIENT_OTHER << 0;
	const uint8_t bRequest = USB_HCREQ_SET_FEATURE;
	const uint16_t wValue = USB_HCFS_PORT_POWER;
	const uint16_t wIndex = port;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, wValue, wIndex, 0,
			      NULL);
}

int usbh_req_set_hcfs_prst(struct usb_device *const udev,
			   const uint8_t port)
{
	const uint8_t bmRequestType = USB_REQTYPE_DIR_TO_DEVICE << 7 |
				      USB_REQTYPE_TYPE_CLASS << 5 |
				      USB_REQTYPE_RECIPIENT_OTHER << 0;
	const uint8_t bRequest = USB_HCREQ_SET_FEATURE;
	const uint16_t wValue = USB_HCFS_PORT_RESET;
	const uint16_t wIndex = port;

	return usbh_req_setup(udev,
			      bmRequestType, bRequest, wValue, wIndex, 0,
			      NULL);
}
