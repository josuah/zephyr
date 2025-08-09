/*
 * Copyright (c) 2023,2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usbh.h>

#include "usbh_class.h"
#include "usbh_host.h"

static int usbh_class_append(struct usbh_context *const uhs_ctx,
			     ,
			     const enum usbd_speed speed)
{
	;
}

int usbh_register_class(struct usbh_context *const uhs_ctx,
			const char *name,
			const enum usbd_speed speed)
{
	struct usbh_class_node *c_nd;
	struct usbh_class_data *c_data;
	int ret;

	c_nd = usbh_class_node_get(name, speed);
	if (c_nd == NULL) {
		return -ENODEV;
	}

	usbh_host_lock(uhs_ctx);

	if (usbh_is_initialized(uds_ctx)) {
		LOG_ERR("USB device support is initialized");
		ret = -EBUSY;
		goto register_class_error;
	}

	ret = usbh_class_append(uhs_ctx, c_nd, speed);
	if (ret == 0) {
		/* Initialize pointer back to the host struct */
		atomic_set_bit(&c_nd->state, USBH_CCTX_REGISTERED);
		c_data->uhs_ctx = uhs_ctx;
	}

class_error:
	usbh_host_unlock(uhs_ctx);
	return ret;
}

LOG_MODULE_REGISTER(usbh_class, CONFIG_USBH_LOG_LEVEL);
