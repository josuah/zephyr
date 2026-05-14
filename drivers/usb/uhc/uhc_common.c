/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_ch9.h>
#include "uhc_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc, CONFIG_UHC_DRIVER_LOG_LEVEL);

K_MEM_SLAB_DEFINE_STATIC_TYPE(uhc_xfer_pool, struct uhc_transfer,
			      CONFIG_UHC_XFER_COUNT);

USB_BUF_POOL_VAR_DEFINE(uhc_ep_pool,
			CONFIG_UHC_BUF_COUNT, CONFIG_UHC_BUF_POOL_SIZE,
			0, NULL);

int uhc_submit_event(const struct device *dev,
		     const enum uhc_event_type type,
		     const int status)
{
	struct uhc_data *data = dev->data;
	struct uhc_event drv_evt = {
		.type = type,
		.status = status,
		.dev = dev,
	};

	if (!uhc_is_initialized(dev)) {
		return -EPERM;
	}

	return data->event_cb(dev, &drv_evt);
}

void uhc_xfer_return(const struct device *dev,
		     struct uhc_transfer *const xfer,
		     const int err)
{
	struct uhc_data *data = dev->data;
	struct uhc_event drv_evt = {
		.type = UHC_EVT_EP_REQUEST,
		.xfer = xfer,
		.dev = dev,
	};

	sys_dlist_remove(&xfer->node);
	xfer->queued = 0;
	xfer->err = err;

	data->event_cb(dev, &drv_evt);
}

/* a < b */
bool uhc_xfer_seq_lt(uint16_t a, uint16_t b, uint16_t max)
{
	if (a < b) {
		/* (b) may be larger, but if too much distance, assume overflow */
		return b - a < max / 2;
	}
	if (a > b) {
		/* (a) may be larger, but if too much distance, assume overflow */
		return a - b > max / 2;
	}
	return false;
}

/* a <= b */
bool uhc_xfer_seq_le(uint16_t a, uint16_t b, uint16_t max)
{
	return !uhc_xfer_seq_lt(b, a, max);
}

static void xfer_schedule_periodic(const struct device *dev,
				  struct uhc_transfer *const xfer,
				  const uint16_t cur_frame,
				  const uint16_t max_frame)
{
	struct uhc_data *data = dev->data;
	struct uhc_transfer *curr;

	xfer->start_frame = cur_frame;

	/* Search for transfers with same address and endpoint */
	SYS_DLIST_FOR_EACH_CONTAINER(&data->periodic_xfers, curr, node) {
		if (xfer->udev->addr == curr->udev->addr &&
		    xfer->ep == curr->ep &&
		    uhc_xfer_seq_lt(xfer->start_frame, curr->start_frame, max_frame)) {
			/* Schedule it on the next interval */
			xfer->start_frame = curr->start_frame;
		}
	}

	xfer->start_frame++;
	xfer->start_frame = ROUND_UP(xfer->start_frame, 1U << (xfer->interval - 1));
	xfer->start_frame %= (uint32_t)max_frame + 1;

	SYS_DLIST_FOR_EACH_CONTAINER(&data->periodic_xfers, curr, node) {
		if (uhc_xfer_seq_lt(curr->start_frame, xfer->start_frame, max_frame)) {
			continue;
		}
		sys_dlist_insert(&curr->node, &xfer->node);
		return;
	}

	sys_dlist_append(&data->periodic_xfers, &xfer->node);
}

void uhc_xfer_set_in_progress(const struct device *const dev, struct uhc_transfer *const xfer)
{
	struct uhc_data *data = dev->data;

	sys_dlist_remove(&xfer->node);
	sys_dlist_append(&data->active_xfers, &xfer->node);
}

struct uhc_transfer *uhc_xfer_get_periodic(const struct device *const dev,
					   const uint16_t cur_frame,
					   const uint16_t max_frame)
{
	struct uhc_data *data = dev->data;
	struct uhc_transfer *xfer;

	xfer = SYS_DLIST_PEEK_HEAD_CONTAINER(&data->periodic_xfers, xfer, node);
	if (xfer == NULL || uhc_xfer_seq_lt(cur_frame, xfer->start_frame, max_frame)) {
		return NULL;
	}

	uhc_xfer_set_in_progress(dev, xfer);

	return xfer;
}

struct uhc_transfer *uhc_xfer_get_non_periodic(const struct device *dev)
{
	struct uhc_data *data = dev->data;
	struct uhc_transfer *xfer;

	xfer = SYS_DLIST_PEEK_HEAD_CONTAINER(&data->ctrl_xfers, xfer, node);
	if (xfer != NULL) {
		goto found;
	}

	xfer = SYS_DLIST_PEEK_HEAD_CONTAINER(&data->bulk_xfers, xfer, node);
	if (xfer != NULL) {
		goto found;
	}

	return NULL;

found:
	uhc_xfer_set_in_progress(dev, xfer);

	return xfer;
}

void uhc_xfer_append(const struct device *dev,
		     struct uhc_transfer *const xfer,
		     uint16_t cur_frame, uint16_t max_frame)
{
	struct uhc_data *data = dev->data;

	switch (xfer->type) {
	case USB_EP_TYPE_CONTROL:
		sys_dlist_append(&data->ctrl_xfers, &xfer->node);
		break;
	case USB_EP_TYPE_BULK:
		sys_dlist_append(&data->bulk_xfers, &xfer->node);
		break;
	case USB_EP_TYPE_ISO:
	case USB_EP_TYPE_INTERRUPT:
		xfer_schedule_periodic(dev, xfer, cur_frame, max_frame);
		break;
	default:
		LOG_ERR("Invalid xfer type: %d", xfer->type);
	}
}

struct net_buf *uhc_xfer_buf_alloc(const struct device *dev,
				   const size_t size,
				   uint16_t mps)
{
	return net_buf_alloc_len(&uhc_ep_pool, ROUND_UP(size, USB_MPS_TO_TPL(mps)), K_NO_WAIT);
}

void uhc_xfer_buf_free(const struct device *dev, struct net_buf *const buf)
{
	net_buf_unref(buf);
}

struct uhc_transfer *uhc_xfer_alloc(const struct device *dev,
				    const uint8_t ep,
				    struct usb_device *const udev,
				    void *const cb,
				    void *const cb_priv)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep) & 0xF;
	const struct uhc_api *api = dev->api;
	struct uhc_transfer *xfer = NULL;
	uint16_t mps;
	uint16_t interval;
	uint8_t type;

	api->lock(dev);

	if (!uhc_is_initialized(dev)) {
		goto xfer_alloc_error;
	}

	if (ep_idx == 0) {
		interval = 0;
		type = USB_EP_TYPE_CONTROL;
		mps = udev->dev_desc.bMaxPacketSize0;
	} else {
		struct usb_ep_descriptor *ep_desc;

		if (USB_EP_DIR_IS_IN(ep)) {
			ep_desc = udev->ep_in[ep_idx].desc;
		} else {
			ep_desc = udev->ep_out[ep_idx].desc;
		}

		if (ep_desc == NULL) {
			LOG_ERR("Endpoint 0x%02x is not configured", ep);
			goto xfer_alloc_error;
		}

		mps = ep_desc->wMaxPacketSize;
		interval = ep_desc->bInterval;
		type = ep_desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK;
	}

	LOG_DBG("Allocate xfer, ep 0x%02x mps %u cb %p", ep, mps, cb);

	if (k_mem_slab_alloc(&uhc_xfer_pool, (void **)&xfer, K_NO_WAIT)) {
		LOG_ERR("Failed to allocate transfer");
		goto xfer_alloc_error;
	}

	memset(xfer, 0, sizeof(struct uhc_transfer));
	xfer->ep = ep;
	xfer->mps = mps;
	xfer->interval = interval;
	xfer->type = type;
	xfer->udev = udev;
	xfer->cb = cb;
	xfer->priv = cb_priv;

xfer_alloc_error:
	api->unlock(dev);

	return xfer;
}

struct uhc_transfer *uhc_xfer_alloc_with_buf(const struct device *dev,
					     const uint8_t ep,
					     struct usb_device *const udev,
					     void *const cb,
					     void *const cb_priv,
					     size_t size)
{
	struct uhc_transfer *xfer;

	xfer = uhc_xfer_alloc(dev, ep, udev, cb, cb_priv);
	if (xfer == NULL) {
		return NULL;
	}

	xfer->buf = uhc_xfer_buf_alloc(dev, size, xfer->mps);
	if (xfer->buf == NULL) {
		uhc_xfer_free(dev, xfer);
		return NULL;
	}

	return xfer;
}

int uhc_xfer_free(const struct device *dev, struct uhc_transfer *const xfer)
{
	const struct uhc_api *api = dev->api;
	int ret = 0;

	api->lock(dev);

	if (xfer->queued) {
		ret = -EBUSY;
		LOG_ERR("Transfer is still queued");
		goto xfer_free_error;
	}

	k_mem_slab_free(&uhc_xfer_pool, (void *)xfer);

xfer_free_error:
	api->unlock(dev);

	return ret;
}

int uhc_xfer_buf_add(const struct device *dev,
		     struct uhc_transfer *const xfer,
		     struct net_buf *buf)
{
	const struct uhc_api *api = dev->api;
	int ret = 0;

	api->lock(dev);
	if (xfer->queued) {
		ret = -EBUSY;
	} else {
		xfer->buf = buf;
	}

	api->unlock(dev);

	return ret;
}

int uhc_ep_enqueue(const struct device *dev, struct uhc_transfer *const xfer)
{
	const struct uhc_api *api = dev->api;
	int ret;

	api->lock(dev);

	if (!uhc_is_initialized(dev)) {
		ret = -EPERM;
		goto ep_enqueue_error;
	}

	xfer->queued = 1;
	ret = api->ep_enqueue(dev, xfer);
	if (ret) {
		xfer->queued = 0;
	}


ep_enqueue_error:
	api->unlock(dev);

	return ret;
}

int uhc_ep_dequeue(const struct device *dev, struct uhc_transfer *const xfer)
{
	const struct uhc_api *api = dev->api;
	int ret;

	api->lock(dev);

	if (!uhc_is_initialized(dev)) {
		ret = -EPERM;
		goto ep_dequeue_error;
	}

	ret = api->ep_dequeue(dev, xfer);
	xfer->queued = 0;

ep_dequeue_error:
	api->unlock(dev);

	return ret;
}

int uhc_enable(const struct device *dev)
{
	const struct uhc_api *api = dev->api;
	struct uhc_data *data = dev->data;
	int ret;

	api->lock(dev);

	if (!uhc_is_initialized(dev)) {
		ret = -EPERM;
		goto uhc_enable_error;
	}

	if (uhc_is_enabled(dev)) {
		ret = -EALREADY;
		goto uhc_enable_error;
	}

	ret = api->enable(dev);
	if (ret == 0) {
		atomic_set_bit(&data->status, UHC_STATUS_ENABLED);
	}

uhc_enable_error:
	api->unlock(dev);

	return ret;
}

int uhc_disable(const struct device *dev)
{
	const struct uhc_api *api = dev->api;
	struct uhc_data *data = dev->data;
	int ret;

	api->lock(dev);

	if (!uhc_is_enabled(dev)) {
		ret = -EALREADY;
		goto uhc_disable_error;
	}

	ret = api->disable(dev);
	atomic_clear_bit(&data->status, UHC_STATUS_ENABLED);

uhc_disable_error:
	api->unlock(dev);

	return ret;
}

int uhc_init(const struct device *dev,
	     uhc_event_cb_t event_cb, const void *const event_ctx)
{
	const struct uhc_api *api = dev->api;
	struct uhc_data *data = dev->data;
	int ret;

	if (event_cb == NULL) {
		return -EINVAL;
	}

	api->lock(dev);

	if (uhc_is_initialized(dev)) {
		ret = -EALREADY;
		goto uhc_init_error;
	}

	data->event_cb = event_cb;
	data->event_ctx = event_ctx;
	sys_dlist_init(&data->ctrl_xfers);
	sys_dlist_init(&data->bulk_xfers);
	sys_dlist_init(&data->periodic_xfers);
	sys_dlist_init(&data->active_xfers);

	ret = api->init(dev);
	if (ret == 0) {
		atomic_set_bit(&data->status, UHC_STATUS_INITIALIZED);
	}

uhc_init_error:
	api->unlock(dev);

	return ret;
}

int uhc_shutdown(const struct device *dev)
{
	const struct uhc_api *api = dev->api;
	struct uhc_data *data = dev->data;
	int ret;

	api->lock(dev);

	if (uhc_is_enabled(dev)) {
		ret = -EBUSY;
		goto uhc_shutdown_error;
	}

	if (!uhc_is_initialized(dev)) {
		ret = -EALREADY;
		goto uhc_shutdown_error;
	}

	ret = api->shutdown(dev);
	atomic_clear_bit(&data->status, UHC_STATUS_INITIALIZED);

uhc_shutdown_error:
	api->unlock(dev);

	return ret;
}
