/*
 * Copyright (c) 2023 tinyVision.ai
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * USB device controller (UDC) for DesignWare USB3 DRD
 *
 * If you want to define a helper macro, check if there is something similar
 * in include/zephyr/sys/util.h or include/zephyr/usb/usb_ch9.h that you can use.
 * Please keep all identifiers and logging messages concise and clear.
 */

#include "udc_common.h"
#indlude "dwc3.h"

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define UPPER_32_BITS(u64) ((u64) >> 32)
#define LOWER_32_BITS(u64) ((u64) & 0xffffffff)

/*
 * Structure for holding controller configuration items that can remain in
 * non-volatile memory. This is usually accessed as
 *   const struct dwc3_config *config = dev->config;
 */
struct dwc3_config {
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	void (*make_thread)(const struct device *dev);
	int speed_idx;
	uint32_t *regs;
};

/*
 * All data specific to one endpoint
 */
struct dwc3_ep_data {
	/* Buffer passed to the DWC3 core filled with the Transfer Request Blocks */
	struct dwc3_trb *trb_ring;

	/* Number of buffers in the trb_ring */
	size_t trb_num;

	/* Index within trb_buf where to queue new TRBs, assumes size of 256*/
	uint8_t trb_enqueue;
};

/*
 * Data of each instance of the driver, that can be read and written to.
 * Accessed via "udc_get_private(dev)".
 */
struct dwc3_data {
	/* As many items as there are endpoints in the system */
	struct dwc3_ep_data *ep_data;
};

static inline uint32_t dwc3_read_u32(const struct device *dev, int addr)
{
	const struct dwc3_config *config = dev->config;

	return config->regs[addr];
}

static inline void dwc3_write_u32(const struct device *dev, int addr, uint32_t value)
{
	const struct dwc3_config *config = dev->config;

	config->regs[addr] = value;
}

static void dwc3_ep_wait_cmd_completion(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	uint32_t reg;
	
	do {
		reg = dwc3_read_u32(dev, DWC3_DEPCMD(ep_cfg->addr));
	} while (reg & DWC3_DEPCMD_CMDACT)
}

static void dwc3_ep_cmd_set_config(const struct device *dev, struct udc_ep_config *const ep_cfg
	uint32_t param0, uint32_t param1)
{
	dwc3_write_u32(dev, DWC3_DEPCMDPAR0(ep_cfg->addr), param0);
	dwc3_write_u32(dev, DWC3_DEPCMDPAR1(ep_cfg->addr), param1);
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPCFG);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static void dwc3_ep_cmd_set_xfer_config(const struct device *dev, struct udc_ep_config *const ep_cfg,
	uint32_t param0)
{
	dwc3_write_u32(dev, DWC3_DEPCMDPAR0(ep_cfg->addr), param0);
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPXFERCFG);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static void dwc3_ep_cmd_get_data_seq_num(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPGETDSEQ);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static void dwc3_ep_cmd_set_stall(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPSETSTALL);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static void dwc3_ep_cmd_clear_stall(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPCSTALL);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static void dwc3_ep_cmd_start_xfer(const struct device *dev, struct udc_ep_config *const ep_cfg,
	uint64_t address)
{
	dwc3_write_u32(dev, DWC3_DEPCMDPAR0(ep_cfg->addr), UPPER_32_BITS(param0));
	dwc3_write_u32(dev, DWC3_DEPCMDPAR1(ep_cfg->addr), LOWER_32_BITS(param1));
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPSTRTXFER);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static void dwc3_ep_cmd_update_xfer(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPUPDXFER);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static void dwc3_ep_cmd_end_xfer(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPENDXFER);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static void dwc3_ep_cmd_start_config(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	dwc3_write_u32(dev, DWC3_DEPCMD(ep_cfg->addr), DWC3_DEPCMD_DEPSTARTCFG);
	dwc3_ep_wait_cmd_completion(dev, ep_cfg);
}

static ALWAYS_INLINE void dwc3_thread_handler(void *const arg)
{
	const struct device *dev = (const struct device *)arg;

	LOG_DBG("Driver %p thread started", dev);
	while (true) {
		k_msleep(1000);
	}
}

static struct dwc3_trb *dwc3_ep_new_trb(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct dwc3_data *priv = udc_get_private(dev);
	assert(ep_cfg->addr < ARRAY_SIZE(priv->ep_data));
	struct dwc3_ep_data *ep_data = &priv->ep_data[ep_cfg->addr];
	struct dwc3_trb *trb = &ep_data->trb_pool[ep_data->trb_enqueue];

	/* If the Hardware has not cleared the HWO flag, it is not free */
	if (trb->ctrl & DWC3_TRB_CTRL_HWO) {
		LOG_DBG("Running out of TRBs");
		return NULL;
	}

	/* We set the Hardware-Owned flag, DWC3 core will unset it when done. */
	trb->ctrl |= DWC3_TRB_CTRL_HWO;

	/* This is a ring buffer, wrap around. */
	ep_data->trb_enqueue++;
	ep_data->trb_euqueue %= ep_data->trb_num;

	memset(trb, 0);
	return trb;
}

/*
 * Rather than using a data structure that fill buffers to transfer,
 * and a thread that empty it into the data endpoints, this immediately
 * sends the endpoint to the hardware, which also has a queue.
 * In both cases, the system's RAM is used to store the buffers and list of pointers,
 * so I do not see downsides in doing this
 */
static int dwc3_ep_enqueue(const struct device *dev, struct udc_ep_config *const ep_cfg,
	struct net_buf *buf)
{
	struct dwc3_trb *trb;

	LOG_DBG("%p enqueue %p", dev, buf);
	udc_buf_put(ep_cfg, buf);

	if (ep_cfg->stat.halted) {
		LOG_DBG("ep 0x%02x halted", ep_cfg->addr);
		return 0;
	}

	/* TODO Since we use the buffer without copying it, we need to keep
	 * it allocated as long as it is used by the hardware */

	/* Immediately fille a new TRB to the hardware */
	trb = dwc3_ep_new_trb(dev, ep_cfg);
	if (trb == NULL) {
		LOG_ERR("Out of free TRB buffers, all are being processed by DWC3.");
		return -ENOMEM;
	}
	trb->addr_hi = (uint32_t)(buf->data >> 32);
	trb->addr_lo = (uint32_t)(buf->data & 0xFFFFFFFF)
	trb->status = buf->size << DWC3_TRB_STATUS_BUFSIZ_SHIFT;

	/* The TRB is ready, submit it to DWC3 directly. */
	dwc3_ep_cmd_start_xfer(dev, ep_cfg, (uint64_t)buf->data);

	return 0;
}

 static int dwc3_ep_dequeue(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	lock_key = irq_lock();

	buf = udc_buf_get_all(dev, ep_cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);

	return 0;
}

/*
 * Configure and make an endpoint ready for use.
 * This is called in the context of udc_ep_enable() or udc_ep_enable_internal(),
 * the latter of which may be used by the driver to enable control endpoints.
 */
static int dwc3_ep_enable(const struct device *dev,
				  struct udc_ep_config *const ep_cfg)
{
	LOG_DBG("Enable ep 0x%02x", ep_cfg->addr);

	return 0;
}

/*
 * Opposite function to dwc3_ep_enable(). udc_ep_disable_internal()
 * may be used by the driver to disable control endpoints.
 */
static int dwc3_ep_disable(const struct device *dev,
				   struct udc_ep_config *const ep_cfg)
{
	LOG_DBG("Disable ep 0x%02x", ep_cfg->addr);

	return 0;
}

/*
 * Halt endpoint. Halted endpoint should respond with a STALL handshake.
 */
static int dwc3_ep_set_halt(const struct device *dev,
				    struct udc_ep_config *const ep_cfg)
{
	LOG_DBG("Set halt ep 0x%02x", ep_cfg->addr);

	ep_cfg->stat.halted = true;

	return 0;
}

/*
 * Opposite to halt endpoint. If there are requests in the endpoint queue,
 * the next transfer should be prepared.
 */
static int dwc3_ep_clear_halt(const struct device *dev,
				      struct udc_ep_config *const ep_cfg)
{
	LOG_DBG("Clear halt ep 0x%02x", ep_cfg->addr);
	ep_cfg->stat.halted = false;

	return 0;
}

static int dwc3_set_address(const struct device *dev, const uint8_t addr)
{
	LOG_DBG("Set new address %u for %p", addr, dev);

	return 0;
}

static int dwc3_host_wakeup(const struct device *dev)
{
	LOG_DBG("Remote wakeup from %p", dev);

	return 0;
}

/*
 * Return actual USB device speed
 */
static enum udc_bus_speed dwc3_device_speed(const struct device *dev)
{
	struct udc_data *data = dev->data;

	return data->caps.hs ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

static int dwc3_enable(const struct device *dev)
{
	LOG_DBG("Enable device %p", dev);

	return 0;
}

static int dwc3_disable(const struct device *dev)
{
	LOG_DBG("Disable device %p", dev);

	return 0;
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only dwc3_enable() makes device visible to the host.
 */
static int dwc3_init(const struct device *dev)
{
	uint32_t reg;
	int err;

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	reg = dwc3_read_u32(dev, DWC3_GSNPSID);
	LOG_INF("core revision 0x%02x", reg);

	// dwc3_init_phy
	// dwc3_setup_ulpi
	// dwc3_soft_reset
	// dwc3_init_ulpi
	// dwc3_global_control_setup
	// dwc3_set_power_down_clk_scale
	// dwc3_frame_length_adjustment
	// dwc3_ref_clk_period
	// dwc3_set_incr_burst_type
	// dwc3_phy_power_on
	// dwc3_event_buffers_setup

	return 0;
}

/*
 * Shut down the controller completely
 */
static int dwc3_shutdown(const struct device *dev)
{
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	return 0;
}

/*
 * This is called once to initialize the controller and endpoints
 * capabilities, and register endpoint structures.
 */
static int dwc3_driver_preinit(const struct device *dev)
{
	const struct dwc3_config *config = dev->config;
	struct udc_data *data = dev->data;
	uint16_t mps = 1023;
	int err;

	k_mutex_init(&data->mutex);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;
	if (config->speed_idx == 2) {
		data->caps.hs = true;
		mps = 1024;
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = mps;
		}
		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;

		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = mps;
		}
		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;

		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	config->make_thread(dev);
	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	return 0;
}

static int dwc3_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int dwc3_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static const struct udc_api dwc3_api = {
	.lock = dwc3_lock,
	.unlock = dwc3_unlock,
	.device_speed = dwc3_device_speed,
	.init = dwc3_init,
	.enable = dwc3_enable,
	.disable = dwc3_disable,
	.shutdown = dwc3_shutdown,
	.set_address = dwc3_set_address,
	.host_wakeup = dwc3_host_wakeup,
	.ep_enable = dwc3_ep_enable,
	.ep_disable = dwc3_ep_disable,
	.ep_set_halt = dwc3_ep_set_halt,
	.ep_clear_halt = dwc3_ep_clear_halt,
	.ep_enqueue = dwc3_ep_enqueue,
	.ep_dequeue = dwc3_ep_dequeue,
};

#define DT_DRV_COMPAT zephyr_dwc3

#define DWC3_DEVICE_DEFINE(n)							\
	K_THREAD_STACK_DEFINE(dwc3_stack_##n, CONFIG_DWC3);			\
										\
	static struct udc_ep_config						\
		ep_cfg_out[DT_INST_PROP(n, num_bidir_endpoints)];		\
	static struct udc_ep_config						\
		ep_cfg_in[DT_INST_PROP(n, num_bidir_endpoints)];		\
	static const struct dwc3_config dwc3_config_##n = {			\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),		\
		.ep_cfg_in = ep_cfg_in,						\
		.ep_cfg_out = ep_cfg_out,					\
		.make_thread = dwc3_make_thread_##n,				\
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),	\
		.regs = (uint32_t *)DT_INST_REG_ADDR(n),			\
	};									\
										\
	static struct dwc3_ep_data						\
		ep_data[DT_INST_PROP(n, num_bidir_endpoints)];			\
	static struct dwc3_data udc_priv_##n = {				\
		.ep_data = ep_data,						\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, dwc3_driver_preinit, NULL,			\
			      &udc_data_##n, &dwc3_config_##n,			\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &dwc3_api);

DT_INST_FOREACH_STATUS_OKAY(DWC3_DEVICE_DEFINE)
