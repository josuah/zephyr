/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private API for USB host controller (UHC) drivers
 */

#ifndef ZEPHYR_INCLUDE_UHC_COMMON_H
#define ZEPHYR_INCLUDE_UHC_COMMON_H

#include <zephyr/drivers/usb/uhc.h>

/**
 * @brief Get driver's private data
 *
 * @param[in] dev    Pointer to device struct of the driver instance
 *
 * @return pointer to driver's private data
 */
static inline void *uhc_get_private(const struct device *dev)
{
	struct uhc_data *data = dev->data;

	return data->priv;
}

/**
 * @brief Locking function for the drivers.
 *
 * @param[in] dev     Pointer to device struct of the driver instance
 * @param[in] timeout Timeout
 *
 * @return values provided by k_mutex_lock()
 */
static inline int uhc_lock_internal(const struct device *dev,
				    k_timeout_t timeout)
{
	struct uhc_data *data = dev->data;

	return k_mutex_lock(&data->mutex, timeout);
}

/**
 * @brief Unlocking function for the drivers.
 *
 * @param[in] dev    Pointer to device struct of the driver instance
 *
 * @return values provided by k_mutex_lock()
 */
static inline int uhc_unlock_internal(const struct device *dev)
{
	struct uhc_data *data = dev->data;

	return k_mutex_unlock(&data->mutex);
}

/**
 * @brief Helper function to return UHC transfer to a higher level.
 *
 * Function to dequeue transfer and send UHC event to a higher level.
 *
 * @param[in] dev    Pointer to device struct of the driver instance
 * @param[in] xfer   Pointer to UHC transfer
 * @param[in] err    Transfer error
 */
void uhc_xfer_return(const struct device *dev,
		     struct uhc_transfer *const xfer,
		     const int err);

/**
 * @brief Helper to get next periodic transfer to process.
 *
 * This includes control or bulk transfer types.
 * If there is no transfer ready for the current frame, then skip
 * to the next frame.
 *
 * @param[in] dev    Pointer to device struct of the driver instance
 * @param cur_frame  The current USB frame number
 * @param max_frame  The maximum value for the USB frame number
 *
 * @return pointer to the next transfer or NULL on error.
 */
struct uhc_transfer *uhc_xfer_get_periodic(const struct device *const dev,
					   uint16_t cur_frame,
					   uint16_t max_frame);

/**
 * @brief Helper to get next non-periodic transfer to process.
 *
 * @param[in] dev    Pointer to device struct of the driver instance
 *
 * @return pointer to the next transfer or NULL on error.
 */
struct uhc_transfer *uhc_xfer_get_non_periodic(const struct device *dev);

/**
 * @brief Helper to append a transfer to internal list.
 *
 * @param[in] dev    Pointer to device struct of the driver instance
 * @param[in] xfer   Pointer to UHC transfer
 */
void uhc_xfer_append(const struct device *dev,
		     struct uhc_transfer *const xfer,
		     uint16_t cur_frame, uint16_t max_frame);

/**
 * @brief Helper to move a transfer to the list of active transfers.
 *
 * @param[in] dev    Pointer to device struct of the driver instance
 * @param[in] xfer   Pointer to UHC transfer
 */
void uhc_xfer_set_in_progress(const struct device *const dev,
			      struct uhc_transfer *const xfer);

/**
 * @brief Helper function to send UHC event to a higher level.
 *
 * The callback would typically sends UHC even to a message queue (k_msgq).
 *
 * @param[in] dev    Pointer to device struct of the driver instance
 * @param[in] type   Event type
 * @param[in] status Event status
 *
 * @return 0 on success, all other values should be treated as error.
 * @retval -EPERM controller is not initialized
 */
int uhc_submit_event(const struct device *dev,
		     const enum uhc_event_type type,
		     const int status);

/**
 * @brief Compare two frame number taking overflow into account.
 *
 * Use the middle of the range (max) as a threshold between overflow and non-overflow of values.
 *
 * @param[in] a      First frame number to compare
 * @param[in] b      Second frame number to compare
 * @param[in] max    Maximum value after which counter overflow happens.
 *
 * @retval true      when a is lower than b
 * @retval false     when b is lower than a
 */
bool uhc_xfer_seq_lt(uint16_t a, uint16_t b, uint16_t max);

/**
 * @brief Compare two frame number taking overflow into account.
 *
 * Use the middle of the range (max) as a threshold between overflow and non-overflow of values.
 *
 * @param[in] a      First frame number to compare
 * @param[in] b      Second frame number to compare
 * @param[in] max    Maximum value after which counter overflow happens.
 *
 * @retval true      when a is lower than b or equal
 * @retval false     when b is lower than a or equal
 */
bool uhc_xfer_seq_le(uint16_t a, uint16_t b, uint16_t max);

#endif /* ZEPHYR_INCLUDE_UHC_COMMON_H */
