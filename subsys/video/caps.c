/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/video.h>
#include <zephyr/logging/log.h>
#include <zephyr/video/video.h>

LOG_MODULE_REGISTER(video_caps, CONFIG_VIDEO_LOG_LEVEL);

int video_get_caps(const struct device *dev, struct video_caps *caps)
{
	if (dev == NULL || caps == NULL ||
	    (caps->type != VIDEO_BUF_TYPE_INPUT && caps->type != VIDEO_BUF_TYPE_OUTPUT)) {
		LOG_ERR("Invalid capabilities call: dev %p, capos %p, type %u",
			dev, caps, caps == NULL ? 0 : caps->type);
		return -EINVAL;
	}

	return video_driver_get_caps(dev, caps);
}

int video_format_caps_index(const struct video_format_cap *fmts, const struct video_format *fmt,
			    size_t *idx)
{
	if (fmts == NULL || fmt == NULL || idx == NULL) {
		LOG_ERR("Invalid format caps index call: fmts %p, fmt %p, idx %p",
			fmts, fmt, idx);
		return -EINVAL;
	}

	for (int i = 0; fmts[i].pixelformat != 0; i++) {
		if (fmts[i].pixelformat == fmt->pixelformat &&
		    IN_RANGE(fmt->width, fmts[i].width_min, fmts[i].width_max) &&
		    IN_RANGE(fmt->height, fmts[i].height_min, fmts[i].height_max)) {
			*idx = i;
			return 0;
		}
	}
	return -ENOENT;
}

int video_transform_cap(const struct device *const dev,
			const struct video_format_cap *const cap,
			struct video_format_cap *const res_cap,
			enum video_buf_type type, uint16_t ind)
{
	if (dev == NULL || cap == NULL || res_cap == NULL ||
	    (type != VIDEO_BUF_TYPE_INPUT && type != VIDEO_BUF_TYPE_OUTPUT)) {
		LOG_ERR("Invalid transform caps call: dev %p, cap %p, res_cap %p, type %u",
			dev, cap, res_cap, type);
		return -EINVAL;
	}

	return video_driver_transform_cap(dev, cap, res_cap, type, ind);
}
