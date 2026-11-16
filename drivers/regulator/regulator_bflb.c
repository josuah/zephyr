/*
 * Copyright (c) 2026 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(regulator_bflb, CONFIG_REGULATOR_LOG_LEVEL);

#include <bflb_soc.h>
#include <hbn_reg.h>
#include <aon_reg.h>

static const struct linear_range regulator_ranges[] = {
	LINEAR_RANGE_INIT(600000u, 50000u, 0u, 15u),
};

#ifdef CONFIG_SOC_SERIES_BL61X
#define REGULATOR_BFLB_MIN_V_ID 2
#else
#define REGULATOR_BFLB_MIN_V_ID 0
#endif

#define REGULATOR_BFLB_MAX_CANON_VOLTAGE_ID 10

static const size_t num_regulator_ranges = ARRAY_SIZE(regulator_ranges);

enum regulator_bflb_type {
	REGULATOR_BFLB_TYPE_LDO11_SOC,
	REGULATOR_BFLB_TYPE_LDO11_RT,
	REGULATOR_BFLB_TYPE_LDO11_AON,
	REGULATOR_BFLB_TYPE_LDO15_RF,
	REGULATOR_BFLB_TYPE_LDO18_IO,
};

struct regulator_bflb_config {
	struct regulator_common_config common;
	const uintptr_t reg;
	const enum regulator_bflb_type type;
	const uint8_t sleep_v_id;
};

struct regulator_bflb_data {
	struct regulator_common_data data;
};

/*
 * APIs
 */

static unsigned int regulator_bflb_count_voltages(const struct device *dev)
{
	return linear_range_group_values_count(regulator_ranges, num_regulator_ranges);
}

static int regulator_bflb_list_voltage(const struct device *dev, unsigned int idx, int32_t *volt_uv)
{
	const struct regulator_bflb_config *config = dev->config;

	switch (config->type) {
	case REGULATOR_BFLB_TYPE_LDO11_AON:
	case REGULATOR_BFLB_TYPE_LDO11_RT:
	case REGULATOR_BFLB_TYPE_LDO11_SOC:
		linear_range_group_get_value(regulator_ranges, num_regulator_ranges, idx, volt_uv);
		break;
	case REGULATOR_BFLB_TYPE_LDO15_RF:
		if (idx > 0) {
			return -EINVAL;
		}
		*volt_uv = 1500000;
		break;
	case REGULATOR_BFLB_TYPE_LDO18_IO:
		if (idx > 0) {
			return -EINVAL;
		}
		*volt_uv = 1800000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int regulator_bflb_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_bflb_config *config = dev->config;
	uint16_t idx;
	int ret;
	uint32_t tmp;

	switch (config->type) {
	case REGULATOR_BFLB_TYPE_LDO15_RF:
		if (!IN_RANGE(1500000, min_uv, max_uv)) {
			goto notsup;
		}
		return 0;
	case REGULATOR_BFLB_TYPE_LDO18_IO:
		if (!IN_RANGE(1800000, min_uv, max_uv)) {
			goto notsup;
		}
		return 0;
	default:
		break;
	}

	ret = linear_range_group_get_win_index(regulator_ranges,
					       num_regulator_ranges, min_uv, max_uv, &idx);
	if (ret < 0) {
		return ret;
	}

	if (idx < REGULATOR_BFLB_MIN_V_ID) {
		return -EINVAL;
	}

	if (idx > REGULATOR_BFLB_MAX_CANON_VOLTAGE_ID) {
		LOG_WRN("Demanded voltage is over default, this may result in damage to the chip");
	}

	tmp = sys_read32(config->reg + HBN_GLB_OFFSET);
	switch (config->type) {
	case REGULATOR_BFLB_TYPE_LDO11_SOC:
		tmp &= HBN_SW_LDO11SOC_VOUT_SEL_AON_UMSK;
		tmp |= idx << HBN_SW_LDO11SOC_VOUT_SEL_AON_POS;
		break;
#ifndef CONFIG_SOC_SERIES_BL70XL
	case REGULATOR_BFLB_TYPE_LDO11_RT:
		tmp &= HBN_SW_LDO11_RT_VOUT_SEL_UMSK;
		tmp |= idx << HBN_SW_LDO11_RT_VOUT_SEL_POS;
		break;
#endif
	case REGULATOR_BFLB_TYPE_LDO11_AON:
		tmp &= HBN_SW_LDO11_AON_VOUT_SEL_UMSK;
		tmp |= idx << HBN_SW_LDO11_AON_VOUT_SEL_POS;
		break;
	default:
		return -EINVAL;
	}
	sys_write32(tmp, config->reg + HBN_GLB_OFFSET);

	return 0;
notsup:
	LOG_WRN("Requested range [%uuv, %uuv] not supported by %s", min_uv, max_uv, dev->name);
	return -ENOTSUP;
}

static int regulator_bflb_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_bflb_config *config = dev->config;
	uint32_t idx;

	idx = sys_read32(config->reg + HBN_GLB_OFFSET);
	switch (config->type) {
	case REGULATOR_BFLB_TYPE_LDO11_SOC:
		idx = (idx & HBN_SW_LDO11SOC_VOUT_SEL_AON_MSK) >> HBN_SW_LDO11SOC_VOUT_SEL_AON_POS;
		break;
#ifndef CONFIG_SOC_SERIES_BL70XL
	case REGULATOR_BFLB_TYPE_LDO11_RT:
		idx = (idx & HBN_SW_LDO11_RT_VOUT_SEL_MSK) >> HBN_SW_LDO11_RT_VOUT_SEL_POS;
		break;
#endif
	case REGULATOR_BFLB_TYPE_LDO11_AON:
		idx = (idx & HBN_SW_LDO11_AON_VOUT_SEL_MSK) >> HBN_SW_LDO11_AON_VOUT_SEL_POS;
		break;
	case REGULATOR_BFLB_TYPE_LDO15_RF:
		*volt_uv = 1500000;
		return 0;
	case REGULATOR_BFLB_TYPE_LDO18_IO:
		*volt_uv = 1800000;
		return 0;
	default:
		return -EINVAL;
	}

	return linear_range_group_get_value(regulator_ranges, num_regulator_ranges, idx, volt_uv);
}

static int regulator_bflb_init(const struct device *dev)
{
	const struct regulator_bflb_config *config = dev->config;
	uint32_t tmp;

	regulator_common_data_init(dev);

	switch (config->type) {
#ifndef CONFIG_SOC_SERIES_BL70XL
	case REGULATOR_BFLB_TYPE_LDO11_RT:
		tmp = sys_read32(config->reg + HBN_CTL_OFFSET);
		tmp &= HBN_LDO11_RT_VOUT_SEL_UMSK;
		tmp |= config->sleep_v_id << HBN_LDO11_RT_VOUT_SEL_POS;
		sys_write32(tmp, config->reg + HBN_CTL_OFFSET);
		break;
#endif
	case REGULATOR_BFLB_TYPE_LDO11_AON:
		tmp = sys_read32(config->reg + HBN_CTL_OFFSET);
		tmp &= HBN_LDO11_AON_VOUT_SEL_UMSK;
		tmp |= config->sleep_v_id << HBN_LDO11_AON_VOUT_SEL_POS;
		sys_write32(tmp, config->reg + HBN_CTL_OFFSET);
		break;
	case REGULATOR_BFLB_TYPE_LDO11_SOC:
	case REGULATOR_BFLB_TYPE_LDO15_RF:
	case REGULATOR_BFLB_TYPE_LDO18_IO:
		break;
	default:
		return -EINVAL;
	}

	return regulator_common_init(dev, true);
}

static int regulator_bflb_enable(const struct device *dev)
{
	const struct regulator_bflb_config *config = dev->config;
	size_t tmp;

	switch (config->type) {
	case REGULATOR_BFLB_TYPE_LDO11_AON:
	case REGULATOR_BFLB_TYPE_LDO11_RT:
	case REGULATOR_BFLB_TYPE_LDO11_SOC:
	case REGULATOR_BFLB_TYPE_LDO18_IO:
		return -ENOTSUP;
	case REGULATOR_BFLB_TYPE_LDO15_RF:
		tmp = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
		tmp |= AON_PU_LDO15RF_AON_UMSK;
		sys_write32(tmp, AON_BASE + AON_RF_TOP_AON_OFFSET);
		return 0;
	default:
		return -EINVAL;
	}

	return 0;
}

static int regulator_bflb_disable(const struct device *dev)
{
	const struct regulator_bflb_config *config = dev->config;
	uint32_t tmp;

	switch (config->type) {
	case REGULATOR_BFLB_TYPE_LDO11_AON:
	case REGULATOR_BFLB_TYPE_LDO11_RT:
	case REGULATOR_BFLB_TYPE_LDO11_SOC:
	case REGULATOR_BFLB_TYPE_LDO18_IO:
		return -ENOTSUP;
	case REGULATOR_BFLB_TYPE_LDO15_RF:
		tmp = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
		tmp &= AON_PU_LDO15RF_AON_UMSK;
		sys_write32(tmp, AON_BASE + AON_RF_TOP_AON_OFFSET);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static DEVICE_API(regulator, api) = {
	.count_voltages = regulator_bflb_count_voltages,
	.list_voltage = regulator_bflb_list_voltage,
	.set_voltage = regulator_bflb_set_voltage,
	.get_voltage = regulator_bflb_get_voltage,
	.enable = regulator_bflb_enable,
	.disable = regulator_bflb_disable,
};

#define REGULATOR_BFLB_CHECKS(n)								\
	IF_ENABLED(DT_NODE_HAS_PROP(n, sleep_microvolt), (					\
		BUILD_ASSERT(DT_ENUM_IDX(n, sleep_microvolt) > REGULATOR_BFLB_MIN_V_ID,		\
			     "Voltage must be at least 0.70v");					\
	))

#define REGULATOR_BFLB_SLEEP_V_ID(n)								\
	IF_ENABLED(DT_NODE_HAS_PROP(n, sleep_microvolt), (					\
		.sleep_v_id = DT_ENUM_IDX(n, sleep_microvolt),					\
	))

#define REGULATOR_BFLB_DEFINE(n, n_type)							\
	REGULATOR_BFLB_CHECKS(n)								\
	static struct regulator_bflb_data data_##n;						\
												\
	static const struct regulator_bflb_config config_##n = {				\
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(n),					\
		.reg = DT_REG_ADDR(n),								\
		.type = n_type,									\
		REGULATOR_BFLB_SLEEP_V_ID(n)							\
	};											\
												\
	DEVICE_DT_DEFINE(n, regulator_bflb_init, NULL, &data_##n, &config_##n,			\
			      PRE_KERNEL_1, CONFIG_REGULATOR_BFLB_INIT_PRIORITY, &api);

DT_FOREACH_STATUS_OKAY_VARGS(bflb_ldo11_aon_regulator, REGULATOR_BFLB_DEFINE,
			     REGULATOR_BFLB_TYPE_LDO11_AON);
DT_FOREACH_STATUS_OKAY_VARGS(bflb_ldo11_rt_regulator, REGULATOR_BFLB_DEFINE,
			     REGULATOR_BFLB_TYPE_LDO11_RT);
DT_FOREACH_STATUS_OKAY_VARGS(bflb_ldo11_soc_regulator, REGULATOR_BFLB_DEFINE,
			     REGULATOR_BFLB_TYPE_LDO11_SOC);
DT_FOREACH_STATUS_OKAY_VARGS(bflb_ldo15_rf_regulator, REGULATOR_BFLB_DEFINE,
			     REGULATOR_BFLB_TYPE_LDO15_RF);
DT_FOREACH_STATUS_OKAY_VARGS(bflb_ldo18_io_regulator, REGULATOR_BFLB_DEFINE,
			     REGULATOR_BFLB_TYPE_LDO18_IO);
