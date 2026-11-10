#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/cache.h>

#include <bouffalolab/bl61x/hbn_reg.h>
#include <bouffalolab/bl61x/bflb_soc.h>

#include <lpfw.h>

LOG_MODULE_REGISTER(bflb_lpfw, LOG_LEVEL_INF);

#define WRAM_ADDR DT_REG_ADDR(DT_NODELABEL(wram))
#define WRAM_SIZE DT_REG_SIZE(DT_NODELABEL(wram))
#define HBN_STATUS_ENTER_FLAG 0x4e424845

uint8_t lpfw_image_data[] = {
#include <lpfw.inc>
};

struct bl_lp_fw_info {
	/* Jump Instruction stored into an array, execute this */
	uint32_t jump_code;
	/* "lpfw" string (little endian is 0x7766706c) */
	uint32_t magic_code;
	uint32_t lpfw_memory_start;
	uint32_t lpfw_code_end;
	uint32_t lpfw_memory_end;
	/* zero-padded version string */
	char lpfw_version_str[36];
} __packed;

void lpfw_copy(void)
{
	struct bl_lp_fw_info *info = (void *)lpfw_image_data;
	char lpfw_version_str[sizeof(info->lpfw_version_str) + 1] = {};

	memcpy(lpfw_version_str, info->lpfw_version_str, sizeof(info->lpfw_version_str));

	LOG_INF("jump 0x%08x, magic 0x%08x, memstart 0x%08x, codeend 0x%08x, memend 0x%08x, '%s'",
		info->jump_code, info->magic_code, info->lpfw_memory_start, info->lpfw_code_end,
		info->lpfw_memory_end, lpfw_version_str);

	LOG_INF("wram addr 0x%08x (0x%08x + 0x%08x)",
		WRAM_ADDR, DT_REG_ADDR(DT_NODELABEL(all_ram)), DT_REG_ADDR(DT_NODELABEL(wram)));

	memcpy((void *)WRAM_ADDR, lpfw_image_data, sizeof(lpfw_image_data));
	sys_cache_data_flush_and_invd_range((void *)WRAM_ADDR, sizeof(lpfw_image_data));
	sys_cache_instr_flush_and_invd_range((void *)WRAM_ADDR, sizeof(lpfw_image_data));
}

void lpfw_prepare(void)
{
	uint32_t tmp;

#if 0
	/* Set bus remap */
	tmp = sys_read32(TZ1_BASE + TZC_SEC_TZC_ROM_TZSRG_CTRL);
	tmp |= TZC_SEC_TZC_BUS_RMP_EN;
	sys_write32(TZ1_BASE + TZC_SEC_TZC_ROM_TZSRG_CTRL, tmp);

	/* Set Lock */
	tmp = sys_read32(TZ1_BASE + TZC_SEC_TZC_ROM_TZSRG_CTRL);
	tmp |= TZC_SEC_TZC_BUS_RMP_EN_LOCK;
	sys_write32(TZ1_BASE + TZC_SEC_TZC_ROM_TZSRG_CTRL, tmp);
#endif
	/* set wakeup callback */

	k_sleep(K_MSEC(1000));

	sys_write32((uint32_t)(BL616_WRAM_BASE), HBN_BASE + HBN_RSV1_OFFSET);
	/* Set HBN flag */
	sys_write32(HBN_STATUS_ENTER_FLAG, HBN_BASE + HBN_RSV0_OFFSET);

	printk("here\n");
}
