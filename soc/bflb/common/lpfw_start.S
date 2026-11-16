#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/offsets.h>
#include <zephyr/arch/riscv/csr.h>

#define WRAM_ADDR 0x63010000

GTEXT(lpfw_start)
SECTION_FUNC(TEXT, lpfw_start)
	lui t0, %hi(WRAM_ADDR)
	addi t0, t0, %lo(WRAM_ADDR)
	jalr t0

GTEXT(lpfw_loop)
SECTION_FUNC(TEXT, lpfw_loop)
	csrci mstatus, MSTATUS_MIE
	j lpfw_loop
