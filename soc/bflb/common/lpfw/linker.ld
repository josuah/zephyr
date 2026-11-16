OUTPUT_ARCH("riscv")
OUTPUT_FORMAT("elf32-littleriscv")

MEMORY {
	RAM (rwx) : ORIGIN = 0x63010000, LENGTH = 0x10000
}

ENTRY("__start")

SECTIONS {
	lpfw.start : {
		*(.text.__start)
	} > RAM

	lpfw.main : {
		*(*)
	} > RAM
}
