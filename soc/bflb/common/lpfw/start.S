#define WRAM_ADDR 0x63010000

.global __start;
.type __start, %function
.section .text.__start
__start:
	add t0, zero, zero
	j __start
	call main
