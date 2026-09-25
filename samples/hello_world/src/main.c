/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	//printf("[0x90000000] = 0x%08x\n", *(volatile uint32_t *)0x90000000);

	return 0;
}
