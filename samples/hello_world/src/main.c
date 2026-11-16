/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>

#include <lpfw.h>

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	lpfw_copy();
	lpfw_prepare();
	lpfw_start();

	return 0;
}
