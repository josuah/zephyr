/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Panoramix Labs
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

int main(void)
{
	*(volatile uint32_t *)0x12340000 = 0x4567;

	while (1) {
		;
	}

	return 0;
}
