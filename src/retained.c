/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <hal/nrf_power.h>
#include "retained.h"

/* nRF52 RAM (really, RAM AHB slaves) are partitioned as:
 * * Up to 8 blocks of two 4 KiBy byte "small" sections
 * * A 9th block of with 32 KiBy "large" sections
 *
 * At time of writing the maximum number of large sections is 6, all
 * within the first large block.  Theoretically there could be more
 * sections in the 9th block, and possibly more blocks.
 */

/* Inclusive address of RAM start */
#define SRAM_BEGIN (uintptr_t)DT_REG_ADDR(DT_NODELABEL(sram0))

/* Exclusive address of RAM end */
#define SRAM_END (SRAM_BEGIN + (uintptr_t)DT_REG_SIZE(DT_NODELABEL(sram0)))

/* Size of a controllable RAM section in the small blocks */
#define SMALL_SECTION_SIZE 4096

/* Number of controllable RAM sections in each of the lower blocks */
#define SMALL_SECTIONS_PER_BLOCK 2

/* Span of a small block */
#define SMALL_BLOCK_SIZE (SMALL_SECTIONS_PER_BLOCK * SMALL_SECTION_SIZE)

/* Number of small blocks */
#define SMALL_BLOCK_COUNT 8

/* Span of the SRAM area covered by small sections */
#define SMALL_SECTION_SPAN (SMALL_BLOCK_COUNT * SMALL_BLOCK_SIZE)

/* Inclusive address of the RAM range covered by large sections */
#define LARGE_SECTION_BEGIN (SRAM_BEGIN + SMALL_SECTION_SPAN)

/* Size of a controllable RAM section in large blocks */
#define LARGE_SECTION_SIZE 32768

/* Set or clear RAM retention in SYSTEM_OFF for the provided object.
 *
 * @note This only works for nRF52 with the POWER module.  The other
 * Nordic chips use a different low-level API, which is not currently
 * used by this function.
 *
 * @param ptr pointer to the start of the retainable object
 *
 * @param len length of the retainable object
 *
 * @param enable true to enable retention, false to clear retention
 */
int ram_range_retain(const void *ptr,
			    size_t len,
			    bool enable)
{
	uintptr_t addr = (uintptr_t)ptr;
	uintptr_t addr_end = addr + len;

	/* Error if the provided range is empty or doesn't lie
	 * entirely within the SRAM address space.
	 */
	if ((len == 0U)
	    || (addr < SRAM_BEGIN)
	    || (addr > (SRAM_END - len))) {
		return -EINVAL;
	}

	/* Iterate over each section covered by the range, setting the
	 * corresponding RAM OFF retention bit in the parent block.
	 */
	do {
		uintptr_t block_base = SRAM_BEGIN;
		uint32_t section_size = SMALL_SECTION_SIZE;
		uint32_t sections_per_block = SMALL_SECTIONS_PER_BLOCK;
		bool is_large = (addr >= LARGE_SECTION_BEGIN);
		uint8_t block = 0;

		if (is_large) {
			block = 8;
			block_base = LARGE_SECTION_BEGIN;
			section_size = LARGE_SECTION_SIZE;

			/* RAM[x] supports only 16 sections, each its own bit
			 * for POWER (0..15) and RETENTION (16..31).  We don't
			 * know directly how many sections are present, so
			 * assume they all are; the true limit will be
			 * determined by the SRAM size.
			 */
			sections_per_block = 16;
		}

		uint32_t section = (addr - block_base) / section_size;

		if (section >= sections_per_block) {
			block += section / sections_per_block;
			section %= sections_per_block;
		}

		uint32_t section_mask =
			(POWER_RAM_POWERSET_S0RETENTION_On
					 << (section + POWER_RAM_POWERSET_S0RETENTION_Pos));

		if (enable) {
			nrf_power_rampower_mask_on(NRF_POWER, block, section_mask);
		} else {
			nrf_power_rampower_mask_off(NRF_POWER, block, section_mask);
		}

		/* Move to the first address in the next section. */
		addr += section_size - (addr % section_size);
	} while (addr < addr_end);

	return 0;
}
