#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * Keeps ONE small settings blob (touch calibration, ...) in the internal flash, so it
 * survives power-off.
 *
 * Where: the last flash sector of the STM32F401xC (sector 5, 128 KB at 0x08020000).
 *        Keep the program smaller than 128 KB, or edit the linker script so that FLASH
 *        ends at 0x08020000 (LENGTH = 128K): then the linker refuses to place code there.
 * Safety: the blob is stored with a magic number and a CRC32, so an empty or half-written
 *         sector is detected and reported as "no settings".
 * Warning: erasing a 128 KB sector stalls the CPU for about 1 - 2 seconds. Write only when the
 *          user changed something; the HLW8032 receiver recovers from the gap by itself.
 */

#ifndef FLASH_STORE_BASE
#define FLASH_STORE_BASE       0x08020000UL
#endif
#define FLASH_STORE_MAX_BYTES  256u

/** Copy the stored blob into dst. Returns false if nothing valid of exactly `len` bytes is stored. */
bool FlashStore_Read(void *dst, uint32_t len);

/** Erase the sector and store the blob (len <= FLASH_STORE_MAX_BYTES). Returns true if it verified OK. */
bool FlashStore_Write(const void *src, uint32_t len);

#endif /* FLASH_STORE_H */
