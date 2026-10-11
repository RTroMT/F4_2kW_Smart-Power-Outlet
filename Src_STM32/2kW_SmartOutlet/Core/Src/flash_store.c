#include "flash_store.h"
#include <string.h>

#define STORE_MAGIC   0x53544F52UL            /* "STOR" */

/* Where to READ from. Normally the flash address itself; only changed by host tests. */
#ifndef FLASH_STORE_PTR
#define FLASH_STORE_PTR   ((const uint8_t *)FLASH_STORE_BASE)
#endif

typedef struct {
    uint32_t magic;
    uint32_t len;          /* payload bytes */
    uint32_t crc;          /* CRC32 of the payload */
} store_header_t;

/* Standard CRC-32 (the one used by zip / Ethernet), bit by bit: small and fast enough here. */
static uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (uint32_t i = 0u; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return ~crc;
}

/**
 * Read the blob back. Three things must be right: the magic number, the length, and the
 * CRC of the payload. An erased sector (all 0xFF) fails the magic check.
 */
bool FlashStore_Read(void *dst, uint32_t len)
{
    if (dst == NULL || len == 0u || len > FLASH_STORE_MAX_BYTES) {
        return false;
    }

    store_header_t hdr;
    memcpy(&hdr, FLASH_STORE_PTR, sizeof(hdr));

    if (hdr.magic != STORE_MAGIC || hdr.len != len) {
        return false;
    }

    const uint8_t *payload = FLASH_STORE_PTR + sizeof(hdr);
    if (crc32_calc(payload, len) != hdr.crc) {
        return false;
    }

    memcpy(dst, payload, len);
    return true;
}

/**
 * Store the blob.
 * Flash can only turn bits from 1 to 0, so a sector must be erased before it is written:
 *   1) build the whole image (header + payload) in RAM
 *   2) unlock the flash, erase the sector
 *   3) program the image word by word
 *   4) lock the flash and read everything back to be sure
 */
bool FlashStore_Write(const void *src, uint32_t len)
{
    if (src == NULL || len == 0u || len > FLASH_STORE_MAX_BYTES) {
        return false;
    }

    /* 1) image in RAM, padded with 0xFF to whole 32-bit words */
    uint32_t image[(sizeof(store_header_t) + FLASH_STORE_MAX_BYTES) / 4u + 1u];
    memset(image, 0xFF, sizeof(image));

    store_header_t hdr;
    hdr.magic = STORE_MAGIC;
    hdr.len   = len;
    hdr.crc   = crc32_calc((const uint8_t *)src, len);
    memcpy(image, &hdr, sizeof(hdr));
    memcpy((uint8_t *)image + sizeof(hdr), src, len);

    const uint32_t words = (uint32_t)((sizeof(hdr) + len + 3u) / 4u);

    /* 2) unlock + erase */
    bool ok = true;
    (void)HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0u;
    memset(&erase, 0, sizeof(erase));
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = FLASH_SECTOR_5;
    erase.NbSectors    = 1u;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;            /* 2.7 .. 3.6 V: 32-bit programming */

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        ok = false;
    }

    /* 3) program */
    for (uint32_t i = 0u; ok && i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORE_BASE + 4u * i, image[i]) != HAL_OK) {
            ok = false;
        }
    }

    /* 4) lock again, then check what is really in the flash */
    (void)HAL_FLASH_Lock();

    if (!ok) {
        return false;
    }

    uint8_t check[FLASH_STORE_MAX_BYTES];
    if (!FlashStore_Read(check, len)) {
        return false;
    }
    return memcmp(check, src, len) == 0;
}