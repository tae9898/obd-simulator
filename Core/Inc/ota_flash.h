/**
 * @file    ota_flash.h
 * @brief   OTA flash erase/write driver (STM32G4)
 * @note    Sequence verification by "writing to OTA data area (0x0801E000~)" without bootloader.
 *          Actual app replacement/jump is bootloader phase (separate build).
 */
#ifndef __OTA_FLASH_H
#define __OTA_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* OTA data area -- last 8KB of flash (unused by app). */
#define OTA_FLASH_BASE   0x0801E000U
#define OTA_FLASH_END    0x08020000U
#define OTA_FLASH_SIZE   (OTA_FLASH_END - OTA_FLASH_BASE)   /* 8KB */

/**
 * @brief  Page (2KB) unit erase
 * @retval 0=success, -1=out of range or failure
 */
int ota_flash_erase(uint32_t addr, uint32_t size);

/**
 * @brief  Doubleword (8B) unit write. Unaligned/residual bytes ignored (simulator simplification).
 * @retval 0=success, -1=out of range or failure
 */
int ota_flash_write(uint32_t addr, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_FLASH_H */
