/**
 * @file    ota_flash.h
 * @brief   OTA용 flash erase/write 드라이버 (STM32G4)
 * @note    bootloader 없이 "OTA 데이터 영역(0x0801E000~)에 쓰기" 로 시퀀스 검증.
 *          실제 앱 교체/점프는 bootloader 단계(별도 빌드).
 */
#ifndef __OTA_FLASH_H
#define __OTA_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* OTA 데이터 영역 — flash 끝 8KB (앱이 안 쓰는 빈 영역). page=2KB. */
#define OTA_FLASH_BASE   0x0801E000U
#define OTA_FLASH_END    0x08020000U
#define OTA_FLASH_SIZE   (OTA_FLASH_END - OTA_FLASH_BASE)   /* 8KB */

/**
 * @brief  page(2KB) 단위 erase
 * @retval 0=성공, -1=범위 초과 또는 실패
 */
int ota_flash_erase(uint32_t addr, uint32_t size);

/**
 * @brief  doubleword(8B) 단위 write. 미정렬/잔여 바이트는 무시(시뮬 단순화).
 * @retval 0=성공, -1=범위 초과/실패
 */
int ota_flash_write(uint32_t addr, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_FLASH_H */
