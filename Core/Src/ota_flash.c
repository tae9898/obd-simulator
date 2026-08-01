/**
 * @file    ota_flash.c
 * @brief   OTA용 flash erase/write (STM32G4 HAL)
 */
#include "ota_flash.h"
#include "stm32g4xx_hal_flash.h"
#include <string.h>

/* 영역 검증: OTA 데이터 영역(0x0801E000~0x0801FFFF) 내인가 */
static int in_range(uint32_t addr, uint32_t size)
{
    return (addr >= OTA_FLASH_BASE) && ((addr + size) <= OTA_FLASH_END);
}

int ota_flash_erase(uint32_t addr, uint32_t size)
{
    if (!in_range(addr, size)) {
        return -1;
    }

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef ei;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = FLASH_BANK_1;
    ei.Page      = (addr - 0x08000000U) / FLASH_PAGE_SIZE;
    ei.NbPages   = (size + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE;

    uint32_t err = 0xFFFFFFFFU;
    HAL_StatusTypeDef s = HAL_FLASHEx_Erase(&ei, &err);

    HAL_FLASH_Lock();
    return ((s == HAL_OK) && (err == 0U)) ? 0 : -1;
}

int ota_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (!in_range(addr, len) || (data == NULL)) {
        return -1;
    }

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef s = HAL_OK;
    uint32_t i = 0U;

    /* doubleword(8바이트) 단위 기록. 잔여(8 미만)는 무시 — 시뮬레이터 단순화. */
    while ((i + 8U) <= len) {
        uint64_t dw;
        (void)memcpy(&dw, &data[i], 8U);
        s = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dw);
        if (s != HAL_OK) {
            break;
        }
        i += 8U;
    }

    HAL_FLASH_Lock();
    return (s == HAL_OK) ? 0 : -1;
}
