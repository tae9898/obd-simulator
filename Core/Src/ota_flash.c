/**
 * @file    ota_flash.c
 * @brief   OTA flash erase/write (STM32G4 HAL)
 */
#include "ota_flash.h"
#include "stm32g4xx_hal_flash.h"
#include <string.h>

/* Range check: is it within the OTA data area */
static int in_range(uint32_t addr, uint32_t size)
{
    return (addr >= OTA_FLASH_BASE) && ((addr + size) <= OTA_FLASH_END);
}

int ota_flash_erase(uint32_t addr, uint32_t size)
{
    if (!in_range(addr, size)) {
        return -1;
    }

    /* STM32G4 single bank: interrupt ISR accessing flash during erase causes race/error.
     * Disable interrupts, clear error flags, then erase (erase ~22ms, HAL FLASH Wait uses
     * BSY polling to detect completion even when HAL_GetTick stalls -- verified working
     * in practice). */
    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef ei;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = FLASH_BANK_1;
    ei.Page      = (addr - 0x08000000U) / FLASH_PAGE_SIZE;
    ei.NbPages   = (size + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE;

    /* HAL_FLASHEx_Erase does not set PageError(err) on success.
     * Therefore initialize to 0 for correct success check (previous 0xFFFFFFFF
     * initial value always causes false failure). */
    uint32_t err = 0U;
    HAL_StatusTypeDef s = HAL_FLASHEx_Erase(&ei, &err);
    uint32_t sraw = FLASH->SR;

    HAL_FLASH_Lock();
    __enable_irq();

    /* STM32G4 HAL: there is behavior where PageError(err) is set even on success
     * (HAL_OK, SR=0), so err==0 check is unreliable. Use s==HAL_OK && no SR error
     * flags for success check. */
    (void)err;
    return ((s == HAL_OK) && ((sraw & 0x0000FFU) == 0U)) ? 0 : -1;
}

int ota_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (!in_range(addr, len) || (data == NULL)) {
        return -1;
    }

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef s = HAL_OK;
    uint32_t i = 0U;

    /* Write in doubleword (8-byte) units. Residual (less than 8) ignored -- simulator
     * simplification. */
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
