/**
 * @file    ota_flash.c
 * @brief   OTA flash erase/write (STM32G4 HAL)
 */
#include "ota_flash.h"
#include "bl_handoff.h"       /* APP_BASE/FLASH_END: BL flash map */
#include "stm32g4xx_hal_flash.h"
#include <string.h>

/* Range check: bootloader flash target -- app slot + meta page.
 * (Was the 8KB OTA region only; the bootloader owns the whole upper map now.) */
static int in_range(uint32_t addr, uint32_t size)
{
    return (addr >= APP_BASE) && ((addr + size) <= FLASH_END);
}

int ota_flash_erase(uint32_t addr, uint32_t size)
{
    if (!in_range(addr, size)) {
        return -1;
    }

    /* Erase one page per HAL call so the loop can re-enable IRQs and refresh
     * the IWDG between pages: the app may have started the watchdog before the
     * handoff reset (only POR stops it), and a whole-slot erase (~46 pages,
     * ~1s) without a refresh would trip the ~2s timeout mid-0x34. Per page:
     * IRQs off (STM32G4 single bank: ISR flash access during erase races the
     * controller; HAL FLASH Wait polls BSY, so completion is detected even
     * when HAL_GetTick stalls), then IRQs on + IWDG reload. */
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef ei;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = FLASH_BANK_1;
    ei.Page      = (addr - 0x08000000U) / FLASH_PAGE_SIZE;
    ei.NbPages   = 1U;

    uint32_t pages = (size + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE;

    /* HAL quirks carried over from the batch erase: PageError(err) must start
     * at 0 and may be set even on success, so success is HAL_OK && SR error
     * flags clear and err itself is ignored. */
    for (uint32_t i = 0U; i < pages; i++) {
        uint32_t err = 0U;
        __disable_irq();
        HAL_StatusTypeDef s = HAL_FLASHEx_Erase(&ei, &err);
        uint32_t sraw = FLASH->SR;
        __enable_irq();
        (void)err;

        if ((s != HAL_OK) || ((sraw & 0x0000FFU) != 0U)) {
            HAL_FLASH_Lock();
            return -1;
        }

        IWDG->KR = IWDG_KEY_RELOAD;
        ei.Page++;
    }

    HAL_FLASH_Lock();
    return 0;
}

int ota_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (!in_range(addr, len) || (data == NULL)) {
        return -1;
    }

    HAL_FLASH_Unlock();
    __disable_irq();   /* same single-bank ISR/flash race as the erase path */
    HAL_StatusTypeDef s = HAL_OK;
    uint32_t i = 0U;

    /* Write in doubleword (8-byte) units. Sub-8 residual is padded with 0xFF
     * (erased state) so trailing bytes are preserved -- the old simulator
     * simplification (drop residual) corrupted odd-sized images. */
    while (i < len) {
        uint8_t chunk[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        uint32_t n = (len - i) >= 8U ? 8U : (len - i);
        (void)memcpy(chunk, &data[i], n);
        uint64_t dw;
        (void)memcpy(&dw, chunk, 8U);
        s = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dw);
        if (s != HAL_OK) {
            break;
        }
        i += 8U;
    }

    __enable_irq();
    HAL_FLASH_Lock();
    return (s == HAL_OK) ? 0 : -1;
}
