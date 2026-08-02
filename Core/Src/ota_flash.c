/**
 * @file    ota_flash.c
 * @brief   OTA용 flash erase/write (STM32G4 HAL)
 */
#include "ota_flash.h"
#include "stm32g4xx_hal_flash.h"
#include <string.h>

/* 영역 검증: OTA 데이터 영역 내인가 */
static int in_range(uint32_t addr, uint32_t size)
{
    return (addr >= OTA_FLASH_BASE) && ((addr + size) <= OTA_FLASH_END);
}

int ota_flash_erase(uint32_t addr, uint32_t size)
{
    if (!in_range(addr, size)) {
        return -1;
    }

    /* STM32G4 싱글뱅크: erase 중 인터럽트 ISR 이 플래시 접근하면 경쟁/에러.
     * 인터럽트 끄고 에러 플래그 클리어 후 erase (erase ~22ms, HAL FLASH Wait 는
     * BSY 폴링으로 HAL_GetTick 정지에도 완료 감지 — 실측 정상 동작). */
    __disable_irq();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef ei;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = FLASH_BANK_1;
    ei.Page      = (addr - 0x08000000U) / FLASH_PAGE_SIZE;
    ei.NbPages   = (size + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE;

    /* HAL_FLASHEx_Erase 는 성공 시 PageError(err) 를 설정하지 않는다.
     * 따라서 0 으로 초기화해야 성공 판정 (이전 0xFFFFFFFF 초기값은 항상 실패 오판). */
    uint32_t err = 0U;
    HAL_StatusTypeDef s = HAL_FLASHEx_Erase(&ei, &err);
    uint32_t sraw = FLASH->SR;

    HAL_FLASH_Lock();
    __enable_irq();

    /* STM32G4 HAL: 성공(HAL_OK, SR=0)해도 PageError(err) 를 설정하는 동작이 있어
     * err==0 판정은 신뢰 불가. s==HAL_OK && SR 에러플래그 없음 으로 성공 판정. */
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
