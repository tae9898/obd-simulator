/**
 * @file    system_stm32g4xx_bl.c
 * @brief   Bootloader 용 SystemInit (app 의 system_stm32g4xx.c 와 별개 파일)
 * @note    빌드는 동일 심볼(SystemInit/SystemCoreClock/...)을 링크하되
 *          Makefile 이 bootloader 빌드에선 이 파일만 포함.
 *
 *          FPU 활성화 + VTOR = 0x08000000 (bootloader 자체 벡터).
 *          클럭은 HSI 16MHz 그대로 — bootloader는 PLL 설정 안 함(app 이 수행).
 */

#include "stm32g4xx.h"

/* HSI_VALUE 는 stm32g4xx_hal_conf.h 에서 정의. bootloader 빌드에선
 * HAL 헤더가 포함 경로에 있으므로 여기서 직접 정의하지 않는다. */
uint32_t SystemCoreClock = 16000000U;  /* HSI 기본 — bootloader 는 PLL 안 함 */

const uint8_t AHBPrescTable[16] = {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U
};

const uint8_t APBPrescTable[8] = {
    0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U
};

void SystemInit(void)
{
    /* FPU (CP10/CP11) 활성화 */
    SCB->CPACR |= ((3U << (10U * 2U)) |
                   (3U << (11U * 2U)));

    /* VTOR = bootloader 자체 벡터 (Flash 시작) */
    SCB->VTOR = 0x08000000U;
}

/* ponytail: bootloader 는 PLL 안 쓰므로 SystemCoreClockUpdate 사실상 불필요.
 * 심볼 누락 링크 에러 방지용 최소 구현만 남김. */
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 16000000U;
}
