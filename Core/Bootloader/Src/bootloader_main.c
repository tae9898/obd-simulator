/**
 * @file    bootloader_main.c
 * @brief   Phase 4.2 Bootloader 1단계 — app 영역(0x08004000)으로 점프
 * @note    HAL/FreeRTOS 없이 레지스터 직접 제어. PLL 설정 안 함(app이 수행).
 *          OTA 수신/CRC는 2단계. 1단계에선 유효한 app이 있으면 즉시 점프.
 */

#include "stm32g4xx.h"

#define APP_ADDRESS 0x08004000U
#define RAM_BASE    0x20000000U
#define RAM_END     0x20008000U

typedef void (*pFunc)(void);

/* app 영역 첫 워드(초기 SP)가 유효한 RAM 주소인지 검사.
 * ponytail: 1단계 검증용 — app이 없거나 손상되면 점프 중단하고 대기.
 * SP = _estack = 0x20008000 (RAM 끝, 스택은 아래로 자람) 도 유효하므로 <=. */
static int app_is_valid(void)
{
    uint32_t app_sp = *(volatile uint32_t *)APP_ADDRESS;
    return (app_sp >= RAM_BASE) && (app_sp <= RAM_END);
}

void bootloader_jump_to_app(void)
{
    uint32_t app_sp    = *(volatile uint32_t *)APP_ADDRESS;
    pFunc   app_reset  = (pFunc)(*(volatile uint32_t *)(APP_ADDRESS + 4U));

    __set_MSP(app_sp);          /* app 초기 스택 포인터 */
    SCB->VTOR = APP_ADDRESS;    /* app 벡터 테이블 (0x200 정렬 만족) */
    app_reset();                /* app Reset_Handler 로 점프 */
}

int main(void)
{
    if (app_is_valid()) {
        bootloader_jump_to_app();
    }

    /* app이 유효하지 않으면 대기 (1단계: 진단용 LED/UART 없음 — 2단계에서 보강) */
    while (1) {
        __WFI();
    }
}
