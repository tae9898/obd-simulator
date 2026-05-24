/**
 * @file    stm32g4xx_it.c
 * @brief   인터럽트 핸들러
 * @note    FDCAN1 RX + FreeRTOS SysTick
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

/**
 * @brief  FDCAN1 IT0 인터럽트 핸들러
 *
 * @note   처리 흐름:
 *         FDCAN1_IT0 인터럽트 → HAL_FDCAN_IRQHandler
 *         → HAL_FDCAN_RxFifo0Callback → ISO_TP_ProcessFrame
 *
 *         FreeRTOS 환경에서는 이 핸들러가 FreeRTOS API를
 *         호출하지 않으므로 우선순위 제약 없음.
 *         Phase 2 Step 2에서 xQueueSendFromISR 추가 시
 *         우선순위를 6 이하로 설정해야 함.
 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/**
 * @brief  SysTick 핸들러 (HAL + FreeRTOS 공유)
 *
 * @note   1ms 주기로 호출됨:
 *         1. HAL_IncTick() - HAL_Delay() / HAL_GetTick() 유지
 *         2. xPortSysTickHandler() - FreeRTOS 틱 (태스크 스케줄링)
 *
 *         순서 중요: HAL_IncTick 먼저 (빠름),
 *         xPortSysTickHandler 나중 (컨텍스트 스위치 발생 가능)
 *
 *         PendSV_Handler와 SVC_Handler는 FreeRTOS port.c에서
 *         정의함 (FreeRTOSConfig.h 매핑 참조).
 *         여기에 정의하면 안 됨 (중복 정의 오류).
 */
void SysTick_Handler(void)
{
    HAL_IncTick();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
