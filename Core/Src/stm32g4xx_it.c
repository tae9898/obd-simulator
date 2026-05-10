/**
 * @file    stm32g4xx_it.c
 * @brief   인터럽트 핸들러
 * @note    FDCAN1 RX 인터럽트 핸들러 (OBD-II 요청 수신)
 */

#include "main.h"

/**
 * @brief  FDCAN1 IT0 인터럽트 핸들러
 * @retval None
 *
 * @note   FDCAN1_IT0_IRQn = IRQ 22 (STM32G431)
 *         HAL 인터럽트 디스패처를 통해
 *         HAL_FDCAN_IRQHandler() -> HAL_FDCAN_RxFifo0Callback() 호출됨
 *
 *         처리 흐름:
 *         1. MCP2562FD 트랜시버가 CAN 버스 메시지 수신
 *         2. FDCAN1 하드웨어가 RX FIFO0에 메시지 저장
 *         3. FDCAN1_IT0 인터럽트 발생
 *         4. 이 핸들러에서 HAL_FDCAN_IRQHandler 호출
 *         5. HAL이 HAL_FDCAN_RxFifo0Callback 콜백 호출
 *         6. 콜백에서 OBD2_ProcessRequest() 실행
 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/**
 * @brief  PendSV 핸들러 (RTOS용, Phase 0에서는 미사용)
 * @retval None
 */
void PendSV_Handler(void)
{
    /* 예약됨 - RTOS 컨텍스트 스위칭에 사용 */
}

/**
 * @brief  SysTick 핸들러 (HAL 타이머 베이스, 1ms 주기)
 * @retval None
 *
 * @note   HAL_Init()에서 HAL_InitTick()이 등록함
 *         HAL_Delay() 등 HAL 타이밍 함수에서 사용됨
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
