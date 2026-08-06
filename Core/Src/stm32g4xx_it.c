/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt handlers
 * @note    FDCAN1 RX + FreeRTOS SysTick
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

/**
 * @brief  FDCAN1 IT0 interrupt handler
 *
 * @note   Processing flow:
 *         FDCAN1_IT0 interrupt -> HAL_FDCAN_IRQHandler
 *         -> HAL_FDCAN_RxFifo0Callback -> ISO_TP_ProcessFrame
 *
 *         In FreeRTOS environment, this handler does not call FreeRTOS API
 *         so there is no priority constraint.
 *         When adding xQueueSendFromISR in Phase 2 Step 2,
 *         priority must be set to 6 or lower.
 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/**
 * @brief  SysTick handler (HAL + FreeRTOS shared)
 *
 * @note   Called at 1ms interval:
 *         1. HAL_IncTick() - maintains HAL_Delay() / HAL_GetTick()
 *         2. xPortSysTickHandler() - FreeRTOS tick (task scheduling)
 *
 *         Order matters: HAL_IncTick first (fast),
 *         xPortSysTickHandler later (may trigger context switch)
 *
 *         PendSV_Handler and SVC_Handler are defined in FreeRTOS port.c
 *         (see FreeRTOSConfig.h mapping).
 *         Must not be defined here (duplicate definition error).
 */
void SysTick_Handler(void)
{
    HAL_IncTick();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/**
 * @brief  USART1 interrupt handler (RS485 receive)
 * @note   Priority 7 (lower than FDCAN=6, FreeRTOS API calls allowed)
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief  FDCAN1 IT1 interrupt handler (error only)
 * @note   IT0 = RX messages, IT1 = error events (Warning/Passive/BusOff)
 *         HAL_FDCAN_IRQHandler checks error flags and calls
 *         HAL_FDCAN_ErrorStatusCallback / HAL_FDCAN_ErrorCallback
 */
void FDCAN1_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}
