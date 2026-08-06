/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS configuration header
 * @note    STM32G431RB (Cortex-M4F @ 170MHz) specific
 *
 * Key configuration details:
 *   - configTICK_RATE_HZ = 1000: 1ms tick (RTOS scheduling minimum unit)
 *   - configTOTAL_HEAP_SIZE = 16384: FreeRTOS dynamic allocation heap (task TCB + stack + queue)
 *   - configCHECK_FOR_STACK_OVERFLOW = 2: stack overflow detection (canary + pointer check)
 *   - configMAX_SYSCALL_INTERRUPT_PRIORITY = 5: highest priority for FreeRTOS API calls from ISR
 *     (FDCAN ISR must be set to priority 6 or lower to call xQueueSendFromISR)
 *
 * RAM budget (of 32KB):
 *   - Phase 1 existing usage: ~3.2KB
 *   - FreeRTOS heap: 16KB (3 tasks + queue + stack)
 *   - Remaining free: ~20KB (for Phase 3 additions)
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* === CPU configuration === */
#define configCPU_CLOCK_HZ                  170000000UL  /* SYSCLK 170MHz */
#define configTICK_RATE_HZ                  1000U        /* 1ms tick */

/* === Scheduling === */
#define configUSE_PREEMPTION                1    /* Preemptive scheduling */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1  /* Cortex-M CLZ instruction utilization */
#define configUSE_TIME_SLICING              1    /* Round-robin for equal priority tasks */
#define configMAX_PRIORITIES                5
#define configMINIMAL_STACK_SIZE            128U /* Unit: word (128 * 4 = 512 bytes) */
#define configMAX_TASK_NAME_LEN             8
#define configUSE_16_BIT_TICKS              0    /* 32-bit tick counter */
#define configIDLE_SHOULD_YIELD             1

/* === Memory management === */
#define configSUPPORT_STATIC_ALLOCATION     0    /* Dynamic allocation only */
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configTOTAL_HEAP_SIZE               16384U /* 16KB */

/* === Synchronization === */
#define configUSE_MUTEXES                   1
#define configUSE_RECURSIVE_MUTEXES         0
#define configUSE_COUNTING_SEMAPHORES       1
#define configUSE_QUEUE_SETS                0
#define configQUEUE_REGISTRY_SIZE           4
#define configUSE_TASK_NOTIFICATIONS        1

/* === Hook functions === */
#define configUSE_IDLE_HOOK                 0
#define configUSE_TICK_HOOK                 0
#define configCHECK_FOR_STACK_OVERFLOW      2    /* canary + pointer check */
#define configUSE_MALLOC_FAILED_HOOK        1

/* === Timers (enable if needed) === */
#define configUSE_TIMERS                    0

/* === Newlib thread safety ===
 * Set to 0: saves memory when printf is called from only one task
 * No issue after ISR->Task separation in Phase 2 Step 2
 */
#define configUSE_NEWLIB_REENTRANT          0

/* === Cortex-M4 interrupt priority ===
 * STM32G4 NVIC: 4-bit priority (NVIC_PRIORITYGROUP_4)
 * Lower number = higher priority
 *
 * Priority assignment:
 *   0-4: FreeRTOS API calls not allowed (hardware real-time interrupts)
 *   5-15: FreeRTOS API calls allowed
 *   15 (0xF0): SysTick (lowest priority, auto-configured)
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15U
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5U
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << 4U)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << 4U)

/* === Interrupt handler mapping ===
 * FreeRTOS port.c function names -> STM32 vector table function names
 * This mapping lets port.c define SVC_Handler and PendSV_Handler directly
 * Therefore must be removed from stm32g4xx_it.c (prevent duplicate definition)
 */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler

/* === Assert === */
#define configASSERT(x) \
    if ((x) == 0) { \
        taskDISABLE_INTERRUPTS(); \
        for (;;); \
    }

/* === Required headers === */
#include <stdint.h>

/* === API enable (unset defaults to 0 = disabled) ===
 * Set only needed functions to 1 to minimize Flash usage
 */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 0
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xEventGroupSetBitFromISR        0
#define INCLUDE_xTimerPendFunctionCall          0
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xTaskGetHandle                  0
#define INCLUDE_xTaskResumeFromISR              1

#endif /* FREERTOS_CONFIG_H */
