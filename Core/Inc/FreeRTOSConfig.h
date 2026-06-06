/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS 설정 헤더
 * @note    STM32G431RB (Cortex-M4F @ 170MHz) 전용
 *
 * 핵심 설정 설명:
 *   - configTICK_RATE_HZ = 1000: 1ms 틱 (RTOS 스케줄링 최소 단위)
 *   - configTOTAL_HEAP_SIZE = 8192: FreeRTOS 동적 할당 힙 (태스크 TCB + 스택 + 큐)
 *   - configCHECK_FOR_STACK_OVERFLOW = 2: 스택 오버플로우 감지 (canary + 포인터 검사)
 *   - configMAX_SYSCALL_INTERRUPT_PRIORITY = 5: ISR에서 FreeRTOS API 호출 가능한 최고 우선순위
 *     (FDCAN ISR은 우선순위 6 이하로 설정해야 xQueueSendFromISR 호출 가능)
 *
 * RAM 예산 (32KB 중):
 *   - Phase 1 기존 사용: ~3.2KB
 *   - FreeRTOS 힙: 8KB (태스크 3개 + 큐 + 스택)
 *   - 남은 여유: ~20KB (Phase 3 추가용)
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* === CPU 설정 === */
#define configCPU_CLOCK_HZ                  170000000UL  /* SYSCLK 170MHz */
#define configTICK_RATE_HZ                  1000U        /* 1ms 틱 */

/* === 스케줄링 === */
#define configUSE_PREEMPTION                1    /* 선점형 스케줄링 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1  /* Cortex-M CLZ 명령어 활용 */
#define configUSE_TIME_SLICING              1    /* 같은 우선순위 태스크 라운드로빈 */
#define configMAX_PRIORITIES                5
#define configMINIMAL_STACK_SIZE            128U /* 단위: word (128 * 4 = 512 bytes) */
#define configMAX_TASK_NAME_LEN             8
#define configUSE_16_BIT_TICKS              0    /* 32-bit tick counter */
#define configIDLE_SHOULD_YIELD             1

/* === 메모리 관리 === */
#define configSUPPORT_STATIC_ALLOCATION     0    /* 동적 할당만 사용 */
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configTOTAL_HEAP_SIZE               (12U * 1024U) /* 12KB */

/* === 동기화 === */
#define configUSE_MUTEXES                   1
#define configUSE_RECURSIVE_MUTEXES         0
#define configUSE_COUNTING_SEMAPHORES       1
#define configUSE_QUEUE_SETS                0
#define configQUEUE_REGISTRY_SIZE           4
#define configUSE_TASK_NOTIFICATIONS        1

/* === 훅 함수 === */
#define configUSE_IDLE_HOOK                 0
#define configUSE_TICK_HOOK                 0
#define configCHECK_FOR_STACK_OVERFLOW      2    /* canary + 포인터 검사 */
#define configUSE_MALLOC_FAILED_HOOK        1

/* === 타이머 (필요시 활성화) === */
#define configUSE_TIMERS                    0

/* === Newlib 스레드 안전성 ===
 * 0으로 설정: printf가 하나의 태스크에서만 호출될 때 메모리 절약
 * Phase 2 Step 2에서 ISR→Task 분리 후 문제 없음
 */
#define configUSE_NEWLIB_REENTRANT          0

/* === Cortex-M4 인터럽트 우선순위 ===
 * STM32G4 NVIC: 4-bit priority (NVIC_PRIORITYGROUP_4)
 * 숫자가 작을수록 높은 우선순위
 *
 * 우선순위 배정:
 *   0-4: FreeRTOS API 호출 불가 (하드웨어 실시간 인터럽트용)
 *   5-15: FreeRTOS API 호출 가능
 *   15 (0xF0): SysTick (가장 낮은 우선순위, 자동 설정)
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15U
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5U
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << 4U)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << 4U)

/* === 인터럽트 핸들러 매핑 ===
 * FreeRTOS port.c 함수명 → STM32 벡터테이블 함수명
 * 이 매핑으로 port.c가 SVC_Handler, PendSV_Handler를 직접 정의함
 * 따라서 stm32g4xx_it.c에서 제거해야 함 (중복 정의 방지)
 */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler

/* === Assert === */
#define configASSERT(x) \
    if ((x) == 0) { \
        taskDISABLE_INTERRUPTS(); \
        for (;;); \
    }

/* === 필수 헤더 === */
#include <stdint.h>

/* === API 활성화 (미설정 시 기본값 0 = 비활성화) ===
 * 필요한 함수만 1로 설정하여 Flash 사용량 최소화
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
