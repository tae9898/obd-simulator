/**
 * @file    main.h
 * @brief   OBD-II ECU 시뮬레이터 메인 헤더
 * @note    STM32G431RB Nucleo 보드용 핀 정의, 클럭 설정, HAL include
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* === HAL 헤더 포함 === */
#include "stm32g4xx_hal.h"

/* === FreeRTOS 헤더 === */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* === CAN 수신 메시지 구조체 (ISR → Task 전달용) === */
typedef struct {
    uint32_t can_id;        /**< 수신 CAN ID */
    uint8_t  data[16];      /**< 수신 데이터 (CAN-FD 최대 16바이트) */
    uint8_t  dlc;           /**< 데이터 길이 */
} CAN_RxMessage_t;

/** CAN RX Queue 깊이: 동시에 대기 가능한 최대 메시지 수 */
#define CAN_RX_QUEUE_LEN  8

/** CAN RX Queue 핸들 (ISR과 Task가 공유) */
extern QueueHandle_t xCanRxQueue;

/** UART Mutex 핸들 (Debug_Print 스레드 안전성 보장) */
extern SemaphoreHandle_t xUartMutex;

/* === 핀 정의 === */
/** FDCAN1 RX - MCP2562FD RXD (PA11) */
#define FDCAN1_RX_PIN         GPIO_PIN_11
#define FDCAN1_RX_PORT        GPIOA
#define FDCAN1_RX_AF          GPIO_AF9_FDCAN1

/** FDCAN1 TX - MCP2562FD TXD (PA12) */
#define FDCAN1_TX_PIN         GPIO_PIN_12
#define FDCAN1_TX_PORT        GPIOA
#define FDCAN1_TX_AF          GPIO_AF9_FDCAN1

/** USART2 TX - ST-LINK VCP 디버그 출력 (PA2) */
#define USART2_TX_PIN         GPIO_PIN_2
#define USART2_TX_PORT        GPIOA
#define USART2_TX_AF          GPIO_AF7_USART2

/** USART2 RX - ST-LINK VCP 디버그 입력 (PA3) */
#define USART2_RX_PIN         GPIO_PIN_3
#define USART2_RX_PORT        GPIOA
#define USART2_RX_AF          GPIO_AF7_USART2

/** 상태 표시 LED - LD4 (PA5, 활성 Low) */
#define LED_PIN               GPIO_PIN_5
#define LED_PORT              GPIOA
#define LED_ON()              HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()             HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_TOGGLE()          HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

/* === 클럭 설정 상수 === */
/* HSE_VALUE, HSI_VALUE는 stm32g4xx_hal_conf.h에서 정의됨 */

/** 시스템 클럭 목표 주파수 (Hz) - PLL 최대 170MHz */
#define SYSCLK_FREQ           170000000U

/** AHB 버스 주파수 (Hz) */
#define HCLK_FREQ             SYSCLK_FREQ

/** APB1 버스 주파수 (Hz) - APB1 prescaler = 4 */
#define PCLK1_FREQ            (SYSCLK_FREQ / 4U)

/** APB2 버스 주파수 (Hz) - APB2 prescaler = 2 */
#define PCLK2_FREQ            (SYSCLK_FREQ / 2U)

/* === FDCAN 클럭 설정 === */
/**
 * FDCAN 클럭 소스: HSE 8MHz (PLLQ가 아닌 HSE 직접 사용)
 * Classic CAN 500kbps 설정 (아비트레이션 페이스):
 *   bitrate = fcan / (prescaler * (1 + TimeSegment1 + TimeSegment2))
 *   8MHz / (1 * (1 + 13 + 2)) = 500kbps
 */
#define FDCAN_CLK_FREQ        HSE_VALUE
#define FDCAN_PRESCALER       1U
#define FDCAN_TIME_SEG1       13U
#define FDCAN_TIME_SEG2       2U
#define FDCAN_SJW             1U

/* === CAN-FD 데이터 페이스 설정 === */
/**
 * CAN-FD 데이터 페이스 2Mbps:
 *   bitrate = fcan / (prescaler * (1 + TimeSegment1 + TimeSegment2))
 *   8MHz / (1 * (1 + 2 + 1)) = 8MHz / 4 = 2Mbps
 *
 * CAN-FD는 두 개의 비트 전송 속도를 가짐:
 *   - 아비트레이션 페이스: 기존 500kbps (버스 충돌 판정용)
 *   - 데이터 페이스: 2Mbps (실제 데이터 전송, BRS 활성화 시에만)
 */
#define FDCAN_DATA_PRESCALER  1U
#define FDCAN_DATA_TIME_SEG1  2U
#define FDCAN_DATA_TIME_SEG2  1U
#define FDCAN_DATA_SJW        1U

/* === OBD-II CAN ID 정의 === */
#define OBD2_REQUEST_ID       0x7E0U
#define OBD2_RESPONSE_ID      0x7E8U

/* === 시뮬레이션 파라미터 === */
/** 메인 루프 업데이트 주기 (ms) */
#define SIM_UPDATE_PERIOD_MS  10U

/** RPM 시뮬레이션 범위 */
#define RPM_IDLE              800U
#define RPM_MAX               4000U
/** RPM 변화 속도 (RPM per 10ms tick) */
#define RPM_RAMP_STEP         10U

/** 냉각수 온도 범위 (Celsius) */
#define COOLANT_TEMP_MIN      80U
#define COOLANT_TEMP_MAX      105U
/** 온도 변화 속도 (0.1°C per 10ms tick) */
#define COOLANT_TEMP_STEP     1U

/** 차속 범위 (km/h) */
#define VEHICLE_SPEED_MIN     0U
#define VEHICLE_SPEED_MAX     120U
/** 차속 변화 속도 (km/h per 10ms tick) */
#define VEHICLE_SPEED_STEP    1U

/* === LED 깜빡임 주기 === */
#define LED_TOGGLE_PERIOD_MS  500U

/* === 외부 변수 (다른 소스에서 참조) === */
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart2;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
