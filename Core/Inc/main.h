/**
 * @file    main.h
 * @brief   OBD-II ECU simulator main header
 * @note    STM32G431RB Nucleo board pin definitions, clock settings, HAL includes
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* === HAL header includes === */
#include "stm32g4xx_hal.h"

/* === FreeRTOS headers === */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* === CAN receive message structure (ISR -> Task delivery) === */
typedef struct {
    uint32_t can_id;        /**< Received CAN ID */
    uint8_t  data[64];      /**< Received data (CAN-FD max 64 bytes) */
    uint8_t  dlc;           /**< Data length (bytes, 0~64) */
} CAN_RxMessage_t;

/** CAN RX Queue depth: maximum simultaneous pending messages */
#define CAN_RX_QUEUE_LEN  8

/** CAN RX Queue handle (shared by ISR and Task) */
extern QueueHandle_t xCanRxQueue;

/** UART Mutex handle (ensure Debug_Print thread safety) */
extern SemaphoreHandle_t xUartMutex;

/* === Pin definitions === */
/** FDCAN1 RX - MCP2562FD RXD (PA11) -- PA group */
#define FDCAN1_RX_PIN         GPIO_PIN_11
#define FDCAN1_RX_PORT        GPIOA
#define FDCAN1_RX_AF          GPIO_AF9_FDCAN1

/** FDCAN1 TX - MCP2562FD TXD (PA12) -- PA group (PA11+PA12) */
#define FDCAN1_TX_PIN         GPIO_PIN_12
#define FDCAN1_TX_PORT        GPIOA
#define FDCAN1_TX_AF          GPIO_AF9_FDCAN1

/** USART2 TX - ST-LINK VCP debug output (PA2) */
#define USART2_TX_PIN         GPIO_PIN_2
#define USART2_TX_PORT        GPIOA
#define USART2_TX_AF          GPIO_AF7_USART2

/** USART2 RX - ST-LINK VCP debug input (PA3) */
#define USART2_RX_PIN         GPIO_PIN_3
#define USART2_RX_PORT        GPIOA
#define USART2_RX_AF          GPIO_AF7_USART2

/** Status indicator LED - LD4 (PA5, active Low) */
#define LED_PIN               GPIO_PIN_5
#define LED_PORT              GPIOA
#define LED_ON()              HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()             HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_TOGGLE()          HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

/** RS485 DE/RE direction control (PA8, MAX485) -- HIGH=transmit, LOW=receive */
#define RS485_DE_PIN          GPIO_PIN_8
#define RS485_DE_PORT         GPIOA
#define RS485_DE_HIGH()       HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_DE_LOW()        HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)

/* === Clock configuration constants === */
/* HSE_VALUE, HSI_VALUE are defined in stm32g4xx_hal_conf.h */

/** System clock target frequency (Hz) - PLL max 170MHz */
#define SYSCLK_FREQ           170000000U

/** AHB bus frequency (Hz) */
#define HCLK_FREQ             SYSCLK_FREQ

/** APB1 bus frequency (Hz) - APB1 prescaler = 4 */
#define PCLK1_FREQ            (SYSCLK_FREQ / 4U)

/** APB2 bus frequency (Hz) - APB2 prescaler = 2 */
#define PCLK2_FREQ            (SYSCLK_FREQ / 2U)

/* === FDCAN clock configuration ===
 * CAN-FD BRS @ HSE 24MHz (arbitration 500kbps + data 2Mbps):
 *   nominal : 24MHz / (4 * (1 + 9 + 2)) = 24MHz / 48 = 500kbps, SP = (1+9)/12 = 83.3%
 *   data    : 24MHz / (1 * (1 + 9 + 2)) = 24MHz / 12 = 2Mbps,   SP = 83.3%
 *   Both phases use 12 TQ (SEG1=9/SEG2=2/SJW=2), only prescaler differs: 4(nominal) vs 1(data).
 *   HSE crystal accuracy +/-50ppm -> sufficient inter-node sync margin.
 */
#define FDCAN_CLK_FREQ        24000000U
#define FDCAN_PRESCALER       4U
#define FDCAN_TIME_SEG1       9U
#define FDCAN_TIME_SEG2       2U
#define FDCAN_SJW             2U

/* === CAN-FD data phase (BRS, 2Mbps) === */
#define FDCAN_DATA_PRESCALER  1U
#define FDCAN_DATA_TIME_SEG1  9U
#define FDCAN_DATA_TIME_SEG2  2U
#define FDCAN_DATA_SJW        2U

/* === OBD-II CAN ID definitions === */
#define OBD2_REQUEST_ID       0x7E0U
#define OBD2_RESPONSE_ID      0x7E8U

/* === Simulation parameters === */
/** Main loop update period (ms) */
#define SIM_UPDATE_PERIOD_MS  10U

/** RPM simulation range */
#define RPM_IDLE              800U
#define RPM_MAX               4000U
/** RPM change rate (RPM per 10ms tick) */
#define RPM_RAMP_STEP         10U

/** Coolant temperature range (Celsius) */
#define COOLANT_TEMP_MIN      80U
#define COOLANT_TEMP_MAX      105U
/** Temperature change rate (0.1 deg-C per 10ms tick) */
#define COOLANT_TEMP_STEP     1U

/** Vehicle speed range (km/h) */
#define VEHICLE_SPEED_MIN     0U
#define VEHICLE_SPEED_MAX     120U
/** Vehicle speed change rate (km/h per 10ms tick) */
#define VEHICLE_SPEED_STEP    1U

/* === LED blink period === */
#define LED_TOGGLE_PERIOD_MS  500U

/* === External variables (referenced from other sources) === */
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef  huart2;
extern UART_HandleTypeDef  huart1;
extern IWDG_HandleTypeDef  hiwdg;

/* === Task alive flags (for IWDG supervision) === */
extern volatile uint8_t g_task_alive_flags;
#define TASK_ALIVE_MAIN   (1U << 0U)
#define TASK_ALIVE_CAN_RX (1U << 1U)
#define TASK_ALIVE_RS485  (1U << 2U)
#define TASK_ALIVE_ALL    (TASK_ALIVE_MAIN | TASK_ALIVE_CAN_RX | TASK_ALIVE_RS485)

/* === FDCAN bus-off event (ISR -> Task) === */
extern volatile uint8_t g_fdcan_busoff_detected;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
