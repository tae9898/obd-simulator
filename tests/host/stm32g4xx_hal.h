/**
 * @file    stm32g4xx_hal.h
 * @brief   Host-side HAL stub for the DTC model unit test (empty: the DTC layer
 *          only needs stdint; pin macros in main.h are never expanded here)
 */
#ifndef __STM32G4XX_HAL_H
#define __STM32G4XX_HAL_H

#include <stdint.h>

/* Dummy handle typedefs: main.h declares externs of these (unused by the DTC layer) */
typedef struct { int dummy; } FDCAN_HandleTypeDef;
typedef struct { int dummy; } UART_HandleTypeDef;
typedef struct { int dummy; } IWDG_HandleTypeDef;

#endif /* __STM32G4XX_HAL_H */
