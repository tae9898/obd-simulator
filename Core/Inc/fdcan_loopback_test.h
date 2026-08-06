/**
 * @file    fdcan_loopback_test.h
 * @brief   FDCAN1 internal loopback diagnostic test header
 * @note    MCU FDCAN peripheral health check.
 *
 *          Completely bypasses external transceiver (MCP) / wiring / CAN pins
 *          and runs TX -> RX loopback inside the FDCAN core, verifying only that
 *          "peripheral + HAL + filter + RX FIFO0 path" is functional.
 *
 *          <Intent>
 *          - PASS : MCU peripheral normal. RX issue cause is external (transceiver RXD,
 *                   PA11 damage, wiring, level, etc.). No board replacement needed.
 *          - FAIL : MCU FDCAN peripheral/configuration issue. Use dump to identify cause.
 *
 *          <Enable>
 *          Set #define RUN_FDCAN_LOOPBACK_TEST 1 at the top of main.c, then build/flash.
 */

#ifndef __FDCAN_LOOPBACK_TEST_H
#define __FDCAN_LOOPBACK_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief  FDCAN1 internal loopback diagnostic run (does not return - infinite loop)
 * @note   Reports each round result via UART debug output.
 *         Call after HAL_Init/SystemClock/UART initialization.
 */
void FDCAN_LoopbackTest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_LOOPBACK_TEST_H */
