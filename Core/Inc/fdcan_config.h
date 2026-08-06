/**
 * @file    fdcan_config.h
 * @brief   FDCAN1 configuration header
 * @note    CAN-FD 500kbps/2Mbps (BRS) initialization, TX/RX FIFO configuration, filter configuration
 */

#ifndef __FDCAN_CONFIG_H
#define __FDCAN_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === FDCAN initialization functions === */

/**
 * @brief  FDCAN1 CAN-FD mode initialization (arbitration 500kbps + data 2Mbps)
 * @param  hfdcan: FDCAN handle pointer
 * @retval HAL status (HAL_OK = success)
 * @note   CAN-FD mode, BRS (Bit Rate Switch) enabled
 *         - Arbitration phase: Classic 500kbps
 *         - Data phase: 2Mbps
 *         - Max DLC: 16 bytes (CAN-FD)
 */
HAL_StatusTypeDef FDCAN1_InitFD(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  FDCAN1 RX filter configuration (receive only CAN ID 0x7E0)
 * @param  hfdcan: FDCAN handle pointer
 * @retval HAL status
 */
HAL_StatusTypeDef FDCAN1_ConfigureFilters(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  FDCAN1 receive interrupt enable
 * @param  hfdcan: FDCAN handle pointer
 * @retval HAL status
 */
HAL_StatusTypeDef FDCAN1_StartNotification(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  FDCAN1 receive message callback
 * @param  hfdcan: FDCAN handle pointer
 * @param  RxFifo0ITs: RX FIFO0 interrupt flags
 * @retval None
 * @note   Callback registered via HAL_FDCAN_ActivateNotification
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs);

/**
 * @brief  Byte count -> FDCAN DLC code (raw, 0~15) conversion
 * @param  bytes: actual data byte count (0~64)
 * @retval HAL FDCAN DLC code (same encoding as FDCAN_DLC_BYTES_* macros)
 * @note   HAL rule: pass raw code to TxHeader.DataLength
 *         (HAL internally shifts <<16). Rounded up to valid CAN-FD DLC sizes
 *         (8/12/16/20/24/32/48/64).
 */
uint32_t FDCAN_BytesToDlc(uint8_t bytes);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_CONFIG_H */
