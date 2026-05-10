/**
 * @file    fdcan_config.h
 * @brief   FDCAN1 설정 헤더
 * @note    Classic CAN 500kbps 초기화, TX/RX FIFO 설정, 필터 설정
 */

#ifndef __FDCAN_CONFIG_H
#define __FDCAN_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === FDCAN 초기화 함수 === */

/**
 * @brief  FDCAN1 주변기기 초기화
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL 상태 (HAL_OK = 성공)
 * @note   Classic CAN 모드, 500kbps, 11-bit ID
 *         - TX FIFO 사용
 *         - RX FIFO0 사용
 *         - 필터: CAN ID 0x7E0만 수신
 */
HAL_StatusTypeDef FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  FDCAN1 CAN-FD 모드 초기화 (아비트레이션 500kbps + 데이터 2Mbps)
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL 상태 (HAL_OK = 성공)
 * @note   CAN-FD 모드, BRS(Bit Rate Switch) 활성화
 *         - 아비트레이션 페이스: Classic 500kbps
 *         - 데이터 페이스: 2Mbps
 *         - 최대 DLC: 16바이트 (CAN-FD)
 */
HAL_StatusTypeDef FDCAN1_InitFD(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  FDCAN1 RX 필터 설정 (CAN ID 0x7E0만 수신)
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL 상태
 */
HAL_StatusTypeDef FDCAN1_ConfigureFilters(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  FDCAN1 수신 인터럽트 활성화
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL 상태
 */
HAL_StatusTypeDef FDCAN1_StartNotification(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  FDCAN1 수신 메시지 처리 콜백
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @param  RxFifo0ITs: RX FIFO0 인터럽트 플래그
 * @retval None
 * @note   HAL_FDCAN_ActivateNotification 등록 콜백
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_CONFIG_H */
