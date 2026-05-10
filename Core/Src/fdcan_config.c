/**
 * @file    fdcan_config.c
 * @brief   FDCAN1 설정 구현
 * @note    Classic CAN 500kbps, TX/RX FIFO, ID 필터, 수신 인터럽트
 */

#include "fdcan_config.h"
#include "obd2_simulator.h"
#include "uart_debug.h"

/* === 수신 버퍼 (인터럽트 핸들러에서 사용) === */
static uint8_t s_rx_data[8];
static FDCAN_RxHeaderTypeDef s_rx_header;

/**
 * @brief  FDCAN1 주변기기 초기화
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL_OK = 성공
 *
 * @note   설정:
 *         - Classic CAN 모드 (CAN-FD 미사용)
 *         - 500kbps @ 8MHz FDCAN 클럭 (HSE)
 *         - Prescaler=1, TimeSegment1=13, TimeSegment2=2, SJW=1
 *         - 11-bit 표준 ID
 *         - TX FIFO: 큐 모드, 3개 엘리먼트
 *         - RX FIFO0: 5개 엘리먼트
 *         - 자동 재전송 비활성화
 */
HAL_StatusTypeDef FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_StatusTypeDef status;

    /* --- FDCAN 인스턴스 설정 --- */
    hfdcan->Instance                 = FDCAN1;
    hfdcan->Init.FrameFormat         = FDCAN_FRAME_CLASSIC;  /* Classic CAN */
    hfdcan->Init.Mode                = FDCAN_MODE_NORMAL;    /* 노멀 모드 */
    hfdcan->Init.AutoRetransmission  = ENABLE;   /* 자동 재전송 활성화 */
    hfdcan->Init.TransmitPause       = DISABLE;  /* 전송 일시정지 비활성화 */
    hfdcan->Init.ProtocolException   = DISABLE;  /* 프로토콜 예외 비활성화 */

    /* --- 비트 타이밍 설정 (Classic CAN 500kbps) --- */
    /*
     * bitrate = fcan / (prescaler * (1 + TimeSegment1 + TimeSegment2))
     * 8MHz / (1 * (1 + 13 + 2)) = 8MHz / 16 = 500kbps
     */
    hfdcan->Init.NominalPrescaler    = FDCAN_PRESCALER;     /* 1 */
    hfdcan->Init.NominalSyncJumpWidth = FDCAN_SJW;          /* 1 */
    hfdcan->Init.NominalTimeSeg1     = FDCAN_TIME_SEG1;    /* 13 */
    hfdcan->Init.NominalTimeSeg2     = FDCAN_TIME_SEG2;    /* 2 */

    /* --- 데이터 비트 타이밍 (Classic CAN에서는 미사용, 초기화만) --- */
    hfdcan->Init.DataPrescaler       = 1U;
    hfdcan->Init.DataSyncJumpWidth   = 1U;
    hfdcan->Init.DataTimeSeg1        = 1U;
    hfdcan->Init.DataTimeSeg2        = 1U;

    /* --- 필터 개수 설정 --- */
    hfdcan->Init.StdFiltersNbr       = 1U;   /* 표준 필터 1개 */
    hfdcan->Init.ExtFiltersNbr       = 0U;   /* 확장 필터 없음 */
    hfdcan->Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;  /* FIFO 모드 */

    /* --- HAL FDCAN 초기화 --- */
    status = HAL_FDCAN_Init(hfdcan);
    if (status != HAL_OK) {
        Debug_Print("[FDCAN] HAL_FDCAN_Init failed: %d\r\n", status);
        return status;
    }

    Debug_Print("[FDCAN] Init OK - Classic CAN 500kbps\r\n");
    return HAL_OK;
}

/**
 * @brief  FDCAN1 RX 필터 설정 (CAN ID 0x7E0만 수신)
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL_OK = 성공
 *
 * @note   필터 0 설정:
 *         - 필터 유형: 마스크 모드
 *         - ID 유형: 표준 (11-bit)
 *         - 필터 ID: 0x7E0
 *         - 필터 마스크: 0x7FF (전체 비트 일치)
 *         - FIFO 할당: FIFO0
 */
HAL_StatusTypeDef FDCAN1_ConfigureFilters(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter_config;

    /* 필터 구조체 초기화 */
    filter_config.IdType       = FDCAN_STANDARD_ID;
    filter_config.FilterIndex  = 0U;
    filter_config.FilterType   = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1    = OBD2_REQUEST_ID;  /* 0x7E0 */
    filter_config.FilterID2    = 0x7FFU;           /* 전체 비트 일치 마스크 */

    /* HAL 필터 설정 API 호출 */
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
        Debug_Print("[FDCAN] Filter config failed\r\n");
        return HAL_ERROR;
    }

    Debug_Print("[FDCAN] Filter OK - ID 0x%03X -> FIFO0\r\n", OBD2_REQUEST_ID);
    return HAL_OK;
}

/**
 * @brief  FDCAN1 수신 인터럽트 활성화
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL_OK = 성공
 *
 * @note   활성화 인터럽트:
 *         - RX FIFO0 새 메시지 수신
 */
HAL_StatusTypeDef FDCAN1_StartNotification(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_StatusTypeDef status;

    /* RX FIFO0 새 메시지 인터럽트 활성화 */
    status = HAL_FDCAN_ActivateNotification(
        hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0U  /* FIFO0 워터마크: 0 = 모든 메시지에 대해 인터럽트 */
    );

    if (status != HAL_OK) {
        Debug_Print("[FDCAN] Notification activate failed\r\n");
        return status;
    }

    Debug_Print("[FDCAN] RX interrupt enabled\r\n");
    return HAL_OK;
}

/**
 * @brief  FDCAN1 RX FIFO0 새 메시지 콜백 (HAL 등록)
 * @param  hfdcan:     FDCAN 핸들러 포인터
 * @param  RxFifo0ITs: FIFO0 인터럽트 플래그
 * @retval None
 *
 * @note   인터럽트 컨텍스트에서 호출됨:
 *         1. RX FIFO0에서 메시지 읽기
 *         2. OBD2_ProcessRequest()로 요청 처리
 *         3. RX FIFO0 ack (버퍼 해제)
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs)
{
    HAL_StatusTypeDef status;

    /* --- 새 메시지 플래그 확인 --- */
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {

        /* --- RX FIFO0에서 메시지 읽기 --- */
        status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                         &s_rx_header, s_rx_data);
        if (status == HAL_OK) {
            /* --- OBD-II 요청 처리 --- */
            OBD2_ProcessRequest(&s_rx_header, s_rx_data);
        }

        /* --- RX FIFO0 ack (메시지 처리 완료, 버퍼 해제) --- */
        HAL_FDCAN_ActivateNotification(
            hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U
        );
    }
}
