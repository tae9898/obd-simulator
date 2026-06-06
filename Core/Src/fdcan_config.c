/**
 * @file    fdcan_config.c
 * @brief   FDCAN1 설정 구현
 * @note    Classic CAN 500kbps, TX/RX FIFO, ID 필터, 수신 인터럽트
 */

#include "fdcan_config.h"
#include "iso_tp.h"
#include "uart_debug.h"
#include <string.h>

/* === 수신 버퍼 (CAN-FD 최대 64바이트) === */
static uint8_t s_rx_data[64];
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
    hfdcan->Init.Mode                = FDCAN_MODE_INTERNAL_LOOPBACK;  /* 내부 루프백 (테스트용) */
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

    /* --- FDCAN 페리페럴 강제 리셋 (잔류 에러 카운터 초기화) --- */
    __HAL_RCC_FDCAN_FORCE_RESET();
    __HAL_RCC_FDCAN_RELEASE_RESET();

    /* --- HAL FDCAN 초기화 --- */
    status = HAL_FDCAN_Init(hfdcan);
    if (status != HAL_OK) {
        Debug_Print("[FDCAN] HAL_FDCAN_Init failed: %d\r\n", status);
        return status;
    }

    Debug_Print("[FDCAN] Init OK - Classic CAN 500kbps (LOOPBACK)\r\n");
    return HAL_OK;
}

/**
 * @brief  FDCAN1 CAN-FD 모드 초기화
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL_OK = 성공
 *
 * @note   Classic CAN과의 차이점:
 *         - FrameFormat: FDCAN_FRAME_FD_BRS (BRS 활성화)
 *         - DataPrescaler/DataTimeSeg1/DataTimeSeg2: 데이터 페이스 타이밍
 *         - 송신 시 FDCAN_BRS_ON, FDCAN_FD_CAN 플래그 사용
 *
 *         Classic CAN init과 비교해 변경된 것은 딱 3줄:
 *         1. FrameFormat → FDCAN_FRAME_FD_BRS
 *         2. DataPrescaler/DataTimeSeg1/DataTimeSeg2 → 실제 값 설정
 *         나머지(NominalPrescaler, 필터, FIFO 등)는 동일
 */
HAL_StatusTypeDef FDCAN1_InitFD(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_StatusTypeDef status;

    /* --- FDCAN 인스턴스 설정 --- */
    hfdcan->Instance                 = FDCAN1;
    hfdcan->Init.FrameFormat         = FDCAN_FRAME_FD_BRS;  /* CAN-FD with BRS */
    hfdcan->Init.Mode                = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission  = ENABLE;
    hfdcan->Init.TransmitPause       = DISABLE;
    hfdcan->Init.ProtocolException   = DISABLE;

    /* --- 아비트레이션 페이스: Classic CAN 500kbps (동일) --- */
    hfdcan->Init.NominalPrescaler     = FDCAN_PRESCALER;
    hfdcan->Init.NominalSyncJumpWidth = FDCAN_SJW;
    hfdcan->Init.NominalTimeSeg1      = FDCAN_TIME_SEG1;
    hfdcan->Init.NominalTimeSeg2      = FDCAN_TIME_SEG2;

    /* --- 데이터 페이스: 2Mbps (여기가 핵심 변경) --- */
    hfdcan->Init.DataPrescaler       = FDCAN_DATA_PRESCALER;  /* 1 */
    hfdcan->Init.DataSyncJumpWidth   = FDCAN_DATA_SJW;       /* 1 */
    hfdcan->Init.DataTimeSeg1        = FDCAN_DATA_TIME_SEG1;  /* 2 */
    hfdcan->Init.DataTimeSeg2        = FDCAN_DATA_TIME_SEG2;  /* 1 */

    /* --- 필터 및 FIFO 설정 (Classic CAN과 동일) --- */
    hfdcan->Init.StdFiltersNbr       = 1U;
    hfdcan->Init.ExtFiltersNbr       = 0U;
    hfdcan->Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;

    /* --- HAL FDCAN 초기화 --- */
    status = HAL_FDCAN_Init(hfdcan);
    if (status != HAL_OK) {
        Debug_Print("[FDCAN] HAL_FDCAN_Init (FD) failed: %d\r\n", status);
        return status;
    }

    Debug_Print("[FDCAN] Init OK - CAN-FD 500kbps/2Mbps (BRS)\r\n");
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

    /*
     * FDCAN 인터럽트 우선순위를 6으로 설정
     *
     * Cortex-M4 NVIC: 숫자가 작을수록 우선순위 높음 (0=최고, 15=최저)
     * FreeRTOS 규칙: 우선순위 5 이하(0~4)에서는 FromISR API 호출 불가
     * 우선순위 6이면 FromISR 호출 가능 → Queue 전송 OK
     */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

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
 *
 * @note   ISR → Queue → Task 흐름:
 *         1. RX FIFO0에서 메시지 읽기
 *         2. CAN_RxMessage_t에 복사
 *         3. xQueueSendFromISR로 Queue에 전달
 *         4. 즉시 리턴 (ISR는 최대한 짧게)
 *
 *         xQueueSendFromISR: ISR 전용 Queue 전송 함수
 *         - 일반 xQueueSend은 ISR에서 호출하면 안 됨 (데드락 위험)
 *         - FromISR 버전은 스케줄러를 직접 깨우지 않고,
 *           pxHigherPriorityTaskWoken으로 "컨텍스트 스위치 필요"만 표시
 *         - portYIELD_FROM_ISR이 실제 스위치를 수행
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs)
{
    HAL_StatusTypeDef status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {

        status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                         &s_rx_header, s_rx_data);
        if (status == HAL_OK) {
            CAN_RxMessage_t rx_msg;
            rx_msg.can_id = s_rx_header.Identifier;
            rx_msg.dlc    = (uint8_t)(s_rx_header.DataLength >> 16U);

            /* data 배열 복사 (최대 16바이트) */
            (void)memset(rx_msg.data, 0, sizeof(rx_msg.data));
            for (uint8_t i = 0U; i < rx_msg.dlc && i < 16U; i++) {
                rx_msg.data[i] = s_rx_data[i];
            }

            /* Queue에 전송 (ISR 전용 함수) */
            xQueueSendFromISR(xCanRxQueue, &rx_msg, &xHigherPriorityTaskWoken);
        }

        HAL_FDCAN_ActivateNotification(
            hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U
        );

        /* Queue 수신 태스크가 더 높은 우선순위면 즉시 스위치 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
