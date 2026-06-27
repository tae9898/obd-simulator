/**
 * @file    fdcan_config.c
 * @brief   FDCAN1 설정 구현
 * @note    CAN-FD 500kbps/2Mbps (BRS), TX/RX FIFO, ID 필터, 수신 인터럽트
 */

#include "fdcan_config.h"
#include "iso_tp.h"
#include "uart_debug.h"
#include <string.h>

/* === 수신 버퍼 (CAN-FD 최대 64바이트) === */
static uint8_t s_rx_data[64];
static FDCAN_RxHeaderTypeDef s_rx_header;

/* === DLC 코드(0~15) → 실제 바이트 수 변환 테이블 ===
 * HAL FDCAN: HAL_FDCAN_GetRxMessage 가 채우는 rx_header.DataLength 는
 * raw DLC 코드(0~15) 이다 (HAL 이 R1[19:16] 에서 >>16 로 추출).
 * Classic(0~8) 은 코드 == 바이트 수이지만, FD(9~15) 는 12/16/20/24/32/48/64.
 * HAL 내부 DLCtoBytes[] 와 동일한 매핑.
 */
static const uint8_t DLC_CODE_TO_BYTES[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U,
    12U, 16U, 20U, 24U, 32U, 48U, 64U
};

/**
 * @brief  바이트 수 → FDCAN DLC 코드(raw, 0~15) 변환
 * @note   HAL 규칙: TxHeader.DataLength 에는 raw 코드를 넘긴다
 *         (HAL 이 내부에서 <<16 하여 R1[19:16] 에 기록).
 *         CAN-FD 유효 DLC 사이즈(8/12/16/20/24/32/48/64)로 올림.
 */
uint32_t FDCAN_BytesToDlc(uint8_t bytes)
{
    uint32_t dlc;

    if (bytes <= 8U) {
        dlc = (uint32_t)bytes;             /* 코드 0~8 == 바이트 수 */
    } else if (bytes <= 12U) {
        dlc = FDCAN_DLC_BYTES_12;          /* 0x9 */
    } else if (bytes <= 16U) {
        dlc = FDCAN_DLC_BYTES_16;          /* 0xA */
    } else if (bytes <= 20U) {
        dlc = FDCAN_DLC_BYTES_20;          /* 0xB */
    } else if (bytes <= 24U) {
        dlc = FDCAN_DLC_BYTES_24;          /* 0xC */
    } else if (bytes <= 32U) {
        dlc = FDCAN_DLC_BYTES_32;          /* 0xD */
    } else if (bytes <= 48U) {
        dlc = FDCAN_DLC_BYTES_48;          /* 0xE */
    } else {
        dlc = FDCAN_DLC_BYTES_64;          /* 0xF → 64 */
    }

    return dlc;
}

/**
 * @brief  FDCAN1 CAN-FD 모드 초기화
 * @param  hfdcan: FDCAN 핸들러 포인터
 * @retval HAL_OK = 성공
 *
 * @note   CAN-FD with BRS:
 *         - 아비트레이션(노미널) 페이스: 500kbps (FDCAN_PRESCALER/SEG1/SEG2)
 *         - 데이터 페이스: 2Mbps (FDCAN_DATA_* 매크로)
 *         - 송신 시 FDCAN_BRS_ON, FDCAN_FD_CAN 플래그 사용
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

    /* --- 필터 및 FIFO 설정 --- */
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

    /* 필터: RANGE 0x000~0x7FF (모든 표준 프레임) -> RX FIFO0
     * 루프백 테스트에서 검증된 구성. (이전 MASK 필터가 외부 수신 시 동작하지 않아 RANGE로 변경) */
    filter_config.IdType       = FDCAN_STANDARD_ID;
    filter_config.FilterIndex  = 0U;
    filter_config.FilterType   = FDCAN_FILTER_RANGE;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1    = 0x000U;
    filter_config.FilterID2    = 0x7FFU;

    /* HAL 필터 설정 API 호출 */
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
        Debug_Print("[FDCAN] Filter config failed\r\n");
        return HAL_ERROR;
    }

    /* 수신 경로 레지스터 덤프 (글로벌 필터 확인 - STM32G4는 RXF0C 없음, FIFO0 크기 고정) */
    {
        uint32_t rxgfc = hfdcan->Instance->RXGFC;
        Debug_Print("[FILT] RXGFC=0x%08lX (ANFS=%lu LSS=%lu RRFE=%lu)\r\n",
                    (unsigned long)rxgfc, (unsigned long)(rxgfc & 0x3UL),
                    (unsigned long)((rxgfc >> 16UL) & 0x7UL),
                    (unsigned long)((rxgfc >> 7UL) & 0x1UL));
    }

    Debug_Print("[FDCAN] Filter OK - RANGE 0x000-0x7FF -> FIFO0\r\n");
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

    /* --- 에러 인터럽트 라인 (IT1) 설정 --- */
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

    /* RX FIFO0 새 메시지 + 에러 인터럽트 활성화 */
    status = HAL_FDCAN_ActivateNotification(
        hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE
      | FDCAN_IT_ERROR_WARNING      /* TEC/REC > 96 */
      | FDCAN_IT_ERROR_PASSIVE      /* TEC/REC > 127 */
      | FDCAN_IT_BUS_OFF,           /* TEC > 255 (치명적) */
        0U  /* FIFO0 워터마크: 0 = 모든 메시지에 대해 인터럽트 */
    );

    if (status != HAL_OK) {
        Debug_Print("[FDCAN] Notification activate failed\r\n");
        return status;
    }

    Debug_Print("[FDCAN] RX + Error interrupts enabled\r\n");
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

            /* DLC 코드(0~15) → 실제 바이트 수. HAL 은 raw 코드를 주므로
             * 과거의 ">> 16" 디코딩(항상 0이 되는 버그) 대신 테이블 사용. */
            uint8_t dlc_code = (uint8_t)(s_rx_header.DataLength & 0xFU);
            uint8_t byte_len = DLC_CODE_TO_BYTES[dlc_code];
            if (byte_len > 64U) {
                byte_len = 64U;   /* 방어 (정상이면 발생 안 함) */
            }
            rx_msg.dlc = byte_len;

            /* data 배열 복사 (CAN-FD 최대 64바이트).
             * HAL 이 s_rx_data 에 이미 byte_len 만큼 채워 넣었음. */
            (void)memset(rx_msg.data, 0, sizeof(rx_msg.data));
            for (uint8_t i = 0U; i < byte_len; i++) {
                rx_msg.data[i] = s_rx_data[i];
            }

            /* Queue에 전송 (ISR 전용 함수) */
            xQueueSendFromISR(xCanRxQueue, &rx_msg, &xHigherPriorityTaskWoken);
        }

        /* 모든 인터럽트 재활성화 (RX + 에러) */
        HAL_FDCAN_ActivateNotification(
            hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE
          | FDCAN_IT_ERROR_WARNING
          | FDCAN_IT_ERROR_PASSIVE
          | FDCAN_IT_BUS_OFF,
            0U
        );

        /* Queue 수신 태스크가 더 높은 우선순위면 즉시 스위치 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
