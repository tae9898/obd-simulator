/**
 * @file    iso_tp.c
 * @brief   ISO 15765-2 (ISO-TP) 전송 계층 구현
 * @note    Single Frame / First Frame + CF + FC 상태머신
 *          CAN-FD 최대 64바이트 지원, 동적 할당 없음
 *
 * 데이터 흐름:
 *   수신: FDCAN ISR → ISO_TP_ProcessFrame() → 조립 완료 → UDS_DispatchRequest()
 *   송신: UDS → ISO_TP_SendResponse() → SF 또는 FF+CF로 분할 → FDCAN TX
 *
 * 상태머신:
 *   IDLE → (SF수신) → COMPLETE → UDS처리 → IDLE
 *   IDLE → (FF수신) → WAIT_CF → (CF수신 반복) → COMPLETE → UDS처리 → IDLE
 *   IDLE → (SF송신) → IDLE
 *   IDLE → (FF송신) → TX_WAIT_FC → (FC수신) → TX_SEND_CF → IDLE
 */

#include "iso_tp.h"
#include "uds_service.h"
#include "uart_debug.h"
#include <string.h>

/* === 타임아웃 === */
#define ISO_TP_RX_TIMEOUT_MS     1000U  /**< CF 수신 대기 타임아웃 */
#define ISO_TP_TX_FC_TIMEOUT_MS  1000U  /**< FC 수신 대기 타임아웃 */

/* === 정적 컨텍스트 === */
static ISO_TP_Context_t s_ctx;

/* === 내부 함수 프로토타입 === */
static void process_single_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
static void process_first_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
static void process_consecutive_frame(const uint8_t *data, uint8_t dlc);
static void process_flow_control(const uint8_t *data, uint8_t dlc);
static void send_single_frame(uint32_t can_id, const uint8_t *data, uint16_t len);
static void send_first_frame(uint32_t can_id, const uint8_t *data, uint16_t total_len);
static void send_consecutive_frames(void);
static void send_flow_control(uint32_t can_id, uint8_t fs, uint8_t bs, uint8_t stmin);
static void dispatch_completed_message(void);
static uint32_t bytes_to_dlc(uint8_t bytes);

/* === 외부 FDCAN 핸들러 (main.c에 정의) === */
extern FDCAN_HandleTypeDef hfdcan1;

/* ====================================================
 * 공개 API
 * ==================================================== */

void ISO_TP_Init(void)
{
    (void)memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = ISO_TP_IDLE;
    Debug_Print("[ISO-TP] Init OK\r\n");
}

void ISO_TP_ProcessFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (data == NULL || dlc == 0U) {
        return;
    }

    /* PCI 타입 판별: 첫 바이트의 상위 4비트 */
    uint8_t pci_type = data[0] & 0xF0U;

    switch (pci_type) {
        case ISO_TP_PCI_SINGLE_FRAME:
            process_single_frame(can_id, data, dlc);
            break;
        case ISO_TP_PCI_FIRST_FRAME:
            process_first_frame(can_id, data, dlc);
            break;
        case ISO_TP_PCI_CONSECUTIVE:
            process_consecutive_frame(data, dlc);
            break;
        case ISO_TP_PCI_FLOW_CONTROL:
            process_flow_control(data, dlc);
            break;
        default:
            Debug_Print("[ISO-TP] Unknown PCI: 0x%02X\r\n", data[0]);
            break;
    }
}

void ISO_TP_SendResponse(uint32_t can_id, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }

    if (len <= ISO_TP_SF_MAX_PAYLOAD) {
        send_single_frame(can_id, data, len);
    } else {
        /* 멀티프레임 송신: FF 보내고 FC 대기 */
        (void)memcpy(s_ctx.tx_buffer, data, len);
        s_ctx.tx_total_size = len;
        s_ctx.tx_sent = 0U;
        s_ctx.tx_seq = 1U;
        s_ctx.tx_can_id = can_id;
        s_ctx.state = ISO_TP_TX_WAIT_FC;
        send_first_frame(can_id, data, len);
    }
}

void ISO_TP_Tick(uint32_t now_ms)
{
    if (s_ctx.state == ISO_TP_IDLE) {
        return;
    }

    /* CF 수신 타임아웃 */
    if (s_ctx.state == ISO_TP_WAIT_CF) {
        if ((now_ms - s_ctx.last_activity_tick) >= ISO_TP_RX_TIMEOUT_MS) {
            Debug_Print("[ISO-TP] RX timeout\r\n");
            s_ctx.state = ISO_TP_IDLE;
        }
    }

    /* FC 수신 타임아웃 */
    if (s_ctx.state == ISO_TP_TX_WAIT_FC) {
        if ((now_ms - s_ctx.last_activity_tick) >= ISO_TP_TX_FC_TIMEOUT_MS) {
            Debug_Print("[ISO-TP] TX FC timeout\r\n");
            s_ctx.state = ISO_TP_IDLE;
        }
    }
}

/* ====================================================
 * 수신 처리 (ISR 컨텍스트)
 * ==================================================== */

/**
 * @brief  Single Frame 처리 (15바이트 이하)
 *
 * PCI: 0x0N (N = 페이로드 길이)
 * 예: [03, 0x41, 0x0C, 0x27, 0x10] → 3바이트 페이로드
 */
static void process_single_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t payload_len;

    if (dlc > 1U && (data[0] & 0x0FU) == 0U) {
        /* CAN-FD 확장 SF: byte0=0x00, byte1=길이 */
        payload_len = data[1];
        if (payload_len > (dlc - 2U)) {
            payload_len = (uint8_t)(dlc - 2U);
        }
        (void)memcpy(s_ctx.rx_buffer, &data[2], payload_len);
    } else {
        /* Classic SF: 하위 니블 = 길이 */
        payload_len = data[0] & 0x0FU;
        if (payload_len > (dlc - 1U)) {
            payload_len = (uint8_t)(dlc - 1U);
        }
        (void)memcpy(s_ctx.rx_buffer, &data[1], payload_len);
    }

    s_ctx.rx_total_size = payload_len;
    s_ctx.rx_can_id = can_id;
    s_ctx.state = ISO_TP_COMPLETE;

    Debug_Print("[ISO-TP] SF len=%u\r\n", payload_len);
    dispatch_completed_message();
}

/**
 * @brief  First Frame 처리 (멀티프레임 시작)
 *
 * PCI: 0x1N + byte1 → 총 길이 (최대 4095)
 * 바로 Flow Control(FC)를 응답해서 "계속 보내라"고 알림
 */
static void process_first_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint16_t total_size = (uint16_t)((data[0] & 0x0FU) << 8U) | (uint16_t)data[1];

    if (total_size > ISO_TP_MAX_MESSAGE_SIZE) {
        Debug_Print("[ISO-TP] FF too large: %u\r\n", total_size);
        send_flow_control(can_id, ISO_TP_FC_OVERFLOW, 0U, 0U);
        s_ctx.state = ISO_TP_IDLE;
        return;
    }

    /* FF 페이로드: byte2부터 (최대 14바이트) */
    uint8_t ff_payload = (uint8_t)(dlc - 2U);
    if (ff_payload > 14U) {
        ff_payload = 14U;
    }

    (void)memcpy(s_ctx.rx_buffer, &data[2], ff_payload);
    s_ctx.rx_total_size = total_size;
    s_ctx.rx_received = ff_payload;
    s_ctx.rx_expected_seq = 1U;
    s_ctx.rx_can_id = can_id;
    s_ctx.last_activity_tick = HAL_GetTick();
    s_ctx.state = ISO_TP_WAIT_CF;

    Debug_Print("[ISO-TP] FF total=%u, got=%u\r\n", total_size, s_ctx.rx_received);

    /* "계속 보내라" FC 응답 */
    send_flow_control(can_id, ISO_TP_FC_CONTINUE, ISO_TP_FC_BLOCK_SIZE, ISO_TP_FC_STMIN);
}

/**
 * @brief  Consecutive Frame 처리 (데이터 조각)
 *
 * PCI: 0x2N (N = 시퀀스 번호, 0~15 순환)
 * seq 번호가 맞으면 버퍼에 이어붙임
 */
static void process_consecutive_frame(const uint8_t *data, uint8_t dlc)
{
    if (s_ctx.state != ISO_TP_WAIT_CF) {
        return;
    }

    uint8_t seq = data[0] & 0x0FU;
    if (seq != s_ctx.rx_expected_seq) {
        Debug_Print("[ISO-TP] CF seq mismatch: got=%u exp=%u\r\n",
                     seq, s_ctx.rx_expected_seq);
        s_ctx.state = ISO_TP_IDLE;
        return;
    }

    uint16_t remaining = s_ctx.rx_total_size - s_ctx.rx_received;
    uint8_t cf_payload = (uint8_t)(dlc - 1U);
    if ((uint16_t)cf_payload > remaining) {
        cf_payload = (uint8_t)remaining;
    }

    (void)memcpy(&s_ctx.rx_buffer[s_ctx.rx_received], &data[1], cf_payload);
    s_ctx.rx_received += cf_payload;
    s_ctx.rx_expected_seq = (s_ctx.rx_expected_seq + 1U) & 0x0FU;
    s_ctx.last_activity_tick = HAL_GetTick();

    Debug_Print("[ISO-TP] CF seq=%u progress=%u/%u\r\n",
                seq, s_ctx.rx_received, s_ctx.rx_total_size);

    if (s_ctx.rx_received >= s_ctx.rx_total_size) {
        s_ctx.state = ISO_TP_COMPLETE;
        Debug_Print("[ISO-TP] Multi-frame complete\r\n");
        dispatch_completed_message();
    }
}

/**
 * @brief  Flow Control 수신 (송신 측에서 "계속 보내라" 받음)
 */
static void process_flow_control(const uint8_t *data, uint8_t dlc)
{
    (void)dlc;

    if (s_ctx.state != ISO_TP_TX_WAIT_FC) {
        return;
    }

    uint8_t fs = data[1];  /* Flow Status */

    if (fs == ISO_TP_FC_CONTINUE) {
        s_ctx.state = ISO_TP_TX_SEND_CF;
        s_ctx.last_activity_tick = HAL_GetTick();
        send_consecutive_frames();
    } else if (fs == ISO_TP_FC_WAIT) {
        s_ctx.last_activity_tick = HAL_GetTick();
    } else {
        s_ctx.state = ISO_TP_IDLE;
    }
}

/* ====================================================
 * 송신 처리
 * ==================================================== */

static void send_single_frame(uint32_t can_id, const uint8_t *data, uint16_t len)
{
    uint8_t frame[ISO_TP_FRAME_SIZE];
    (void)memset(frame, 0xCCU, sizeof(frame));

    frame[0] = (uint8_t)(ISO_TP_PCI_SINGLE_FRAME | (len & 0x0FU));
    (void)memcpy(&frame[1], data, len);

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier          = can_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;
    tx_header.DataLength          = bytes_to_dlc((uint8_t)(len + 1U));

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, frame) != HAL_OK) {
        Debug_Print("[ISO-TP] SF TX failed\r\n");
    }
}

static void send_first_frame(uint32_t can_id, const uint8_t *data, uint16_t total_len)
{
    uint8_t frame[ISO_TP_FRAME_SIZE];

    frame[0] = (uint8_t)(ISO_TP_PCI_FIRST_FRAME | ((total_len >> 8U) & 0x0FU));
    frame[1] = (uint8_t)(total_len & 0xFFU);

    /* FF 페이로드: 최대 14바이트 */
    (void)memcpy(&frame[2], data, 14U);
    s_ctx.tx_sent = 14U;

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier          = can_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;
    tx_header.DataLength          = FDCAN_DLC_BYTES_16;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, frame) != HAL_OK) {
        Debug_Print("[ISO-TP] FF TX failed\r\n");
        s_ctx.state = ISO_TP_IDLE;
    } else {
        s_ctx.last_activity_tick = HAL_GetTick();
        Debug_Print("[ISO-TP] FF TX total=%u\r\n", total_len);
    }
}

static void send_consecutive_frames(void)
{
    while (s_ctx.tx_sent < s_ctx.tx_total_size && s_ctx.state == ISO_TP_TX_SEND_CF) {
        uint8_t frame[ISO_TP_FRAME_SIZE];
        (void)memset(frame, 0xCCU, sizeof(frame));

        frame[0] = (uint8_t)(ISO_TP_PCI_CONSECUTIVE | (s_ctx.tx_seq & 0x0FU));

        uint16_t remaining = s_ctx.tx_total_size - s_ctx.tx_sent;
        uint8_t cf_payload = 15U;
        if ((uint16_t)cf_payload > remaining) {
            cf_payload = (uint8_t)remaining;
        }

        (void)memcpy(&frame[1], &s_ctx.tx_buffer[s_ctx.tx_sent], cf_payload);

        FDCAN_TxHeaderTypeDef tx_header;
        tx_header.Identifier          = s_ctx.tx_can_id;
        tx_header.IdType              = FDCAN_STANDARD_ID;
        tx_header.TxFrameType         = FDCAN_DATA_FRAME;
        tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header.BitRateSwitch       = FDCAN_BRS_ON;
        tx_header.FDFormat            = FDCAN_FD_CAN;
        tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        tx_header.MessageMarker       = 0U;
        tx_header.DataLength          = bytes_to_dlc((uint8_t)(cf_payload + 1U));

        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, frame) != HAL_OK) {
            Debug_Print("[ISO-TP] CF TX failed\r\n");
            s_ctx.state = ISO_TP_IDLE;
            return;
        }

        s_ctx.tx_sent += cf_payload;
        s_ctx.tx_seq = (s_ctx.tx_seq + 1U) & 0x0FU;
    }

    if (s_ctx.tx_sent >= s_ctx.tx_total_size) {
        Debug_Print("[ISO-TP] TX complete\r\n");
        s_ctx.state = ISO_TP_IDLE;
    }
}

static void send_flow_control(uint32_t can_id, uint8_t fs, uint8_t bs, uint8_t stmin)
{
    uint8_t frame[ISO_TP_FRAME_SIZE];
    (void)memset(frame, 0xCCU, sizeof(frame));

    frame[0] = ISO_TP_PCI_FLOW_CONTROL;
    frame[1] = fs;
    frame[2] = bs;
    frame[3] = stmin;

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier          = can_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;
    tx_header.DataLength          = FDCAN_DLC_BYTES_16;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, frame) != HAL_OK) {
        Debug_Print("[ISO-TP] FC TX failed\r\n");
    }
}

/* ====================================================
 * 내부 유틸리티
 * ==================================================== */

/**
 * @brief  조립 완료 → UDS 디스패처 호출 → 응답 전송
 */
static void dispatch_completed_message(void)
{
    uint8_t response[UDS_MAX_RESPONSE_SIZE];
    uint16_t resp_len = 0U;

    UDS_DispatchRequest(s_ctx.rx_buffer, s_ctx.rx_total_size,
                        response, &resp_len);

    if (resp_len > 0U) {
        uint32_t resp_id = s_ctx.rx_can_id + 8U;  /* 0x7E0→0x7E8 */
        ISO_TP_SendResponse(resp_id, response, resp_len);
    }

    s_ctx.state = ISO_TP_IDLE;
}

/**
 * @brief  바이트 수를 FDCAN DLC 코드로 변환
 */
static uint32_t bytes_to_dlc(uint8_t bytes)
{
    if (bytes <= 8U) {
        return (uint32_t)bytes << 16U;
    }
    switch (bytes) {
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return FDCAN_DLC_BYTES_16;
    }
}
