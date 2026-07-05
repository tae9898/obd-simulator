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
 * 상태머신 (수신/송신 독립 — ISO 15765-2:2016 §9.8.3 full-duplex):
 *   rx: RX_IDLE → (SF수신) → RX_COMPLETE → UDS처리 → RX_IDLE
 *   rx: RX_IDLE → (FF수신) → RX_WAIT_CF → (CF수신 반복) → RX_COMPLETE → ... → RX_IDLE
 *   tx: TX_IDLE → (SF송신) → TX_IDLE
 *   tx: TX_IDLE → (FF송신) → TX_WAIT_FC → (FC수신) → TX_SEND_CF → TX_IDLE
 *   rx_state 와 tx_state 는 독립이므로 세그먼트 송·수신이 동시에 진행될 수 있다.
 */

#include "iso_tp.h"
#include "uds_service.h"
#include "fdcan_config.h"
#include "uart_debug.h"
#include <string.h>

/* === 타임아웃 === */
#define ISO_TP_RX_TIMEOUT_MS     1000U  /**< CF 수신 대기 타임아웃 */
#define ISO_TP_TX_FC_TIMEOUT_MS  1000U  /**< FC 수신 대기 타임아웃 */

/* === 정적 컨텍스트 === */
static ISO_TP_Context_t s_ctx;
static ISO_TP_StreamSink_t s_stream_sink = NULL;  /**< OTA 스트림 싱크 (NULL=비활성) */

/* === 내부 함수 프로토타입 === */
static void process_single_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
static void process_first_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
static void process_consecutive_frame(const uint8_t *data, uint8_t dlc);
static void process_flow_control(const uint8_t *data, uint8_t dlc);
static void send_single_frame(uint32_t can_id, const uint8_t *data, uint16_t len);
static void send_first_frame(uint32_t can_id, const uint8_t *data, uint32_t total_len);
static void send_consecutive_frames(void);
static void send_flow_control(uint32_t can_id, uint8_t fs, uint8_t bs, uint8_t stmin);
static void dispatch_completed_message(void);
static void abort_stream_rx(void);

/* === 외부 FDCAN 핸들러 (main.c에 정의) === */
extern FDCAN_HandleTypeDef hfdcan1;

/* ====================================================
 * 공개 API
 * ==================================================== */

void ISO_TP_Init(void)
{
    (void)memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.rx_state = ISO_TP_RX_IDLE;
    s_ctx.tx_state = ISO_TP_TX_IDLE;
    Debug_Print("[ISO-TP] Init OK\r\n");
}

void ISO_TP_RegisterStreamSink(ISO_TP_StreamSink_t sink)
{
    s_stream_sink = sink;
}

void ISO_TP_ProcessFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (data == NULL || dlc == 0U) {
        return;
    }

    /* PCI 타입 판별: 첫 바이트의 상위 4비트 */
    uint8_t pci_type = data[0] & 0xF0U;

    /* Flow Control 은 진행 중인 TX 트랜잭션에 묶인 제어 프레임.
     * tester 의 FC 송신 can_id 가 노드 설정에 따라 달라질 수 있으므로
     * 어드레싱 필터와 무관하게 처리한다 (process_flow_control 이 tx_state 로 가드). */
    if (pci_type == ISO_TP_PCI_FLOW_CONTROL) {
        process_flow_control(data, dlc);
        return;
    }

    /* 어드레싱 분류 (RX 조립 경로에만 적용).
     * 이 노드 대상(Physical 0x7E0 / Functional 0x7DF)이 아니면 무시 —
     * 다른 노드 트래픽/노이즈는 조립·FC·응답 모두 스킵한다. */
    AddrType_t addr = CAN_Addr_Classify(can_id);
    if (addr == ADDR_IGNORE) {
#if DEBUG_VERBOSE
        Debug_Print("[ISO-TP] Ignored CAN ID 0x%lX (not addressed to us)\r\n",
                    (unsigned long)can_id);
#endif
        return;
    }
    s_ctx.rx_addr_type = addr;

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

    /* tx_buffer 풀-버퍼 상한 초과 방지.
     * > MAX 송신은 TX 스트리밍 소스(미구현)가 필요하다. */
    if (len > ISO_TP_MAX_MESSAGE_SIZE) {
        Debug_Print("[ISO-TP] TX too large (%u > %u), rejected\r\n",
                    (unsigned)len, (unsigned)ISO_TP_MAX_MESSAGE_SIZE);
        return;
    }

    if (len <= ISO_TP_SF_MAX_PAYLOAD) {
        send_single_frame(can_id, data, len);
    } else {
        /* 멀티프레임 송신: FF 보내고 FC 대기.
         * 진행 중인 세그먼트 송신이 있으면 거부(단일 송신 채널은 한 번에 하나). */
        if (s_ctx.tx_state != ISO_TP_TX_IDLE) {
            Debug_Print("[ISO-TP] TX busy, new response rejected\r\n");
            return;
        }

        (void)memcpy(s_ctx.tx_buffer, data, len);
        s_ctx.tx_total_size = (uint32_t)len;
        s_ctx.tx_sent = 0U;
        s_ctx.tx_seq = 1U;
        s_ctx.tx_can_id = can_id;
        s_ctx.tx_state = ISO_TP_TX_WAIT_FC;
        send_first_frame(can_id, data, (uint32_t)len);
    }
}

void ISO_TP_Tick(uint32_t now_ms)
{
    /* CF 수신 타임아웃 (RX 상태만 검사 — 송신과 독립) */
    if (s_ctx.rx_state == ISO_TP_RX_WAIT_CF) {
        if ((now_ms - s_ctx.last_rx_tick) >= ISO_TP_RX_TIMEOUT_MS) {
            Debug_Print("[ISO-TP] RX timeout\r\n");
            abort_stream_rx();
            s_ctx.rx_state = ISO_TP_RX_IDLE;
        }
    }

    /* FC 수신 타임아웃 (TX 상태만 검사 — 수신과 독립) */
    if (s_ctx.tx_state == ISO_TP_TX_WAIT_FC) {
        if ((now_ms - s_ctx.last_tx_tick) >= ISO_TP_TX_FC_TIMEOUT_MS) {
            Debug_Print("[ISO-TP] TX FC timeout\r\n");
            s_ctx.tx_state = ISO_TP_TX_IDLE;
        }
    }
}

/* ====================================================
 * 수신 처리 (ISR 컨텍스트)
 * ==================================================== */

/**
 * @brief  Single Frame 처리 (CAN-FD: 62바이트 이하, escape SF)
 *
 * PCI: 0x0N (N = 페이로드 길이)
 * 예: [03, 0x41, 0x0C, 0x27, 0x10] → 3바이트 페이로드
 */
static void process_single_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t payload_len;

    /* §9.8.3 Table 23 (full-duplex): 송신 중에도 수신은 독립 동작. 단, 세그먼트
     * 수신이 진행 중이면 현재 수신을 종료(스트림이면 ERROR 통지)하고 이 SF 를
     * 새 수신의 시작으로 처리한다. */
    if (s_ctx.rx_state == ISO_TP_RX_WAIT_CF) {
        abort_stream_rx();
        Debug_Print("[ISO-TP] SF restarts in-progress reception\r\n");
    }

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
    s_ctx.rx_state = ISO_TP_RX_COMPLETE;

#if DEBUG_VERBOSE
    Debug_Print("[ISO-TP] SF len=%u\r\n", payload_len);
#endif
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
    /* 기능적 주소(0x7DF)로의 멀티프레임(FF)은 비표준 — FC를 브로드캐스트 버스에
     * 쏘면 다른 ECU 와 충돌하므로 기능적 주소에서는 SF 만 허용하고 FF 는 무시한다. */
    if (s_ctx.rx_addr_type == ADDR_FUNCTIONAL) {
        Debug_Print("[ISO-TP] FF dropped on functional address\r\n");
        return;
    }

    /* §9.8.3 Table 23 (full-duplex): 송신 중에도 수신은 독립 동작. 세그먼트 수신이
     * 진행 중이면 현재 수신을 종료(스트림이면 ERROR 통지)하고 이 FF 로 재시작. */
    if (s_ctx.rx_state == ISO_TP_RX_WAIT_CF) {
        abort_stream_rx();
        Debug_Print("[ISO-TP] FF restarts in-progress reception\r\n");
    }

    /* 길이 디코딩: classic(12-bit) 또는 escape(32-bit).
     * escape 감지: data[0]=0x10 & data[1]=0x00. 정상 FF에선 길이 0(무효)이라
     * 이 값을 escape 마커로 재사용 (ISO 15765-2:2016). */
    uint32_t total_size;
    uint8_t  payload_off;
    uint16_t len12 = (uint16_t)(((uint16_t)(data[0] & 0x0FU) << 8U) | (uint16_t)data[1]);

    if (len12 != 0U) {
        /* classic FF (≤4095) */
        total_size  = (uint32_t)len12;
        payload_off = 2U;
    } else {
        /* escape FF (>4095): bytes[2..5] = 32-bit 길이 (big-endian) */
        if (dlc < 6U) {
            Debug_Print("[ISO-TP] escape FF too short (dlc=%u)\r\n", dlc);
            s_ctx.rx_state = ISO_TP_RX_IDLE;
            return;
        }
        total_size = (((uint32_t)data[2]) << 24U)
                   | (((uint32_t)data[3]) << 16U)
                   | (((uint32_t)data[4]) << 8U)
                   | ((uint32_t)data[5]);
        payload_off = 6U;
    }

    /* FF 데이터 청크 (PCI + 길이 필드 이후) */
    uint8_t ff_payload = 0U;
    if (dlc > payload_off) {
        ff_payload = (uint8_t)(dlc - payload_off);
    }
    if ((uint32_t)ff_payload > (ISO_TP_FRAME_SIZE - payload_off)) {
        ff_payload = (uint8_t)(ISO_TP_FRAME_SIZE - payload_off);
    }
    if ((uint32_t)ff_payload > total_size) {
        ff_payload = (uint8_t)total_size;
    }

    /* 버퍼 한계 초과 → 스트림 싱크로 처리 (싱크 미등록 시 거부) */
    if (total_size > ISO_TP_MAX_MESSAGE_SIZE) {
        if (s_stream_sink == NULL) {
            Debug_Print("[ISO-TP] FF too large: %lu, no sink -> OVERFLOW\r\n",
                        (unsigned long)total_size);
            send_flow_control(can_id, ISO_TP_FC_OVERFLOW, 0U, 0U);
            s_ctx.rx_state = ISO_TP_RX_IDLE;
            return;
        }
        /* 스트림 모드 진입: rx_buffer 미사용, CF 청크를 싱크로 직접 전달 */
        s_ctx.rx_total_size     = total_size;
        s_ctx.rx_received       = 0U;
        s_ctx.rx_expected_seq   = 1U;
        s_ctx.rx_can_id         = can_id;
        s_ctx.stream_mode       = 1U;
        s_ctx.last_rx_tick      = HAL_GetTick();
        s_ctx.rx_state = ISO_TP_RX_WAIT_CF;

        s_stream_sink(ISO_TP_STREAM_BEGIN, NULL, 0U, total_size);
        if (ff_payload > 0U) {
            s_stream_sink(ISO_TP_STREAM_DATA, &data[payload_off],
                          (uint32_t)ff_payload, total_size);
            s_ctx.rx_received = (uint32_t)ff_payload;
        }
        Debug_Print("[ISO-TP] FF(stream) total=%lu, got=%lu\r\n",
                    (unsigned long)total_size, (unsigned long)s_ctx.rx_received);
        send_flow_control(can_id, ISO_TP_FC_CONTINUE, ISO_TP_FC_BLOCK_SIZE, ISO_TP_FC_STMIN);
        return;
    }

    /* 버퍼 경로 (≤4095): rx_buffer에 통째로 조립 */
    (void)memcpy(s_ctx.rx_buffer, &data[payload_off], ff_payload);
    s_ctx.rx_total_size     = total_size;
    s_ctx.rx_received       = (uint32_t)ff_payload;
    s_ctx.rx_expected_seq   = 1U;
    s_ctx.rx_can_id         = can_id;
    s_ctx.stream_mode       = 0U;
    s_ctx.last_rx_tick      = HAL_GetTick();
    s_ctx.rx_state = ISO_TP_RX_WAIT_CF;

    Debug_Print("[ISO-TP] FF total=%lu, got=%lu\r\n",
                (unsigned long)total_size, (unsigned long)s_ctx.rx_received);

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
    if (s_ctx.rx_state != ISO_TP_RX_WAIT_CF) {
        return;
    }

    uint8_t seq = data[0] & 0x0FU;
    if (seq != s_ctx.rx_expected_seq) {
        Debug_Print("[ISO-TP] CF seq mismatch: got=%u exp=%u\r\n",
                     seq, s_ctx.rx_expected_seq);
        abort_stream_rx();
        s_ctx.rx_state = ISO_TP_RX_IDLE;
        return;
    }

    uint32_t remaining = s_ctx.rx_total_size - s_ctx.rx_received;
    uint8_t cf_payload = (uint8_t)(dlc - 1U);
    if ((uint32_t)cf_payload > remaining) {
        cf_payload = (uint8_t)remaining;
    }

    if (s_ctx.stream_mode != 0U) {
        /* 스트림 경로: 싱크로 청크 전달 (rx_buffer 미사용) */
        if (s_stream_sink != NULL) {
            s_stream_sink(ISO_TP_STREAM_DATA, &data[1],
                          (uint32_t)cf_payload, s_ctx.rx_total_size);
        }
    } else {
        (void)memcpy(&s_ctx.rx_buffer[s_ctx.rx_received], &data[1], cf_payload);
    }

    s_ctx.rx_received += (uint32_t)cf_payload;
    s_ctx.rx_expected_seq = (uint8_t)((s_ctx.rx_expected_seq + 1U) & 0x0FU);
    s_ctx.last_rx_tick = HAL_GetTick();

    Debug_Print("[ISO-TP] CF seq=%u progress=%lu/%lu\r\n",
                seq, (unsigned long)s_ctx.rx_received, (unsigned long)s_ctx.rx_total_size);

    if (s_ctx.rx_received >= s_ctx.rx_total_size) {
        if (s_ctx.stream_mode != 0U) {
            if (s_stream_sink != NULL) {
                s_stream_sink(ISO_TP_STREAM_END, NULL, 0U, s_ctx.rx_total_size);
            }
            s_ctx.stream_mode = 0U;
            s_ctx.rx_state = ISO_TP_RX_IDLE;
            Debug_Print("[ISO-TP] Stream complete\r\n");
        } else {
            s_ctx.rx_state = ISO_TP_RX_COMPLETE;
            Debug_Print("[ISO-TP] Multi-frame complete\r\n");
            dispatch_completed_message();
        }
    }
}

/**
 * @brief  Flow Control 수신 (송신 측에서 "계속 보내라" 받음)
 */
static void process_flow_control(const uint8_t *data, uint8_t dlc)
{
    (void)dlc;

    if (s_ctx.tx_state != ISO_TP_TX_WAIT_FC) {
        return;
    }

    uint8_t fs = (uint8_t)(data[0] & 0x0FU);  /* Flow Status: byte0 하위 니블 */

    if (fs == ISO_TP_FC_CONTINUE) {
        s_ctx.tx_state = ISO_TP_TX_SEND_CF;
        s_ctx.last_tx_tick = HAL_GetTick();
        send_consecutive_frames();
    } else if (fs == ISO_TP_FC_WAIT) {
        s_ctx.last_tx_tick = HAL_GetTick();
    } else {
        s_ctx.tx_state = ISO_TP_TX_IDLE;
    }
}

/* ====================================================
 * 송신 처리
 * ==================================================== */

static void send_single_frame(uint32_t can_id, const uint8_t *data, uint16_t len)
{
    uint8_t frame[ISO_TP_FRAME_SIZE];
    uint8_t used;   /* 프레임에 실제 사용한 바이트 수 */

    (void)memset(frame, 0xCCU, sizeof(frame));

    if (len <= 7U) {
        /* Classic SF: byte0 = 0x0N (하위 니블 = 길이) */
        frame[0] = (uint8_t)(ISO_TP_PCI_SINGLE_FRAME | (uint8_t)(len & 0x0FU));
        (void)memcpy(&frame[1], data, len);
        used = (uint8_t)(len + 1U);
    } else {
        /* CAN-FD escape SF: byte0 = 0x00, byte1 = 길이(8~62) */
        frame[0] = ISO_TP_PCI_SINGLE_FRAME;   /* 0x00 */
        frame[1] = (uint8_t)len;
        (void)memcpy(&frame[2], data, len);
        used = (uint8_t)(len + 2U);
    }

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier          = can_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;
    tx_header.DataLength          = FDCAN_BytesToDlc(used);

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, frame) != HAL_OK) {
        Debug_Print("[ISO-TP] SF TX failed\r\n");
    }
}

static void send_first_frame(uint32_t can_id, const uint8_t *data, uint32_t total_len)
{
    uint8_t frame[ISO_TP_FRAME_SIZE];
    uint8_t payload_off;

    (void)memset(frame, 0xCCU, sizeof(frame));

    if (total_len <= ISO_TP_FF_ESCAPE_THRESHOLD) {
        /* classic FF: 12-bit 길이 (byte0 하위 니블 + byte1) */
        frame[0] = (uint8_t)(ISO_TP_PCI_FIRST_FRAME
                             | (uint8_t)((total_len >> 8U) & 0x0FU));
        frame[1] = (uint8_t)(total_len & 0xFFU);
        payload_off = 2U;
    } else {
        /* escape FF (>4095): 0x10 0x00 + 32-bit 길이 (big-endian, CAN-FD 전용) */
        frame[0] = ISO_TP_PCI_FIRST_FRAME;              /* 0x10 */
        frame[1] = 0x00U;                                /* escape 마커 */
        frame[2] = (uint8_t)((total_len >> 24U) & 0xFFU);
        frame[3] = (uint8_t)((total_len >> 16U) & 0xFFU);
        frame[4] = (uint8_t)((total_len >> 8U) & 0xFFU);
        frame[5] = (uint8_t)(total_len & 0xFFU);
        payload_off = 6U;
    }

    /* FF 페이로드: 프레임에서 (PCI + 길이) 이후, 전체 길이 이하 */
    uint32_t ff_payload = total_len;
    if (ff_payload > ((uint32_t)ISO_TP_FRAME_SIZE - (uint32_t)payload_off)) {
        ff_payload = (uint32_t)ISO_TP_FRAME_SIZE - (uint32_t)payload_off;
    }
    (void)memcpy(&frame[payload_off], data, ff_payload);
    s_ctx.tx_sent = ff_payload;

    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.Identifier          = can_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;
    tx_header.DataLength          = FDCAN_DLC_BYTES_64;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, frame) != HAL_OK) {
        Debug_Print("[ISO-TP] FF TX failed\r\n");
        s_ctx.tx_state = ISO_TP_TX_IDLE;
    } else {
        s_ctx.last_tx_tick = HAL_GetTick();
        Debug_Print("[ISO-TP] FF TX total=%lu\r\n", (unsigned long)total_len);
    }
}

static void send_consecutive_frames(void)
{
    while (s_ctx.tx_sent < s_ctx.tx_total_size && s_ctx.tx_state == ISO_TP_TX_SEND_CF) {
        uint8_t frame[ISO_TP_FRAME_SIZE];
        (void)memset(frame, 0xCCU, sizeof(frame));

        frame[0] = (uint8_t)(ISO_TP_PCI_CONSECUTIVE | (s_ctx.tx_seq & 0x0FU));

        uint32_t remaining = s_ctx.tx_total_size - s_ctx.tx_sent;
        uint8_t cf_payload = ISO_TP_CF_PAYLOAD_SIZE;   /* 63 */
        if ((uint32_t)cf_payload > remaining) {
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
        tx_header.DataLength          = FDCAN_BytesToDlc((uint8_t)(cf_payload + 1U));

        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, frame) != HAL_OK) {
            Debug_Print("[ISO-TP] CF TX failed\r\n");
            s_ctx.tx_state = ISO_TP_TX_IDLE;
            return;
        }

        s_ctx.tx_sent += (uint32_t)cf_payload;
        s_ctx.tx_seq = (s_ctx.tx_seq + 1U) & 0x0FU;
    }

    if (s_ctx.tx_sent >= s_ctx.tx_total_size) {
        Debug_Print("[ISO-TP] TX complete\r\n");
        s_ctx.tx_state = ISO_TP_TX_IDLE;
    }
}

static void send_flow_control(uint32_t can_id, uint8_t fs, uint8_t bs, uint8_t stmin)
{
    uint8_t frame[ISO_TP_FRAME_SIZE];
    (void)memset(frame, 0xCCU, sizeof(frame));

    /* ISO 15765-2 FC: byte0=0x30|FS, byte1=BS, byte2=STmin */
    frame[0] = (uint8_t)(ISO_TP_PCI_FLOW_CONTROL | (fs & 0x0FU));  /* FS */
    frame[1] = bs;       /* Block Size */
    frame[2] = stmin;    /* STmin */

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
 * @brief  진행 중인 스트림 수신을 중단 통지 (싱크가 등록된 경우)
 *         stream_mode 는 호출자가 클리어한다.
 */
static void abort_stream_rx(void)
{
    if ((s_ctx.stream_mode != 0U) && (s_stream_sink != NULL)) {
        s_stream_sink(ISO_TP_STREAM_ERROR, NULL, 0U, s_ctx.rx_total_size);
    }
    s_ctx.stream_mode = 0U;
}

/**
 * @brief  조립 완료 → UDS 디스패처 호출 → 응답 전송
 * @note   수신 종료만 담당. 응답 송신(TX)은 ISO_TP_SendResponse() 가 tx_state 로
 *         독립적으로 관리하므로 여기서 건드리지 않는다. (이전 단일-state 설계에선
 *         마지막 state=IDLE 이 멀티프레임 응답 TX_WAIT_FC 를 덮어쓰는 버그가 있었음.)
 */
static void dispatch_completed_message(void)
{
    uint8_t response[UDS_MAX_RESPONSE_SIZE];
    uint16_t resp_len = 0U;

    UDS_DispatchRequest(s_ctx.rx_buffer, s_ctx.rx_total_size,
                        s_ctx.rx_addr_type,
                        response, &resp_len);

    /* RX 종료. TX 상태는 건드리지 않는다. */
    s_ctx.rx_state = ISO_TP_RX_IDLE;

    if (resp_len > 0U) {
        /* 응답 ID:
         *   기능적 요청(0x7DF) → 이 노드의 물리적 응답 ID(0x7E8).
         *   물리적 요청(0x7E0) → rx_can_id + 8 (0x7E0→0x7E8).
         *   (0x7DF + 8 = 0x7E7 은 잘못된 ID 이므로 분기 필요) */
        uint32_t resp_id;
        if (s_ctx.rx_addr_type == ADDR_FUNCTIONAL) {
            resp_id = CAN_ID_PHYSICAL_RESP;   /* 0x7E8 */
        } else {
            resp_id = s_ctx.rx_can_id + 8U;   /* 0x7E0→0x7E8 */
        }
        ISO_TP_SendResponse(resp_id, response, resp_len);
    }
}
