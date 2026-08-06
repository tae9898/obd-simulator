/**
 * @file    iso_tp.c
 * @brief   ISO 15765-2 (ISO-TP) transport layer implementation
 * @note    Single Frame / First Frame + CF + FC state machine
 *          CAN-FD up to 64 bytes, no dynamic allocation
 *
 * Data flow:
 *   Receive: FDCAN ISR → ISO_TP_ProcessFrame() → reassembly complete → UDS_DispatchRequest()
 *   Transmit: UDS → ISO_TP_SendResponse() → SF or FF+CF segmentation → FDCAN TX
 *
 * State machine (RX/TX independent — ISO 15765-2:2016 §9.8.3 full-duplex):
 *   rx: RX_IDLE → (SF received) → RX_COMPLETE → UDS processing → RX_IDLE
 *   rx: RX_IDLE → (FF received) → RX_WAIT_CF → (CF received repeatedly) → RX_COMPLETE → ... → RX_IDLE
 *   tx: TX_IDLE → (SF transmitted) → TX_IDLE
 *   tx: TX_IDLE → (FF transmitted) → TX_WAIT_FC → (FC received) → TX_SEND_CF → TX_IDLE
 *   rx_state and tx_state are independent so segment transmit/receive can proceed simultaneously.
 */

#include "iso_tp.h"
#include "uds_service.h"
#include "fdcan_config.h"
#include "uart_debug.h"
#include "task.h"            /* taskENTER/EXIT_CRITICAL (TX state cross-task protection) */
#include <string.h>

/* === Timeouts === */
#define ISO_TP_RX_TIMEOUT_MS     1000U  /**< CF receive wait timeout */
#define ISO_TP_TX_FC_TIMEOUT_MS  1000U  /**< FC receive wait timeout */

/* === Static context === */
static ISO_TP_Context_t s_ctx;
static ISO_TP_StreamSink_t s_stream_sink = NULL;  /**< OTA stream sink (NULL=inactive) */

/* === Internal function prototypes === */
static void process_single_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
static void process_first_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
static void process_consecutive_frame(const uint8_t *data, uint8_t dlc);
static void process_flow_control(const uint8_t *data, uint8_t dlc);
static void send_single_frame(uint32_t can_id, const uint8_t *data, uint16_t len);
static void send_first_frame(uint32_t can_id, const uint8_t *data, uint32_t total_len);
static void send_next_cf(void);
static void send_flow_control(uint32_t can_id, uint8_t fs, uint8_t bs, uint8_t stmin);
static void dispatch_completed_message(void);
static void abort_stream_rx(void);

/* === External FDCAN handle (defined in main.c) === */
extern FDCAN_HandleTypeDef hfdcan1;

/* ====================================================
 * Public API
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

    /* PCI type detection: upper 4 bits of first byte */
    uint8_t pci_type = data[0] & 0xF0U;

    /* Flow Control is a control frame bound to an in-progress TX transaction.
     * The tester's FC transmit can_id may vary by node configuration, so
     * process it regardless of addressing filter (process_flow_control guards via tx_state). */
    if (pci_type == ISO_TP_PCI_FLOW_CONTROL) {
        process_flow_control(data, dlc);
        return;
    }

    /* Addressing classification (applies only to RX reassembly path).
     * If not addressed to this node (Physical 0x7E0 / Functional 0x7DF), ignore —
     * other node traffic/noise skips reassembly, FC, and response. */
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

    /* Prevent tx_buffer full-buffer overflow.
     * >MAX transmit requires a TX streaming source (not implemented). */
    if (len > ISO_TP_MAX_MESSAGE_SIZE) {
        Debug_Print("[ISO-TP] TX too large (%u > %u), rejected\r\n",
                    (unsigned)len, (unsigned)ISO_TP_MAX_MESSAGE_SIZE);
        return;
    }

    if (len <= ISO_TP_SF_MAX_PAYLOAD) {
        send_single_frame(can_id, data, len);
    } else {
        /* Multi-frame transmit: send FF then wait for FC.
         * Reject if a segment transmit is already in progress (single TX channel handles one at a time). */
        if (s_ctx.tx_state != ISO_TP_TX_IDLE) {
            Debug_Print("[ISO-TP] TX busy, new response rejected\r\n");
            return;
        }

        /* Start multi-frame transmit: set TX state atomically (H2).
         * send_first_frame's FDCAN TX (long operation) is outside critical section. */
        taskENTER_CRITICAL();
        (void)memcpy(s_ctx.tx_buffer, data, len);
        s_ctx.tx_total_size = (uint32_t)len;
        s_ctx.tx_sent = 0U;
        s_ctx.tx_seq = 1U;
        s_ctx.tx_can_id = can_id;
        s_ctx.tx_wait_frame_count = 0U;   /* New transaction: reset WAIT counter */
        s_ctx.tx_state = ISO_TP_TX_WAIT_FC;
        taskEXIT_CRITICAL();
        send_first_frame(can_id, data, (uint32_t)len);
    }
}

void ISO_TP_Tick(uint32_t now_ms)
{
    /* TX state machine is shared with vCanRxTask (FC receive/first CF) (H2).
     * If vCanRxTask preempts while vMainTask reads→decides→transitions (or sends CF),
     * it can change tx_state, and on resume the stale state overwrites IDLE — a race.
     * Bind read-decide-transition into one critical section.
     * Critical section is short (send_next_cf's FDCAN queue load is non-blocking). */
    taskENTER_CRITICAL();

    /* CF receive timeout (check only RX state — independent of transmit) */
    if (s_ctx.rx_state == ISO_TP_RX_WAIT_CF) {
        if ((now_ms - s_ctx.last_rx_tick) >= ISO_TP_RX_TIMEOUT_MS) {
            Debug_Print("[ISO-TP] RX timeout\r\n");
            abort_stream_rx();
            s_ctx.rx_state = ISO_TP_RX_IDLE;
        }
    }

    /* FC receive timeout (check only TX state — independent of receive) */
    if (s_ctx.tx_state == ISO_TP_TX_WAIT_FC) {
        if ((now_ms - s_ctx.last_tx_tick) >= ISO_TP_TX_FC_TIMEOUT_MS) {
            Debug_Print("[ISO-TP] TX FC timeout\r\n");
            s_ctx.tx_state = ISO_TP_TX_IDLE;
        }
    }

    /* CF pacing: transmit one CF after STmin elapses (TX_SEND_CF state). */
    if (s_ctx.tx_state == ISO_TP_TX_SEND_CF) {
        if ((now_ms - s_ctx.last_tx_tick) >= s_ctx.tx_stmin) {
            send_next_cf();
        }
    }

    taskEXIT_CRITICAL();
}

/* ====================================================
 * Receive processing (ISR context)
 * ==================================================== */

/**
 * @brief  Single Frame processing (CAN-FD: up to 62 bytes, escape SF)
 *
 * PCI: 0x0N (N = payload length)
 * Example: [03, 0x41, 0x0C, 0x27, 0x10] → 3-byte payload
 */
static void process_single_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t payload_len;

    /* §9.8.3 Table 23 (full-duplex): receive operates independently even during transmit.
     * However, if a segment receive is in progress, terminate current receive (notify ERROR
     * if streaming) and treat this SF as the start of a new receive. */
    if (s_ctx.rx_state == ISO_TP_RX_WAIT_CF) {
        abort_stream_rx();
        Debug_Print("[ISO-TP] SF restarts in-progress reception\r\n");
    }

    if (dlc > 1U && (data[0] & 0x0FU) == 0U) {
        /* CAN-FD extended SF: byte0=0x00, byte1=length */
        payload_len = data[1];
        if (payload_len > (dlc - 2U)) {
            payload_len = (uint8_t)(dlc - 2U);
        }
        (void)memcpy(s_ctx.rx_buffer, &data[2], payload_len);
    } else {
        /* Classic SF: lower nibble = length */
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
 * @brief  First Frame processing (multi-frame start)
 *
 * PCI: 0x1N + byte1 → total length (max 4095)
 * Immediately responds with Flow Control (FC) to signal "continue sending"
 */
static void process_first_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    /* Multi-frame (FF) to a functional address (0x7DF) is non-standard — broadcasting FC
     * on the bus would collide with other ECUs, so only SF is allowed on functional address;
     * FF is ignored. */
    if (s_ctx.rx_addr_type == ADDR_FUNCTIONAL) {
        Debug_Print("[ISO-TP] FF dropped on functional address\r\n");
        return;
    }

    /* §9.8.3 Table 23 (full-duplex): receive operates independently even during transmit.
     * If a segment receive is in progress, terminate current receive (notify ERROR if streaming)
     * and restart with this FF. */
    if (s_ctx.rx_state == ISO_TP_RX_WAIT_CF) {
        abort_stream_rx();
        Debug_Print("[ISO-TP] FF restarts in-progress reception\r\n");
    }

    /* Length decoding: classic (12-bit) or escape (32-bit).
     * Escape detection: data[0]=0x10 & data[1]=0x00. A normal FF with length 0 (invalid)
     * repurposes this value as an escape marker (ISO 15765-2:2016). */
    uint32_t total_size;
    uint8_t  payload_off;
    uint16_t len12 = (uint16_t)(((uint16_t)(data[0] & 0x0FU) << 8U) | (uint16_t)data[1]);

    if (len12 != 0U) {
        /* classic FF (≤4095) */
        total_size  = (uint32_t)len12;
        payload_off = 2U;
    } else {
        /* escape FF (>4095): bytes[2..5] = 32-bit length (big-endian) */
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

    /* FF data chunk (after PCI + length fields) */
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

    /* Buffer limit exceeded → process via stream sink (reject if no sink registered) */
    if (total_size > ISO_TP_MAX_MESSAGE_SIZE) {
        if (s_stream_sink == NULL) {
            Debug_Print("[ISO-TP] FF too large: %lu, no sink -> OVERFLOW\r\n",
                        (unsigned long)total_size);
            send_flow_control(can_id, ISO_TP_FC_OVERFLOW, 0U, 0U);
            s_ctx.rx_state = ISO_TP_RX_IDLE;
            return;
        }
        /* Enter stream mode: rx_buffer unused, CF chunks forwarded directly to sink */
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

    /* Buffer path (≤4095): reassemble entirely in rx_buffer */
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

    /* "Continue sending" FC response */
    send_flow_control(can_id, ISO_TP_FC_CONTINUE, ISO_TP_FC_BLOCK_SIZE, ISO_TP_FC_STMIN);
}

/**
 * @brief  Consecutive Frame processing (data segment)
 *
 * PCI: 0x2N (N = sequence number, 0~15 cyclic)
 * If seq number matches, append to buffer
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
        /* Stream path: forward chunk to sink (rx_buffer unused) */
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
 * @brief  Flow Control receive (transmit side gets "continue sending")
 */
static void process_flow_control(const uint8_t *data, uint8_t dlc)
{
    (void)dlc;

    /* Atomically read and modify TX state (H2: races with ISO_TP_Tick). */
    taskENTER_CRITICAL();
    if (s_ctx.tx_state != ISO_TP_TX_WAIT_FC) {
        taskEXIT_CRITICAL();
        return;
    }

    uint8_t fs = (uint8_t)(data[0] & 0x0FU);  /* Flow Status: byte0 lower nibble */

    if (fs == ISO_TP_FC_CONTINUE) {
        /* Reflect tester-requested BS / STmin (ISO 15765-2 FC).
         * STmin: 0x00-0x7F = ms, 0x80-0xF9 = 100us, 0xFA-0xFF = reserved.
         * Simulator only supports ms resolution → us/reserved mapped to 0 (minimum interval). */
        uint8_t stmin = data[2];
        s_ctx.tx_stmin        = (stmin < 0x80U) ? stmin : 0U;
        s_ctx.tx_block_size   = data[1];
        s_ctx.tx_block_counter = 0U;
        s_ctx.last_tx_tick    = HAL_GetTick();
        s_ctx.tx_state        = ISO_TP_TX_SEND_CF;
        /* First CF immediately after FC without STmin wait (ISO 15765-2). */
        send_next_cf();
    } else if (fs == ISO_TP_FC_WAIT) {
        /* WFTmax protection: prevent hang from infinite WAIT. */
        s_ctx.tx_wait_frame_count++;
        if ((ISO_TP_MAX_WFT != 0U) &&
            (s_ctx.tx_wait_frame_count > ISO_TP_MAX_WFT)) {
            Debug_Print("[ISO-TP] FC.WAIT exceeded WFTmax (%u), abort\r\n",
                        ISO_TP_MAX_WFT);
            s_ctx.tx_state = ISO_TP_TX_IDLE;
        } else {
            s_ctx.last_tx_tick = HAL_GetTick();  /* Reset FC timeout */
        }
    } else {
        /* ISO_TP_FC_OVERFLOW (0x02) or unknown → abort transmit */
        Debug_Print("[ISO-TP] FC overflow/unknown (fs=%u), abort\r\n", fs);
        s_ctx.tx_state = ISO_TP_TX_IDLE;
    }
    taskEXIT_CRITICAL();
}

/* ====================================================
 * Transmit processing
 * ==================================================== */

static void send_single_frame(uint32_t can_id, const uint8_t *data, uint16_t len)
{
    uint8_t frame[ISO_TP_FRAME_SIZE];
    uint8_t used;   /* Number of bytes actually used in the frame */

    (void)memset(frame, 0xCCU, sizeof(frame));

    if (len <= 7U) {
        /* Classic SF: byte0 = 0x0N (lower nibble = length) */
        frame[0] = (uint8_t)(ISO_TP_PCI_SINGLE_FRAME | (uint8_t)(len & 0x0FU));
        (void)memcpy(&frame[1], data, len);
        used = (uint8_t)(len + 1U);
    } else {
        /* CAN-FD escape SF: byte0 = 0x00, byte1 = length (8..62) */
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
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;   /* BRS: data phase 2Mbps */
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
        /* classic FF: 12-bit length (byte0 lower nibble + byte1) */
        frame[0] = (uint8_t)(ISO_TP_PCI_FIRST_FRAME
                             | (uint8_t)((total_len >> 8U) & 0x0FU));
        frame[1] = (uint8_t)(total_len & 0xFFU);
        payload_off = 2U;
    } else {
        /* escape FF (>4095): 0x10 0x00 + 32-bit length (big-endian, CAN-FD only) */
        frame[0] = ISO_TP_PCI_FIRST_FRAME;              /* 0x10 */
        frame[1] = 0x00U;                                /* escape marker */
        frame[2] = (uint8_t)((total_len >> 24U) & 0xFFU);
        frame[3] = (uint8_t)((total_len >> 16U) & 0xFFU);
        frame[4] = (uint8_t)((total_len >> 8U) & 0xFFU);
        frame[5] = (uint8_t)(total_len & 0xFFU);
        payload_off = 6U;
    }

    /* FF payload: after (PCI + length) in the frame, capped by total length */
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
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;   /* BRS: data phase 2Mbps */
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

/**
 * @brief  Transmit one CF (tick-based pacing)
 * @note   Called by ISO_TP_Tick() one at a time after STmin elapses.
 *           - BS>0 and block reached → TX_WAIT_FC (wait for next FC, flow control)
 *           - Transmit complete       → TX_IDLE
 *           - Otherwise               → stay in TX_SEND_CF (retransmit after STmin on next tick)
 */
static void send_next_cf(void)
{
    if (s_ctx.tx_sent >= s_ctx.tx_total_size) {
        Debug_Print("[ISO-TP] TX complete\r\n");
        s_ctx.tx_state = ISO_TP_TX_IDLE;
        return;
    }

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
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;   /* BRS: data phase 2Mbps */
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
    s_ctx.last_tx_tick = HAL_GetTick();   /* STmin reference point for next CF */
    s_ctx.tx_block_counter++;

    /* Transmit complete */
    if (s_ctx.tx_sent >= s_ctx.tx_total_size) {
        Debug_Print("[ISO-TP] TX complete\r\n");
        s_ctx.tx_state = ISO_TP_TX_IDLE;
        return;
    }

    /* BS>0: wait for next FC when block reached (flow control). BS=0: continue unlimited. */
    if ((s_ctx.tx_block_size != 0U) &&
        (s_ctx.tx_block_counter >= s_ctx.tx_block_size)) {
        Debug_Print("[ISO-TP] block %u sent, wait FC\r\n", s_ctx.tx_block_counter);
        s_ctx.tx_state = ISO_TP_TX_WAIT_FC;
    }
    /* Otherwise stay in TX_SEND_CF → ISO_TP_Tick() calls next CF after STmin */
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
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;   /* BRS: data phase 2Mbps */
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;
    tx_header.DataLength          = FDCAN_DLC_BYTES_16;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, frame) != HAL_OK) {
        Debug_Print("[ISO-TP] FC TX failed\r\n");
    }
}

/* ====================================================
 * Internal utilities
 * ==================================================== */

/**
 * @brief  Notify in-progress stream receive abort (if sink is registered)
 *         Caller clears stream_mode.
 */
static void abort_stream_rx(void)
{
    if ((s_ctx.stream_mode != 0U) && (s_stream_sink != NULL)) {
        s_stream_sink(ISO_TP_STREAM_ERROR, NULL, 0U, s_ctx.rx_total_size);
    }
    s_ctx.stream_mode = 0U;
}

/**
 * @brief  Reassembly complete → UDS dispatcher call → transmit response
 * @note   Only handles receive termination. Response transmit (TX) is independently
 *         managed by ISO_TP_SendResponse() via tx_state, so we do not touch it here.
 *         (The previous single-state design had a bug where the final state=IDLE
 *         would overwrite a multi-frame response's TX_WAIT_FC.)
 */
static void dispatch_completed_message(void)
{
    uint8_t response[UDS_MAX_RESPONSE_SIZE];
    uint16_t resp_len = 0U;

    UDS_DispatchRequest(s_ctx.rx_buffer, s_ctx.rx_total_size,
                        s_ctx.rx_addr_type,
                        response, &resp_len);

    /* End RX. Do not touch TX state. */
    s_ctx.rx_state = ISO_TP_RX_IDLE;

    if (resp_len > 0U) {
        /* Response ID:
         *   Functional request (0x7DF) → this node's physical response ID (0x7E8).
         *   Physical request (0x7E0) → rx_can_id + 8 (0x7E0→0x7E8).
         *   (0x7DF + 8 = 0x7E7 is an incorrect ID, so branching is needed) */
        uint32_t resp_id;
        if (s_ctx.rx_addr_type == ADDR_FUNCTIONAL) {
            resp_id = CAN_ID_PHYSICAL_RESP;   /* 0x7E8 */
        } else {
            resp_id = s_ctx.rx_can_id + 8U;   /* 0x7E0→0x7E8 */
        }
        ISO_TP_SendResponse(resp_id, response, resp_len);
    }
}
