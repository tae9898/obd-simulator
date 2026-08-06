/**
 * @file    iso_tp.h
 * @brief   ISO 15765-2 (ISO-TP) transport layer header
 * @note    Single/multi-frame reassembly and segmentation, CAN-FD support (up to 64 bytes)
 *
 * Why ISO-TP is needed:
 *   A CAN-FD frame can carry up to 64 bytes of payload, but
 *   UDS messages can be longer.
 *   ISO-TP splits long messages into multiple CAN-FD frames and
 *   reassembles them on the other side.
 *
 *   ≤62 bytes → Single Frame (completes in 1 frame, escape SF)
 *   ≥63 bytes → First Frame + Consecutive Frames + Flow Control
 */

#ifndef __ISO_TP_H
#define __ISO_TP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can_addressing.h"   /* AddrType_t (Physical/Functional/Ignore) */

/* === ISO-TP Constants === */
/**
 * Maximum message size (buffer full-reassembly upper limit).
 * - total ≤ this value: reassemble entirely in rx_buffer/tx_buffer (normal UDS multi-frame)
 * - total > this value: deliver CF chunks directly via stream sink (OTA bulk, not full-buffering)
 *
 * Reduced to 512B to save RAM — sufficient for OBD-II/UDS use (previously 4095B).
 * The longest response (VIN 17B + header) is under 100B, and UDS_MAX_RESPONSE_SIZE(64) is the upper bound.
 * OTA bulk (>512) is already bypassed via ISO_TP_RegisterStreamSink() stream path.
 * Unrelated to the classic FF limit (FF_ESCAPE_THRESHOLD=4095) — escape FF logic remains unchanged.
 * Reduction effect: rx_buffer + tx_buffer combined RAM savings ~−7.1KB.
 */
#define ISO_TP_MAX_MESSAGE_SIZE    512U

/** classic FF 12-bit length limit. Exceeding this uses escape FF (ISO 15765-2:2016) */
#define ISO_TP_FF_ESCAPE_THRESHOLD 4095U

/** CAN-FD frame maximum payload */
#define ISO_TP_FRAME_SIZE          64U

/** Single Frame maximum payload (64 - 2-byte escape PCI) */
#define ISO_TP_SF_MAX_PAYLOAD      62U

/** Consecutive Frame maximum payload (64 - 1-byte PCI) */
#define ISO_TP_CF_PAYLOAD_SIZE     63U

/** Flow Control Block Size (0 = unlimited) */
#define ISO_TP_FC_BLOCK_SIZE       0U

/** Flow Control STmin (ms, 0 = minimum delay) */
#define ISO_TP_FC_STMIN            5U

/**
 * Maximum number of FC.WAIT frames the receiving tester can send (transmitter-side limit).
 * 0 = unlimited. If >0, exceeding WFTmax aborts transmit → prevents hang.
 * Corresponds to N_WFTmax in ISO 15765-2. Value is implementation-defined (typical 1~tens); 7 = safe upper bound for simulator.
 */
#define ISO_TP_MAX_WFT             7U

/* === ISO-TP PCI types (Protocol Control Information) === */
/**
 * PCI uses the upper 4 bits of the first byte of each frame to distinguish frame types:
 *   0x0N = Single Frame   (N = payload length)
 *   0x1N = First Frame    (N + next byte = total length, max 4095)
 *   0x2N = Consecutive    (N = sequence number 0~F cyclic)
 *   0x30 = Flow Control   (FS + BS + STmin)
 */
#define ISO_TP_PCI_SINGLE_FRAME    0x00U
#define ISO_TP_PCI_FIRST_FRAME     0x10U
#define ISO_TP_PCI_CONSECUTIVE     0x20U
#define ISO_TP_PCI_FLOW_CONTROL    0x30U

/* === Flow Control FS (Flow Status) === */
#define ISO_TP_FC_CONTINUE         0x00U  /* Continue sending */
#define ISO_TP_FC_WAIT             0x01U  /* Wait */
#define ISO_TP_FC_OVERFLOW         0x02U  /* Buffer full, abort */

/* === ISO-TP State Machine States ===
 * ISO 15765-2:2016 §9.8.3 Table 23: receive (RX) and transmit (TX) state machines are independent.
 * Since segment transmit and receive can proceed simultaneously even on a single N_AI (full-duplex),
 * rx_state and tx_state are separated. The old structure shared a single state, which could not
 * represent concurrent transmit and receive, and had a potential bug where multi-frame response
 * transmit was overwritten by receive termination processing.
 */

/** Receive state (independent of transmit) */
typedef enum {
    ISO_TP_RX_IDLE,         /**< Waiting for receive */
    ISO_TP_RX_WAIT_CF,      /**< Receiving CF (multi-frame reassembly) */
    ISO_TP_RX_COMPLETE      /**< Receive complete (transient state before dispatch) */
} ISO_TP_RxState_t;

/** Transmit state (independent of receive) */
typedef enum {
    ISO_TP_TX_IDLE,         /**< Waiting for transmit */
    ISO_TP_TX_WAIT_FC,      /**< Waiting for FC after FF transmit */
    ISO_TP_TX_SEND_CF       /**< CF transmit in progress */
} ISO_TP_TxState_t;

/* === ISO-TP Control Block === */
typedef struct {
    ISO_TP_RxState_t rx_state;             /**< Receive state (independent of TX) */
    ISO_TP_TxState_t tx_state;             /**< Transmit state (independent of RX) */

    /* Receive related */
    uint8_t  rx_buffer[ISO_TP_MAX_MESSAGE_SIZE]; /**< Receive reassembly buffer (for buffer path) */
    uint32_t rx_total_size;                 /**< Total received message size (escape FF: 32-bit) */
    uint32_t rx_received;                   /**< Bytes received so far */
    uint8_t  rx_expected_seq;               /**< Next expected CF sequence number */
    uint32_t rx_can_id;                     /**< Receive CAN ID */
    AddrType_t rx_addr_type;                /**< Receive addressing type (Physical/Functional) */
    uint8_t  stream_mode;                   /**< 1=stream path (>MAX, forwarded to sink), 0=buffer path */

    /* Transmit related */
    uint8_t  tx_buffer[ISO_TP_MAX_MESSAGE_SIZE]; /**< Transmit segmentation buffer */
    uint32_t tx_total_size;                 /**< Total transmit message size */
    uint32_t tx_sent;                       /**< Bytes transmitted so far */
    uint8_t  tx_seq;                        /**< Next CF sequence number */
    uint32_t tx_can_id;                     /**< Transmit CAN ID */
    uint8_t  tx_stmin;                      /**< Tester-requested CF interval (ms, FC.STmin) */
    uint8_t  tx_block_size;                 /**< Tester-requested BS (0=unlimited, FC.BS) */
    uint8_t  tx_block_counter;              /**< Number of CF sent in current block */
    uint8_t  tx_wait_frame_count;           /**< FC.WAIT receive count (WFTmax protection) */

    /* Timeouts (RX/TX independent tracking) */
    uint32_t last_rx_tick;                  /**< Last receive activity time (ms) */
    uint32_t last_tx_tick;                  /**< Last transmit activity time (ms) */
} ISO_TP_Context_t;

/* === OTA Stream Sink (for bulk receive, > MAX_MESSAGE_SIZE) ===
 * When a message exceeds MAX_MESSAGE_SIZE(4095), it cannot be buffered entirely in rx_buffer.
 * Instead, CF chunks are delivered sequentially to a registered sink (e.g., sequential flash writes)
 * to handle bulk transfers (OTA) without buffering. If no sink is registered, >4095 FF is
 * rejected with FC_OVERFLOW.
 */
typedef enum {
    ISO_TP_STREAM_BEGIN,  /**< Transfer start (total_size passed, data=NULL, len=0) */
    ISO_TP_STREAM_DATA,   /**< Data chunk (data, len valid) */
    ISO_TP_STREAM_END,    /**< Transfer complete */
    ISO_TP_STREAM_ERROR   /**< Abort (timeout/seq error) */
} ISO_TP_StreamEvent_t;

typedef void (*ISO_TP_StreamSink_t)(ISO_TP_StreamEvent_t event,
                                    const uint8_t *data,
                                    uint32_t len,
                                    uint32_t total_size);

/* === API === */

/**
 * @brief  Initialize ISO-TP module
 */
void ISO_TP_Init(void);

/**
 * @brief  Process a received CAN frame in the ISO-TP layer
 * @param  can_id: Receive CAN ID
 * @param  data:   Frame data
 * @param  dlc:    Data length
 * @note   Called from FDCAN RX callback.
 *         On message reassembly completion, calls UDS_DispatchRequest() and handles response transmission
 */
void ISO_TP_ProcessFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief  Transmit UDS response via ISO-TP
 * @param  can_id: Transmit CAN ID
 * @param  data:   Response data
 * @param  len:    Response length
 */
void ISO_TP_SendResponse(uint32_t can_id, const uint8_t *data, uint16_t len);

/**
 * @brief  ISO-TP timeout processing (called periodically from main loop)
 * @param  now_ms: Current time (ms)
 */
void ISO_TP_Tick(uint32_t now_ms);

/**
 * @brief  Register OTA stream sink (for receive > MAX_MESSAGE_SIZE)
 * @param  sink: Stream event callback (NULL = stream inactive, >4095 FF rejected)
 * @note   Register before OTA transfer starts, unregister (NULL) after completion recommended.
 *         Sink is called in sequence: BEGIN→(DATA repeated)→END/ERROR.
 */
void ISO_TP_RegisterStreamSink(ISO_TP_StreamSink_t sink);

#ifdef __cplusplus
}
#endif

#endif /* __ISO_TP_H */
