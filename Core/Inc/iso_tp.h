/**
 * @file    iso_tp.h
 * @brief   ISO 15765-2 (ISO-TP) 전송 계층 헤더
 * @note    단일/다중 프레임 조립·분할, CAN-FD 지원 (최대 64바이트)
 *
 * ISO-TP가 필요한 이유:
 *   CAN-FD 프레임은 최대 64바이트 페이로드를 실을 수 있지만,
 *   UDS 메시지는 그보다 길 수 있음.
 *   ISO-TP가 긴 메시지를 여러 CAN-FD 프레임으로 쪼개고 반대편에서 조립.
 *
 *   62바이트 이하 → Single Frame (1프레임으로 끝, escape SF)
 *   63바이트 이상 → First Frame + Consecutive Frame들 + Flow Control
 */

#ifndef __ISO_TP_H
#define __ISO_TP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === ISO-TP 상수 === */
/**
 * 최대 메시지 크기 (버퍼 풀-조립 상한).
 * - total ≤ 이 값: rx_buffer/tx_buffer 에 통째로 조립 (일반 UDS 멀티프레임)
 * - total > 이 값: 스트림 싱크로 CF 청크를 직접 전달 (OTA 대용량, 풀-버퍼링 아님)
 * 4095 = classic FF(12-bit) 한계. 초과분은 escape FF(32-bit)로 처리.
 */
#define ISO_TP_MAX_MESSAGE_SIZE    4095U

/** classic FF 12-bit 길이 한계. 이 값 초과 시 escape FF 사용 (ISO 15765-2:2016) */
#define ISO_TP_FF_ESCAPE_THRESHOLD 4095U

/** CAN-FD 프레임 최대 페이로드 */
#define ISO_TP_FRAME_SIZE          64U

/** Single Frame 최대 페이로드 (64 - 2바이트 escape PCI) */
#define ISO_TP_SF_MAX_PAYLOAD      62U

/** Consecutive Frame 최대 페이로드 (64 - 1바이트 PCI) */
#define ISO_TP_CF_PAYLOAD_SIZE     63U

/** Flow Control Block Size (0 = 무제한) */
#define ISO_TP_FC_BLOCK_SIZE       0U

/** Flow Control STmin (ms, 0 = 최소 지연) */
#define ISO_TP_FC_STMIN            5U

/* === ISO-TP PCI 타입 (Protocol Control Information) === */
/**
 * PCI는 각 프레임의 첫 바이트 상위 4비트로 프레임 타입을 구분:
 *   0x0N = Single Frame   (N = 페이로드 길이)
 *   0x1N = First Frame    (N + 다음 바이트 = 총 길이, 최대 4095)
 *   0x2N = Consecutive    (N = 시퀀스 번호 0~F 순환)
 *   0x30 = Flow Control   (FS + BS + STmin)
 */
#define ISO_TP_PCI_SINGLE_FRAME    0x00U
#define ISO_TP_PCI_FIRST_FRAME     0x10U
#define ISO_TP_PCI_CONSECUTIVE     0x20U
#define ISO_TP_PCI_FLOW_CONTROL    0x30U

/* === Flow Control FS (Flow Status) === */
#define ISO_TP_FC_CONTINUE         0x00U  /* 계속 보내라 */
#define ISO_TP_FC_WAIT             0x01U  /* 잠깐 기다려라 */
#define ISO_TP_FC_OVERFLOW         0x02U  /* 버퍼 꽉참, 중단 */

/* === ISO-TP 상태머신 상태 === */
typedef enum {
    ISO_TP_IDLE,             /**< 대기 중 */
    ISO_TP_WAIT_CF,          /**< CF 수신 대기 중 (멀티프레임 조립) */
    ISO_TP_TX_WAIT_FC,       /**< 송신 중 FC 대기 */
    ISO_TP_TX_SEND_CF,       /**< 송신 중 CF 전송 진행 */
    ISO_TP_COMPLETE,         /**< 수신 완료 */
    ISO_TP_ERROR             /**< 오류 */
} ISO_TP_State_t;

/* === ISO-TP 제어 블록 === */
typedef struct {
    ISO_TP_State_t state;                  /**< 현재 상태 */

    /* 수신 관련 */
    uint8_t  rx_buffer[ISO_TP_MAX_MESSAGE_SIZE]; /**< 수신 조립 버퍼 (버퍼 경로용) */
    uint32_t rx_total_size;                 /**< 총 수신 메시지 크기 (escape FF: 32-bit) */
    uint32_t rx_received;                   /**< 지금까지 수신한 바이트 수 */
    uint8_t  rx_expected_seq;               /**< 다음에 올 CF 시퀀스 번호 */
    uint32_t rx_can_id;                     /**< 수신 CAN ID */
    uint8_t  stream_mode;                   /**< 1=스트림 경로(>MAX, 싱크로 전달), 0=버퍼 경로 */

    /* 송신 관련 */
    uint8_t  tx_buffer[ISO_TP_MAX_MESSAGE_SIZE]; /**< 송신 분할 버퍼 */
    uint32_t tx_total_size;                 /**< 총 송신 메시지 크기 */
    uint32_t tx_sent;                       /**< 지금까지 송신한 바이트 수 */
    uint8_t  tx_seq;                        /**< 다음 CF 시퀀스 번호 */
    uint32_t tx_can_id;                     /**< 송신 CAN ID */

    /* 타임아웃 */
    uint32_t last_activity_tick;            /**< 마지막 활동 시간 (ms) */
} ISO_TP_Context_t;

/* === OTA 스트림 싱크 (대용량 수신용, > MAX_MESSAGE_SIZE) ===
 * 메시지가 MAX_MESSAGE_SIZE(4095)를 초과하면 rx_buffer에 통째로 담을 수 없다.
 * 대신 등록된 싱크로 CF 청크를 순차 전달하여 (예: 플래시 순차 기록)
 * 버퍼링 없이 대용량 전송(OTA)을 처리한다. 싱크 미등록 시 >4095 FF는
 * FC_OVERFLOW 로 거부된다.
 */
typedef enum {
    ISO_TP_STREAM_BEGIN,  /**< 전송 시작 (total_size 전달, data=NULL, len=0) */
    ISO_TP_STREAM_DATA,   /**< 데이터 청크 (data, len 유효) */
    ISO_TP_STREAM_END,    /**< 전송 완료 */
    ISO_TP_STREAM_ERROR   /**< 중단 (타임아웃/seq 오류) */
} ISO_TP_StreamEvent_t;

typedef void (*ISO_TP_StreamSink_t)(ISO_TP_StreamEvent_t event,
                                    const uint8_t *data,
                                    uint32_t len,
                                    uint32_t total_size);

/* === API === */

/**
 * @brief  ISO-TP 모듈 초기화
 */
void ISO_TP_Init(void);

/**
 * @brief  수신 CAN 프레임을 ISO-TP 계층에서 처리
 * @param  can_id: 수신 CAN ID
 * @param  data:   프레임 데이터
 * @param  dlc:    데이터 길이
 * @note   FDCAN RX 콜백에서 호출.
 *         메시지 조립 완료 시 UDS_DispatchRequest() 호출 후 응답 전송까지 처리
 */
void ISO_TP_ProcessFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

/**
 * @brief  UDS 응답을 ISO-TP로 전송
 * @param  can_id: 송신 CAN ID
 * @param  data:   응답 데이터
 * @param  len:    응답 길이
 */
void ISO_TP_SendResponse(uint32_t can_id, const uint8_t *data, uint16_t len);

/**
 * @brief  ISO-TP 타임아웃 처리 (메인 루프에서 주기적 호출)
 * @param  now_ms: 현재 시간 (ms)
 */
void ISO_TP_Tick(uint32_t now_ms);

/**
 * @brief  OTA 스트림 싱크 등록 (> MAX_MESSAGE_SIZE 수신용)
 * @param  sink: 스트림 이벤트 콜백 (NULL = 스트림 비활성, >4095 FF 거부)
 * @note   OTA 전송 시작 전 등록, 완료 후 해제(NULL) 권장.
 *         싱크는 BEGIN→(DATA 반복)→END/ERROR 순으로 호출된다.
 */
void ISO_TP_RegisterStreamSink(ISO_TP_StreamSink_t sink);

#ifdef __cplusplus
}
#endif

#endif /* __ISO_TP_H */
