/**
 * @file    can_addressing.h
 * @brief   CAN 어드레싱 계층 — 물리적(Physical) vs 기능적(Functional) 주소 구분
 * @note    ISO 15765-2 / ISO 14229-1 (11-bit 표준 ID) 기반
 *
 * 왜 필요한가:
 *   수신 CAN ID만으로 이 프레임이
 *     (1) 이 노드를 1:1 로 지정한 물리적 요청(0x7E0~0x7E7),
 *     (2) 모든 ECU 에 대한 기능적 브로드캐스트(0x7DF),
 *     (3) 이 노드와 무관한 트래픽(그 외)
 *   인지 구분해야 응답 ID 계산과 서비스별 응답 억제 정책을 올바르게 적용할 수 있다.
 *
 *   기능적 요청(0x7DF) 응답 ID = 이 노드의 물리적 응답 ID (0x7E8).
 *   (단순히 0x7DF+8=0x7E7 로 보내면 안 됨.)
 *
 * 멀티-ECU 확장:
 *   노드 물리적 주소는 아래 매크로 3개로 컴파일타임에 결정된다.
 *   다른 ECU 슬롯(예: 0x7E2/0x7EA)을 사용하려면 이 헤더 한 곳만 수정.
 */

#ifndef __CAN_ADDRESSING_H
#define __CAN_ADDRESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === 노드 CAN ID (컴파일타임 매크로) ===
 * ISO 15765-2 11-bit canonical addressing:
 *   기능적 요청 0x7DF (브로드캐스트)
 *   물리적 요청 0x7E0~0x7E7, 물리적 응답 0x7E8~0x7EF (ECU별 1쌍)
 * 이 노드 기본 배정: 요청 0x7E0 / 응답 0x7E8
 */
#define CAN_ID_FUNCTIONAL_REQ    0x7DFU   /**< 기능적 요청 (모든 ECU 대상 브로드캐스트) */
#define CAN_ID_PHYSICAL_REQ      0x7E0U   /**< 이 노드 물리적 요청 */
#define CAN_ID_PHYSICAL_RESP     0x7E8U   /**< 이 노드 물리적 응답 */

/* === 어드레싱 타입 === */
typedef enum {
    ADDR_IGNORE    = 0,   /**< 이 노드 대상 아님 → 무시 */
    ADDR_PHYSICAL  = 1,   /**< 물리적 요청 (1:1, 0x7E0) */
    ADDR_FUNCTIONAL = 2   /**< 기능적 요청 (브로드캐스트, 0x7DF) */
} AddrType_t;

/**
 * @brief  수신 CAN ID → 어드레싱 타입 분류
 * @param  can_id: 수신 표준 CAN ID
 * @retval ADDR_PHYSICAL / ADDR_FUNCTIONAL / ADDR_IGNORE
 * @note   ISO-TP/UDS 처리 전 1차 필터로 사용. ADDR_IGNORE 는 상위 계층이 무시.
 */
static inline AddrType_t CAN_Addr_Classify(uint32_t can_id)
{
    if (can_id == CAN_ID_FUNCTIONAL_REQ) {
        return ADDR_FUNCTIONAL;
    }
    if (can_id == CAN_ID_PHYSICAL_REQ) {
        return ADDR_PHYSICAL;
    }
    return ADDR_IGNORE;
}

#ifdef __cplusplus
}
#endif

#endif /* __CAN_ADDRESSING_H */
