/**
 * @file    diag_session.h
 * @brief   UDS 진단 세션 매니저 헤더
 * @note    세션 타입, 시큐리티 액세스, S3 타임아웃 관리
 *
 * 세션이 필요한 이유:
 *   UDS에서는 현재 세션 상태에 따라 허용되는 서비스가 다름.
 *   - Default 세션: 기본 읽기만 가능 (OBD-II PID 조회 등)
 *   - Extended 세션: 고급 진단 가능 (루틴 실행, 액추에이터 제어)
 *   - Programming 세션: 펌웨어 업데이트
 *
 *   S3 타임아웃: 5초간 활동 없으면 Default 세션으로 자동 복귀
 *   (안전 장치: 진단 도구가 연결 끊기면 ECU가 정상 상태로 돌아감)
 */

#ifndef __DIAG_SESSION_H
#define __DIAG_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === 세션 타입 === */
#define DIAG_SESSION_DEFAULT       0x01U  /**< 기본 세션 (읽기만) */
#define DIAG_SESSION_PROGRAMMING   0x02U  /**< 프로그래밍 세션 (FW 업데이트) */
#define DIAG_SESSION_EXTENDED      0x03U  /**< 확장 세션 (고급 진단) */

/* === 시큐리티 레벨 === */
#define DIAG_SEC_LOCKED            0x00U  /**< 잠금 상태 */
#define DIAG_SEC_LEVEL1            0x01U  /**< 언락 상태 */

/* === S3 타임아웃 === */
/**
 * S3 타임아웃: 5초
 * 의미: 5초간 UDS 요청이 없으면 Default 세션으로 복귀
 * 이유: 진단 도구가 비정상 종료되어도 ECU가 안전 상태로 돌아감
 */
#define DIAG_S3_TIMEOUT_MS         5000U

/* === Seed-Key 시뮬레이션 === */
#define DIAG_SEED_XOR_MASK         0x5A3CU

/* === SecurityAccess brute-force 완충 === */
/**
 * 부팅 직후 짧은 시간 동안 0x27 (sendKey) 검증을 거부한다 (NRC 0x37).
 * power-on 직후 무차별 대입을 막는 최소 완충. iso14229 참조.
 */
#define DIAG_BOOT_DELAY_MS         1000U

/* === Key 검증 결과 — 핸들러가 NRC 를 선택하기 위해 사유를 구분 ===
 * 기존 int (0/-1) 반환에서 확장: 잠금 중이거나 딜레이 미경과일 때
 * 단순 "잘못된 키"가 아닌 ISO 14229-1 표준 NRC(0x36/0x37)를 응답하기 위함.
 */
typedef enum {
    DIAG_KEY_OK = 0,              /**< 성공 (언락) */
    DIAG_KEY_INVALID,            /**< 키 불일치 / seed 미발행 → NRC 0x35 */
    DIAG_KEY_EXCEEDED_ATTEMPTS,  /**< 시도 초과 / 잠금 중 → NRC 0x36 */
    DIAG_KEY_DELAY_NOT_EXPIRED   /**< 부팅/딜레이 미경과 → NRC 0x37 */
} DiagKeyResult_t;

/* === 세션 제어 블록 === */
typedef struct {
    uint8_t  session_type;          /**< 현재 세션 타입 */
    uint8_t  security_level;        /**< 시큐리티 레벨 (LOCKED/LEVEL1) */
    uint16_t seed;                  /**< 마지막 생성 seed */
    uint8_t  seed_is_fresh;         /**< seed가 아직 사용 안 됨 (재사용 방지) */
    uint32_t last_activity_tick;    /**< 마지막 활동 시간 (ms) */
} Diag_Session_t;

/* === API === */

void DiagSession_Init(void);

/**
 * @brief  통신 준비 완료 시점을 SecurityAccess boot-delay 기준점으로 설정
 * @note   DiagSession_Init() 은 scheduler 시작 전(클럭/GPIO/FDCAN 초기화 이전)이라
 *         너무 빠름 — 실제 부팅이 1초를 넘으면 boot delay 가 이미 만료되어 무의미.
 *         FDCAN 시작 등 진단 통신 준비가 끝난 시점(main) 에서 호출해야 의미가 있음.
 */
void DiagSession_MarkBootReady(void);

int  DiagSession_SetSession(uint8_t session_type);

uint16_t DiagSession_GenerateSeed(void);

/**
 * @brief  Key 검증 (사유별 결과 반환)
 * @retval DIAG_KEY_OK / DIAG_KEY_INVALID / DIAG_KEY_EXCEEDED_ATTEMPTS /
 *         DIAG_KEY_DELAY_NOT_EXPIRED
 * @note   핸들러는 반환값에 따라 NRC 0x00(긍정)/0x35/0x36/0x37 를 선택한다.
 *         부팅 후 DIAG_BOOT_DELAY_MS 이내거나, 3회 실패 후 잠금 기간이면
 *         키 내용과 무관하게 EXCEEDED/DELAY 를 반환한다.
 */
DiagKeyResult_t DiagSession_VerifyKey(uint16_t key);

void DiagSession_ResetS3Timeout(void);
void DiagSession_Tick(uint32_t now_ms);

/**
 * @brief  서비스 접근 권한 확인
 * @retval 0: 허용, -1: 거부
 * @note   SID 0x31은 Extended + Security Unlock 필요
 */
int  DiagSession_CheckAccess(uint8_t sid);

#ifdef __cplusplus
}
#endif

#endif /* __DIAG_SESSION_H */
