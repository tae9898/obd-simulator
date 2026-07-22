/**
 * @file    diag_session.c
 * @brief   UDS 진단 세션 매니저 구현
 * @note    세션 전환, S3 타임아웃, Seed-Key 시뮬레이션
 */

#include "diag_session.h"
#include "uds_service.h"
#include "uart_debug.h"
#include <string.h>

/* === 시큐리티 실패 제한 === */
#define DIAG_MAX_FAIL_ATTEMPTS   3U     /**< 최대 Key 실패 횟수 */
#define DIAG_LOCKOUT_TIME_MS     10000U /**< 실패 후 잠금 시간 (10초) */

/* === 세션 제어 블록 === */
static Diag_Session_t s_session;
static uint8_t  s_fail_count = 0U;
static uint32_t s_lockout_start = 0U;
static uint32_t s_boot_tick = 0U;           /**< 부팅 시각 (boot delay 기준점) */

/* === 내부 함수 === */
static uint16_t compute_key(uint16_t seed);

void DiagSession_Init(void)
{
    (void)memset(&s_session, 0, sizeof(s_session));
    s_session.session_type = DIAG_SESSION_DEFAULT;
    s_session.security_level = DIAG_SEC_LOCKED;
    s_session.last_activity_tick = HAL_GetTick();
    s_fail_count = 0U;
    s_boot_tick = HAL_GetTick();
    Debug_Print("[DIAG] Session manager init OK\r\n");
}

int DiagSession_SetSession(uint8_t session_type)
{
    if (session_type != DIAG_SESSION_DEFAULT &&
        session_type != DIAG_SESSION_PROGRAMMING &&
        session_type != DIAG_SESSION_EXTENDED) {
        return -1;
    }

    uint8_t prev = s_session.session_type;
    s_session.session_type = session_type;
    s_session.last_activity_tick = HAL_GetTick();

    /* Default로 복귀하면 시큐리티도 잠금 */
    if (session_type == DIAG_SESSION_DEFAULT) {
        s_session.security_level = DIAG_SEC_LOCKED;
    }

    Debug_Print("[DIAG] Session: %u -> %u\r\n", prev, session_type);
    return 0;
}

/**
 * @brief  Seed 생성
 * @note   SysTick 카운터를 엔트로피 소스로 사용.
 *         실제 ECU에서는 TRNG(True Random Number Generator) 사용.
 *         시뮬레이션이라 SysTick으로 충분.
 */
uint16_t DiagSession_GenerateSeed(void)
{
    uint32_t tick = HAL_GetTick();
    s_session.seed = (uint16_t)((tick ^ (tick >> 16U)) & 0xFFFFU);

    if (s_session.seed == 0U) {
        s_session.seed = 0x0001U;  /* 0은 이미 언락됨을 의미하므로 금지 */
    }

    s_session.seed_is_fresh = 1U;
    s_session.last_activity_tick = HAL_GetTick();

    Debug_Print("[DIAG] Seed: 0x%04X\r\n", s_session.seed);
    return s_session.seed;
}

/**
 * @brief  Key 검증 (사유별 결과 반환)
 * @retval DIAG_KEY_OK / DIAG_KEY_INVALID / DIAG_KEY_EXCEEDED_ATTEMPTS /
 *         DIAG_KEY_DELAY_NOT_EXPIRED
 *
 * 시큐리티 절차 (우선순위순):
 *   1. 부팅 직후 딜레이 미경과 → DELAY_NOT_EXPIRED (NRC 0x37)
 *   2. 3회 실패 후 잠금 기간   → EXCEEDED_ATTEMPTS   (NRC 0x36)
 *   3. Seed 미발행             → INVALID             (NRC 0x35)
 *   4. Key = compute_key(seed) 비교 → OK(언락) / INVALID(카운터++)
 *
 * @note   잠금/딜레이 중에는 올바른 키라도 거부한다 (ISO 14229-1).
 *         이전 구조(int 0/-1)에서는 잠금 중에도 NRC 0x35(InvalidKey) 만 반환해
 *         공격자에게 "현재 잠겨 있음"이라는 표준 신호(0x36/0x37)를 주지 못했다.
 */
DiagKeyResult_t DiagSession_VerifyKey(uint16_t key)
{
    /* 1. 부팅 직후 딜레이 — power-on brute-force 완충 (NRC 0x37) */
    uint32_t since_boot = HAL_GetTick() - s_boot_tick;
    if (since_boot < DIAG_BOOT_DELAY_MS) {
        Debug_Print("[DIAG] Boot delay (%lu ms left)\r\n",
                    (unsigned long)(DIAG_BOOT_DELAY_MS - since_boot));
        return DIAG_KEY_DELAY_NOT_EXPIRED;
    }

    /* 2. 시도 초과 잠금 — 올바른 키여도 잠금 기간엔 거부 (NRC 0x36) */
    if (s_fail_count >= DIAG_MAX_FAIL_ATTEMPTS) {
        uint32_t elapsed = HAL_GetTick() - s_lockout_start;
        if (elapsed < DIAG_LOCKOUT_TIME_MS) {
            Debug_Print("[DIAG] Locked out (%lu ms left)\r\n",
                        (unsigned long)(DIAG_LOCKOUT_TIME_MS - elapsed));
            return DIAG_KEY_EXCEEDED_ATTEMPTS;
        }
        s_fail_count = 0U;
    }

    /* 3. Seed 유효성 */
    if (!s_session.seed_is_fresh) {
        Debug_Print("[DIAG] Seed not fresh\r\n");
        return DIAG_KEY_INVALID;
    }

    uint16_t expected = compute_key(s_session.seed);
    s_session.seed_is_fresh = 0U;  /* seed는 한 번만 사용 */

    /* 4. 비교 → 성공(언락) / 실패(카운터++) */
    if (key == expected) {
        s_session.security_level = DIAG_SEC_LEVEL1;
        s_fail_count = 0U;
        s_session.last_activity_tick = HAL_GetTick();
        Debug_Print("[DIAG] Security unlocked\r\n");
        return DIAG_KEY_OK;
    }

    s_fail_count++;
    Debug_Print("[DIAG] Key mismatch: got=0x%04X exp=0x%04X (fail %u/%u)\r\n",
                key, expected, s_fail_count, DIAG_MAX_FAIL_ATTEMPTS);

    if (s_fail_count >= DIAG_MAX_FAIL_ATTEMPTS) {
        s_lockout_start = HAL_GetTick();
        Debug_Print("[DIAG] Lockout for %us\r\n", DIAG_LOCKOUT_TIME_MS / 1000U);
    }

    return DIAG_KEY_INVALID;
}

void DiagSession_ResetS3Timeout(void)
{
    s_session.last_activity_tick = HAL_GetTick();
}

/**
 * @brief  S3 타임아웃 처리
 * @note   5초간 활동 없으면 Default 세션으로 복귀.
 *         Default 세션에서는 타임아웃 없음 (이미 기본 상태).
 */
void DiagSession_Tick(uint32_t now_ms)
{
    if (s_session.session_type == DIAG_SESSION_DEFAULT) {
        return;
    }

    if ((now_ms - s_session.last_activity_tick) >= DIAG_S3_TIMEOUT_MS) {
        Debug_Print("[DIAG] S3 timeout -> Default\r\n");
        DiagSession_SetSession(DIAG_SESSION_DEFAULT);
    }
}

/**
 * @brief  서비스 접근 권한 확인
 * @note   SID 0x31 (RoutineControl) 규칙:
 *         - Extended 세션이어야 함
 *         - 시큐리티가 언락되어야 함
 *         둘 중 하나라도 불만족 → NRC 0x33 (SecurityAccessDenied)
 */
int DiagSession_CheckAccess(uint8_t sid)
{
    if (sid == UDS_SID_ROUTINE_CONTROL) {
        if (s_session.session_type != DIAG_SESSION_EXTENDED) {
            return -1;
        }
        if (s_session.security_level == DIAG_SEC_LOCKED) {
            return -1;
        }
    }
    return 0;
}

/**
 * @brief  Seed-Key 알고리즘
 * @note   key = ((seed ^ 0x5A3C) rotate_left 3) & 0xFFFF
 *
 *         시뮬레이션용 단순 알고리즘.
 *         실제 양산에서는 메모리 보호, 난수 품질 등 더 엄격함.
 *
 *         계산 예시: seed = 0xA3F7
 *           1. XOR:  0xA3F7 ^ 0x5A3C = 0xF9CB
 *           2. ROL3: 0xF9CB <<< 3 = (0xF9CB << 3) | (0xF9CB >> 13)
 *                         = 0xFCE58 & 0xFFFF | 0x1F
 *                         = 0xCE5D
 */
static uint16_t compute_key(uint16_t seed)
{
    uint16_t xored = seed ^ DIAG_SEED_XOR_MASK;
    uint16_t rotated = (uint16_t)((xored << 3U) | (xored >> (16U - 3U)));
    return rotated & 0xFFFFU;
}
