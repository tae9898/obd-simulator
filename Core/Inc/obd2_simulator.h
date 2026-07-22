/**
 * @file    obd2_simulator.h
 * @brief   OBD-II ECU 시뮬레이터 헤더
 * @note    지원 PID 정의, 시뮬레이션 상태 구조체, 요청/응답 처리 함수
 */

#ifndef __OBD2_SIMULATOR_H
#define __OBD2_SIMULATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* === OBD-II 서비스 모드 정의 === */
#define OBD2_MODE_CURRENT_DATA       0x01U  /* Mode 01: 현재 데이터 요청 */
#define OBD2_MODE_RESPONSE_PREFIX    0x40U  /* 응답 모드 = 요청 모드 + 0x40 */

/* === 지원 PID 정의 === */
#define OBD2_PID_SUPPORTED_PIDS      0x00U  /* 지원 PID 목록 */
#define OBD2_PID_COOLANT_TEMP        0x05U  /* 냉각수 온도 */
#define OBD2_PID_ENGINE_RPM          0x0CU  /* 엔진 RPM */
#define OBD2_PID_VEHICLE_SPEED       0x0DU  /* 차속 */

/* === 시뮬레이션 상태 구조체 === */
typedef struct {
    /** 현재 엔진 RPM (실제값, 예: 800.0 ~ 4000.0) */
    uint16_t engine_rpm;

    /** 현재 냉각수 온도 (실제값 Celsius, 예: 80 ~ 105) */
    uint8_t  coolant_temp;

    /** 현재 차속 (km/h, 예: 0 ~ 120) */
    uint8_t  vehicle_speed;

    /** RPM 램프 방향: 0 = 램프 업 (증가), 1 = 램프 다운 (감소) */
    uint8_t  rpm_direction;

    /** 온도 변화 방향: 0 = 증가, 1 = 감소 */
    uint8_t  temp_direction;

    /** 차속 변화 방향: 0 = 증가, 1 = 감소 */
    uint8_t  speed_direction;
} OBD2_SimState_t;

/* === DTC (Diagnostic Trouble Code) 시스템 ===
 * Mode 03(stored)/07(pending) 가 항상 빈 응답(numDTC=0)을 반환하던 한계 해소.
 * OBD2_DtcUpdate() 가 시뮬 값으로 fault 를 감지해 상태머신 갱신:
 *   INACTIVE → (debounce) → PENDING → (지속) → CONFIRMED
 *   - Mode 03 : CONFIRMED DTC 노출
 *   - Mode 07 : PENDING  DTC 노출
 *   - Mode 04 / RoutineControl 0x0201 : OBD2_DtcClear() 로 리셋
 * 조건 해제 시 PENDING 은 INACTIVE 로 회수되지만 CONFIRMED 는 clear 전까지 유지.
 */
typedef enum {
    DTC_STATE_INACTIVE = 0,
    DTC_STATE_PENDING,    /* Mode 07 (pending) 에 노출 */
    DTC_STATE_CONFIRMED   /* Mode 03 (stored)  에 노출 */
} DtcState_t;

typedef struct {
    uint16_t   code;            /* SAE J2010 DTC (P0217 → 0x0217) */
    DtcState_t state;
    uint8_t    debounce;        /* 조건 연속 감지 카운터 */
    uint8_t    hold;            /* PENDING 유지 카운터 (→ CONFIRMED) */
} DtcEntry_t;

#define OBD2_DTC_COUNT            3U
#define OBD2_DTC_DEBOUNCE_THRESH  5U    /* 5회(=50ms) 연속 감지 시 PENDING 승격 */
#define OBD2_DTC_CONFIRM_HOLD     50U   /* 50회(=500ms) PENDING 유지 시 CONFIRMED */

/* 감지 대상 DTC (SAE J2010 2바이트 인코딩) */
#define DTC_ENGINE_OVERTEMP       0x0217U  /* P0217: 냉각수 과온 (coolant >= MAX) */
#define DTC_VSS_MALFUNCTION       0x0500U  /* P0500: 차속=0 인데 고RPM */
#define DTC_COOLANT_THERMOSTAT    0x0128U  /* P0128: 냉각수 과냉 (워밍업 미완료) */

extern DtcEntry_t g_dtc_table[OBD2_DTC_COUNT];

/* === 순수 로직 API (CAN I/O 없음) === */

/**
 * @brief  OBD-II Mode 01 서비스 핸들러 (순수 로직)
 * @param  pid:     요청된 PID 번호
 * @param  pTxData: 응답 데이터 버퍼 (최소 8바이트)
 * @retval 응답 데이터 길이 (0 = 지원하지 않는 PID)
 * @note   UDS 디스패처에서 SID 0x01 수신 시 호출됨
 *         응답 형식: [len, 0x41, PID, data..., 0x00, 0x00, 0x00]
 *         CAN 송수신 없음 - 호출한 쪽이 전송 담당
 */
uint8_t OBD2_HandleService01(uint8_t pid, uint8_t *pTxData);

/**
 * @brief  시뮬레이션 상태 값을 주기적으로 업데이트 (10ms마다 호출)
 * @param  pState: 시뮬레이션 상태 구조체 포인터
 * @retval None
 * @note   메인 루프 또는 타이머 인터럽트에서 호출
 */
void OBD2_UpdateSimValues(OBD2_SimState_t *pState);

/**
 * @brief  PID 0x00 응답 생성: 지원 PID 비트맵
 * @param  pTxData: 전송 데이터 버퍼 (8바이트)
 * @retval 응답 데이터 길이 (DLC)
 */
uint8_t OBD2_GetSupportedPIDs(uint8_t *pTxData);

/**
 * @brief  PID 0x05 응답 생성: 냉각수 온도
 * @param  pTxData: 전송 데이터 버퍼
 * @param  temp: 냉각수 온도 (Celsius)
 * @retval 응답 데이터 길이 (DLC)
 */
uint8_t OBD2_GetCoolantTemp(uint8_t *pTxData, uint8_t temp);

/**
 * @brief  PID 0x0C 응답 생성: 엔진 RPM
 * @param  pTxData: 전송 데이터 버퍼
 * @param  rpm: 엔진 RPM
 * @retval 응답 데이터 길이 (DLC)
 */
uint8_t OBD2_GetEngineRPM(uint8_t *pTxData, uint16_t rpm);

/**
 * @brief  PID 0x0D 응답 생성: 차속
 * @param  pTxData: 전송 데이터 버퍼
 * @param  speed: 차속 (km/h)
 * @retval 응답 데이터 길이 (DLC)
 */
uint8_t OBD2_GetVehicleSpeed(uint8_t *pTxData, uint8_t speed);

/* === DTC (Fault) 관리 API === */

/**
 * @brief  시뮬 값으로 DTC 상태머신 갱신 (10ms 주기 호출)
 * @note   main 루프에서 OBD2_UpdateSimValues() 직후에 호출.
 *         fault 조건 연속 감지(debounce) → PENDING → (지속) → CONFIRMED.
 */
void OBD2_DtcUpdate(const OBD2_SimState_t *st);

/**
 * @brief  DTC 코드를 버퍼에 순차 기록 (2바이트/DTC, big-endian)
 * @param  out:       출력 버퍼 (최소 max_pairs*2 바이트)
 * @param  max_pairs: 기록할 최대 DTC 수
 * @retval 실제 기록한 DTC 수
 */
uint8_t OBD2_DtcGetConfirmed(uint8_t *out, uint8_t max_pairs);
uint8_t OBD2_DtcGetPending(uint8_t *out, uint8_t max_pairs);

/** 모든 DTC 를 INACTIVE 로 리셋 (Mode 04 / RoutineControl 0x0201) */
void OBD2_DtcClear(void);

/* === 전역 시뮬레이션 상태 === */
extern OBD2_SimState_t g_sim_state;

#ifdef __cplusplus
}
#endif

#endif /* __OBD2_SIMULATOR_H */
