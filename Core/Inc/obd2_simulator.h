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

/* === 전역 시뮬레이션 상태 === */
extern OBD2_SimState_t g_sim_state;

#ifdef __cplusplus
}
#endif

#endif /* __OBD2_SIMULATOR_H */
