/**
 * @file    obd2_simulator.c
 * @brief   OBD-II ECU 시뮬레이터 구현
 * @note    CAN 요청 파싱, PID별 응답 생성, 시뮬레이션 값 주기적 업데이트
 */

#include "obd2_simulator.h"
#include <string.h>

/* === 시뮬레이션 상태 전역 변수 (정의는 main.c에 있음) === */

/**
 * @brief  OBD-II Mode 01 순수 로직 핸들러
 * @param  pid:     요청된 PID
 * @param  pTxData: 응답 버퍼 (최소 8바이트)
 * @retval 응답 길이 (0 = 미지원 PID)
 * @note   CAN I/O 없음. UDS 디스패처에서 호출됨.
 */
uint8_t OBD2_HandleService01(uint8_t pid, uint8_t *pTxData)
{
    if (pTxData == NULL) {
        return 0U;
    }

    (void)memset(pTxData, 0, 8);

    switch (pid) {
        case OBD2_PID_SUPPORTED_PIDS:
            return OBD2_GetSupportedPIDs(pTxData);
        case OBD2_PID_COOLANT_TEMP:
            return OBD2_GetCoolantTemp(pTxData, g_sim_state.coolant_temp);
        case OBD2_PID_ENGINE_RPM:
            return OBD2_GetEngineRPM(pTxData, g_sim_state.engine_rpm);
        case OBD2_PID_VEHICLE_SPEED:
            return OBD2_GetVehicleSpeed(pTxData, g_sim_state.vehicle_speed);
        default:
            return 0U;
    }
}

/**
 * @brief  수신된 CAN 메시지를 파싱하고 OBD-II 요청 처리 (Phase 0 호환)
 * @param  pRxHeader: 수신 CAN 메시지 헤더
 * @param  pRxData:   수신 CAN 데이터
 *
 * @note   OBD-II 요청 형식:
 *         [len, ServiceMode, PID, 0x00, 0x00, 0x00, 0x00, 0x00]
 *         예: [02, 0x01, 0x0C, 00, 00, 00, 00, 00] -> RPM 요청
 *
 *         OBD-II 응답 형식:
 *         [len, ServiceMode+0x40, PID, data..., 0x00, 0x00, 0x00]
 *         예: [04, 0x41, 0x0C, A, B8, 00, 00, 00] -> RPM 응답
 */
void OBD2_ProcessRequest(const FDCAN_RxHeaderTypeDef *pRxHeader,
                          const uint8_t *pRxData)
{
    uint8_t  tx_data[8] = {0};
    uint8_t  tx_len     = 0;
    uint8_t  pid;

    /* --- 순수 로직 함수 호출 --- */
    (void)pRxHeader;  /* Phase 0 호환성: 아직 ISR에서 직접 호출됨 */

    if (pRxData == NULL) {
        return;
    }

    /* 최소 [len, sid, pid] 필요 */
    if (pRxData[0] < 2U) {
        return;
    }

    pid = pRxData[2];
    tx_len = OBD2_HandleService01(pid, tx_data);

    /* --- CAN 응답 전송 (Phase 0 호환성 유지) --- */
    if (tx_len > 0U) {
        FDCAN_TxHeaderTypeDef tx_header;
        tx_header.Identifier          = OBD2_RESPONSE_ID;
        tx_header.IdType              = FDCAN_STANDARD_ID;
        tx_header.TxFrameType         = FDCAN_DATA_FRAME;
        tx_header.DataLength          = FDCAN_DLC_BYTES_8;
        tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header.BitRateSwitch       = FDCAN_BRS_ON;
        tx_header.FDFormat            = FDCAN_FD_CAN;
        tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
        tx_header.MessageMarker       = 0U;

        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
    }
}

/**
 * @brief  시뮬레이션 상태 값을 주기적으로 업데이트
 * @param  pState: 시뮬레이션 상태 구조체
 *
 * @note   10ms마다 호출됨:
 *         - RPM: 800~4000 RPM 램프 업/다운 반복 (10 RPM/10ms = 1000 RPM/s)
 *         - 냉각수 온도: 80~105°C 서서히 변화 (0.1°C/10ms = 10°C/s)
 *         - 차속: 0~120 km/h 서서히 변화 (1 km/h/10ms = 100 km/h/s)
 */
void OBD2_UpdateSimValues(OBD2_SimState_t *pState)
{
    /* --- 엔진 RPM 램프 시뮬레이션 --- */
    if (pState->rpm_direction == 0U) {
        /* 램프 업: RPM 증가 */
        pState->engine_rpm += RPM_RAMP_STEP;
        if (pState->engine_rpm >= RPM_MAX) {
            pState->engine_rpm = RPM_MAX;
            pState->rpm_direction = 1U;  /* 방향 전환: 램프 다운 */
        }
    } else {
        /* 램프 다운: RPM 감소 */
        if (pState->engine_rpm > RPM_IDLE) {
            if (pState->engine_rpm < (RPM_IDLE + RPM_RAMP_STEP)) {
                pState->engine_rpm = RPM_IDLE;
            } else {
                pState->engine_rpm -= RPM_RAMP_STEP;
            }
        }
        if (pState->engine_rpm <= RPM_IDLE) {
            pState->engine_rpm = RPM_IDLE;
            pState->rpm_direction = 0U;  /* 방향 전환: 램프 업 */
        }
    }

    /* --- 냉각수 온도 서서히 변화 --- */
    if (pState->temp_direction == 0U) {
        /* 온도 증가 */
        pState->coolant_temp += COOLANT_TEMP_STEP;
        if (pState->coolant_temp >= COOLANT_TEMP_MAX) {
            pState->coolant_temp = COOLANT_TEMP_MAX;
            pState->temp_direction = 1U;
        }
    } else {
        /* 온도 감소 */
        if (pState->coolant_temp > COOLANT_TEMP_MIN) {
            if (pState->coolant_temp < (COOLANT_TEMP_MIN + COOLANT_TEMP_STEP)) {
                pState->coolant_temp = COOLANT_TEMP_MIN;
            } else {
                pState->coolant_temp -= COOLANT_TEMP_STEP;
            }
        }
        if (pState->coolant_temp <= COOLANT_TEMP_MIN) {
            pState->coolant_temp = COOLANT_TEMP_MIN;
            pState->temp_direction = 0U;
        }
    }

    /* --- 차속 서서히 변화 --- */
    if (pState->speed_direction == 0U) {
        /* 차속 증가 */
        pState->vehicle_speed += VEHICLE_SPEED_STEP;
        if (pState->vehicle_speed >= VEHICLE_SPEED_MAX) {
            pState->vehicle_speed = VEHICLE_SPEED_MAX;
            pState->speed_direction = 1U;
        }
    } else {
        /* 차속 감소 */
        if (pState->vehicle_speed > VEHICLE_SPEED_MIN) {
            pState->vehicle_speed -= VEHICLE_SPEED_STEP;
        }
        if (pState->vehicle_speed <= VEHICLE_SPEED_MIN) {
            pState->vehicle_speed = VEHICLE_SPEED_MIN;
            pState->speed_direction = 0U;
        }
    }
}

/**
 * @brief  PID 0x00: 지원 PID 비트맵 응답 생성
 *
 * @note   비트맵 형식: 각 비트가 해당 PID 지원 여부를 나타냄
 *         Byte 4 (PID 0x00~0x07): Bit0=PID01, Bit1=PID02, ...
 *         Byte 5 (PID 0x08~0x0F): Bit0=PID09, Bit1=PID0A, ...
 *         Byte 6 (PID 0x10~0x17): Bit0=PID11, ...
 *         Byte 7 (PID 0x18~0x1F): Bit0=PID19, ...
 *
 *         지원 PID: 0x05, 0x0C, 0x0D
 *         Byte 4: Bit4(0x05) -> 0x10
 *         Byte 5: Bit4(0x0C), Bit5(0x0D) -> 0x18
 *         Byte 6: 0x00
 *         Byte 7: 0x00
 */
uint8_t OBD2_GetSupportedPIDs(uint8_t *pTxData)
{
    /* 응답: [len, 0x41, 0x00, bitmap4, bitmap5, 0x00, 0x00, 0x00] */
    pTxData[0] = 6U;  /* ISO-TP Single Frame: 하위 니블 = 페이로드 길이 */
    pTxData[1] = OBD2_MODE_RESPONSE_PREFIX + OBD2_MODE_CURRENT_DATA;  /* 0x41 */
    pTxData[2] = OBD2_PID_SUPPORTED_PIDS;                              /* 0x00 */

    /* PID 0x01~0x07 비트맵: PID 0x05(bit4) 지원 */
    pTxData[3] = (1U << 4);  /* 0x10 */

    /* PID 0x09~0x0F 비트맵: PID 0x0C(bit3), PID 0x0D(bit4) 지원 */
    pTxData[4] = (1U << 3) | (1U << 4);  /* 0x18 */

    /* PID 0x11~0x17, 0x19~0x1F 비트맵: 지원 없음 */
    pTxData[5] = 0x00;
    pTxData[6] = 0x00;
    pTxData[7] = 0x00;

    return 6U;  /* 실제 전송 DLC */
}

/**
 * @brief  PID 0x05: 냉각수 온도 응답 생성
 * @param  pTxData: 전송 데이터 버퍼
 * @param  temp:    냉각수 온도 (Celsius, 예: 90)
 *
 * @note   OBD-II 인코딩: A = temp + 40 (예: 90C -> A = 130 = 0x82)
 *         응답: [03, 0x41, 0x05, A, 00, 00, 00, 00]
 */
uint8_t OBD2_GetCoolantTemp(uint8_t *pTxData, uint8_t temp)
{
    uint8_t encoded = (uint8_t)(temp + 40U);

    pTxData[0] = 3U;  /* ISO-TP Single Frame: 3바이트 페이로드 */
    pTxData[1] = OBD2_MODE_RESPONSE_PREFIX + OBD2_MODE_CURRENT_DATA;  /* 0x41 */
    pTxData[2] = OBD2_PID_COOLANT_TEMP;                                /* 0x05 */
    pTxData[3] = encoded;
    pTxData[4] = 0x00;
    pTxData[5] = 0x00;
    pTxData[6] = 0x00;
    pTxData[7] = 0x00;

    return 3U;
}

/**
 * @brief  PID 0x0C: 엔진 RPM 응답 생성
 * @param  pTxData: 전송 데이터 버퍼
 * @param  rpm:     엔진 RPM (예: 2500)
 *
 * @note   OBD-II 인코딩: ((A*256) + B) / 4 = RPM
 *         따라서: raw_value = RPM * 4 (예: 2500 * 4 = 10000 = 0x2710)
 *         A = raw_value >> 8, B = raw_value & 0xFF
 *         응답: [04, 0x41, 0x0C, A, B, 00, 00, 00]
 */
uint8_t OBD2_GetEngineRPM(uint8_t *pTxData, uint16_t rpm)
{
    uint32_t raw_value = (uint32_t)rpm * 4U;

    pTxData[0] = 4U;  /* ISO-TP Single Frame: 4바이트 페이로드 */
    pTxData[1] = OBD2_MODE_RESPONSE_PREFIX + OBD2_MODE_CURRENT_DATA;  /* 0x41 */
    pTxData[2] = OBD2_PID_ENGINE_RPM;                                  /* 0x0C */
    pTxData[3] = (uint8_t)(raw_value >> 8U);  /* 상위 바이트 */
    pTxData[4] = (uint8_t)(raw_value & 0xFFU); /* 하위 바이트 */
    pTxData[5] = 0x00;
    pTxData[6] = 0x00;
    pTxData[7] = 0x00;

    return 4U;
}

/**
 * @brief  PID 0x0D: 차속 응답 생성
 * @param  pTxData: 전송 데이터 버퍼
 * @param  speed:   차속 (km/h, 예: 60)
 *
 * @note   OBD-II 인코딩: A = speed (1 km/h 단위)
 *         응답: [03, 0x41, 0x0D, A, 00, 00, 00, 00]
 */
uint8_t OBD2_GetVehicleSpeed(uint8_t *pTxData, uint8_t speed)
{
    pTxData[0] = 3U;  /* ISO-TP Single Frame: 3바이트 페이로드 */
    pTxData[1] = OBD2_MODE_RESPONSE_PREFIX + OBD2_MODE_CURRENT_DATA;  /* 0x41 */
    pTxData[2] = OBD2_PID_VEHICLE_SPEED;                               /* 0x0D */
    pTxData[3] = speed;
    pTxData[4] = 0x00;
    pTxData[5] = 0x00;
    pTxData[6] = 0x00;
    pTxData[7] = 0x00;

    return 3U;
}
