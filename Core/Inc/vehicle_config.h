/**
 * @file    vehicle_config.h
 * @brief   ECU 정체 정보 (차량/ECU 식별) 단일 설정 포인트
 * @note    UDS ReadDataByIdentifier (DID 0xF190 VIN / 0xF193 HW / 0xF195 SW /
 *          0xF198 ECU 이름) 와 OBD-II Mode 09 (VIN) 이 공유하는 정체 문자열.
 *          보드/빌드별 커스터마이징은 이 헤더 한 곳에서.
 */
#ifndef __VEHICLE_CONFIG_H
#define __VEHICLE_CONFIG_H

/* === 차량 식별 — ISO 3779 기준 VIN 17자리 === */
#define VEHICLE_VIN             "WVWZZZ3CZWE000001"

/* === ECU 정체 (UDS DID 0xF193/0xF195/0xF198) === */
#define VEHICLE_HW_VERSION      "HW Rev1.0"
#define VEHICLE_SW_VERSION      "SW Phase1.0"
#define ECU_NAME                "OBD-SIM-G431"

#endif /* __VEHICLE_CONFIG_H */
