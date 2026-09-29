# UDS 서비스 커버리지 — 구현 / 미구현 사유

> 이슈 [#10 "Can you add the all 27 UDS services?"](https://github.com/tae9898/obd-simulator/issues/10) 대응 문서.
> ISO 14229-1:2013 표준 서비스 26종 + :2020 추가 1종(0x29) 기준으로
> 구현 완료 / 명시적 미지원 / 2차 후보를 정리.
>
> 업데이트: 2026-09-29 · 구현체: `Core/Src/uds_service.c`

---

## 0. 요약

| 분류 | 개수 | 비고 |
|------|------|------|
| **구현됨 (UDS)** | **16** | 0x10, 0x11, 0x14, 0x19, 0x22, 0x23, 0x27, 0x28, 0x2E, 0x2F, 0x31, 0x34, 0x36, 0x37, 0x3E, 0x85 |
| **구현됨 (OBD-II)** | 5 | 0x01, 0x03, 0x04, 0x07, 0x09 (ISO 15031-5 브리지) |
| **2차 후보** | 2 | 0x2A, 0x2C — 실사용 가치 있음, 스케줄러 필요 |
| **명시적 미지원** | 9 | 아래 사유 표 참조. 물리 요청 → NRC 0x11, 기능 요청(0x7DF) → 무응답 |

**핵심 원칙**: 미구현 서비스도 ISO 규격 준수 방식으로 거부한다.
미지원 SID는 디스패처 `default:` 경로에서 `NRC 0x11 (serviceNotSupported)` 를
반환하고, 기능 주소(0x7DF) 요청은 규격대로 응답 자체를 억제한다.
즉 "미구현 = 규격 위반"이 아니라, **"해당 ECU가 지원하지 않는 서비스"라는
정당한 구현** 상태다.

---

## 1. 전체 서비스 표 (ISO 14229-1:2013 분류)

### Unit 1 — 진단/통신 관리 (Diagnostic and communication management)

| SID | 서비스 | 상태 | 세션/보안 | 비고 |
|-----|--------|------|-----------|------|
| 0x10 | DiagnosticSessionControl | ✅ 구현 | 전체 | 0x01/02/03, P2/P2* 응답 |
| 0x11 | ECU Reset | ✅ 구현 | 전체 | 0x01 Hard(NVIC), 0x03 Soft |
| 0x27 | SecurityAccess | ✅ 구현 | 전체 | Seed-Key, 3회 잠금 / 10s / 부팅 지연 |
| 0x28 | CommunicationControl | ✅ 구현 | Extended | 플래그 저장(수동), 기능 주소 허용 |
| 0x3E | TesterPresent | ✅ 구현 | 전체 | S3 타임아웃 유지, 기능 주소 허용 |
| 0x85 | ControlDTCSetting | ✅ 구현 **(신규)** | Extended | 아래 §2.3 |
| 0x83 | AccessTimingParameter | ❌ 미지원 | — | §3 참조 |
| 0x84 | SecuredDataTransmission | ❌ 미지원 | — | §3 참조 |
| 0x86 | ResponseOnEvent | ❌ 미지원 | — | §3 참조 |
| 0x87 | LinkControl | ❌ 미지원 | — | §3 참조 |

### Unit 2 — 데이터 전송 (Data transmission)

| SID | 서비스 | 상태 | 세션/보안 | 비고 |
|-----|--------|------|-----------|------|
| 0x22 | ReadDataByIdentifier | ✅ 구현 | 전체 | VIN/HW/SW/ECU명, DID 레지스트리 |
| 0x23 | ReadMemoryByAddress | ✅ 구현 **(신규)** | Extended+보안 | 아래 §2.2 |
| 0x2E | WriteDataByIdentifier | ✅ 구현 | Extended+보안 | VIN(0xF190)만 쓰기 허용 |
| 0x2A | ReadDataByPeriodicIdentifier | 🔶 2차 후보 | — | §4 참조 |
| 0x2C | DynamicallyDefineDataIdentifier | 🔶 2차 후보 | — | §4 참조 |
| 0x24 | ReadScalingDataByIdentifier | ❌ 미지원 | — | §3 참조 |
| 0x3D | WriteMemoryByAddress | ❌ 미지원 | — | §3 참조 |

### Unit 3 — 저장 데이터 전송 (Stored data transmission)

| SID | 서비스 | 상태 | 세션/보안 | 비고 |
|-----|--------|------|-----------|------|
| 0x19 | ReadDTCInformation | ✅ 구현 | 전체 | 전체 sub-function (이슈 #8) |
| 0x14 | ClearDiagnosticInformation | ✅ 구현 **(신규)** | Extended | 아래 §2.1 |

### Unit 4 — 입출력 (Input/output)

| SID | 서비스 | 상태 | 세션/보안 | 비고 |
|-----|--------|------|-----------|------|
| 0x2F | InputOutputControlByIdentifier | ✅ 구현 | Extended+보안 | DID 0x0200 가상 IO |

### Unit 5 — 루틴 (Routine)

| SID | 서비스 | 상태 | 세션/보안 | 비고 |
|-----|--------|------|-----------|------|
| 0x31 | RoutineControl | ✅ 구현 | Extended+보안 | 0x0201 DTC Clear, 0x0202 Self Test |

### Unit 6 — 업로드/다운로드 (Upload/download)

| SID | 서비스 | 상태 | 세션/보안 | 비고 |
|-----|--------|------|-----------|------|
| 0x34 | RequestDownload | ✅ 구현 | Extended+보안 | OTA 리전 검증 + 소거 |
| 0x36 | TransferData | ✅ 구현 | Extended+보안 | 블록 시퀀스 + 플래시 쓰기 |
| 0x37 | RequestTransferExit | ✅ 구현 | Extended+보안 | CRC/서명 생략(시뮬레이터) |
| 0x35 | RequestUpload | ❌ 미지원 | — | §3 참조 |
| 0x38 | RequestFileTransfer | ❌ 미지원 | — | §3 참조 |

### ISO 14229-1:2020 추가

| SID | 서비스 | 상태 | 비고 |
|-----|--------|------|------|
| 0x29 | Authentication | ❌ 해당 없음 | :2020 서비스 — 프로젝트는 :2013 기준 |

> 이슈 #10의 "27개" 표기는 :2020의 0x29 Authentication을 포함한 표에서 온 것으로
> 추정 (:2013 표준 서비스는 26종). 본 프로젝트는 :2013 기준으로 판단한다.

---

## 2. 이번에 구현한 서비스 (이슈 #10 1차)

### 2.1 — 0x14 ClearDiagnosticInformation

- **요청**: `[0x14, 0xFF, 0xFF, 0xFF]` (groupOfDTC = 전체 그룹만 지원, ISO 11.2.1)
- **응답**: `[0x54]`
- **백엔드**: 기존 `OBD2_DtcClear()` 재사용 (Mode 04 / RoutineControl 0x0201과 동일 경로).
  DTC 상태·스냅샷 리셋 + CONFIRMED 항목 미러 메모리 아카이브.
- **NRC**: 길이 오류 0x13 / group 값 오류 0x31 / Default 세션 0x33
- **시뮬레이터 한계** (기존과 동일): permanent DTC도 함께 지워짐
  (실차는 모니터 3사이클 통과 시에만 소거).

### 2.2 — 0x23 ReadMemoryByAddress

- **요청**: `[0x23, ALFID, address..., memorySize...]` — ALFID 파싱은 0x34와 동일
- **응답**: `[0x63, data...]`
- **읽기 허용 리전** (STM32G431RB 메모리맵, 직접 읽기):
  - Flash `0x08000000 ~ 0x0801FFFF` (128KB 전체 — 벡터 테이블·OTA 리전 포함)
  - SRAM `0x20000000 ~ 0x20007FFF` (SRAM1+2, 32KB)
  - 그 외(주변기기 레지스터 등) → NRC 0x31
- **크기 제한**: 최대 63바이트 (CAN-FD 단일 SF 응답). 초과 → NRC 0x31,
  테스터가 분할 요청.
- **보안**: Extended + SecurityAccess 필요 (메모리 읽기 = 펌웨어 노출).
  0x2E(쓰기)와 동일 정책.
- **용도**: 0x36으로 기록한 OTA 데이터 read-back 검증, 벡터 테이블 확인.

### 2.3 — 0x85 ControlDTCSetting

- **요청**: `[0x85, 0x01]` on / `[0x85, 0x02]` off
- **응답**: `[0xC5, sub]`
- **동작**: off 시 `OBD2_DtcUpdate()` 상태 머신 동결 — 플래시 시퀀스 중
  일시적 결함 기록 방지 (실제 OTA 흐름: 0x85 off → 0x34/0x36/0x37 → 0x85 on).
- **자동 복원**: Default 세션 진입 시 on 복원 (ISO 14229-1 8.4, `DiagSession_SetSession`),
  ECU 리셋 시에도 복원.
- **기능 주소(0x7DF) 허용**: 플래시 세션에서 전 ECU 브로드캐스트가 표준 사용례.
- **NRC**: sub 오류 0x12 / Default 세션 0x33

---

## 3. 명시적 미지원 사유 (9종)

| SID | 서비스 | 미지원 사유 | 필요 시 대응 |
|-----|--------|-------------|--------------|
| 0x24 | ReadScalingDataByIdentifier | DID 스케일링 메타데이터 모델이 없고, 실제 ECU에서도 지원이 드묾 (미지원 시 NRC 0x11이 규격상 정상) | DID 레지스트리에 scale 항목 추가 |
| 0x3D | WriteMemoryByAddress | 임의 주소 쓰기는 코드 영역 붕괴 위험 대비 시뮬레이터 가치가 낮음 — 플래시 쓰기는 0x36 TransferData가 이미 커버 | 0x23과 대칭 구조로 확장 (리전 검증 재사용) |
| 0x35 | RequestUpload | 0x34의 대칭 구조라 구현은 쉽지만, 업로드 시나리오(시뮬레이터→테스터 펌웨어 전송)가 없음 | 0x34 핸들러 재사용, 소규모 확장 |
| 0x38 | RequestFileTransfer | 파일시스템이 없음 (임베디드 베어메탈) | FatFS 도입 시에만 의미 있음 |
| 0x83 | AccessTimingParameter | 타이밍 협상은 전송 계층(ISO 15765-2) 영역 — 현재 ISO-TP 구현은 고정 타이밍이며 진단 응답 지연 없음 | iso_tp.c 타이밍 파라미터화 시 함께 |
| 0x84 | SecuredDataTransmission | ISO 14229-2 인증 기반, 상용 ECU에서도 구현이 극히 드묾 (참고repo CHARON-Server도 미구현) | 인증 프레임워크 도입 시에만 |
| 0x86 | ResponseOnEvent | 이벤트 조건 모니터링 + 능동 송신 프레임워크 필요, 실사용 사례 드묾 | 이벤트 트리거 태스크 추가 시 함께 |
| 0x87 | LinkControl | baudrate 전환 = FDCAN 실제 재초기화 필요 — 테스트 환경(gs_usb can0)도 함께 재설정해야 해서 검증 비용이 큼 | 하드웨어 재구성 감수 시 fdcan_config 확장 |
| 0x29 | Authentication | ISO 14229-1:**2020** 서비스 — 프로젝트는 :2013 기준 | :2020 전환 시 0x27 확장 형태로 |

---

## 4. 2차 후보 (0x2A / 0x2C)

| SID | 서비스 | 가치 | 필요 작업 |
|-----|--------|------|-----------|
| 0x2C | DynamicallyDefineDataIdentifier | 여러 DID/오프셋을 조합한 가상 DID 정의 — 테스터 고급 시나리오 | 정의 테이블(최대 2~3개 슬롯) + 0x22 경유 읽기 |
| 0x2A | ReadDataByPeriodicIdentifier | 지정 DID 주기 자동 송신 (SlowRate/MediumRate/FastRate) | ISO-TP TX 태스크에 주기 송신 스케줄러 추가 |

두 서비스는 세트로 설계(0x2C로 정의 → 0x2A로 주기 조회)하는 것이 표준적.
**선행 조건**: RAM 여유 확인 — 주기 송신 버퍼·스케줄러가 현재 대역폭
(2026-07 기준 RAM ~73%)에서 들어가는지 점검 필요.
`UDS_IsFunctionallyAddressable`·suppressPosRsp 처리는 0x2A만 sub-function 기반.

---

## 5. 테스트 커버리지

`tests/uds_test.py` — 신규 3개 서비스 와이어 계약 테스트 추가:

| 테스트 | 항목 |
|--------|------|
| `test_clear_dtc` | Default 세션 거부(0x33) / 전체 그룹 소거(0x54) / 잘못된 group(0x31) / 길이 오류(0x13) |
| `test_control_dtc_setting` | Default 세션 거부 / on(0xC5 01) / off(0xC5 02) / 미지원 sub(0x12) |
| `test_read_mem_by_addr` | Default 세션 거부 / 잠금 상태 거부 / 플래시 읽기@0x08000000 (MSP값 검증으로 실제 메모리 내용 확인) / 불가 리전(0x31) / 63B 초과(0x31) |

하드웨어 실행 (테스트 리그: STLINK-V3 + gs_usb can0):

```bash
# 플래시 후
sudo python3 tests/uds_test.py can0
```

---

## 6. 이슈 #10 대응 결론

1. **1차 구현 완료**: 0x14, 0x23, 0x85 — 실사용 빈도·구현 비용 기준 상위 3개.
2. **2차 후보**: 0x2A, 0x2C — 스케줄러 설계 후 착수.
3. **나머지 9종**: 위 §3 사유로 명시적 미지원. NRC 0x11 응답은 ISO 규격상
   정당하며, 전수 구현은 시뮬레이터 목적(UDS 학습·테스트 도구)에 부합하지 않음.
