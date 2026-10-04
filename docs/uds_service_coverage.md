# UDS 서비스 커버리지 — 구현 / 미구현 사유

> 이슈 [#10 "Can you add the all 27 UDS services?"](https://github.com/tae9898/obd-simulator/issues/10) 대응 문서.
> ISO 14229-1:2013 표준 서비스 26종 + :2020 추가 1종(0x29) 기준으로
> 구현 완료 / 명시적 미지원 / 2차 후보를 정리.
>
> 업데이트: 2026-09-30 (0x2A/0x2C 구현 완료) · 구현체: `Core/Src/uds_service.c`

---

## 0. 요약

| 분류 | 개수 | 비고 |
|------|------|------|
| **구현됨 (UDS)** | **18** | 0x10, 0x11, 0x14, 0x19, 0x22, 0x23, 0x27, 0x28, 0x2A, 0x2C, 0x2E, 0x2F, 0x31, 0x34, 0x36, 0x37, 0x3E, 0x85 |
| **구현됨 (OBD-II)** | 5 | 0x01, 0x03, 0x04, 0x07, 0x09 (ISO 15031-5 브리지) |
| **명시적 미지원** | 8 | 아래 사유 표 참조. 물리 요청 → NRC 0x11, 기능 요청(0x7DF) → 무응답 |

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
| 0x22 | ReadDataByIdentifier | ✅ 구현 | 전체 | VIN/HW/SW/ECU명 + 0x2C 동적 DID |
| 0x23 | ReadMemoryByAddress | ✅ 구현 | Extended+보안 | §2.2 |
| 0x2A | ReadDataByPeriodicIdentifier | ✅ 구현 **(2차)** | Extended | §2.4 |
| 0x2C | DynamicallyDefineDataIdentifier | ✅ 구현 **(2차)** | Extended | §2.5 |
| 0x2E | WriteDataByIdentifier | ✅ 구현 | Extended+보안 | VIN(0xF190)만 쓰기 허용 |
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

## 2. 이슈 #10로 구현한 서비스 (1차: §2.1~2.3 / 2차: §2.4~2.5)

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

### 2.4 — 0x2A ReadDataByPeriodicIdentifier (2차, 2026-09-30)

- **요청**: `[0x2A, mode, pdid...]` — pdid는 1바이트 (상위 바이트 `0xF2` 고정,
  즉 `0xF2xx` DID의 하위 바이트). 한 요청에 여러 pdid 지정 가능.
- **mode**: 0x01 slow(1s) / 0x02 medium(100ms) / 0x03 fast(20ms) /
  0x04 stop (pdid 없으면 전체 정지)
- **즉시 응답**: `[0x6A, mode]`, 이후 주기 프레임은 ISO 형식
  `[pdid(1B) + data]` — **RSID 없음** (py-uds는 `0x6A`를 붙인 확장 형식을
  기본으로 쓰지만 본 구현은 ISO 규격 형식만 따름)
- **구독 조건**: pdid는 0x2C로 정의된 `0xF2xx` 동적 DID여야 함 (미정의 → 0x31)
- **스케줄러**: `UDS_PeriodicTick()` — 메인 루프 10ms 틱에서 동작,
  구독 슬롯 4개. 조립 실패(0x2C clear/redefine) 시 자동 구독 해제.
- **자동 정지**: Default 세션 진입 / S3 타임아웃 / ECU 리셋
  (`DiagSession_SetSession` → `UDS_PeriodicStopAll`)

### 2.5 — 0x2C DynamicallyDefineDataIdentifier (2차, 2026-09-30)

- **sub 0x01 defineByIdentifier**: `[0x2C 01 dynDID(2) {srcDID(2) pos(1) size(1)}+]`
  — 기존 DID(VIN/HW/SW/ECU명/IO 0x0200)에서 슬라이스 조합. pos는 1부터 시작.
- **sub 0x02 defineByMemoryAddress**: `[0x2C 02 dynDID(2) ALFID {addr size}+]`
  — 0x23과 동일한 리전 검증(Flash/SRAM만) 재사용.
- **sub 0x03 clear**: `[0x2C 03 dynDID(2)]` — 구독 중이면 다음 조립 실패로 자동 정지.
- **dynDID 범위**: `0xF200-0xF3FF`. 0x2A 주기 구독은 `0xF2xx`만 가능
  (PDID가 하위 1바이트라서).
- **읽기**: 0x22 ReadDataByIdentifier로 일반 DID와 동일하게 조회
  (`[0x62 dynDID data...]`). 동적 DID 중첩 정의는 미지원 (소스는 정적 DID만).
- **제한**: 슬롯 4개 × 레코드 3개, 조립 크기 ≤ 56B (0x22 응답/0x2A SF 수용).
- **NRC**: 길이 0x13 / 범위·소스·리전 오류 0x31 / 미지원 sub 0x12

---

## 3. 명시적 미지원 사유 (8종)

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

## 4. 2차 후보 → 구현 완료 (2026-09-30)

0x2A/0x2C 2종은 §2.4/§2.5로 구현 완료됨. RAM 영향 +176B (bss 22068/32768, 67%).

---

## 5. 테스트 커버리지

`tests/uds_test.py` — 신규 3개 서비스 와이어 계약 테스트 추가:

| 테스트 | 항목 |
|--------|------|
| `test_clear_dtc` | Default 세션 거부(0x33) / 전체 그룹 소거(0x54) / 잘못된 group(0x31) / 길이 오류(0x13) |
| `test_control_dtc_setting` | Default 세션 거부 / on(0xC5 01) / off(0xC5 02) / 미지원 sub(0x12) |
| `test_read_mem_by_addr` | Default 세션 거부 / 잠금 상태 거부 / 플래시 읽기@0x08000000 (MSP값 검증으로 실제 메모리 내용 확인) / 불가 리전(0x31) / 63B 초과(0x31) |
| `test_dynamic_define_did` | 세션 거부 / DID 슬라이스 정의 + 0x22 조립 조회 / 메모리 주소 정의 + 0x23 직접 읽기와 일치 검증 / 알 수 없는 소스·범위 초과·영역 외·불가 리전(0x31) / clear 후 0x31 |
| `test_periodic_read` | 세션 거부 / fast 구독(250ms에 13프레임, 데이터 일치) / stop 후 침묵 / medium 속도 / Default 세션 자동 정지 |

하드웨어 실행 (테스트 리그: STLINK-V3 + canable/candleLight can0) — **95/95 PASS (2026-09-30)**:

```bash
# 플래시 후
sudo python3 tests/uds_test.py can0
```

---

## 6. 이슈 #10 대응 결론

1. **1차 구현 완료 (2026-09-29)**: 0x14, 0x23, 0x85.
2. **2차 구현 완료 (2026-09-30)**: 0x2A, 0x2C — 주기 송신 스케줄러 포함.
3. **나머지 8종**: 위 §3 사유로 명시적 미지원. NRC 0x11 응답은 ISO 규격상
   정당하며, 전수 구현은 시뮬레이터 목적(UDS 학습·테스트 도구)에 부합하지 않음.

표준 26종 중 18종 구현 + 8종 정당 거부 = 이슈 #10 요청에 대한 완결 대응.
