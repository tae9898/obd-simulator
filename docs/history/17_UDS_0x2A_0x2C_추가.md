# 17. UDS 0x2A / 0x2C 서비스 추가 (이슈 #10 2차)

> **날짜:** 2026-09-30 (구현 + 하드웨어 검증 동일일)
> **이슈:** [#10 "Can you add the all 27 UDS services?"](https://github.com/tae9898/obd-simulator/issues/10)
> **관련 문서:** `docs/uds_service_coverage.md` §2.4/§2.5 (서비스 상세 규격)

---

## 1. 개요

이슈 #10의 2차 대응. 16장(1차: 0x14/0x23/0x85)에서 "2차 후보"로 남겨둔 두 서비스를
세트로 구현했다 — 0x2C로 가상 DID를 정의하고 0x2A로 주기 조회하는 표준 사용례 지원.

| SID | 서비스 | 한 줄 요약 |
|-----|--------|-----------|
| 0x2C | DynamicallyDefineDataIdentifier | 기존 DID 슬라이스/메모리 주소로 동적 DID 정의 (0xF200-0xF3FF) |
| 0x2A | ReadDataByPeriodicIdentifier | 동적 DID 주기 자동 송신 (slow 1s / medium 100ms / fast 20ms) |

구현 서비스 수 16 → **18**. 상세 규격(요청/응답 형식, NRC, 제한)은
`docs/uds_service_coverage.md` §2.4/§2.5 참조.

---

## 2. 수정 파일 목록

| 파일 | 변경 | 내용 |
|------|------|------|
| `Core/Inc/uds_service.h` | +20 | SID 2개, 동적 DID 범위/슬롯 상수, `UDS_PeriodicTick()`/`UDS_PeriodicStopAll()` API |
| `Core/Src/uds_service.c` | +478 | 동적 DID 테이블(4슬롯×3레코드), 주기 구독 스케줄러, 0x22 동적 DID 조회 확장, 핸들러 2개, 디스패치+suppress 처리 |
| `Core/Src/diag_session.c` | +4 | 접근 제어 2행 (Extended only), Default 진입 시 `UDS_PeriodicStopAll()` |
| `Core/Src/main.c` | +4 | 메인 루프에 `UDS_PeriodicTick()` 연결, 배너 18 SID 갱신 |
| `tests/uds_test.py` | +193 | `test_dynamic_define_did`, `test_read_data_periodic` (+18 케이스) |
| `docs/uds_service_coverage.md` | 갱신 | §2.4/§2.5 추가, 2차 후보 절 → 구현 완료로 전환 |

합계: 746 insertions / 28 deletions

---

## 3. 설계 요점

- **동적 DID 테이블**: 4슬롯 × 레코드 3개, 조립 크기 ≤ 56B (0x22 응답/0x2A SF 수용).
  소스는 정적 DID 또는 메모리 주소(0x23과 동일한 Flash/SRAM 리전 검증 재사용).
  동적 DID 중첩 정의는 미지원.
- **주기 스케줄러**: 메인 루프 10ms 틱의 분주(100/10/2)로 3속도 구현.
  주기 프레임은 ISO 규격 형식 `[pdid(1B) + data]` — RSID 없음
  (py-uds 확장 형식과 다름, 테스트도 ISO 형식으로 검증).
- **자동 정지**: Default 세션 진입(S3 타임아웃 포함)·구독 중 조립 실패(0x2C clear/redefine).
  `DiagSession_SetSession()` → `UDS_PeriodicStopAll()` 1줄로 세 경로 모두 처리.
- **크리티컬 섹션**: 주기 송신(main 틱)과 0x22/0x2C 핸들러(ISO-TP 태스크)가
  동적 DID 테이블을 공유 → 조립/조회 경로는 기존 `taskENTER_CRITICAL` 패턴 유지.
- **RAM 영향**: +176B (bss 22068/32768 = 67%, 여유 확인 후 도입 — 1차 문서의 선행조건 충족).

---

## 4. 하드웨어 검증 (2026-09-30, 리그: STLINK-V3 + canable/candleLight can0)

**결과: 95/95 PASS** (`sudo python3 tests/uds_test.py can0`, FD 500k/2M)

| 확인 항목 | 결과 |
|-----------|------|
| 0x2C DID 슬라이스 정의 + 0x22 조립 조회 | ✅ |
| 0x2C 메모리 주소 정의 → 0x23 직접 읽기와 일치 | ✅ |
| 0x2A fast 구독 (250ms에 13프레임, 데이터 일치) | ✅ |
| 0x2A stop / Default 세션 자동 정지 | ✅ |
| 범위·소스·리전 오류 NRC 0x31 / 미지원 sub 0x12 | ✅ |
| 기존 77케이스 회귀 | ✅ |

---

## 5. 남은 작업

- [x] 2차 구현 (본 장)
- [ ] 이슈 #10 응답 코멘트는 전체 커버리지 정리 후 일괄 (M4 부트로더 완성과 함께 검토)
