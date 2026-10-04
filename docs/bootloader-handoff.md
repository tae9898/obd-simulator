# UDS OTA 부트로더(FBL) 구현 — 핸드오프 문서

> 2026-10-04, sdv-diag-gateway 세션에서 작성. 이 문서를 읽고 바로 플랜 모드부터 시작할 것.

## 1. 배경

- 외부 이슈 [tae9898/sdv-diag-gateway#1](https://github.com/tae9898/sdv-diag-gateway/issues/1):
  "STM32F767 UDS OTA 부트로더를 만들어 달라" — 8단계 플래싱 시퀀스 + A/B 파티션 + 안티롤백 요구
  (ElectRay 상품 광고 섞인 요청이지만 요구사항 자체는 정통 FBL 스펙)
- 스코프 판정 완료: 게이트웨이(리눅스)가 아니라 **ECU 펌웨어(=이 레포) 작업**. 게이트웨이는 UDS를 투명 릴레이하므로 수정 불요.
- 이슈의 F767 요구는 기각: 실제 하드웨어는 **Nucleo-G431RB** (G431RBT6, 128KB flash / 32KB RAM).

## 2. 하드웨어 제약 → 설계 결정 (이미 확정된 사항)

**실측 근거 (2026-10-04 빌드 기준):**

| 항목 | 값 |
|---|---|
| 링커 | `STM32G431RBTX_FLASH.ld` — FLASH 0x08000000 128K, RAM 32K |
| 현재 앱 크기 | `build/phase0-obd-simulator.elf` = text 67,948 + data 220 ≈ **66.6KB** |
| 기존 OTA 영역 | `Core/Inc/ota_flash.h` — 0x0801E000~0x08020000 (마지막 8KB) |
| 플래시 뱅크 | 싱글뱅크 (`ota_flash.c:21` 주석), 페이지 2KB |

**결정: 클래식 FBL 방식 (부트로더가 직접 플래싱).**

이유: 진짜 A/B(앱 이미지 2벌)는 66.6KB×2 + BL + 메타 > 128KB로 **물리적으로 불가**.
이슈의 8단계 시퀀스(0x10 02 → 0x27 → 0x85/0x28 → 0x34 → 0x36 → 0x37 → 0x31 → 0x11)는
본래 BL이 플래싱 주체인 이 방식을 기술한 것 — 오히려 원 스펙에 부합.
브릭 안전성: 업데이트 도중 전원 단절 → 앱 슬롯 CRC 불일치 → BL이 UDS 서버로 대기 → 재플래시 가능.

**새 플래시 맵:**

```
0x0800_0000  BL        ~20KB  (bare-metal, UDS 플래시 서버: 0x10 02/0x27/0x28/0x85/0x34/0x36/0x37/0x31/0x11 + ISO-TP + FDCAN)
0x0800_5000  App slot  ~100KB (기존 앱, 시작주소 이동. 현재 66.6KB → 여유 충분)
0x0801_E000  Meta      8KB    (기존 OTA 영역 재활용: 활성 이미지 CRC·크기·버전·안티롤백 카운터·stay-in-BL 플래그)
0x0802_0000
```

## 3. 현재 자산 (이 레포 내, 재사용 가능)

- `Core/Src/uds_service.c` — 0x10/0x11/0x27 구현됨, 0x28/0x85는 **flag-only**(실차단 없음, `uds_service.c:825`), 0x31은 루틴 2개뿐(DTC-clear 0x0201, self-test 0x0202 — **CheckMemory 없음**), 0x34/0x36/0x37은 HAL 실쓰기 백엔드와 함께 구현됨
- `Core/Src/ota_flash.c` — HAL 플래시 erase/write 실구현 (싱글뱅크 인터럽트 주의 처리, 8바이트 미만 잔여 무시 = "simulator simplification" → BL에서는 바로잡아야)
- `Core/Src/iso_tp.c` — ECU 수신측 구현 (BL 재사용)
- `Core/Src/fdcan_config.c` — CAN 초기화 (BL 재사용)
- 최근 커밋: UDS 서비스 계속 추가 중(`6b47822` 0x14/0x23/0x85) — 시작 전 `git status`/log 재확인

**BL은 bare-metal 슈퍼루프로** (FreeRTOS 미포함, 크기 절약). uds_service/iso_tp가 RTOS 의존 없는지 확인 후 링크.

## 4. 동작 흐름 (목표)

```
평상시 부팅: 리셋 → BL: 메타 CRC 확인 → OK: VTOR/MSP 설정 후 앱 점프 / 불일치·무효: BL이 UDS 대기
업데이트:   테스터(또는 RPi3 게이트웨이) 0x10 02 → 앱이 stay-in-BL 플래그 설정 후 0x11 리셋
            → BL이 프로그래밍 세션 UDS 서버 오픈
            → 0x27 언락 → 0x85/0x28 → 0x34(앱 슬롯 주소) → 0x36 블록 전송 → 0x37
            → 0x31 CheckMemory(CRC32) → 메타 기록(버전 ≥ 안티롤백 카운터 강제) → 0x11 → 새 앱 부팅
브릭 시:    앱 CRC 불일치 → BL 대기 → 언제든 0x7E0으로 재플래시
```

점프 코드 본체 (참고):

```c
typedef void (*pFunc)(void);
void jump_to_app(uint32_t app_base) {
    uint32_t sp   = *(volatile uint32_t *)app_base;
    pFunc  reset = (pFunc)(*(volatile uint32_t *)(app_base + 4));
    __disable_irq();
    SysTick->CTRL = 0;            /* 사용한 페리페럴(FDCAN 등) deinit */
    SCB->VTOR = app_base;
    __set_MSP(sp);
    __set_CONTROL(0);
    reset();                      /* 복귀 없음 */
}
```

## 5. 구현 시 검증 필수 목록 (플랜 모드에서 확인)

1. **앱 링커 이동**: FLASH ORIGIN 0x08000000 → 0x08005000, LENGTH 128K → 103K + 앱 SystemInit의 VTOR 설정 확인(G4 Cube SystemInit이 VTOR를 덮어쓰는지 — 덮어쓰면 VECT_TAB_OFFSET 필요)
2. **stay-in-BL 플래그 저장 위치**: no-init RAM(리셋 생존) vs 메타 플래시 페이지 — 결정 필요 (no-init RAM이 페이지 마모 없이 깔끔)
3. **앱의 기존 0x34/0x36/0x37 처리 변경**: 앱은 더 이상 자기 플래시에 쓰면 안 됨 → 프로그래밍 세션 진입 시 플래그+리셋으로 BL에 넘기는 동작으로 교체 (설계 확정 필요)
4. **메타 페이지 구조/마모**: 업데이트당 1페이지 erase-program, 버전 카운터는 단조 증가만
5. **0x36 잔여 바이트 처리**: ota_flash.c의 8바이트 미만 무시 simplification 제거 (더블워드 패딩 처리)
6. **0x28/0x85**: 1차는 flag-only 유지 가능(세션 동안 실차단은 2차) — ceiling 주석
7. **BL 크기 실측**: 첫 빌드 후 20KB 예산 초과 시 앱 슬롯 경계 조정
8. **CRC32 다항식/범위**: 앱 이미지 전체 + 메타에 저장, 테스터(게이트웨이)와 범위 합의
9. **보안 수준**: v1 = CRC32(부패 감지)만. 서명 검증(ECDSA/HSM)은 이슈의 AIS-189 요구이나 별도 작업 — `# ponytail:` ceiling 주석으로 명시
10. **FreeRTOS 의존**: uds_service.c/iso_tp.c가 RTOS 심볼 쓰는지 grep (HAL_GetTick 수준이면 OK)

## 6. 마일스톤

- **M1** 메모리맵 재편: BL 빌드 타깃 골격(별도 ld) + 앱 0x08005000 이동 + BL→앱 점프 동작, 기존 기능 회귀(VIN 등)
- **M2** BL UDS 플래시 서버: 0x10 02/0x27/0x34/0x36/0x37로 앱 슬롯 직접 플래싱 + CRC 메타 + 부팅 게이트(불일치 시 BL 대기)
- **M3** 완성: 0x31 CheckMemory 루틴 + 안티롤백 카운터 + 0x85/0x28 세션 처리 + 0x11 재부팅 흐름
- **M4** 검증/마무리: end-to-end 데모 — PC → RPi3 게이트웨이(sdv-diag-gateway Mode B, `DIAG_TRANSPORT=can`) → CANable → G431 플래시 → 재부팅 → 새 버전 VIN/스웨버전 확인. README·worklog 갱신. 완료 후 원 이슈(sdv-diag-gateway#1)에 결과 답글

## 7. 관련 자료

- 원 이슈: https://github.com/tae9898/sdv-diag-gateway/issues/1 (완료 시 답글/클로즈)
- 게이트웨이 레포: `/var/home/playtron/work/stm/sdv-diag-gateway` — 수정 불요, 벤치로 사용 (실행법 README "Run (Mode B)" 절: `VSOMEIP_CONFIGURATION=… DIAG_TRANSPORT=can ./build/diag_service`)
- 벤치 토폴로지: PC ↔ (eth SOME/IP) ↔ RPi3(Yocto core-image-diag-gw) ↔ CAN-FD 500k/2M ↔ CANable 2.0 ↔ Nucleo-G431RB, UDS 0x7E0/0x7E8
- 이슈 원문의 8단계 시퀀스는 위 §4 흐름과 동일 (0x10 02 / 0x27 / 0x85+0x28 / 0x34 / 0x36 / 0x37 / 0x31 CheckMemory / 0x11)
