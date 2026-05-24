# Phase 2 Step 1: FreeRTOS 수동 포팅 가이드

## 개요

Phase 1까지의 코드는 **bare-metal** (운영체제 없이 직접 하드웨어 제어) 방식이었다.
`main()`의 `while(1)` 루프가 모든 처리를 담당했다.

Step 1에서는 이 `while(1)` 루프를 **FreeRTOS 태스크**로 바꾼다.
동작은 동일하지만, 이제 여러 태스크를 추가할 수 있는 기반이 생겼다.

---

## 변경 파일 요약

| 파일 | 변경 유형 | 역할 |
|------|----------|------|
| `Core/Inc/FreeRTOSConfig.h` | **신규** | FreeRTOS 설정 (CPU 클럭, 힙 크기, 인터럽트 우선순위) |
| `Makefile` | 수정 | FreeRTOS 소스 파일 + include 경로 추가 |
| `Core/Src/stm32g4xx_it.c` | 수정 | SysTick 핸들러에 FreeRTOS 틱 추가 |
| `Core/Src/main.c` | 수정 | while 루프 → 태스크 함수로 분리 |

---

## 세부 변경 내역

### 1. FreeRTOSConfig.h (신규)

FreeRTOS의 모든 동작을 결정하는 설정 파일이다.
CubeMX를 쓰면 자동 생성해주지만, 수동으로 만들면 각 설정이 왜 필요한지 이해할 수 있다.

#### 1-1. CPU 클럭 설정

```c
#define configCPU_CLOCK_HZ    170000000UL  // SYSCLK 170MHz
#define configTICK_RATE_HZ    1000U        // 1ms 틱
```

**왜 170MHz?** STM32G431RB의 최대 클럭. `main.c`의 `SystemClock_Config()`에서 PLL로 170MHz로 설정한다.

**왜 1ms 틱?** FreeRTOS의 시간 단위. `vTaskDelay(pdMS_TO_TICKS(10))`은 "10틱 대기" = 10ms 대기.
자동차/산업용에서 1ms가 표준이다.

#### 1-2. 힙 크기

```c
#define configTOTAL_HEAP_SIZE  8192U  // 8KB
```

**왜 8KB?** FreeRTOS가 태스크 생성(`xTaskCreate`), 큐 생성(`xQueueCreate`) 시 사용하는 메모리 풀이다.

RAM 사용량 계산:
```
태스크 1개 (Main):  TCB(~80B) + 스택(2048B) = ~2.1KB
Idle 태스크:        TCB(~80B) + 스택(512B)  = ~0.6KB
큐/뮤텍스 예약분:   ~1KB
여유:              ~4.3KB (Step 2에서 추가 태스크용)
```

G431RB의 RAM은 32KB. Phase 1에서 3.2KB 사용 중. 8KB 힙 할당 후에도 20KB 여유.

#### 1-3. 인터럽트 우선순위

```c
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5U
```

**이게 왜 중요하냐?** Cortex-M4에서 인터럽트 우선순위는 **숫자가 작을수록 높다** (0=최고, 15=최저).

```
우선순위 0~4:  FreeRTOS API 호출 불가 (하드실시간용, 절대 지연되면 안 되는 것)
우선순위 5~15: FreeRTOS API 호출 가능 (xQueueSendFromISR 등)
```

Phase 2 Step 2에서 CAN ISR이 `xQueueSendFromISR()`을 호출하려면
FDCAN 인터럽트 우선순위를 **6 이상**으로 설정해야 한다.

#### 1-4. 핸들러 매핑

```c
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
```

**왜 필요하냐?** FreeRTOS는 컨텍스트 스위칭(태스크 전환)을 위해 두 개의 인터럽트 핸들러를 사용한다:

| 핸들러 | 역할 |
|--------|------|
| `PendSV_Handler` | 태스크 컨텍스트 저장/복원 (스위칭) |
| `SVC_Handler` | 첫 번째 태스크 시작 시 호출 |

FreeRTOS port.c는 이 핸들러들을 `xPortPendSVHandler`, `vPortSVCHandler`라는 이름으로 정의한다.
이 매핑 매크로가 "FreeRTOS의 함수명 = STM32 벡터테이블의 함수명"으로 연결해준다.

**그래서 `stm32g4xx_it.c`에서 PendSV_Handler를 삭제한 것** — FreeRTOS port.c가 대신 정의한다.

#### 1-5. INCLUDE_* 매크로

```c
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
// ...
```

**왜 필요하냐?** FreeRTOS는 사용하지 않는 API를 컴파일에서 아예 빼서 Flash를 절약한다.
`INCLUDE_xxx = 1`이어야 해당 함수가 컴파일된다.

Step 1에서 사용한 함수들:
- `xTaskCreate` → 기본 활성화
- `vTaskStartScheduler` → 기본 활성화
- `vTaskDelay` → `INCLUDE_vTaskDelay = 1` 필요
- `xTaskGetSchedulerState` → `INCLUDE_xTaskGetSchedulerState = 1` 필요

빌드 에러 중 `undefined reference to vTaskDelay`가 났던 이유가 이 설정이 빠져서였다.

---

### 2. Makefile 수정

FreeRTOS 소스 파일을 빌드에 추가했다.

#### 2-1. 소스 파일 추가

```makefile
FREERTOS_DIR = ThirdParty/FreeRTOS-Kernel

FREERTOS_SOURCES = \
$(FREERTOS_DIR)/tasks.c          # 태스크 생성/스케줄링
$(FREERTOS_DIR)/queue.c          # 큐 (태스크 간 데이터 전달)
$(FREERTOS_DIR)/list.c           # 내부 연결 리스트
$(FREERTOS_DIR)/timers.c         # 소프트웨어 타이머 (향후 사용)
$(FREERTOS_DIR)/event_groups.c   # 이벤트 그룹 (향후 사용)
$(FREERTOS_DIR)/stream_buffer.c  # 스트림 버퍼 (향후 사용)
$(FREERTOS_DIR)/portable/GCC/ARM_CM4F/port.c   # Cortex-M4F 포트
$(FREERTOS_DIR)/portable/MemMang/heap_4.c       # 메모리 할당자
```

**각 파일이 왜 필요한가?**

| 파일 | 필수? | 이유 |
|------|-------|------|
| `tasks.c` | 필수 | `xTaskCreate`, `vTaskDelay`, `vTaskStartScheduler` |
| `queue.c` | 필수 | 태스크 내부에서 큐 사용 (tasks.c가 의존) |
| `list.c` | 필수 | tasks.c, queue.c가 내부적으로 사용 |
| `port.c` | 필수 | Cortex-M4F 전용 컨텍스트 스위칭 어셈블리 |
| `heap_4.c` | 필수 | `pvPortMalloc`/`vPortFree` (free 병합 지원) |
| `timers.c` | 선택 | 향후 소프트웨어 타이머용 |
| `event_groups.c` | 선택 | 향후 다중 이벤트 대기용 |
| `stream_buffer.c` | 선택 | 향후 바이트 스트림 전달용 |

선택 파일들은 현재 사용하지 않아도 `--gc-sections`로 링크 시 제거된다.

**왜 `heap_4.c`인가?** 5가지 메모리 관리 옵션이 있다:

| 파일 | 특징 | 용도 |
|------|------|------|
| heap_1 | malloc만, free 없음 | 가장 단순, 할당만 함 |
| heap_2 | malloc/free 있지만 병합 안 함 | 고정 크기 블록 |
| **heap_4** | **malloc/free + 인접 블록 병합** | **일반적 용도 (권장)** |
| heap_5 | heap_4 + 여러 메모리 영역 지원 | 멀티 RAM 영역 |

#### 2-2. Include 경로 추가

```makefile
-I$(FREERTOS_DIR)/include                        # FreeRTOS.h, task.h 등
-I$(FREERTOS_DIR)/portable/GCC/ARM_CM4F          # portmacro.h
```

`Core/Inc/`는 이미 포함되어 있으므로 `FreeRTOSConfig.h`는 자동으로 찾아진다.

#### 2-3. 빌드 규칙 추가

```makefile
$(BUILD_DIR)/ThirdParty/%.o: ThirdParty/%.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$(CC) -std=gnu11 $(CFLAGS) -MMD -MP -c -o $@ $<
```

`%` 패턴 규칙으로 `ThirdParty/` 아래 모든 서브디렉토리의 `.c` 파일을 컴파일한다.
`portable/GCC/ARM_CM4F/port.c`도, `portable/MemMang/heap_4.c`도 하나의 규칙으로 처리된다.

---

### 3. stm32g4xx_it.c 수정

#### 3-1. SysTick 핸들러 변경

```c
// Before (Phase 0-1)
void SysTick_Handler(void) {
    HAL_IncTick();
}

// After (Phase 2)
void SysTick_Handler(void) {
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
```

**무슨 일이 일어나는가?**

```
SysTick (1ms마다 하드웨어 자동 발생)
  → SysTick_Handler() 호출
    → HAL_IncTick()         // HAL_Delay()용 카운터 증가 (항상)
    → xPortSysTickHandler() // FreeRTOS 틱 처리 (스케줄러 시작 후에만)
      → 태스크 delay 만료 확인
      → 필요하면 컨텍스트 스위치 트리거 (PendSV 발생)
```

**왜 `if` 조건이 필요한가?** `main()` 초기화 중(스케줄러 시작 전)에는 FreeRTOS가 아직 준비되지 않았다.
이때 `xPortSysTickHandler()`를 호출하면 크래시 발생.

**왜 `HAL_IncTick()`을 먼저 호출하는가?** 순서가 중요하다.
`xPortSysTickHandler()` 안에서 컨텍스트 스위치가 발생할 수 있는데,
HAL tick은 모든 태스크가 공유하므로 스위치 전에 먼저 증가시켜야 한다.

#### 3-2. PendSV_Handler 삭제

```c
// 삭제됨 (FreeRTOS port.c가 대신 정의)
void PendSV_Handler(void) { ... }
```

FreeRTOS가 컨텍스트 스위칭을 위해 PendSV를 사용한다.
`FreeRTOSConfig.h`의 매핑 매크로로 port.c의 `xPortPendSVHandler`가 `PendSV_Handler`가 된다.
여기에 정의하면 **중복 정의** 에러가 발생한다.

---

### 4. main.c 수정

#### 4-1. while 루프 → 태스크

```c
// Before: bare-metal 루프
int main(void) {
    // ... 초기화 ...
    while (1) {
        OBD2_UpdateSimValues(&g_sim_state);
        HAL_Delay(10);  // 바쁜 대기 (CPU 점유)
    }
}

// After: FreeRTOS 태스크
int main(void) {
    // ... 초기화 (동일) ...
    xTaskCreate(vMainTask, "Main", 512, NULL, 2, NULL);
    vTaskStartScheduler();
    while (1);  // 도달하면 안 됨
}

static void vMainTask(void *pvParameters) {
    while (1) {
        OBD2_UpdateSimValues(&g_sim_state);
        vTaskDelay(pdMS_TO_TICKS(10));  // CPU 양보
    }
}
```

**무엇이 달라졌는가?**

| 항목 | bare-metal | FreeRTOS |
|------|-----------|----------|
| 대기 방식 | `HAL_Delay(10)` — CPU가 카운터 폴링 | `vTaskDelay(ticks)` — CPU를 다른 태스크에 양보 |
| 다른 태스크 추가 | 불가 (루프 하나) | `xTaskCreate`로 계속 추가 가능 |
| 인터럽트와의 관계 | ISR이 main을 선점 | ISR → 스케줄러 → 우선순위 높은 태스크 먼저 |

#### 4-2. xTaskCreate 파라미터

```c
xTaskCreate(vMainTask, "Main", 512, NULL, 2, NULL);
//             함수      이름   스택   파람 우선순위 핸들
```

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| 함수 | `vMainTask` | 태스크로 실행할 함수 |
| 이름 | `"Main"` | 디버깅용 (최대 8자) |
| 스택 | `512` | **단위: word** (512 * 4 = 2048 바이트) |
| 파라미터 | `NULL` | 태스크에 전달할 인자 (없음) |
| 우선순위 | `2` | 0(최저)~4(최고). Idle은 0. Main은 2 |
| 핸들 | `NULL` | 태스크 핸들 (나중에 삭제/일시정지할 때 필요) |

**왜 스택이 2048바이트인가?** 이 태스크 안에서:
- `OBD2_UpdateSimValues()` — 지역 변수 몇 개 (~100B)
- `ISO_TP_Tick()` / `DiagSession_Tick()` — 가벼운 함수 (~200B)
- `Debug_Print()` — 256바이트 버퍼 사용
- 안전 여유 — 스택 오버플로우 방지

`uxTaskGetStackHighWaterMark()`로 실제 사용량을 나중에 측정할 수 있다.

#### 4-3. HAL_Delay vs vTaskDelay

```c
// HAL_Delay(10)이 하는 일:
// 1. 현재 tick 저장
// 2. while 루프로 tick이 10 증가할 때까지 대기
// 3. CPU 100% 점유 (다른 작업 불가)

// vTaskDelay(pdMS_TO_TICKS(10))이 하는 일:
// 1. 태스크를 "지연 큐"에 넣음
// 2. CPU를 즉시 다른 태스크(또는 Idle)에 양보
// 3. 10틱 후 스케줄러가 이 태스크를 다시 실행 가능 상태로 변경
```

Step 1에서는 태스크가 하나뿐이라 체감 차이가 없다.
Step 2에서 CAN-Rx 태스크를 추가하면, Main 태스크가 `vTaskDelay`로 CPU를 양보하는 동안
CAN-Rx 태스크가 메시지를 처리할 수 있다.

#### 4-4. 훅 함수

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // 스택이 부족하면 여기서 멈춤 → 스택 크기 늘려야 함
}

void vApplicationMallocFailedHook(void) {
    // 힙이 부족하면 여기서 멈춤 → configTOTAL_HEAP_SIZE 늘려야 함
}
```

**왜 필요한가?** 임베디드에서는 에러가 조용히 넘어가면 위험하다.
이 훅들이 디버그 출력으로 어떤 태스크에서 문제가 발생했는지 알려준다.

---

## 빌드 결과

```
   text    data     bss     dec     hex
  33384     116   12476   45976    b398
```

| 항목 | 크기 | Flash/RAM | 사용률 |
|------|------|-----------|--------|
| Flash (text+data) | 33.5KB | 128KB | 26% |
| RAM (data+bss) | 12.6KB | 32KB | 39% |

Phase 1 대비 증가분:
- Flash: +1.3KB (FreeRTOS 커널)
- RAM: +9.4KB (힙 8KB + 커널 데이터 + 태스크 스택)

---

## 다음 Step에서 추가할 것 (Step 2)

```
현재 (Step 1):                    Step 2:
┌──────────────┐                  ┌──────────────┐
│  Main Task   │                  │  Main Task   │
│  (sim+LED)   │                  │  (sim+LED)   │
└──────────────┘                  ├──────────────┤
                                  │ CAN-Rx Task  │  ← 신규
                                  │ (ISO-TP+UDS) │
                                  └──────────────┘

ISR → 직접 ISO-TP 호출             ISR → Queue → CAN-Rx Task
```

Step 2에서 핵심 변경: FDCAN ISR이 더 이상 ISO-TP/UDS를 직접 호출하지 않고
Queue에 데이터만 넣는다. CAN-Rx 태스크가 Queue에서 꺼내서 처리한다.
이렇게 하면 ISR 실행 시간이 짧아져 실시간성이 보장된다.
