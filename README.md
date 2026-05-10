# Phase 0: OBD-II ECU 시뮬레이터

STM32G431RB Nucleo 보드 기반 OBD-II ECU 시뮬레이터 펌웨어. Classic CAN 500kbps로 PC에서 OBD-II 요청 시 시뮬레이션된 차량 데이터로 응답한다.

## 프로젝트 구조

```
phase0-obd-simulator/
├── Core/
│   ├── Inc/                    # 헤더 파일
│   │   ├── main.h              # 핀 정의, 클럭 상수
│   │   ├── obd2_simulator.h    # OBD-II PID 정의, 시뮬레이션 상태 구조체
│   │   ├── fdcan_config.h      # FDCAN 초기화 프로토타입
│   │   ├── uart_debug.h        # 디버그 UART 프로토타입
│   │   └── stm32g4xx_hal_conf.h # HAL 모듈 활성화 설정
│   └── Src/                    # 소스 파일
│       ├── main.c              # 메인 루프, 클럭 설정, GPIO 초기화
│       ├── obd2_simulator.c    # OBD-II 요청 파싱, PID 응답 생성
│       ├── fdcan_config.c      # FDCAN1 Classic CAN 500kbps, 필터, 인터럽트
│       ├── uart_debug.c        # USART2 printf retarget
│       ├── stm32g4xx_hal_msp.c # HAL MSP (클럭, GPIO AF, NVIC)
│       ├── stm32g4xx_it.c      # 인터럽트 핸들러
│       └── system_stm32g4xx.c  # SystemInit, SystemCoreClockUpdate
├── docs/
│   └── CUBE_SETUP.md           # STM32CubeMX 설정 가이드
├── tests/
│   ├── socketcan_test.sh       # Bash CAN 테스트 (can-utils)
│   └── obd2_test.py            # Python CAN 테스트 (python-can)
├── Drivers/                    # HAL 드라이버 (별도 clone 필요)
├── Makefile                    # arm-none-eabi-gcc 빌드
├── STM32G431RBTX_FLASH.ld      # 링커 스크립트 (128KB Flash, 32KB RAM)
└── startup_stm32g431xx.s       # Cortex-M4 startup 어셈블리
```

## 하드웨어 요구사항

| 부품 | 용도 |
|------|------|
| STM32 Nucleo-G431RB | 메인 MCU 보드 |
| MCP2562FD | CAN-FD 트랜시버 |
| 120Ω 저항 x2 | CAN 버스 종단 |
| USB-CAN 어댑터 | PC↔CAN 연결 (테스트용) |

### 핀 할당

| 핀 | 기능 | 연결 |
|----|------|------|
| PA11 | FDCAN1_RX | MCP2562FD RXD |
| PA12 | FDCAN1_TX | MCP2562FD TXD |
| PA2 | USART2_TX | ST-LINK VCP (디버그) |
| PA3 | USART2_RX | ST-LINK VCP (디버그) |
| PA5 | GPIO_Output | LD4 LED |

## 빌드 방법

### 1. HAL 드라이버 확보

```bash
# HAL 드라이버
git clone --depth 1 https://github.com/STMicroelectronics/stm32g4xx_hal_driver.git Drivers/STM32G4xx_HAL_Driver

# CMSIS 디바이스 헤더
mkdir -p Drivers/CMSIS/Device/ST
git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_g4.git Drivers/CMSIS/Device/ST/STM32G4xx
```

### 2. 툴체인 설치

```bash
sudo apt install gcc-arm-none-eabi stlink-tools
```

### 3. 빌드

```bash
make -j$(nproc)
```

### 4. 플래시

```bash
make flash
```

### 5. 디버그 출력 확인

```bash
# 시리얼 포트 연결 (115200 baud)
screen /dev/ttyACM0 115200

# 또는 소프트웨어 리셋 후 cat으로 확인
st-flash reset && cat /dev/ttyACM0
```

## 지원 OBD-II PID

| PID | 데이터 | 시뮬레이션 |
|-----|--------|-----------|
| 0x00 | 지원 PID 목록 | 비트맵 응답 |
| 0x05 | 냉각수 온도 | 80~105°C 서서히 변화 |
| 0x0C | 엔진 RPM | 800~4000 RPM 램프 |
| 0x0D | 차속 | 0~120 km/h 서서히 변화 |

## SocketCAN 테스트

```bash
# CAN 인터페이스 설정
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# 차속 요청
cansend can0 7E0#02010D0000000000

# 응답 확인
candump can0,7E8:7FF -t a -n 1

# Python 테스트
python3 tests/obd2_test.py can0
```

## 빌드 결과

```
   text    data     bss     dec     hex
  23528     112    2720   26360    66f8  build/phase0-obd-simulator.elf
```

- Flash 사용: 23.5KB / 128KB (18%)
- RAM 사용: 2.8KB / 32KB (9%)

## 개발 환경

- **방식:** Bare-metal (CubeMX 없이 HAL 직접 작성)
- **툴체인:** arm-none-eabi-gcc 13.2.1
- **MCU:** STM32G431RB (Cortex-M4 @ 170MHz, 128KB Flash, 32KB RAM)
- **프로토콜:** Classic CAN 500kbps, OBD-II (SAE J1979)
