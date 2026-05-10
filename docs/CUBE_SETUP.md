# STM32CubeMX FDCAN1 Classic CAN 설정 가이드

> 대상 보드: STM32G431RB Nucleo (Nucleo-G431RB)
> 목적: FDCAN1을 Classic CAN 500kbps로 설정하여 OBD-II 통신 구현

---

## 1. CubeMX 프로젝트 생성

1. STM32CubeMX 실행
2. **ACCESS TO MCU SELECTOR** 클릭
3. MCU 선택: **STM32G431RBTx** 검색 후 선택 (Nucleo-G431RB)
4. **Start Project** 클릭
5. 상단 메뉴 **Project** -> **Settings**
   - **Project Name**: `phase0-obd-simulator`
   - **Toolchain / IDE**: `Makefile`
   - **Code Generator** 탭에서 "Generate peripheral initialization as a pair of '.c/.h' files..." 체크

---

## 2. 클럭 설정

**Clock Configuration** 탭으로 이동합니다.

### HSE 설정

| 항목 | 설정값 |
|------|--------|
| HSE | Crystal/Ceramic Resonator |
| HSE Frequency | 8 MHz (Nucleo 보드 온보드 크리스탈) |

### PLL 설정 (HSE -> SYSCLK 170MHz)

| 항목 | 설정값 | 계산 |
|------|--------|------|
| PLL Source Mux | HSE | - |
| PLLM | 1 | 8MHz / 1 = 8MHz (PLL 입력) |
| PLLN | 85 | 8MHz * 85 = 680MHz (VCO) |
| PLLP | 7 | 680MHz / 7 = ~97.1MHz (PLL1 main) |
| PLLQ | 8 | 680MHz / 8 = 85MHz (PLL1 Q) |

### 버스 클럭 분주비

| 항목 | 설정값 | 결과 클럭 |
|------|--------|-----------|
| System Clock Mux | PLL1CLK | 170MHz |
| AHB Prescaler | 1 | HCLK = 170MHz |
| APB1 Prescaler | 4 | PCLK1 = 42.5MHz |
| APB2 Prescaler | 4 | PCLK2 = 42.5MHz |

> **참고**: SYSCLK 170MHz는 STM32G431의 최대 동작 주파수입니다. PLL1CLK는 내부적으로 VCO 출력을 통해 170MHz를 생성합니다.

---

## 3. FDCAN1 설정

**Pinout & Configuration** 탭 -> 좌측 **FDCAN1** 활성화합니다.

### 모드 설정 (FDCAN Configuration 탭 - Bit Timings Parameters)

| 항목 | 설정값 | 설명 |
|------|--------|------|
| FDCAN Operation Mode | **Normal** | 일반 통신 모드 |
| Frame Format | **Classic CAN** | CAN 2.0A (11-bit ID), FDCAN 미사용 |
| Clock Prescaler | **1** | FDCAN 입력 클럭 분주 없음 |
| Sync Jump Width | **1 Time Quanta** | 동기화 보정 범위 |
| Time Segment 1 | **13 Time Quanta** | Propagation + Phase Segment 1 |
| Time Segment 2 | **2 Time Quanta** | Phase Segment 2 |
| ReSync Jump Width | **1** | - |

### 비트레이트 계산

```
bitrate = FDCAN_CLK / (Prescaler * (Sync + Seg1 + Seg2))
        = 8 MHz / (1 * (1 + 13 + 2))
        = 8 MHz / 16
        = 500 kbps
```

> **중요**: FDCAN1의 클럭 소스는 HSE(8MHz)를 사용합니다. 이는 PLL을 거치지 않은 직접 클럭이므로, 위 계산이 유효합니다.

### FDCAN 클럭 소스 설정

**Clock Configuration** 탭에서 FDCAN1 클럭 소스를 확인합니다:

| 항목 | 설정값 |
|------|--------|
| FDCAN Clock Source | **HSE** (8MHz) |

> CubeMX에서는 **RCC** 설정의 FDCAN1 clock source mux를 HSE로 선택해야 합니다.

### TX/RX FIFO 설정 (FDCAN Configuration 탭)

#### TX Settings

| 항목 | 설정값 |
|------|--------|
| Tx FIFO Queue Mode | **FIFO** |
| Tx FIFO Priority | None |

#### RX FIFO 0

| 항목 | 설정값 |
|------|--------|
| Activation | **Enabled** |
| FIFO Operation Mode | **FIFO** |
| FIFO Data Size | 8 bytes |
| FIFO Element Size | 8 bytes |

#### RX FIFO 1

| 항목 | 설정값 |
|------|--------|
| Activation | **Disabled** |

---

## 4. FDCAN1 수신 필터 설정

**Pinout & Configuration** 탭 -> FDCAN1 -> **FDCAN Filter Configuration** 탭

### Filter Index 0 설정

| 항목 | 설정값 | 설명 |
|------|--------|------|
| Filter Index | **0** | 첫 번째 필터 |
| Filter Type | **Range (DUAL)** | ID 범위 필터링 |
| ID1 (SFID1/EFID1) | **0x7E0** | OBD-II 요청 ID (ECU #1) |
| ID2 (SFID2/EFID2) | **0x7E0** | 동일 ID (단일 ID 필터) |
| Target FIFO | **RX FIFO 0** | 일치하는 프레임을 FIFO 0으로 전달 |
| Filter Type | **Standard** | 11-bit Standard ID |

> **참고**: 0x7E0은 OBD-II 표준 요청 ID입니다. 추가 ECU ID(0x7E1~0x7E7)를 수신하려면 별도 필터를 추가하거나 Range를 확장하면 됩니다.

---

## 5. 핀 설정

**Pinout & Configuration** 탭에서 다음 핀을 설정합니다.

### 핀 할당 표

| 핀 | 기능 | 설정값 | 설명 |
|----|------|--------|------|
| **PA11** | FDCAN1_RX | FDCAN1_RX | CAN 수신 (CubeMX 자동 할당) |
| **PA12** | FDCAN1_TX | FDCAN1_TX | CAN 송신 (CubeMX 자동 할당) |
| **PA2** | USART2_TX | USART2_TX | ST-LINK VCP 송신 (디버그 출력) |
| **PA3** | USART2_RX | USART2_RX | ST-LINK VCP 수신 (디버그 입력) |
| **PA5** | GPIO_Output | LD4 | Nucleo 보드 내장 LED |

### PA5 (LD4) GPIO 설정

| 항목 | 설정값 |
|------|--------|
| GPIO mode | **Output Push Pull** |
| GPIO Pull-up/Pull-down | **No pull-up and no pull-down** |
| Maximum output speed | **Low** |
| User Label | **LD4** |

> PA5는 Nucleo-G431RB 보드의 녹색 LED(LD4)에 직접 연결되어 있습니다. CAN 통신 상태 표시 등에 사용합니다.

---

## 6. USART2 설정

**Pinout & Configuration** 탭 -> 좌측 **USART2** 활성화합니다.

### Mode 설정

| 항목 | 설정값 |
|------|--------|
| Mode | **Asynchronous** |

### Parameter Settings

| 항목 | 설정값 |
|------|--------|
| Baud Rate | **115200** |
| Word Length | **8 Bits** |
| Stop Bits | **1** |
| Parity | **None** |
| Data Direction | **Receive and Transmit** |
| Over Sampling | **16 Samples** |

> USART2는 ST-LINK VCP를 통해 PC와 연결되므로, USB 직렬 터미널로 디버그 메시지를 확인할 수 있습니다.

---

## 7. NVIC 설정

**Pinout & Configuration** 탭 -> **System Core** -> **NVIC**

### 인터럽트 활성화

| 항목 | 설정값 | 설명 |
|------|--------|------|
| FDCAN1 IT 0 | **Enabled** | RX FIFO 0 새 메시지 수신 인터럽트 |
| USART2 global interrupt | **Enabled** | 디버그용 수신 인터럽트 (선택사항) |

### Preemption Priority 설정

| 항목 | Preemption | Sub Priority |
|------|------------|--------------|
| FDCAN1 IT 0 | 0 | 0 |
| USART2 global interrupt | 1 | 0 |

> FDCAN1 인터럽트를 최우선으로 설정하여 CAN 메시지 손실을 방지합니다.

---

## 8. MCP2562FD 트랜시버 배선도

STM32G431RB의 FDCAN1은 CAN 트랜시버(CAN Transceiver) 없이 CAN 버스에 직접 연결할 수 없습니다. **MCP2562FD** (Microchip) 트랜시버를 사용하여 FDCAN1 TX/RX 신호를 CAN_H/CAN_L differential 신호로 변환합니다.

### 배선도

```
G431RB Nucleo         MCP2562FD              CAN 버스
+--------------+     +--------------+
|              |     |              |
| PA12 (TX) ---+----- TXD           |
| PA11 (RX) ---+----- RXD           |
|              |     |              |
| 3.3V --------+----- VIO           |  (I/O 로직 전압)
| 5V   --------+------ VCC          |
| GND  --------+------ GND          |
|              |     |              |
|              |     | S (Standby)--+---- GND (Normal 모드)
|              |     |              |
|              |     | CANH ---------+---- CAN_H ---- USB-CAN 어댑터
|              |     | CANL ---------+---- CAN_L ---- (120 Ohm 종단)
|              |     |              |
+--------------+     +--------------+
```

### MCP2562FD 핀 설명

| 핀 | 연결 | 설명 |
|----|------|------|
| **TXD** | PA12 (FDCAN1_TX) | MCU -> 트랜시버 송신 데이터 |
| **RXD** | PA11 (FDCAN1_RX) | 트랜시버 -> MCU 수신 데이터 |
| **VCC** | 5V | 전원 (4.5V ~ 5.5V) |
| **VIO** | 3.3V | I/O 로직 전압 (3.3V 호환) |
| **GND** | GND | 공통 접지 |
| **S** | GND | Standby 제어 (LOW = Normal, HIGH = Standby) |
| **CANH** | CAN_H | CAN 버스 하이 라인 |
| **CANL** | CAN_L | CAN 버스 로우 라인 |

### 주의사항

- **S 핀**: GND에 연결하면 **Normal 모드** (통신 가능), VIO에 연결하면 **Standby 모드** (버스에서 분리)
- **VIO = 3.3V**: MCP2562FD는 3.3V VIO를 지원하므로 STM32G431RB(3.3V 로직)와 **레벨 변환 없이 직접 연결** 가능
- **120 Ohm 종단 저항**: CAN_H와 CAN_L 양단에 120 Ohm 저항이 필수입니다. 버스 양 끝에 각각 배치합니다. USB-CAN 어댑터 내부에 이미 종단 저항이 있는 경우 중복되지 않도록 확인하세요.
- **디커플링 캐패시터**: VCC와 GND 사이에 100nF 세라믹 캐패시터를 가까이 배치하세요.
- **5V 전원**: Nucleo 보드의 5V 핀(USB 전원)을 사용하면 별도 전원 공급이 필요 없습니다.

---

## 9. 프로젝트 코드 생성

1. **Project** -> **Generate Code** 클릭 (또는 Ctrl+Shift+G)
2. 생성 완료 후 지정한 폴더에 다음 구조가 생성됩니다:

```
phase0-obd-simulator/
+-- Core/
|   +-- Inc/
|   |   +-- main.h
|   |   +-- fdcan.h
|   |   +-- usart.h
|   |   +-- gpio.h
|   |   +-- stm32g4xx_hal_conf.h
|   |   +-- stm32g4xx_it.h
|   +-- Src/
|       +-- main.c
|       +-- fdcan.c
|       +-- usart.c
|       +-- gpio.c
|       +-- stm32g4xx_it.c
|       +-- stm32g4xx_hal_msp.c
|       +-- system_stm32g4xx.c
+-- Drivers/
+-- Makefile
+-- startup_stm32g431rbtx.s
```

3. `Core/Src`와 `Core/Inc`에 작성한 펌웨어 코드를 추가합니다.

---

## 10. 빌드 및 플래시

### 빌드

```bash
cd phase0-obd-simulator
make -j$(nproc)
```

### 플래시 (ST-LINK 사용)

```bash
# st-flash 도구 설치 (Ubuntu/Debian)
sudo apt install stlink-tools

# 바이너리 플래시
st-flash write build/phase0-obd-simulator.bin 0x08000000
```

### 플래시 (OpenOCD 사용)

```bash
# OpenOCD로 플래시
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "program build/phase0-obd-simulator.elf verify reset exit"
```

### 디버그 터미널 확인

```bash
# ST-LINK VCP (USART2) 터미널 연결
# /dev/ttyACM0 또는 /dev/ttyUSB0
minicom -D /dev/ttyACM0 -b 115200
```

---

## 요약 설정 체크리스트

- [ ] MCU: STM32G431RBTx
- [ ] HSE: 8MHz 크리스탈
- [ ] SYSCLK: 170MHz (PLL)
- [ ] FDCAN1: Classic CAN, 500kbps
- [ ] FDCAN1 Clock Source: HSE (8MHz)
- [ ] FDCAN1 Filter: ID 0x7E0 -> RX FIFO 0
- [ ] PA11: FDCAN1_RX, PA12: FDCAN1_TX
- [ ] PA2/PA3: USART2 (디버그)
- [ ] PA5: LD4 LED
- [ ] MCP2562FD: S=GND (Normal 모드), VIO=3.3V
- [ ] CAN 버스 120 Ohm 종단 저항 확인
- [ ] NVIC: FDCAN1 IT 0 활성화
