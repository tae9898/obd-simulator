# STM32 OBD/UDS ECU Simulator + FreeRTOS Gateway

STM32G431RB firmware for the SDV diagnostic gateway project. Acts as the
**ECU node**: responds to OBD-II / UDS diagnostics over CAN-FD, and bridges
CAN ↔ RS485 internally under FreeRTOS.

> Phases 0–2 of the project. Tested end-to-end against the RPi3 gateway
> (see the top-level project README).

## What it does

- **OBD-II simulator** (SAE J1979): Mode 01 PIDs — RPM (0x0C), speed (0x0D),
  coolant temp (0x05), supported-PID bitmap (0x00) — with simulated values.
- **UDS server** (ISO 14229): DiagnosticSessionControl (0x10), ECUReset (0x11),
  ReadDataByIdentifier (0x22), SecurityAccess (0x27), RoutineControl (0x31),
  with NRC handling.
- **ISO-TP** (ISO 15765-2): single/multi-frame assembly, flow control.
- **CAN-FD** (BRS): 500 kbps arbitration / 2 Mbps data.
- **FreeRTOS gateway**: separate tasks for CAN-Rx, RS485, debug UART;
  CAN↔RS485 bidirectional message routing; queues + mutex.
- **Safety**: IWDG watchdog, error handler (CAN bus-off, UART framing → safe state).

## Project structure

```
obd-simulator/
├── Core/
│   ├── Inc/   main.h, obd2_simulator.h, fdcan_config.h, diag_session.h,
│   │          uds_service.h, iso_tp.h, rs485.h, uart_debug.h, ...
│   └── Src/   main.c (FreeRTOS tasks), fdcan_config.c, obd2_simulator.c,
│              iso_tp.c, diag_session.c, uds_service.c, rs485.c, uart_debug.c
├── Middlewares/FreeRTOS/
├── docs/                      # per-phase guides (CAN-FD, ISO-TP, UDS, RTOS, routing, IWDG, MISRA)
├── tests/                     # Python (UDS/ISO-TP/OBD-II) + SocketCAN tests
├── Makefile                   # arm-none-eabi-gcc
├── STM32G431RBTX_FLASH.ld     # 128KB Flash / 32KB RAM linker script
└── startup_stm32g431xx.s
```

## Hardware

| Part | Use |
|------|-----|
| STM32 Nucleo-G431RB | ECU (Cortex-M4 @ 170 MHz, 128 KB Flash, 32 KB RAM) |
| MCP2562FD | CAN-FD transceiver (PA11 RX, PA12 TX) |
| MAX485 | RS485 transceiver (PA9 TX, PA10 RX, PA8 DE/RE) |
| 120 Ω × 2 | CAN / RS485 termination |

| Pin | Function |
|-----|----------|
| PA11 / PA12 | FDCAN1_RX / TX |
| PA9 / PA10 | USART1 TX/RX (RS485) |
| PA8 | MAX485 DE/RE direction |
| PA2 / PA3 | USART2 TX/RX — ST-LINK debug VCP |
| PA5 | LD4 LED |

## Build & flash

```bash
# toolchain
sudo apt install gcc-arm-none-eabi stlink-tools   # (or rpm-ostree / distrobox equivalent)

# HAL drivers (first time)
git clone --depth 1 https://github.com/STMicroelectronics/stm32g4xx_hal_driver.git Drivers/STM32G4xx_HAL_Driver
mkdir -p Drivers/CMSIS/Device/ST
git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_g4.git Drivers/CMSIS/Device/ST/STM32G4xx

# build & flash
make -j$(nproc)
make flash                 # via ST-LINK

# debug console (115200 8N1, ST-LINK VCP)
screen /dev/ttyACM0 115200
```

## CAN interface

- **Mode**: CAN-FD with BRS — arbitration 500 kbps, data 2 Mbps
- **Request ID**: 0x7E0 · **Response ID**: 0x7E8 (OBD-II/UDS diagnostic)
- **Filter**: accepts standard frames 0x000–0x7FF on RX FIFO0
- Firmware is passive: waits for requests, sends responses (no periodic TX).

## RS485

- USART1 @ 115200 8N1, half-duplex (DE/RE on PA8)
- Frame format: `[ID_H][ID_L][DLC][DATA 0..N]`
- Bidirectional CAN↔RS485 routing in the FreeRTOS gateway task

## Test (against RPi3 gateway)

With the STM32 connected to the RPi3 gateway over CAN-FD, the PC tester
queries it through DoIP (see top-level README `tester/doip_tester.py`):
```bash
python3 tester/doip_tester.py <RPi_IP> 0x0C    # → RPM value (round-trip via DoIP↔CAN)
```

Direct SocketCAN test (CAN bus only):
```bash
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on && sudo ip link set can0 up
cansend can0 7E0#02010C0000000000              # RPM request
candump can0                                    # → 7E8 response
```

## Build size (final, Phase 2)

| Resource | Used | Limit | % |
|----------|------|-------|---|
| Flash | ~46.8 KB | 128 KB | 37% |
| RAM | ~12.5 KB | 32 KB | 39% |

## Development notes

- Bare-metal HAL (no CubeMX code generation); FreeRTOS added by hand.
- Toolchain: arm-none-eabi-gcc 13.x.
- Static analysis: cppcheck MISRA-C:2012 spot checks (e.g., Rule 10.4 explicit casts).
