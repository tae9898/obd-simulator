# STM32 OBD/UDS ECU Simulator + FreeRTOS Gateway

[![CI](https://github.com/tae9898/obd-simulator/actions/workflows/ci.yml/badge.svg)](https://github.com/tae9898/obd-simulator/actions/workflows/ci.yml)

STM32G431RB firmware for the SDV diagnostic gateway project. Acts as the
**ECU node**: responds to OBD-II / UDS diagnostics over CAN-FD, and bridges
CAN ↔ RS485 internally under FreeRTOS.

> Application + UDS flash bootloader (FBL), hardware-validated on the bench:
> 95 UDS / 30 FBL / 22 OBD-II SocketCAN checks green, plus host unit tests
> (no-HW) for the DTC engine.

## What it does

- **OBD-II simulator** (SAE J1979 / ISO 15031-5): Mode 01 PIDs — RPM (0x0C),
  speed (0x0D), coolant temp (0x05), supported-PID bitmap (0x00) — plus Mode 03
  (stored DTC), 04 (clear DTC), 07 (pending DTC), 09 (vehicle info / VIN) — with
  simulated values.
- **UDS server** (ISO 14229): 15 services in the app — 0x10 session control
  (P2/P2* advertised), 0x11 reset, 0x14 clear DTC, 0x19 DTC information
  (20 subfunctions: status bits, snapshot, extended data, mirror memory),
  0x22/0x23 read DID / read memory, 0x27 security access, 0x28/0x85
  communication & DTC-setting control, 0x2A periodic read + 0x2C dynamic DID,
  0x2E write DID, 0x2F IO control, 0x31 routines, 0x3E tester present — plus
  0x34/0x36/0x37 served by the bootloader (below). NRCs per ISO 14229-1:
  0x7F wrong-session vs 0x33 locked split, 0x78 responsePending before the
  slot erase.
- **UDS flash bootloader** (FBL): 28 KB bare-metal BL at 0x08000000 owns
  reprogramming with a CRC32 boot gate, anti-rollback counter and a
  brick-safe update marker — details in the FBL section below.
- **ISO-TP** (ISO 15765-2): single/multi-frame assembly, flow control; RX/TX
  state machines split for full-duplex (§9.8.3).
- **CAN-FD** (BRS): 500 kbps arbitration / 2 Mbps data.
- **CAN addressing** (ISO 15765-2 / 14229-1): physical (0x7E0↔0x7E8) and
  functional (0x7DF broadcast) — functional requests map to this node's physical
  response ID; UDS services suppress responses on functional requests.
- **FreeRTOS gateway**: separate tasks for CAN-Rx, RS485, debug UART;
  CAN↔RS485 bidirectional message routing; queues + mutex.
- **Safety**: IWDG refreshed only when all tasks (main / CAN-Rx / RS485) set
  their alive bits — any stalled task resets the board within ~2 s. The
  watchdog keeps running in the bootloader (only POR stops it), so the BL
  refreshes it in its superloop and between flash-erase pages. CAN bus-off
  recovery runs in task context (flag from ISR), and the error handler drops
  to a safe state on CAN/UART faults.

## Project structure

```
obd-simulator/
├── Core/
│   ├── Inc/   main.h, obd2_simulator.h, fdcan_config.h, can_addressing.h,
│   │          diag_session.h, uds_service.h, iso_tp.h, rs485.h, uart_debug.h, ...
│   └── Src/   main.c (FreeRTOS tasks), fdcan_config.c, obd2_simulator.c,
│              iso_tp.c, diag_session.c, uds_service.c, rs485.c, uart_debug.c,
│              bl_main.c / bl_boot.c / bl_uds.c / ota_flash.c (bootloader, -DBL_BUILD),
│              fdcan_loopback_test.c (diagnostic, gated by RUN_FDCAN_LOOPBACK_TEST)
├── Middlewares/FreeRTOS-Kernel/
├── docs/                      # setup notes, development history (docs/history/), blog draft
├── tests/                     # Python bench suites (UDS / FBL / OBD-II, SocketCAN)
│                              # + host unit tests (host_dtc_test.c — no hardware)
├── Makefile                   # arm-none-eabi-gcc
├── MCP2562FD_PINOUT.md        # CAN-FD transceiver pinout
├── STM32G431RBTX_FLASH.ld     # 128KB Flash / 32KB RAM linker script
└── startup_stm32g431xx.s
```

## Hardware

```
 ┌─────────────┐      CAN-FD (BRS 500k/2M)       ┌──────────────────────────┐
 │   Tester    │     0x7E0↔0x7E8 · 0x7DF func     │       STM32G431RB        │
 │  PC / RPi   │ ◄─────────────────────────────► │        (ECU node)        │
 │ SocketCAN / │      via MCP2562FD (PA11/PA12)   │                          │
 │   DoIP gw   │                                  │  FDCAN1 → ISO-TP → UDS   │
 └─────────────┘                                  │          │               │
                                                  │   FreeRTOS gateway       │
 ┌─────────────┐   SWD (flash/debug) +            │          │               │
 │  ST-LINK    │   UART VCP (PA2/PA3, 115200)     │          ▼               │
 │   VCP →     │ ◄─────────────────────────────► │  USART1 (PA9/PA10) →     │
 │ /dev/ttyACM0│                                  │  MAX485 (DE/RE=PA8) ─────┼──► RS485 bus
 └─────────────┘                                  └──────────────────────────┘
```

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
sudo apt install gcc-arm-none-eabi stlink-tools   # Fedora: dnf install arm-none-eabi-gcc stlink (Atomic: rpm-ostree)

# HAL drivers (first time)
git clone --depth 1 https://github.com/STMicroelectronics/stm32g4xx_hal_driver.git Drivers/STM32G4xx_HAL_Driver
mkdir -p Drivers/CMSIS/Device/ST
git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_g4.git Drivers/CMSIS/Device/ST/STM32G4xx

# build & flash
make -j$(nproc)              # application
make bl                      # bootloader (28 KB slot at 0x08000000)
make flash                   # mass erase + BL + app via ST-LINK

# debug console (115200 8N1, ST-LINK VCP)
screen /dev/ttyACM0 115200
```

## CAN interface

- **Mode**: CAN-FD with BRS — arbitration 500 kbps, data 2 Mbps
- **Physical**: request 0x7E0 · response 0x7E8 (1:1 OBD-II/UDS diagnostic)
- **Functional**: request 0x7DF (broadcast) → this node responds on its physical
  response ID 0x7E8 (not 0x7E7); UDS services suppress functional responses.
- **Filter**: accepts standard frames 0x000–0x7FF on RX FIFO0
- Firmware is passive: waits for requests, sends responses (no periodic TX).

## RS485

- USART1 @ 115200 8N1, half-duplex (DE/RE on PA8)
- Frame format: `[ID_H][ID_L][DLC][DATA 0..N]` — RX assembly completes on
  3 + DLC bytes (variable length), DLC > 64 resyncs
- Bidirectional CAN↔RS485 routing in the FreeRTOS gateway task

## Flash bootloader (FBL)

128 KB flash cannot hold A/B partitions (app ≈ 66 KB ×2 + BL + meta), so a
classic FBL layout is used (`Core/Inc/bl_handoff.h`):

| Region | Range | Size | Contents |
|--------|-------|------|----------|
| BL | 0x08000000 – 0x08006FFF | 28 KB | bare-metal UDS flash server (`make bl`) |
| App | 0x08007000 – 0x0801DFFF | 92 KB | FreeRTOS application |
| Meta | 0x0801E000 – 0x0801FFFF | 8 KB | magic / state / version / size / CRC32 |

- **Boot gate**: meta VALID + CRC32 match → jump to app; meta absent with
  plausible vectors → jump (dev / SWD flash path); anything else → stay in
  the BL waiting for UDS (brick-safe).
- **Update flow**: `0x10 02` in the app sets a no-init RAM handoff flag
  (0x20007F00 — top 256 B reserved in both linker scripts) and resets; the
  BL then serves the programming sequence `0x27 → 0x34 → 0x36 → 0x37 → 0x31
  CheckMemory` (CRC32 + anti-rollback). Meta is invalidated to UPDATING
  *before* the slot erase, so power loss mid-flash leaves the board in the
  BL, never bricked.
- **0x34 timing**: NRC 0x78 (responsePending) goes out before the page-wise
  erase (~1 s for a full 92 KB slot) so the tester switches to its P2*
  budget; the erase refreshes the IWDG between pages.

## Test

Bench suites run over SocketCAN against the board (`python-can` required):

```bash
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on && sudo ip link set can0 up

sudo python3 tests/uds_test.py can0    # UDS: 95 checks (sessions, security, NRC,
                                      #       DTC info, 0x2A/0x2C, OBD-II modes)
sudo python3 tests/fbl_test.py can0    # FBL: 30 checks (full flash cycle, brick
                                      #       recovery, anti-rollback, 0x78)
sudo python3 tests/obd2_test.py can0   # OBD-II modes 01/03/04/07/09: 22 checks
```

Host unit test — no hardware needed (DTC fault manager):

```bash
gcc -Wall -I tests/host -I Core/Inc Core/Src/obd2_simulator.c \
    tests/host/host_dtc_test.c -o /tmp/host_dtc_test && /tmp/host_dtc_test
```

Quick manual check:

```bash
cansend can0 7E0#02010C0000000000              # Mode 01 PID 0x0C (RPM)
candump can0                                    # → 7E8 response
```

> Latest full-bench run: UDS 95 + FBL 30 + OBD-II 22 all green on the
> committed IWDG-enabled build (2026-10, FBL suite entered via `0x10 02`
> handoff from the running app).

## Build size (current, HEAD)

Measured via `arm-none-eabi-size`:

| Image | Flash (text+data) | Slot | RAM (data+bss) |
|-------|-------------------|------|----------------|
| Application | ~65.8 KB | 92 KB app slot (70%) | ~21.8 KB / 32 KB (68%) |
| Bootloader | ~23.1 KB | 28 KB BL slot (81%) | ~5.6 KB |

> App RAM includes the 16 KB FreeRTOS heap; the top 256 B of RAM is reserved
> for the BL handoff flag. ISO-TP RX/TX buffers are 512 B each
> (`ISO_TP_MAX_MESSAGE_SIZE`) — larger receptions (OTA bulk) stream through
> the sink callback instead of full buffering.

## AUTOSAR (Classic) mapping

Each hand-written module fills a standard AUTOSAR BSW role:

| Module | Role | AUTOSAR BSW |
|--------|------|-------------|
| `fdcan_config.c` | FDCAN init, filters, RX/TX FIFO, ISR | Can / CanIf |
| `iso_tp.c` | SF/FF/CF/FC segmentation, STmin/BS pacing, stream sink | CanTp |
| `uds_service.c` + `diag_session.c` | service dispatch, sessions, S3 timer, SecurityAccess | Dcm (DSL/DSD/DSP) |
| `obd2_simulator.c` | fault debounce, DTC status machine, freeze frame, mirror memory | Dem |
| `bl_*.c` + `ota_flash.c` | boot gate (CRC32 / anti-rollback), 0x34/36/37/31 reprogramming | FBL / Fls |
| `main.c` (alive bits + IWDG) | watchdog refresh only when all tasks alive | WdgM |

## Development notes

- Bare-metal HAL (no CubeMX code generation); FreeRTOS added by hand.
- Toolchain: arm-none-eabi-gcc 15.x (tested 15.2.0, Fedora).
- Static analysis: cppcheck MISRA-C:2012 spot checks (e.g., Rule 10.4 explicit casts).
