# MCP2562FD Pinout (Reference)

> Source: Microchip DS20005284A (MCP2561/2FD High-Speed CAN FD Transceiver)
> Table 1-2 PIN DESCRIPTIONS — MCP2562FD PDIP/SOIC
> Verified: 2026-06-27

## ⚠️ MCP2562FD ≠ MCP2561FD pin assignment

MCP2562FD and MCP2561FD share the same 8-pin package but **differ on pin 5**:
- **MCP2562FD**: pin 5 = **VIO** (digital I/O supply, matches MCU logic level)
- **MCP2561FD**: pin 5 = **SPLIT** (common-mode stabilization output, VDD/2)

Identify the exact part by the marking on the package (`MCP2562FD` vs `MCP2561FD`).

## Pin layout (8-pin SOIC / PDIP)

```
        pin1 dot (●)
         ┌────╮
   TXD 1 │    │ 8 STBY   → GND (Normal mode)
   VSS 2 │    │ 7 CANH   → bus CANH
   VDD 3 │    │ 6 CANL   → bus CANL
   RXD 4 │    │ 5 VIO    → 3.3V
         └────╯
```

| Pin | Name | Function | STM32G431RB connection |
|-----|------|----------|------------------------|
| 1 | TXD | Transmit Data Input (MCU→chip) | **PA12** (FDCAN1_TX, CN10-12) |
| 2 | VSS | Ground | **GND** |
| 3 | VDD | Main supply (transceiver/receiver power) | **5V** (4.5–5.5V) |
| 4 | RXD | Receive Data Output (chip→MCU) | **PA11** (FDCAN1_RX, CN10-14) |
| 5 | VIO | Digital I/O supply (sets RXD/TXD/STBY logic level) | **3.3V** (matches MCU logic) |
| 6 | CANL | CAN Low bus | bus CANL |
| 7 | CANH | CAN High bus | bus CANH |
| 8 | STBY | Standby mode input | **GND** (LOW = Normal mode) |

## STM32 ↔ MCP2562FD wiring summary

```
STM32 PA12 (FDCAN1_TX) ── pin 1 (TXD)
STM32 PA11 (FDCAN1_RX) ── pin 4 (RXD)
STM32 3.3V            ── pin 5 (VIO)    ← MCU logic voltage
Nucleo 5V             ── pin 3 (VDD)    ← transceiver main supply
GND                   ── pin 2 (VSS), pin 8 (STBY)
bus                   ── pin 6 (CANL), pin 7 (CANH)
```

## Key electrical characteristics (datasheet)

- **VDD supply range**: 4.5V–5.5V (undervoltage threshold ~4V)
- **VIO supply range**: 1.8V–5.5V (undervoltage threshold ~1.2V)
- **Dominant differential input voltage**: VDIFF > 0.9V (Normal mode)
- **Dominant differential output voltage**: typical 2.0V (CANH ~3.5V, CANL ~1.5V)

## Operating notes

1. **VDD undervoltage (< 4V) forces RXD to Recessive (HIGH)** — TX may still work while RX does not. Always verify pin 3 (VDD) measures 5V.
2. **STBY (pin 8) HIGH = Standby mode** — the high-speed receiver is OFF and RXD outputs only the wake-up filtered signal. Drive STBY LOW (GND) for normal operation.
3. **VIO (pin 5) sets the RXD output level** — VIO = 3.3V keeps RXD at 0–3.3V (safe for a 3.3V MCU); VIO = 5V drives RXD to 0–5V (overvoltage risk for a 3.3V MCU pin).

## Datasheet

- Official: https://ww1.microchip.com/downloads/en/DeviceDoc/20005284A.pdf (DS20005284A)

---

# MCP2562FD 핀맵 (참조용)

> 출처: Microchip DS20005284A (MCP2561/2FD High-Speed CAN FD Transceiver)
> Table 1-2 PIN DESCRIPTIONS — MCP2562FD PDIP/SOIC 기준
> 최종 확인: 2026-06-27

## ⚠️ 주의: MCP2562FD ≠ MCP2561FD 핀배치

MCP2562FD와 MCP2561FD는 같은 8핀 패키지이지만 **pin 5가 다름**:
- **MCP2562FD**: pin 5 = **VIO** (디지털 I/O 전원, MCU 로직 레벨 맞춤용)
- **MCP2561FD**: pin 5 = **SPLIT** (커먼 모드 안정화 출력, VDD/2)

칩 표면 글자로 정확히 구분할 것 (`MCP2562FD` vs `MCP2561FD`).

## MCP2562FD 핀 배치 (8-pin SOIC / PDIP)

```
        pin1 표시(●)
         ┌────╮
   TXD 1 │    │ 8 STBY   → GND (정상모드)
   VSS 2 │    │ 7 CANH   → 버스 CANH
   VDD 3 │    │ 6 CANL   → 버스 CANL
   RXD 4 │    │ 5 VIO    → 3.3V
         └────╯
```

| 핀 | 이름 | 기능 | STM32G431RB 연결 |
|----|------|------|------------------|
| 1 | TXD | Transmit Data Input (MCU→칩) | **PA12** (FDCAN1_TX, CN10-12) |
| 2 | VSS | Ground | **GND** |
| 3 | VDD | 주 전원 (트랜시버/수신기 전원) | **5V** (4.5~5.5V) |
| 4 | RXD | Receive Data Output (칩→MCU) | **PA11** (FDCAN1_RX, CN10-14) |
| 5 | VIO | 디지털 I/O 전원 (RXD/TXD/STBY 레벨 결정) | **3.3V** (MCU 로직과 일치) |
| 6 | CANL | CAN Low 버스 | 버스 CANL |
| 7 | CANH | CAN High 버스 | 버스 CANH |
| 8 | STBY | Standby 모드 입력 | **GND** (LOW=Normal 모드) |

## STM32 ↔ MCP2562FD 배선 요약

```
STM32 PA12 (FDCAN1_TX) ── pin 1 (TXD)
STM32 PA11 (FDCAN1_RX) ── pin 4 (RXD)
STM32 3.3V            ── pin 5 (VIO)    ← MCU 로직 전압
Nucleo 5V             ── pin 3 (VDD)    ← 트랜시버 주 전원
GND                   ── pin 2 (VSS), pin 8 (STBY)
버스                   ── pin 6 (CANL), pin 7 (CANH)
```

## 핵심 전기 특성 (데이터시트 발췌)

- **VDD 전원 범위**: 4.5V ~ 5.5V (언더전압 임계 ~4V)
- **VIO 전원 범위**: 1.8V ~ 5.5V (언더전압 임계 ~1.2V)
- **Dominant 차동 입력 전압**: VDIFF > 0.9V (Normal 모드)
- **Dominant 차동 출력 전압**: 전형 2.0V (CANH ~3.5V, CANL ~1.5V)

## 동작 시 주의사항

1. **VDD 언더전압 (< 4V) 시 RXD 강제 Recessive(HIGH)** — TX는 동작해도 RX만 안 될 수 있음. pin 3 (VDD)이 5V인지 반드시 실측할 것.
2. **STBY(pin 8) HIGH = Standby 모드** — 고속 수신기 OFF, RXD는 wake-up 필터 신호만 출력. 정상 동작하려면 STBY = LOW(GND).
3. **VIO(pin 5)는 RXD 출력 레벨 결정** — VIO=3.3V면 RXD가 0~3.3V로 나와 3.3V MCU에 안전. VIO=5V면 RXD가 0~5V → 3.3V MCU 핀 과전압 위험.

## 데이터시트

- 공식: https://ww1.microchip.com/downloads/en/DeviceDoc/20005284A.pdf (DS20005284A)
