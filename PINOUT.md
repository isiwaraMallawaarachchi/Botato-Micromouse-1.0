# 🐭 BOTATO — Hardware Pinout & Peripheral Configuration
### STM32F411CEU6 · University of Moratuwa · REV 1.3

This document contains ONLY verified hardware configuration — pin
assignments, timer settings, clock config, and power architecture.
For software architecture and design decisions, see `DECISIONS.md`.

This pinout is sourced from a verified-clean `.ioc` file, cross-checked
pin-by-pin against physical board wiring. Treat every value here as
ground truth.

---

## MCU

| Parameter | Value |
|---|---|
| Part | STM32F411CEU6 |
| Package | UFQFPN48 |
| Board | Black Pill |
| Firmware package | STM32Cube FW_F4 V1.28.3 |
| Toolchain | STM32CubeIDE / GCC (C++17, `-fno-exceptions -fno-rtti`) |

---

## Clock Configuration

| Parameter | Value |
|---|---|
| HSE (external crystal) | 25 MHz |
| PLLM | 12 |
| PLLN | 96 |
| PLLP | ÷2 |
| SYSCLK | 100 MHz |
| AHB (HCLK) | 100 MHz |
| APB1 prescaler | ÷2 → PCLK1 = 50 MHz (timer clock = 100 MHz) |
| APB2 prescaler | ÷1 → PCLK2 = 100 MHz (timer clock = 100 MHz) |

---

## Bill of Materials

| Component | Part | Qty |
|---|---|---|
| Microcontroller | STM32F411CEU6 (Black Pill) | 1 |
| Distance sensor | VL53L0X ToF (I2C) | 5 |
| IMU | MPU6050 (I2C) | 1 |
| Motor driver | TB6612FNG | 1 |
| Drive motors | N20 with quadrature encoder, 50:1 gearbox | 2 |
| Motor supply regulator | MP1584EN step-down | 1 × 6V |
| Logic supply regulator | MP1584EN step-down | 1 × 3.3V |

---

## Complete Pin Map

| Pin | Function |
|---|---|
| PA0 | Left encoder A (TIM5_CH1) |
| PA1 | Left encoder B (TIM5_CH2) |
| PA2 | ToF Left — GPIO1 (wired, unused) |
| PA3 | ToF LeftFront — GPIO1 (wired, unused) |
| PA4 | ToF Front — GPIO1 (wired, unused) |
| PA5 | BTN1 (input, pull-up) |
| PA6 | BTN2 (input, pull-up) |
| PA7 | ToF RightFront — XSHUT |
| PA8 | Left motor PWM (TIM1_CH1) |
| PA9 | Right motor PWM (TIM1_CH2) |
| PA10 | ToF Left — XSHUT |
| PA13 | SWDIO |
| PA14 | SWCLK |
| PA15 | Right encoder A (TIM2_CH1) |
| PB0 | ToF RightFront — GPIO1 (wired, unused) |
| PB1 | ToF Right — GPIO1 (wired, unused) |
| PB3 | Right encoder B (TIM2_CH2) |
| PB4 | ToF LeftFront — XSHUT |
| PB5 | ToF Front — XSHUT |
| PB6 | I2C1 SCL — ToF sensors (AF4) |
| PB7 | I2C1 SDA — ToF sensors (AF4) |
| PB8 | ToF Right — XSHUT |
| PB9 | I2C2 SDA — MPU6050 (**AF9**) |
| PB10 | I2C2 SCL — MPU6050 (AF4) |
| PB12 | AIN1 — Right motor direction |
| PB13 | AIN2 — Right motor direction |
| PB14 | BIN1 — Left motor direction |
| PB15 | BIN2 — Left motor direction |
| PC13 | Onboard status LED (active LOW) |

**Genuinely unused:** PA11, PA12, PB2, PC14, PC15

---

## ToF Sensors — I2C + XSHUT

All 5 sensors share I2C1. Each has its own XSHUT line for sequential
address assignment at boot.

| Position | Index | XSHUT Pin | I2C Addr (post-boot) | GPIO1 Pin (unused) |
|---|---|---|---|---|
| Left | 0 | PA10 | `0x30` | PA2 |
| LeftFront | 1 | PB4 | `0x31` | PA3 |
| Front | 2 | PB5 | `0x32` | PA4 |
| RightFront | 3 | PA7 | `0x33` | PB0 |
| Right | 4 | PB8 | `0x34` | PB1 |

> ⚠️ **XSHUT_RIGHT is PB8, not PB10.** PB10 is I2C2_SCL. Older revisions
> of `README.md` listed PB10 here — that is stale and wrong.

**XSHUT pin config:** `GPIO_MODE_OUTPUT_OD` (open-drain), `GPIO_NOPULL`.
Sensor logic is 2.8V — open-drain lets the STM32 pull LOW (disable) or
release (sensor's own pull-up brings it to 2.8V) without ever driving
3.3V onto the pin.

**Boot sequence:** hold all XSHUT LOW → release one → sensor appears at
default `0x29` → write new address → repeat. Address assignment lives in
sensor RAM and is lost on power-down, so this runs at every boot.

### GPIO1 data-ready pins are wired but NOT used

The five GPIO1 pins are physically connected but **no EXTI is configured
on them**. The production driver runs the sensors in continuous ranging
mode and polls them from the main loop instead — see DECISIONS.md #2
(revised). These pins are effectively spare; do not assume an interrupt
exists on them.

---

## I2C Buses — TWO separate buses

The ToF sensors and the IMU are on **separate I2C peripherals**. They were
split because five ToF breakout boards each carry a 10 kΩ pull-up; in
parallel these dragged the shared bus down to ~1.5 kΩ, below the level at
which the MPU6050 could reliably pull the line low. See DECISIONS.md #16.

| Bus | Signal | Pin | AF | Devices |
|---|---|---|---|---|
| I2C1 | SCL | PB6 | AF4 | 5× ToF sensors |
| I2C1 | SDA | PB7 | AF4 | 5× ToF sensors |
| I2C2 | SCL | PB10 | AF4 | MPU6050 only |
| I2C2 | SDA | PB9 | **AF9** | MPU6050 only |

Both buses run Fast Mode 400 kHz.

> ⚠️ **PB9 uses AF9, not AF4.** PB9 can serve either I2C1_SDA (AF4) or
> I2C2_SDA (AF9). If generated code sets AF4 for PB9 under I2C2, the pin
> stays internally wired to I2C1 and the bus silently does nothing.
> Verify `GPIO_AF9_I2C2` appears in `HAL_I2C_MspInit` in `Core/Src/i2c.c`.

**Pull-ups:** I2C1 uses the ToF boards' own onboard 10 kΩ resistors
(~2 kΩ combined). I2C2 uses only the MPU6050 board's onboard pull-ups —
no external resistors were needed, confirmed working.

> **Diagnostic note:** because every I2C pull-up on both buses lives on a
> sensor breakout rather than the MCU board, a dead 3.3V sensor rail
> removes the pull-ups entirely and both buses go silent while the MCU
> still runs happily on debugger power. If a bus scan finds zero devices,
> measure 3.3V at a sensor VCC pin *before* suspecting firmware.

---

## MPU6050 (IMU)

| Signal | Pin | Notes |
|---|---|---|
| SCL | PB10 | I2C2 — dedicated bus, not shared with ToF |
| SDA | PB9 | I2C2 — dedicated bus, not shared with ToF |
| AD0 | GND | Fixed I2C address `0x68` |
| INT | Not connected | Not wired — see DECISIONS.md #3 |
| XDA / XCL | Not connected | Auxiliary I2C unused |

---

## Encoders — Hardware Quadrature (32-bit Timers)

| Motor | Phase A (CH1) | Phase B (CH2) | Timer | Mode |
|---|---|---|---|---|
| Left | PA0 | PA1 | TIM5 | `TIM_ENCODERMODE_TI12`, 32-bit |
| Right | PA15 | PB3 | TIM2 | `TIM_ENCODERMODE_TI12`, 32-bit |

No AFIO remap required or used. Setting `Debug = Serial Wire` in CubeMX
already frees PA15 for `TIM2_CH1` automatically on the F411 — this MCU
has no AFIO peripheral.

**Right encoder is negated in software** so both wheels read positive
when driving forward — see DECISIONS.md #18.

### Derived drivetrain constants (`Config.h`)

| Constant | Value |
|---|---|
| Encoder PPR | 7 |
| Gear ratio | 50:1 |
| Quadrature | 4× |
| Counts per wheel rev | 1400 |
| Wheel diameter | 43 mm (nominal — measure effective rolling dia) |
| mm per tick | ≈ 0.0965 |
| Wheelbase | 87 mm |

---

## Motor PWM — TIM1

| Signal | Pin | Timer Channel | Motor |
|---|---|---|---|
| PWMB (speed) | PA8 | TIM1_CH1 | Left |
| PWMA (speed) | PA9 | TIM1_CH2 | Right |

**TIM1 config:** Prescaler = 0, Period (ARR) = 4999 → 20 kHz PWM at
100 MHz timer clock.

> **Practical speed ceiling:** N20 200 rpm × 43 mm wheel ≈ **450 mm/s**
> maximum, less under load. Commanding a target above this saturates the
> forward PID, which clips one side of the differential mix and produces
> asymmetric steering authority. Keep search/speed targets well below it.

---

## Motor Direction — TB6612FNG

| Signal | Pin | Motor | Mode |
|---|---|---|---|
| AIN1 | PB12 | Right | `GPIO_MODE_OUTPUT_PP` |
| AIN2 | PB13 | Right | `GPIO_MODE_OUTPUT_PP` |
| BIN1 | PB14 | Left | `GPIO_MODE_OUTPUT_PP` |
| BIN2 | PB15 | Left | `GPIO_MODE_OUTPUT_PP` |

**TB6612FNG output wiring:**

| TB6612 Output | Connected to |
|---|---|
| A_OUT1 | Right motor M1 |
| A_OUT2 | Right motor M2 |
| B_OUT1 | Left motor M2 |
| B_OUT2 | Left motor M1 |

**Direction polarity is inverted** relative to the datasheet's textbook
mapping — handled inside `Motor`, see DECISIONS.md #15.

**STBY:** Hardwired to 3.3V. Driver always enabled — no software kill switch.

---

## Mode Buttons

| Signal | Pin | Mode |
|---|---|---|
| BTN1 | PA5 | `GPIO_MODE_INPUT`, `GPIO_PULLUP` — NOT EXTI |
| BTN2 | PA6 | `GPIO_MODE_INPUT`, `GPIO_PULLUP` — NOT EXTI |

Active LOW (wired to GND), polled and debounced in the main loop.

---

## Status LED

| Signal | Pin | Notes |
|---|---|---|
| LED | PC13 | Onboard Black Pill LED, **active LOW** |

Driven by `Indicator`. Buzzer output is stubbed in firmware, no pin assigned.

---

## SWD Programming

| Signal | Pin | Mode |
|---|---|---|
| SWDIO | PA13 | Serial Wire |
| SWCLK | PA14 | Serial Wire |

---

## Timer Summary

| Timer | Role | Config | ISR? |
|---|---|---|---|
| TIM1 | Motor PWM | Prescaler 0, Period 4999 (20 kHz) | No |
| TIM2 | Right encoder | Encoder mode TI12, 32-bit | No |
| TIM3 | Master control loop | Prescaler 99, Period 999 (1 kHz), AutoReloadPreload enabled | Yes |
| TIM5 | Left encoder | Encoder mode TI12, 32-bit | No |

---

## NVIC Priority Table

| Interrupt | Preemption Priority | Sub-Priority |
|---|---|---|
| TIM3 global | 0 (highest) | 0 |
| SysTick | 15 (lowest) | 0 |

> EXTI0–EXTI4 were configured in earlier revisions for ToF data-ready.
> They are **no longer enabled** — the ToF driver polls in continuous
> mode instead. If the `.ioc` is regenerated, make sure these EXTI lines
> stay disabled so stray edges can't fire empty handlers.

---

## Power Architecture

| Rail | Voltage | Regulator | Supplies |
|---|---|---|---|
| Motor power | 6V | MP1584EN #1 | TB6612FNG VMOT (motor windings) |
| Logic power | 3.3V | MP1584EN #2 | MCU, all ToF sensors, MPU6050, TB6612FNG VCC, N20 Hall-effect encoders |

> **No motor decoupling capacitor is fitted across VMOT.** PWM switching
> noise on the shared ground is a known cause of I2C and encoder glitches.
> See DECISIONS.md #19.

---

*BOTATO — Department of Computer Science & Engineering, University of Moratuwa*
