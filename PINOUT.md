# 🐭 BOTATO — Hardware Pinout & Peripheral Configuration
### STM32F411CEU6 · University of Moratuwa

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
| Toolchain | STM32CubeIDE / GCC |

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
| Distance sensor | VL53-series ToF (I2C) | 5 |
| IMU | MPU6050 (I2C) | 1 |
| Motor driver | TB6612FNG | 1 |
| Drive motors | N20 with quadrature encoder | 2 |
| Motor supply regulator | MP1584EN step-down | 1 × 6V |
| Logic supply regulator | MP1584EN step-down | 1 × 3.3V |

---

## ToF Sensors — I2C + XSHUT + Dedicated EXTI

All 5 sensors share one I2C bus. Each has its own dedicated EXTI line —
no shared interrupt handlers.

| Position | GPIO1 Pin | EXTI Line | XSHUT Pin | I2C Addr (post-boot) |
|---|---|---|---|---|
| Left | PA2 | EXTI2 | PA10 | `0x30` |
| LeftFront | PA3 | EXTI3 | PB4 | `0x31` |
| Front | PA4 | EXTI4 | PB5 | `0x32` |
| RightFront | PB0 | EXTI0 | PA7 | `0x33` |
| Right | PB1 | EXTI1 | PB10 | `0x34` |

**GPIO1 pin config:** `GPIO_MODE_IT_FALLING`, `GPIO_PULLUP`.
**XSHUT pin config:** `GPIO_MODE_OUTPUT_OD` (open-drain), `GPIO_NOPULL`.
Sensor logic is 2.8V — open-drain lets the STM32 pull LOW (disable) or
release (sensor's own pull-up brings it to 2.8V) without ever driving
3.3V onto the pin.

---

## I2C Bus — Shared by ToF Sensors + MPU6050

| Signal | Pin | Notes |
|---|---|---|
| I2C1_SCL | PB8 | Fast Mode 400 kHz. External 4.7 kΩ pull-up to 3.3V required. |
| I2C1_SDA | PB9 | External 4.7 kΩ pull-up to 3.3V required. |

---

## MPU6050 (IMU)

| Signal | Pin | Notes |
|---|---|---|
| SCL | PB8 | Shared I2C1 bus |
| SDA | PB9 | Shared I2C1 bus |
| AD0 | GND | Fixed I2C address `0x68` |
| INT | Not connected | Not wired — see DECISIONS.md for rationale |
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

---

## Motor PWM — TIM1

| Signal | Pin | Timer Channel | Motor |
|---|---|---|---|
| PWMB (speed) | PA8 | TIM1_CH1 | Left |
| PWMA (speed) | PA9 | TIM1_CH2 | Right |

**TIM1 config:** Prescaler = 0, Period (ARR) = 4999 → 20 kHz PWM at
100 MHz timer clock.

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

**STBY:** Hardwired to 3.3V. Driver always enabled — no software kill switch.

---

## Mode Buttons

| Signal | Pin | Mode |
|---|---|---|
| BTN1 | PA5 | `GPIO_MODE_INPUT`, `GPIO_PULLUP` — NOT EXTI |
| BTN2 | PA6 | `GPIO_MODE_INPUT`, `GPIO_PULLUP` — NOT EXTI |

---

## SWD Programming

| Signal | Pin | Mode |
|---|---|---|
| SWDIO | PA13 | Serial Wire |
| SWCLK | PA14 | Serial Wire |

---

## Reserved / Unused Pins

| Pin | Status |
|---|---|
| PB6, PB7 | Not configured in this project. Do not assign — see DECISIONS.md for the board revision history behind this. |

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
| EXTI0 | 1 | 0 |
| EXTI1 | 1 | 0 |
| EXTI2 | 1 | 0 |
| EXTI3 | 1 | 0 |
| EXTI4 | 1 | 0 |
| SysTick | 15 (lowest) | 0 |

---

## Power Architecture

| Rail | Voltage | Regulator | Supplies |
|---|---|---|---|
| Motor power | 6V | MP1584EN #1 | TB6612FNG VMOT (motor windings) |
| Logic power | 3.3V | MP1584EN #2 | MCU, all ToF sensors, MPU6050, TB6612FNG VCC, N20 Hall-effect encoders |

---

*BOTATO — Department of Computer Science & Engineering, University of Moratuwa*
