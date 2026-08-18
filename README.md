# 🐭 BOTATO — Micromouse Robot
### STM32F411CEU6 · Maze Solver · University of Moratuwa · REV 1.2

---

## Overview

BOTATO is a competitive micromouse robot built for IEEE-standard 16×16 maze-solving competitions. It uses differential drive with dual 32-bit hardware quadrature encoders, five VL53-series Time-of-Flight distance sensors for wall detection, an MPU6050 IMU for heading control, and a flood-fill algorithm for maze exploration and speed-run path planning.

**Microcontroller:** STM32F411CEU6 (Black Pill)  
**Toolchain:** STM32CubeIDE · GCC

### Hardware Components (Bill of Materials)

| Component | Part | Quantity |
|---|---|---|
| Microcontroller | STM32F411CEU6 (Black Pill) | 1 |
| Distance Sensor | VL53-series ToF (I2C) | 5 |
| IMU | MPU6050 (I2C) | 1 |
| Motor Driver | TB6612FNG | 1 |
| Drive Motors | N20 with quadrature encoder | 2 |
| Motor Supply Regulator | MP1584EN step-down | 1 × 6V |
| Logic Supply Regulator | MP1584EN step-down | 1 × 3.3V |

---

## 1. ToF Sensor

All five VL53 sensors share a single I2C bus (I2C1). XSHUT pins are used for sequential address assignment on boot. GPIO1 pins signal data-ready via falling-edge EXTI interrupts.

### ToF Pinout & Addressing

| Position | GPIO1 Pin | EXTI Line | XSHUT Pin | I2C Address (Post-Boot) |
|---|---|---|---|---|
| ToF Left | PA2 | EXTI2 | PA10 | `0x30` |
| ToF LeftFront | PA3 | EXTI3 | PB4 | `0x31` |
| ToF Front | PA4 | EXTI4 | PB5 | `0x32` |
| ToF RightFront | PB0 | EXTI0 | PA7 | `0x33` |
| ToF Right | PB1 | EXTI1 | PB10 | `0x34` |

> **Note:** Every ToF sensor now has a dedicated EXTI line.
> 
> **XSHUT Boot Sequence:** Pull all XSHUT pins LOW at startup (all sensors disabled). Bring each HIGH one at a time, assign a unique I2C address, then proceed to the next. Default address is `0x29`; assign `0x30`–`0x34`.
> 
> **XSHUT Drive:** Configure XSHUT pins as open-drain output. Write LOW to disable, set to input/high-Z to enable — do not drive HIGH with 3.3V push-pull as the sensor's GPIO logic level is 2.8V.

### XSHUT Pin Definitions

```c
// XSHUT GPIO — open-drain, active LOW
#define XSHUT_LEFT_PORT         GPIOA
#define XSHUT_LEFT_PIN          GPIO_PIN_10

#define XSHUT_LEFTFRONT_PORT    GPIOB
#define XSHUT_LEFTFRONT_PIN     GPIO_PIN_4

#define XSHUT_FRONT_PORT        GPIOB
#define XSHUT_FRONT_PIN         GPIO_PIN_5

#define XSHUT_RIGHTFRONT_PORT   GPIOA
#define XSHUT_RIGHTFRONT_PIN    GPIO_PIN_7

#define XSHUT_RIGHT_PORT        GPIOB
#define XSHUT_RIGHT_PIN         GPIO_PIN_10
```

### Data-Ready Interrupt Flow (Planned Design)

> **Not yet implemented.** This is the intended design, not current firmware behaviour. The EXTI handlers in `stm32f4xx_it.c` today only call the HAL-generated `HAL_GPIO_EXTI_IRQHandler()`; no `HAL_GPIO_EXTI_Callback()` exists yet, so the flags below are not currently set by anything.

```c
// Sensor index definitions — must match XSHUT boot sequence order
typedef enum {
    TOF_LEFT        = 0,
    TOF_LEFTFRONT   = 1,
    TOF_FRONT       = 2,
    TOF_RIGHTFRONT  = 3,
    TOF_RIGHT       = 4
} ToF_Index;

volatile uint8_t  tof_ready[5]    = {0};  // Set by ISR, cleared by main loop
volatile uint16_t tof_distance[5] = {0};  // Written by main loop after I2C read
```

```c
// Every sensor now has a dedicated ISR. Keep these short!
void EXTI0_IRQHandler(void) {
    tof_ready[TOF_RIGHTFRONT] = 1;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void) {
    tof_ready[TOF_RIGHT] = 1;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void) {
    tof_ready[TOF_LEFT] = 1;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
}

// ... etc for EXTI3 and EXTI4

// Main loop handles the actual I2C transaction
for (int i = 0; i < 5; i++) {
    if (tof_ready[i]) {
        tof_distance[i] = read_tof_i2c(i);  // I2C read happens here
        tof_ready[i] = 0;
    }
}
```

---

## 2. IMU

The MPU6050 shares the fast-mode I2C bus with the ToF sensors.

| Signal | Pin | Notes |
|---|---|---|
| SCL | PB8 | Shared I2C1 bus (4.7 kΩ pull-up to 3.3V required) |
| SDA | PB9 | Shared I2C1 bus (4.7 kΩ pull-up to 3.3V required) |
| AD0 | GND | Fixed I2C address `0x68` |
| INT | N/A | IMU read synchronously in control loop — no interrupt needed |
| XDA/XCL | N/A | Auxiliary I2C not used |

> **Firmware Note (planned, not yet implemented):** The MPU6050's Digital Low Pass Filter (DLPF) must be manually configured in the firmware to match the 1 kHz frequency of the master control loop.

---

## 3. Motor Driver, Motor and Encoder

### Power Architecture

| Rail | Voltage | Regulator | Supplies |
|---|---|---|---|
| Motor power | 6V | MP1584EN #1 | TB6612FNG VMOT (Motor windings) |
| Logic power | 3.3V | MP1584EN #2 | STM32, ToF sensors, MPU6050, TB6612FNG VCC, N20 Hall-effect sensors |

### Motor PWM (Speed)

| Signal | Pin | Timer | Motor |
|---|---|---|---|
| PWMB (Left motor speed) | PA8 | TIM1_CH1 | Left |
| PWMA (Right motor speed) | PA9 | TIM1_CH2 | Right |

> TIM1 is the advanced timer — most precise PWM on the F411. Recommended frequency: 20 kHz (above human hearing, below N20 switching losses). Update speed by writing to `TIM1->CCR1` (left) and `TIM1->CCR2` (right).

### Motor Direction

| Signal | Pin | Motor | Connected To |
|---|---|---|---|
| B_IN1 | PB14 | Left | Left motor M2 (via B_OUT1) |
| B_IN2 | PB15 | Left | Left motor M1 (via B_OUT2) |
| A_IN1 | PB12 | Right | Right motor M1 (via A_OUT1) |
| A_IN2 | PB13 | Right | Right motor M2 (via A_OUT2) |

> **STBY pin:** Hardwired to 3.3V — driver always enabled. Emergency motor kill during development is handled physically via the main battery switch cutting power to the 6V motor regulator.

### Encoders (Hardware Quadrature)

Encoders utilize hardware quadrature processing with zero CPU overhead. Both TIM2 and TIM5 are 32-bit timers configured in encoder mode. Counter registers increment/decrement automatically based on phase relationship — no ISR required. Read `TIM2->CNT` and `TIM5->CNT` directly inside the control loop.

| Motor | Phase | Pin | Timer |
|---|---|---|---|
| Left | Phase A (CH1) | PA0 | TIM5_CH1 |
| Left | Phase B (CH2) | PA1 | TIM5_CH2 |
| Right | Phase A (CH1) | PA15 | TIM2_CH1 |
| Right | Phase B (CH2) | PB3 | TIM2_CH2 |

> **PA15 Note:** PA15 is the JTDI pin, but the STM32F411 does **not** have an AFIO peripheral — that remap mechanism is F1-series only and has no equivalent in the F4 HAL. No remap call is needed or possible: setting Debug = Serial Wire in CubeMX already releases PA15 from JTAG for `TIM2_CH1` automatically. Do **not** add `__HAL_AFIO_REMAP_SWJ_NOJTAG()` — it doesn't exist in the F4 HAL and will fail to compile.

---

## 4. Other Peripherals

| Function | Pin | Type / Notes |
|---|---|---|
| Mode Button 1 | PA5 | GPIO Input — polled at 1 kHz (Main loop) |
| Mode Button 2 | PA6 | GPIO Input — polled at 1 kHz (Main loop) |
| SWDIO | PA13 | SWD Programming |
| SWCLK | PA14 | SWD Programming |

> **Why PA5/PA6 for buttons?** PB0 and PB1 are occupied by ToF RightFront (EXTI0) and ToF Right (EXTI1) respectively. PB0 and PB1 must not be used for any other purpose.
>
> **PA5/PA6 EXTI warning:** PA5 is EXTI5 and PA6 is EXTI6, both under the shared `EXTI9_5_IRQHandler`. In CubeMX, configure PA5 and PA6 strictly as `GPIO_Input` with **no EXTI trigger assigned** — not even a disabled one. An accidental EXTI configuration here will cause phantom interrupts from button bounce inside the shared handler.
>
> Add a 100nF capacitor across each button to GND for hardware debounce. Keep PA13/PA14 free to flash via ST-Link V2.

---

## 5. Firmware and Architecture

### Timer Assignments

| Timer | Function | Notes |
|---|---|---|
| TIM1 | Motor PWM | Advanced timer, ~20 kHz |
| TIM2 | Right encoder | 32-bit quadrature, no ISR |
| TIM3 | Master control loop | 1 kHz (Prescaler=99, Period=999) |
| TIM5 | Left encoder | 32-bit quadrature, no ISR |
| SysTick | HAL system tick | 1 ms, HAL internal only |

### NVIC Priority Table

| Priority | Interrupt | Role |
|---|---|---|
| 0 (highest) | TIM3 — control loop | PID, encoder read, PWM update |
| 1 | EXTI0 to EXTI4 | ToF dedicated data-ready lines |
| 15 (lowest) | SysTick | HAL tick only |

### Control Loop (TIM3 — 1 kHz) (Planned Design)

> **Not yet implemented.** `main.c`'s main loop is currently an empty stub — none of the steps below exist in code yet. This describes the intended one-line-ISR architecture: `TIM3_IRQHandler` will only set a flag, and all real work happens in the main loop.

```text
TIM3 IRQ fires every 1 ms
│
└── Set control_tick = 1                  (signals main loop; ISR does nothing else)

Main loop, on control_tick:
├── Read TIM5->CNT  → left encoder delta   (no I2C, no ISR)
├── Read TIM2->CNT  → right encoder delta  (no I2C, no ISR)
├── Read MPU6050 over I2C                  (synchronous, no INT pin)
├── Run PID speed + alignment controller
└── Write TIM1->CCR1, TIM1->CCR2          (left/right PWM duty)
```

---

*BOTATO — Department of Computer Science & Engineering, University of Moratuwa*  
*Drawn by: Isiwara · Date: 2026-08-06 · REV 1.2*