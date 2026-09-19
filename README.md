# 🐭 BOTATO — Micromouse Robot
### STM32F411CEU6 · Maze Solver · University of Moratuwa · REV 2.0

---

## Overview

BOTATO is a competitive micromouse robot built for IEEE-standard 16×16
maze-solving competitions (ROBOFEST 2026). It uses differential drive with
dual 32-bit hardware quadrature encoders, five VL53L0X Time-of-Flight
distance sensors for wall detection, an MPU6050 IMU for heading control,
and a flood-fill algorithm for maze exploration and speed-run planning.

**Microcontroller:** STM32F411CEU6 (Black Pill)
**Language:** C++17 application layer over the C HAL
**Toolchain:** STM32CubeIDE · GCC

**Companion documents**
- `PINOUT.md` — verified hardware configuration (pins, timers, clocks, power)
- `DECISIONS.md` — why the firmware is built the way it is

### Hardware Components (Bill of Materials)

| Component | Part | Quantity |
|---|---|---|
| Microcontroller | STM32F411CEU6 (Black Pill) | 1 |
| Distance Sensor | VL53L0X ToF (I2C) | 5 |
| IMU | MPU6050 (I2C) | 1 |
| Motor Driver | TB6612FNG | 1 |
| Drive Motors | N20 with quadrature encoder, 50:1 | 2 |
| Motor Supply Regulator | MP1584EN step-down | 1 × 6V |
| Logic Supply Regulator | MP1584EN step-down | 1 × 3.3V |

---

## 1. ToF Sensors

All five VL53L0X sensors share **I2C1** (PB6/PB7). The MPU6050 is on a
**separate I2C2 bus** (PB10/PB9) — see DECISIONS.md #16 for why. XSHUT
pins are used for sequential address assignment at boot.

### ToF Pinout & Addressing

| Position | Index | XSHUT Pin | I2C Address (Post-Boot) |
|---|---|---|---|
| ToF Left | 0 | PA10 | `0x30` |
| ToF LeftFront | 1 | PB4 | `0x31` |
| ToF Front | 2 | PB5 | `0x32` |
| ToF RightFront | 3 | PA7 | `0x33` |
| ToF Right | 4 | PB8 | `0x34` |

> **XSHUT Boot Sequence:** Pull all XSHUT pins LOW at startup (all sensors
> disabled). Bring each HIGH one at a time, assign a unique I2C address,
> then proceed to the next. Default address is `0x29`; assign `0x30`–`0x34`.
> Assignment lives in sensor RAM and is lost at power-down, so this runs
> at every boot.
>
> **XSHUT Drive:** Configure XSHUT pins as open-drain output. Write LOW to
> disable, release to enable — do not drive HIGH with 3.3V push-pull, as
> the sensor's GPIO logic level is 2.8V.

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
#define XSHUT_RIGHT_PIN         GPIO_PIN_8   // NOT PB10 — that is I2C2_SCL
```

### Reading strategy — continuous mode, polled

The sensors run in **continuous ranging mode** and are polled from the
main loop by `WallSensorArray::poll()`. The GPIO1 data-ready pins are
physically wired but **no EXTI is configured on them** — see
DECISIONS.md #2 (revised) for why the interrupt approach was dropped.

Each reading passes through a median + EMA filter in `tof.c`, then a
per-sensor linear calibration (`true = a·filtered + b`) in
`WallSensorArray`.

**Validity, not just presence.** `WallSensorArray::ok(i)` means *this
reading is fresh and in range*, not merely *the sensor booted*. An
out-of-range sensor reports a far sentinel value, never 0 — a 0 would be
read downstream as "wall touching the nose" and would end a cell
immediately. See DECISIONS.md #27.

---

## 2. IMU

The MPU6050 has its own dedicated I2C2 bus, **not shared** with the ToF
sensors.

| Signal | Pin | Notes |
|---|---|---|
| SCL | PB10 | Dedicated I2C2 bus |
| SDA | PB9 | Dedicated I2C2 bus (AF9, not AF4 — see DECISIONS.md #16) |
| AD0 | GND | Fixed I2C address `0x68` |
| INT | N/A | IMU read synchronously in control loop — no interrupt needed |
| XDA/XCL | N/A | Auxiliary I2C not used |

### Heading conventions (do not change casually)

- **Anticlockwise-positive.** `TURN_LEFT = +90`, `TURN_RIGHT = −90`.
- **The angle is continuous and unbounded** — no ±180 wrapping in `Gyro`.
  Wrapping happens only on the *error* inside `ControlLoop::tick()`.
- **Turn targets are relative** (`headingRef_ += ±90`), never absolute
  compass bearings.
- **Never command a single 180° turn** — that lands exactly on the wrap
  boundary where gyro noise decides the direction. Use two 90° steps.

See DECISIONS.md #21.

---

## 3. Motor Driver, Motors and Encoders

### Power Architecture

| Rail | Voltage | Regulator | Supplies |
|---|---|---|---|
| Motor power | 6V | MP1584EN #1 | TB6612FNG VMOT (motor windings) |
| Logic power | 3.3V | MP1584EN #2 | STM32, ToF sensors, MPU6050, TB6612FNG VCC, N20 Hall-effect sensors |

### Motor PWM (Speed)

| Signal | Pin | Timer | Motor |
|---|---|---|---|
| PWMB (Left motor speed) | PA8 | TIM1_CH1 | Left |
| PWMA (Right motor speed) | PA9 | TIM1_CH2 | Right |

> TIM1 is the advanced timer — most precise PWM on the F411. 20 kHz
> (above human hearing, below N20 switching losses).
>
> **Speed ceiling ≈ 450 mm/s.** Commanding more saturates the forward
> PID and clips one side of the differential mix, producing asymmetric
> steering. See DECISIONS.md #26.

### Motor Direction

| Signal | Pin | Motor | Connected To |
|---|---|---|---|
| B_IN1 | PB14 | Left | Left motor M2 (via B_OUT1) |
| B_IN2 | PB15 | Left | Left motor M1 (via B_OUT2) |
| A_IN1 | PB12 | Right | Right motor M1 (via A_OUT1) |
| A_IN2 | PB13 | Right | Right motor M2 (via A_OUT2) |

> **STBY pin:** Hardwired to 3.3V — driver always enabled. Emergency motor
> kill during development is handled physically via the main battery switch.

### Encoders (Hardware Quadrature)

Both TIM2 and TIM5 are 32-bit timers in encoder mode. Counters
increment/decrement automatically — no ISR required.

| Motor | Phase | Pin | Timer |
|---|---|---|---|
| Left | Phase A (CH1) | PA0 | TIM5_CH1 |
| Left | Phase B (CH2) | PA1 | TIM5_CH2 |
| Right | Phase A (CH1) | PA15 | TIM2_CH1 |
| Right | Phase B (CH2) | PB3 | TIM2_CH2 |

> **PA15 Note:** PA15 is JTDI by default, but the STM32F411 has **no AFIO
> peripheral** — `__HAL_RCC_AFIO_CLK_ENABLE()` and
> `__HAL_AFIO_REMAP_SWJ_NOJTAG()` are F1-series only and will not compile.
> Setting Debug = Serial Wire in CubeMX already releases PA15 for
> TIM2_CH1. See DECISIONS.md #9.

---

## 4. Other Peripherals

| Function | Pin | Type / Notes |
|---|---|---|
| Mode Button 1 | PA5 | GPIO Input, pull-up, active LOW — polled |
| Mode Button 2 | PA6 | GPIO Input, pull-up, active LOW — polled |
| Status LED | PC13 | Onboard LED, active LOW |
| SWDIO | PA13 | SWD Programming |
| SWCLK | PA14 | SWD Programming |

> **PA5/PA6 EXTI warning:** PA5 is EXTI5 and PA6 is EXTI6, both under the
> shared `EXTI9_5_IRQHandler`. Configure them strictly as `GPIO_Input`
> with **no EXTI trigger assigned** — not even a disabled one.
>
> Add a 100nF capacitor across each button to GND for hardware debounce.
> Keep PA13/PA14 free to flash via ST-Link V2.

---

## 5. Firmware Architecture

The application is C++; the vendor HAL and the VL53L0X driver stay C.
`main.cpp` is the only renamed file — all other CubeMX files remain `.c`.

```
Core/App/
├── Robot.hpp/.cpp          facade; owns every subsystem
├── config/    Config.h     all physical + tuning constants (cfg::)
├── tof/       tof.c/.h     C driver: XSHUT, continuous mode, median+EMA
├── encoder/   Encoder
├── gyro/      Gyro
├── wallsensor/WallSensorArray
├── motor/     Motor
├── drive/     DifferentialDrive
├── control/   PIDController, ControlLoop
├── buttons/   ButtonManager
├── modes/     ModeController
├── maze/      MazeConfig.h, MazeMap
├── floodfill/ FloodFill
├── planner/   Planner
├── navigator/ Navigator
├── indicator/ Indicator
└── tests/     Tests, Phase2Test, Tests_I2C
```

`main.cpp` reduces to `robot.init()`, `robot.onMainLoop()`, and a TIM3
callback into `robot.onControlTick()`.

### Two execution contexts

```
┌──────────────────────────────┐   ┌──────────────────────────────┐
│  TIM3 ISR — 1 kHz, fixed      │   │  while(1) — main loop         │
│  Robot::onControlTick()       │   │  Robot::onMainLoop()          │
├──────────────────────────────┤   ├──────────────────────────────┤
│  gyro_.update()               │   │  walls_.poll()   (I2C, slow)  │
│  encL_.update()               │   │  btn_.update()                │
│  encR_.update()               │   │  modes_.update() ── Navigator │
│  ctrl_.tick()  → motors       │   │  led_.update()                │
└──────────────────────────────┘   └──────────────────────────────┘
     FAST · DETERMINISTIC              BACKGROUND · NON-BLOCKING
```

### Timer Assignments

| Timer | Function | Notes |
|---|---|---|
| TIM1 | Motor PWM | Advanced timer, 20 kHz |
| TIM2 | Right encoder | 32-bit quadrature, no ISR |
| TIM3 | Master control loop | 1 kHz (Prescaler=99, Period=999) |
| TIM5 | Left encoder | 32-bit quadrature, no ISR |
| SysTick | HAL system tick | 1 ms, HAL internal only |

### NVIC Priority Table

| Priority | Interrupt | Role |
|---|---|---|
| 0 (highest) | TIM3 — control loop | Sensor update, PID, PWM |
| 15 (lowest) | SysTick | HAL tick only |

EXTI0–EXTI4 are no longer enabled (ToF is polled).

### Control Loop (TIM3 — 1 kHz)

```text
TIM3 IRQ fires every 1 ms
│
├── gyro_.update()     → rate + integrated heading  (I2C2)
├── encL_.update()     → left distance + speed      (TIM5->CNT)
├── encR_.update()     → right distance + speed     (TIM2->CNT)
└── ctrl_.tick()
      ├── pvX = mean wheel speed,  pvW = gyro rate
      ├── heading error → pidHeading → rate target  (outer)
      ├── rate error    → pidW       → turn PWM     (inner)
      ├── speed error   → pidX       → fwd PWM
      └── mix: L = fwd − turn,  R = fwd + turn  → DifferentialDrive
```

---

## 6. Maze Solving

### Maze configuration (`MazeConfig.h`)

| Constant | Value | Meaning |
|---|---|---|
| `WIDTH` / `HEIGHT` | 16 | Competition maze |
| `MAX_DIM` | 16 | Static array bound |
| `START_X` / `START_Y` | 0 / 0 | Bottom-left; `y = 0` is the bottom row |
| `GOAL_X_MIN/MAX` | 7 / 8 | Centre 2×2 block |
| `GOAL_Y_MIN/MAX` | 7 / 8 | |
| `CELL_MM` | measured | Wall-centre to wall-centre pitch |

The goal is an **inclusive bounding box**, not a cell list. Any cell
inside counts as reached, so the robot stops at the first mouth it enters
and turns back rather than driving to a geometric centre. Retargeting a
different maze means editing these numbers only.

### Per-cell state machine (`Navigator`)

```
┌────────┐  ┌────────┐  ┌───────────┐       ┌──────────┐
│ SENSE  │─►│ DECIDE │─►│  TURNING  │──────►│ DRIVING  │──► ARRIVE
│ read   │  │ flood, │  │ pivot 90° │   ┌──►│ one cell │      │
│ walls  │  │ choose │  └───────────┘   │   └──────────┘      │
│ → map  │  │ turn   │──(no turn)───────┘                     │
└────────┘  └───┬────┘                                        │
     ▲          ├──(U-turn)──► DEAD_END (K-turn)               │
     └──────────┴──(goal / home)──► goalOrReturnTransition()◄──┘
```

**Run states:** `SEARCH` → `RETURN` → `DONE`, plus `SPEED`.

### Position referencing

| Question | Decisive source |
|---|---|
| Which way am I pointing? | Gyro, sole authority |
| How far along the cell? | Front ToF if a wall is in range, else encoders |
| How fast am I going? | Encoders |
| Is there a wall here? | ToF |
| Which grid cell am I in? | Dead reckoning — counted moves, never sensed |

`cellStartDistance_` is re-anchored at every `beginDrive()`, so distance
error cannot compound across cells. Front-wall stops are absolute and
actively erase accumulated offset. Overshoot is carried into the next
cell so a long straight run does not drift.

### Dead-end K-turn

A 120 mm body pivoting about a rear axle cannot sweep a clean 180° inside
one cell. Dead ends use a multi-point turn: swing out on one wheel →
reverse while angled → straighten → two 90° turns → repeat → settle.
Always turns left. See DECISIONS.md #24.

---

## 7. Bench Tests

Tests live in `Core/App/tests/`, selected by a single `#define` at the top
of `main.cpp`. Enable exactly one motion test at a time.

```c
//#define RUN_TEST_I2C        // bus scan, ToF address assignment
//#define RUN_TEST_ENCODER
//#define RUN_TEST_GYRO
//#define RUN_TEST_TOF
//#define RUN_TEST_MOTOR
//#define P2_TEST_DRIVE       // straight-line distance + heading hold
//#define P2_TEST_PIVOT
```

### Live Expressions

C++ methods cannot be called from Live Expressions — watch members directly:

```
robot.encL_.distanceMm_
robot.encR_.distanceMm_
robot.gyro_.angleDeg_
robot.gyro_.rateDps_
robot.walls_.distMm_[0..4]
robot.navigator_.pose_.x
robot.navigator_.pose_.y
robot.navigator_.headingRef_
robot.ctrl_.pvW_
robot.modes_.state_
```

Set Number Format to **Float** for float members.

### Status LED

Fast blink = busy/calibrating · three flashes = calibration done ·
slow blink = idle awaiting button.

---

*BOTATO — Department of Computer Science & Engineering, University of Moratuwa*
*REV 2.0*
