# 🐭 BOTATO — Software Architecture Decisions

This document exists so any AI assistant or new contributor can get full
context on *why* the firmware is built the way it is, without re-deriving
every decision from scratch. Read this alongside `PINOUT.md` (hardware
config) before making any changes.

Each decision below states what was decided, why, and what alternatives
were considered and rejected.

---

## 1. Interrupt philosophy — the one-line-ISR rule

**Decision:** Every ISR in this codebase does exactly one thing: set a
`volatile` flag and return. No I2C calls, no floating-point math, no
long computation ever happens inside an interrupt handler. All real work
(I2C reads, PID, flood-fill) happens in the main loop, gated by checking
flags set by ISRs.

**Why:** This keeps every ISR's execution time under ~10ns regardless of
how many interrupts are configured simultaneously. It also means the
1kHz control loop timer can never be blocked or delayed by a sensor
interrupt doing slow work. This is the single most important
architectural rule in the project — violating it is the most common way
to introduce timing jitter or missed control loop ticks.

**Alternatives rejected:** Doing I2C reads directly inside EXTI ISRs —
rejected because I2C transactions can take 100µs+ and would block other
interrupts from firing during that window, and would make execution
timing non-deterministic.

---

## 2. ToF sensors — interrupt-driven data-ready, not polling

**Decision:** Each of the 5 ToF sensors gets its own dedicated GPIO1
data-ready interrupt (falling edge). The ISR sets a `tof_ready[i]` flag;
the main loop performs the actual I2C read when it sees the flag set.

**Why:** A VL53-series ToF measurement takes 20–50ms depending on ranging
mode — a variable, unpredictable duration. Polling would mean either
blocking the control loop for that whole window, or busy-polling an I2C
status register every tick (wasting bus bandwidth on 5 sensors that
mostly aren't ready). The GPIO1 interrupt tells us the instant a specific
sensor's result becomes available, at zero polling cost.

**Supporting research:** Verified this approach is genuinely used in
competitive micromouse designs — the RoboFest team's published firmware
breakdown and general practice confirm that variable-latency sensors are
handled via data-ready signaling wherever the sensor makes it available,
rather than fixed-schedule polling.

**Hardware detail:** Each ToF has its own dedicated EXTI line (0,1,2,3,4)
rather than sharing a line — see decision #6 for why this matters.

---

## 3. MPU6050 IMU — synchronous polling, NOT interrupt-driven

**Decision:** The MPU6050's INT pin is deliberately left unconnected.
The gyro is read synchronously inside the 1kHz TIM3 control loop tick
(specifically every 5th tick, giving a 5ms cadence) — never via its own
data-ready interrupt.

**Why:** This is the key distinction that took research to establish.
Unlike the ToF (unpredictable timing), the MPU6050 outputs data at a
fixed, configurable rate that you set via its DLPF register. Since our
control loop already runs at a known fixed rate, the IMU's INT pin would
fire at exactly the moment we were already going to read it anyway — it
carries zero additional information. Using it would add EXTI complexity
for no benefit.

**Supporting research:** Verified against KERISE v4 (a real competition
finalist's published firmware) — its IMU task runs as a periodic 1ms task
reading the ICM-20602 directly over SPI with no data-ready interrupt used
at all. This matches our approach exactly.

**The general rule this establishes:** Use a sensor interrupt only when
it carries *unpredictable* timing information (ToF ranging completion).
Skip it when the sensor's timing is already deterministic and matches
your control loop rate (IMU output data rate).

---

## 4. Encoders — hardware quadrature timers, zero software involvement

**Decision:** Both encoders are read via STM32 hardware timers configured
in Encoder Mode (TI1+TI2) — TIM5 for left, TIM2 for right. No interrupt
is used for encoder reading at all. The main loop reads `TIM2->CNT` and
`TIM5->CNT` directly whenever it needs a position/delta.

**Why:** This is not a case of "avoiding interrupts" — hardware quadrature
decoding is categorically superior to any interrupt-based approach. The
timer peripheral counts edges in hardware with zero CPU involvement, and
because both TIM2 and TIM5 are 32-bit timers on this MCU, there's no
16-bit overflow handling needed either.

**Why TIM2 and TIM5 specifically:** They are the only two 32-bit timers
available on the STM32F411 — using them for encoders means never having
to write overflow-handling code for the counters.

---

## 5. Motor PWM — hardware timer, zero software PWM

**Decision:** TIM1 generates motor PWM entirely in hardware (advanced
timer, most precise on this MCU). Speed control is a single register
write (`TIM1->CCR1` / `CCR2`) — no interrupt, no software PWM bit-banging.

**Why:** Same principle as encoders — hardware peripherals that can do
the job with zero CPU overhead should always be preferred over software
or interrupt-driven equivalents. PWM frequency is set to ~20kHz (above
human hearing, below N20 motor switching losses).

---

## 6. Buttons — polled, deliberately NOT using EXTI

**Decision:** Both mode buttons (PA5, PA6) are configured as plain
`GPIO_MODE_INPUT` with pull-up, polled in the main loop. No EXTI is
configured on these pins.

**Why — two compounding reasons:**
1. On the STM32F4, EXTI lines 5–9 all share one interrupt vector
   (`EXTI9_5_IRQHandler`). PA5=EXTI5 and PA6=EXTI6 would both land in that
   shared handler, requiring manual demuxing by checking the pending bit
   for each pin — extra complexity for no benefit on a signal that changes
   at human-reaction-time speed.
2. Buttons without hardware debounce (a 100nF cap to GND) bounce and can
   fire dozens of spurious interrupts per physical press. Polling with a
   simple debounce counter avoids this entirely.

**History:** The ToF RightFront and ToF Right sensors were originally
also being considered for PA5/PA6 (which would have created a genuine
EXTI9_5 sharing conflict with either the buttons or between the two ToF
sensors themselves). The resolution was to move those two ToF interrupt
lines to PB0 (EXTI0) and PB1 (EXTI1) instead — both fully dedicated,
unshared vectors — and keep PA5/PA6 exclusively for the polled buttons.
This is why the final pinout has all 5 ToF sensors on EXTI0–EXTI4 with
no shared handler required anywhere in the interrupt matrix.

---

## 7. Single master timer architecture

**Decision:** One hardware timer (TIM3, 1kHz) is the sole "heartbeat" for
all periodic, deterministic work: encoder reads, PID, gyro reads (every
5th tick), flood-fill steps, and PWM updates all happen inside the
`control_tick` flag check in the main loop, all derived from the same
TIM3 interrupt.

**Why:** Using one clock source for everything eliminates drift between
subsystems. Early integration of a teammate's gyro-heading code originally
used an independent DWT (cycle counter) based 5ms timer running in
parallel with TIM3 — this was consolidated into a simple tick-counter
(count 5 TIM3 ticks = 5ms) so there's exactly one deterministic clock
driving the whole robot, rather than two timing mechanisms that could
drift relative to each other. The DWT dependency was removed entirely.

**Research validation:** This matches the dominant pattern found across
competitive micromouse firmware — a single master control loop timer
orchestrating everything else synchronously, rather than scattered
independent interrupts racing each other.

---

## 8. Flood-fill — main loop only, hardware-independent

**Decision:** Flood-fill maze-solving logic runs exclusively in the main
loop, gated by the `control_tick` flag — never inside any ISR. Going
forward, `floodfill.c` should have zero HAL/hardware dependencies at all,
operating purely on a grid data structure.

**Why:** Keeping flood-fill hardware-free means it can be unit tested
against hand-built maze cases without any board attached — this was
identified as one of the highest-value moves for debuggability in this
project.

---

## 9. Corrected mistake — the AFIO remap was wrong

**Decision (correction):** No AFIO remap code should exist anywhere in
this project.

**What happened:** Early in this project, it was believed that PA15
(used for `TIM2_CH1`, right encoder) required an explicit JTAG-to-SWD
remap via `__HAL_RCC_AFIO_CLK_ENABLE()` and `__HAL_AFIO_REMAP_SWJ_NOJTAG()`
before `MX_TIM2_Init()`. This produced a hard build failure: **the
STM32F411 (F4 series) has no AFIO peripheral at all** — those macros are
STM32F1-series only and don't exist in the F4 HAL.

**Correct behavior:** Setting `Debug = Serial Wire` in CubeMX (which this
project already does, via PA13/PA14) automatically releases PA15 from its
JTDI function on the F4. No remap call of any kind is needed. If this
mistake resurfaces in old documentation or old code, remove it — it will
not compile and was never actually needed.

**Lesson for future AI assistants:** Don't carry forward F1-series
assumptions onto F4-series parts. Verify peripheral existence before
prescribing "mandatory" fixes.

---

## 10. PB6/PB7 — historical hardware conflict, now avoided entirely

**Decision:** PB6 and PB7 are not used anywhere in this project's pin
configuration.

**Why:** An earlier board schematic revision showed PB6/PB7 physically
wired to the shared I2C bus (as an apparent duplicate of the PB8/PB9 I2C
lines actually in use), while a draft firmware plan simultaneously wanted
to assign PB6/PB7 as USART1 TX/RX for debug output. Had both been
realized, the UART peripheral would have driven voltage onto the I2C
clock/data lines, corrupting every I2C transaction on the bus. The
resolution was to simply not use PB6/PB7 for anything in this project —
no USART, no GPIO, nothing. If a future board revision needs a debug
UART, use different pins and re-verify against the physical schematic
first, not just the `.ioc` file, since board revisions have drifted from
documentation before.

---

## 11. MPU6050 gyro integration — merged from teammate's contribution

**Decision:** A teammate's gyro-heading integration code was reviewed and
merged into the main control loop, with one architectural change (see
decision #7 — DWT timer replaced with TIM3 tick-counting).

**What the logic does:**
1. WHO_AM_I sanity check (register `0x75`, expect `0x68`/104)
2. Wake the sensor (register `0x6B` ← `0x00`, since it boots in sleep mode)
3. Set gyro full-scale range to ±1000°/s (register `0x1B` ← `0x10`,
   giving 32.8 LSB per °/s sensitivity)
4. Calibrate Z-axis offset: average 1000 samples over ~2 seconds at boot
   — **robot must be stationary during this window**
5. Every 5ms (5 TIM3 ticks): read Z-gyro, subtract calibration offset,
   convert to °/s, trapezoidal-integrate into a running `yaw` heading
   angle, wrapped to ±180°

**Tunable parameter:** `gyro_scale_factor` (default 1.0) — calibrate by
rotating the robot exactly 90° by hand and adjusting until `yaw` reads 90.

**Known unresolved issue at time of writing:** A debugging session found
`yaw` not updating in Live Expressions. Debug checklist established (in
priority order): check `who_am_i` reads 104, check `i2c_error_count` isn't
climbing, check `control_tick` is toggling, breakpoint at the `gyro_z =`
assignment line to see if `HAL_I2C_Mem_Read` is even succeeding, and
physically rotate the robot while watching `yaw` (it only accumulates
during actual rotation). **This may or may not still be relevant** — the
project was reset to a clean base afterward (see decision #13) and this
gyro logic was not yet re-integrated into the fresh codebase as of the
last update to this document.

---

## 12. Forward code architecture — modular, OOP-style C

**Decision:** Application logic should NOT live inline in `main.c`'s
USER CODE sections beyond a couple of orchestration calls. Instead, each
subsystem gets its own header/source pair under `Core/App/`:

```
Core/App/
├── motor.h / motor.c        — Motor_Init(), Motor_SetSpeed(), Motor_Stop()
├── encoder.h / encoder.c    — Encoder_Init(), Encoder_GetDelta()
├── tof.h / tof.c            — ToF_InitAll(), ToF_OnDataReady(), ToF_ProcessReady()
├── imu.h / imu.c            — IMU_Init(), IMU_Calibrate(), IMU_Update(), IMU_GetYaw()
├── button.h / button.c      — Button_Init(), Button_Update(), Button_IsPressed()
├── floodfill.h / floodfill.c — pure algorithm, zero hardware dependencies
└── app.h / app.c            — owns one instance of each module, exposes
                                 App_Init() and App_Run()
```

`main.c` should reduce to approximately:
```c
/* USER CODE BEGIN 2 */
App_Init();
/* USER CODE END 2 */

while (1) {
  /* USER CODE BEGIN WHILE */
  App_Run();
  /* USER CODE END WHILE */
}
```

**Why:** State lives in struct instances passed by pointer rather than
scattered globals — this makes each module independently reasoned-about
and testable. Hardware-touching code (HAL calls) stays inside the module
that owns that hardware; `app.c` only orchestrates *when* things happen.
ISR callbacks in `stm32f4xx_it.c` call exactly one function from `app.h`
(e.g. `App_OnToFReady(TOF_LEFT)`) — the one-line-ISR rule (decision #1)
still applies, just routed through the App layer instead of raw globals.

---

## 13. Project reset — starting from a verified-clean base

**Decision:** The original project accumulated inconsistent state across
multiple contributors: stale documentation (the AFIO mistake persisted in
README after being fixed in code), a teammate's gyro code that used a
different timing mechanism than the rest of the codebase, and a gap
between what documentation described as "already implemented" versus
what actually existed in `main.c` (which was found to still be an empty
CubeMX-generated stub). Rather than trying to reconcile all of this, the
decision was made to restart from scratch using a freshly verified `.ioc`
file as the seed, with the new modular architecture (decision #12) applied
from the very beginning rather than retrofitted.

**Process:** New empty project folder → clean `.ioc` loaded into CubeMX
standalone (pin config verified against `PINOUT.md`) → code generated
fresh → imported into CubeIDE → build verified to produce 0 errors before
any application logic is added.

---

## 14. Git workflow policy

**Decision:** Any AI coding assistant (Claude Code or otherwise) working
on this repository must NEVER run `git add`, `git commit`, `git push`, or
create branches on its own. All staging, committing, and pushing is done
manually by the project owner. AI assistants should edit the working tree
and clearly summarize what changed, then stop — never assume permission
to finalize changes into version control.

**Why:** The project owner wants full control and visibility over what
enters the commit history, and wants to review every change before it's
staged, as part of learning the codebase deeply rather than accepting
AI-driven commits wholesale.

---

## Quick reference — decision summary table

| # | Area | Decision | Interrupt used? |
|---|---|---|---|
| 2 | ToF sensors | Data-ready EXTI, dedicated line per sensor | Yes — flag only |
| 3 | MPU6050 IMU | Synchronous poll inside TIM3 tick | No |
| 4 | Encoders | Hardware quadrature timer | No (not even needed) |
| 5 | Motor PWM | Hardware timer PWM | No |
| 6 | Buttons | Polled in main loop | No |
| 7 | Control loop | Single master TIM3 @ 1kHz | Yes — flag only |
| 8 | Flood-fill | Main loop, hardware-independent | No |

---

*BOTATO — Department of Computer Science & Engineering, University of Moratuwa*
*This document should be updated whenever a new architectural decision is
made or an old one is revised — keep it as the single source of truth for
project reasoning.*

---

## 15. Motor direction polarity — inverted from TB6612 textbook mapping

**Decision:** Motor direction logic uses `IN1=LOW, IN2=HIGH` for physical
forward, and `IN1=HIGH, IN2=LOW` for physical reverse — the OPPOSITE of
the textbook TB6612FNG truth table (which states `IN1=HIGH,IN2=LOW` =
forward).

**Why:** Confirmed via physical motor spin test — both the left and
right motor spun backward when driven with the textbook mapping. Since
both motors showed the identical inverted behavior, this is not a
per-motor wiring fault; it reflects how this board's motor leads are
physically connected to the TB6612's M1/M2 outputs. No rewiring was
done or is needed — this is purely a firmware-level polarity convention.

**Action required:** Any future motor driver code (the real `motor.c`
module under `Core/App/`) MUST use this inverted mapping, not the
textbook one. If direction ever seems backward again after further
hardware changes (e.g. replacing a motor), re-verify with the
`Motor_Spin_Test` module before assuming the polarity constant needs to
change again — it may instead indicate a newly reversed motor lead on
just the replaced unit, which should be handled per-motor rather than
by flipping the global convention.

**Reference implementation:** See `Core/App/tests/Motor_spin_test/motor_test.c`,
function `SetMotorDuty()`, for the exact working polarity.

---

## 16. Split I2C: ToF sensors on I2C1, IMU on a dedicated I2C2

**Decision:** The MPU6050 was moved off the shared bus onto its own I2C2
peripheral. Final configuration:

| Bus | SCL | SDA | Devices |
|---|---|---|---|
| I2C1 | PB6 | PB7 | 5× ToF sensors |
| I2C2 | PB10 | PB9 (AF9) | MPU6050 only |

XSHUT_RIGHT moved from PB10 to PB8 to free PB10 for I2C2_SCL.

**Why — the root cause:** every VL53L0X breakout board carries its own
10 kΩ pull-up resistors on SCL and SDA. Five boards in parallel gave
roughly 2 kΩ; measured on the assembled bus it was ~1.5 kΩ. At that
resistance a device pulling the line low must sink well over the 3 mA
the I2C specification allows. The MPU6050 could not pull the line low
enough to register a valid logic 0, so its address was never
acknowledged — `imu_fail_step = 1`, failing at the very first
`HAL_I2C_IsDeviceReady()` call.

This is called **excessive I2C bus loading** (or the *parallel pull-up
problem*). The failure mode is a **V_OL violation** — the device cannot
meet its output-low voltage against excessive pull-up current.

**Why it appeared intermittent:** the bus sat right at the threshold.
Temperature, which sensors happened to be connected, supply variation,
and part tolerance each shifted the balance slightly, so the same setup
worked on some power-ups and not others. Empirically the IMU worked
above roughly 2.5 kΩ and failed below it.

**Debugging path that led here (worth not repeating):** the symptom was
first blamed on a defective ToF module, then on interrupted debug
sessions wedging the bus, then on EXTI storms starving SysTick and
corrupting `HAL_GetTick()` timeouts. Fixes were attempted for each —
XSHUT masking, EXTI masking during IMU init, bus-recovery retry loops.
None worked, because none addressed the actual DC loading problem.

**Alternatives rejected:**
- *Remove pull-ups from four ToF boards* — they are part of an integrated
  resistor network that also serves XSHUT and GPIO1; removing it would
  break those functions.
- *TCA9548A I2C multiplexer* — would work and would also eliminate XSHUT
  sequencing entirely, but adds a module, wiring, and per-read channel
  switching latency.
- *Software (bit-banged) I2C on PB6/PB7* — no peripheral conflicts, but
  rejected in favour of a real hardware peripheral.
- *I2C2 on PB10/PB3* — PB3 is the right encoder's TIM2_CH2. Would have
  forced the right encoder onto 16-bit TIM4, losing the no-overflow
  guarantee from decision #4.
- *I2C3 on PA8* — PA8 is left motor PWM (TIM1_CH1), which has no
  alternate pin on this package.

**The AF9 trap:** PB9 maps to I2C1_SDA at AF4 but I2C2_SDA at **AF9**.
Some STM32CubeMX versions generate the wrong AF value when I2C pins are
moved off their defaults (fixed in CubeMX 6.7.0). If PB9 is set to AF4
under I2C2, the pin stays internally attached to I2C1 and the bus fails
silently with no error. Always verify `GPIO_AF9_I2C2` in
`HAL_I2C_MspInit` in `Core/Src/i2c.c` after regenerating.

**Consequence for future work:** if another I2C sensor is ever added,
check the combined pull-up resistance on that bus before assuming it
will work. This failure cost significant debugging time and is easy to
reintroduce.

---

## 17. Test suite structure — one module per subsystem

**Decision:** All hardware test code lives in a flat `Core/App/tests/`
directory, one `.c`/`.h` pair per subsystem, selected by a `#define`
switch at the top of `main.c`.

```
Core/App/
├── tof.c / tof.h              <- production VL53L0X driver
└── tests/
    ├── test_i2c.c   / .h      <- bus recovery + scan, both buses
    ├── test_tof.c   / .h      <- addressing, coexistence, distance
    ├── test_imu.c   / .h      <- MPU6050 presence, yaw
    ├── test_motor.c / .h      <- both motors, PWM + direction
    └── test_encoder.c / .h    <- both encoders, counts + delta
```

Select a test by uncommenting one line in `main.c`:
```c
//#define RUN_TEST_I2C
//#define RUN_TEST_TOF
//#define RUN_TEST_IMU
//#define RUN_TEST_MOTOR
#define RUN_TEST_ENCODER
```

**Why flat rather than a folder per test:** each subfolder needs its own
include path added manually in CubeIDE, which becomes tedious friction
every time a test is added. One folder, one include path.

**Why unselected tests cost nothing:** the build uses
`-ffunction-sections -fdata-sections` with `--gc-sections` at link time,
so functions nothing references are discarded. Measured with all five
tests compiled in: 13072 bytes flash (2.5% of 512 KB), 1980 bytes BSS
(1.5% of 128 KB).

**Bus recovery is shared:** `I2C1_BusRecover()` and `I2C2_BusRecover()`
live in `test_i2c.c` and are called by the other test modules rather
than duplicated. They bit-bang up to 9 SCL pulses to free a device
holding SDA low after an interrupted debug session — a reflash alone
does not clear this, since it doesn't reset external peripherals.

---

## 18. Encoder direction — right encoder negated in software

**Decision:** The right encoder's count and delta are negated in
firmware so both wheels report positive values when the robot moves
forward.

**Why:** the two encoders are physically mirrored on the chassis, so the
wheels turn in opposite directions during forward motion. Combined with
how each encoder's A/B phases are wired, the right encoder counts down
where the left counts up.

**Implementation** (in `test_encoder.c`, carry into the production
`encoder.c`):
```c
enc_right_count = -(right - (int32_t)ENC_START_VALUE);
enc_right_delta = -(right - prev_right);
```

Both must be negated together, or count and delta disagree on direction.

**Also note:** both encoder counters start at `0x80000000` (mid-range)
rather than 0, so turning backwards reads as a small negative number
instead of wrapping to ~4 billion. The displayed count subtracts this
offset so it reads from 0.

**Alternative not taken:** swapping TIM2_CH1/CH2 in CubeMX, or swapping
the two physical phase wires. Either would fix it at the source, but the
software negation avoids disturbing a verified-working `.ioc`.

---

## 19. Hardware verification status

As of the last test session:

| Subsystem | Status |
|---|---|
| I2C1 bus + 5× ToF | ✅ Verified — addressing, coexistence, live distance |
| I2C2 bus + MPU6050 | ✅ Verified working when isolated from ToF loading |
| Encoders (both) | ✅ Verified — counts, direction, delta |
| Motors (both) | ✅ Verified — both directions, polarity corrected |
| Buttons | ❌ Not yet tested |
| Combined operation | ❌ Not yet tested with motors running |

**Still untested and worth flagging:** all sensor verification was done
with motors stopped. PWM switching noise on a shared ground plane is a
known cause of I2C and encoder glitches, and no motor decoupling
capacitor is fitted (see decision #5 discussion). The next honest
checkpoint is re-running the sensor tests *while motors are spinning*.
