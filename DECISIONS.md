# 🐭 BOTATO — Software Architecture Decisions

This document exists so any AI assistant or new contributor can get full
context on *why* the firmware is built the way it is, without re-deriving
every decision from scratch. Read this alongside `PINOUT.md` (hardware
config) before making any changes.

Each decision below states what was decided, why, and what alternatives
were considered and rejected. Decisions that were later **revised** say so
explicitly rather than being silently rewritten — the history is useful.

---

## 1. Interrupt philosophy — the one-line-ISR rule

**Decision:** No slow or unbounded work ever happens inside an interrupt
handler. I2C transactions, flood-fill, and maze decisions all live in the
main loop.

**Revision (still compatible):** the TIM3 control ISR now does real work —
it updates the gyro, both encoders, and runs the control loop. This is a
deliberate, bounded exception: that work is fixed-duration, uses only
hard-FPU float math and one short I2C2 read, and *is* the deterministic
task the timer exists for. The rule it still enforces is the important
one: **nothing with unpredictable duration runs in an ISR.** The five ToF
sensors — the slow, variable-latency devices — stay in the main loop.

**Why:** the 1 kHz control loop's PID math assumes an exact fixed `dt`.
Anything that can stretch a tick introduces timing jitter.

**Alternatives rejected:** doing ToF I2C reads inside EXTI ISRs — a VL53L0X
transaction can take 100 µs+ and would block the control tick.

---

## 2. ToF sensors — REVISED: continuous mode, polled from the main loop

**Original decision:** each ToF sensor got a dedicated GPIO1 data-ready
EXTI line; the ISR set a flag and the main loop did the I2C read.

**Revised decision:** the sensors run in **continuous ranging mode** and
are polled non-blockingly by `WallSensorArray::poll()` from the main loop.
**No EXTI is configured on the GPIO1 pins.** They remain physically wired
but unused.

**Why the change:** in continuous mode the sensor free-runs at a known
~30 Hz and simply holds its latest result in a register. The data-ready
signal therefore stops carrying information the main loop needs — polling
at main-loop rate always finds either a fresh sample or the previous one,
which is exactly what the navigator wants. Removing five EXTI lines also
removed five interrupt vectors, the flag-array plumbing, and a whole class
of "which sensor fired?" race conditions, at no measured cost.

This makes the ToF path consistent with decision #3's general rule: use a
sensor interrupt only when it carries *unpredictable* timing information.
Once the sensor free-runs, it no longer does.

**Consequence:** if the `.ioc` is ever regenerated, make sure EXTI0–EXTI4
stay disabled.

---

## 3. MPU6050 IMU — synchronous polling, NOT interrupt-driven

**Decision:** the MPU6050's INT pin is deliberately left unconnected. The
gyro is read synchronously inside the TIM3 control tick.

**Revision:** it is now read on **every** tick (1 kHz), not every 5th tick
as originally specified. The DWT-based 5 ms timer and the tick-counting
scheme that replaced it are both gone.

**Why:** the rate feeds the inner loop of the turn cascade, which runs at
1 kHz. Feeding a 200 Hz measurement into a 1 kHz loop added quantisation
the rate PID could see. A single MPU6050 register read on a dedicated,
lightly loaded I2C2 bus is fast enough to fit in the tick.

**Why still no interrupt:** unlike the ToF, the MPU6050 outputs at a fixed,
configurable rate. Its INT would fire at exactly the moment we were already
going to read it — zero additional information, extra EXTI complexity.

**Supporting research:** matches KERISE v4 (a competition finalist's
published firmware), whose IMU task is a periodic 1 ms task with no
data-ready interrupt at all.

---

## 4. Encoders — hardware quadrature timers, zero software involvement

**Decision:** both encoders use STM32 hardware Encoder Mode (TI1+TI2) —
TIM5 left, TIM2 right. No interrupt at all.

**Why:** the timer peripheral counts edges in hardware with zero CPU
involvement. TIM2 and TIM5 are the only two 32-bit timers on the F411, so
using them for encoders means never writing overflow-handling code.

---

## 5. Motor PWM — hardware timer, zero software PWM

**Decision:** TIM1 generates motor PWM entirely in hardware. Speed control
is a single register write. 20 kHz — above hearing, below N20 switching
losses.

---

## 6. Buttons — polled, deliberately NOT using EXTI

**Decision:** both mode buttons (PA5, PA6) are plain `GPIO_MODE_INPUT` with
pull-up, polled and debounced in the main loop.

**Why — two compounding reasons:**
1. EXTI lines 5–9 share one vector (`EXTI9_5_IRQHandler`). PA5=EXTI5 and
   PA6=EXTI6 would both land there, needing manual demuxing — complexity
   for no benefit on a human-speed signal.
2. Buttons without hardware debounce fire dozens of spurious interrupts
   per press. Polling with a debounce counter avoids this entirely.

**History:** ToF RightFront and Right were originally considered for
PA5/PA6, which would have created a real EXTI9_5 conflict. They were moved
to PB0/PB1 instead. (That EXTI assignment is itself now obsolete — see #2.)

---

## 7. Single master timer architecture

**Decision:** TIM3 at 1 kHz is the sole heartbeat for all periodic
deterministic work — gyro, encoders, PID, PWM.

**Why:** one clock source eliminates drift between subsystems. An early
teammate contribution used an independent DWT cycle-counter timer running
in parallel; this was consolidated into TIM3 so exactly one deterministic
clock drives the robot. The DWT dependency was removed entirely.

**Note:** maze decisions (flood-fill, navigation) deliberately do *not* run
on this clock — they run at main-loop rate, which is variable and fine.

---

## 8. Flood-fill — main loop only, hardware-independent

**Decision:** flood-fill runs exclusively in the main loop, never in an ISR.
`FloodFill`, `MazeMap` and `Planner` have **zero HAL/hardware dependencies**.

**Why:** keeping them hardware-free means they can be reasoned about and
tested against hand-built maze cases with no board attached.

---

## 9. Corrected mistake — the AFIO remap was wrong

**Decision (correction):** no AFIO remap code should exist anywhere.

**What happened:** it was believed PA15 (TIM2_CH1) needed
`__HAL_RCC_AFIO_CLK_ENABLE()` / `__HAL_AFIO_REMAP_SWJ_NOJTAG()`. This hard
build-failed: **the STM32F411 has no AFIO peripheral** — those macros are
F1-only.

**Correct behavior:** `Debug = Serial Wire` in CubeMX already releases PA15
from JTDI on the F4.

**Lesson:** don't carry F1-series assumptions onto F4 parts.

---

## 10. PB6/PB7 — historical conflict, RESOLVED

**Original decision:** PB6/PB7 unused, because an old schematic wired them
to I2C while a draft firmware plan wanted them as USART1 — which would have
driven UART voltage onto I2C lines.

**Current status:** PB6/PB7 now carry **I2C1 for the five ToF sensors**
(see #16). There is no debug UART in this project. If one is ever added,
use different pins and verify against the physical board, not the `.ioc`.

---

## 11. MPU6050 gyro integration — merged, now working

**Decision:** a teammate's gyro-heading code was reviewed and merged, with
the DWT timer replaced by TIM3 (#7).

**What the logic does:**
1. WHO_AM_I sanity check (`0x75`, expect 104)
2. Wake the sensor (`0x6B` ← `0x00`)
3. Gyro full-scale ±1000 °/s (`0x1B` ← `0x10`, 32.8 LSB per °/s)
4. Calibrate Z-axis offset at boot — **robot must be stationary**
5. Every tick: read Z-gyro, subtract offset, convert to °/s, integrate

**Status update:** the "yaw not updating" issue noted in earlier revisions
is resolved. The gyro is integrated, verified, and is now the sole heading
authority (see #21). The bias offset is measured once at `calibrate()` —
at boot or on a long-press — and is **not** refreshed by `zeroAngle()` at
run start. MPU6050 bias drifts with temperature, so recalibrate warm,
immediately before a run, rather than relying on a cold boot calibration.

---

## 12. REVISED: application layer is C++, not OOP-style C

**Original decision:** modular "OOP-style C" — one `.c`/`.h` pair per
subsystem with structs passed by pointer.

**Revised decision:** the application layer is **C++17**. The vendor HAL
and the VL53L0X driver (`tof.c`) stay C. `main.cpp` is the only renamed
file; all other CubeMX-generated files remain `.c`.

**Build flags:** `-std=gnu++17 -fno-exceptions -fno-rtti
-fno-use-cxa-atexit`, hard FPU.

**What this means in practice** — C++ is used for *organisation*, not
runtime features:

| Banned | Consequence |
|---|---|
| Exceptions | Failures signalled by return value / `bool ok` |
| RTTI | No `dynamic_cast`, no `typeid`; types stay concrete |
| Dynamic allocation | No `new`, no `std::vector`/`std::string`; everything fixed-size |

RAM footprint is therefore essentially identical to the C equivalent, with
no hidden runtime machinery that could stall the 1 kHz tick.

**C/C++ boundary rules:**
- HAL headers are included inside `extern "C" { }`
- **Never forward-declare HAL types** (`struct TIM_HandleTypeDef;`) — they
  are typedefs; include the header instead
- `tof.h` carries a `__cplusplus` guard

**Structure:** a `Robot` facade owns every subsystem and exposes two entry
points (`onControlTick`, `onMainLoop`). Subsystems receive their
collaborators through an `init()` call rather than constructing them —
this is how a no-allocation, single-global-instance system wires itself up
without depending on static constructor order.

---

## 13. Project reset — starting from a verified-clean base

**Decision:** the original project accumulated inconsistent state across
contributors — stale docs, mismatched timing mechanisms, and a gap between
documented and actual `main.c`. It was restarted from a freshly verified
`.ioc` with the modular architecture applied from the beginning.

---

## 14. Git workflow policy

**Decision:** any AI coding assistant working on this repository must NEVER
run `git add`, `git commit`, `git push`, or create branches. All staging,
committing and pushing is done manually by the project owner. Edit the
working tree, summarize what changed, then stop.

**Why:** the owner wants full visibility over commit history as part of
learning the codebase deeply.

---

## 15. Motor direction polarity — inverted from TB6612 textbook mapping

**Decision:** direction pin polarity is inverted relative to the datasheet's
textbook mapping, handled inside `Motor`. Nothing above the motor layer
needs to know.

---

## 16. Split I2C: ToF sensors on I2C1, IMU on a dedicated I2C2

**Decision:** five ToF sensors on I2C1 (PB6/PB7); MPU6050 alone on I2C2
(PB10/PB9). Both Fast Mode 400 kHz.

**Why:** each ToF breakout carries its own 10 kΩ pull-up. Five in parallel
gave ~2 kΩ, and combined with the MPU6050 board's pull-ups the shared bus
fell to ~1.5 kΩ — below what the MPU6050 could pull low, violating V_OL.
The IMU failed silently while the ToF sensors worked fine. This cost
significant debugging time.

**Alternatives rejected:**
- *Software I2C* — rejected in favour of a real hardware peripheral.
- *I2C2 on PB10/PB3* — PB3 is the right encoder's TIM2_CH2. Would have
  forced the right encoder onto 16-bit TIM4, losing the no-overflow
  guarantee from #4.
- *I2C3 on PA8* — PA8 is left motor PWM, no alternate pin on this package.

**The AF9 trap:** PB9 maps to I2C1_SDA at AF4 but I2C2_SDA at **AF9**. If
PB9 is set to AF4 under I2C2, the pin stays attached to I2C1 and the bus
fails silently. Always verify `GPIO_AF9_I2C2` in `HAL_I2C_MspInit`.

**Consequence for future work:** check combined pull-up resistance before
adding any I2C device.

---

## 17. REVISED: test suite structure

**Decision:** all hardware test code lives in a flat `Core/App/tests/`
directory, selected by a `#define` at the top of `main.cpp`. Files are now
`.cpp`/`.hpp`.

```
Core/App/tests/
├── Tests.cpp / .hpp         <- encoder, gyro, ToF, motor, drive, modes
├── Phase2Test.cpp / .hpp    <- pivot turns, one-cell drive, heading hold
└── Tests_I2C.cpp / .hpp     <- bus scan + recovery, ToF XSHUT addressing
```

**Why flat rather than a folder per test:** each subfolder needs its own
include path added manually in CubeIDE — friction every time a test is added.

**Why unselected tests cost nothing:** `-ffunction-sections
-fdata-sections` with `--gc-sections` discards unreferenced functions.

**Bus recovery is shared:** bit-bangs up to 9 SCL pulses to free a device
holding SDA low after an interrupted debug session — a reflash alone does
not clear this, since it doesn't reset external peripherals.

**Live Expressions cannot call C++ methods.** Watch member variables
directly (`robot.gyro_.angleDeg_`, `robot.walls_.distMm_[0..4]`, …) and set
Number Format to Float.

---

## 18. Encoder direction — right encoder negated in software

**Decision:** the right encoder's count and delta are negated in firmware so
both wheels report positive when moving forward.

**Why:** the encoders are physically mirrored on the chassis. Combined with
phase wiring, the right counts down where the left counts up. Both count
and delta must be negated together, or they disagree on direction.

**Alternative not taken:** swapping TIM2_CH1/CH2 in CubeMX or swapping the
phase wires — either fixes it at source but disturbs a verified `.ioc`.

---

## 19. Hardware verification status

| Subsystem | Status |
|---|---|
| I2C1 bus + 5× ToF | ✅ Addressing, coexistence, live distance |
| I2C2 bus + MPU6050 | ✅ Verified once isolated from ToF loading |
| Encoders (both) | ✅ Counts, direction, delta |
| Motors (both) | ✅ Both directions, polarity corrected |
| Buttons | ✅ Short/long press, mode selection |
| Heading hold + pivot turns | ✅ |
| One-cell drive | ✅ |
| Wall sensing → map | ✅ |
| Dead-end K-turn | ✅ Working on test maze |
| Full 16×16 search run | 🔄 In progress |
| Speed run | ❌ Not implemented |

**Still worth flagging:** early sensor verification was done with motors
stopped, and **no motor decoupling capacitor is fitted across VMOT**. PWM
switching noise on the shared ground is a known cause of I2C and encoder
glitches. If sensor faults appear only while driving, suspect this before
firmware.

---

## 20. Layer boundaries — decisions go up, hardware stays down

**Decision:** higher layers make decisions, lower layers touch hardware, and
the dependency only ever points downward.

```
Robot (facade)
 ├── ModeController / Navigator      ← state & policy
 ├── FloodFill / MazeMap / Planner   ← pure algorithm, no HAL
 ├── ControlLoop / PIDController     ← control
 ├── Gyro / Encoder / Motor / Drive / WallSensorArray  ← HW abstraction
 └── STM32 HAL + tof.c               ← vendor C
```

**The test:** if you find yourself calling HAL from `Navigator`, or making
maze decisions inside `Motor`, something belongs elsewhere.

**Concretely:** `Navigator` never touches PWM. It speaks intentions —
`setTargets(speedX, speedW)`, `holdHeading(deg)`,
`driveStraight(speedX, deg)` — and `ControlLoop` decides how.

---

## 21. Heading conventions — the four rules that were learned the hard way

Each of these was a real bug. Do not change them casually.

**1. Anticlockwise-positive.** `TURN_LEFT = +90`, `TURN_RIGHT = −90`. The
convention is applied in exactly one place, `Navigator::beginTurn()`.

**2. `Planner::turnFor()` stays purely geometric.** The `Dir` enum is
clockwise (N=0, E=1, S=2, W=3), so N→E is always `TURN_RIGHT` —
*independent* of the gyro's sign convention. Mixing the two broke turn
decisions.

**3. The gyro angle is continuous and unbounded.** The ±180 wrap was removed
from `Gyro::update()`; wrapping happens only on the *error* inside
`ControlLoop::tick()`. Wrapping the angle itself created a false 180°
attractor where the robot oscillated at the opposite heading.

**4. Turn targets are relative** — `headingRef_ += ±90`, never absolute
compass bearings. Absolute targets against an unbounded gyro caused
infinite spinning.

---

## 22. Never command a single 180° turn

**Decision:** any reversal is executed as **two successive 90° turns**, never
one 180° command.

**Why:** the heading error is wrapped into ±180. A 180° command lands
exactly on that boundary, where the sign is decided by noise:

```
gyro at −0.5° → err = 180.5 → wraps to −179.5 → turns CLOCKWISE
gyro at +0.5° → err = 179.5 → stays  +179.5  → turns ANTICLOCKWISE
```

Half a degree of noise flipped the direction of the turn. In a dead end
this swung the robot into the wall it had just made room away from. Two
90° steps sit far from the boundary and are always unambiguous. The swept
circle is identical — this changes determinism, not clearance.

---

## 23. Cascaded control, and one authority per axis

**Decision:** turning is a cascade — heading error → `pidHeading` → rate
target → `pidW` → turn PWM. Forward motion is a single `pidX` loop. The two
abstract outputs are mixed as `L = fwd − turn`, `R = fwd + turn`.

**Guards:** NaN guards on every process variable, `MAX_RATE_DPS` clamp on the
cascade (so the outer loop can't demand an impossible spin), turn timeout,
and integral anti-windup clamps in `PIDController`.

**One authority per axis:** heading is owned by the gyro alone; forward
position by the front ToF when available and the encoders otherwise; lateral
position by the wall sensors. These never vote on the same question, which
is why no conflict-resolution logic exists anywhere.

**Tuned values that work:**
```cpp
pidX_{6.0f, 0.15f, 0.0f, 400.0f}       // forward (Ki needed to hit target)
pidW_{4.0f, 0.0f,  2.0f, 400.0f}       // rate — these values fixed the jiggle
pidHeading_{4.0f, 0.0f, 0.5f, 200.0f}  // heading outer loop
```

---

## 24. Dead ends use a multi-point (K) turn, not an in-place pivot

**Decision:** a `TURN_AROUND` routes to `beginDeadEndTurn()`, a sub-state
machine: swing out on one wheel → reverse while angled → straighten →
two 90° turns → repeat → settle back into the cell.

**Why:** the body is 120 mm long with the axle 25 mm from the back, so the
nose extends ~95 mm ahead of the pivot point. An in-place rotation sweeps a
circle wider than the corridor and clips the walls.

**The geometry that matters:** the angled reverse contributes roughly **4.5×
more backward displacement than lateral** (≈77 mm back vs ≈17 mm sideways at
20° / 50 mm). Getting away from the dead-end wall is what actually prevents
the collision — lateral offset alone cannot, because a circle needs its full
diameter wherever you centre it.

**Single-wheel swings** are produced through the existing cascade rather than
raw PWM: the mixer gives `v_left = v − ωW/2`, so choosing `v = ±ωW/2` parks
one wheel and swings the body about it.

**Always turns left.** A sensor-driven side choice was tried and removed — it
swapped sides without regard to where the robot actually was and swung into
walls. Fixed-left is predictable and easier to tune. The cost: a systematic
bias now repeats in the same direction every dead end, so tune
`DE_REVERSE_MM` / `DE_SETTLE_MM` rather than reintroducing alternation.

---

## 25. Cell completion — absolute beats relative

**Decision:** a drive ends on the **front ToF** when a wall is in range, and
falls back to encoder distance otherwise.

**Why:** encoders measure *relative* travel and accumulate slip with no way
to detect it. The front ToF measures *absolute* distance to a real object and
doesn't care what happened earlier in the run. Every front wall is a chance
to erase accumulated error.

**Supporting mechanisms:**
- `cellStartDistance_` is re-anchored at every `beginDrive()`, so distance
  error cannot compound across cells.
- Overshoot is **carried** into the next cell rather than discarded, so a long
  straight run doesn't drift cell by cell.
- `approachSlowdown()` tapers speed near a front wall, because the ToF's
  ~30 Hz update plus median filter lags real position at speed.
- A post-turn drive adds a small rear-axle offset (`TURN_ADVANCE_MM`), applied
  **only** in the encoder branch and **only** after a turn — applying it to
  every cell would compound down a corridor.

**Known fragility:** the grid pose is pure dead reckoning — `stepForward()`
fires unconditionally when a drive completes. Nothing verifies the pose
against the world, so a badly-placed stop silently corrupts the map. The
eventual fix is side-wall edge detection for mid-corridor re-sync.

---

## 26. Motor speed ceiling — targets must stay under ~450 mm/s

**Decision:** search and speed targets must stay well below the drivetrain's
physical maximum (200 rpm × 43 mm wheel ≈ 450 mm/s, less under load).

**Why this is an architectural note and not just tuning:** an unreachable
target leaves `pidX` with a permanent error, so `fwd` pins at `PWM_MAX` and
the integral sits at its clamp. With `fwd` saturated, the differential mix
clips:

```
left  = 4999 − turn → survives
right = 4999 + turn → clamped to 4999 → correction lost
```

Steering authority becomes **asymmetric** — the robot can correct one way but
not the other. This presents as weak turns, mushy heading correction and
one-sided drift, and it is easily misdiagnosed as a gyro or PID problem.

---

## 27. Sensor validity is a first-class signal

**Decision:** `WallSensorArray::ok(i)` means *this reading is fresh and in
range*, not *the sensor booted successfully*. Out-of-range reports a far
sentinel (`TOF_NO_TARGET_MM`), never 0, and readings expire after
`TOF_STALE_MS`.

**Why:** the original code only wrote `distMm_[i]` on `HAL_OK` and kept a
separate boot-time `ok_` flag. In a long corridor the front sensor sees
nothing, so its distance was never updated — it held either its `{0,...}`
initialiser or the last small value from a wall just passed, while `ok_`
still said "trust me." `driveComplete()` then read 0, compared it against
`FRONT_STOP_MM`, and ended the cell after 10 mm of travel.

**The general rule:** a sensor abstraction must distinguish *no reading*
from *a reading of zero*. Zero is a legitimate value for a distance sensor
and must never be the fallback for "unknown".

**Related trap:** `CAL_B[3]` (RightFront) is ≈ −71, correcting a mounting
angle. Any raw reading under ~74 mm calibrates negative and clamps to 0, so
that sensor reports 0 when *close* to a wall. Linear calibration is the wrong
model for it at short range; re-fit before relying on the diagonals.

---

## 28. Lateral centring trims the heading setpoint, not the rate

**Decision:** wall-based centring feeds a small **heading offset** into
`driveStraight()`, rather than adding a competing term to the rate target.

**Why:** injecting a wall correction into the rate target makes it fight the
heading loop. Re-centring *requires* the nose to lean off `headingRef_`
temporarily; heading-hold sees that necessary lean as an error and cancels
it. The two settle at a compromise where the lateral error is non-zero — the
robot runs parallel to the centreline but permanently offset. That is a
structural limitation, not a gain problem, and raising Kp only makes it
oscillate.

Trimming the heading *setpoint* leaves one heading authority. Lateral
position becomes a single integrator driven by heading offset, so
proportional control converges to zero error.

**Gating (all conditions must hold):** post gate away from cell boundaries;
both side readings fresh and near; the **width invariant** `left + right ≈
corridor width` — which holds regardless of lateral offset and so cleanly
rejects stubs, openings and junctions. Single-wall following holds half the
corridor width from whichever wall is visible, at reduced confidence.

**`WC_CENTER_TRIM_MM` must be measured.** Any left/right sensor asymmetry is
a constant error that steers the robot continuously; with the integral term
active it is *integrated* into a growing lean. Place the robot physically
centred with a ruler and set the trim to the measured `left − right`.

**History:** an earlier version injected into the rate target, was disabled
entirely while chasing a heading drift, then reinstated in this form.

---

## 29. The goal is a region, defined as a bounding box

**Decision:** the destination is an inclusive bounding box
(`GOAL_X_MIN/MAX`, `GOAL_Y_MIN/MAX`), not a hardcoded cell list. Any cell
inside counts as reached.

**Why:** the rules give the destination block one entrance, so driving to a
geometric centre wastes time. Stopping at the first mouth entered is both
faster and simpler — and the 180° back out is emitted automatically by
`Planner::turnFor()` on the return leg, with no special case anywhere.

**Retargeting a maze is four numbers in one file.** `isGoal()` is a range
test; `FloodFill::recompute()` builds its seed list from the same bounds.

**Coordinate trap:** `y = 0` is the **bottom** row. The centre 2×2 of a 16×16
is 7–8 on both axes, which is symmetric and therefore immune to a top/bottom
mix-up — but `START_X/START_Y` is **not**. Reading a start corner off a photo
without accounting for the flip puts the whole map at the wrong offset.

**Sizing note:** `FloodFill`'s BFS queue is `MAX_DIM × MAX_DIM`. At 16×16 the
algorithm pushes each cell exactly once, filling it to exactly 100% with no
bounds check. It fits today with zero margin — add headroom and a guard
before changing the push condition or the goal size.

---

## Quick reference — decision summary table

| # | Area | Decision | Interrupt used? |
|---|---|---|---|
| 2 | ToF sensors | Continuous mode, polled in main loop | **No** (revised) |
| 3 | MPU6050 IMU | Synchronous read every TIM3 tick | No |
| 4 | Encoders | Hardware quadrature timer | No (not even needed) |
| 5 | Motor PWM | Hardware timer PWM | No |
| 6 | Buttons | Polled in main loop | No |
| 7 | Control loop | Single master TIM3 @ 1 kHz | Yes |
| 8 | Flood-fill | Main loop, hardware-independent | No |
| 12 | Application layer | C++17, no exceptions/RTTI/allocation | — |
| 21 | Heading | Gyro sole authority, relative targets | — |
| 22 | Reversals | Two 90° turns, never one 180° | — |
| 25 | Cell completion | Front ToF absolute, encoder fallback | — |
| 28 | Lateral centring | Trims heading setpoint, not rate | — |
| 29 | Goal | Inclusive bounding box | — |

---

*BOTATO — Department of Computer Science & Engineering, University of Moratuwa*
*This document should be updated whenever a new architectural decision is
made or an old one is revised — keep it as the single source of truth for
project reasoning. Mark revisions explicitly rather than rewriting history.*
