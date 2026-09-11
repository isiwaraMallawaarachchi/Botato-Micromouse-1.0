# BOTATO Firmware — Codebase Guide

*A complete walkthrough of the software: how it's organised, why it's built this
way, and how the whole system runs as a flow. Written for anyone with basic C++
knowledge. Hardware is mentioned only where it explains a software choice.*

---

## 1. What this firmware does

BOTATO is a **micromouse**: a small autonomous robot that explores an unknown
grid maze, finds the goal, and then drives back. It does this with no map given
in advance — it *discovers* walls by sensing as it moves.

There are two kinds of run:

- **Search run** — explore the maze cell by cell, building a wall map, using
  flood-fill to always head toward the goal. On reaching the goal it turns
  around and returns to the start.
- **Speed run** — reuse the map learned during the search to drive a fast,
  known route (this layer is scaffolded and grows during Phase 2).

Everything else in the codebase exists to serve those two behaviours reliably:
read the sensors, hold a straight line, turn exactly 90°, and decide where to go
next.

---

## 2. Design philosophy — how choices were made

Every structural decision traces back to a few hard constraints and one goal:
**the code must be predictable on a small microcontroller, and readable enough
that the team can reason about it under competition pressure.**

### 2.1 Hard constraints (the rules that shaped everything)

The firmware is compiled with these flags:

```
-fno-exceptions  -fno-rtti  -fno-use-cxa-atexit  -std=gnu++17  (hard FPU)
```

That means, by deliberate choice:

| Constraint | Consequence in the design |
|---|---|
| **No exceptions** | Functions signal failure with return values / `bool ok`, never `throw`. |
| **No RTTI** | No `dynamic_cast`, no `typeid`. Polymorphism is avoided; types are concrete. |
| **No dynamic allocation** | No `new`/`malloc`, no `std::vector`/`std::string`. Everything is fixed-size and lives in static or stack memory. |
| **Hard FPU** | `float` math is cheap, so control code uses floats freely; `double` is avoided. |

The payoff: the RAM footprint is essentially the same as the old C version, the
memory map is knowable at compile time, and there is no hidden runtime
machinery that could stall the 1 kHz control loop.

### 2.2 Why C++ at all, then?

If dynamic features are banned, why not stay in C? Because the value of C++ here
is **organisation, not runtime features**. The project uses C++ purely for:

- **Classes** to bundle each piece of hardware with the state and logic that
  belongs to it (an `Encoder` owns its counter, an object you can't misuse).
- **Encapsulation** so modules expose a small, intention-revealing interface and
  hide their internals.
- **Composition** so the whole robot is assembled from small, testable parts.

This is "C with objects and strong boundaries," not "C++ with the kitchen sink."

### 2.3 The C-HAL / C++-application split

The vendor's hardware layer (STM32 HAL, and the VL53L0X ToF driver `tof.c`) is
**C**, and stays C. Only the *application* is C++. The two meet through
`extern "C"` wrappers around HAL headers. Rules that keep this boundary clean:

- HAL headers are included inside `extern "C" { ... }`.
- HAL types (e.g. `TIM_HandleTypeDef`) are **never forward-declared** — they are
  typedefs, so the header is included instead.
- The only file renamed from `.c` to `.cpp` is `main`; all CubeMX-generated
  files stay in C.

Think of it as: **C owns the silicon, C++ owns the behaviour.**

### 2.4 The patterns you'll see repeatedly

- **Facade** — a single `Robot` object owns every subsystem and exposes just two
  entry points. `main.cpp` stays ~10 lines.
- **Dependency injection via `init()`** — objects don't construct their
  collaborators; they receive pointers in an `init()` call. This is how the
  "no dynamic allocation, single global instance" world wires itself together
  without constructors running in an undefined order.
- **Pure-logic vs hardware separation** — algorithm modules (`FloodFill`,
  `Planner`, `MazeMap`) contain *no hardware calls at all*. They can be reasoned
  about, and in principle tested, on their own.
- **One source of truth for constants** — all physical/tuning numbers live in
  `Config.h` (`cfg::`) and `MazeConfig.h` (`maze::`). No magic numbers scattered
  through the logic.

---

## 3. Layered architecture — how things are organised

The code is a stack of layers. Each layer only talks to the one below it. Higher
layers make decisions; lower layers touch hardware.

```
            ┌─────────────────────────────────────────────┐
   FACADE   │                   Robot                      │  owns everything,
            │        onControlTick()  onMainLoop()         │  two entry points
            └───────────────┬─────────────────┬───────────┘
                            │                 │
   STATE / LOGIC   ┌────────▼──────┐  ┌───────▼─────────┐
                   │ ModeController│  │   Navigator     │  run lifecycle +
                   │ (IDLE/RUN/…)  │  │ (maze solving)  │  cell-by-cell driving
                   └───────┬───────┘  └──┬───────────┬──┘
                           │             │           │
   ALGORITHM (no HW)       │      ┌──────▼───┐ ┌─────▼────┐ ┌─────────┐
                           │      │ FloodFill│ │ MazeMap  │ │ Planner │
                           │      └──────────┘ └──────────┘ └─────────┘
                           │
   CONTROL          ┌──────▼───────────────────────────────┐
                    │            ControlLoop                │  cascaded PID
                    │      (uses PIDController ×4)           │  @ 1 kHz
                    └───┬───────────────┬──────────────┬────┘
                        │               │              │
   HARDWARE          ┌──▼───┐  ┌────────▼───────┐  ┌───▼──────────────┐
   ABSTRACTION       │ Gyro │  │ Encoder ×2     │  │ DifferentialDrive│
   (C++ objects)     └──┬───┘  └───────┬────────┘  └───────┬──────────┘
                        │              │                   │  Motor ×2
                  ┌─────▼──────┐       │            (WallSensorArray,
                  │  (I2C)     │       │             ButtonManager,
                  └────────────┘       │             Indicator alongside)
                                       │
   VENDOR (C)     ┌───────────────────────────────────────┐
                  │   STM32 HAL   +   tof.c (VL53L0X)      │
                  └───────────────────────────────────────┘
```

### 3.1 Module directory map

Each folder under `Core/App/` is one responsibility:

| Module | Type | Responsibility |
|---|---|---|
| `Robot` | facade | Owns all subsystems; wires them; two entry points. |
| `config/Config.h` | constants | All physical + tuning numbers (`cfg::`). |
| `tof/` (C) | driver | VL53L0X: XSHUT sequencing, continuous mode, median+EMA filter. |
| `wallsensor/WallSensorArray` | HW abstraction | The 5 ToF sensors as one object + per-sensor calibration. |
| `encoder/Encoder` | HW abstraction | One quadrature timer → distance + speed. |
| `gyro/Gyro` | HW abstraction | MPU6050 yaw rate → integrated heading angle. |
| `motor/Motor` | HW abstraction | One TB6612 channel: PWM + direction, dead-band. |
| `drive/DifferentialDrive` | HW abstraction | Owns both motors; mixes forward+turn into L/R PWM. |
| `control/PIDController` | control | Generic reusable PID (header-only). |
| `control/ControlLoop` | control | The 1 kHz cascaded controller. |
| `buttons/ButtonManager` | input | Debounced button events (short/long/both). |
| `modes/ModeController` | state | Top-level run lifecycle, button-driven. |
| `maze/MazeMap` | algorithm | Wall + visited storage for the grid. |
| `maze/MazeConfig.h` | constants | Maze size, start, goal (`maze::`). |
| `floodfill/FloodFill` | algorithm | BFS distance-to-goal gradient. |
| `planner/Planner` | algorithm | Absolute direction ↔ relative turn; pose tracking. |
| `navigator/Navigator` | state | The maze-solving brain: per-cell sense→decide→move. |
| `indicator/Indicator` | output | Status LED patterns (buzzer stubbed). |
| `tests/` | scaffolding | Bench tests selected by `#define` in `main.cpp`. |

---

## 4. The two execution contexts (the single most important idea)

The firmware runs in **two places at once**, and understanding which code runs
where explains most of the design.

```
   ┌──────────────────────────────┐        ┌──────────────────────────────┐
   │  TIM3 ISR  — 1 kHz, fixed     │        │  while(1)  — main loop        │
   │  Robot::onControlTick()       │        │  Robot::onMainLoop()          │
   ├──────────────────────────────┤        ├──────────────────────────────┤
   │  gyro_.update()               │        │  walls_.poll()   (I2C, slow)  │
   │  encL_.update()               │        │  btn_.update()                │
   │  encR_.update()               │        │  modes_.update() ── Navigator │
   │  ctrl_.tick()  → motors       │        │  led_.update()                │
   └──────────────────────────────┘        └──────────────────────────────┘
        FAST · DETERMINISTIC                    BACKGROUND · NON-BLOCKING
     must never block or wait               may take variable time; that's ok
```

**Why the split?**

- The **control loop must run at an exact, steady 1 kHz** or the PID maths (which
  assume a fixed `dt = 0.001 s`) drift. So sensor reads it depends on — the gyro
  rate and the wheel encoders — are updated *inside the ISR*, immediately before
  `ctrl_.tick()` consumes them. Order matters: fresh data first, control second.
- The **ToF sensors are read over I2C, which is comparatively slow** and would
  wreck the ISR's timing if done there. So wall sensing lives in the main loop
  via a *non-blocking* `poll()` — it grabs whatever reading is ready and moves
  on. The navigator reads the latest calibrated values whenever it needs them.

This is the classic embedded pattern: **a fast deterministic control tick, and a
slower best-effort background loop**, sharing data through member variables.

---

## 5. The control system

`ControlLoop::tick()` is the heartbeat. Its job: given target motions, drive the
two motors so the robot actually moves that way. It uses four instances of one
small `PIDController` class.

### 5.1 The reusable PID

`PIDController` (header-only) is a plain PID with two production touches:

- **Derivative on error** — `deriv = error - prevError`.
- **Integral anti-windup clamp** — the integral term is capped at `±iClamp` so it
  can't accumulate a huge lag while the output is saturated.

```cpp
return kp_*error + ki_*integral_ + kd_*deriv;   // integral_ is clamped
```

Config is passed in the constructor; the accumulators are the only state.
`reset()` clears them (used whenever a control mode changes, so stale integral
doesn't leak into a new movement).

### 5.2 Process variables (what's measured)

- **Forward speed** `pvX` = average of the two wheel speeds (`Encoder::speedMmPerS`).
- **Turn rate** `pvW` = the gyro's yaw rate (`Gyro::rateDps`).

Both are guarded: if a reading comes back NaN, it's forced to 0 so a bad sample
can't poison the loop.

### 5.3 The cascade

Turning is controlled as a **cascade** — an outer loop sets the target for an
inner loop:

```
   heading error ──► pidHeading ──► rate target ──► pidW ──► turn PWM
      (deg)                          (deg/s)                 (raw)
```

- **Outer (heading):** how far off the commanded heading are we? → produces a
  desired turn *rate*. Clamped to `MAX_RATE_DPS` so the cascade can't demand an
  impossible spin (which would saturate and oscillate).
- **Inner (rate):** make the measured gyro rate match that target → produces the
  turn PWM.

Forward motion is a single simpler loop: `pidX` drives measured speed to
`targetX`.

### 5.4 Corridor centering

When the robot is driving down a corridor with a wall on **both** sides, those
walls are a drift-free reference the gyro isn't. A dedicated `pidWall` takes the
left-minus-right distance error and feeds a **steering correction into the
heading cascade**, continuously nudging the robot back to the centreline and
cancelling slow gyro drift.

Crucially this is **gated** — it only engages when the navigator confirms a
genuine, stable corridor (see §8.4). In an open cell or at a junction the
left/right difference is meaningless, so it's switched off.

### 5.5 From control output to wheels

The loop produces two abstract numbers, `fwd` (common-mode) and `turn`
(differential), and hands them to `DifferentialDrive`, which mixes:

```
   left  PWM = fwd - turn
   right PWM = fwd + turn         (then clamped to ±PWM_MAX)
```

Each `Motor` then applies a **dead-band remap** (small commands are lifted to the
PWM that actually overcomes static friction, so the wheel moves instead of just
buzzing) and sets direction pins + PWM duty. Direction polarity is inverted
versus the driver datasheet to match this board's wiring — hidden entirely
inside `Motor`, so nothing above it needs to know.

### 5.6 Command interface

The navigator never touches PWM. It speaks to the control loop in intentions:

| Call | Meaning |
|---|---|
| `setTargets(speedX, speedW)` | direct speed + turn-rate (open control). |
| `holdHeading(deg)` | pivot in place to a heading and hold it (turns). |
| `driveStraight(speedX, deg)` | move forward while holding a heading (corridors). |

Guards throughout keep it safe: rate clamp, a turn-authority cap while driving
(so steering can't starve forward PWM), and NaN guards on every process
variable.

---

## 6. The navigation system

This is the "brain." It's built from three pure-logic modules plus one state
machine that ties them to the control loop.

### 6.1 MazeMap — what the robot knows

The maze is a grid of cells. Each cell is **one byte**:

```
   bit0 bit1 bit2 bit3 bit4
    N    E    S    W   visited
```

- Walls are stored per **absolute** direction (North/East/South/West), where
  `Dir` is `enum { NORTH=0, EAST=1, SOUTH=2, WEST=3 }`.
- `setWall()` marks the wall on **both** the current cell and its neighbour, so
  the map is consistent viewed from either side.
- `reset()` pre-fills the outer boundary walls (the maze always has an outer
  perimeter).
- Fixed-size `uint8_t cell_[MAX_DIM][MAX_DIM]` — no allocation.

### 6.2 FloodFill — which way is the goal

Classic micromouse flood-fill: a **breadth-first search** from the goal outward,
filling every cell with its step-distance to the nearest goal, respecting known
walls.

- `recompute()` floods from the goal region; `recomputeTo(x,y)` floods toward an
  arbitrary target (used to return to start).
- `nextDir()` then just looks at the open neighbours and returns the direction
  with the **lowest distance** — i.e. "downhill toward the goal."

Because it re-floods on the *current* known map each step, the robot always
follows the best route given everything it has discovered so far, and naturally
re-routes when a newly-sensed wall blocks the old path.

### 6.3 Planner — geometry and pose

The maze thinks in **absolute** directions; the robot experiences **relative**
turns. `Planner` is the pure translator between them, plus the pose bookkeeper.

- `GridPose { int x, y; Dir facing; }` — where the robot is and which way it
  points.
- `turnFor(facing, target)` — "I face EAST, flood-fill says go NORTH: what turn
  is that?" Computed with modulo-4 arithmetic on the clockwise `Dir` enum, so
  N→E is always a RIGHT turn. **This stays purely geometric** regardless of the
  gyro's sign convention — mixing the two up was a real bug once.
- `stepForward(pose)` / `applyTurn(pose, turn)` — update the pose after a
  completed move.

### 6.4 Navigator — the per-cell state machine

`Navigator::update()` runs one small state machine **per cell**. Each cell is a
five-phase cycle:

```
   ┌────────┐   ┌────────┐   ┌────────────┐        ┌──────────┐
   │ SENSE  │──►│ DECIDE │──►│  TURNING   │──(done)─┤          │
   │ read   │   │ flood, │   │ pivot 90°  │        │ DRIVING  │──(cell done)──►
   │ walls  │   │ choose │   └────────────┘   ┌────►│ forward  │      ARRIVE
   │ into   │   │ turn   │──(no turn)──────────┘    │ one cell │      → next
   │ map    │   └───┬────┘                          └──────────┘        SENSE
   └────────┘       │
                    └─(goal / back-at-start)──► goalOrReturnTransition()
```

- **SENSE** — read left/front/right wall sensors, record them into `MazeMap` at
  the current pose, mark the cell visited.
- **DECIDE** — if this cell is the goal (search) or the start (return), stop and
  transition. Otherwise re-run flood-fill and ask `Planner::turnFor()` for the
  turn that heads downhill.
- **TURNING** — if a turn is needed, command `holdHeading()` to the new heading
  and wait until settled (within tolerance and low residual rate), with a
  timeout guard so a stuck turn can't hang forever.
- **DRIVING** — command `driveStraight()` and run forward until the cell is
  complete.
- **ARRIVE** — advance the pose one cell and loop back to SENSE.

Heading targets are handled as **relative accumulations** (`headingRef_ += ±90`)
rather than absolute compass values, which keeps them consistent with the gyro's
continuous, unbounded angle.

### 6.5 Search / Return / Speed

The Navigator's own `State` enum sequences the whole run:

- **SEARCH** floods toward the goal region. On reaching any goal cell it flips to
  **RETURN**, which floods back toward the start — the planner naturally emits a
  180° turn to leave the goal and head home.
- **SPEED** reuses the learned map to run a route.
- **DONE** stops the motors and disables the loop.

---

## 7. End-to-end flow (power-on to finish)

```
  power on
     │
     ▼
  main(): HAL + peripheral init  ──►  robot.init()
     │                                  ├─ bring up every subsystem
     │                                  ├─ calibrate gyro (hold still)
     │                                  └─ start TIM3 1 kHz control ISR
     ▼
  while(1) { robot.onMainLoop(); }        ◄── ISR fires 1000×/s in parallel:
     │                                        gyro+enc update → ctrl.tick → motors
     ▼
  IDLE  ── button ──►  ModeController
     │                    ├─ SEARCH short  → Navigator.startSearch()
     │                    ├─ SPEED  short  → Navigator.startSpeed()
     │                    └─ SEARCH long   → recalibrate gyro
     ▼
  SEARCH_RUN: every main loop → Navigator.update()
     │            SENSE → DECIDE → TURN/DRIVE → ARRIVE → repeat per cell
     │            (control ISR keeps the robot straight & on-heading throughout)
     ▼
  reached goal → RETURN → flood back to start → DONE (motors off)
     ▲
     └────────── any button while running → ABORT → IDLE
```

Two loops, always: the **control ISR** physically keeps the robot moving
correctly, while the **main loop** decides *where* to move. They never block each
other; they communicate only through shared member variables.

---

## 8. Key conventions & invariants (do not break)

These are the non-obvious rules the codebase depends on. Most were learned the
hard way.

### 8.1 Gyro sign & heading are relative
The gyro is **anticlockwise-positive** and its angle is **continuous/unbounded**
(no ±180 wrapping in the gyro itself). Wrapping happens *only* inside the
controller's heading-error calculation. Turn targets accumulate relatively
(`headingRef_ += ±90`); using absolute compass targets against an unbounded gyro
caused infinite spinning. `TURN_LEFT = +90`, `TURN_RIGHT = −90` live in one place
(`beginTurn`), and nowhere else applies the sign.

### 8.2 Planner stays geometric
`turnFor()` uses the clockwise `Dir` enum (N=0,E=1,S=2,W=3), so N→E is always a
RIGHT turn — **independent of the gyro sign convention.** Only `beginTurn()`
applies the sign. Flipping this breaks turn decisions.

### 8.3 Cell completion is absolute when it can be
A cell finishes on **encoder** distance, but when a wall is ahead the **front
ToF** gives an absolute reference — the robot stops at a fixed sensor-to-wall
distance so accumulated slip can't drift the stopping point. Because the robot
pivots about its **rear axle**, a drive that *follows a turn* adds a small
forward offset; straight-through cells do not (else the offset would compound
over a long corridor).

### 8.4 Wall-centering only in a real corridor
Centering must not react to a stub, a wall edge, or a junction. It engages only
when: both side walls are seen, their **distance sum matches the known corridor
width** (an invariant that holds regardless of how off-centre the robot is), the
condition has **persisted over a short run** of travel, and the robot is **clear
of the cell boundaries**. The forward-angled sensors preview whether the corridor
continues into the next cell. All of this gating lives in the Navigator; the
control loop just applies whatever correction it's handed.

### 8.5 The goal is a region, not a point
Any cell inside the destination block counts as "reached," so the robot stops at
the **first mouth** it enters and turns back — it never wastes time driving to a
geometric centre. The block is defined in one place in `MazeConfig.h`.

---

## 9. Configuration surfaces

Two files hold everything you'd tune. Nothing else should contain magic numbers.

### 9.1 `Config.h` (`cfg::`) — the robot
Drivetrain geometry (wheel diameter, gear ratio, counts/rev → mm-per-tick),
control constants (`CONTROL_DT_S`, `PWM_MAX`, motor dead-band), gyro settings,
ToF filter + wall thresholds, ToF index order, and button timings. Derived
values (`MM_PER_TICK`, `WHEEL_CIRCUM_MM`) are `constexpr` expressions, so editing
one input recomputes the rest at compile time.

### 9.2 `MazeConfig.h` (`maze::`) — the maze
Maze `WIDTH`/`HEIGHT`, the `MAX_DIM` array bound, the start cell + start heading,
the goal region, and the physical cell size. **Walls are never set here** — the
robot discovers them.

### 9.3 PID gains
The four controllers' gains live where they're declared (in `ControlLoop.hpp`):
`pidX` (forward speed), `pidW` (turn rate), `pidHeading` (outer heading),
`pidWall` (centering). They're also live-tunable at runtime via `setGains()` from
a test harness.

---

## 10. How to extend it

- **Add a run behaviour** → extend the `Navigator` state machine (a new `State`
  and the phase logic to drive it); the control loop rarely needs changes.
- **Add a maze rule** (different goal, bigger maze) → edit `MazeConfig.h` only.
- **Change how a turn or drive completes** → `Navigator::turnComplete()` /
  `driveComplete()`; these are deliberately small and isolated.
- **Retune motion** → gains in `ControlLoop.hpp`, geometry in `Config.h`.
- **Add a status signal** → `Indicator` (the buzzer hook is stubbed and ready).

The guiding rule: **decisions go up the stack, hardware stays down.** If you find
yourself calling HAL from the Navigator, or making maze decisions inside a Motor,
the layering has been crossed and something belongs elsewhere.

---

## 11. Debugging notes

- **Live Expressions can't call C++ methods** — watch member variables directly,
  e.g. `robot.encL_.distanceMm_`, `robot.gyro_.angleDeg_`,
  `robot.walls_.distMm_[0..4]`, `robot.navigator_.pose_.x`, `robot.ctrl_.pvW_`,
  `robot.modes_.state_`. Set the Number Format to *Float* for float members.
- **Bench tests** are selected by a single `#define` block at the top of
  `main.cpp` (encoder / gyro / ToF / motor / drive / modes / pivot / one-cell
  drive). Enable exactly one motion test at a time.
- The status **LED** encodes state: fast blink = busy/calibrating, three flashes
  = calibration done, slow blink = idle waiting for a button.

---

## 12. One-paragraph summary

`main.cpp` builds the hardware and starts a 1 kHz control interrupt, then loops
forever calling `Robot::onMainLoop()`. The **`Robot`** facade owns everything.
Twice-over, the system runs: the **control ISR** reads the gyro and encoders and
runs a **cascaded PID** (`ControlLoop`) that keeps the robot moving straight, on
heading, and centred between walls; the **main loop** reads the ToF sensors,
handles buttons through **`ModeController`**, and — during a run — steps the
**`Navigator`** through a per-cell *sense → decide → turn/drive → arrive* cycle.
The Navigator leans on three pure-logic helpers: **`MazeMap`** (what walls are
known), **`FloodFill`** (which way is the goal), and **`Planner`** (turn geometry
and pose). Decisions flow down the stack; only the bottom layer touches silicon.
That separation, plus the ban on exceptions/RTTI/dynamic allocation, is what
keeps BOTATO predictable, debuggable, and fast.
