#ifndef APP_MODECONTROLLER_HPP
#define APP_MODECONTROLLER_HPP

#include "ButtonManager.hpp"
#include "ControlLoop.hpp"
#include "Gyro.hpp"
#include "Navigator.hpp"
#include "Indicator.hpp"

/*
 * ModeController — run lifecycle, driven by the buttons. Owns the LED policy.
 *
 *   PA6 short          start SEARCH run (explore, reach goal, return, park)
 *   PA5 short          start SPEED run  (needs a completed search)
 *   PA6 long  (>3s)    recalibrate the gyro (robot still)
 *   any press in a run abort to idle
 *   both together      abort and clear the maze map
 *
 * LED: blinks while calibrating, off when calibrated, one flash per
 * navigation decision. A fault shows as a repeating flash code (count the
 * flashes between pauses) and stops the robot:
 *   2  GYRO_NOT_FOUND   MPU6050 did not answer at boot (battery off? I2C2?)
 *   3  GYRO_LOST        answered at boot, then stopped (connector, 3V3 dip)
 *   4  NO_CONTROL_TICK  the 1kHz TIM3 interrupt is not running
 * PA6 long press in FAULT re-initialises the gyro and tries again.
 *
 * PA5 with no map yet (no search has reached the goal) does not start a
 * speed run: the LED shows 5 flashes for 3s instead, and speedRefused_ counts.
 *
 * A run that finishes (home and parked) shows a double flash for 3s, so
 * "finished" never looks the same as "still running" (LED off in both).
 *
 * PA5 long (hold 3s) while idle: switch between NORMAL and CURVED speed
 * runs, back and forth as often as you like. While idle the LED shows the
 * mode: OFF = normal, HEARTBEAT (short flash every second) = curved.
 *
 * Button handling: one debounced press = one event; after the robot acts on
 * a press, other pending events are discarded, and presses are ignored for
 * RUN_GRACE_MS after a run starts, so a start press can never abort its run.
 */
class ModeController {
public:
    enum State : uint8_t { CALIBRATING, IDLE, SEARCH_RUN, SPEED_RUN, FAULT };
    enum Fault : uint8_t { NONE = 0, GYRO_NOT_FOUND = 2, GYRO_LOST = 3, NO_CONTROL_TICK = 4 };

    void init(ButtonManager* btn, ControlLoop* ctrl, Gyro* gyro,
              Navigator* nav, Indicator* led);
    void update();

    State state() const { return state_; }
    Fault fault() const { return fault_; }

private:
    ButtonManager* btn_  = nullptr;
    ControlLoop*   ctrl_ = nullptr;
    Gyro*          gyro_ = nullptr;
    Navigator*     nav_  = nullptr;
    Indicator*     led_  = nullptr;

    State    state_        = CALIBRATING;
    Fault    fault_        = NONE;
    uint32_t seenDecisions_ = 0;
    uint32_t calStartMs_   = 0;
    uint32_t refusedUntil_ = 0;     // 5-flash "no map" notice, until this time
    uint32_t speedRefused_ = 0;     // Live Expression: PA5 presses refused

    uint32_t runStartMs_   = 0;

    void calibrate();
    void idleLed();                 // OFF (normal mode) or HEARTBEAT (curved mode)
    void enterFault(Fault f);
    void enterIdle();
    void startRun(State s);
};

#endif // APP_MODECONTROLLER_HPP
