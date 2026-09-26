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

    void calibrate();
    void enterFault(Fault f);
    void enterIdle();
    void startRun(State s);
};

#endif // APP_MODECONTROLLER_HPP
