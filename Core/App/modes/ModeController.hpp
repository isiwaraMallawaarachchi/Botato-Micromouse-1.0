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
 * navigation decision, triple flash if the gyro is missing.
 */
class ModeController {
public:
    enum State : uint8_t { CALIBRATING, IDLE, SEARCH_RUN, SPEED_RUN, FAULT };

    void init(ButtonManager* btn, ControlLoop* ctrl, Gyro* gyro,
              Navigator* nav, Indicator* led);
    void update();

    State state() const { return state_; }

private:
    ButtonManager* btn_  = nullptr;
    ControlLoop*   ctrl_ = nullptr;
    Gyro*          gyro_ = nullptr;
    Navigator*     nav_  = nullptr;
    Indicator*     led_  = nullptr;

    State    state_        = CALIBRATING;
    uint32_t seenDecisions_ = 0;

    void calibrate();
    void enterIdle();
    void startRun(State s);
};

#endif // APP_MODECONTROLLER_HPP
