#ifndef APP_MODECONTROLLER_HPP
#define APP_MODECONTROLLER_HPP

#include "ButtonManager.hpp"
#include "ControlLoop.hpp"
#include "Gyro.hpp"
#include "Navigator.hpp"


/*
 * ModeController — top-level state machine driven by the buttons.
 * Phase 1 scope: it manages the run lifecycle (idle / calibrate / running /
 * abort) and gates the control loop. The actual maze navigation that fills a
 * SEARCH or SPEED run is added in Phase 2; here those states just enable the
 * control loop so you can confirm the plumbing end to end.
 *
 * Button map (confirmed):
 *   PA6 short  -> start SEARCH run
 *   PA5 short  -> start SPEED (fast) run
 *   any press while running -> ABORT to idle
 *   PA6 long (>3s), from idle -> CALIBRATE (gyro bias)
 *   both together -> full RESET to power-on idle
 */
class ModeController {
public:
    enum State { IDLE, CALIBRATING, SEARCH_RUN, SPEED_RUN };

    void init(ButtonManager* btn, ControlLoop* ctrl, Gyro* gyro, Navigator* nav);

    // Call every main-loop pass. Handles button events + state transitions.
    void update();

    State state() const { return state_; }

private:
    Navigator* nav_ = nullptr;   // add alongside btn_, ctrl_, gyro_
    ButtonManager* btn_  = nullptr;
    ControlLoop*   ctrl_ = nullptr;
    Gyro*          gyro_ = nullptr;
    State state_ = IDLE;

    void enterIdle();
    void startSearch();
    void startSpeed();
    void doCalibrate();
    void abortToIdle();
    void fullReset();
};

#endif // APP_MODECONTROLLER_HPP
