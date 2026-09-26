#include "ModeController.hpp"

void ModeController::init(ButtonManager* btn, ControlLoop* ctrl, Gyro* gyro,
                          Navigator* nav, Indicator* led) {
    btn_ = btn; ctrl_ = ctrl; gyro_ = gyro; nav_ = nav; led_ = led;
    ctrl_->enable(false);
    if (gyro_->ok()) {
        // Robot::initCore() has already started the boot calibration.
        if (!gyro_->calibrating() && !gyro_->calibrated()) gyro_->beginCalibration();
        state_ = CALIBRATING;
        led_->set(Indicator::BLINK);
    } else {
        state_ = FAULT;                 // cannot hold a heading: refuse to run
        led_->set(Indicator::FAIL);
    }
}

void ModeController::calibrate() {
    ctrl_->enable(false);
    gyro_->beginCalibration();          // runs in the ISR; main loop stays free
    state_ = CALIBRATING;
    led_->set(Indicator::BLINK);
}

void ModeController::enterIdle() {
    ctrl_->enable(false);
    state_ = IDLE;
    led_->set(Indicator::OFF);
}

void ModeController::startRun(State s) {
    if (s == SPEED_RUN) {
        if (!nav_->startSpeed()) return;   // no search map yet: stay idle
    } else {
        nav_->startSearch();
    }
    seenDecisions_ = nav_->decisions();
    state_ = s;
    led_->set(Indicator::OFF);
}

void ModeController::update() {
    if (state_ == FAULT) { btn_->takeAny(); return; }

    if (btn_->takeBothPress()) {
        nav_->abort();
        nav_->clearMap();
        btn_->takeAny();
        if (state_ != CALIBRATING) enterIdle();
        return;
    }

    switch (state_) {
    case CALIBRATING:
        btn_->takeAny();                    // ignore presses while calibrating
        if (!gyro_->calibrating()) enterIdle();
        break;

    case IDLE:
        if (btn_->takeSearchLong())  { calibrate();          return; }
        if (btn_->takeSearchShort()) { startRun(SEARCH_RUN); return; }
        if (btn_->takeFastShort())   { startRun(SPEED_RUN);  return; }
        break;

    case SEARCH_RUN:
    case SPEED_RUN:
        if (btn_->takeAny()) { nav_->abort(); enterIdle(); return; }   // rules 2.4.8 / 2.5.7

        nav_->update();

        if (nav_->decisions() != seenDecisions_) {   // one flash per decision
            seenDecisions_ = nav_->decisions();
            led_->pulse();
        }
        if (nav_->ended()) enterIdle();
        break;

    case FAULT:
        break;
    }
}
