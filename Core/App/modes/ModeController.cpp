#include "ModeController.hpp"
#include "GyroConfig.h"
extern "C" {
#include "main.h"
#include "i2c.h"
}

namespace {
    // No ISR call at all this long after calibration starts = TIM3 not running.
    constexpr uint32_t NO_TICK_MS = 500;
    // Calibration normally takes CAL_TICKS ms; far longer = the tick is broken.
    constexpr uint32_t CAL_LIMIT_MS = 2u * gyrocfg::CAL_TICKS + 2000u;
}

void ModeController::init(ButtonManager* btn, ControlLoop* ctrl, Gyro* gyro,
                          Navigator* nav, Indicator* led) {
    btn_ = btn; ctrl_ = ctrl; gyro_ = gyro; nav_ = nav; led_ = led;
    ctrl_->enable(false);
    if (!gyro_->ok()) { enterFault(GYRO_NOT_FOUND); return; }

    // Robot::initCore() has already started the boot calibration.
    if (!gyro_->calibrating() && !gyro_->calibrated()) gyro_->beginCalibration();
    state_ = CALIBRATING;
    calStartMs_ = HAL_GetTick();
    led_->set(Indicator::BLINK);
}

void ModeController::enterFault(Fault f) {
    nav_->abort();
    ctrl_->enable(false);
    fault_ = f;
    state_ = FAULT;
    led_->code(f);
}

void ModeController::calibrate() {
    ctrl_->enable(false);
    gyro_->beginCalibration();          // runs in the ISR; main loop stays free
    state_ = CALIBRATING;
    calStartMs_ = HAL_GetTick();
    led_->set(Indicator::BLINK);
}

void ModeController::enterIdle() {
    ctrl_->enable(false);
    state_ = IDLE;
    btn_->takeAny();                    // nothing pressed during the run carries over
    idleLed();
}

namespace { constexpr int NO_MAP_CODE = 5; constexpr uint32_t NO_MAP_SHOW_MS = 3000;
            constexpr uint32_t RUN_GRACE_MS = 400; }

void ModeController::idleLed() {
    led_->set(nav_->curves() ? Indicator::HEARTBEAT : Indicator::OFF);
}

void ModeController::startRun(State s) {
    if (s == SPEED_RUN) {
        if (!nav_->startSpeed()) {          // no search has reached the goal yet
            ++speedRefused_;
            led_->code(NO_MAP_CODE);        // tell the user instead of doing nothing
            refusedUntil_ = HAL_GetTick() + NO_MAP_SHOW_MS;
            return;
        }
    } else {
        nav_->startSearch();
    }
    seenDecisions_ = nav_->decisions();
    state_ = s;
    refusedUntil_ = 0;
    runStartMs_ = HAL_GetTick();
    btn_->takeAny();                    // leftovers must not abort the new run
    led_->set(Indicator::OFF);
}

void ModeController::update() {
    if (state_ == FAULT) {
        // PA6 long: re-initialise the gyro (after reseating a connector).
        if (btn_->takeSearchLong() && gyro_->init(&hi2c2)) {
            fault_ = NONE;
            calibrate();
        }
        btn_->takeAny();
        return;
    }

    if (gyro_->lost()) { enterFault(GYRO_LOST); return; }   // also stops a run

    if (btn_->takeBothPress()) {
        nav_->abort();
        nav_->clearMap();
        btn_->takeAny();
        if (state_ != CALIBRATING) enterIdle();
        return;
    }

    switch (state_) {
    case CALIBRATING: {
        btn_->takeAny();                    // ignore presses while calibrating
        const uint32_t elapsed = HAL_GetTick() - calStartMs_;
        if ((gyro_->updates() == 0 && elapsed > NO_TICK_MS) || elapsed > CAL_LIMIT_MS) {
            enterFault(NO_CONTROL_TICK);
            break;
        }
        if (!gyro_->calibrating()) enterIdle();
        break;
    }

    case IDLE:
        if (refusedUntil_ && static_cast<int32_t>(HAL_GetTick() - refusedUntil_) >= 0) {
            refusedUntil_ = 0;
            idleLed();                      // end of a notice: back to the mode pattern
        }
        if (btn_->takeFastLong()) {                     // NORMAL <-> CURVED speed runs
            nav_->setCurves(!nav_->curves());
            refusedUntil_ = 0;
            btn_->takeAny();
            idleLed();                                  // shows the new mode at once
            return;
        }
        if (btn_->takeSearchLong())  { btn_->takeAny(); calibrate();          return; }
        if (btn_->takeSearchShort()) { btn_->takeAny(); startRun(SEARCH_RUN); return; }
        if (btn_->takeFastShort())   { btn_->takeAny(); startRun(SPEED_RUN);  return; }
        break;

    case SEARCH_RUN:
    case SPEED_RUN:
        // Any NEW press aborts (rules 2.4.8 / 2.5.7) — but not in the first
        // RUN_GRACE_MS, so the press that started the run can never stop it.
        if ((HAL_GetTick() - runStartMs_) < RUN_GRACE_MS) { btn_->takeAny(); }
        else if (btn_->takeAny()) { nav_->abort(); enterIdle(); return; }

        nav_->update();

        if (nav_->decisions() != seenDecisions_) {   // one flash per decision
            seenDecisions_ = nav_->decisions();
            led_->pulse();
        }
        if (nav_->ended()) {
            const bool ok = (nav_->state() == Navigator::DONE);
            enterIdle();
            if (ok) {                                   // visible "finished"
                led_->set(Indicator::PASS);
                refusedUntil_ = HAL_GetTick() + NO_MAP_SHOW_MS;   // reuse the 3s timer
            }
        }
        break;

    case FAULT:
        break;
    }
}
