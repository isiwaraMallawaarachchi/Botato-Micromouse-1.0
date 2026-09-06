#include "ModeController.hpp"
#include "telemetry/Telemetry.hpp"

void ModeController::init(ButtonManager* btn, ControlLoop* ctrl, Gyro* gyro, Navigator* nav) {
    btn_  = btn;
    ctrl_ = ctrl;
    gyro_ = gyro;
    nav_ = nav;
    enterIdle();
}

void ModeController::enterIdle() {
    state_ = IDLE;
    ctrl_->enable(false);      // motors off in idle
    ctrl_->resetControllers();
    telemetry.event(EV_MODE, (int)state_);
}

void ModeController::startSearch() {
    state_ = SEARCH_RUN;
    telemetry.event(EV_MODE, (int)state_);
    nav_->startSearch();     // Navigator drives the maze
}

void ModeController::startSpeed() {
    state_ = SPEED_RUN;
    telemetry.event(EV_MODE, (int)state_);
    nav_->startSpeed();
}

void ModeController::doCalibrate() {
    state_ = CALIBRATING;
    telemetry.event(EV_MODE, (int)state_);
    ctrl_->enable(false);
    gyro_->calibrate();              // blocking ~1s; keep robot still
    enterIdle();
}

void ModeController::abortToIdle() {
    nav_->abort();
    enterIdle();
}

void ModeController::fullReset() {
    // Phase 2: also clears the maze map here.
    abortToIdle();
}

void ModeController::update() {
    // Both-press = full reset, highest priority, any state.
    if (btn_->takeBothPress()) { fullReset(); return; }

    switch (state_) {
    case IDLE:
        if (btn_->takeSearchLong())  { doCalibrate();  return; }
        if (btn_->takeSearchShort()) { startSearch();  return; }
        if (btn_->takeFastShort())   { startSpeed();   return; }
        break;

    case SEARCH_RUN:
    case SPEED_RUN:
    	nav_->update();
        // Any button press during a run aborts it (rule 2.4.8 / 2.5.7).
        if (btn_->takeSearchShort() || btn_->takeFastShort() ||
            btn_->takeSearchLong()) {
            abortToIdle();
            return;
        }
        break;

    case CALIBRATING:
        // Blocking calibrate returns to idle on its own; nothing to do.
        break;
    }
}
