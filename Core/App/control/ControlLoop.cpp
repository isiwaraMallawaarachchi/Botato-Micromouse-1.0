#include "ControlLoop.hpp"
#include "Config.h"
#include "ControlConfig.h"
#include "Critical.hpp"
#include <cmath>

void ControlLoop::init(Gyro* gyro, Encoder* encL, Encoder* encR,
                       DifferentialDrive* drive) {
    gyro_ = gyro; encL_ = encL; encR_ = encR; drive_ = drive;
    resetForRun();
}

void ControlLoop::enable(bool on) {
    CriticalSection cs;
    enabled_ = on;
    vCmd_ = rawRateCmd_ = trim_ = 0.0f;
    ramp_.reset(0.0f);
    heading_.reset(heading_.goal());   // stop any turn in progress where it aims
    pidSpeed_.reset(); pidRate_.reset(); pidHeading_.reset();
    if (!on) drive_->stop();
}

void ControlLoop::resetForRun() {
    CriticalSection cs;
    if (gyro_) gyro_->zeroAngle();
    heading_.reset(0.0f);
    ramp_.reset(0.0f);
    vCmd_ = rawRateCmd_ = trim_ = 0.0f;
    rawMode_ = false;
    turnTicks_ = 0;
    pidSpeed_.reset(); pidRate_.reset(); pidHeading_.reset();
    pvX_ = pvW_ = headingErr_ = 0.0f;
}

void ControlLoop::setForwardSpeed(float mmps) { vCmd_ = mmps; }

void ControlLoop::turnBy(float deltaDeg) {
    CriticalSection cs;
    if (rawMode_) { heading_.reset(gyro_->angleDeg()); rawMode_ = false; }
    heading_.moveBy(deltaDeg);
    turnTicks_ = 0;
}

void ControlLoop::holdHeading() {
    CriticalSection cs;
    if (rawMode_) { heading_.reset(gyro_->angleDeg()); rawMode_ = false; }
}

void ControlLoop::setHeadingTrim(float deg) {
    if (deg >  ctrlcfg::MAX_HEADING_TRIM_DEG) deg =  ctrlcfg::MAX_HEADING_TRIM_DEG;
    if (deg < -ctrlcfg::MAX_HEADING_TRIM_DEG) deg = -ctrlcfg::MAX_HEADING_TRIM_DEG;
    trim_ = deg;
}

void ControlLoop::setRate(float mmps, float dps) {
    CriticalSection cs;
    rawMode_    = true;
    vCmd_       = mmps;
    rawRateCmd_ = dps;
}

bool ControlLoop::turnDone() const {
    if (turnTicks_ > ctrlcfg::TURN_TIMEOUT_MS) return true;   // 1 tick = 1ms
    return heading_.done() &&
           std::fabs(heading_.goal() - gyro_->angleDeg()) < ctrlcfg::TURN_TOL_DEG &&
           std::fabs(pvW_) < ctrlcfg::TURN_SETTLE_DPS;
}

void ControlLoop::tick() {
    if (!enabled_) return;

    pvX_ = 0.5f * (encL_->speedMmPerS() + encR_->speedMmPerS());
    pvW_ = gyro_->rateDps();
    if (!(pvW_ == pvW_)) pvW_ = 0.0f;

    const float vPrev = ramp_.value();
    const float vRef  = ramp_.step(vCmd_, cfg::ACCEL_MMPS2, cfg::DECEL_MMPS2,
                                   cfg::CONTROL_DT_S);
    const float aRef  = (vRef - vPrev) / cfg::CONTROL_DT_S;

    float rateTarget;
    if (rawMode_) {
        rateTarget  = rawRateCmd_;
        headingErr_ = 0.0f;
    } else {
        heading_.step(cfg::TURN_RATE_DPS, cfg::TURN_ACCEL_DPS2, cfg::CONTROL_DT_S);
        if (turnTicks_ <= ctrlcfg::TURN_TIMEOUT_MS) ++turnTicks_;

        // Continuous angles: no +/-180 wrap, so a 180 turn is never ambiguous.
        headingErr_ = (heading_.pos() + trim_) - gyro_->angleDeg();
        float corr = pidHeading_.compute(headingErr_);
        if (corr >  ctrlcfg::HEADING_CORR_MAX_DPS) corr =  ctrlcfg::HEADING_CORR_MAX_DPS;
        if (corr < -ctrlcfg::HEADING_CORR_MAX_DPS) corr = -ctrlcfg::HEADING_CORR_MAX_DPS;
        rateTarget = heading_.vel() + corr;          // feed-forward + correction
    }

    const float fwd = ctrlcfg::SPEED_FF * vRef + ctrlcfg::ACCEL_FF * aRef +
                      pidSpeed_.compute(vRef - pvX_);
    float turn      = pidRate_.compute(rateTarget - pvW_);

    if (std::fabs(vRef) > 1.0f) {
        if (turn >  ctrlcfg::MAX_TURN_PWM_DRIVING) turn =  ctrlcfg::MAX_TURN_PWM_DRIVING;
        if (turn < -ctrlcfg::MAX_TURN_PWM_DRIVING) turn = -ctrlcfg::MAX_TURN_PWM_DRIVING;
    }

    drive_->setPwm(fwd, turn);   // float in: clamped and desaturated in drive
}
