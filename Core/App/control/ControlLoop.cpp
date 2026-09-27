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
    arcActive_ = false;
    arcFfDps_ = 0.0f;
    arcFfPwmDps_ = 0.0f;
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
    arcActive_ = false;
    arcFfDps_ = 0.0f;
    arcFfPwmDps_ = 0.0f;
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

void ControlLoop::arcTurn(float deltaDeg, float lengthMm) {
    CriticalSection cs;
    if (rawMode_) { heading_.reset(gyro_->angleDeg()); rawMode_ = false; }
    arcStartDeg_ = heading_.goal();                       // the straight we are on
    arcDeltaDeg_ = deltaDeg;
    arcLenMm_    = lengthMm;
    arcS0Mm_     = 0.5f * (encL_->distanceMm() + encR_->distanceMm());
    arcFfDps_    = 0.0f;
    arcFfPwmDps_ = 0.0f;
    arcMaxLagDeg_ = 0.0f;
    trim_        = 0.0f;                                  // no wall lean in a curve
    arcActive_   = true;
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
        float ref, rateFf;
        if (arcActive_) {
            // Curved turn: heading as a function of distance along the curve.
            const float s = 0.5f * (encL_->distanceMm() + encR_->distanceMm()) - arcS0Mm_;
            float u = s / arcLenMm_;
            if (u < 0.0f) u = 0.0f;
            if (u >= 1.0f) {                         // curve complete: hold the new heading
                heading_.reset(arcStartDeg_ + arcDeltaDeg_);
                arcEndErrDeg_ = (arcStartDeg_ + arcDeltaDeg_ - gyro_->angleDeg()) *
                                (arcDeltaDeg_ >= 0.0f ? 1.0f : -1.0f);   // + = under-turned
                arcActive_ = false;
                arcFfDps_  = 0.0f;
                arcFfPwmDps_ = 0.0f;
                ref    = heading_.pos();
                rateFf = 0.0f;
            } else {
                ref    = arcStartDeg_ + arcDeltaDeg_ * ArcShape::angle(u, ctrlcfg::ARC_RAMP_FRAC);
                const float k = vRef / arcLenMm_;                     // du/dt
                rateFf = arcDeltaDeg_ * ArcShape::rate(u, ctrlcfg::ARC_RAMP_FRAC) * k;
                const float rateAcc = arcDeltaDeg_ * ArcShape::rateSlope(u, ctrlcfg::ARC_RAMP_FRAC) * k * k;
                arcFfDps_    = rateFf;
                arcFfPwmDps_ = ctrlcfg::ARC_TURN_FF_GAIN * (rateFf + ctrlcfg::ARC_TURN_LEAD_S * rateAcc);
                // lag: how far the heading is behind the plan, in the turn's direction
                const float lag = (ref - gyro_->angleDeg()) * (arcDeltaDeg_ >= 0.0f ? 1.0f : -1.0f);
                if (lag > arcMaxLagDeg_) arcMaxLagDeg_ = lag;
            }
        } else {
            heading_.step(cfg::TURN_RATE_DPS, cfg::TURN_ACCEL_DPS2, cfg::CONTROL_DT_S);
            if (turnTicks_ <= ctrlcfg::TURN_TIMEOUT_MS) ++turnTicks_;
            ref    = heading_.pos();
            rateFf = heading_.vel();
        }

        // Continuous angles: no +/-180 wrap, so a 180 turn is never ambiguous.
        headingErr_ = (ref + trim_) - gyro_->angleDeg();
        float corr = pidHeading_.compute(headingErr_);
        if (corr >  ctrlcfg::HEADING_CORR_MAX_DPS) corr =  ctrlcfg::HEADING_CORR_MAX_DPS;
        if (corr < -ctrlcfg::HEADING_CORR_MAX_DPS) corr = -ctrlcfg::HEADING_CORR_MAX_DPS;
        rateTarget = rateFf + corr;                  // feed-forward + correction
    }

    const float fwd = ctrlcfg::SPEED_FF * vRef + ctrlcfg::ACCEL_FF * aRef +
                      pidSpeed_.compute(vRef - pvX_);
    float turn      = pidRate_.compute(rateTarget - pvW_);

    // In a curve, supply the turning PWM directly; the PID only corrects.
    const bool inArc = arcActive_;
    if (inArc) turn += ctrlcfg::ARC_RATE_FF * arcFfPwmDps_;

    if (std::fabs(vRef) > 1.0f) {
        const float cap = inArc ? ctrlcfg::MAX_TURN_PWM_ARC : ctrlcfg::MAX_TURN_PWM_DRIVING;
        if (turn >  cap) turn =  cap;
        if (turn < -cap) turn = -cap;
    }

    drive_->setPwm(fwd, turn);   // float in: clamped and desaturated in drive
}
