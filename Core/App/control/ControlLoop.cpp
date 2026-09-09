#include "ControlLoop.hpp"
#include "Config.h"

namespace {
    constexpr float MAX_RATE_DPS      = 200.0f;  // cap outer-loop demand
    constexpr float MAX_WALL_CORR_DPS = 60.0f;   // cap wall correction
    constexpr float MAX_WALL_HEADING_DEG = 20.0f;  // cap the wall-centering heading lean
}

void ControlLoop::init(Gyro* gyro, Encoder* encL, Encoder* encR,
                       DifferentialDrive* drive) {
    gyro_ = gyro; encL_ = encL; encR_ = encR; drive_ = drive;
    resetControllers();
}

void ControlLoop::setTargets(float speedX, float speedW) {
    targetX_ = speedX; targetW_ = speedW; headingHold_ = false;
}

void ControlLoop::holdHeading(float deg) {
    targetHeadingDeg_ = deg; headingHold_ = true; targetX_ = 0.0f;
    wallValid_ = false;              // no centering during a pivot
}

void ControlLoop::driveStraight(float speedX, float headingDeg) {
    targetHeadingDeg_ = headingDeg; headingHold_ = true; targetX_ = speedX;
}

void ControlLoop::enable(bool on) {
    enabled_ = on;
    if (!on) drive_->stop();
}

void ControlLoop::resetControllers() {
    pidX_.reset(); pidW_.reset(); pidHeading_.reset(); pidWall_.reset();
    if (gyro_) gyro_->zeroAngle();
    targetX_ = targetW_ = 0.0f;
    targetHeadingDeg_ = 0.0f;
    headingHold_ = false;
    wallValid_ = false;
    wallErrorMm_ = wallCorr_ = 0.0f;
    pvX_ = pvW_ = 0.0f;
}

void ControlLoop::tick() {
    if (!enabled_) return;

    pvX_ = 0.5f * (encL_->speedMmPerS() + encR_->speedMmPerS());
    pvW_ = gyro_->rateDps();
    if (!(pvW_ == pvW_)) pvW_ = 0.0f;          // NaN guard

    float rateTarget = targetW_;

    if (headingHold_) {
        // Wall centering trims the HEADING target, not the rate. One heading
        // authority => the wall loop and heading loop no longer fight, so the
        // bot converges to the centreline instead of settling off-centre.
        float headingTarget = targetHeadingDeg_;
        if (wallValid_ && targetX_ > 1.0f) {
            wallCorr_ = pidWall_.compute(wallErrorMm_);      // now DEGREES of lean
            if (wallCorr_ >  MAX_WALL_HEADING_DEG) wallCorr_ =  MAX_WALL_HEADING_DEG;
            if (wallCorr_ < -MAX_WALL_HEADING_DEG) wallCorr_ = -MAX_WALL_HEADING_DEG;
            headingTarget += wallCorr_;
        } else {
            wallCorr_ = 0.0f;
            pidWall_.reset();
        }

        float headErr = headingTarget - gyro_->angleDeg();
        while (headErr >  180.0f) headErr -= 360.0f;
        while (headErr < -180.0f) headErr += 360.0f;
        rateTarget = pidHeading_.compute(headErr);
        if (rateTarget >  MAX_RATE_DPS) rateTarget =  MAX_RATE_DPS;
        if (rateTarget < -MAX_RATE_DPS) rateTarget = -MAX_RATE_DPS;
    } else {
        wallCorr_ = 0.0f;
    }

    float fwd  = pidX_.compute(targetX_ - pvX_);
    float turn = pidW_.compute(rateTarget - pvW_);
    if (!(fwd  == fwd))  fwd  = 0.0f;
    if (!(turn == turn)) turn = 0.0f;

    // Cap turn authority while driving, so steering corrections can't
    // starve the forward command of PWM headroom.
    if (targetX_ > 1.0f) {
        const float MAX_TURN_WHILE_DRIVING = 1200.0f;
        if (turn >  MAX_TURN_WHILE_DRIVING) turn =  MAX_TURN_WHILE_DRIVING;
        if (turn < -MAX_TURN_WHILE_DRIVING) turn = -MAX_TURN_WHILE_DRIVING;
    }
    drive_->setPwm(static_cast<int16_t>(fwd), static_cast<int16_t>(turn));
}
