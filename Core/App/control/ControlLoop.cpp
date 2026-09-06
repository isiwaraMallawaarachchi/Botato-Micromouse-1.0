#include "ControlLoop.hpp"
#include "Config.h"

void ControlLoop::init(Gyro* gyro, Encoder* encL, Encoder* encR,
                       DifferentialDrive* drive) {
    gyro_ = gyro; encL_ = encL; encR_ = encR; drive_ = drive;
    resetControllers();
}

void ControlLoop::setTargets(float speedX, float speedW) {
    targetX_ = speedX;
    targetW_ = speedW;
    headingHold_ = false;
}

void ControlLoop::holdHeading(float deg) {
    targetHeadingDeg_ = deg;
    headingHold_ = true;
    targetX_ = 0.0f;
}

void ControlLoop::driveStraight(float speedX, float headingDeg) {
    targetHeadingDeg_ = headingDeg;
    headingHold_ = true;
    targetX_ = speedX;      // forward speed WITH heading hold
}

void ControlLoop::enable(bool on) {
    enabled_ = on;
    if (!on) drive_->stop();
}

void ControlLoop::resetControllers() {
    pidX_.reset(); pidW_.reset(); pidHeading_.reset();
    if (gyro_) gyro_->zeroAngle();
    targetX_ = targetW_ = 0.0f;
    targetHeadingDeg_ = 0.0f;
    headingHold_ = false;
    pvX_ = pvW_ = 0.0f;
}

void ControlLoop::tick() {
    if (!enabled_) return;

    pvX_ = 0.5f * (encL_->speedMmPerS() + encR_->speedMmPerS());
    pvW_ = gyro_->rateDps();

    float rateTarget = targetW_;
    if (headingHold_) {
        float headErr = targetHeadingDeg_ - gyro_->angleDeg();
        while (headErr >  180.0f) headErr -= 360.0f;
        while (headErr < -180.0f) headErr += 360.0f;
        rateTarget = pidHeading_.compute(headErr);
    }

    float fwd  = pidX_.compute(targetX_ - pvX_);
    float turn = pidW_.compute(rateTarget - pvW_);

    drive_->setPwm(static_cast<int16_t>(fwd), static_cast<int16_t>(turn));
}
