#ifndef APP_CONTROLLOOP_HPP
#define APP_CONTROLLOOP_HPP

#include "Encoder.hpp"
#include "Gyro.hpp"
#include "DifferentialDrive.hpp"
#include "PIDController.hpp"

/*
 * ControlLoop — 1kHz control. Modes:
 *   setTargets(speedX, speedW) — direct rate mode (speedW = turn rate).
 *   holdHeading(deg)           — pivot: zero forward, hold absolute heading.
 *   driveStraight(speedX, deg) — forward at speedX while holding heading deg.
 * All heading control is cascaded: heading error -> rate target -> rate PID.
 */
class ControlLoop {
public:
    void init(Gyro* gyro, Encoder* encL, Encoder* encR, DifferentialDrive* drive);

    void setTargets(float speedX, float speedW);
    void holdHeading(float deg);
    void driveStraight(float speedX, float headingDeg);

    void tick();
    void enable(bool on);
    void resetControllers();

    void setWallError(float mm, bool valid) { wallErrorMm_ = mm; wallValid_ = valid; }

    float pvX()  const { return pvX_; }
    float pvW()  const { return pvW_; }
    float headingDeg() const { return gyro_->angleDeg(); }
    int16_t leftPwm()  const { return drive_->lastLeftPwm(); }
    int16_t rightPwm() const { return drive_->lastRightPwm(); }

private:
    Gyro*              gyro_  = nullptr;
    Encoder*           encL_  = nullptr;
    Encoder*           encR_  = nullptr;
    DifferentialDrive* drive_ = nullptr;

    float targetX_ = 0.0f;
    float targetW_ = 0.0f;
    float targetHeadingDeg_ = 0.0f;
    bool  headingHold_ = false;

    float pvX_ = 0.0f;
    float pvW_ = 0.0f;

    float wallErrorMm_ = 0.0f;
    bool  wallValid_   = false;

    bool enabled_ = false;

    PIDController pidX_{5.0f, 0.0f, 0.0f, 400.0f};
    PIDController pidW_{4.0f, 0.0f, 2.0f, 400.0f};
    PIDController pidHeading_{40.0f, 0.0f, 20.0f, 200.0f};
};

#endif // APP_CONTROLLOOP_HPP
