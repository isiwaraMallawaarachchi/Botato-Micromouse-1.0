#ifndef APP_CONTROLLOOP_HPP
#define APP_CONTROLLOOP_HPP

#include "Encoder.hpp"
#include "Gyro.hpp"
#include "DifferentialDrive.hpp"
#include "PIDController.hpp"

/*
 * ControlLoop — 1kHz control.
 *   setTargets(speedX, speedW) — direct rate mode
 *   holdHeading(deg)           — pivot, zero forward
 *   driveStraight(speedX, deg) — forward while holding heading
 *
 * Heading is cascaded: heading error -> rate target -> rate PID -> PWM.
 * When driving a corridor with both side walls visible, a wall-centering
 * PID adds a correction to the rate target, cancelling gyro drift.
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

    // Corridor centering input, set each main loop from the wall sensors.
    // errorMm = leftDistance - rightDistance (0 = centred). valid = both
    // walls actually seen; when false the correction is skipped entirely.
    void setWallError(float errorMm, bool valid) {
        wallErrorMm_ = errorMm;
        wallValid_   = valid;
    }

    float pvX()  const { return pvX_; }
    float pvW()  const { return pvW_; }
    float wallCorrection() const { return wallCorr_; }
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
    float wallCorr_    = 0.0f;   // last applied correction (debug)

    bool enabled_ = false;

    PIDController pidX_{4.0f, 0.15f, 0.0f, 400.0f};
    PIDController pidW_{65.0f, 0.0f, 200.0f, 400.0f};
    PIDController pidHeading_{8.0f, 0.0f, 0.5f, 200.0f};
    PIDController pidWall_{0.8f, 0.0f, 0.8f, 20.0f};  // wall offset: mm error -> deg of heading lean
};

#endif
