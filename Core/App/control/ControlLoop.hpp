#ifndef APP_CONTROLLOOP_HPP
#define APP_CONTROLLOOP_HPP

#include <cstdint>
#include "Encoder.hpp"
#include "Gyro.hpp"
#include "DifferentialDrive.hpp"
#include "PIDController.hpp"
#include "MotionProfile.hpp"

/*
 * ControlLoop — the 1kHz controller. tick() runs in the TIM3 ISR.
 *
 * Two axes, each fed through a motion profile before its PID:
 *
 *   forward   setForwardSpeed(v) -> SpeedRamp -> feed-forward + speed PID -> fwd PWM
 *   heading   turnBy(delta)      -> AngleProfile
 *               heading error    -> heading PID -> + profile rate (feed-forward)
 *               rate error       -> rate PID    -> turn PWM
 *
 * Heading error is NOT wrapped to +/-180. The gyro angle and the heading
 * reference are both continuous, so turnBy(+180) is always an anticlockwise
 * half turn with no ambiguity. Anticlockwise is positive.
 *
 * setRate() is a raw mode for tuning the rate loop in tests; it bypasses the
 * heading loop until the next turnBy()/holdHeading().
 */
class ControlLoop {
public:
    void init(Gyro* gyro, Encoder* encL, Encoder* encR, DifferentialDrive* drive);

    void enable(bool on);            // off: motors stopped, commands zeroed
    bool enabled() const { return enabled_; }
    void resetForRun();              // zero heading, profiles and PIDs

    // ---- commands (main loop) ----
    void setForwardSpeed(float mmps);
    void turnBy(float deltaDeg);
    void holdHeading();              // leave raw mode, hold the current goal
    void setHeadingTrim(float deg);  // wall-centring lean, clamped
    void setRate(float mmps, float dps);

    bool turnDone() const;

    // ---- 1kHz ISR ----
    void tick();

    // ---- telemetry ----
    float pvX()          const { return pvX_; }
    float pvW()          const { return pvW_; }
    float speedRef()     const { return ramp_.value(); }
    float speedCmd()     const { return vCmd_; }
    float headingDeg()   const { return gyro_->angleDeg(); }
    float headingRef()   const { return heading_.pos(); }
    float headingGoal()  const { return heading_.goal(); }
    float rateRef()      const { return heading_.vel(); }
    float headingErr()   const { return headingErr_; }
    float trimDeg()      const { return trim_; }

private:
    Gyro*              gyro_  = nullptr;
    Encoder*           encL_  = nullptr;
    Encoder*           encR_  = nullptr;
    DifferentialDrive* drive_ = nullptr;

    volatile bool enabled_ = false;
    bool     rawMode_    = false;
    float    vCmd_       = 0.0f;
    float    rawRateCmd_ = 0.0f;
    float    trim_       = 0.0f;
    uint32_t turnTicks_  = 0;

    SpeedRamp    ramp_;
    AngleProfile heading_;

    float pvX_ = 0.0f, pvW_ = 0.0f, headingErr_ = 0.0f;

    PIDController pidSpeed_  {ctrlcfg::SPEED};
    PIDController pidRate_   {ctrlcfg::RATE};
    PIDController pidHeading_{ctrlcfg::HEADING};
};

#endif // APP_CONTROLLOOP_HPP
