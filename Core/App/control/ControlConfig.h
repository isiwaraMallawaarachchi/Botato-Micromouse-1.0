#ifndef APP_CONTROLCONFIG_H
#define APP_CONTROLCONFIG_H

#include <cstdint>
#include "Config.h"

namespace ctrlcfg {

struct Gains { float kp, ki, kd, iClamp; };

// Tuned values carried over unchanged from the previous robot.
constexpr Gains SPEED   = { 4.0f,   0.15f, 0.0f,   400.0f };  // mm/s error -> fwd PWM
constexpr Gains RATE    = { 100.0f, 0.0f,  200.0f, 400.0f };  // deg/s error -> turn PWM
constexpr Gains HEADING = { 4.0f,   0.0f,  0.5f,   200.0f };  // deg error -> deg/s

// Forward feed-forward, PWM per mm/s: fwd PWM = SPEED_FF * speed + speed PID.
// The PID alone cannot hold a speed (its integral is clamped to 60 PWM), so
// without this the robot settles far below the commanded speed. Default comes
// from the free-run speed; replace it with motorTest.suggestedSpeedFF.
constexpr float SPEED_FF = cfg::PWM_MAX / cfg::FREE_RUN_MMPS;              // ~7.2

// Acceleration feed-forward. The wheels lag the speed reference by about the
// motor time constant, so while braking the robot runs faster than the
// reference and overshoots every stop by roughly speed * tau. Pushing
// SPEED_FF * tau * accel ahead cancels that lag. MOTOR_TAU_S: motorTest.tauMs.
constexpr float MOTOR_TAU_S = 0.040f;
constexpr float ACCEL_FF    = SPEED_FF * MOTOR_TAU_S;

// The heading loop adds a correction on top of the profile's feed-forward
// rate. This caps the correction, not the total.
constexpr float HEADING_CORR_MAX_DPS  = 120.0f;

// Cap on turn authority while driving, so a steering correction cannot starve
// the forward command of PWM headroom.
constexpr float MAX_TURN_PWM_DRIVING  = 1200.0f;

// Largest heading lean the wall-centring trim may request.
constexpr float MAX_HEADING_TRIM_DEG  = 12.0f;

// A turn is complete when the profile has finished AND the robot has settled
// on the target, or when the timeout expires.
constexpr float    TURN_TOL_DEG       = 2.0f;
constexpr float    TURN_SETTLE_DPS    = 20.0f;
constexpr uint32_t TURN_TIMEOUT_MS    = 3000;

} // namespace ctrlcfg

#endif // APP_CONTROLCONFIG_H
