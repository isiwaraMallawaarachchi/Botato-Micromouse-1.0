#ifndef APP_CONTROLCONFIG_H
#define APP_CONTROLCONFIG_H

#include <cstdint>
#include "Config.h"

namespace ctrlcfg {

struct Gains { float kp, ki, kd, iClamp; };

// Tuned values carried over unchanged from the previous robot.
constexpr Gains SPEED   = { 4.0f,   0.15f, 0.0f,   400.0f };  // mm/s error -> fwd PWM
constexpr Gains RATE    = { 10.0f, 0.0f,  25.0f, 400.0f };  // deg/s error -> turn PWM
constexpr Gains HEADING = { 8.0f,   0.0f,  5.0f,   200.0f };  // deg error -> deg/s

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

// ---- Curved turns ----------------------------------------------------------
// Turn rate ramps up over the first ARC_RAMP_FRAC of the curve and down over
// the last, so the heading never steps (your rate loop is soft: kp 10).
constexpr float ARC_RAMP_FRAC    = 0.25f;
// PWM that makes the robot turn at 1 deg/s while driving: the wheel speed
// difference for that rate (WHEELBASE/2 * rad) times SPEED_FF. Fed forward
// during a curve so the rate PID only has to correct, not do the whole turn.
constexpr float ARC_RATE_FF      = SPEED_FF * (cfg::WHEELBASE_MM * 0.5f) * (cfg::PI_F / 180.0f);
// Turn-PWM cap during a curve (a curve needs ~1100 at 300 mm/s; the normal
// cap while driving is MAX_TURN_PWM_DRIVING).
constexpr float MAX_TURN_PWM_ARC = 3000.0f;

// >>> CURVE SHAPE TUNING (tune with TEST_CURVE, see tests/TestConfig.h) <<<
// If the heading falls behind the planned heading during a curve, the robot
// keeps going straight a little too long and the curve comes out WIDE.
//   ARC_TURN_FF_GAIN  scales the turning push fed forward in a curve.
//                     > 1.0 turns harder (tighter curve), < 1.0 softer.
//                     Steps of 0.05. Default 1.0 = the value from SPEED_FF.
//   ARC_TURN_LEAD_S   leads the turn command by the motor's response lag, so
//                     the turn rate builds up on time at the start of the
//                     curve. 0 = off; try MOTOR_TAU_S (0.040).
// Watch robot.ctrl_.arcMaxLagDeg_ (how far the heading fell behind, worst
// point) and robot.ctrl_.arcEndErrDeg_ (heading error as the curve ended).
constexpr float ARC_TURN_FF_GAIN = 1.70f;
constexpr float ARC_TURN_LEAD_S  = 0.10f;

// Largest heading lean the wall-centring trim may request.
constexpr float MAX_HEADING_TRIM_DEG  = 12.0f;

// A turn is complete when the profile has finished AND the robot has settled
// on the target, or when the timeout expires.
constexpr float    TURN_TOL_DEG       = 2.0f;
constexpr float    TURN_SETTLE_DPS    = 20.0f;
constexpr uint32_t TURN_TIMEOUT_MS    = 3000;

} // namespace ctrlcfg

#endif // APP_CONTROLCONFIG_H
