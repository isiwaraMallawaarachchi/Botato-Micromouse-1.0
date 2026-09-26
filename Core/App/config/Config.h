#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <cstdint>

/*
 * Config.h — robot-wide constants only.
 *
 * Rule: a value used by MORE THAN ONE component lives here. A value only one
 * component needs lives in that component's folder:
 *
 *   control/ControlConfig.h     PID gains, turn completion
 *   motor/MotorConfig.h         dead-band
 *   encoder/EncoderConfig.h     speed estimate window
 *   gyro/GyroConfig.h           MPU6050 setup, calibration length
 *   tof/ToFConfig.h             timing budget, filter, CALIBRATION TABLE
 *   buttons/ButtonConfig.h      debounce, long-press
 *   indicator/IndicatorConfig.h LED pin and timing
 *   navigator/NavConfig.h       stop points, wall thresholds, centring
 *   maze/MazeConfig.h           maze size, start, goal
 *   tests/TestConfig.h          test parameters
 */

/* ===========================================================================
 * BUILD SELECT
 *
 * Commented out  -> competition firmware, Core/Src/main.cpp
 * Uncommented    -> test firmware,        Core/Src/main_test.cpp
 *                   (pick the test with the TEST_* defines in that file)
 * =========================================================================== */
//#define BOTATO_TEST_BUILD

namespace cfg {

constexpr float PI_F = 3.14159265f;

/* ---- Drivetrain (measured) ---------------------------------------------- */
constexpr float WHEEL_DIAMETER_MM = 44.26f;   // effective rolling diameter
constexpr float WHEELBASE_MM      = 75.4f;    // contact point to contact point
constexpr float GEAR_RATIO        = 50.0f;
constexpr float ENC_PPR           = 7.0f;     // motor-shaft pulses per rev
constexpr float QUADRATURE        = 4.0f;
constexpr float MOTOR_RPM         = 300.0f;   // wheel-shaft free-run

constexpr float COUNTS_PER_REV    = ENC_PPR * GEAR_RATIO * QUADRATURE;    // 1400
constexpr float WHEEL_CIRCUM_MM   = PI_F * WHEEL_DIAMETER_MM;             // 139.0
constexpr float MM_PER_TICK       = WHEEL_CIRCUM_MM / COUNTS_PER_REV;     // 0.0993
constexpr float FREE_RUN_MMPS     = MOTOR_RPM / 60.0f * WHEEL_CIRCUM_MM;  // ~695

/* ---- Body (measured) ------------------------------------------------------ */
constexpr float BODY_LENGTH_MM    = 85.2f;
constexpr float BODY_WIDTH_MM     = 85.2f;
constexpr float AXLE_TO_FRONT_MM  = 55.7f;
constexpr float AXLE_TO_REAR_MM   = BODY_LENGTH_MM - AXLE_TO_FRONT_MM;    // 29.5

/* ---- ToF mounting: lens face ahead of the wheel axle (measured) ---------- */
enum ToFIndex { TOF_LEFT = 0, TOF_LEFTFRONT, TOF_FRONT, TOF_RIGHTFRONT, TOF_RIGHT,
                TOF_COUNT };

constexpr float TOF_SIDE_AHEAD_MM  = 30.0f;   // Left, Right
constexpr float TOF_DIAG_AHEAD_MM  = 47.0f;   // LeftFront, RightFront
constexpr float TOF_FRONT_AHEAD_MM = 56.0f;   // Front

/* ---- Maze geometry (rule book 2.3.1) --------------------------------------
 * Unit cell 180mm clear between walls, walls 12mm thick, so wall-centre to
 * wall-centre is 192mm.                                                     */
constexpr float CORRIDOR_MM       = 180.0f;
constexpr float WALL_THICKNESS_MM = 12.0f;
constexpr float CELL_PITCH_MM     = CORRIDOR_MM + WALL_THICKNESS_MM;      // 192

/* ===========================================================================
 * >>> CELL DISTANCE — the single place to adjust it <<<
 *
 * Distance the robot drives for one cell. Starts at the rule-book pitch. If a
 * long straight in the real maze under- or overshoots, correct it here: it is
 * the only value the navigator and the drive tests use for one cell.
 * =========================================================================== */
constexpr float CELL_TRAVEL_MM    = CELL_PITCH_MM;

/* ---- Control loop / PWM --------------------------------------------------- */
constexpr float CONTROL_DT_S      = 0.001f;   // TIM3 @ 1kHz
constexpr int   PWM_MAX           = 4999;     // TIM1 ARR

/* ---- Motion limits: the acceleration curves -------------------------------
 * Forward speed never jumps: it ramps up at ACCEL and down at DECEL. Turns
 * follow the same shape — rate ramps up at TURN_ACCEL to TURN_RATE, then ramps
 * down so the robot arrives on the target angle at zero rate.              */
constexpr float SEARCH_SPEED_MMPS = 300.0f;
constexpr float SPEED_RUN_MMPS    = 450.0f;   // keep well under FREE_RUN_MMPS
constexpr float ACCEL_MMPS2       = 1500.0f;
constexpr float DECEL_MMPS2       = 2500.0f;
constexpr float TURN_RATE_DPS     = 240.0f;
constexpr float TURN_ACCEL_DPS2   = 1200.0f;

} // namespace cfg

#endif // APP_CONFIG_H
