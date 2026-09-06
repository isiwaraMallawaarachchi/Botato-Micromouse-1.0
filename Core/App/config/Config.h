#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * Config.h — single source of truth for all physical + tuning constants.
 * Edit values here; never hard-code magic numbers elsewhere.
 */

// ---- Physical drivetrain --------------------------------------------------
namespace cfg {

// N20 6V 200rpm, 50:1 gearbox, 7 PPR magnetic encoder, 4x quadrature.
constexpr float ENC_PPR          = 7.0f;
constexpr float GEAR_RATIO       = 50.0f;
constexpr float QUADRATURE       = 4.0f;
constexpr float COUNTS_PER_REV   = ENC_PPR * GEAR_RATIO * QUADRATURE;   // 1400

constexpr float WHEEL_DIAMETER_MM = 43.0f;
constexpr float PI_F              = 3.14159265f;
constexpr float WHEEL_CIRCUM_MM   = PI_F * WHEEL_DIAMETER_MM;           // ~135.09
constexpr float MM_PER_TICK       = WHEEL_CIRCUM_MM / COUNTS_PER_REV;   // ~0.0965

constexpr float WHEELBASE_MM      = 87.0f;   // contact-point to contact-point

// ---- Maze geometry (IEEE standard; overridable for a test maze) -----------
constexpr float CELL_SIZE_MM      = 180.0f;  // 18cm cell

// ---- Control loop ---------------------------------------------------------
constexpr float CONTROL_DT_S      = 0.001f;  // TIM3 @ 1kHz
constexpr int   PWM_MAX           = 4999;    // TIM1 ARR

// Motor dead-band: minimum PWM that actually moves the wheel. Tune on bench.
constexpr int   MOTOR_DEADBAND    = 500;       // start 0, raise if wheels stall

// ---- Gyro -----------------------------------------------------------------
// FS_SEL=1 -> +/-500 dps, 65.5 LSB per deg/s.
constexpr float GYRO_SENSITIVITY  = 65.5f;
constexpr int   GYRO_CAL_SAMPLES  = 5000;    // bias calibration sample count
constexpr int   GYRO_OVERSAMPLE   = 4;       // reads summed per tick

// ---- ToF ------------------------------------------------------------------
constexpr uint32_t TOF_TIMING_BUDGET_US = 33000;   // balanced speed/accuracy
constexpr float    TOF_EMA_ALPHA        = 0.5f;
constexpr int      TOF_MEDIAN_WINDOW    = 5;
// A wall is "present" if a side sensor reads closer than this (mm).
constexpr float    WALL_PRESENT_MM      = 100.0f;
// Readings beyond this are treated as "no wall" and gated out (mm).
constexpr float    MAX_WALL_DETECT_MM   = 120.0f;

// ToF sensor indices (match XSHUT boot order + PINOUT.md).
enum ToFIndex { TOF_LEFT = 0, TOF_LEFTFRONT, TOF_FRONT, TOF_RIGHTFRONT, TOF_RIGHT };

// ---- Buttons --------------------------------------------------------------
constexpr uint32_t BTN_LONGPRESS_MS = 3000;  // >3s = calibration
constexpr uint32_t BTN_DEBOUNCE_MS  = 30;

} // namespace cfg

#endif // APP_CONFIG_H
