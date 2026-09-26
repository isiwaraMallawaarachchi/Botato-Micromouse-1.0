#ifndef APP_TESTCONFIG_H
#define APP_TESTCONFIG_H

#include <cstdint>
#include "Config.h"

/* Parameters for the test build only (Core/Src/main_test.cpp). */
namespace testcfg {

/* TEST_TOF_SINGLE — one sensor alone on I2C1. A lone breakout gives the bus a
 * single ~10k pull-up, marginal at 400kHz. If the sensor is missing or
 * busErrors climb, set 100000 before blaming the sensor. 0 = keep the .ioc. */
constexpr uint32_t TOF1_FORCE_BUS_HZ = 0;

/* TEST_TOF_CAL — distances from the sensor's LENS FACE to the barrier. */
constexpr int   CAL_POINTS = 6;
constexpr float CAL_POINTS_MM[CAL_POINTS] = { 30.0f, 50.0f, 80.0f, 120.0f, 160.0f, 200.0f };
constexpr int   CAL_SAMPLES = 40;          // averaged per point (~1s)

/* TEST_ENCODER — push the robot exactly this far along a ruler. */
constexpr float ENC_PUSH_MM = 500.0f;

/* TEST_MOTOR — open-loop PWM per phase, wheels OFF the ground. */
constexpr int32_t  MOTOR_TEST_PWM = 1500;
constexpr uint32_t MOTOR_PHASE_MS = 1500;

/* TEST_TURN — cycles of: L90, R90, R90, L90, 180, 180. */
constexpr int      TURN_CYCLES   = 3;
constexpr uint32_t TURN_PAUSE_MS = 400;

/* TEST_DRIVE_CELLS — straight run length and cruise speed. */
constexpr int   DRIVE_CELLS      = 4;
constexpr float DRIVE_SPEED_MMPS = cfg::SEARCH_SPEED_MMPS;

} // namespace testcfg

#endif // APP_TESTCONFIG_H
