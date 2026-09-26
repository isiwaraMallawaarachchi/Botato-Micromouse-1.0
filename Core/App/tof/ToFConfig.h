#ifndef APP_TOFCONFIG_H
#define APP_TOFCONFIG_H

/*
 * ToFConfig.h — everything specific to the VL53L0X sensors.
 * C-compatible top half (read by tof.c), C++ bottom half (WallSensorArray,
 * tests).
 */

/* ---- Speed vs accuracy ---------------------------------------------------
 * TIMING_BUDGET: photon integration per measurement. 20000us is the sensor's
 * minimum and gives ~45 samples/s per sensor. Raise toward 33000 for less
 * noise at the cost of rate.
 *
 * Filter, applied to every good sample:
 *   status gate -> median of MEDIAN_WINDOW -> EMA with EMA_ALPHA
 * Median 3 rejects single spikes with one sample of lag; EMA 0.6 smooths
 * what is left while still following a wall that appears.                  */
#define TOF_TIMING_BUDGET_US  20000u
#define TOF_MEDIAN_WINDOW     3
#define TOF_EMA_ALPHA         0.6f

#ifdef __cplusplus
#include <cstdint>

namespace tofcfg {

constexpr uint32_t TIMING_BUDGET_US = TOF_TIMING_BUDGET_US;
constexpr uint32_t SAMPLE_PERIOD_MS = TOF_TIMING_BUDGET_US / 1000u;

// A sensor is not polled again until its next measurement is due. Saves I2C
// time; the few ms of margin covers the sensor's own overhead.
constexpr uint32_t POLL_MARGIN_MS   = 2;

// No valid sample for this long -> the reading is stale and reported as
// no-target. ~4 missed measurement periods.
constexpr uint32_t STALE_MS         = 4 * SAMPLE_PERIOD_MS;

// No-target sentinel. Far, never 0 — 0 would read as "wall on the nose".
constexpr float    NO_TARGET_MM     = 8190.0f;

// XSHUT boot sequencing, index order = cfg::ToFIndex.
constexpr uint8_t  ADDR[5]             = { 0x30, 0x31, 0x32, 0x33, 0x34 };
constexpr uint32_t XSHUT_BOOT_DELAY_MS = 10;

/* ===========================================================================
 * >>> CALIBRATION — the single place for the per-sensor fit <<<
 *
 *   true distance = CAL_A[i] * filtered + CAL_B[i]
 *
 * "true" = millimetres from the sensor's lens face to the target. From
 * TEST_TOF_CAL (tofCal.fitA[] / tofCal.fitB[]), 6 points 30..200mm against a
 * flat barrier. Redo it if a sensor is remounted, replaced or re-covered.
 * Index: 0 Left, 1 LeftFront, 2 Front, 3 RightFront, 4 Right.
 *
 * Fit quality (worst point, mm):  L 2.1  LF 3.6  F 3.0  RF 2.8  R 2.2
 *                                 (RF and R recaptured in a second session)
 * Error at working distances:     front stop 34mm +1.0, side walls 47mm
 *                                 +0.1 (L) / +0.3 (R), front wall 64mm -2.2
 * =========================================================================== */
constexpr float CAL_A[5] = { 0.921611f, 0.942740f, 0.920044f, 0.925280f, 0.982126f };
constexpr float CAL_B[5] = { -18.6685f, -20.2615f, -13.8093f,  -1.9881f, -20.7765f };

} // namespace tofcfg
#endif // __cplusplus

#endif // APP_TOFCONFIG_H
