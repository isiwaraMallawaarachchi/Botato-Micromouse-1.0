#ifndef APP_TOF_H
#define APP_TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * VL53L0X driver — properly multi-instance, correctly addressed.
 *
 * Adapted from the verified, complete VL53L0X init sequence (SPAD
 * configuration + full default tuning settings + reference
 * calibration) as published in ST's original API and widely-mirrored
 * community ports. Restructured here around a per-device struct so
 * all 5 sensors on this board operate independently and correctly —
 * unlike some published STM32 ports of this library, which hardcode a
 * single global I2C address into every transaction and therefore
 * cannot support multiple simultaneous sensors.
 *
 * RAM footprint: ~16 bytes per VL53L0X_Dev_t instance. For 5 sensors,
 * well under 100 bytes total persistent RAM. Local arrays used only
 * during one-time Init() (e.g. the 6-byte SPAD map) live on the stack
 * and are freed immediately after Init() returns.
 */

/* Filter configuration (TOF_MEDIAN_WINDOW, TOF_EMA_ALPHA) lives in
 * ToFConfig.h so every ToF setting is in one place.                   */
#include "ToFConfig.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            address;      /* 8-bit I2C address, already shifted */
    uint8_t            stop_variable;
    uint32_t           measurement_timing_budget_us;

    /* ---- Filter state (per sensor) ---- */
    uint16_t window[TOF_MEDIAN_WINDOW];  /* ring buffer of recent good raw reads */
    uint8_t  window_count;               /* how many valid samples collected so far */
    uint8_t  window_index;               /* next write position in the ring         */
    float    ema;                        /* exponential moving average (filtered mm) */
    bool     ema_valid;                  /* false until the first good reading lands */
    uint16_t last_raw_mm;                /* most recent raw reading (pre-filter)     */
    uint8_t  last_status;                /* range status of the most recent read     */
} VL53L0X_Dev_t;

/**
 * Full sensor bring-up: data init, static init (SPAD config + default
 * tuning settings), and reference calibration (VHV + phase).
 * Call this ONCE per sensor, after XSHUT has brought it up and BEFORE
 * any ranging calls. addr_8bit should be the sensor's CURRENT address
 * at the time this is called (usually the factory default 0x29<<1,
 * before you reassign it — reassign via VL53L0X_SetAddress() after
 * this succeeds, or pass the already-reassigned address if you
 * reassigned first).
 */
HAL_StatusTypeDef VL53L0X_Init(VL53L0X_Dev_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_8bit);

/**
 * Changes the sensor's I2C address and updates dev->address to match.
 * new_addr_8bit should already be left-shifted (7-bit address << 1).
 */
HAL_StatusTypeDef VL53L0X_SetAddress(VL53L0X_Dev_t *dev, uint8_t new_addr_8bit);

/**
 * Performs one single-shot ranging measurement and returns the result
 * in millimeters. Blocking — waits for the measurement to complete.
 * Returns HAL_TIMEOUT if the sensor doesn't respond within ~100ms.
 * Kept for compatibility; prefer continuous mode below for the mouse.
 */
HAL_StatusTypeDef VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev_t *dev, uint16_t *out_mm);

/**
 * Sets the measurement timing budget in microseconds — how long the
 * sensor integrates photons per reading. This is the master speed vs.
 * accuracy knob: lower = faster but noisier, higher = slower but more
 * accurate. Typical values: 20000 (fast), 33000 (balanced, default),
 * 66000 (accurate). Call before starting continuous mode.
 */
HAL_StatusTypeDef VL53L0X_SetTimingBudget(VL53L0X_Dev_t *dev, uint32_t budget_us);

/**
 * Starts back-to-back continuous ranging. After this, the sensor keeps
 * measuring on its own and you just read the latest result — no
 * per-read trigger, no blocking start-up latency. This is the main
 * speed win over single-shot. period_ms = 0 means "range as fast as the
 * timing budget allows" (back-to-back); a non-zero value paces readings
 * to that interval.
 */
HAL_StatusTypeDef VL53L0X_StartContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms);

/** Stops continuous ranging. */
HAL_StatusTypeDef VL53L0X_StopContinuous(VL53L0X_Dev_t *dev);

/**
 * Reads the latest continuous measurement and pushes it through the
 * filter chain: (1) range-status gating — bad-status readings are
 * rejected and never pollute the filter, (2) median-of-N to kill
 * outlier spikes, (3) light EMA to smooth residual jitter.
 *
 * out_mm receives the FILTERED distance. Return values:
 *   HAL_OK      fresh, good reading incorporated
 *   HAL_ERROR   fresh reading with bad status or out of range (out_mm keeps
 *               the last good filtered value)
 *   HAL_BUSY    no new measurement yet (non-blocking — safe every pass)
 *   HAL_TIMEOUT I2C transaction failed — a BUS fault, not a range fault.
 *               Kept distinct so callers never mistake a dying link for a
 *               sensor pointing at open air.
 *
 * Raw and status for the last read are also stored in dev->last_raw_mm
 * and dev->last_status for debugging / display.
 */
HAL_StatusTypeDef VL53L0X_ReadRangeContinuousFiltered(VL53L0X_Dev_t *dev, uint16_t *out_mm);

#ifdef __cplusplus
}
#endif

#endif /* APP_TOF_H */
