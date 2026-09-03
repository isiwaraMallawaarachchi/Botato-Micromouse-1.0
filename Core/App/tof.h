#ifndef APP_TOF_H
#define APP_TOF_H

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

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            address;      /* 8-bit I2C address, already shifted */
    uint8_t            stop_variable;
    uint32_t           measurement_timing_budget_us;
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
 */
HAL_StatusTypeDef VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev_t *dev, uint16_t *out_mm);

#endif /* APP_TOF_H */
