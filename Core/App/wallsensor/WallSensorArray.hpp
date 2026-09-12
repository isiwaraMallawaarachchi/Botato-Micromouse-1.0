#ifndef APP_WALLSENSORARRAY_HPP
#define APP_WALLSENSORARRAY_HPP

#include <cstdint>

extern "C" {
#include "i2c.h"
}

/*
 * WallSensorArray — the 5 VL53L0X sensors as one object.
 * Wraps the C driver in tof.c/.h (XSHUT sequencing, continuous mode,
 * median+EMA filter) and adds the per-sensor linear calibration.
 *
 * Validity: ok(i) means "this distance is a fresh, in-range measurement",
 * NOT merely "the sensor booted". An out-of-range sensor reports
 * TOF_NO_TARGET_MM (far), never 0 — a 0 would read as "wall touching us".
 *
 * Index order (cfg::ToFIndex): LEFT, LEFTFRONT, FRONT, RIGHTFRONT, RIGHT.
 */
class WallSensorArray {
public:
    void init(I2C_HandleTypeDef* hi2c);
    void poll();

    float distanceMm(int i) const { return distMm_[i]; }   // calibrated, mm
    bool  ok(int i)         const { return valid_[i]; }    // fresh + in range
    bool  present(int i)    const { return present_[i]; }  // initialised at boot
    bool  wallPresent(int i) const;

private:
    I2C_HandleTypeDef* hi2c_ = nullptr;
    float    distMm_[5]     = {0,0,0,0,0};
    bool     present_[5]    = {false,false,false,false,false};
    bool     valid_[5]      = {false,false,false,false,false};
    uint32_t lastGoodMs_[5] = {0,0,0,0,0};
};

#endif // APP_WALLSENSORARRAY_HPP
