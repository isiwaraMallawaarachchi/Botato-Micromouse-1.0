#ifndef APP_WALLSENSORARRAY_HPP
#define APP_WALLSENSORARRAY_HPP

#include <cstdint>

extern "C" {
#include "i2c.h"
}

/*
 * WallSensorArray — the 5 VL53L0X sensors as one object.
 * Wraps the C driver in tof.c/.h (XSHUT sequencing, continuous mode,
 * median+EMA filter) and adds the per-sensor linear calibration fitted
 * on the bench. Reads are non-blocking; call poll() each main-loop pass.
 *
 * Index order (cfg::ToFIndex): LEFT, LEFTFRONT, FRONT, RIGHTFRONT, RIGHT.
 */
class WallSensorArray {
public:
    // Full bring-up on I2C1: address assignment, driver init, continuous mode.
    void init(I2C_HandleTypeDef* hi2c);

    // Non-blocking: pulls any fresh filtered reading and calibrates it.
    void poll();

    float distanceMm(int i) const { return distMm_[i]; }   // calibrated, mm
    bool  ok(int i)         const { return ok_[i]; }
    bool  wallPresent(int i) const;                         // < WALL_PRESENT_MM

private:
    I2C_HandleTypeDef* hi2c_ = nullptr;
    float distMm_[5] = {0,0,0,0,0};
    bool  ok_[5]     = {false,false,false,false,false};
};

#endif // APP_WALLSENSORARRAY_HPP
