#ifndef APP_GYRO_HPP
#define APP_GYRO_HPP

#include <cstdint>
extern "C" {
#include "i2c.h"
}

/*
 * Gyro — MPU6050 Z-axis yaw on the dedicated I2C2 bus (DECISIONS.md #16).
 *
 * Every I2C2 transaction after init() happens inside update(), in the 1kHz
 * ISR — including bias calibration. Calibration used to be a blocking loop in
 * the main context while the ISR also read I2C2, so the two could collide
 * mid-transaction. It also froze the main loop, which is why the LED could not
 * blink during calibration. Now beginCalibration() only raises a flag.
 *
 * Heading is ANTICLOCKWISE-POSITIVE, continuous and unbounded (never wrapped).
 */
class Gyro {
public:
    bool init(I2C_HandleTypeDef* hi2c);   // blocking, main context, once
    void beginCalibration();              // non-blocking; robot must be still
    void update();                        // 1kHz ISR

    bool  ok()          const { return ok_; }
    bool  calibrating() const { return calibrating_; }
    bool  calibrated()  const { return calibrated_; }
    float rateDps()     const { return rateDps_; }
    float angleDeg()    const { return angleDeg_; }
    float bias()        const { return bias_; }
    uint8_t  whoAmI()   const { return whoAmI_; }
    uint32_t readErrors() const { return readErrors_; }

    void zeroAngle() { angleDeg_ = 0.0f; }   // call inside a CriticalSection

private:
    I2C_HandleTypeDef* hi2c_ = nullptr;
    bool     ok_          = false;
    volatile bool calibrating_ = false;
    bool     calibrated_  = false;
    uint8_t  whoAmI_      = 0;
    float    bias_        = 0.0f;
    float    rateDps_     = 0.0f;
    float    angleDeg_    = 0.0f;
    float    calSum_      = 0.0f;
    int      calTicks_    = 0;
    uint32_t readErrors_  = 0;

    bool readRawZ(int16_t& out) const;
};

#endif // APP_GYRO_HPP
