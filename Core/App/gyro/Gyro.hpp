#ifndef APP_GYRO_HPP
#define APP_GYRO_HPP

#include <cstdint>
extern "C" {
#include "i2c.h"
}

/*
 * Gyro — MPU6050 Z-axis yaw, on the dedicated I2C2 bus.
 * Bias-calibrated at startup, oversampled per read, integrated to an angle.
 * init() and calibrate() are blocking and run once; readRate()/integrate()
 * are called each control tick.
 */
class Gyro {
public:
    // Returns false if the sensor doesn't answer / wrong WHO_AM_I.
    bool init(I2C_HandleTypeDef* hi2c);

    // Blocking ~1s bias calibration. Keep the robot still.
    void calibrate();

    // One control-tick update: reads (oversampled), stores rate, integrates yaw.
    void update();

    float rateDps()  const { return rateDps_; }   // deg/s, bias-removed
    float angleDeg() const { return angleDeg_; }   // integrated heading, deg

    void  zeroAngle() { angleDeg_ = 0.0f; }
    bool  ok()       const { return ok_; }
    uint8_t whoAmI() const { return whoAmI_; }

private:
    I2C_HandleTypeDef* hi2c_ = nullptr;
    bool    ok_       = false;
    uint8_t whoAmI_   = 0;
    float   bias_     = 0.0f;
    float   rateDps_  = 0.0f;
    float   angleDeg_ = 0.0f;

    bool readRawZ(int16_t& out) const;   // one raw Z sample
};

#endif // APP_GYRO_HPP
