#include "Gyro.hpp"
#include "Config.h"
#include "i2c.h"      // hi2c2, HAL_I2C_* (C linkage)

namespace {
    constexpr uint16_t MPU_ADDR        = (0x68 << 1);  // AD0 -> GND
    constexpr uint8_t  REG_WHO_AM_I    = 0x75;
    constexpr uint8_t  REG_PWR_MGMT_1  = 0x6B;
    constexpr uint8_t  REG_CONFIG      = 0x1A;  // DLPF
    constexpr uint8_t  REG_GYRO_CONFIG = 0x1B;
    constexpr uint8_t  REG_GYRO_ZOUT_H = 0x47;
}

bool Gyro::init(I2C_HandleTypeDef* hi2c) {
    hi2c_ = hi2c;
    ok_   = false;

    if (HAL_I2C_IsDeviceReady(hi2c_, MPU_ADDR, 3, 100) != HAL_OK) return false;

    if (HAL_I2C_Mem_Read(hi2c_, MPU_ADDR, REG_WHO_AM_I, 1, &whoAmI_, 1, 100) != HAL_OK)
        return false;
    if (whoAmI_ != 0x68) return false;

    uint8_t d;
    d = 0x00;  // wake
    if (HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, REG_PWR_MGMT_1, 1, &d, 1, 100) != HAL_OK) return false;
    d = 0x08;  // gyro +/-500 dps
    if (HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, REG_GYRO_CONFIG, 1, &d, 1, 100) != HAL_OK) return false;
    d = 0x03;  // DLPF ~44Hz
    if (HAL_I2C_Mem_Write(hi2c_, MPU_ADDR, REG_CONFIG, 1, &d, 1, 100) != HAL_OK) return false;

    ok_ = true;
    return true;
}

bool Gyro::readRawZ(int16_t& out) const {
    uint8_t b[2];
    if (HAL_I2C_Mem_Read(hi2c_, MPU_ADDR, REG_GYRO_ZOUT_H, 1, b, 2, 2) != HAL_OK)
        return false;
    out = static_cast<int16_t>((b[0] << 8) | b[1]);
    return true;
}

void Gyro::calibrate() {
    if (!ok_) return;
    int32_t  sum = 0;
    uint16_t n   = 0;
    for (int i = 0; i < cfg::GYRO_CAL_SAMPLES; ++i) {
        int16_t z;
        if (readRawZ(z)) { sum += z; ++n; }
        HAL_Delay(1);
    }
    if (n > 0) bias_ = static_cast<float>(sum) / static_cast<float>(n);
}

void Gyro::update() {
    if (!ok_) return;

    // Oversample: sum N raw reads, then average (guide 4.2).
    int32_t sum = 0; int n = 0;
    for (int i = 0; i < cfg::GYRO_OVERSAMPLE; ++i) {
        int16_t z;
        if (readRawZ(z)) { sum += z; ++n; }
    }
    if (n == 0) return;

    float rawAvg = static_cast<float>(sum) / static_cast<float>(n);
    rateDps_  = (rawAvg - bias_) / cfg::GYRO_SENSITIVITY;
    angleDeg_ += rateDps_ * cfg::CONTROL_DT_S;
}
