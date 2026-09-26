#include "Gyro.hpp"
#include "Config.h"
#include "GyroConfig.h"

namespace {
    constexpr uint16_t ADDR            = gyrocfg::ADDR_7B << 1;
    constexpr uint8_t  REG_WHO_AM_I    = 0x75;
    constexpr uint8_t  REG_PWR_MGMT_1  = 0x6B;
    constexpr uint8_t  REG_CONFIG      = 0x1A;
    constexpr uint8_t  REG_GYRO_CONFIG = 0x1B;
    constexpr uint8_t  REG_GYRO_ZOUT_H = 0x47;
}

bool Gyro::init(I2C_HandleTypeDef* hi2c) {
    hi2c_ = hi2c;
    ok_   = false;
    lost_ = false;
    failStreak_ = 0;
    calibrating_ = false;

    if (HAL_I2C_IsDeviceReady(hi2c_, ADDR, 3, 100) != HAL_OK) return false;
    if (HAL_I2C_Mem_Read(hi2c_, ADDR, REG_WHO_AM_I, 1, &whoAmI_, 1, 100) != HAL_OK)
        return false;
    if (whoAmI_ != gyrocfg::ADDR_7B) return false;

    uint8_t d = 0x00;   // wake
    if (HAL_I2C_Mem_Write(hi2c_, ADDR, REG_PWR_MGMT_1, 1, &d, 1, 100) != HAL_OK) return false;
    d = gyrocfg::GYRO_CONFIG_VAL;
    if (HAL_I2C_Mem_Write(hi2c_, ADDR, REG_GYRO_CONFIG, 1, &d, 1, 100) != HAL_OK) return false;
    d = gyrocfg::DLPF_VAL;
    if (HAL_I2C_Mem_Write(hi2c_, ADDR, REG_CONFIG, 1, &d, 1, 100) != HAL_OK) return false;

    ok_ = true;
    return true;
}

void Gyro::beginCalibration() {
    if (!ok_) return;
    calSum_   = 0.0f;
    calTicks_ = 0;
    calibrating_ = true;          // set last: the ISR starts on this flag
}

bool Gyro::readRawZ(int16_t& out) const {
    uint8_t b[2];
    if (HAL_I2C_Mem_Read(hi2c_, ADDR, REG_GYRO_ZOUT_H, 1, b, 2,
                         gyrocfg::READ_TIMEOUT_MS) != HAL_OK)
        return false;
    out = static_cast<int16_t>((b[0] << 8) | b[1]);
    return true;
}

void Gyro::update() {
    ++updates_;
    if (!ok_) return;

    int32_t sum = 0;
    int     n   = 0;
    for (int i = 0; i < gyrocfg::OVERSAMPLE; ++i) {
        int16_t z;
        if (!readRawZ(z)) { ++readErrors_; break; }   // don't stall the tick
        sum += z;
        ++n;
    }
    if (n == 0) {
        // Every read failed. Give up after LOST_TICKS in a row so the robot
        // reports a fault instead of calibrating (or steering) forever.
        if (++failStreak_ >= gyrocfg::LOST_TICKS) {
            ok_ = false;
            lost_ = true;
            calibrating_ = false;
        }
        return;
    }
    failStreak_ = 0;

    const float raw = static_cast<float>(sum) / static_cast<float>(n);

    if (calibrating_) {
        calSum_ += raw;
        if (++calTicks_ >= gyrocfg::CAL_TICKS) {
            bias_        = calSum_ / static_cast<float>(calTicks_);
            rateDps_     = 0.0f;
            angleDeg_    = 0.0f;
            calibrated_  = true;
            calibrating_ = false;
        }
        return;                    // no integration while calibrating
    }

    rateDps_   = (raw - bias_) / gyrocfg::LSB_PER_DPS;
    angleDeg_ += rateDps_ * cfg::CONTROL_DT_S;
}
