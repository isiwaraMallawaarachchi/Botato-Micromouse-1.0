#include "WallSensorArray.hpp"
#include "Config.h"
#include "ToFConfig.h"

namespace {
    constexpr uint16_t DEFAULT_ADDR = 0x29 << 1;
    constexpr uint8_t  REG_ADDR     = 0x8A;
}

WallSensorArray::XshutPin WallSensorArray::xshutPin(int i) {
    // PINOUT.md. XSHUT_RIGHT is PB8, not PB10 (PB10 is I2C2_SCL).
    switch (i) {
        case cfg::TOF_LEFT:       return { XSHUT_LEFT_GPIO_Port,  XSHUT_LEFT_Pin  };
        case cfg::TOF_LEFTFRONT:  return { XSHUT_LF_GPIO_Port,    XSHUT_LF_Pin    };
        case cfg::TOF_FRONT:      return { XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin };
        case cfg::TOF_RIGHTFRONT: return { XSHUT_RF_GPIO_Port,    XSHUT_RF_Pin    };
        default:                  return { XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin };
    }
}

void WallSensorArray::markInvalid(int i) {
    distMm_[i] = tofcfg::NO_TARGET_MM;
    valid_[i]  = false;
}

void WallSensorArray::init(I2C_HandleTypeDef* hi2c) {
    hi2c_ = hi2c;

    for (int i = 0; i < cfg::TOF_COUNT; ++i) {
        present_[i] = false;
        markInvalid(i);
        const XshutPin x = xshutPin(i);
        HAL_GPIO_WritePin(x.port, x.pin, GPIO_PIN_RESET);   // all into standby
    }
    HAL_Delay(tofcfg::XSHUT_BOOT_DELAY_MS);

    // Release one at a time: it boots at 0x29, is moved to its own address,
    // and only then is the next one released.
    for (int i = 0; i < cfg::TOF_COUNT; ++i) {
        const XshutPin x = xshutPin(i);
        HAL_GPIO_WritePin(x.port, x.pin, GPIO_PIN_SET);
        HAL_Delay(tofcfg::XSHUT_BOOT_DELAY_MS);

        uint8_t addr = tofcfg::ADDR[i];
        bool ok = HAL_I2C_IsDeviceReady(hi2c_, DEFAULT_ADDR, 2, 100) == HAL_OK &&
                  HAL_I2C_Mem_Write(hi2c_, DEFAULT_ADDR, REG_ADDR, 1, &addr, 1, 100) == HAL_OK;
        if (ok) {
            HAL_Delay(2);
            ok = VL53L0X_Init(&dev_[i], hi2c_, static_cast<uint8_t>(addr << 1)) == HAL_OK &&
                 VL53L0X_SetTimingBudget(&dev_[i], tofcfg::TIMING_BUDGET_US) == HAL_OK &&
                 VL53L0X_StartContinuous(&dev_[i], 0) == HAL_OK;
        }

        if (!ok) {
            // Hold a failed sensor in standby so it cannot reappear at 0x29
            // and collide with the next one being addressed.
            HAL_GPIO_WritePin(x.port, x.pin, GPIO_PIN_RESET);
            continue;
        }
        present_[i] = true;
    }
}

int WallSensorArray::presentCount() const {
    int n = 0;
    for (int i = 0; i < cfg::TOF_COUNT; ++i) if (present_[i]) ++n;
    return n;
}

void WallSensorArray::poll() {
    const uint32_t now = HAL_GetTick();

    for (int i = 0; i < cfg::TOF_COUNT; ++i) {
        if (!present_[i]) { markInvalid(i); continue; }

        // Stale check runs every pass, even when the sensor is not due.
        if (valid_[i] && (now - lastGoodMs_[i]) > tofcfg::STALE_MS) markInvalid(i);

        if (static_cast<int32_t>(now - nextDueMs_[i]) < 0) continue;   // not due

        uint16_t filtered = 0;
        const HAL_StatusTypeDef st = VL53L0X_ReadRangeContinuousFiltered(&dev_[i], &filtered);

        if (st == HAL_BUSY) continue;                      // check again next pass
        if (st == HAL_TIMEOUT) { ++busErrors_[i]; continue; }

        // A measurement was consumed (good or bad): the next is one period away.
        ++samples_[i];
        lastSampleMs_[i] = now;
        nextDueMs_[i]    = now + tofcfg::SAMPLE_PERIOD_MS - tofcfg::POLL_MARGIN_MS;

        if (st == HAL_OK) {
            filtMm_[i] = filtered;
            float c = tofcfg::CAL_A[i] * static_cast<float>(filtered) + tofcfg::CAL_B[i];
            if (c < 0.0f) c = 0.0f;         // genuinely closer than the fit covers
            distMm_[i]     = c;
            valid_[i]      = true;
            lastGoodMs_[i] = now;
        } else {
            markInvalid(i);                 // no target / bad status: far, not 0
        }
    }
}
