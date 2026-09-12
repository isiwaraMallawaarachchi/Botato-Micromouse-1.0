#include "WallSensorArray.hpp"
#include "Config.h"

// tof.c is C; include its header with C linkage.
extern "C" {
    #include "tof.h"
    #include "i2c.h"
}

namespace {
    constexpr uint16_t VL53_DEFAULT_ADDR   = (0x29 << 1);
    constexpr uint8_t  VL53_ADDR_REG       = 0x8A;
    constexpr uint32_t XSHUT_BOOT_DELAY_MS = 10;

    // Per-sensor linear calibration: true = a*filtered + b. Bench-fitted.
    // Index: 0=Left 1=LeftFront 2=Front 3=RightFront 4=Right.
    constexpr float CAL_A[5] = { 0.89823f, 0.94864f, 0.95607f, 0.95485f, 0.97145f };
    constexpr float CAL_B[5] = { -1.3703f,  2.2047f, -3.6470f, -70.9937f, -6.4211f };

    const uint8_t TARGET_ADDR[5] = { 0x30, 0x31, 0x32, 0x33, 0x34 };

    struct XPin { GPIO_TypeDef* port; uint16_t pin; };
    // Matches PINOUT.md XSHUT order. Right XSHUT is on PB8.
    XPin xshut[5];   // filled in init() from generated pin macros

    VL53L0X_Dev_t dev[5];
}

void WallSensorArray::init(I2C_HandleTypeDef* hi2c) {
    hi2c_ = hi2c;

    xshut[cfg::TOF_LEFT]       = { XSHUT_LEFT_GPIO_Port,  XSHUT_LEFT_Pin  };
    xshut[cfg::TOF_LEFTFRONT]  = { XSHUT_LF_GPIO_Port,    XSHUT_LF_Pin    };
    xshut[cfg::TOF_FRONT]      = { XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin };
    xshut[cfg::TOF_RIGHTFRONT] = { XSHUT_RF_GPIO_Port,    XSHUT_RF_Pin    };
    xshut[cfg::TOF_RIGHT]      = { XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin };

    for (int i = 0; i < 5; ++i) {
        distMm_[i]  = cfg::TOF_NO_TARGET_MM;   // never start at 0
        present_[i] = false;
        valid_[i]   = false;
    }

    // Disable all, then bring up one at a time and assign a unique address.
    for (int i = 0; i < 5; ++i)
        HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
    HAL_Delay(XSHUT_BOOT_DELAY_MS);

    for (int i = 0; i < 5; ++i) {
        HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_SET);
        HAL_Delay(XSHUT_BOOT_DELAY_MS);

        if (HAL_I2C_IsDeviceReady(hi2c_, VL53_DEFAULT_ADDR, 2, 100) != HAL_OK) {
            HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
            continue;
        }
        uint8_t addr = TARGET_ADDR[i];
        if (HAL_I2C_Mem_Write(hi2c_, VL53_DEFAULT_ADDR, VL53_ADDR_REG,
                              1, &addr, 1, 100) != HAL_OK) {
            HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
            continue;
        }
        HAL_Delay(2);
        if (VL53L0X_Init(&dev[i], hi2c_, static_cast<uint8_t>(addr << 1)) != HAL_OK) {
            HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
            continue;
        }
        VL53L0X_SetTimingBudget(&dev[i], cfg::TOF_TIMING_BUDGET_US);
        VL53L0X_StartContinuous(&dev[i], 0);
        present_[i] = true;   // NEW
    }
}

void WallSensorArray::poll() {
    const uint32_t now = HAL_GetTick();

    for (int i = 0; i < 5; ++i) {
        if (!present_[i]) { valid_[i] = false; continue; }

        uint16_t filtered = 0;
        const HAL_StatusTypeDef st =
            VL53L0X_ReadRangeContinuousFiltered(&dev[i], &filtered);

        if (st == HAL_OK) {
            float c = CAL_A[i] * static_cast<float>(filtered) + CAL_B[i];
            if (c < 0.0f) c = 0.0f;
            distMm_[i]     = c;
            lastGoodMs_[i] = now;
            valid_[i]      = true;
        }
        else if (st == HAL_ERROR) {
            // Out of range / no target / bad status. Report FAR, not 0:
            // a 0 here would read as "wall against the nose" and end a cell.
            distMm_[i] = cfg::TOF_NO_TARGET_MM;
            valid_[i]  = false;
        }
        else {
            // HAL_BUSY — no new sample yet. Keep the last value, but expire
            // it so a stale reading can't masquerade as current.
            if ((now - lastGoodMs_[i]) > cfg::TOF_STALE_MS) {
                distMm_[i] = cfg::TOF_NO_TARGET_MM;
                valid_[i]  = false;
            }
        }
    }
}

bool WallSensorArray::wallPresent(int i) const {
    return valid_[i] && distMm_[i] < cfg::WALL_PRESENT_MM;
}
