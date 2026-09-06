#include "DifferentialDrive.hpp"
#include "Config.h"
#include "tim.h"
#include "gpio.h"

void DifferentialDrive::init() {
    // Left  = TIM1_CH1 (PA8), dir BIN1/BIN2 (PB14/PB15)
    left_.init(&htim1, TIM_CHANNEL_1,
               BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);
    // Right = TIM1_CH2 (PA9), dir AIN1/AIN2 (PB12/PB13)
    right_.init(&htim1, TIM_CHANNEL_2,
                AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin);
    stop();
}

static inline int16_t clampPwm(int v) {
    if (v >  cfg::PWM_MAX) return  cfg::PWM_MAX;
    if (v < -cfg::PWM_MAX) return -cfg::PWM_MAX;
    return static_cast<int16_t>(v);
}

void DifferentialDrive::setPwm(int16_t fwdPwm, int16_t turnPwm) {
    lastLeft_  = clampPwm(fwdPwm - turnPwm);
    lastRight_ = clampPwm(fwdPwm + turnPwm);
    left_.setPwm(lastLeft_);
    right_.setPwm(lastRight_);
}

void DifferentialDrive::stop() {
    lastLeft_ = lastRight_ = 0;
    left_.stop();
    right_.stop();
}
