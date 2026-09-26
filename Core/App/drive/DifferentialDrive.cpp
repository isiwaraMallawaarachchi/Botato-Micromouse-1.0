#include "DifferentialDrive.hpp"
#include "Config.h"

void DifferentialDrive::init() {
    // Left  = TIM1_CH1 (PA8), direction BIN1/BIN2 (PB14/PB15)
    left_.init(&htim1, TIM_CHANNEL_1,
               BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);
    // Right = TIM1_CH2 (PA9), direction AIN1/AIN2 (PB12/PB13)
    right_.init(&htim1, TIM_CHANNEL_2,
                AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin);
    stop();
}

void DifferentialDrive::setPwm(float fwd, float turn) {
    constexpr float MAXF = static_cast<float>(cfg::PWM_MAX);

    if (!(fwd == fwd))   fwd  = 0.0f;   // NaN guards
    if (!(turn == turn)) turn = 0.0f;

    if (turn >  MAXF) turn =  MAXF;
    if (turn < -MAXF) turn = -MAXF;

    const float room = MAXF - (turn >= 0.0f ? turn : -turn);
    if (fwd >  room) fwd =  room;
    if (fwd < -room) fwd = -room;

    setWheelPwm(static_cast<int32_t>(fwd - turn), static_cast<int32_t>(fwd + turn));
}

void DifferentialDrive::setWheelPwm(int32_t left, int32_t right) {
    if (left  >  cfg::PWM_MAX) left  =  cfg::PWM_MAX;
    if (left  < -cfg::PWM_MAX) left  = -cfg::PWM_MAX;
    if (right >  cfg::PWM_MAX) right =  cfg::PWM_MAX;
    if (right < -cfg::PWM_MAX) right = -cfg::PWM_MAX;
    lastLeft_  = left;
    lastRight_ = right;
    left_.setPwm(left);
    right_.setPwm(right);
}

void DifferentialDrive::stop() {
    lastLeft_ = lastRight_ = 0;
    left_.stop();
    right_.stop();
}
