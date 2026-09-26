#include "Motor.hpp"
#include "Config.h"
#include "MotorConfig.h"

void Motor::init(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel,
                 GPIO_TypeDef* in1Port, uint16_t in1Pin,
                 GPIO_TypeDef* in2Port, uint16_t in2Pin) {
    pwmTim_     = pwmTim;
    pwmChannel_ = pwmChannel;
    in1Port_ = in1Port; in1Pin_ = in1Pin;
    in2Port_ = in2Port; in2Pin_ = in2Pin;
    HAL_TIM_PWM_Start(pwmTim_, pwmChannel_);
    stop();
}

void Motor::setPwm(int32_t pwm) {
    if (pwm >  cfg::PWM_MAX) pwm =  cfg::PWM_MAX;
    if (pwm < -cfg::PWM_MAX) pwm = -cfg::PWM_MAX;

    // Dead-band: map any non-zero command into [DEADBAND, PWM_MAX].
    constexpr int32_t span = cfg::PWM_MAX - motorcfg::DEADBAND;
    if (pwm > 0)      pwm =  motorcfg::DEADBAND + pwm * span / cfg::PWM_MAX;
    else if (pwm < 0) pwm = -motorcfg::DEADBAND + pwm * span / cfg::PWM_MAX;

    // Inverted polarity: forward = IN1 LOW, IN2 HIGH (DECISIONS.md #15).
    if (pwm >= 0) {
        HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(pwmTim_, pwmChannel_, static_cast<uint32_t>(pwm));
    } else {
        HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(pwmTim_, pwmChannel_, static_cast<uint32_t>(-pwm));
    }
}

void Motor::stop() {
    HAL_GPIO_WritePin(in1Port_, in1Pin_, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2Port_, in2Pin_, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(pwmTim_, pwmChannel_, 0);
}
