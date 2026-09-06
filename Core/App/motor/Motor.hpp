#ifndef APP_MOTOR_HPP
#define APP_MOTOR_HPP

#include <cstdint>
extern "C" {
#include "tim.h"
#include "gpio.h"
}

/*
 * Motor — one TB6612 channel: one PWM timer channel + two direction pins.
 * Direction polarity is INVERTED vs the datasheet truth table to match this
 * board's motor lead wiring (DECISIONS.md #15). Applies dead-band so small
 * commands still overcome static friction.
 */
class Motor {
public:
    void init(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel,
              GPIO_TypeDef* in1Port, uint16_t in1Pin,
              GPIO_TypeDef* in2Port, uint16_t in2Pin);

    // pwm in [-PWM_MAX, +PWM_MAX]. Positive = physical forward.
    void setPwm(int16_t pwm);
    void stop();   // coast (both direction pins low)

private:
    TIM_HandleTypeDef* pwmTim_ = nullptr;
    uint32_t      pwmChannel_ = 0;
    GPIO_TypeDef* in1Port_ = nullptr; uint16_t in1Pin_ = 0;
    GPIO_TypeDef* in2Port_ = nullptr; uint16_t in2Pin_ = 0;
};

#endif // APP_MOTOR_HPP
