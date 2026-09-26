#ifndef APP_DIFFERENTIALDRIVE_HPP
#define APP_DIFFERENTIALDRIVE_HPP

#include <cstdint>
#include "Motor.hpp"

/*
 * DifferentialDrive — owns both motors, mixes forward + turn into wheel PWM:
 *     L = fwd - turn,   R = fwd + turn
 *
 * Takes float so no caller can overflow a narrow integer before the clamp
 * (the old int16 cast wrapped a large turn command into the opposite sign).
 *
 * Steering keeps priority when saturated: if |fwd| + |turn| would exceed
 * PWM_MAX, forward is reduced rather than clipping one wheel. Clipping one
 * side made steering one-sided at speed (DECISIONS.md #26).
 */
class DifferentialDrive {
public:
    void init();

    void setPwm(float fwdPwm, float turnPwm);     // mixed, used by ControlLoop
    void setWheelPwm(int32_t left, int32_t right); // direct, used by TEST_MOTOR
    void stop();

    int32_t lastLeftPwm()  const { return lastLeft_; }
    int32_t lastRightPwm() const { return lastRight_; }

private:
    Motor   left_;
    Motor   right_;
    int32_t lastLeft_  = 0;
    int32_t lastRight_ = 0;
};

#endif // APP_DIFFERENTIALDRIVE_HPP
