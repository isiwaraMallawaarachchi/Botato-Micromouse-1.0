#ifndef APP_DIFFERENTIALDRIVE_HPP
#define APP_DIFFERENTIALDRIVE_HPP

#include "Motor.hpp"

/*
 * DifferentialDrive — owns both motors, mixes a forward + turn command into
 * left/right PWM (guide section 2):  L = fwd - turn,  R = fwd + turn.
 * This is the only object the control loop talks to for actuation.
 */
class DifferentialDrive {
public:
    void init();

    // Both in raw PWM units. fwd = common-mode, turn = differential.
    void setPwm(int16_t fwdPwm, int16_t turnPwm);
    void stop();

    int16_t lastLeftPwm()  const { return lastLeft_; }
    int16_t lastRightPwm() const { return lastRight_; }

private:
    Motor left_;
    Motor right_;
    int16_t lastLeft_  = 0;
    int16_t lastRight_ = 0;
};

#endif // APP_DIFFERENTIALDRIVE_HPP
