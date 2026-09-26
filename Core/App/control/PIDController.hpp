#ifndef APP_PIDCONTROLLER_HPP
#define APP_PIDCONTROLLER_HPP

#include "ControlConfig.h"

/*
 * PID with derivative-on-error and an integral anti-windup clamp.
 * compute() is called once per control tick; the integral and derivative are
 * therefore per-tick quantities, which is what the tuned gains assume.
 */
class PIDController {
public:
    explicit PIDController(const ctrlcfg::Gains& g)
        : kp_(g.kp), ki_(g.ki), kd_(g.kd), iClamp_(g.iClamp) {}

    float compute(float error) {
        integral_ += error;
        if (integral_ >  iClamp_) integral_ =  iClamp_;
        if (integral_ < -iClamp_) integral_ = -iClamp_;
        const float deriv = error - prevError_;
        prevError_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * deriv;
    }

    void reset() { integral_ = 0.0f; prevError_ = 0.0f; }

    // Live tuning from a debugger or test harness.
    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

private:
    float kp_, ki_, kd_, iClamp_;
    float integral_  = 0.0f;
    float prevError_ = 0.0f;
};

#endif // APP_PIDCONTROLLER_HPP
