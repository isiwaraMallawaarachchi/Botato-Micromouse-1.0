#ifndef APP_PIDCONTROLLER_HPP
#define APP_PIDCONTROLLER_HPP

/*
 * Generic PID with derivative-on-error and integral anti-windup clamp.
 * Stateless config, stateful accumulators. No allocation.
 */
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float iClamp)
        : kp_(kp), ki_(ki), kd_(kd), iClamp_(iClamp) {}

    // One step. error = setpoint - measurement. Returns control output.
    float compute(float error) {
        integral_ += error;
        if (integral_ >  iClamp_) integral_ =  iClamp_;
        if (integral_ < -iClamp_) integral_ = -iClamp_;
        float deriv = error - prevError_;
        prevError_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * deriv;
    }

    void reset() { integral_ = 0.0f; prevError_ = 0.0f; }

    // Live-tunable setters (call from a test harness / debugger).
    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }

private:
    float kp_, ki_, kd_, iClamp_;
    float integral_  = 0.0f;
    float prevError_ = 0.0f;
};

#endif // APP_PIDCONTROLLER_HPP
