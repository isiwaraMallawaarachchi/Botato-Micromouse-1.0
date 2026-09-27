#ifndef APP_MOTIONPROFILE_HPP
#define APP_MOTIONPROFILE_HPP

#include <cmath>

/*
 * MotionProfile — the acceleration curves.
 *
 * Nothing that reaches a PID ever steps. Every command passes through one of
 * these first, so wheel torque rises and falls smoothly and the tyres keep
 * their grip.
 *
 *   SpeedRamp     forward speed: moves toward the commanded speed no faster
 *                 than ACCEL when speeding up and DECEL when slowing down.
 *   AngleProfile  turns: trapezoidal rate profile. Rate ramps up, cruises,
 *                 then ramps down so the reference lands on the target angle
 *                 at zero rate. Its rate is fed forward to the rate loop, so
 *                 the heading PID only corrects error instead of doing the
 *                 whole turn from a step.
 *   approachSpeed position-aware braking: the fastest speed from which the
 *                 robot can still stop in `remaining` mm at `decel`. Used by
 *                 the navigator and the drive test so both stop identically.
 */

class SpeedRamp {
public:
    void  reset(float v = 0.0f) { value_ = v; }
    float value() const { return value_; }

    float step(float target, float accel, float decel, float dt) {
        // "Speeding up" = moving away from zero; anything else is slowing.
        const bool speedingUp = (target * value_ >= 0.0f) &&
                                (std::fabs(target) > std::fabs(value_));
        const float maxDelta = (speedingUp ? accel : decel) * dt;
        float d = target - value_;
        if (d >  maxDelta) d =  maxDelta;
        if (d < -maxDelta) d = -maxDelta;
        value_ += d;
        return value_;
    }

private:
    float value_ = 0.0f;
};

class AngleProfile {
public:
    void reset(float at) { pos_ = goal_ = at; vel_ = 0.0f; }
    void moveBy(float delta) { goal_ += delta; }

    float pos()  const { return pos_; }
    float vel()  const { return vel_; }
    float goal() const { return goal_; }
    bool  done() const { return pos_ == goal_ && vel_ == 0.0f; }

    void step(float vmax, float amax, float dt) {
        const float remaining = goal_ - pos_;
        if (remaining == 0.0f && vel_ == 0.0f) return;

        // Fastest rate from which we can still stop exactly on the goal.
        const float dir     = (remaining >= 0.0f) ? 1.0f : -1.0f;
        float vAllowed      = std::sqrt(2.0f * amax * std::fabs(remaining));
        if (vAllowed > vmax) vAllowed = vmax;
        const float vTarget = dir * vAllowed;

        float dv = vTarget - vel_;
        const float maxDv = amax * dt;
        if (dv >  maxDv) dv =  maxDv;
        if (dv < -maxDv) dv = -maxDv;
        vel_ += dv;
        pos_ += vel_ * dt;

        // Land exactly: within one step of the goal, or crossed it.
        const float after = goal_ - pos_;
        if ((after * dir) <= 0.0f ||
            (std::fabs(after) < std::fabs(vel_) * dt && std::fabs(vel_) <= maxDv * 2.0f)) {
            pos_ = goal_;
            vel_ = 0.0f;
        }
    }

private:
    float pos_  = 0.0f;
    float vel_  = 0.0f;
    float goal_ = 0.0f;
};

// Curved-turn shape. u = distance along the curve / curve length, 0..1.
// Turn rate is a trapezoid: ramps up over [0, rho], constant, ramps down over
// [1-rho, 1]. rate() is normalised so its mean over the curve is 1; angle()
// is its integral, 0..1 (fraction of the turn done). The shape is symmetric,
// so the robot leaves the curve exactly on the new corridor's centreline.
struct ArcShape {
    static float rate(float u, float rho) {
        const float peak = 1.0f / (1.0f - rho);
        if (u <= 0.0f || u >= 1.0f) return 0.0f;
        if (u < rho)        return peak * u / rho;
        if (u > 1.0f - rho) return peak * (1.0f - u) / rho;
        return peak;
    }
    // d(rate)/du: +peak/rho while ramping up, -peak/rho ramping down, else 0.
    static float rateSlope(float u, float rho) {
        const float peak = 1.0f / (1.0f - rho);
        if (u <= 0.0f || u >= 1.0f) return 0.0f;
        if (u < rho)        return  peak / rho;
        if (u > 1.0f - rho) return -peak / rho;
        return 0.0f;
    }
    static float angle(float u, float rho) {
        const float peak = 1.0f / (1.0f - rho);
        if (u <= 0.0f) return 0.0f;
        if (u >= 1.0f) return 1.0f;
        if (u < rho)        return peak * u * u / (2.0f * rho);
        if (u > 1.0f - rho) { const float w = 1.0f - u; return 1.0f - peak * w * w / (2.0f * rho); }
        return peak * (0.5f * rho + (u - rho));
    }
    // Curve length / radius, so that a 90-degree curve starting on one
    // centreline ends on the perpendicular one `radius` further on (for a
    // circle this is pi/2). Integrated numerically once; cheap.
    static float lengthPerRadius(float rho) {
        constexpr int N = 400;
        float sum = 0.0f;
        for (int i = 0; i < N; ++i) {
            const float u = (static_cast<float>(i) + 0.5f) / N;
            sum += std::cos(1.5707963f * angle(u, rho));
        }
        return static_cast<float>(N) / sum;
    }
};

// Speed to command while approaching a stop point `remaining` mm away.
// The braking curve reaches zero `creepZone` mm BEFORE the stop point, and the
// robot covers those last millimetres at `creep`. Braking straight onto the
// point arrives at ~sqrt(2*decel*stopTol) (~90mm/s) and coasts past it by the
// motor lag; arriving at creep speed stops within ~1-2mm. Returns 0 once
// within stopTol.
inline float approachSpeed(float remaining, float cruise, float decel,
                           float creep, float creepZone, float stopTol) {
    if (remaining <= stopTol) return 0.0f;
    const float braking = remaining - creepZone;
    float v = (braking > 0.0f) ? std::sqrt(2.0f * decel * braking) : 0.0f;
    if (v > cruise) v = cruise;
    if (v < creep)  v = creep;
    return v;
}

#endif // APP_MOTIONPROFILE_HPP
