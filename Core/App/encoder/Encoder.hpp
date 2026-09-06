#ifndef APP_ENCODER_HPP
#define APP_ENCODER_HPP

#include <cstdint>
extern "C" {
#include "tim.h"
}

/*
 * Encoder — reads one hardware quadrature timer, exposes distance + speed.
 * The right wheel is negated (DECISIONS.md #18) so forward = +ve on both.
 * Counters start mid-range so reverse reads negative, not a huge wrap value.
 */

class Encoder {
public:
    // htim: the encoder-mode timer. invert: true for the right wheel.
    void init(TIM_HandleTypeDef* htim, bool invert);

    // Call once per control tick. Updates delta/distance/speed.
    void update();

    int32_t deltaTicks()   const { return delta_; }       // ticks since last update
    float   distanceMm()   const { return distanceMm_; }   // accumulated, mm
    float   speedMmPerS()  const { return speedMmPerS_; }  // instantaneous, mm/s
    int32_t rawCount()     const { return lastCount_; }     // signed, offset-removed

    void resetDistance()   { distanceMm_ = 0.0f; }

private:
    TIM_HandleTypeDef* htim_ = nullptr;
    bool    invert_ = false;
    int32_t lastCount_ = 0;
    int32_t delta_ = 0;
    float   distanceMm_ = 0.0f;
    float   speedMmPerS_ = 0.0f;

    int32_t readCounter() const;   // signed, invert applied, mid-range removed
};

#endif // APP_ENCODER_HPP
