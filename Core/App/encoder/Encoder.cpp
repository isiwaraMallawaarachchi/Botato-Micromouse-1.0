#include "Encoder.hpp"
#include "Config.h"
#include "tim.h"      // HAL timer handles + __HAL_TIM_* macros (C linkage)

namespace {
    // Counters start here so reverse motion reads as a small negative number
    // instead of wrapping toward 4 billion on the first backward tick.
    constexpr int32_t START_VALUE = static_cast<int32_t>(0x80000000);
}

void Encoder::init(TIM_HandleTypeDef* htim, bool invert) {
    htim_   = htim;
    invert_ = invert;
    HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim_, START_VALUE);
    lastCount_   = readCounter();
    delta_       = 0;
    distanceMm_  = 0.0f;
    speedMmPerS_ = 0.0f;
}

int32_t Encoder::readCounter() const {
    int32_t raw = static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_)) - START_VALUE;
    return invert_ ? -raw : raw;
}

void Encoder::update() {
    int32_t c = readCounter();
    delta_     = c - lastCount_;
    lastCount_ = c;

    float dMm = static_cast<float>(delta_) * cfg::MM_PER_TICK;
    distanceMm_  += dMm;
    speedMmPerS_  = dMm / cfg::CONTROL_DT_S;   // mm per tick / dt = mm/s
}
