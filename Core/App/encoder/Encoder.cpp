#include "Encoder.hpp"
#include "Config.h"

namespace {
    // Counters start mid-range so reverse motion reads as a small negative
    // number instead of wrapping toward 4 billion on the first backward tick.
    constexpr uint32_t START_VALUE = 0x80000000u;
}

void Encoder::init(TIM_HandleTypeDef* htim, bool invert) {
    htim_   = htim;
    invert_ = invert;
    HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim_, START_VALUE);
    count_ = readCounter();
    delta_ = 0;
    distanceMm_ = speedMmPerS_ = 0.0f;
    for (int i = 0; i < enccfg::SPEED_WINDOW; ++i) window_[i] = 0;
    windowSum_ = 0;
    windowIdx_ = 0;
}

int32_t Encoder::readCounter() const {
    const int32_t raw = static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_) - START_VALUE);
    return invert_ ? -raw : raw;
}

void Encoder::update() {
    const int32_t c = readCounter();
    delta_ = c - count_;
    count_ = c;
    distanceMm_ += static_cast<float>(delta_) * cfg::MM_PER_TICK;

    windowSum_ += delta_ - window_[windowIdx_];
    window_[windowIdx_] = delta_;
    windowIdx_ = (windowIdx_ + 1) % enccfg::SPEED_WINDOW;

    speedMmPerS_ = static_cast<float>(windowSum_) * cfg::MM_PER_TICK /
                   (enccfg::SPEED_WINDOW * cfg::CONTROL_DT_S);
}
