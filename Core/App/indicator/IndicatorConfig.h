#ifndef APP_INDICATORCONFIG_H
#define APP_INDICATORCONFIG_H

#include <cstdint>
extern "C" {
#include "gpio.h"
}

namespace ledcfg {

// Black Pill onboard LED, PC13, ACTIVE LOW.
inline GPIO_TypeDef* const PORT = GPIOC;
constexpr uint16_t PIN          = GPIO_PIN_13;

constexpr uint32_t BLINK_MS     = 500;   // calibrating: toggles every 0.5s
constexpr uint32_t PULSE_MS     = 80;    // one flash per navigation decision
constexpr uint32_t PATTERN_MS   = 150;   // step time for test pass/fail codes

} // namespace ledcfg

#endif // APP_INDICATORCONFIG_H
