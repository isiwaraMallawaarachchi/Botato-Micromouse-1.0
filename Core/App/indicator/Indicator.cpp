#include "Indicator.hpp"
#include "IndicatorConfig.h"

void Indicator::write(bool on) {
    HAL_GPIO_WritePin(ledcfg::PORT, ledcfg::PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Indicator::init() {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {};
    g.Pin   = ledcfg::PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ledcfg::PORT, &g);
    pulsing_ = false;
    set(OFF);
    write(false);
}

void Indicator::set(Pattern p) {
    if (p == pattern_) return;       // re-setting must not restart the cycle
    pattern_ = p;
    step_    = 0;
    lastMs_  = HAL_GetTick();
    if (p == BLINK) write(true);     // visible immediately
}

void Indicator::pulse() {
    pulsing_    = true;
    pulseStart_ = HAL_GetTick();
    write(true);
}

void Indicator::update() {
    const uint32_t now = HAL_GetTick();

    if (pulsing_) {
        if (now - pulseStart_ < ledcfg::PULSE_MS) return;
        pulsing_ = false;            // fall through to the base pattern
    }

    const uint32_t dt = now - lastMs_;
    switch (pattern_) {
    case OFF: write(false); break;
    case ON:  write(true);  break;

    case BLINK:
        if (dt >= ledcfg::BLINK_MS) { lastMs_ = now; step_ ^= 1; }
        write(step_ == 0);
        break;

    case PASS:   // two flashes, pause
    case FAIL: { // three flashes, pause
        const int cycle = (pattern_ == PASS) ? 8 : 10;
        if (dt >= ledcfg::PATTERN_MS) { lastMs_ = now; step_ = (step_ + 1) % cycle; }
        const int flashes = (pattern_ == PASS) ? 2 : 3;
        write(step_ < flashes * 2 && (step_ % 2) == 0);
        break;
    }
    }
}
