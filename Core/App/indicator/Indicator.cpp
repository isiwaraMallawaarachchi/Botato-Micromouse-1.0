#include <Indicator.hpp>
extern "C" {
#include "gpio.h"
}

/* Blackpill onboard LED is PC13, ACTIVE LOW.
 * If you use a different pin, change these two lines only. */
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13

void Indicator::write(bool on) {
    // active low: LOW = lit
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Indicator::init() {
    // PC13 as output. (If CubeMX already configures it, this is harmless.)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = LED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
    write(false);
    pattern_ = OFF;
}

void Indicator::set(Pattern p) {
    pattern_ = p;
    step_    = 0;
    lastMs_  = HAL_GetTick();
}

void Indicator::flash(int times, uint32_t onMs, uint32_t offMs) {
    for (int i = 0; i < times; ++i) {
        write(true);  HAL_Delay(onMs);
        write(false); HAL_Delay(offMs);
    }
}

void Indicator::update() {
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - lastMs_;

    switch (pattern_) {
    case OFF:   write(false); break;
    case SOLID: write(true);  break;

    case SLOW_BLINK:
        if (dt >= 500) { lastMs_ = now; step_ ^= 1; write(step_); }
        break;

    case FAST_BLINK:
        if (dt >= 120) { lastMs_ = now; step_ ^= 1; write(step_); }
        break;

    case DOUBLE_BLINK:   // blink blink ... pause
        if (dt >= 150) {
            lastMs_ = now;
            step_ = (step_ + 1) % 8;
            write(step_ == 0 || step_ == 2);
        }
        break;

    case TRIPLE_BLINK:
        if (dt >= 150) {
            lastMs_ = now;
            step_ = (step_ + 1) % 10;
            write(step_ == 0 || step_ == 2 || step_ == 4);
        }
        break;
    }
}
