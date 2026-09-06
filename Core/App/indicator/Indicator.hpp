#ifndef APP_INDICATOR_HPP
#define APP_INDICATOR_HPP

#include <cstdint>

/*
 * Indicator — status LED (and later buzzer) feedback.
 * Non-blocking: call update() every main loop pass. Patterns are set by
 * the caller and play out on their own.
 *
 * Buzzer hooks are stubbed and marked TODO — wire a GPIO/PWM pin when the
 * buzzer is added, and fill in beep()/beepPattern().
 */
class Indicator {
public:
    enum Pattern {
        OFF,
        SOLID,
        SLOW_BLINK,     // idle / waiting
        FAST_BLINK,     // busy (calibrating)
        DOUBLE_BLINK,   // done / success
        TRIPLE_BLINK    // error
    };

    void init();
    void set(Pattern p);
    void update();                 // call every main loop pass

    // Blocking convenience: N quick flashes (use only outside a run).
    void flash(int times, uint32_t onMs = 100, uint32_t offMs = 100);

    // --- Buzzer (TODO: wire when hardware is added) ---
    void beep(uint32_t /*ms*/) { /* TODO: buzzer pin */ }

private:
    Pattern  pattern_ = OFF;
    uint32_t lastMs_  = 0;
    int      step_    = 0;
    void write(bool on);
};

#endif
