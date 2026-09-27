#ifndef APP_INDICATOR_HPP
#define APP_INDICATOR_HPP

#include <cstdint>

/*
 * Indicator — status LED, non-blocking. update() every main-loop pass.
 *
 * Competition behaviour:
 *   BLINK      while the gyro calibrates (0.5s on / 0.5s off)
 *   OFF        once calibrated, and whenever idle or running
 *   pulse()    one short flash per navigation decision, then back to OFF
 *   code(n)    fault: n quick flashes, pause, repeat (ModeController::Fault)
 *   HEARTBEAT  idle in CURVED speed-run mode: short flash once a second
 *              (normal mode: OFF when idle)
 *   solid ON   Error_Handler (HAL init failure)
 *
 * Test builds: PASS = code(2), FAIL = code(3).
 */
class Indicator {
public:
    enum Pattern { OFF, ON, BLINK, PASS, FAIL, CODE, RAPID, HEARTBEAT };

    void init();
    void set(Pattern p);
    void code(int flashes);       // fault code: n flashes, pause, repeat
    void pulse();                 // one flash over the current pattern
    void update();

    Pattern pattern() const { return pattern_; }

private:
    Pattern  pattern_    = OFF;
    uint32_t lastMs_     = 0;
    int      step_       = 0;
    int      flashes_    = 0;
    bool     pulsing_    = false;
    uint32_t pulseStart_ = 0;
    void write(bool on);
};

#endif // APP_INDICATOR_HPP
