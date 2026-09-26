#ifndef APP_INDICATOR_HPP
#define APP_INDICATOR_HPP

#include <cstdint>

/*
 * Indicator — status LED, non-blocking. update() every main-loop pass.
 *
 * Competition behaviour:
 *   BLINK   while the gyro calibrates (0.5s on / 0.5s off)
 *   OFF     once calibrated, and whenever idle or running
 *   pulse() one short flash per navigation decision, then back to OFF
 *   FAIL    triple flash: gyro not found (the robot cannot run)
 *
 * PASS / FAIL are also the test-build verdicts.
 */
class Indicator {
public:
    enum Pattern { OFF, ON, BLINK, PASS, FAIL };

    void init();
    void set(Pattern p);
    void pulse();                 // one flash over the current pattern
    void update();

    Pattern pattern() const { return pattern_; }

private:
    Pattern  pattern_    = OFF;
    uint32_t lastMs_     = 0;
    int      step_       = 0;
    bool     pulsing_    = false;
    uint32_t pulseStart_ = 0;
    void write(bool on);
};

#endif // APP_INDICATOR_HPP
