#ifndef APP_BUTTONMANAGER_HPP
#define APP_BUTTONMANAGER_HPP

#include <cstdint>

/*
 * ButtonManager — debounced edge + long-press + both-press detection for the
 * two mode buttons (PA6 search, PA5 fast). Polled in the main loop.
 * Events are one-shot: read them with the take* methods, which clear the flag.
 */
class ButtonManager {
public:
    void init();
    void update();   // call every main-loop pass

    // One-shot event getters (return true once, then self-clear).
    bool takeSearchShort();   // PA6 short press released
    bool takeFastShort();     // PA5 short press released
    bool takeSearchLong();    // PA6 held > BTN_LONGPRESS_MS
    bool takeBothPress();     // both held together

private:
    struct Btn {
        uint8_t  pressed = 0;       // debounced state
        uint32_t downTime = 0;      // tick when pressed
        uint8_t  longFired = 0;     // long event already emitted this hold
    };
    Btn search_;   // PA6
    Btn fast_;     // PA5

    bool searchShortEvt_ = false;
    bool fastShortEvt_   = false;
    bool searchLongEvt_  = false;
    bool bothEvt_        = false;

    uint32_t lastSample_ = 0;

    bool readSearchRaw() const;   // active-low
    bool readFastRaw()   const;
};

#endif // APP_BUTTONMANAGER_HPP
