#ifndef APP_BUTTONMANAGER_HPP
#define APP_BUTTONMANAGER_HPP

#include <cstdint>

/*
 * ButtonManager — two active-low buttons, polled and debounced in the main
 * loop (DECISIONS.md #6: deliberately not EXTI).
 *
 *   PA6 (BTN2) "search":  short press, long press
 *   PA5 (BTN1) "fast":    short press
 *   both together:        one both-press event, suppresses the shorts
 *
 * Events are one-shot: take*() returns true once, then clears.
 */
class ButtonManager {
public:
    void init();
    void update();

    bool takeSearchShort();
    bool takeFastShort();
    bool takeSearchLong();
    bool takeBothPress();
    bool takeAny();              // consumes every pending event

    bool searchHeld() const { return search_.pressed; }
    bool fastHeld()   const { return fast_.pressed; }

private:
    struct Btn {
        bool     pressed   = false;
        uint32_t downTime  = 0;
        bool     longFired = false;
    };
    Btn search_, fast_;

    bool searchShortEvt_ = false;
    bool fastShortEvt_   = false;
    bool searchLongEvt_  = false;
    bool bothEvt_        = false;
    bool bothLatched_    = false;
    uint32_t lastSample_ = 0;
};

#endif // APP_BUTTONMANAGER_HPP
