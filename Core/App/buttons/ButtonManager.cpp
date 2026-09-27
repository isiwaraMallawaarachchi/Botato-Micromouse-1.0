#include "ButtonManager.hpp"
#include "ButtonConfig.h"
extern "C" {
#include "gpio.h"
}

namespace {
    // Active-low, pull-up. PA6 = BTN2 = search, PA5 = BTN1 = fast.
    bool searchRaw() { return HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == GPIO_PIN_RESET; }
    bool fastRaw()   { return HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET; }

    bool take(bool& flag) { const bool e = flag; flag = false; return e; }
}

void ButtonManager::init() {
    *this = ButtonManager{};
    lastSample_ = HAL_GetTick();
}

// Debounced level: follows the raw pin only after it has been steady for
// DEBOUNCE_MS. Returns the debounced state.
bool ButtonManager::debounce(Btn& b, bool raw, uint32_t now) {
    if (raw != b.raw) { b.raw = raw; b.rawSince = now; }
    return ((now - b.rawSince) >= btncfg::DEBOUNCE_MS) ? b.raw : b.pressed;
}

void ButtonManager::update() {
    const uint32_t now = HAL_GetTick();
    if (now - lastSample_ < btncfg::SAMPLE_MS) return;
    lastSample_ = now;

    const bool s = debounce(search_, searchRaw(), now);
    const bool f = debounce(fast_,   fastRaw(),   now);

    if (s && f && !bothLatched_) { bothEvt_ = true; bothLatched_ = true; }

    // search: press, long-press while held, short on release
    if (s && !search_.pressed) {
        search_.pressed = true; search_.downTime = now; search_.longFired = false;
    } else if (s && !search_.longFired && !bothLatched_ &&
               (now - search_.downTime) >= btncfg::LONGPRESS_MS) {
        searchLongEvt_ = true;
        search_.longFired = true;
    } else if (!s && search_.pressed) {
        if (!search_.longFired && !bothLatched_) searchShortEvt_ = true;
        search_.pressed = false;
    }

    // fast: press, long-press while held, short on release
    if (f && !fast_.pressed) {
        fast_.pressed = true; fast_.downTime = now; fast_.longFired = false;
    } else if (f && !fast_.longFired && !bothLatched_ &&
               (now - fast_.downTime) >= btncfg::LONGPRESS_MS) {
        fastLongEvt_ = true;
        fast_.longFired = true;
    } else if (!f && fast_.pressed) {
        if (!fast_.longFired && !bothLatched_) fastShortEvt_ = true;
        fast_.pressed = false;
    }

    // Re-arm only after BOTH are released, and only after the releases above
    // were judged — so neither release of a both-press becomes a short press.
    if (!s && !f) bothLatched_ = false;
}

bool ButtonManager::takeSearchShort() { return take(searchShortEvt_); }
bool ButtonManager::takeFastShort()   { return take(fastShortEvt_); }
bool ButtonManager::takeSearchLong()  { return take(searchLongEvt_); }
bool ButtonManager::takeFastLong()    { return take(fastLongEvt_); }
bool ButtonManager::takeBothPress()   { return take(bothEvt_); }

bool ButtonManager::takeAny() {
    const bool a = takeSearchShort();
    const bool b = takeFastShort();
    const bool c = takeSearchLong();
    const bool d = takeBothPress();
    const bool e = takeFastLong();
    return a || b || c || d || e;
}
