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

void ButtonManager::update() {
    const uint32_t now = HAL_GetTick();
    if (now - lastSample_ < btncfg::SAMPLE_MS) return;
    lastSample_ = now;

    const bool s = searchRaw();
    const bool f = fastRaw();

    if (s && f && !bothLatched_) { bothEvt_ = true; bothLatched_ = true; }

    // search: press, long-press while held, short on release
    if (s && !search_.pressed) {
        search_ = { true, now, false };
    } else if (s && !search_.longFired && !bothLatched_ &&
               (now - search_.downTime) >= btncfg::LONGPRESS_MS) {
        searchLongEvt_ = true;
        search_.longFired = true;
    } else if (!s && search_.pressed) {
        if (!search_.longFired && !bothLatched_) searchShortEvt_ = true;
        search_.pressed = false;
    }

    // fast: short on release
    if (f && !fast_.pressed) {
        fast_ = { true, now, false };
    } else if (!f && fast_.pressed) {
        if (!bothLatched_) fastShortEvt_ = true;
        fast_.pressed = false;
    }

    // Re-arm only after BOTH are released, and only after the releases above
    // were judged — so neither release of a both-press becomes a short press.
    if (!s && !f) bothLatched_ = false;
}

bool ButtonManager::takeSearchShort() { return take(searchShortEvt_); }
bool ButtonManager::takeFastShort()   { return take(fastShortEvt_); }
bool ButtonManager::takeSearchLong()  { return take(searchLongEvt_); }
bool ButtonManager::takeBothPress()   { return take(bothEvt_); }

bool ButtonManager::takeAny() {
    const bool a = takeSearchShort();
    const bool b = takeFastShort();
    const bool c = takeSearchLong();
    const bool d = takeBothPress();
    return a || b || c || d;
}
