#include "ButtonManager.hpp"
#include "Config.h"
#include "gpio.h"

volatile int dbg_search_raw = 0;
volatile int dbg_fast_raw   = 0;
volatile int dbg_search_pressed = 0;
volatile int dbg_fast_pressed   = 0;

// Buttons are active-low (pull-up), PA6 = search (BTN2), PA5 = fast (BTN1).
bool ButtonManager::readSearchRaw() const {
    return HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin) == GPIO_PIN_RESET;
}
bool ButtonManager::readFastRaw() const {
    return HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET;
}

void ButtonManager::init() {
    search_ = Btn{};
    fast_   = Btn{};
}

void ButtonManager::update() {
    uint32_t now = HAL_GetTick();
    if (now - lastSample_ < cfg::BTN_DEBOUNCE_MS) return;
    lastSample_ = now;

    bool s = readSearchRaw();
    bool f = readFastRaw();

    dbg_search_raw = s ? 1 : 0;
    dbg_fast_raw   = f ? 1 : 0;
    dbg_search_pressed = search_.pressed;
    dbg_fast_pressed   = fast_.pressed;

    // --- both-press: fires once when both are down together ---
    static bool bothLatched = false;
    if (s && f) {
        if (!bothLatched) { bothEvt_ = true; bothLatched = true; }
    } else {
        bothLatched = false;
    }

    // --- search button edge + long-press ---
    if (s && !search_.pressed) {              // just pressed
        search_.pressed = 1; search_.downTime = now; search_.longFired = 0;
    } else if (s && search_.pressed) {         // held
        if (!search_.longFired && (now - search_.downTime) >= cfg::BTN_LONGPRESS_MS) {
            searchLongEvt_ = true; search_.longFired = 1;
        }
    } else if (!s && search_.pressed) {        // released
        // short only if long didn't fire and it wasn't part of a both-press
        if (!search_.longFired && !bothLatched) searchShortEvt_ = true;
        search_.pressed = 0;
    }

    // --- fast button edge (short only) ---
    if (f && !fast_.pressed) {
        fast_.pressed = 1; fast_.downTime = now;
    } else if (!f && fast_.pressed) {
        if (!bothLatched) fastShortEvt_ = true;
        fast_.pressed = 0;
    }
}

bool ButtonManager::takeSearchShort() { bool e = searchShortEvt_; searchShortEvt_ = false; return e; }
bool ButtonManager::takeFastShort()   { bool e = fastShortEvt_;   fastShortEvt_   = false; return e; }
bool ButtonManager::takeSearchLong()  { bool e = searchLongEvt_;  searchLongEvt_  = false; return e; }
bool ButtonManager::takeBothPress()   { bool e = bothEvt_;        bothEvt_        = false; return e; }
