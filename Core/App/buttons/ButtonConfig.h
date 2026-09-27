#ifndef APP_BUTTONCONFIG_H
#define APP_BUTTONCONFIG_H

#include <cstdint>

namespace btncfg {

constexpr uint32_t SAMPLE_MS    = 5;      // raw sampling interval
// A change counts only after the pin has held its new level this long, so a
// bouncing contact gives exactly one press (no phantom second press).
constexpr uint32_t DEBOUNCE_MS  = 20;
constexpr uint32_t LONGPRESS_MS = 3000;   // held this long = long press (PA6 recalibrate, PA5 curves)

} // namespace btncfg

#endif // APP_BUTTONCONFIG_H
