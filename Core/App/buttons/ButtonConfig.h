#ifndef APP_BUTTONCONFIG_H
#define APP_BUTTONCONFIG_H

#include <cstdint>

namespace btncfg {

constexpr uint32_t SAMPLE_MS    = 30;     // debounce: sample interval
constexpr uint32_t LONGPRESS_MS = 3000;   // PA6 held this long = recalibrate

} // namespace btncfg

#endif // APP_BUTTONCONFIG_H
