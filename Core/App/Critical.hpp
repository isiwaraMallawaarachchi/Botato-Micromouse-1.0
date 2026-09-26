#ifndef APP_CRITICAL_HPP
#define APP_CRITICAL_HPP

#include <cstdint>
extern "C" {
#include "main.h"
}

/*
 * CriticalSection — masks interrupts for the lifetime of the object.
 *
 * The TIM3 ISR and the main loop share state (control targets, gyro angle,
 * PID accumulators). A single 32-bit write is atomic on Cortex-M4, but an
 * update that touches several fields is not: the ISR can fire halfway and
 * run one tick on a half-written command. Wrap such updates in one of these.
 * Restores the previous PRIMASK, so nesting is safe. Keep the scope tiny.
 */
class CriticalSection {
public:
    CriticalSection() : primask_(__get_PRIMASK()) { __disable_irq(); }
    ~CriticalSection() { __set_PRIMASK(primask_); }
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
private:
    uint32_t primask_;
};

#endif // APP_CRITICAL_HPP
