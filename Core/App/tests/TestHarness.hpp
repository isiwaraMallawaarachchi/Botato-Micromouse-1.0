#ifndef APP_TESTHARNESS_HPP
#define APP_TESTHARNESS_HPP

#include <cstdint>
#include <cstring>
extern "C" {
#include "main.h"
#include "i2c.h"
}

/*
 * TestHarness — everything more than one test needs, written once.
 * Tests contain only what is specific to them.
 */
namespace th {

/* ---- I2C bus diagnostics ------------------------------------------------- */

struct PinState {
    uint8_t afScl, afSda;        // alternate-function number actually applied
    uint8_t moderScl, moderSda;  // 2 = alternate function
    uint8_t idleScl, idleSda;    // 1 = line high = pull-up present
};

PinState readPins(GPIO_TypeDef* port, uint8_t sclBit, uint8_t sdaBit);
bool     pinsOk(const PinState& p, uint8_t afScl, uint8_t afSda);

// 7-bit scan of 0x08..0x77. Writes up to `max` addresses, returns the count.
uint8_t scan(I2C_HandleTypeDef* h, uint8_t* out, uint8_t max);
bool    ready(I2C_HandleTypeDef* h, uint8_t addr7, uint32_t timeoutMs = 100);
bool    readReg8(I2C_HandleTypeDef* h, uint8_t addr7, uint8_t reg, uint8_t& out);
bool    writeReg8(I2C_HandleTypeDef* h, uint8_t addr7, uint8_t reg, uint8_t val);

// Bit-bang 9 SCL pulses + STOP to free a slave holding SDA, then reset and
// re-init the peripheral through CubeMX's own init (DECISIONS.md #17).
void recoverBus(I2C_HandleTypeDef* h, GPIO_TypeDef* port,
                uint16_t sclPin, uint16_t sdaPin, void (*reinit)());

// All five XSHUT lines released (sensors run) or held low (standby).
void setAllXshut(bool release);

/* ---- statistics ---------------------------------------------------------- */

// Rolling window over the last 32 samples.
class Window {
public:
    void     push(uint16_t v);
    void     clear() { n_ = idx_ = 0; }
    uint16_t min() const { return min_; }
    uint16_t max() const { return max_; }
    uint16_t mean() const { return mean_; }
    uint16_t spread() const { return static_cast<uint16_t>(max_ - min_); }
private:
    static constexpr int N = 32;
    uint16_t buf_[N] = {};
    int n_ = 0, idx_ = 0;
    uint16_t min_ = 0, max_ = 0, mean_ = 0;
};

// Events per second over a 1s window.
class RateMeter {
public:
    void     count() { ++n_; }
    void     update(uint32_t now);
    uint32_t hz() const { return hz_; }
private:
    uint32_t n_ = 0, hz_ = 0, start_ = 0;
};

// True once per period.
class Every {
public:
    explicit Every(uint32_t periodMs) : period_(periodMs) {}
    bool due();
private:
    uint32_t period_, last_ = 0;
};

/* ---- reports -------------------------------------------------------------- */

// Report globals are volatile so the debugger always sees fresh values.
// C++ cannot assign a struct into a volatile struct member; this does it.
template <class T>
inline void vset(volatile T& dst, const T& src) {
    std::memcpy(const_cast<T*>(&dst), &src, sizeof(T));
}

/* ---- robot-based tests --------------------------------------------------- */

// robot.initCore(): hardware, control loop, TIM3, gyro calibration start.
void coreInit();

// robot.serviceCore() every pass. Handles the LED through calibration (blink,
// then off). Returns true once the gyro is calibrated and the test may move.
bool coreService();

// Test verdict on the LED: double flash = pass, triple = fail.
void verdict(bool pass);

} // namespace th

#endif // APP_TESTHARNESS_HPP
