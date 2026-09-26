#include "TestHarness.hpp"
#include "Config.h"
#include "Robot.hpp"
#include "WallSensorArray.hpp"
extern "C" {
#include "gpio.h"
}

namespace th {

/* ---- I2C bus diagnostics ------------------------------------------------- */

PinState readPins(GPIO_TypeDef* port, uint8_t scl, uint8_t sda) {
    auto af = [port](uint8_t bit) -> uint8_t {
        const uint32_t reg = (bit < 8) ? port->AFR[0] : port->AFR[1];
        return static_cast<uint8_t>((reg >> ((bit % 8) * 4)) & 0xFu);
    };
    PinState p;
    p.afScl    = af(scl);
    p.afSda    = af(sda);
    p.moderScl = static_cast<uint8_t>((port->MODER >> (scl * 2)) & 0x3u);
    p.moderSda = static_cast<uint8_t>((port->MODER >> (sda * 2)) & 0x3u);
    p.idleScl  = static_cast<uint8_t>((port->IDR >> scl) & 0x1u);
    p.idleSda  = static_cast<uint8_t>((port->IDR >> sda) & 0x1u);
    return p;
}

bool pinsOk(const PinState& p, uint8_t afScl, uint8_t afSda) {
    return p.afScl == afScl && p.afSda == afSda &&
           p.moderScl == 2 && p.moderSda == 2 &&
           p.idleScl == 1 && p.idleSda == 1;
}

uint8_t scan(I2C_HandleTypeDef* h, uint8_t* out, uint8_t max) {
    uint8_t n = 0;
    for (uint8_t a = 0x08; a < 0x78; ++a) {
        if (HAL_I2C_IsDeviceReady(h, static_cast<uint16_t>(a << 1), 1, 2) == HAL_OK) {
            if (n < max) out[n] = a;
            ++n;
        }
    }
    return n;
}

bool ready(I2C_HandleTypeDef* h, uint8_t addr7, uint32_t timeoutMs) {
    return HAL_I2C_IsDeviceReady(h, static_cast<uint16_t>(addr7 << 1), 2, timeoutMs) == HAL_OK;
}

bool readReg8(I2C_HandleTypeDef* h, uint8_t addr7, uint8_t reg, uint8_t& out) {
    return HAL_I2C_Mem_Read(h, static_cast<uint16_t>(addr7 << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, &out, 1, 100) == HAL_OK;
}

bool writeReg8(I2C_HandleTypeDef* h, uint8_t addr7, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(h, static_cast<uint16_t>(addr7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK;
}

void recoverBus(I2C_HandleTypeDef* h, GPIO_TypeDef* port,
                uint16_t sclPin, uint16_t sdaPin, void (*reinit)()) {
    HAL_I2C_DeInit(h);

    GPIO_InitTypeDef g = {};
    g.Pin   = sclPin | sdaPin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);

    HAL_GPIO_WritePin(port, sdaPin, GPIO_PIN_SET);
    for (int i = 0; i < 9; ++i) {
        HAL_GPIO_WritePin(port, sclPin, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(port, sclPin, GPIO_PIN_SET);   HAL_Delay(1);
    }
    HAL_GPIO_WritePin(port, sdaPin, GPIO_PIN_RESET); HAL_Delay(1);   // STOP
    HAL_GPIO_WritePin(port, sclPin, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(port, sdaPin, GPIO_PIN_SET);   HAL_Delay(1);

    h->Instance->CR1 |= I2C_CR1_SWRST;
    h->Instance->CR1 &= ~I2C_CR1_SWRST;
    reinit();
}

void setAllXshut(bool release) {
    for (int i = 0; i < cfg::TOF_COUNT; ++i) {
        const WallSensorArray::XshutPin x = WallSensorArray::xshutPin(i);
        HAL_GPIO_WritePin(x.port, x.pin, release ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/* ---- statistics ---------------------------------------------------------- */

void Window::push(uint16_t v) {
    buf_[idx_] = v;
    idx_ = (idx_ + 1) % N;
    if (n_ < N) ++n_;
    uint16_t lo = 0xFFFF, hi = 0;
    uint32_t sum = 0;
    for (int i = 0; i < n_; ++i) {
        if (buf_[i] < lo) lo = buf_[i];
        if (buf_[i] > hi) hi = buf_[i];
        sum += buf_[i];
    }
    min_ = lo; max_ = hi;
    mean_ = static_cast<uint16_t>(sum / static_cast<uint32_t>(n_));
}

void RateMeter::update(uint32_t now) {
    if (start_ == 0) start_ = now;
    if (now - start_ >= 1000) { hz_ = n_; n_ = 0; start_ = now; }
}

bool Every::due() {
    const uint32_t now = HAL_GetTick();
    if (now - last_ < period_) return false;
    last_ = now;
    return true;
}

/* ---- robot-based tests --------------------------------------------------- */

void coreInit() {
    robot.initCore();
    if (!robot.gyro().ok()) robot.led().set(Indicator::FAIL);
}

bool coreService() {
    robot.serviceCore();
    Gyro& g = robot.gyro();
    if (!g.ok()) return false;                       // LED stays FAIL
    if (g.calibrating()) { robot.led().set(Indicator::BLINK); return false; }
    if (robot.led().pattern() == Indicator::BLINK) robot.led().set(Indicator::OFF);
    return g.calibrated();
}

void verdict(bool pass) {
    robot.led().set(pass ? Indicator::PASS : Indicator::FAIL);
}

} // namespace th
