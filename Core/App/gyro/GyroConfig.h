#ifndef APP_GYROCONFIG_H
#define APP_GYROCONFIG_H

#include <cstdint>

namespace gyrocfg {

constexpr uint8_t  ADDR_7B          = 0x68;   // AD0 tied to GND
constexpr uint8_t  GYRO_CONFIG_VAL  = 0x08;   // FS_SEL=1 -> +/-500 dps
constexpr float    LSB_PER_DPS      = 65.5f;  // matches FS_SEL=1
constexpr uint8_t  DLPF_VAL         = 0x03;   // ~44Hz
// The MPU6050 updates its output register at 1kHz, so reading it several
// times per 1ms tick returns the same sample: one read per tick.
constexpr int      OVERSAMPLE       = 1;
constexpr uint32_t READ_TIMEOUT_MS  = 2;

// Bias calibration length in 1ms ticks. The robot must stay still.
constexpr int      CAL_TICKS        = 5000;

// This many consecutive failed ticks = gyro lost (loose I2C2 / power dip).
// The robot stops and shows a fault instead of calibrating forever.
constexpr int      LOST_TICKS       = 100;

} // namespace gyrocfg

#endif // APP_GYROCONFIG_H
