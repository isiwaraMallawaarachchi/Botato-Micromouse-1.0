#include "Tests_I2C.hpp"
#include "Robot.hpp"
#include "Config.h"

extern "C" {
#include "i2c.h"
#include "gpio.h"
#include "tof.h"
}

I2CGyroReport i2cGyro;
I2CToFReport  i2cToF;

namespace {

// ---- MPU6050 ---------------------------------------------------------------
// AD0 tied to GND (PINOUT.md) -> fixed 7-bit address 0x68. HAL wants it shifted.
constexpr uint8_t  MPU_ADDR_7BIT = 0x68;
constexpr uint16_t MPU_ADDR      = static_cast<uint16_t>(MPU_ADDR_7BIT << 1);

constexpr uint8_t REG_WHO_AM_I    = 0x75;
constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
constexpr uint8_t REG_CONFIG      = 0x1A;   // DLPF
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;

// Values Gyro::init() writes. Read back to prove the write path, not just read.
constexpr uint8_t VAL_PWR_MGMT_1  = 0x00;   // wake
constexpr uint8_t VAL_GYRO_CONFIG = 0x08;   // +/-500 dps -> 65.5 LSB/dps
constexpr uint8_t VAL_DLPF        = 0x03;   // ~44Hz

// ---- I2C2 pins (PINOUT.md) -------------------------------------------------
constexpr uint16_t SCL_PIN = GPIO_PIN_10;   // PB10, AF4
constexpr uint16_t SDA_PIN = GPIO_PIN_9;    // PB9,  AF9  <- the trap, #16
constexpr uint8_t  SCL_BIT = 10;
constexpr uint8_t  SDA_BIT = 9;

constexpr uint32_t SCAN_TIMEOUT_MS = 2;
constexpr uint32_t IO_TIMEOUT_MS   = 100;   // bring-up: generous
// Live reads use a short timeout on purpose. At 400kHz a 2-byte mem read is
// ~120us; anything past 10ms is a fault, not slowness. A 100ms timeout here
// would stall the loop for a tenth of a second per failure and distort the
// error rate the test is trying to measure.
constexpr uint32_t LIVE_TIMEOUT_MS = 10;

constexpr int      SAMPLE_COUNT      = 20;  // raw-Z samples taken in Init
constexpr uint32_t LIVE_PERIOD_MS    = 100; // live re-read cadence
constexpr uint32_t RECOVER_AFTER     = 3;   // consecutive failures before unwedge
constexpr uint32_t RECOVER_COOLDOWN_MS = 1000;

uint32_t lastLiveMs     = 0;
uint32_t lastRecoverMs  = 0;
bool     awaitingRecoveryProof = false;

// ---- Low-level helpers -----------------------------------------------------

bool readReg8(uint8_t reg, uint8_t& out) {
    return HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, reg, 1, &out, 1, IO_TIMEOUT_MS) == HAL_OK;
}

bool writeReg8(uint8_t reg, uint8_t val) {
    uint8_t d = val;
    return HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, reg, 1, &d, 1, IO_TIMEOUT_MS) == HAL_OK;
}

bool readRawZ(int16_t& out, uint32_t timeoutMs) {
    uint8_t b[2];
    if (HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, REG_GYRO_ZOUT_H, 1, b, 2, timeoutMs) != HAL_OK)
        return false;
    out = static_cast<int16_t>((b[0] << 8) | b[1]);
    return true;
}

// Read the pin configuration the HAL actually applied. The only way to catch
// the AF9 trap (DECISIONS.md #16) at runtime: if PB9 came up AF4 it stays
// internally wired to I2C1 and I2C2 silently does nothing.
void capturePinState() {
    // AFR[1] covers pins 8..15, 4 bits each.
    i2cGyro.afSda = static_cast<uint8_t>((GPIOB->AFR[1] >> ((SDA_BIT - 8) * 4)) & 0xF);
    i2cGyro.afScl = static_cast<uint8_t>((GPIOB->AFR[1] >> ((SCL_BIT - 8) * 4)) & 0xF);
    // MODER: 2 bits per pin, 0b10 = alternate function.
    i2cGyro.moderSda = static_cast<uint8_t>((GPIOB->MODER >> (SDA_BIT * 2)) & 0x3);
    i2cGyro.moderScl = static_cast<uint8_t>((GPIOB->MODER >> (SCL_BIT * 2)) & 0x3);
    // IDR reflects the physical level even in AF mode. Both must idle high.
    i2cGyro.idleSda = static_cast<uint8_t>((GPIOB->IDR >> SDA_BIT) & 0x1);
    i2cGyro.idleScl = static_cast<uint8_t>((GPIOB->IDR >> SCL_BIT) & 0x1);
}

// Sampled every live pass. Line state plus the BUSY flag is what separates a
// wedged peripheral (lines idle high, BUSY still stuck at 1) from a slave
// holding the bus down (SDA reads 0).
void captureLiveState() {
    i2cGyro.liveSda  = static_cast<uint8_t>((GPIOB->IDR >> SDA_BIT) & 0x1);
    i2cGyro.liveScl  = static_cast<uint8_t>((GPIOB->IDR >> SCL_BIT) & 0x1);
    i2cGyro.busyFlag = (hi2c2.Instance->SR2 & I2C_SR2_BUSY) ? 1 : 0;
    i2cGyro.halState = static_cast<uint32_t>(hi2c2.State);
}

// Classify ONE fault, then clear ErrorCode. HAL ORs bits in and never clears
// them, so without this every later read inherits the history of the first.
void classifyError() {
    const uint32_t e = hi2c2.ErrorCode;
    i2cGyro.lastErrorCode = e;
    if (e & HAL_I2C_ERROR_AF)      ++i2cGyro.errAf;       // no ACK
    if (e & HAL_I2C_ERROR_TIMEOUT) ++i2cGyro.errTimeout;  // line stuck / no clock
    if (e & HAL_I2C_ERROR_BERR)    ++i2cGyro.errBerr;     // misplaced START/STOP
    if (e & HAL_I2C_ERROR_ARLO)    ++i2cGyro.errArlo;     // arbitration lost
    hi2c2.ErrorCode = HAL_I2C_ERROR_NONE;
}

// Bit-bang up to 9 SCL pulses to free a slave still holding SDA low from an
// interrupted debug session (DECISIONS.md #17). A reflash does not clear this
// — the MCU resets, the sensor does not. Also force-resets the peripheral,
// which is what clears a latched BUSY flag.
void busRecover() {
    HAL_I2C_DeInit(&hi2c2);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = SCL_PIN | SDA_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;      // open-drain: only ever pull low
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, SDA_PIN, GPIO_PIN_SET);   // release SDA
    for (int i = 0; i < 9; ++i) {
        HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_SET);
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_SET) break;
    }

    // Manual STOP: SDA low -> high while SCL is high.
    HAL_GPIO_WritePin(GPIOB, SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    // Reset the peripheral, then let CubeMX's own init reconfigure the pins so
    // the AF check above is still testing the real generated code.
    __HAL_RCC_I2C2_FORCE_RESET();
    HAL_Delay(1);
    __HAL_RCC_I2C2_RELEASE_RESET();
    MX_I2C2_Init();
}

// Walk the whole 7-bit range. Exactly one device should answer on I2C2 — the
// buses were split (#16) precisely to keep extra pull-up load off the IMU.
void scanBus() {
    i2cGyro.scanCount = 0;
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, static_cast<uint16_t>(a << 1), 1,
                                  SCAN_TIMEOUT_MS) == HAL_OK) {
            if (i2cGyro.scanCount < 8) i2cGyro.scanAddr[i2cGyro.scanCount] = a;
            ++i2cGyro.scanCount;
        }
    }
    // A scan leaves AF flags set from every address that did not answer.
    hi2c2.ErrorCode = HAL_I2C_ERROR_NONE;
}

void noteSample(int16_t z) {
    i2cGyro.rawZ = z;
    if (z < i2cGyro.rawZMin) i2cGyro.rawZMin = z;
    if (z > i2cGyro.rawZMax) i2cGyro.rawZMax = z;
    if (i2cGyro.rawZMax != i2cGyro.rawZMin) i2cGyro.dataVarying = 1;
}

void noteSuccess(int16_t z) {
    noteSample(z);
    i2cGyro.lastSuccessMs = HAL_GetTick();
    i2cGyro.consecErrors  = 0;
    if (awaitingRecoveryProof) {
        i2cGyro.recoveredOk       = 1;
        awaitingRecoveryProof     = false;
    }
}

void noteFailure() {
    const uint32_t now = HAL_GetTick();
    ++i2cGyro.readErrors;
    ++i2cGyro.consecErrors;
    if (i2cGyro.consecErrors > i2cGyro.maxConsecErrors)
        i2cGyro.maxConsecErrors = i2cGyro.consecErrors;
    if (i2cGyro.firstErrorMs == 0) i2cGyro.firstErrorMs = now;
    classifyError();
    i2cGyro.pass = 0;              // live verdict, never re-armed
}

} // namespace

void Test_I2C_Gyro_Init() {
    i2cGyro = I2CGyroReport{};
    i2cGyro.rawZMin = 32767;
    i2cGyro.rawZMax = -32768;

    robot.led().init();
    robot.led().set(Indicator::FAST_BLINK);

    // 1 — pin level, before any transaction. If this stage is wrong, nothing
    //     downstream can work and the cause is the .ioc, not the bus.
    capturePinState();

    // 2 — unwedge only if a line is actually stuck low. Running recovery on a
    //     healthy bus is harmless but hides the real state.
    if (i2cGyro.idleSda == 0 || i2cGyro.idleScl == 0) {
        i2cGyro.recoveryRun = 1;
        ++i2cGyro.recoveries;
        busRecover();
        capturePinState();          // did recovery actually free the line?
    }

    // 3 — who is on this bus?
    scanBus();

    // 4 — does the MPU6050 answer at its own address?
    i2cGyro.deviceReady =
        (HAL_I2C_IsDeviceReady(&hi2c2, MPU_ADDR, 3, IO_TIMEOUT_MS) == HAL_OK) ? 1 : 0;

    // 5 — identity. WHO_AM_I reads 0x68 regardless of the AD0 pin.
    if (i2cGyro.deviceReady) {
        uint8_t w = 0;
        if (readReg8(REG_WHO_AM_I, w)) {
            i2cGyro.whoAmI   = w;
            i2cGyro.whoAmIOk = (w == 0x68) ? 1 : 0;
        }
    }

    // 6 — write path. Write-then-read-back separates "the bus can read" from
    //     "can read AND write"; a marginal SDA often manages only one.
    if (i2cGyro.whoAmIOk) {
        bool wrote = writeReg8(REG_PWR_MGMT_1,  VAL_PWR_MGMT_1);
        wrote     &= writeReg8(REG_GYRO_CONFIG, VAL_GYRO_CONFIG);
        wrote     &= writeReg8(REG_CONFIG,      VAL_DLPF);
        HAL_Delay(10);              // let the sensor come out of sleep

        bool read = readReg8(REG_PWR_MGMT_1,  i2cGyro.pwrMgmtReadback);
        read     &= readReg8(REG_GYRO_CONFIG, i2cGyro.gyroConfigReadback);
        read     &= readReg8(REG_CONFIG,      i2cGyro.dlpfReadback);

        i2cGyro.regWriteOk = (wrote && read &&
                              i2cGyro.pwrMgmtReadback    == VAL_PWR_MGMT_1 &&
                              i2cGyro.gyroConfigReadback == VAL_GYRO_CONFIG &&
                              i2cGyro.dlpfReadback       == VAL_DLPF) ? 1 : 0;
    }

    // 7 — data path. A wedged bus can still ACK and return a frozen byte
    //     pattern, so identity alone is not proof. Even at rest Z dithers a
    //     few LSB; a dead-flat spread means the data is not real.
    if (i2cGyro.regWriteOk) {
        for (int i = 0; i < SAMPLE_COUNT; ++i) {
            int16_t z = 0;
            ++i2cGyro.reads;
            if (readRawZ(z, IO_TIMEOUT_MS)) noteSample(z);
            else                            { ++i2cGyro.readErrors; classifyError(); }
            HAL_Delay(5);
        }
    }

    captureLiveState();

    i2cGyro.bringUpPass = (i2cGyro.afScl       == 4 &&
                           i2cGyro.afSda       == 9 &&
                           i2cGyro.moderScl    == 2 &&
                           i2cGyro.moderSda    == 2 &&
                           i2cGyro.idleScl     == 1 &&
                           i2cGyro.idleSda     == 1 &&
                           i2cGyro.scanCount   == 1 &&
                           i2cGyro.scanAddr[0] == MPU_ADDR_7BIT &&
                           i2cGyro.deviceReady == 1 &&
                           i2cGyro.whoAmIOk    == 1 &&
                           i2cGyro.regWriteOk  == 1 &&
                           i2cGyro.dataVarying == 1 &&
                           i2cGyro.readErrors  == 0) ? 1 : 0;

    // pass starts as the bring-up result and is cleared by the FIRST live
    // failure. Rev 1's single flag was frozen at boot and read as a live
    // verdict, which hid thousands of later errors behind a green 1.
    i2cGyro.pass          = i2cGyro.bringUpPass;
    i2cGyro.lastSuccessMs = HAL_GetTick();

    robot.led().set(i2cGyro.bringUpPass ? Indicator::DOUBLE_BLINK
                                        : Indicator::TRIPLE_BLINK);
    lastLiveMs = HAL_GetTick();
}

void Test_I2C_Gyro_Update() {
    robot.led().update();

    const uint32_t now = HAL_GetTick();
    if (now - lastLiveMs < LIVE_PERIOD_MS) return;
    lastLiveMs = now;

    if (!i2cGyro.regWriteOk) return;   // nothing to poll if bring-up failed

    captureLiveState();

    int16_t z = 0;
    ++i2cGyro.reads;
    if (readRawZ(z, LIVE_TIMEOUT_MS)) {
        noteSuccess(z);
    } else {
        noteFailure();
        robot.led().set(Indicator::TRIPLE_BLINK);

        // Unwedge once the run is long enough to rule out a one-off glitch.
        // The cooldown stops a genuinely dead bus from spending every pass
        // bit-banging, which would swamp the error rate being measured.
        if (i2cGyro.consecErrors >= RECOVER_AFTER &&
            (now - lastRecoverMs) > RECOVER_COOLDOWN_MS) {
            lastRecoverMs = HAL_GetTick();
            ++i2cGyro.recoveries;
            awaitingRecoveryProof = true;
            busRecover();
            captureLiveState();
        }
    }
}


/* ==========================================================================
 * PART 2 — I2C1 + five VL53L0X + XSHUT address assignment
 * ========================================================================== */

namespace {

// ---- I2C1 pins (PINOUT.md): PB6 SCL AF4, PB7 SDA AF4 -----------------------
constexpr uint8_t I2C1_SCL_BIT = 6;
constexpr uint8_t I2C1_SDA_BIT = 7;

// ---- VL53L0X ---------------------------------------------------------------
constexpr uint8_t  VL53_DEFAULT_7BIT = 0x29;
constexpr uint16_t VL53_DEFAULT_ADDR = (VL53_DEFAULT_7BIT << 1);
constexpr uint8_t  VL53_ADDR_REG     = 0x8A;   // I2C_SLAVE_DEVICE_ADDRESS
constexpr uint8_t  VL53_MODEL_ID_REG = 0xC0;   // reads 0xEE on a real VL53L0X
constexpr uint8_t  VL53_MODEL_ID     = 0xEE;   // 238 decimal

// Same targets and boot delay as WallSensorArray::init() — the point of this
// test is to prove the production sequence, so it must not diverge from it.
constexpr uint8_t  TOF_TARGET_ADDR[5] = { 0x30, 0x31, 0x32, 0x33, 0x34 };
constexpr uint32_t XSHUT_BOOT_DELAY_MS = 10;

constexpr uint32_t TOF_SCAN_TIMEOUT_MS = 2;
constexpr uint32_t TOF_IO_TIMEOUT_MS   = 100;
constexpr uint32_t TOF_POLL_PERIOD_MS  = 50;   // sensors free-run at ~30Hz

struct XPin { GPIO_TypeDef* port; uint16_t pin; };
XPin tofXshut[5];

VL53L0X_Dev_t tofDev[5];
uint32_t lastToFPollMs = 0;

// XSHUT is open-drain (PINOUT.md): writing LOW pulls the sensor into standby,
// writing HIGH releases the pin and the breakout's own pull-up takes it to
// 2.8V. Never drive 3.3V push-pull onto it — the sensor's logic is 2.8V.
void xshutDown(int i) { HAL_GPIO_WritePin(tofXshut[i].port, tofXshut[i].pin, GPIO_PIN_RESET); }
void xshutUp  (int i) { HAL_GPIO_WritePin(tofXshut[i].port, tofXshut[i].pin, GPIO_PIN_SET);   }

void captureToFPinState() {
    // PB6/PB7 are in AFR[0] (pins 0..7), 4 bits per pin.
    i2cToF.afScl = static_cast<uint8_t>((GPIOB->AFR[0] >> (I2C1_SCL_BIT * 4)) & 0xF);
    i2cToF.afSda = static_cast<uint8_t>((GPIOB->AFR[0] >> (I2C1_SDA_BIT * 4)) & 0xF);
    i2cToF.moderScl = static_cast<uint8_t>((GPIOB->MODER >> (I2C1_SCL_BIT * 2)) & 0x3);
    i2cToF.moderSda = static_cast<uint8_t>((GPIOB->MODER >> (I2C1_SDA_BIT * 2)) & 0x3);
    i2cToF.idleScl  = static_cast<uint8_t>((GPIOB->IDR >> I2C1_SCL_BIT) & 0x1);
    i2cToF.idleSda  = static_cast<uint8_t>((GPIOB->IDR >> I2C1_SDA_BIT) & 0x1);
}

// Scans I2C1 and returns how many devices answered. Up to `maxOut` of the
// 7-bit addresses are written into out[].
uint8_t scanToFBus(uint8_t* out, uint8_t maxOut) {
    uint8_t n = 0;
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, static_cast<uint16_t>(a << 1), 1,
                                  TOF_SCAN_TIMEOUT_MS) == HAL_OK) {
            if (out && n < maxOut) out[n] = a;
            ++n;
        }
    }
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;   // clear the AF flood from no-shows
    return n;
}

bool tofReadReg8(uint16_t addr8bit, uint8_t reg, uint8_t& out) {
    return HAL_I2C_Mem_Read(&hi2c1, addr8bit, reg, 1, &out, 1,
                            TOF_IO_TIMEOUT_MS) == HAL_OK;
}

void noteToFSample(int i, uint16_t raw, uint16_t filt) {
    i2cToF.rawMm[i]  = raw;
    i2cToF.filtMm[i] = filt;
    if (filt < i2cToF.minMm[i]) i2cToF.minMm[i] = filt;
    if (filt > i2cToF.maxMm[i]) i2cToF.maxMm[i] = filt;
}

} // namespace


void Test_I2C_ToF_Init() {
    i2cToF = I2CToFReport{};
    for (int i = 0; i < 5; ++i) {
        i2cToF.minMm[i] = 0xFFFF;
        i2cToF.maxMm[i] = 0;
    }

    robot.led().init();
    robot.led().set(Indicator::FAST_BLINK);

    // Pin macros, in the index order cfg::ToFIndex defines. Same mapping as
    // WallSensorArray::init(); XSHUT_RIGHT is PB8, NOT PB10 (PB10 is I2C2_SCL).
    tofXshut[cfg::TOF_LEFT]       = { XSHUT_LEFT_GPIO_Port,  XSHUT_LEFT_Pin  };
    tofXshut[cfg::TOF_LEFTFRONT]  = { XSHUT_LF_GPIO_Port,    XSHUT_LF_Pin    };
    tofXshut[cfg::TOF_FRONT]      = { XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin };
    tofXshut[cfg::TOF_RIGHTFRONT] = { XSHUT_RF_GPIO_Port,    XSHUT_RF_Pin    };
    tofXshut[cfg::TOF_RIGHT]      = { XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin };

    // ---- Stage 0: pin level, before any transaction ------------------------
    captureToFPinState();

    // ---- Stage 1: everything down. THE "BEFORE" CHECK ----------------------
    // With all five in standby the bus must be empty. A device answering here
    // is a sensor whose XSHUT does not actually hold it down — it will still
    // be sitting at 0x29 when the next sensor boots, and the address write
    // will hit both of them at once.
    for (int i = 0; i < 5; ++i) xshutDown(i);
    HAL_Delay(XSHUT_BOOT_DELAY_MS * 3);
    i2cToF.countAllDown = scanToFBus(i2cToF.addrAllDown, 4);

    // ---- Stage 2: bring them up one at a time ------------------------------
    for (int i = 0; i < 5; ++i) {
        xshutUp(i);
        HAL_Delay(XSHUT_BOOT_DELAY_MS);

        // Did the line physically rise? Open-drain only pulls low; the rise
        // depends on the breakout's pull-up. A 0 here means XSHUT is floating
        // and the sensor may be in an undefined state rather than running.
        i2cToF.xshutHigh[i] =
            (HAL_GPIO_ReadPin(tofXshut[i].port, tofXshut[i].pin) == GPIO_PIN_SET) ? 1 : 0;

        // Scan now: expect the i already-moved sensors plus this one at 0x29.
        i2cToF.scanCountAtStage[i] = scanToFBus(nullptr, 0);

        i2cToF.bootedAt29[i] =
            (HAL_I2C_IsDeviceReady(&hi2c1, VL53_DEFAULT_ADDR, 2,
                                   TOF_IO_TIMEOUT_MS) == HAL_OK) ? 1 : 0;

        if (!i2cToF.bootedAt29[i]) { xshutDown(i); continue; }

        // Identity before the move. An ACK alone can come from a bus glitch;
        // 0xEE out of the model-ID register cannot.
        tofReadReg8(VL53_DEFAULT_ADDR, VL53_MODEL_ID_REG, i2cToF.modelId[i]);

        // The move itself.
        uint8_t target = TOF_TARGET_ADDR[i];
        const bool wrote = (HAL_I2C_Mem_Write(&hi2c1, VL53_DEFAULT_ADDR, VL53_ADDR_REG,
                                              1, &target, 1, TOF_IO_TIMEOUT_MS) == HAL_OK);
        HAL_Delay(2);

        if (wrote) {
            const uint16_t newAddr = static_cast<uint16_t>(target << 1);

            i2cToF.movedOk[i] =
                (HAL_I2C_IsDeviceReady(&hi2c1, newAddr, 2, TOF_IO_TIMEOUT_MS) == HAL_OK) ? 1 : 0;
            if (i2cToF.movedOk[i]) i2cToF.assignedAddr[i] = target;

            // 0x29 must now be silent. If it still answers, the write landed
            // nowhere and this sensor collides with the next one released.
            i2cToF.oldAddrGone[i] =
                (HAL_I2C_IsDeviceReady(&hi2c1, VL53_DEFAULT_ADDR, 2,
                                       TOF_IO_TIMEOUT_MS) != HAL_OK) ? 1 : 0;

            // Same chip on the new address?
            tofReadReg8(newAddr, VL53_MODEL_ID_REG, i2cToF.modelIdAfter[i]);

            // Full driver bring-up: SPAD config, tuning settings, ref cal.
            if (VL53L0X_Init(&tofDev[i], &hi2c1, static_cast<uint8_t>(target << 1)) == HAL_OK) {
                i2cToF.initOk[i] = 1;
                VL53L0X_SetTimingBudget(&tofDev[i], cfg::TOF_TIMING_BUDGET_US);
                i2cToF.rangingOk[i] =
                    (VL53L0X_StartContinuous(&tofDev[i], 0) == HAL_OK) ? 1 : 0;
            }
        }

        i2cToF.stageOk[i] = (i2cToF.xshutHigh[i]        == 1 &&
                             i2cToF.bootedAt29[i]       == 1 &&
                             i2cToF.modelId[i]          == VL53_MODEL_ID &&
                             i2cToF.scanCountAtStage[i] == (i + 1) &&
                             i2cToF.movedOk[i]          == 1 &&
                             i2cToF.oldAddrGone[i]      == 1 &&
                             i2cToF.modelIdAfter[i]     == VL53_MODEL_ID &&
                             i2cToF.assignedAddr[i]     == TOF_TARGET_ADDR[i] &&
                             i2cToF.initOk[i]           == 1 &&
                             i2cToF.rangingOk[i]        == 1) ? 1 : 0;
    }

    // ---- Stage 3: the finished bus ----------------------------------------
    i2cToF.finalScanCount = scanToFBus(i2cToF.finalScanAddr, 8);

    uint8_t allStages = 1;
    for (int i = 0; i < 5; ++i) if (!i2cToF.stageOk[i]) allStages = 0;

    i2cToF.bringUpPass = (i2cToF.afScl         == 4 &&
                          i2cToF.afSda         == 4 &&
                          i2cToF.moderScl      == 2 &&
                          i2cToF.moderSda      == 2 &&
                          i2cToF.idleScl       == 1 &&
                          i2cToF.idleSda       == 1 &&
                          i2cToF.countAllDown  == 0 &&
                          i2cToF.finalScanCount == 5 &&
                          allStages) ? 1 : 0;

    i2cToF.pass = i2cToF.bringUpPass;

    robot.led().set(i2cToF.bringUpPass ? Indicator::DOUBLE_BLINK
                                       : Indicator::TRIPLE_BLINK);
    lastToFPollMs = HAL_GetTick();
}


void Test_I2C_ToF_Update() {
    robot.led().update();

    const uint32_t now = HAL_GetTick();
    if (now - lastToFPollMs < TOF_POLL_PERIOD_MS) return;
    lastToFPollMs = now;

    ++i2cToF.polls;

    for (int i = 0; i < 5; ++i) {
        if (!i2cToF.rangingOk[i]) continue;

        uint16_t filtered = 0;
        const HAL_StatusTypeDef st =
            VL53L0X_ReadRangeContinuousFiltered(&tofDev[i], &filtered);

        i2cToF.lastStatus[i] = tofDev[i].last_status;

        if (st == HAL_OK) {
            ++i2cToF.goodReads[i];
            noteToFSample(i, tofDev[i].last_raw_mm, filtered);
        } else if (st == HAL_ERROR) {
            // Bad range status — out of range or no target. Normal when a
            // sensor points into open air; only worrying if it NEVER clears.
            ++i2cToF.badStatus[i];
        }
        // HAL_BUSY: no new sample yet. Expected most passes at ~30Hz vs 20Hz
        // polling; not an error and deliberately not counted as one.
    }

    // Live verdict: every sensor that came up must still be producing data.
    uint8_t alive = 1;
    for (int i = 0; i < 5; ++i)
        if (i2cToF.rangingOk[i] && i2cToF.goodReads[i] == 0 && i2cToF.polls > 40)
            alive = 0;
    if (!alive) i2cToF.pass = 0;
}
