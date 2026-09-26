#include "Tests.hpp"
#include "TestConfig.h"
#include "Config.h"
#include "GyroConfig.h"
#include "ToFConfig.h"
#include "Robot.hpp"
extern "C" {
#include "i2c.h"
#include "gpio.h"
#include "tof.h"
}

/*
 * Standalone bus tests. Nothing else is initialised: only the bus under test,
 * so a failure here can only come from that bus, its wiring or its devices.
 * The LED is used for the verdict only.
 */

volatile I2cGyroReport i2cGyro;
volatile I2cToFReport  i2cToF;
volatile Tof1Report    tof1;

namespace {
    // I2C2: SCL PB10 AF4, SDA PB9 AF9 (PB3 is the right encoder, NOT I2C).  I2C1: SCL PB6 AF4, SDA PB7 AF4.
    constexpr uint8_t I2C2_SCL = 10, I2C2_SDA = 9, I2C2_AF_SCL = 4, I2C2_AF_SDA = 9;
    constexpr uint8_t I2C1_SCL = 6,  I2C1_SDA = 7, I2C1_AF     = 4;

    constexpr uint8_t VL_DEFAULT   = 0x29;
    constexpr uint8_t VL_REG_ADDR  = 0x8A;
    constexpr uint8_t VL_REG_MODEL = 0xC0;
    constexpr uint8_t VL_REG_REV   = 0xC2;
    constexpr uint8_t VL_MODEL_ID  = 0xEE;

    void reinitI2C2() { MX_I2C2_Init(); }

    void standaloneLed() {
        robot.led().init();
        robot.led().set(Indicator::BLINK);
    }
}

/* ===========================================================================
 * TEST_I2C_GYRO
 * =========================================================================== */

namespace {
    constexpr uint8_t  MPU_PWR_MGMT_1  = 0x6B;
    constexpr uint8_t  MPU_CONFIG      = 0x1A;
    constexpr uint8_t  MPU_GYRO_CONFIG = 0x1B;
    constexpr uint8_t  MPU_WHO_AM_I    = 0x75;
    constexpr uint8_t  MPU_GYRO_ZOUT_H = 0x47;
    constexpr uint32_t RECOVER_AFTER   = 10;   // consecutive errors
}

void Test_I2CGyro_Init() {
    standaloneLed();
    volatile I2cGyroReport& r = i2cGyro;

    th::PinState p = th::readPins(GPIOB, I2C2_SCL, I2C2_SDA);
    r.powerSuspect = (!p.idleScl && !p.idleSda);
    if (!p.idleScl || !p.idleSda) {
        th::recoverBus(&hi2c2, GPIOB, GPIO_PIN_10, GPIO_PIN_9, reinitI2C2);
        r.recoveryRun = 1;
        p = th::readPins(GPIOB, I2C2_SCL, I2C2_SDA);
    }
    th::vset(r.pins, p);
    r.pinsOk = th::pinsOk(p, I2C2_AF_SCL, I2C2_AF_SDA);

    uint8_t found[8] = {};
    r.scanCount = th::scan(&hi2c2, found, 8);
    for (int i = 0; i < 8; ++i) r.scanAddr[i] = found[i];

    const uint8_t a = gyrocfg::ADDR_7B;
    r.deviceReady = th::ready(&hi2c2, a);

    uint8_t v = 0;
    if (th::readReg8(&hi2c2, a, MPU_WHO_AM_I, v)) r.whoAmI = v;
    r.whoAmIOk = (r.whoAmI == gyrocfg::ADDR_7B);

    th::writeReg8(&hi2c2, a, MPU_PWR_MGMT_1, 0x00);
    th::writeReg8(&hi2c2, a, MPU_GYRO_CONFIG, gyrocfg::GYRO_CONFIG_VAL);
    th::writeReg8(&hi2c2, a, MPU_CONFIG, gyrocfg::DLPF_VAL);
    if (th::readReg8(&hi2c2, a, MPU_PWR_MGMT_1, v))  r.pwrMgmt = v;
    if (th::readReg8(&hi2c2, a, MPU_GYRO_CONFIG, v)) r.gyroConfig = v;
    if (th::readReg8(&hi2c2, a, MPU_CONFIG, v))      r.dlpf = v;
    r.configOk = (r.pwrMgmt == 0x00) &&
                 ((r.gyroConfig & 0x18) == gyrocfg::GYRO_CONFIG_VAL) &&
                 ((r.dlpf & 0x07) == gyrocfg::DLPF_VAL);

    r.bringUpPass = r.pinsOk && r.deviceReady && r.whoAmIOk && r.configOk;
    r.rawZMin = 32767;
    r.rawZMax = -32768;
    if (!r.bringUpPass) th::verdict(false);
}

void Test_I2CGyro_Update() {
    volatile I2cGyroReport& r = i2cGyro;
    robot.led().update();
    if (!r.bringUpPass) return;

    uint8_t b[2];
    const HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c2, gyrocfg::ADDR_7B << 1,
                                    MPU_GYRO_ZOUT_H, I2C_MEMADD_SIZE_8BIT, b, 2,
                                    gyrocfg::READ_TIMEOUT_MS);
    ++r.reads;

    if (st == HAL_OK) {
        r.consecErrors = 0;
        const int16_t z = static_cast<int16_t>((b[0] << 8) | b[1]);
        r.rawZ = z;
        if (z < r.rawZMin) r.rawZMin = z;
        if (z > r.rawZMax) r.rawZMax = z;
        r.dataVarying = (r.rawZMax - r.rawZMin) > 2;   // a live sensor always jitters
    } else {
        ++r.readErrors;
        if (r.firstErrorMs == 0) r.firstErrorMs = HAL_GetTick();
        const uint32_t e = HAL_I2C_GetError(&hi2c2);
        if (e & HAL_I2C_ERROR_AF)      ++r.errAf;
        if (e & HAL_I2C_ERROR_TIMEOUT) ++r.errTimeout;
        if (e & HAL_I2C_ERROR_BERR)    ++r.errBerr;
        if (e & HAL_I2C_ERROR_ARLO)    ++r.errArlo;
        if (++r.consecErrors > r.maxConsecErrors) r.maxConsecErrors = r.consecErrors;
        if (r.consecErrors >= RECOVER_AFTER) {
            th::recoverBus(&hi2c2, GPIOB, GPIO_PIN_10, GPIO_PIN_9, reinitI2C2);
            ++r.recoveries;
            r.consecErrors = 0;
        }
    }

    r.liveScl = static_cast<uint8_t>((GPIOB->IDR >> I2C2_SCL) & 1u);
    r.liveSda = static_cast<uint8_t>((GPIOB->IDR >> I2C2_SDA) & 1u);
    r.pass = r.reads > 1000 && r.readErrors == 0 && r.dataVarying;
    if (r.reads > 1000) th::verdict(r.pass);
}

/* ===========================================================================
 * TEST_I2C_TOF
 * =========================================================================== */

namespace { VL53L0X_Dev_t stagedDev[5]; }

void Test_I2CToF_Init() {
    standaloneLed();
    volatile I2cToFReport& r = i2cToF;

    th::setAllXshut(false);
    HAL_Delay(tofcfg::XSHUT_BOOT_DELAY_MS);

    th::vset(r.pins, th::readPins(GPIOB, I2C1_SCL, I2C1_SDA));
    r.pinsOk = th::pinsOk(th::readPins(GPIOB, I2C1_SCL, I2C1_SDA), I2C1_AF, I2C1_AF);

    uint8_t found[8] = {};
    r.scanAllOff = th::scan(&hi2c1, found, 8);     // must be 0

    for (int i = 0; i < cfg::TOF_COUNT; ++i) {
        const WallSensorArray::XshutPin x = WallSensorArray::xshutPin(i);
        const uint8_t addr = tofcfg::ADDR[i];
        volatile I2cToFReport::Stage& s = r.stage[i];

        HAL_GPIO_WritePin(x.port, x.pin, GPIO_PIN_SET);
        HAL_Delay(tofcfg::XSHUT_BOOT_DELAY_MS);

        s.seen29 = th::ready(&hi2c1, VL_DEFAULT);
        if (s.seen29 && th::writeReg8(&hi2c1, VL_DEFAULT, VL_REG_ADDR, addr)) {
            HAL_Delay(2);
            s.moved = th::ready(&hi2c1, addr);
        }
        if (!s.moved) {                       // keep it off the bus for the next one
            HAL_GPIO_WritePin(x.port, x.pin, GPIO_PIN_RESET);
            continue;
        }

        uint8_t m = 0;
        if (th::readReg8(&hi2c1, addr, VL_REG_MODEL, m)) s.modelId = m;

        s.initOk = VL53L0X_Init(&stagedDev[i], &hi2c1, static_cast<uint8_t>(addr << 1)) == HAL_OK;
        if (s.initOk) {
            VL53L0X_SetTimingBudget(&stagedDev[i], tofcfg::TIMING_BUDGET_US);
            uint16_t mm = 0;
            s.rangeOk = VL53L0X_ReadRangeSingleMillimeters(&stagedDev[i], &mm) == HAL_OK;
            s.rangeMm = mm;
        }
    }

    r.finalScanCount = th::scan(&hi2c1, found, 8);
    for (int i = 0; i < 8; ++i) r.finalScan[i] = found[i];

    uint8_t ok = 0;
    for (int i = 0; i < cfg::TOF_COUNT; ++i) ok += r.stage[i].rangeOk;
    r.sensorsOk = ok;
    r.pass = r.pinsOk && r.scanAllOff == 0 && ok == cfg::TOF_COUNT;
    th::verdict(r.pass);
}

void Test_I2CToF_Update() { robot.led().update(); }

/* ===========================================================================
 * TEST_TOF_SINGLE — exactly one sensor wired to I2C1, any XSHUT line
 * =========================================================================== */

namespace {
    constexpr uint8_t  LIVE_ADDR        = 0x30;  // not default: a reboot shows up at 0x29
    constexpr uint16_t CLOSE_MM         = 40;    // a zero out of this range is plausible
    constexpr uint32_t FROZEN_LIMIT     = 100;   // identical raw readings in a row
    constexpr uint32_t SILENT_LIMIT_MS  = 200;
    constexpr uint32_t MIN_HZ           = 30;
    constexpr uint32_t WARMUP_MS        = 3000;
    constexpr uint32_t RECENT_MS        = 2000;
    constexpr uint32_t ADDR_CHECK_MS    = 500;

    VL53L0X_Dev_t oneDev;
    th::Window    win;
    th::RateMeter rate;
    th::Every     addrCheck{ADDR_CHECK_MS};
    th::Every     ledTick{250};

    uint32_t liveStartMs = 0, lastSampleMs = 0;
    uint32_t lastBusErrMs = 0, lastLostMs = 0, lastZeroFarMs = 0;
    uint32_t lastSilentMs = 0, lastFrozenMs = 0;
    uint16_t prevRaw = 0, prevGood = 0;

    bool recent(uint32_t t, uint32_t now) { return t != 0 && (now - t) < RECENT_MS; }
    void fault(uint32_t bit) { tof1.faultMask = tof1.faultMask | bit; }

    uint8_t modelAt(uint8_t addr) {
        uint8_t m = 0;
        th::readReg8(&hi2c1, addr, VL_REG_MODEL, m);
        return m;
    }
}

void Test_ToFSingle_Init() {
    standaloneLed();
    volatile Tof1Report& r = tof1;

    if (testcfg::TOF1_FORCE_BUS_HZ != 0) {
        HAL_I2C_DeInit(&hi2c1);
        hi2c1.Init.ClockSpeed = testcfg::TOF1_FORCE_BUS_HZ;
        HAL_I2C_Init(&hi2c1);
    }
    r.busClockHz = hi2c1.Init.ClockSpeed;

    // Power-cycle through XSHUT so the sensor starts at its default address.
    th::setAllXshut(false);
    HAL_Delay(tofcfg::XSHUT_BOOT_DELAY_MS);
    th::setAllXshut(true);
    HAL_Delay(tofcfg::XSHUT_BOOT_DELAY_MS);

    const th::PinState p = th::readPins(GPIOB, I2C1_SCL, I2C1_SDA);
    th::vset(r.pins, p);
    r.pinsOk = th::pinsOk(p, I2C1_AF, I2C1_AF);
    if (!r.pinsOk) fault(TF_PINS);

    uint8_t found[4] = {};
    r.scanCount = th::scan(&hi2c1, found, 4);
    for (int i = 0; i < 4; ++i) r.scanAddr[i] = found[i];
    if (r.scanCount == 0) { fault(TF_NOT_FOUND); th::verdict(false); return; }
    r.foundAddr = found[0];

    // Identity
    uint8_t v = 0;
    r.modelId = modelAt(r.foundAddr);
    if (th::readReg8(&hi2c1, r.foundAddr, VL_REG_REV, v)) r.revisionId = v;
    r.identityOk = (r.modelId == VL_MODEL_ID);
    if (!r.identityOk) fault(TF_IDENTITY);

    // Address move to LIVE_ADDR, proving the address register works and
    // setting up reboot detection for the live phase.
    bool moved = th::writeReg8(&hi2c1, r.foundAddr, VL_REG_ADDR, LIVE_ADDR);
    HAL_Delay(2);
    moved = moved && th::ready(&hi2c1, LIVE_ADDR) && !th::ready(&hi2c1, VL_DEFAULT) &&
            modelAt(LIVE_ADDR) == VL_MODEL_ID;
    r.addrTestOk = moved;
    if (!moved) fault(TF_ADDR_MOVE);
    const uint8_t liveAddr = moved ? LIVE_ADDR : r.foundAddr;

    // Bring-up, identical to the real array
    r.initOk = VL53L0X_Init(&oneDev, &hi2c1, static_cast<uint8_t>(liveAddr << 1)) == HAL_OK;
    if (!r.initOk) fault(TF_INIT);
    r.budgetOk = r.initOk &&
                 VL53L0X_SetTimingBudget(&oneDev, tofcfg::TIMING_BUDGET_US) == HAL_OK;
    if (r.initOk && !r.budgetOk) fault(TF_BUDGET);
    uint16_t mm = 0;
    r.singleShotOk = r.initOk && VL53L0X_ReadRangeSingleMillimeters(&oneDev, &mm) == HAL_OK;
    r.singleShotMm = mm;
    if (r.initOk && !r.singleShotOk) fault(TF_SINGLE_SHOT);
    r.continuousOk = r.initOk && VL53L0X_StartContinuous(&oneDev, 0) == HAL_OK;
    if (r.initOk && !r.continuousOk) fault(TF_CONTINUOUS);

    r.bringUpOk = r.pinsOk && r.identityOk && r.initOk && r.budgetOk &&
                  r.singleShotOk && r.continuousOk;
    liveStartMs = lastSampleMs = HAL_GetTick();
    if (!r.bringUpOk) th::verdict(false);
}

void Test_ToFSingle_Update() {
    volatile Tof1Report& r = tof1;
    robot.led().update();
    if (!r.bringUpOk) return;

    const uint32_t now = HAL_GetTick();
    r.uptimeMs = now - liveStartMs;

    uint16_t filt = 0;
    const HAL_StatusTypeDef st = VL53L0X_ReadRangeContinuousFiltered(&oneDev, &filt);
    ++r.polls;

    if (st == HAL_TIMEOUT) {
        ++r.busErrors; lastBusErrMs = now; fault(TF_BUS_ERROR);
    } else if (st != HAL_BUSY) {                 // a measurement arrived
        ++r.samples;
        rate.count();
        lastSampleMs = now;
        const uint16_t raw = oneDev.last_raw_mm;
        r.rawMm = raw;
        r.lastStatus = oneDev.last_status;
        r.statusHist[oneDev.last_status & 0x0F] = r.statusHist[oneDev.last_status & 0x0F] + 1;

        if (st == HAL_OK) {
            ++r.goodReads;
            r.filtMm = filt;
            win.push(raw);
            r.winMin = win.min(); r.winMax = win.max();
            r.winMean = win.mean(); r.winSpread = win.spread();

            if (raw == 0) {
                if (prevGood > CLOSE_MM) { ++r.zeroFar; lastZeroFarMs = now; fault(TF_ZERO_FAR); }
                else                     { ++r.zeroNear; }
                r.lastZeroPrevMm = prevGood;
            }
            r.frozenRun = (raw == prevRaw) ? r.frozenRun + 1 : 0;
            if (r.frozenRun > r.maxFrozenRun) r.maxFrozenRun = r.frozenRun;
            if (r.frozenRun > FROZEN_LIMIT) { lastFrozenMs = now; fault(TF_FROZEN); }
            prevRaw = raw;
            prevGood = raw;
        } else {
            ++r.badStatus;                       // no target in range: not a fault
        }
    }

    r.silentMs = now - lastSampleMs;
    if (r.silentMs > r.maxSilentMs) r.maxSilentMs = r.silentMs;
    if (r.silentMs > SILENT_LIMIT_MS) { lastSilentMs = now; fault(TF_SILENT); }

    if (addrCheck.due()) {
        ++r.addrChecks;
        const uint8_t live = static_cast<uint8_t>(oneDev.address >> 1);
        if (!th::ready(&hi2c1, live, 5)) { ++r.lost; lastLostMs = now; fault(TF_LOST); }
        if (live != VL_DEFAULT && th::ready(&hi2c1, VL_DEFAULT, 5)) { ++r.reboots; fault(TF_REBOOTED); }
    }

    rate.update(now);
    r.sampleHz = rate.hz();
    r.warmedUp = r.uptimeMs > WARMUP_MS;
    if (r.warmedUp) {
        if (r.minSampleHz == 0 || r.sampleHz < r.minSampleHz) r.minSampleHz = r.sampleHz;
        if (r.sampleHz < MIN_HZ) fault(TF_SLOW);
    }

    r.healthy = r.warmedUp && r.goodReads > 0 && r.reboots == 0 &&
                r.sampleHz >= MIN_HZ &&
                !recent(lastBusErrMs, now) && !recent(lastLostMs, now) &&
                !recent(lastZeroFarMs, now) && !recent(lastSilentMs, now) &&
                !recent(lastFrozenMs, now);

    if (ledTick.due() && r.warmedUp) th::verdict(r.healthy);
}
