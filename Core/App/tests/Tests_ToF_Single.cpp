#include "Tests_ToF_Single.hpp"
#include "Robot.hpp"
#include "Config.h"

extern "C" {
#include "i2c.h"
#include "gpio.h"
#include "tof.h"
}

/* --------------------------------------------------------------------------
 * I2C1 bus speed for this test.
 *
 * 0 = keep whatever CubeMX generated (400kHz Fast Mode).
 *
 * On the assembled robot, five ToF breakouts put five ~10k pull-ups in
 * parallel (~2k) and 400kHz is comfortable. On this bench there is ONE sensor,
 * so the bus has a single ~10k pull-up and the rise time is marginal. If the
 * sensor does not show up, or i2cErrors climb while ranging, set this to
 * 100000 and re-run before blaming the sensor.
 * -------------------------------------------------------------------------- */
#define TOF1_FORCE_BUS_HZ 0

ToFSingleReport tof1;

namespace {

// ---- I2C1 pins (PINOUT.md): PB6 SCL AF4, PB7 SDA AF4 -----------------------
constexpr uint8_t SCL_BIT = 6;
constexpr uint8_t SDA_BIT = 7;

// ---- VL53L0X registers and constants ---------------------------------------
constexpr uint8_t  ADDR_DEFAULT_7B = 0x29;
constexpr uint8_t  ADDR_LIVE_7B    = 0x30;   // where the sensor runs afterwards
constexpr uint8_t  REG_ADDR        = 0x8A;   // I2C_SLAVE_DEVICE_ADDRESS
constexpr uint8_t  REG_MODEL_ID    = 0xC0;   // reads 0xEE
constexpr uint8_t  REG_REVISION_ID = 0xC2;
constexpr uint8_t  MODEL_ID_VALUE  = 0xEE;   // 238

constexpr uint32_t SCAN_TIMEOUT_MS  = 2;
constexpr uint32_t IO_TIMEOUT_MS    = 100;
constexpr uint32_t PROBE_TIMEOUT_MS = 5;     // live integrity probes, kept short
constexpr uint32_t BOOT_DELAY_MS    = 10;

// ---- live sampling tuning ---------------------------------------------------
constexpr uint32_t POLL_PERIOD_MS = 5;     // sensor free-runs ~30Hz; 5ms catches
                                           // each sample promptly without
                                           // flooding the bus
constexpr uint32_t WARMUP_MS      = 3000;  // grace before judging health
constexpr uint32_t ADDR_CHECK_MS  = 2000;  // reset-detection probe interval
constexpr uint32_t RATE_WINDOW_MS = 1000;

// Thresholds. Derived from the 33ms timing budget: ~30Hz nominal, so 15Hz is
// half rate and unambiguous. A sensor that merely points at air still posts
// measurements at full rate — only a sick one slows down.
constexpr uint32_t MIN_SAMPLE_HZ    = 15;
constexpr uint32_t MAX_STALE_MS     = 400;   // >10 missed measurement periods
constexpr uint32_t MAX_FROZEN_RUN   = 40;    // identical raw 40x = not measuring
constexpr uint32_t MAX_CONSEC_BAD   = 200;   // ~7s of unbroken invalid status

constexpr int WIN_N = 32;

VL53L0X_Dev_t dev;

uint32_t startMs        = 0;
uint32_t lastPollMs     = 0;
uint32_t lastGoodMs     = 0;
uint32_t lastAddrChkMs  = 0;
uint32_t rateWindowMs   = 0;
uint32_t rateCounter    = 0;
uint16_t lastRawSeen    = 0xFFFF;

uint16_t win[WIN_N];
int      winIdx   = 0;
int      winCount = 0;

inline uint16_t addr8(uint8_t addr7) { return static_cast<uint16_t>(addr7 << 1); }

void fault(uint32_t bit) { tof1.faultMask |= bit; }

// Re-init I2C1 at a different speed without touching the .ioc. Only the clock
// changes; pin config and everything else come from the generated init.
void applyBusClock() {
    tof1.busClockHz = hi2c1.Init.ClockSpeed;
#if TOF1_FORCE_BUS_HZ > 0
    HAL_I2C_DeInit(&hi2c1);
    hi2c1.Init.ClockSpeed = TOF1_FORCE_BUS_HZ;
    HAL_I2C_Init(&hi2c1);
    tof1.busClockHz    = hi2c1.Init.ClockSpeed;
    tof1.busClockForced = 1;
#endif
}

// --- XSHUT ------------------------------------------------------------------
// Not used for sequencing — there is only one device. But MX_GPIO_Init drives
// every XSHUT pin LOW, which is hardware standby, so all five are released
// here in case the sensor's XSHUT happens to be wired. Open-drain: writing
// HIGH lets go of the pin and the breakout's pull-up brings it to 2.8V. Never
// drive 3.3V push-pull onto it.
void releaseAllXshut() {
    HAL_GPIO_WritePin(XSHUT_LEFT_GPIO_Port,  XSHUT_LEFT_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(XSHUT_LF_GPIO_Port,    XSHUT_LF_Pin,    GPIO_PIN_SET);
    HAL_GPIO_WritePin(XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(XSHUT_RF_GPIO_Port,    XSHUT_RF_Pin,    GPIO_PIN_SET);
    HAL_GPIO_WritePin(XSHUT_RIGHT_GPIO_Port, XSHUT_RIGHT_Pin, GPIO_PIN_SET);
    HAL_Delay(BOOT_DELAY_MS * 3);            // let the sensor boot
    tof1.xshutReleased = 1;
}

void capturePins() {
    // PB6/PB7 live in AFR[0] (pins 0..7), four bits per pin.
    tof1.afScl    = static_cast<uint8_t>((GPIOB->AFR[0] >> (SCL_BIT * 4)) & 0xF);
    tof1.afSda    = static_cast<uint8_t>((GPIOB->AFR[0] >> (SDA_BIT * 4)) & 0xF);
    tof1.moderScl = static_cast<uint8_t>((GPIOB->MODER  >> (SCL_BIT * 2)) & 0x3);
    tof1.moderSda = static_cast<uint8_t>((GPIOB->MODER  >> (SDA_BIT * 2)) & 0x3);
    tof1.idleScl  = static_cast<uint8_t>((GPIOB->IDR    >> SCL_BIT) & 0x1);
    tof1.idleSda  = static_cast<uint8_t>((GPIOB->IDR    >> SDA_BIT) & 0x1);
}

bool ready(uint8_t addr7, uint32_t timeout = IO_TIMEOUT_MS) {
    const bool r = (HAL_I2C_IsDeviceReady(&hi2c1, addr8(addr7), 2, timeout) == HAL_OK);
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;   // a no-show leaves an AF flag set
    return r;
}

bool readReg(uint8_t addr7, uint8_t reg, uint8_t& out) {
    const bool r = (HAL_I2C_Mem_Read(&hi2c1, addr8(addr7), reg, 1, &out, 1,
                                     IO_TIMEOUT_MS) == HAL_OK);
    if (!r) hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
    return r;
}

// Writes the new address, waits, then proves the move three ways: the new
// address answers, the old one has gone quiet, and the model ID still reads
// 0xEE from the new address. An address write can ACK and do nothing, so
// checking only the new address is not enough.
bool moveAddress(uint8_t from7, uint8_t to7, uint8_t* modelIdOut, uint8_t* oldSilentOut) {
    uint8_t target = to7;
    if (HAL_I2C_Mem_Write(&hi2c1, addr8(from7), REG_ADDR, 1, &target, 1,
                          IO_TIMEOUT_MS) != HAL_OK) {
        hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
        return false;
    }
    HAL_Delay(2);

    const bool newOk = ready(to7);
    if (oldSilentOut) *oldSilentOut = ready(from7) ? 0 : 1;
    if (modelIdOut)   readReg(to7, REG_MODEL_ID, *modelIdOut);
    return newOk;
}

uint8_t scanBus() {
    uint8_t n = 0;
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr8(a), 1, SCAN_TIMEOUT_MS) == HAL_OK) {
            if (n < 4) tof1.scanAddr[n] = a;
            if (n == 0) tof1.foundAddr = a;
            ++n;
        }
    }
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
    return n;
}

void pushWindow(uint16_t v) {
    win[winIdx] = v;
    winIdx = (winIdx + 1) % WIN_N;
    if (winCount < WIN_N) ++winCount;

    uint16_t lo = 0xFFFF, hi = 0;
    uint32_t sum = 0;
    for (int i = 0; i < winCount; ++i) {
        if (win[i] < lo) lo = win[i];
        if (win[i] > hi) hi = win[i];
        sum += win[i];
    }
    tof1.windowMin  = lo;
    tof1.windowMax  = hi;
    tof1.windowMean = static_cast<uint16_t>(sum / static_cast<uint32_t>(winCount));
    tof1.spreadMm   = static_cast<uint16_t>(hi - lo);
}

// Live reset detection. A VL53L0X only returns to 0x29 through a power-on
// reset, so 0x29 answering while we are running at 0x30 is proof the sensor
// browned out and rebooted — the exact failure seen on the LeftFront unit.
void addressIntegrityCheck() {
    ++tof1.addrChecks;

    if (ready(ADDR_DEFAULT_7B, PROBE_TIMEOUT_MS)) {
        ++tof1.reboots;
        fault(TOFF_REBOOT);
    }
    if (!ready(ADDR_LIVE_7B, PROBE_TIMEOUT_MS)) {
        ++tof1.lostAtLiveAddr;
        fault(TOFF_REBOOT);
    }
}

void evaluateHealth() {
    const uint32_t now = HAL_GetTick();
    tof1.uptimeMs = now - startMs;
    tof1.warmedUp = (tof1.uptimeMs > WARMUP_MS) ? 1 : 0;
    if (!tof1.warmedUp) { tof1.healthy = 0; return; }

    if (tof1.goodReads == 0)                    fault(TOFF_NO_DATA);
    if (tof1.i2cErrors > 0)                     fault(TOFF_I2C_ERRORS);
    if (tof1.sampleHz < MIN_SAMPLE_HZ)          fault(TOFF_SLOW_RATE);
    if (tof1.maxStaleMs > MAX_STALE_MS)         fault(TOFF_STALE);
    if (tof1.maxFrozenRun > MAX_FROZEN_RUN)     fault(TOFF_FROZEN);
    if (tof1.zeroReadings > 0)                  fault(TOFF_ZERO_READING);
    if (tof1.maxConsecBad > MAX_CONSEC_BAD)     fault(TOFF_BAD_STATUS);

    // healthy is a LIVE verdict: everything must be fine right now. faultMask
    // is the sticky record of anything that ever went wrong.
    tof1.healthy = (tof1.bringUpOk                    &&
                    tof1.goodReads   > 0              &&
                    tof1.i2cErrors   == 0             &&
                    tof1.reboots     == 0             &&
                    tof1.lostAtLiveAddr == 0          &&
                    tof1.zeroReadings == 0            &&
                    tof1.sampleHz    >= MIN_SAMPLE_HZ &&
                    tof1.staleMs     <= MAX_STALE_MS  &&
                    tof1.frozenRun   <= MAX_FROZEN_RUN) ? 1 : 0;
}

} // namespace


void Test_ToF_Single_Init() {
    tof1 = ToFSingleReport{};
    tof1.minMm     = 0xFFFF;
    tof1.windowMin = 0xFFFF;

    robot.led().init();
    robot.led().set(Indicator::FAST_BLINK);

    // ---- phase 1: bus, and let the sensor out of standby -------------------
    applyBusClock();
    releaseAllXshut();
    capturePins();
    if (!(tof1.afScl == 4 && tof1.afSda == 4 &&
          tof1.moderScl == 2 && tof1.moderSda == 2 &&
          tof1.idleScl == 1 && tof1.idleSda == 1)) {
        fault(TOFF_BUS_PINS);
    }

    // ---- phase 2: presence -------------------------------------------------
    tof1.scanCount = scanBus();
    if (tof1.scanCount != 1) fault(TOFF_DEVICE_COUNT);
    if (tof1.scanCount == 0) {
        tof1.bringUpOk = 0;
        robot.led().set(Indicator::TRIPLE_BLINK);
        startMs = HAL_GetTick();
        return;                       // nothing to talk to; stop here
    }
    tof1.startedAtNonDefault = (tof1.foundAddr != ADDR_DEFAULT_7B) ? 1 : 0;

    // ---- phase 3: identity --------------------------------------------------
    readReg(tof1.foundAddr, REG_MODEL_ID,    tof1.modelId);
    readReg(tof1.foundAddr, REG_REVISION_ID, tof1.revisionId);
    tof1.identityOk = (tof1.modelId == MODEL_ID_VALUE) ? 1 : 0;
    if (!tof1.identityOk) fault(TOFF_IDENTITY);

    // ---- phase 4: address change, both directions ---------------------------
    // Normalise to 0x29 first if a previous run left it elsewhere (the
    // assignment lives in sensor RAM and survives a reflash, but not a
    // power cycle).
    uint8_t base = tof1.foundAddr;
    if (base != ADDR_DEFAULT_7B) {
        uint8_t dummyId = 0, dummySilent = 0;
        if (moveAddress(base, ADDR_DEFAULT_7B, &dummyId, &dummySilent))
            base = ADDR_DEFAULT_7B;
    }

    if (base == ADDR_DEFAULT_7B) {
        // forward: 0x29 -> 0x30
        tof1.movedTo30 = moveAddress(ADDR_DEFAULT_7B, ADDR_LIVE_7B,
                                     &tof1.modelIdAt30,
                                     &tof1.addr29SilentAfter) ? 1 : 0;

        // reverse: 0x30 -> 0x29. Proves the write path both ways, not just once.
        if (tof1.movedTo30) {
            uint8_t silent30 = 0;
            tof1.movedBackTo29 = moveAddress(ADDR_LIVE_7B, ADDR_DEFAULT_7B,
                                             &tof1.modelIdAt29, &silent30) ? 1 : 0;
        }

        // settle at 0x30 for the live phase, so any later appearance of 0x29
        // can only mean the sensor reset itself.
        if (tof1.movedBackTo29) {
            uint8_t id = 0, silent = 0;
            if (moveAddress(ADDR_DEFAULT_7B, ADDR_LIVE_7B, &id, &silent))
                tof1.finalAddr = ADDR_LIVE_7B;
        }
    }

    tof1.addrTestOk = (tof1.movedTo30         == 1 &&
                       tof1.addr29SilentAfter == 1 &&
                       tof1.modelIdAt30       == MODEL_ID_VALUE &&
                       tof1.movedBackTo29     == 1 &&
                       tof1.modelIdAt29       == MODEL_ID_VALUE &&
                       tof1.finalAddr         == ADDR_LIVE_7B) ? 1 : 0;
    if (!tof1.addrTestOk) fault(TOFF_ADDR_CHANGE);

    // ---- phase 5: driver bring-up -------------------------------------------
    if (tof1.finalAddr == ADDR_LIVE_7B) {
        // Init ends in reference calibration, which fires the VCSEL. That is
        // the largest current draw of the whole sequence, so a sensor with a
        // marginal supply or solder joint typically dies exactly here after
        // sailing through every register write above.
        tof1.initOk = (VL53L0X_Init(&dev, &hi2c1, addr8(ADDR_LIVE_7B)) == HAL_OK) ? 1 : 0;
        if (!tof1.initOk) fault(TOFF_INIT);
    }

    if (tof1.initOk) {
        tof1.budgetUs = cfg::TOF_TIMING_BUDGET_US;
        tof1.budgetOk = (VL53L0X_SetTimingBudget(&dev, tof1.budgetUs) == HAL_OK) ? 1 : 0;

        uint16_t mm = 0;
        tof1.singleShotOk = (VL53L0X_ReadRangeSingleMillimeters(&dev, &mm) == HAL_OK) ? 1 : 0;
        tof1.singleShotMm = mm;
        if (!tof1.singleShotOk) fault(TOFF_SINGLE_SHOT);

        tof1.continuousOk = (VL53L0X_StartContinuous(&dev, 0) == HAL_OK) ? 1 : 0;
        if (!tof1.continuousOk) fault(TOFF_CONTINUOUS);
    }

    tof1.bringUpOk = (tof1.faultMask == 0 &&
                      tof1.identityOk && tof1.addrTestOk &&
                      tof1.initOk && tof1.budgetOk &&
                      tof1.singleShotOk && tof1.continuousOk) ? 1 : 0;

    robot.led().set(tof1.bringUpOk ? Indicator::DOUBLE_BLINK : Indicator::TRIPLE_BLINK);

    const uint32_t now = HAL_GetTick();
    startMs = now; lastPollMs = now; lastGoodMs = now;
    lastAddrChkMs = now; rateWindowMs = now;
}


void Test_ToF_Single_Update() {
    robot.led().update();

    const uint32_t now = HAL_GetTick();
    if (now - lastPollMs < POLL_PERIOD_MS) return;
    lastPollMs = now;

    if (!tof1.continuousOk) { evaluateHealth(); return; }

    ++tof1.polls;

    // Clear first so a non-zero afterwards can only have come from this read.
    // The driver returns HAL_ERROR for BOTH an I2C fault and a bad range
    // status, so the ErrorCode is the only way to tell them apart.
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;

    uint16_t filtered = 0;
    const HAL_StatusTypeDef st = VL53L0X_ReadRangeContinuousFiltered(&dev, &filtered);
    const uint32_t busErr = hi2c1.ErrorCode;

    if (st == HAL_BUSY) {
        ++tof1.busyReads;              // no new measurement yet — expected
    }
    else if (busErr != HAL_I2C_ERROR_NONE) {
        ++tof1.i2cErrors;
        fault(TOFF_I2C_ERRORS);
        hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
    }
    else {
        ++tof1.freshSamples;
        ++rateCounter;

        tof1.lastStatus = dev.last_status;
        if (dev.last_status < 16) ++tof1.statusHist[dev.last_status];

        tof1.rawMm = dev.last_raw_mm;

        if (st == HAL_OK) {
            ++tof1.goodReads;
            tof1.consecBad = 0;
            lastGoodMs = now;

            tof1.filtMm = filtered;
            if (filtered < tof1.minMm) tof1.minMm = filtered;
            if (filtered > tof1.maxMm) tof1.maxMm = filtered;
            pushWindow(filtered);

            // A distance sensor reporting 0 with a VALID status is the trap
            // from DECISIONS.md #27 — downstream that reads as "wall touching
            // the nose" and ends a cell after 10mm of travel.
            if (dev.last_raw_mm == 0) {
                ++tof1.zeroReadings;
                fault(TOFF_ZERO_READING);
            }

            // Identical raw values repeating means the sensor is handing back
            // a held register rather than actually ranging. Real measurements
            // dither by a millimetre or two even against a fixed target.
            if (dev.last_raw_mm == lastRawSeen) {
                ++tof1.frozenRun;
                if (tof1.frozenRun > tof1.maxFrozenRun) tof1.maxFrozenRun = tof1.frozenRun;
            } else {
                tof1.frozenRun = 0;
            }
            lastRawSeen = dev.last_raw_mm;
        }
        else {
            ++tof1.badStatus;          // status != 11, or >= 8000 = no target
            ++tof1.consecBad;
            if (tof1.consecBad > tof1.maxConsecBad) tof1.maxConsecBad = tof1.consecBad;
        }
    }

    // staleness
    tof1.staleMs = now - lastGoodMs;
    if (tof1.staleMs > tof1.maxStaleMs) tof1.maxStaleMs = tof1.staleMs;

    // measured sample rate over a 1s window
    if (now - rateWindowMs >= RATE_WINDOW_MS) {
        tof1.sampleHz = rateCounter;
        rateCounter   = 0;
        rateWindowMs  = now;
        if (tof1.warmedUp &&
            (tof1.minSampleHz == 0 || tof1.sampleHz < tof1.minSampleHz))
            tof1.minSampleHz = tof1.sampleHz;
    }

    // periodic reset detection
    if (now - lastAddrChkMs >= ADDR_CHECK_MS) {
        lastAddrChkMs = now;
        addressIntegrityCheck();
    }

    evaluateHealth();

    robot.led().set(tof1.healthy ? Indicator::DOUBLE_BLINK : Indicator::TRIPLE_BLINK);
}
