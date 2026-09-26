#ifndef APP_TESTS_TOF_SINGLE_HPP
#define APP_TESTS_TOF_SINGLE_HPP

#include <cstdint>

/*
 * Tests_ToF_Single — exhaustive bench test of ONE VL53L0X, in isolation.
 *
 * Purpose: the assembled robot works everywhere except the ToF array, so each
 * sensor is taken off the robot and tested completely alone. Nothing else is
 * connected to the STM32 — no MPU6050, no motor driver, no other ToF. That
 * removes every shared variable at once (address collisions, combined bus
 * loading, XSHUT sequencing, motor PWM noise, rail current draw), so whatever
 * still misbehaves belongs to this sensor or its own four wires.
 *
 * --------------------------------------------------------------------------
 * BENCH WIRING — this is the whole circuit
 *
 *     Black Pill 3V3  ->  sensor VCC
 *     Black Pill GND  ->  sensor GND
 *     PB6 (I2C1 SCL)  ->  sensor SCL
 *     PB7 (I2C1 SDA)  ->  sensor SDA
 *     XSHUT           ->  leave unwired (see below)
 *
 * Board powered from USB / ST-Link. No battery, no regulators, no motors.
 * --------------------------------------------------------------------------
 *
 * THE ONE PULL-UP PROBLEM. PINOUT.md records that I2C1 has no pull-ups on the
 * MCU board — every one is on a ToF breakout, and five in parallel give about
 * 2k. Alone, this sensor puts a single ~10k on the bus. At 400kHz Fast Mode
 * that rise time is marginal, and it can manufacture exactly the intermittent
 * faults this test exists to find. So if the sensor does not appear, or if
 * i2cErrors climb during ranging, set:
 *
 *     #define TOF1_FORCE_BUS_HZ 100000      (in Tests_ToF_Single.cpp)
 *
 * which re-inits I2C1 at Standard Mode for this test only, without touching
 * the .ioc. tof1.busClockHz reports what was actually used. If a sensor is
 * clean at 100kHz and faulty at 400kHz, that is the wiring, not the sensor —
 * and it says nothing about behaviour on the real board, where four more
 * pull-ups sit in parallel.
 *
 * XSHUT. Not needed: with one device there is nothing to sequence. But
 * MX_GPIO_Init() drives all five XSHUT pins LOW at boot, and LOW is hardware
 * standby, so this test releases all five before touching I2C — otherwise a
 * sensor whose XSHUT happens to be wired would sit in reset and look dead.
 * Most VL53L0X breakouts pull XSHUT up on-board, so leaving it unwired runs
 * the sensor enabled. If scanCount comes back 0, wire XSHUT to any of the
 * five XSHUT pins (PA10, PB4, PB5, PA7, PB8) and re-run: those are open-drain
 * and release high, which is safe for the sensor's 2.8V logic.
 *
 * WHAT IT CHECKS, in order. Each phase gates the next, so the first failing
 * field is the root cause and everything below it reads 0.
 *
 *   1. Bus pins      PB6/PB7 really are I2C1 AF4, lines idle high
 *   2. Presence      exactly one device answers a full 7-bit scan
 *   3. Identity      MODEL_ID 0xC0 = 238 (0xEE), REVISION_ID 0xC2
 *   4. Address       0x29 -> 0x30, verify; 0x30 -> 0x29, verify; back to 0x30.
 *                    Both directions, each confirmed by a model-ID read at the
 *                    new address and silence at the old one.
 *   5. Bring-up      VL53L0X_Init (SPAD + tuning + reference calibration),
 *                    timing budget, one blocking single-shot range
 *   6. Continuous    start continuous mode, then sample it live
 *   7. Live faults   the ways a VL53L0X misbehaves without simply vanishing:
 *                      - I2C link errors during ranging
 *                      - silent reset (reappears at the factory 0x29)
 *                      - sample rate collapse
 *                      - stale data / measurements stopping
 *                      - frozen value (same raw over and over)
 *                      - zero-distance readings with a VALID status
 *                      - persistent bad range status
 *
 * The sensor is left running at 0x30, NOT 0x29, on purpose: a VL53L0X only
 * returns to 0x29 by power-on reset, so 0x29 answering during the live phase
 * is positive proof the sensor rebooted. That is exactly the failure already
 * seen on the LeftFront unit.
 *
 * LIVE EXPRESSIONS: add ONE expression and expand it.
 *
 *     tof1
 *
 * The two fields to read first:
 *
 *     tof1.healthy     1 = sensor is good RIGHT NOW (live, recomputed)
 *     tof1.faultMask   0 = nothing has ever failed (sticky). Non-zero names
 *                      every fault seen since boot — see FAULT bits below.
 *
 * LED: fast blink while testing · double blink = healthy ·
 *      triple blink = something failed.
 */

// faultMask bits. Sticky: once set, stays set, so a fault lasting one
// millisecond three minutes ago is still visible.
enum ToFFault : uint32_t {
    TOFF_BUS_PINS      = 0x0001,  // PB6/PB7 not AF4, or a line not idling high
    TOFF_DEVICE_COUNT  = 0x0002,  // not exactly one device on the bus
    TOFF_IDENTITY      = 0x0004,  // MODEL_ID not 0xEE
    TOFF_ADDR_CHANGE   = 0x0008,  // an address write did not take
    TOFF_INIT          = 0x0010,  // VL53L0X_Init failed (often ref-cal power)
    TOFF_SINGLE_SHOT   = 0x0020,  // blocking single measurement failed
    TOFF_CONTINUOUS    = 0x0040,  // continuous mode would not start
    TOFF_NO_DATA       = 0x0080,  // no good reading after warm-up
    TOFF_I2C_ERRORS    = 0x0100,  // bus faults while ranging
    TOFF_REBOOT        = 0x0200,  // answered at 0x29 = it power-cycled
    TOFF_SLOW_RATE     = 0x0400,  // sample rate below the timing budget's floor
    TOFF_STALE         = 0x0800,  // gap between good readings too long
    TOFF_FROZEN        = 0x1000,  // identical raw value repeating
    TOFF_ZERO_READING  = 0x2000,  // raw 0 with a VALID status (DECISIONS #27)
    TOFF_BAD_STATUS    = 0x4000,  // sustained run of invalid range statuses
};

struct ToFSingleReport {
    /* ---- phase 1: bus (I2C1 = PB6 SCL, PB7 SDA, both AF4) --------------- */
    uint8_t  afScl;              // expect 4
    uint8_t  afSda;              // expect 4
    uint8_t  moderScl;           // expect 2 (alternate function)
    uint8_t  moderSda;           // expect 2
    uint8_t  idleScl;            // expect 1
    uint8_t  idleSda;            // expect 1  (both 0 = no pull-up / no 3V3)
    uint8_t  xshutReleased;      // expect 1  (all five lines let go high)
    uint32_t busClockHz;         // speed actually in use on I2C1
    uint8_t  busClockForced;     // 1 = TOF1_FORCE_BUS_HZ overrode the .ioc

    /* ---- phase 2: presence --------------------------------------------- */
    uint8_t  scanCount;          // expect 1. 0 = absent, >1 = extra device
    uint8_t  scanAddr[4];        // 7-bit addresses that answered
    uint8_t  foundAddr;          // where it was found: 41 (0x29) on a cold board
    uint8_t  startedAtNonDefault;// 1 = left over from a previous run, not a fault

    /* ---- phase 3: identity --------------------------------------------- */
    uint8_t  modelId;            // expect 238 (0xEE)
    uint8_t  revisionId;         // informational (commonly 16)
    uint8_t  identityOk;

    /* ---- phase 4: address change, both directions ----------------------- */
    uint8_t  movedTo30;          // 0x29 -> 0x30 took
    uint8_t  addr29SilentAfter;  // old address went quiet
    uint8_t  modelIdAt30;        // expect 238 — same chip on the new address
    uint8_t  movedBackTo29;      // 0x30 -> 0x29 took (reverse direction)
    uint8_t  modelIdAt29;        // expect 238
    uint8_t  finalAddr;          // expect 48 (0x30) — where it runs from here
    uint8_t  addrTestOk;

    /* ---- phase 5: driver bring-up --------------------------------------- */
    uint8_t  initOk;             // SPAD config + tuning + reference calibration
    uint8_t  budgetOk;
    uint32_t budgetUs;           // cfg::TOF_TIMING_BUDGET_US
    uint8_t  singleShotOk;
    uint16_t singleShotMm;
    uint8_t  continuousOk;
    uint8_t  bringUpOk;          // all of phases 1-5

    /* ---- phase 6: live sampling ----------------------------------------- */
    uint32_t polls;              // poll attempts (many return "no new sample")
    uint32_t freshSamples;       // measurements the sensor actually produced
    uint32_t goodReads;          // status 11 and in range
    uint32_t badStatus;          // status != 11 (normal when aimed at open air)
    uint32_t busyReads;          // no new sample yet — expected, not a fault
    uint32_t i2cErrors;          // bus fault during a ranging read
    uint16_t rawMm;              // latest unfiltered
    uint16_t filtMm;             // latest after median + EMA
    uint16_t minMm;              // lifetime
    uint16_t maxMm;
    uint16_t windowMin;          // last 32 good samples
    uint16_t windowMax;
    uint16_t windowMean;
    uint16_t spreadMm;           // windowMax - windowMin = noise at this distance
    uint8_t  lastStatus;         // 11 = valid
    uint32_t statusHist[16];     // how many of each range-status code

    /* ---- phase 7: failure-mode detectors -------------------------------- */
    uint32_t zeroReadings;       // raw 0 with valid status — must stay 0
    uint32_t frozenRun;          // current run of identical raw values
    uint32_t maxFrozenRun;       // longest such run
    uint32_t consecBad;
    uint32_t maxConsecBad;
    uint32_t staleMs;            // since the last good reading
    uint32_t maxStaleMs;
    uint32_t sampleHz;           // measured, over a 1s window
    uint32_t minSampleHz;        // worst second seen after warm-up
    uint32_t addrChecks;         // integrity probes run
    uint32_t reboots;            // times 0x29 answered = sensor reset itself
    uint32_t lostAtLiveAddr;     // times 0x30 stopped answering
    uint32_t uptimeMs;

    /* ---- verdict --------------------------------------------------------- */
    uint8_t  warmedUp;           // 1 once the grace period has elapsed
    uint8_t  healthy;            // THE FLAG — live, recomputed every poll
    uint32_t faultMask;          // sticky; see ToFFault
};

extern ToFSingleReport tof1;

void Test_ToF_Single_Init();     // blocking, ~2s
void Test_ToF_Single_Update();   // call every main-loop pass

#endif // APP_TESTS_TOF_SINGLE_HPP
