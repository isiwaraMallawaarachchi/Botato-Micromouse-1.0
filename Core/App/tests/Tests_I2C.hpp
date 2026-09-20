#ifndef APP_TESTS_I2C_HPP
#define APP_TESTS_I2C_HPP

#include <cstdint>

/*
 * Tests_I2C — bus-level bring-up tests for the two I2C peripherals.
 *
 * This project has no debug UART (DECISIONS.md #10), so every result is
 * written into a plain global struct and read with Live Expressions.
 *
 * Part 1 (this file): I2C2 + MPU6050.
 * Part 2 (next):      I2C1 + the five VL53L0X, XSHUT address assignment.
 *
 * IMPORTANT — runs STANDALONE. main.cpp must NOT call robot.init() when
 * RUN_TEST_I2C_GYRO is active: robot.init() blocks ~5s in gyro_.calibrate()
 * and then starts the TIM3 ISR, which reads the MPU6050 four times every
 * millisecond. Those reads would interleave with the test's own.
 *
 * ---------------------------------------------------------------------------
 * REV 2 — why the extra fields
 *
 * Rev 1 reported a clean bring-up followed by thousands of read errors, and
 * could not distinguish the two causes, which need opposite fixes:
 *
 *   (a) The F4 I2C peripheral WEDGED. One transient fault latches the BUSY
 *       flag and every later transaction fails forever — nothing self-heals.
 *       A halted debugger mid-transfer does this too: the MCU stops, the
 *       slave does not, and it is left holding SDA low.
 *       Signature: one unbroken run. maxConsecErrors ~= readErrors.
 *
 *   (b) The link is MARGINAL. I2C2's only pull-ups are on the MPU6050
 *       breakout (PINOUT.md); slow rise time at 400kHz gives scattered
 *       failures that recover on their own.
 *       Signature: maxConsecErrors small, errors spread over time.
 *
 * So rev 2 records consecutive-error runs, samples the line state and BUSY
 * flag live rather than only at boot, splits the HAL fault bits per event
 * instead of letting HAL OR them together forever, and auto-recovers so the
 * test keeps producing data instead of dying at the first fault.
 * ---------------------------------------------------------------------------
 *
 * Live Expressions — add the whole struct as one expression: i2cGyro
 *
 *   ---- verdicts (read these two first) ----
 *   bringUpPass   expect 1 — frozen snapshot of the boot checks
 *   pass          expect 1 — LIVE; cleared the moment a read fails and never
 *                            set again. This is the real verdict.
 *
 *   ---- wiring / pin level, captured at boot ----
 *   afScl 4 · afSda 9 · moderScl 2 · moderSda 2 · idleScl 1 · idleSda 1
 *   (afSda must be 9, not 4 — see DECISIONS.md #16)
 *
 *   ---- bus scan ----
 *   scanCount 1 · scanAddr[0] 104 (0x68)
 *
 *   ---- device + write path ----
 *   deviceReady 1 · whoAmI 104 · whoAmIOk 1
 *   pwrMgmtReadback 0 · gyroConfigReadback 8 · dlpfReadback 3 · regWriteOk 1
 *
 *   ---- data path ----
 *   rawZ / rawZMin / rawZMax · dataVarying 1
 *
 *   ---- live health: THE DIAGNOSIS ----
 *   reads             climbs ~10/s
 *   readErrors        expect 0
 *   consecErrors      current unbroken run of failures
 *   maxConsecErrors   longest run seen. Compare against readErrors:
 *                       ~equal -> wedged   (cause a)
 *                       small  -> marginal (cause b)
 *   firstErrorMs      HAL tick of the very first failure (0 = none yet)
 *   lastSuccessMs     HAL tick of the last good read
 *   errAf / errTimeout / errBerr / errArlo
 *                     per-event fault counts, NOT OR'd history
 *   lastErrorCode     ErrorCode of the most recent failure alone
 *   liveScl / liveSda line state sampled NOW. 0 = something holds it low
 *   busyFlag          I2C2 SR2 BUSY. 1 with idle lines = peripheral wedged
 *   halState          hi2c2.State (0x20 = READY)
 *   recoveries        times the bit-bang unwedge fired and re-inited I2C2
 *   recoveredOk       1 if a recovery was followed by a successful read
 *
 * LED: fast blink while testing, double = bring-up passed, triple = failure.
 */

struct I2CGyroReport {
    // wiring / pin level (boot)
    uint8_t  afScl;
    uint8_t  afSda;
    uint8_t  moderScl;
    uint8_t  moderSda;
    uint8_t  idleScl;
    uint8_t  idleSda;
    uint8_t  recoveryRun;

    // bus scan (7-bit addresses)
    uint8_t  scanCount;
    uint8_t  scanAddr[8];

    // device identity
    uint8_t  deviceReady;
    uint8_t  whoAmI;
    uint8_t  whoAmIOk;

    // write path
    uint8_t  pwrMgmtReadback;
    uint8_t  gyroConfigReadback;
    uint8_t  dlpfReadback;
    uint8_t  regWriteOk;

    // data path
    int16_t  rawZ;
    int16_t  rawZMin;
    int16_t  rawZMax;
    uint8_t  dataVarying;

    // live health
    uint32_t reads;
    uint32_t readErrors;
    uint32_t consecErrors;
    uint32_t maxConsecErrors;
    uint32_t firstErrorMs;
    uint32_t lastSuccessMs;
    uint32_t errAf;
    uint32_t errTimeout;
    uint32_t errBerr;
    uint32_t errArlo;
    uint32_t lastErrorCode;
    uint8_t  liveScl;
    uint8_t  liveSda;
    uint8_t  busyFlag;
    uint32_t halState;
    uint32_t recoveries;
    uint8_t  recoveredOk;

    uint8_t  bringUpPass;
    uint8_t  pass;
};

extern I2CGyroReport i2cGyro;

// Call once from USER CODE 2. Blocking, ~1s. Fills i2cGyro and sets the LED.
void Test_I2C_Gyro_Init();

// Call every main-loop pass. Polls the bus, tracks fault runs, and unwedges
// the peripheral after 3 consecutive failures so data keeps coming.
void Test_I2C_Gyro_Update();

#endif // APP_TESTS_I2C_HPP
