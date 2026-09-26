#ifndef APP_TESTS_I2C_HPP
#define APP_TESTS_I2C_HPP

#include <cstdint>

/*
 * Tests_I2C — bus-level bring-up tests for the two I2C peripherals.
 *
 * No debug UART in this project (DECISIONS.md #10), so every result is
 * written into a plain global struct and read with Live Expressions.
 *
 *   Part 1: I2C2 + MPU6050              -> i2cGyro   (Test_I2C_Gyro_*)
 *   Part 2: I2C1 + 5x VL53L0X + XSHUT   -> i2cToF    (Test_I2C_ToF_*)
 *
 * IMPORTANT — both run STANDALONE. main.cpp must NOT call robot.init() when
 * either RUN_TEST_I2C_* macro is active:
 *   - robot.init() blocks ~5s in gyro_.calibrate() and starts the TIM3 ISR,
 *     which reads the MPU6050 four times every millisecond.
 *   - robot.init() also runs WallSensorArray::init(), which performs its OWN
 *     XSHUT address-assignment sequence. Two sequences racing on one bus
 *     produce a result nobody can interpret.
 */

/* ==========================================================================
 * PART 1 — I2C2 + MPU6050
 * ==========================================================================
 *
 * Rev 2 note: rev 1 could not tell a WEDGED peripheral (one transient fault
 * latches BUSY and every later transaction fails forever) from a MARGINAL
 * link (scattered faults that self-recover). Those need opposite fixes, so
 * rev 2 tracks consecutive-error runs, samples line state and BUSY live, and
 * splits the HAL fault bits per event instead of letting HAL OR them forever.
 *
 * Live Expressions — add the whole struct: i2cGyro
 *
 *   bringUpPass  1 — frozen snapshot of the boot checks
 *   pass         1 — LIVE; cleared by the first failed read, never re-armed
 *   afScl 4 · afSda 9 · moderScl 2 · moderSda 2 · idleScl 1 · idleSda 1
 *     (afSda must be 9, not 4 — DECISIONS.md #16)
 *     (idleScl AND idleSda BOTH 0 = no pull-ups at all = dead 3V3 sensor
 *      rail, not a bus fault. Every I2C2 pull-up is on the MPU6050 breakout.)
 *   scanCount 1 · scanAddr[0] 104 (0x68)
 *   deviceReady 1 · whoAmI 104 · whoAmIOk 1
 *   pwrMgmtReadback 0 · gyroConfigReadback 8 · dlpfReadback 3 · regWriteOk 1
 *   rawZ / rawZMin / rawZMax · dataVarying 1
 *   maxConsecErrors vs readErrors:  ~equal -> wedged, small -> marginal
 *   liveScl / liveSda · busyFlag · halState (0x20 = READY)
 *   recoveries · recoveredOk
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

void Test_I2C_Gyro_Init();     // blocking ~1s
void Test_I2C_Gyro_Update();   // call every main-loop pass


/* ==========================================================================
 * PART 2 — I2C1 + five VL53L0X + XSHUT address assignment
 * ==========================================================================
 *
 * WHY THIS IS STAGED. A single scan taken after the whole XSHUT sequence has
 * run tells you how many sensors ended up on the bus, but not which one
 * failed or where. All five ship at the same factory address 0x29, so a
 * sensor that never booted and a sensor that booted but refused the address
 * write look identical once the sequence is over.
 *
 * So the bus is scanned at EVERY stage:
 *
 *   "all down"    all XSHUT LOW -> expect ZERO devices.
 *                 Anything answering here has an XSHUT line that is not
 *                 actually holding it in standby: wrong pin, broken trace, or
 *                 an XSHUT tied high on the breakout. That sensor will
 *                 collide later and corrupt the whole sequence.
 *
 *   per sensor i: release XSHUT[i] -> it boots at 0x29
 *                 scan -> expect exactly i+1 devices
 *                         (the i already moved, plus this one at 0x29)
 *                 read MODEL_ID 0xC0 -> expect 238 (0xEE): proof a real
 *                         VL53L0X answered and not a stray ACK
 *                 write 0x8A = target -> confirm it answers at the NEW
 *                         address AND that 0x29 has gone quiet
 *
 *   "final"       all five up -> expect 5 devices at 0x30..0x34
 *                                (48, 49, 50, 51, 52 in decimal)
 *
 * The order, addresses and 10ms boot delay deliberately mirror
 * WallSensorArray::init(), so a pass here means the production path works.
 * After addressing, the real tof.c driver is initialised and continuous
 * ranging started, so Update() also gives live per-sensor distances.
 *
 * Live Expressions — add the whole struct: i2cToF
 *
 *   ---- verdicts ----
 *   bringUpPass         1 = all five addressed and initialised
 *   pass                1 = LIVE; every sensor still producing readings
 *
 *   ---- bus / pin level (I2C1 = PB6 SCL, PB7 SDA, both AF4) ----
 *   afScl 4 · afSda 4 · moderScl 2 · moderSda 2 · idleScl 1 · idleSda 1
 *     (both idle lines 0 = dead 3V3 sensor rail — same failure as the gyro)
 *
 *   ---- staged scans ----
 *   countAllDown        expect 0   <- the "before" check
 *   addrAllDown[0..3]   anything that answered while held down (expect none)
 *   scanCountAtStage[i] expect i+1
 *   finalScanCount      expect 5
 *   finalScanAddr[0..4] expect 48 49 50 51 52  (0x30..0x34)
 *
 *   ---- per sensor: 0=Left 1=LeftFront 2=Front 3=RightFront 4=Right ----
 *   xshutHigh[i]     1 — line actually rose when released. Open-drain needs
 *                        the breakout's pull-up; 0 = floating XSHUT
 *   bootedAt29[i]    1 — answered at the factory address
 *   modelId[i]       238 (0xEE) read at 0x29
 *   assignedAddr[i]  48+i — address it answers on after the move
 *   movedOk[i]       1 — responds at the new address
 *   oldAddrGone[i]   1 — 0x29 went quiet. 0 means the address write silently
 *                        failed and this sensor WILL collide with the next
 *   modelIdAfter[i]  238 — the same chip really is answering on the new addr
 *   initOk[i]        1 — VL53L0X_Init() (SPAD + tuning + ref cal) succeeded
 *   rangingOk[i]     1 — continuous mode started
 *   stageOk[i]       1 — everything above passed for this sensor
 *
 *   ---- live ranging ----
 *   rawMm[i] / filtMm[i]  current reading; wave a hand to watch it move
 *   minMm[i] / maxMm[i]   spread seen so far
 *   goodReads[i]          climbs steadily
 *   badStatus[i]          out-of-range / bad status (normal pointing at air)
 *   lastStatus[i]         VL53L0X range status of the most recent read
 *   polls                 total poll passes
 *
 * LED: fast blink while testing, double = bring-up passed, triple = failed.
 */
struct I2CToFReport {
    // bus / pin level
    uint8_t  afScl;
    uint8_t  afSda;
    uint8_t  moderScl;
    uint8_t  moderSda;
    uint8_t  idleScl;
    uint8_t  idleSda;

    // staged scans
    uint8_t  countAllDown;
    uint8_t  addrAllDown[4];
    uint8_t  scanCountAtStage[5];
    uint8_t  finalScanCount;
    uint8_t  finalScanAddr[8];

    // per sensor
    uint8_t  xshutHigh[5];
    uint8_t  bootedAt29[5];
    uint8_t  modelId[5];
    uint8_t  assignedAddr[5];
    uint8_t  movedOk[5];
    uint8_t  oldAddrGone[5];
    uint8_t  modelIdAfter[5];
    uint8_t  initOk[5];
    uint8_t  rangingOk[5];
    uint8_t  stageOk[5];

    // live ranging
    uint16_t rawMm[5];
    uint16_t filtMm[5];
    uint16_t minMm[5];
    uint16_t maxMm[5];
    uint32_t goodReads[5];
    uint32_t badStatus[5];
    uint8_t  lastStatus[5];
    uint32_t polls;

    uint8_t  bringUpPass;
    uint8_t  pass;
};

extern I2CToFReport i2cToF;

void Test_I2C_ToF_Init();      // blocking ~3s (staged scans + 5x ref calibration)
void Test_I2C_ToF_Update();    // call every main-loop pass

#endif // APP_TESTS_I2C_HPP
