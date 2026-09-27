#ifndef APP_TESTS_HPP
#define APP_TESTS_HPP

#include <cstdint>
#include "TestHarness.hpp"
#include "Navigator.hpp"

/*
 * Tests — every test's report struct and entry points.
 * Pick one in Core/Src/main_test.cpp. In the debugger add the ONE global
 * named below to Live Expressions and expand it: that is the whole report.
 *
 * Test_Main() (main_test.cpp) is called from main.cpp USER CODE 2 in the
 * test build: it runs the selected test and never returns.
 *
 * LED in robot-based tests: blinking = gyro calibrating (keep still),
 * off = ready, double flash = PASS, triple flash = FAIL.
 */

void Test_Main();

/* ===========================================================================
 * Bus diagnostics — standalone, run before anything else is initialised
 * =========================================================================== */

// TEST_I2C_GYRO  ->  i2cGyro        I2C2 (PB10 SCL / PB9 SDA) + MPU6050
struct I2cGyroReport {
    th::PinState pins;       bool pinsOk;
    uint8_t  powerSuspect;   // both lines low at idle: 3V3 rail down (battery off?)
    uint8_t  recoveryRun;
    uint8_t  scanCount;      uint8_t scanAddr[8];
    uint8_t  deviceReady;    uint8_t whoAmI;     uint8_t whoAmIOk;
    uint8_t  pwrMgmt, gyroConfig, dlpf;          // read back after write
    uint8_t  configOk;
    uint8_t  bringUpPass;
    // live stress
    int16_t  rawZ, rawZMin, rawZMax;             uint8_t dataVarying;
    uint32_t reads, readErrors, consecErrors, maxConsecErrors, firstErrorMs;
    uint32_t errAf, errTimeout, errBerr, errArlo;
    uint8_t  liveScl, liveSda;                   uint32_t recoveries;
    uint8_t  pass;
};
extern volatile I2cGyroReport i2cGyro;
void Test_I2CGyro_Init();
void Test_I2CGyro_Update();

// TEST_I2C_TOF  ->  i2cToF          I2C1 staged XSHUT bring-up of all five
struct I2cToFReport {
    th::PinState pins;       bool pinsOk;
    uint8_t  scanAllOff;     // must be 0: no sensor escaped standby
    uint8_t  scanAll[8];     uint8_t scanAllCount;
    struct Stage {
        uint8_t seen29;      // appeared at 0x29 when released
        uint8_t moved;       // answered at its new address
        uint8_t modelId;     // 0xEE expected
        uint8_t initOk;      // full VL53L0X_Init
        uint8_t rangeOk;     // one single-shot measurement
        uint16_t rangeMm;
    } stage[5];
    uint8_t  finalScan[8];   uint8_t finalScanCount;
    uint8_t  sensorsOk;
    uint8_t  pass;
};
extern volatile I2cToFReport i2cToF;
void Test_I2CToF_Init();
void Test_I2CToF_Update();

// TEST_TOF_SINGLE  ->  tof1         ONE sensor alone on I2C1, full workup
enum Tof1Fault : uint32_t {
    TF_PINS        = 1u << 0,   TF_NOT_FOUND   = 1u << 1,
    TF_IDENTITY    = 1u << 2,   TF_ADDR_MOVE   = 1u << 3,
    TF_INIT        = 1u << 4,   TF_BUDGET      = 1u << 5,
    TF_SINGLE_SHOT = 1u << 6,   TF_CONTINUOUS  = 1u << 7,
    TF_BUS_ERROR   = 1u << 8,   TF_REBOOTED    = 1u << 9,
    TF_LOST        = 1u << 10,  TF_SILENT      = 1u << 11,
    TF_FROZEN      = 1u << 12,  TF_ZERO_FAR    = 1u << 13,
    TF_SLOW        = 1u << 14,
};
struct Tof1Report {
    // bus
    th::PinState pins;       bool pinsOk;       uint32_t busClockHz;
    // presence and identity
    uint8_t  scanCount;      uint8_t scanAddr[4];  uint8_t foundAddr;
    uint8_t  modelId, revisionId, identityOk;
    // address move 0x29 -> 0x30 -> 0x29
    uint8_t  addrTestOk;
    // bring-up
    uint8_t  initOk, budgetOk, singleShotOk, continuousOk, bringUpOk;
    uint16_t singleShotMm;
    // live
    uint32_t polls, samples, goodReads, badStatus, busErrors;
    uint16_t rawMm, filtMm;
    uint16_t winMin, winMax, winMean, winSpread;   // last 32 good readings
    uint8_t  lastStatus;     uint32_t statusHist[16];
    uint32_t sampleHz;       uint32_t minSampleHz;
    // faults. SILENT = no sample of ANY kind (good or bad) arriving.
    // A zero while the previous reading was close is a genuine close-range
    // reading (zeroNear); a zero out of a far reading is a fault (zeroFar).
    uint32_t zeroNear, zeroFar;  uint16_t lastZeroPrevMm;
    uint32_t silentMs, maxSilentMs;
    uint32_t frozenRun, maxFrozenRun;
    uint32_t addrChecks, reboots, lost;
    uint32_t uptimeMs;
    // verdict: faultMask is sticky history; healthy judges the last 2s
    uint32_t faultMask;
    uint8_t  warmedUp, healthy;
};
extern volatile Tof1Report tof1;
void Test_ToFSingle_Init();
void Test_ToFSingle_Update();

/* ===========================================================================
 * Sensors — robot core running
 * =========================================================================== */

// TEST_TOF_LIVE  ->  tofLive        all five through the full pipeline
struct ToFLiveReport {
    uint8_t  present[5];     uint8_t presentCount;
    float    mm[5];          // calibrated — what the navigator uses
    uint16_t filtMm[5];      // before calibration
    uint16_t rawMm[5];
    uint8_t  ok[5];          uint8_t status[5];
    uint32_t hz[5];          uint32_t busErrors[5];
    // navigator's view, same thresholds as a real run
    uint8_t  wallLeft, wallFront, wallRight;
    // centred between two walls, copy these into NavConfig.h:
    float    sideSum;        // -> WC_SUM_CENTERED_MM
    float    sideDiff;       // -> WC_CENTER_TRIM_MM
    float    sideSumExpected, frontStopMm;   // derived values, for comparison
};
extern volatile ToFLiveReport tofLive;
void Test_ToFLive_Init();
void Test_ToFLive_Update();

// TEST_TOF_CAL  ->  tofCal          per-sensor calibration capture
struct ToFCalReport {
    uint8_t  selected;       // sensor being calibrated (PA5 short = next)
    uint8_t  point;          // next point index 0..5
    float    nextTruthMm;    // put the barrier at THIS distance from the lens
    uint8_t  capturing;      uint8_t samplesTaken;
    uint16_t liveFiltMm;     float liveCalMm;    uint8_t liveOk;
    float    truthMm[6];
    float    avgFiltMm[5][6];
    uint8_t  captured[5];
    // results — send these; they go into ToFConfig.h CAL_A / CAL_B
    float    fitA[5], fitB[5], maxResidualMm[5];
    uint8_t  fitValid[5];
};
extern volatile ToFCalReport tofCal;
void Test_ToFCal_Init();
void Test_ToFCal_Update();

// TEST_GYRO  ->  gyroTest
struct GyroTestReport {
    uint8_t  ok, whoAmI, calibrating, calibrated;
    float    bias, angleDeg, rateDps;
    float    driftDegPerMin;   // since calibration; valid only if kept still
    float    maxAbsRateStill;
    uint32_t readErrors;
    float    spinDeg;          // PA6 short zeroes; turn by hand against a square
};
extern volatile GyroTestReport gyroTest;
void Test_Gyro_Init();
void Test_Gyro_Update();

// TEST_ENCODER  ->  encTest
struct EncTestReport {
    int32_t  ticksL, ticksR;
    float    mmL, mmR, mmAvg, mmDiff;
    float    speedL, speedR;
    float    pushMm;                 // push exactly this far along a ruler
    float    suggestedWheelDiaMm;    // -> Config.h WHEEL_DIAMETER_MM
    float    angleDeg;
    float    suggestedWheelbaseMm;   // spin in place by hand >=180 deg
};
extern volatile EncTestReport encTest;
void Test_Encoder_Init();
void Test_Encoder_Update();

// TEST_BUTTONS  ->  btnTest
struct BtnTestReport {
    uint32_t searchShort, fastShort, searchLong, both;
    uint8_t  searchHeld, fastHeld;
    uint8_t  lastEvent;       // 1 PA6 short, 2 PA5 short, 3 PA6 long, 4 both, 5 PA5 long
    uint32_t fastLong;        // PA5 held >3s (curves ON/OFF in the competition build)
};
extern volatile BtnTestReport btnTest;
void Test_Buttons_Init();
void Test_Buttons_Update();

/* ===========================================================================
 * Motion — robot core running. PA6 short starts; any press stops.
 * =========================================================================== */

// TEST_MOTOR  ->  motorTest         open loop, WHEELS OFF THE GROUND
struct MotorTestReport {
    uint8_t  phase;           // 0 idle, 1 L fwd, 2 L rev, 3 R fwd, 4 R rev,
                              // 5 both fwd, 6 both rev, 7 done
    int32_t  pwmL, pwmR;
    float    speedL, speedR;  // live
    float    lFwd, lRev, rFwd, rRev;      // driven wheel, mm/s
    float    bothFwdL, bothFwdR, bothRevL, bothRevR;
    float    crossTalk;       // worst speed of the wheel NOT driven
    float    balancePct;      // (L-R)/avg, both forward
    float    suggestedSpeedFF; // PWM per mm/s -> ControlConfig.h SPEED_FF
    float    tauMs;            // motor time constant -> ControlConfig.h MOTOR_TAU_S
    uint8_t  dirOk, crossOk, balanceOk, pass;
};
extern volatile MotorTestReport motorTest;
void Test_Motor_Init();
void Test_Motor_Update();

// TEST_HEADING_HOLD  ->  holdTest   hold heading, twist it by hand
// Starts holding automatically once calibrated (LED stops blinking).
// PA6 short toggles it off / on (on = hold the CURRENT heading).
struct HoldTestReport {
    uint8_t active;
    float   angleDeg, errDeg, maxAbsErrDeg, rateDps;
    int32_t pwmL, pwmR;
    float   speedL, speedR;   // wheel response to the PWM
    uint8_t noResponse;       // big PWM, wheels not turning: battery/VMOT/driver
    uint8_t signFault;        // spun AWAY from the target: gyro vs motor direction
                              // mismatch — motors stopped for safety
};
extern volatile HoldTestReport holdTest;
void Test_HeadingHold_Init();
void Test_HeadingHold_Update();

// TEST_TURN  ->  turnTest           profiled pivots incl. the single 180
struct TurnStats {
    uint32_t count, timeouts;
    float    lastErrDeg, maxAbsErrDeg;
    uint32_t lastMs, maxMs;
};
struct TurnTestReport {
    uint8_t   active;  uint8_t step;  uint8_t cycle;
    float     goalDeg, refDeg, angleDeg, rateRefDps, rateDps;
    TurnStats left, right, around;
    uint8_t   pass;
};
extern volatile TurnTestReport turnTest;
void Test_Turn_Init();
void Test_Turn_Update();

// TEST_CURVE  ->  curveTest         one curve on open floor, for tuning its SHAPE
// PA6 short: RIGHT curve, PA5 short: LEFT curve. Axle on a start mark, facing
// along a taped line. The robot drives CURVE_LEAD_IN_MM, curves, drives
// CURVE_LEAD_OUT_MM and stops. Mark the ideal end point on the floor:
//   expectForwardMm ahead of the start mark (along the start line) and
//   expectSideMm to the right (PA6) or left (PA5).
// Compare where the AXLE (wheel centres) ends up:
//   further FORWARD than marked  -> curve too wide  (raise ARC_TURN_FF_GAIN / lead)
//   short of it                  -> curve too tight (lower ARC_TURN_FF_GAIN)
struct CurveTestReport {
    uint8_t  active, dir;               // dir: 1 right, 2 left
    float    expectForwardMm, expectSideMm;
    float    arcMaxLagDeg;              // worst heading lag in the curve (+ = going wide)
    float    arcEndErrDeg;              // heading error as the curve ended (+ = under-turned)
    float    headingErrStopDeg;         // heading error after stopping
    float    peakTurnPwm;               // largest turn PWM in the curve (3000 = capped)
    uint32_t arcMs, runs;
    uint8_t  pass;                      // heading lag < 5 deg and end error < 3 deg
};
extern volatile CurveTestReport curveTest;
void Test_Curve_Init();
void Test_Curve_Update();

// TEST_DRIVE_CELLS  ->  cellTest    PA6 short: N cells forward, PA5 short: back
struct CellTestReport {
    uint8_t  active;
    float    targetMm, travelledMm, remainingMm, stopErrMm;
    float    speedCmd, speedRef, speedMeas, peakSpeed;
    float    headingErrDeg;
    uint32_t durationMs;
    uint32_t runs;
};
extern volatile CellTestReport cellTest;
void Test_DriveCells_Init();
void Test_DriveCells_Update();

// TEST_NAV  ->  navTest             full robot, same as the competition build
struct NavTestReport {
    uint8_t  mode;                 // ModeController::State
    Navigator::Telemetry nav;
    float    headingDeg;
    float    tofMm[5];
};
extern volatile NavTestReport navTest;
void Test_Nav_Init();
void Test_Nav_Update();

#endif // APP_TESTS_HPP
