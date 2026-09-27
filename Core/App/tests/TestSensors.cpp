#include "Tests.hpp"
#include "TestConfig.h"
#include "Config.h"
#include "NavConfig.h"
#include "Robot.hpp"
#include <cmath>

volatile ToFLiveReport  tofLive;
volatile ToFCalReport   tofCal;
volatile GyroTestReport gyroTest;
volatile EncTestReport  encTest;
volatile BtnTestReport  btnTest;

/* ===========================================================================
 * TEST_TOF_LIVE
 * Hold the robot centred between two walls to read sideSum / sideDiff.
 * =========================================================================== */

namespace { uint32_t liveSamples[5]; th::Every liveSecond{1000}; }

void Test_ToFLive_Init() {
    th::coreInit();
    tofLive.sideSumExpected = navcfg::WC_SUM_CENTERED_MM;
    tofLive.frontStopMm     = navcfg::FRONT_STOP_MM;
}

void Test_ToFLive_Update() {
    th::coreService();
    volatile ToFLiveReport& r = tofLive;
    const WallSensorArray& w = robot.walls();
    const bool second = liveSecond.due();

    for (int i = 0; i < cfg::TOF_COUNT; ++i) {
        r.present[i]   = w.present(i);
        r.mm[i]        = w.distanceMm(i);
        r.filtMm[i]    = w.filteredMm(i);
        r.rawMm[i]     = w.rawMm(i);
        r.ok[i]        = w.ok(i);
        r.status[i]    = w.status(i);
        r.busErrors[i] = w.busErrors(i);
        if (second) { r.hz[i] = w.samples(i) - liveSamples[i]; liveSamples[i] = w.samples(i); }
    }
    r.presentCount = static_cast<uint8_t>(w.presentCount());

    r.wallLeft  = w.seen(cfg::TOF_LEFT,  navcfg::SIDE_WALL_PRESENT_MM);
    r.wallFront = w.seen(cfg::TOF_FRONT, navcfg::FRONT_WALL_PRESENT_MM);
    r.wallRight = w.seen(cfg::TOF_RIGHT, navcfg::SIDE_WALL_PRESENT_MM);

    if (w.ok(cfg::TOF_LEFT) && w.ok(cfg::TOF_RIGHT)) {
        r.sideSum  = w.distanceMm(cfg::TOF_LEFT) + w.distanceMm(cfg::TOF_RIGHT);
        r.sideDiff = w.distanceMm(cfg::TOF_LEFT) - w.distanceMm(cfg::TOF_RIGHT);
    }
}

/* ===========================================================================
 * TEST_TOF_CAL
 *
 *   PA5 short   select the next sensor (0 L, 1 LF, 2 F, 3 RF, 4 R)
 *   PA6 short   capture the point shown in nextTruthMm (barrier that far
 *               from the sensor's lens face, square to its beam)
 *   PA6 long    clear the selected sensor's points
 *
 * After six points the sensor's fitA / fitB appear. Calibration applies to
 * the FILTERED reading, so the filtered value is what is averaged.
 * =========================================================================== */

namespace {
    float    calSum     = 0.0f;
    uint32_t calLastSeq = 0;

    void fit(int s) {
        volatile ToFCalReport& r = tofCal;
        const int n = testcfg::CAL_POINTS;
        float sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int k = 0; k < n; ++k) {
            const float x = r.avgFiltMm[s][k], y = testcfg::CAL_POINTS_MM[k];
            sx += x; sy += y; sxx += x * x; sxy += x * y;
        }
        const float den = n * sxx - sx * sx;
        if (std::fabs(den) < 1e-3f) { r.fitValid[s] = 0; return; }
        const float a = (n * sxy - sx * sy) / den;
        const float b = (sy - a * sx) / n;
        float worst = 0.0f;
        for (int k = 0; k < n; ++k) {
            const float e = std::fabs(a * r.avgFiltMm[s][k] + b - testcfg::CAL_POINTS_MM[k]);
            if (e > worst) worst = e;
        }
        r.fitA[s] = a; r.fitB[s] = b; r.maxResidualMm[s] = worst;
        r.fitValid[s] = 1;
    }

    void showNext() {
        volatile ToFCalReport& r = tofCal;
        r.point = r.captured[r.selected];
        r.nextTruthMm = (r.point < testcfg::CAL_POINTS) ? testcfg::CAL_POINTS_MM[r.point] : 0.0f;
    }
}

void Test_ToFCal_Init() {
    th::coreInit();
    for (int k = 0; k < testcfg::CAL_POINTS; ++k) tofCal.truthMm[k] = testcfg::CAL_POINTS_MM[k];
    showNext();
}

void Test_ToFCal_Update() {
    th::coreService();
    volatile ToFCalReport& r = tofCal;
    ButtonManager& b = robot.buttons();
    const WallSensorArray& w = robot.walls();
    const int s = r.selected;

    r.liveFiltMm = w.filteredMm(s);
    r.liveCalMm  = w.distanceMm(s);
    r.liveOk     = w.ok(s);

    if (b.takeSearchLong()) {                       // clear this sensor
        r.capturing = 0;
        r.captured[s] = 0;
        r.fitValid[s] = 0;
        showNext();
        robot.led().set(Indicator::OFF);
    }
    if (!r.capturing && b.takeFastShort()) {        // next sensor
        r.selected = static_cast<uint8_t>((s + 1) % cfg::TOF_COUNT);
        showNext();
        robot.led().set(Indicator::OFF);
        return;
    }
    if (!r.capturing && b.takeSearchShort() && r.point < testcfg::CAL_POINTS) {
        r.capturing = 1;
        r.samplesTaken = 0;
        calSum = 0.0f;
        calLastSeq = w.samples(s);
    }

    if (r.capturing && w.samples(s) != calLastSeq) {   // one fresh sample
        calLastSeq = w.samples(s);
        if (w.ok(s)) {
            calSum += w.filteredMm(s);
            r.samplesTaken = r.samplesTaken + 1;
        }
        if (r.samplesTaken >= testcfg::CAL_SAMPLES) {
            r.avgFiltMm[s][r.point] = calSum / r.samplesTaken;
            r.captured[s] = r.captured[s] + 1;
            r.capturing = 0;
            robot.led().pulse();
            if (r.captured[s] == testcfg::CAL_POINTS) {
                fit(s);
                th::verdict(r.fitValid[s] && r.maxResidualMm[s] < 5.0f);
            }
            showNext();
        }
    }
}

/* ===========================================================================
 * TEST_GYRO
 *   Keep still after calibration: driftDegPerMin.
 *   PA6 short: zero spinDeg, then turn by hand exactly 360 against a square
 *   edge — spinDeg should read +360 (anticlockwise) or -360.
 *   PA6 long: recalibrate.
 * =========================================================================== */

namespace { uint32_t gyroCalMs = 0; float gyroCalAngle = 0.0f, spinZero = 0.0f; }

void Test_Gyro_Init() { th::coreInit(); }

void Test_Gyro_Update() {
    const bool ready = th::coreService();
    volatile GyroTestReport& r = gyroTest;
    Gyro& g = robot.gyro();
    ButtonManager& b = robot.buttons();

    r.ok = g.ok(); r.whoAmI = g.whoAmI();
    r.calibrating = g.calibrating(); r.calibrated = g.calibrated();
    r.bias = g.bias(); r.angleDeg = g.angleDeg(); r.rateDps = g.rateDps();
    r.readErrors = g.readErrors();

    if (b.takeSearchLong()) { g.beginCalibration(); gyroCalMs = 0; r.maxAbsRateStill = 0.0f; }
    if (!ready) return;

    const uint32_t now = HAL_GetTick();
    if (gyroCalMs == 0) { gyroCalMs = now; gyroCalAngle = g.angleDeg(); spinZero = gyroCalAngle; }

    const float minutes = (now - gyroCalMs) / 60000.0f;
    if (minutes > 0.05f) r.driftDegPerMin = (g.angleDeg() - gyroCalAngle) / minutes;
    if (std::fabs(g.rateDps()) > r.maxAbsRateStill) r.maxAbsRateStill = std::fabs(g.rateDps());

    if (b.takeSearchShort()) spinZero = g.angleDeg();
    r.spinDeg = g.angleDeg() - spinZero;
}

/* ===========================================================================
 * TEST_ENCODER
 *   PA6 short zeroes. Push straight along a ruler exactly pushMm:
 *   suggestedWheelDiaMm. Zero again, spin in place by hand >= 180 deg:
 *   suggestedWheelbaseMm (needs the gyro calibrated).
 * =========================================================================== */

namespace { int32_t baseL = 0, baseR = 0; float baseMmL = 0, baseMmR = 0, baseAngle = 0; }

void Test_Encoder_Init() {
    th::coreInit();
    encTest.pushMm = testcfg::ENC_PUSH_MM;
}

void Test_Encoder_Update() {
    th::coreService();
    volatile EncTestReport& r = encTest;
    Encoder& L = robot.encoderL();
    Encoder& R = robot.encoderR();

    if (robot.buttons().takeSearchShort()) {
        baseL = L.count(); baseR = R.count();
        baseMmL = L.distanceMm(); baseMmR = R.distanceMm();
        baseAngle = robot.gyro().angleDeg();
    }

    r.ticksL = L.count() - baseL;
    r.ticksR = R.count() - baseR;
    r.mmL = L.distanceMm() - baseMmL;
    r.mmR = R.distanceMm() - baseMmR;
    r.mmAvg = 0.5f * (r.mmL + r.mmR);
    r.mmDiff = r.mmL - r.mmR;
    r.speedL = L.speedMmPerS();
    r.speedR = R.speedMmPerS();

    const float avg = std::fabs(r.mmAvg);
    r.suggestedWheelDiaMm = (avg > 50.0f) ? cfg::WHEEL_DIAMETER_MM * testcfg::ENC_PUSH_MM / avg : 0.0f;

    r.angleDeg = robot.gyro().angleDeg() - baseAngle;
    r.suggestedWheelbaseMm = (std::fabs(r.angleDeg) > 180.0f)
        ? (r.mmR - r.mmL) / (r.angleDeg * cfg::PI_F / 180.0f) : 0.0f;
}

/* ===========================================================================
 * TEST_BUTTONS
 * =========================================================================== */

void Test_Buttons_Init() { th::coreInit(); }

void Test_Buttons_Update() {
    th::coreService();
    volatile BtnTestReport& r = btnTest;
    ButtonManager& b = robot.buttons();
    if (b.takeSearchShort()) { ++r.searchShort; r.lastEvent = 1; robot.led().pulse(); }
    if (b.takeFastShort())   { ++r.fastShort;   r.lastEvent = 2; robot.led().pulse(); }
    if (b.takeSearchLong())  { ++r.searchLong;  r.lastEvent = 3; robot.led().pulse(); }
    if (b.takeBothPress())   { ++r.both;        r.lastEvent = 4; robot.led().pulse(); }
    if (b.takeFastLong())    { ++r.fastLong;    r.lastEvent = 5; robot.led().pulse(); }
    r.searchHeld = b.searchHeld();
    r.fastHeld   = b.fastHeld();
}
