#include "Tests.hpp"
#include "TestConfig.h"
#include "Config.h"
#include "ControlConfig.h"
#include "NavConfig.h"
#include "MotionProfile.hpp"
#include "EncoderConfig.h"
#include "Robot.hpp"
#include <cmath>

volatile MotorTestReport motorTest;
volatile HoldTestReport  holdTest;
volatile TurnTestReport  turnTest;
volatile CellTestReport  cellTest;
volatile NavTestReport   navTest;

namespace {
    float axleMm() {
        return 0.5f * (robot.encoderL().distanceMm() + robot.encoderR().distanceMm());
    }
    // Fresh heading 0, controller on, standing still.
    void armControl() {
        ControlLoop& c = robot.control();
        c.resetForRun();
        c.enable(true);
        c.setForwardSpeed(0.0f);
    }
}

/* ===========================================================================
 * TEST_MOTOR — open loop, controller off, WHEELS OFF THE GROUND.
 * Checks direction, that each PWM channel drives only its own wheel, and
 * left/right balance at the same PWM.
 * =========================================================================== */

namespace {
    constexpr float MOVING_MMPS = 20.0f;
    uint32_t motorPhaseStart = 0;
    float    accL = 0, accR = 0, crossMax = 0;
    int      accN = 0;

    // Step response for the time constant: time for the LEFT wheel to reach
    // 63.2% of its final speed in the both-forward phase. The left wheel is at
    // rest through phases 3-4 (the right one is still reversing), so its step
    // is clean.
    constexpr int STEP_SAMPLES = 250;                 // 1 per ms, first 250ms
    float stepTrace[STEP_SAMPLES];

    void motorPhasePwm(int phase, int32_t& l, int32_t& r) {
        const int32_t P = testcfg::MOTOR_TEST_PWM;
        static const int8_t tbl[7][2] = { {0,0}, {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {-1,-1} };
        l = tbl[phase][0] * P;
        r = tbl[phase][1] * P;
    }

    void motorEvaluate() {
        volatile MotorTestReport& m = motorTest;
        m.dirOk = m.lFwd > MOVING_MMPS && m.lRev < -MOVING_MMPS &&
                  m.rFwd > MOVING_MMPS && m.rRev < -MOVING_MMPS &&
                  m.bothFwdL > MOVING_MMPS && m.bothFwdR > MOVING_MMPS &&
                  m.bothRevL < -MOVING_MMPS && m.bothRevR < -MOVING_MMPS;
        m.crossTalk = crossMax;
        m.crossOk = crossMax < MOVING_MMPS;
        const float avg = 0.5f * (m.bothFwdL + m.bothFwdR);
        m.balancePct = (avg > 1.0f) ? 100.0f * (m.bothFwdL - m.bothFwdR) / avg : 0.0f;
        m.balanceOk = std::fabs(m.balancePct) < 15.0f;
        const float fwd = 0.5f * (m.bothFwdL + m.bothFwdR);
        const float rev = -0.5f * (m.bothRevL + m.bothRevR);
        const float spd = 0.5f * (fwd + rev);
        m.suggestedSpeedFF = (spd > 1.0f) ? testcfg::MOTOR_TEST_PWM / spd : 0.0f;

        m.tauMs = 0.0f;
        const float target = 0.632f * m.bothFwdL;
        // The encoder speed is a SPEED_WINDOW-tick average: ~1.5ms of its own lag.
        constexpr float WINDOW_LAG_MS = (enccfg::SPEED_WINDOW - 1) * 0.5f;
        for (int i = 1; i < STEP_SAMPLES && m.bothFwdL > MOVING_MMPS; ++i) {
            if (stepTrace[i] >= target) {             // interpolate between samples
                const float f = (target - stepTrace[i - 1]) / (stepTrace[i] - stepTrace[i - 1]);
                m.tauMs = (i - 1) + f - WINDOW_LAG_MS;
                break;
            }
        }
        m.pass = m.dirOk && m.crossOk && m.balanceOk;
        th::verdict(m.pass);
    }
}

void Test_Motor_Init() { th::coreInit(); }

void Test_Motor_Update() {
    th::coreService();
    volatile MotorTestReport& m = motorTest;
    ButtonManager& b = robot.buttons();
    DifferentialDrive& d = robot.drive();
    const uint32_t now = HAL_GetTick();

    m.speedL = robot.encoderL().speedMmPerS();
    m.speedR = robot.encoderR().speedMmPerS();

    if (m.phase == 0 || m.phase == 7) {
        if (b.takeSearchShort()) {
            m.phase = 1; motorPhaseStart = now;
            accL = accR = crossMax = 0; accN = 0;
            robot.led().set(Indicator::OFF);
        }
        return;
    }
    if (b.takeAny()) { d.stop(); m.phase = 0; m.pwmL = m.pwmR = 0; return; }

    int32_t l, r;
    motorPhasePwm(m.phase, l, r);
    d.setWheelPwm(l, r);
    m.pwmL = l; m.pwmR = r;

    const uint32_t t = now - motorPhaseStart;
    if (m.phase == 5 && t < STEP_SAMPLES)
        stepTrace[t] = m.speedL;
    if (t > testcfg::MOTOR_PHASE_MS / 2) {           // measure once spun up
        accL += m.speedL; accR += m.speedR; ++accN;
        const float undriven = (l == 0) ? std::fabs(m.speedL) : (r == 0) ? std::fabs(m.speedR) : 0.0f;
        if (undriven > crossMax) crossMax = undriven;
    }
    if (t < testcfg::MOTOR_PHASE_MS) return;

    const float aL = accN ? accL / accN : 0.0f;
    const float aR = accN ? accR / accN : 0.0f;
    switch (m.phase) {
        case 1: m.lFwd = aL; break;
        case 2: m.lRev = aL; break;
        case 3: m.rFwd = aR; break;
        case 4: m.rRev = aR; break;
        case 5: m.bothFwdL = aL; m.bothFwdR = aR; break;
        case 6: m.bothRevL = aL; m.bothRevR = aR; break;
    }
    accL = accR = 0; accN = 0;
    motorPhaseStart = now;
    m.phase = m.phase + 1;
    if (m.phase == 7) { d.stop(); m.pwmL = m.pwmR = 0; motorEvaluate(); }
}

/* ===========================================================================
 * TEST_HEADING_HOLD — holds heading 0 as soon as calibration finishes.
 * Twist the robot by hand and let go: it must return to 0 without
 * oscillating. PA6 short toggles holding off / on.
 * =========================================================================== */

namespace {
    constexpr int32_t  BIG_PWM          = 1500;
    constexpr float    STILL_MMPS       = 10.0f;
    constexpr uint32_t NO_RESPONSE_MS   = 300;
    constexpr float    RUNAWAY_ERR_DEG  = 180.0f;  // past a half turn: no hand twist
    constexpr float    RUNAWAY_RATE_DPS = 60.0f;   // ...and never keeps spinning
    bool     holdStarted  = false;
    uint32_t bigPwmSince  = 0;

    void holdOn() {
        volatile HoldTestReport& h = holdTest;
        armControl();                       // holds the heading it has right now
        h.active = 1;
        h.maxAbsErrDeg = 0.0f;
        h.noResponse = h.signFault = 0;
        bigPwmSince = 0;
        robot.led().set(Indicator::OFF);
    }
}

void Test_HeadingHold_Init() { th::coreInit(); }

void Test_HeadingHold_Update() {
    if (!th::coreService()) return;
    volatile HoldTestReport& h = holdTest;
    ControlLoop& c = robot.control();
    const uint32_t now = HAL_GetTick();

    if (!holdStarted) { holdStarted = true; holdOn(); }      // no button needed

    if (robot.buttons().takeSearchShort()) {
        if (h.active) { c.enable(false); h.active = 0; }
        else          { holdOn(); }
    }

    h.angleDeg = c.headingDeg();
    h.errDeg   = c.headingErr();
    h.rateDps  = c.pvW();
    h.pwmL     = robot.drive().lastLeftPwm();
    h.pwmR     = robot.drive().lastRightPwm();
    h.speedL   = robot.encoderL().speedMmPerS();
    h.speedR   = robot.encoderR().speedMmPerS();
    if (!h.active) return;

    if (std::fabs(h.errDeg) > h.maxAbsErrDeg) h.maxAbsErrDeg = std::fabs(h.errDeg);

    // Driving hard but the wheels don't move: no motor power reaching them.
    const bool big   = h.pwmL > BIG_PWM || h.pwmL < -BIG_PWM || h.pwmR > BIG_PWM || h.pwmR < -BIG_PWM;
    const bool still = std::fabs(h.speedL) < STILL_MMPS && std::fabs(h.speedR) < STILL_MMPS;
    if (big && still) {
        if (bigPwmSince == 0) bigPwmSince = now;
        else if (now - bigPwmSince > NO_RESPONSE_MS) h.noResponse = 1;
    } else {
        bigPwmSince = 0;
    }

    // Moving further from the target, fast, on its own: the correction pushes
    // the wrong way. Stop before it spins in circles.
    const bool movingAway = (h.errDeg * h.rateDps) < 0.0f;
    if (std::fabs(h.errDeg) > RUNAWAY_ERR_DEG && movingAway &&
        std::fabs(h.rateDps) > RUNAWAY_RATE_DPS) {
        c.enable(false);
        h.active = 0;
        h.signFault = 1;
        th::verdict(false);
    }
}

/* ===========================================================================
 * TEST_TURN — PA6 short starts TURN_CYCLES of: L90, R90, R90, L90, 180, 180.
 * The 180s are the single continuous pivot the navigator uses at dead ends.
 * =========================================================================== */

namespace {
    constexpr int   STEPS = 6;
    constexpr float DELTA[STEPS] = { 90.0f, -90.0f, -90.0f, 90.0f, 180.0f, 180.0f };
    bool     turning = false;
    uint32_t stepStartMs = 0, pauseStartMs = 0;

    volatile TurnStats& statsFor(int step) {
        const float d = DELTA[step];
        return (d > 170.0f) ? turnTest.around : (d > 0.0f) ? turnTest.left : turnTest.right;
    }

    void startStep() {
        robot.control().turnBy(DELTA[turnTest.step]);
        turning = true;
        stepStartMs = HAL_GetTick();
    }

    void turnEvaluate() {
        volatile TurnTestReport& t = turnTest;
        const float tol = ctrlcfg::TURN_TOL_DEG;
        t.pass = t.left.timeouts == 0 && t.right.timeouts == 0 && t.around.timeouts == 0 &&
                 t.left.maxAbsErrDeg < tol && t.right.maxAbsErrDeg < tol &&
                 t.around.maxAbsErrDeg < tol;
        th::verdict(t.pass);
    }
}

void Test_Turn_Init() { th::coreInit(); }

void Test_Turn_Update() {
    if (!th::coreService()) return;
    volatile TurnTestReport& t = turnTest;
    ControlLoop& c = robot.control();
    ButtonManager& b = robot.buttons();
    const uint32_t now = HAL_GetTick();

    t.goalDeg = c.headingGoal(); t.refDeg = c.headingRef();
    t.angleDeg = c.headingDeg(); t.rateRefDps = c.rateRef(); t.rateDps = c.pvW();

    if (!t.active) {
        if (b.takeSearchShort()) {
            t.active = 1; t.step = 0; t.cycle = 0;
            robot.led().set(Indicator::OFF);
            armControl();
            startStep();
        }
        return;
    }
    if (b.takeAny()) { c.enable(false); t.active = 0; turning = false; return; }

    if (turning) {
        if (!c.turnDone()) return;
        const uint32_t ms = now - stepStartMs;
        const float err = c.headingGoal() - c.headingDeg();
        volatile TurnStats& s = statsFor(t.step);
        s.count = s.count + 1;
        s.lastErrDeg = err;
        s.lastMs = ms;
        if (std::fabs(err) > s.maxAbsErrDeg) s.maxAbsErrDeg = std::fabs(err);
        if (ms > s.maxMs) s.maxMs = ms;
        if (ms >= ctrlcfg::TURN_TIMEOUT_MS) s.timeouts = s.timeouts + 1;
        turning = false;
        pauseStartMs = now;
        return;
    }

    if (now - pauseStartMs < testcfg::TURN_PAUSE_MS) return;
    t.step = t.step + 1;
    if (t.step == STEPS) {
        t.step = 0;
        t.cycle = t.cycle + 1;
        if (t.cycle >= testcfg::TURN_CYCLES) {
            c.enable(false);
            t.active = 0;
            turnEvaluate();
            return;
        }
    }
    startStep();
}

/* ===========================================================================
 * TEST_DRIVE_CELLS — PA6 short: DRIVE_CELLS forward. PA5 short: same back.
 * Uses the navigator's braking curve. Measure the real distance with a tape
 * and compare with travelledMm; correct CELL_TRAVEL_MM or WHEEL_DIAMETER_MM.
 * =========================================================================== */

namespace {
    float    cellDir = 1.0f, cellStartMm = 0.0f;
    uint32_t cellStartMs = 0, cellSettleMs = 0;
    bool     cellSettling = false;

    void cellStart(float dir) {
        volatile CellTestReport& r = cellTest;
        cellDir = dir;
        armControl();
        cellStartMm = axleMm();
        cellStartMs = HAL_GetTick();
        cellSettling = false;
        r.active = 1; r.peakSpeed = 0.0f; r.runs = r.runs + 1;
        robot.led().set(Indicator::OFF);
    }
}

void Test_DriveCells_Init() {
    th::coreInit();
    cellTest.targetMm = testcfg::DRIVE_CELLS * cfg::CELL_TRAVEL_MM;
}

void Test_DriveCells_Update() {
    if (!th::coreService()) return;
    volatile CellTestReport& r = cellTest;
    ControlLoop& c = robot.control();
    ButtonManager& b = robot.buttons();
    const uint32_t now = HAL_GetTick();

    r.speedRef = c.speedRef();
    r.speedMeas = c.pvX();
    r.headingErrDeg = c.headingErr();

    if (!r.active) {
        if (b.takeSearchShort()) cellStart(+1.0f);
        else if (b.takeFastShort()) cellStart(-1.0f);
        return;
    }
    if (b.takeAny()) { c.enable(false); r.active = 0; return; }

    r.travelledMm = cellDir * (axleMm() - cellStartMm);
    r.remainingMm = r.targetMm - r.travelledMm;
    if (std::fabs(r.speedMeas) > r.peakSpeed) r.peakSpeed = std::fabs(r.speedMeas);

    if (!cellSettling) {
        const float v = approachSpeed(r.remainingMm, testcfg::DRIVE_SPEED_MMPS,
                                      navcfg::BRAKE_DECEL_MMPS2,
                                      navcfg::CREEP_MMPS, navcfg::CREEP_ZONE_MM,
                                      navcfg::STOP_TOL_MM);
        r.speedCmd = cellDir * v;
        c.setForwardSpeed(r.speedCmd);
        if (v == 0.0f) { cellSettling = true; cellSettleMs = now; }
        return;
    }

    const bool still = std::fabs(c.pvX()) < navcfg::SETTLE_MMPS && c.speedRef() == 0.0f;
    if (still || now - cellSettleMs > navcfg::SETTLE_TIMEOUT_MS) {
        r.stopErrMm  = r.travelledMm - r.targetMm;
        r.durationMs = now - cellStartMs;
        c.enable(false);
        r.active = 0;
        th::verdict(std::fabs(r.stopErrMm) < 5.0f);
    }
}

/* ===========================================================================
 * TEST_NAV — the competition firmware with a report attached.
 * =========================================================================== */

void Test_Nav_Init() { robot.init(); }

void Test_Nav_Update() {
    robot.onMainLoop();
    volatile NavTestReport& r = navTest;
    r.mode = robot.modes().state();
    th::vset(r.nav, robot.navigator().tel);
    r.headingDeg = robot.gyro().angleDeg();
    for (int i = 0; i < cfg::TOF_COUNT; ++i) r.tofMm[i] = robot.walls().distanceMm(i);
}
