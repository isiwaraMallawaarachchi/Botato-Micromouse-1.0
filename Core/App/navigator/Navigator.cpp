#include "Navigator.hpp"
#include "Config.h"
#include "MazeConfig.h"
#include <cmath>

namespace {
    constexpr float SEARCH_SPEED_MMPS  = 1200.0f;   // NOTE: >450 exceeds hardware
    constexpr float SPEED_SPEED_MMPS   = 1200.0f;
    constexpr float TURN_TOLERANCE_DEG = 5.0f;
    constexpr float TURN_SETTLE_DPS    = 25.0f;
    constexpr uint32_t TURN_TIMEOUT_MS = 3000;
    constexpr float TURN_ADVANCE_MM    = 3.0f;      // rear-axle offset after a 90 turn

    // ---- Dead-end K-turn tuning ----
    constexpr float DE_SWING_ANGLE_DEG    = 20.0f;  // angle before reversing
    constexpr float DE_REVERSE_MM         = 50.0f;  // reverse while angled (main clearance knob)
    constexpr float DE_SETTLE_MM          = 30.0f;  // final reverse to seat in the cell
    constexpr float DE_SWING_SPEED_DPS    = 90.0f;  // single-wheel swing rate
    constexpr float DE_REVERSE_SPEED_MMPS = 500.0f;
    constexpr float DE_SWING_TOL_DEG      = 3.0f;

    // ---- Front-wall referencing ----
    constexpr float FRONT_STOP_MM        = 30.0f;   // TOF_FRONT when correctly stopped
    constexpr float FRONT_REF_MAX_MM     = 120.0f;  // only trust the front stop within this
    constexpr float FRONT_ENTRY_GUARD_MM = 40.0f;   // min travel before ToF may end a cell
    constexpr float FRONT_SLOW_MM        = 120.0f;  // begin easing off here
    constexpr float FRONT_MIN_MMPS       = 90.0f;   // creep speed on final approach
}

void Navigator::init(ControlLoop* ctrl, WallSensorArray* walls,
                     Encoder* encL, Encoder* encR) {
    ctrl_ = ctrl; walls_ = walls; encL_ = encL; encR_ = encR;
    map_.reset();
    state_ = IDLE;
}

float Navigator::avgDistanceMm() const {
    return 0.5f * (encL_->distanceMm() + encR_->distanceMm());
}

void Navigator::startSearch() {
    map_.reset();
    pose_ = { maze::START_X, maze::START_Y, NORTH };
    searchSpeed_ = SEARCH_SPEED_MMPS;
    ctrl_->resetControllers();
    ctrl_->enable(true);
    headingRef_ = 0.0f;          // gyro just zeroed; NORTH == 0
    deStep_ = DE_IDLE;
    state_ = SEARCH;
    beginCellSequence();
}

void Navigator::startSpeed() {
    pose_ = { maze::START_X, maze::START_Y, NORTH };
    searchSpeed_ = SPEED_SPEED_MMPS;
    flood_.recompute(map_);
    ctrl_->resetControllers();
    ctrl_->enable(true);
    headingRef_ = 0.0f;
    deStep_ = DE_IDLE;
    state_ = SPEED;
    beginCellSequence();
}

void Navigator::abort() {
    ctrl_->enable(false);
    state_ = IDLE;
}

void Navigator::beginCellSequence() { phase_ = SENSE; }

void Navigator::senseWalls() {
    bool wallLeft  = walls_->wallPresent(cfg::TOF_LEFT);
    bool wallFront = walls_->wallPresent(cfg::TOF_FRONT);
    bool wallRight = walls_->wallPresent(cfg::TOF_RIGHT);

    Dir f = pose_.facing;
    Dir left  = (Dir)(((int)f + 3) % 4);
    Dir right = (Dir)(((int)f + 1) % 4);

    if (wallFront) map_.setWall(pose_.x, pose_.y, f);
    if (wallLeft)  map_.setWall(pose_.x, pose_.y, left);
    if (wallRight) map_.setWall(pose_.x, pose_.y, right);

    map_.markVisited(pose_.x, pose_.y);
}

bool Navigator::decideNextMove() {
    if (state_ == SEARCH && map_.isGoal(pose_.x, pose_.y)) return false;
    if (state_ == SPEED  && map_.isGoal(pose_.x, pose_.y)) return false;
    if (state_ == RETURN && pose_.x == maze::START_X && pose_.y == maze::START_Y)
        return false;

    if (state_ == RETURN) flood_.recomputeTo(map_, maze::START_X, maze::START_Y);
    else                  flood_.recompute(map_);

    Dir target;
    if (!flood_.nextDir(map_, pose_.x, pose_.y, target)) return false;

    pendingTurn_ = Planner::turnFor(pose_.facing, target);
    return true;
}

// Turn target is RELATIVE to the current heading reference, so the
// continuous (unbounded) gyro angle never causes a mismatch.
void Navigator::beginTurn(Turn t) {
    float delta = 0.0f;
    switch (t) {
        case TURN_LEFT:  delta =  90.0f; break;   // anticlockwise-positive
        case TURN_RIGHT: delta = -90.0f; break;
        default:         delta =   0.0f; break;
    }
    headingRef_ += delta;
    turnTargetHeading_ = headingRef_;
    ctrl_->holdHeading(turnTargetHeading_);
    turnStartMs_ = HAL_GetTick();
    phase_ = TURNING;
}

bool Navigator::turnComplete() {
    float err = turnTargetHeading_ - ctrl_->headingDeg();
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    bool settled = (std::fabs(err) < TURN_TOLERANCE_DEG) &&
                   (std::fabs(ctrl_->pvW()) < TURN_SETTLE_DPS);
    bool timedOut = (HAL_GetTick() - turnStartMs_) > TURN_TIMEOUT_MS;
    return settled || timedOut;
}

void Navigator::beginDrive(bool afterTurn) {
    driveFollowsTurn_  = afterTurn;
    cellStartDistance_ = avgDistanceMm();
    ctrl_->driveStraight(searchSpeed_, headingRef_);
    phase_ = DRIVING;
}

bool Navigator::driveComplete() {
    const float traveled = avgDistanceMm() - cellStartDistance_;

    // Front wall in range => absolute reference, always wins over the encoder.
    if (walls_->ok(cfg::TOF_FRONT)) {
        const float fd = walls_->distanceMm(cfg::TOF_FRONT);
        if (fd < FRONT_REF_MAX_MM && traveled > FRONT_ENTRY_GUARD_MM) {
            return fd <= FRONT_STOP_MM;
        }
    }

    float target = maze::CELL_MM;
    if (driveFollowsTurn_) target += TURN_ADVANCE_MM;
    return traveled >= target;
}

void Navigator::approachSlowdown() {
    if (!walls_->ok(cfg::TOF_FRONT)) return;
    float fd = walls_->distanceMm(cfg::TOF_FRONT);
    if (fd > FRONT_SLOW_MM) return;
    float k = (fd - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM);
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    float v = FRONT_MIN_MMPS + k * (searchSpeed_ - FRONT_MIN_MMPS);
    ctrl_->driveStraight(v, headingRef_);
}

// ---- Dead-end K-turn (always LEFT) -----------------------------------------

// Pivot about ONE wheel. The mixer gives v_left = v - w*W/2, v_right = v + w*W/2,
// so v = ±w*W/2 parks one wheel and swings the body about it, producing the
// lateral offset a centred pivot cannot.
void Navigator::beginWheelSwing(bool pivotOnLeftWheel, float deltaDeg) {
    deSwingTargetDeg_ = ctrl_->headingDeg() + deltaDeg;

    const float wDps = (deltaDeg >= 0.0f) ? DE_SWING_SPEED_DPS : -DE_SWING_SPEED_DPS;
    const float wRad = wDps * 3.14159265f / 180.0f;
    const float half = cfg::WHEELBASE_MM * 0.5f;
    const float vMmPerS = pivotOnLeftWheel ? (wRad * half) : (-wRad * half);

    ctrl_->setTargets(vMmPerS, wDps);
}

bool Navigator::swingComplete() const {
    return std::fabs(deSwingTargetDeg_ - ctrl_->headingDeg()) < DE_SWING_TOL_DEG;
}

void Navigator::beginDeadEndReverse(float holdHeadingDeg) {
    deRefDistance_ = avgDistanceMm();
    ctrl_->driveStraight(-DE_REVERSE_SPEED_MMPS, holdHeadingDeg);
}

bool Navigator::deadEndReverseComplete(float distanceMm) const {
    return (avgDistanceMm() - deRefDistance_) <= -distanceMm;
}

void Navigator::beginDeadEndTurn() {
    phase_  = DEAD_END;
    deStep_ = DE_SWING_OUT_A;
    beginWheelSwing(false, DE_SWING_ANGLE_DEG);   // pivot on right wheel, swing +20
}

void Navigator::updateDeadEndTurn() {
    switch (deStep_) {
    case DE_SWING_OUT_A:
        if (swingComplete()) {
            beginDeadEndReverse(headingRef_ + DE_SWING_ANGLE_DEG);
            deStep_ = DE_REVERSE_A;
        }
        break;

    case DE_REVERSE_A:
        if (deadEndReverseComplete(DE_REVERSE_MM)) {
            beginWheelSwing(true, -DE_SWING_ANGLE_DEG);
            deStep_ = DE_STRAIGHTEN_A;
        }
        break;

    case DE_STRAIGHTEN_A:
        if (swingComplete()) {
            headingRef_ += 180.0f;                 // always turn left
            turnTargetHeading_ = headingRef_;
            ctrl_->holdHeading(turnTargetHeading_);
            turnStartMs_ = HAL_GetTick();
            deStep_ = DE_ROTATE_180;
        }
        break;

    case DE_ROTATE_180:
        if (turnComplete()) {
            beginWheelSwing(false, DE_SWING_ANGLE_DEG);
            deStep_ = DE_SWING_OUT_B;
        }
        break;

    case DE_SWING_OUT_B:
        if (swingComplete()) {
            beginDeadEndReverse(headingRef_ + DE_SWING_ANGLE_DEG);
            deStep_ = DE_REVERSE_B;
        }
        break;

    case DE_REVERSE_B:
        if (deadEndReverseComplete(DE_REVERSE_MM)) {
            beginWheelSwing(true, -DE_SWING_ANGLE_DEG);
            deStep_ = DE_STRAIGHTEN_B;
        }
        break;

    case DE_STRAIGHTEN_B:
        if (swingComplete()) {
            beginDeadEndReverse(headingRef_);
            deStep_ = DE_SETTLE;
        }
        break;

    case DE_SETTLE:
        if (deadEndReverseComplete(DE_SETTLE_MM)) deStep_ = DE_FINISHED;
        break;

    default:
        break;
    }
}

// ---- Main state machine ----------------------------------------------------

void Navigator::update() {
    if (state_ == IDLE || state_ == DONE) return;

    switch (phase_) {
    case SENSE:
        senseWalls();
        phase_ = DECIDE;
        break;

    case DECIDE:
        if (!decideNextMove()) { goalOrReturnTransition(); break; }
        if      (pendingTurn_ == TURN_NONE)   beginDrive(false);
        else if (pendingTurn_ == TURN_AROUND) beginDeadEndTurn();
        else {
            ctrl_->setTargets(0.0f, 0.0f);   // stop
            HAL_Delay(2000);                 // TEST ONLY — delete these 2 lines after testing
            beginTurn(pendingTurn_);
        }
        break;

    case TURNING:
        if (turnComplete()) {
            Planner::applyTurn(pose_, pendingTurn_);
            beginDrive(true);
        }
        break;

    case DEAD_END:
        updateDeadEndTurn();
        if (deStep_ == DE_FINISHED) {
            Planner::applyTurn(pose_, TURN_AROUND);
            deStep_ = DE_IDLE;
            beginDrive(false);
        }
        break;

    case DRIVING:
        approachSlowdown();
        if (driveComplete()) {
            Planner::stepForward(pose_);
            phase_ = ARRIVE;
        }
        break;

    case ARRIVE:
        beginCellSequence();
        break;
    }
}

void Navigator::goalOrReturnTransition() {
    if (state_ == SEARCH) {
        state_ = RETURN;
        beginCellSequence();
    } else {
        ctrl_->setTargets(0.0f, 0.0f);
        ctrl_->enable(false);
        state_ = DONE;
    }
}
