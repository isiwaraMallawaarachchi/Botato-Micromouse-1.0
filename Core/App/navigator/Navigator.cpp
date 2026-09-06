#include "Navigator.hpp"
#include "Config.h"
#include "MazeConfig.h"
#include <cmath>

namespace {
    constexpr float SEARCH_SPEED_MMPS  = 250.0f;
    constexpr float SPEED_SPEED_MMPS   = 300.0f;
    constexpr float TURN_TOLERANCE_DEG = 5.0f;
    constexpr float TURN_SETTLE_DPS    = 25.0f;
    constexpr uint32_t TURN_TIMEOUT_MS = 3000;   // give up if a turn hangs
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
    headingRef_ = 0.0f;          // gyro was just zeroed; NORTH == 0
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
    pendingDir_  = target;
    return true;
}

// Turn target is RELATIVE to the current heading reference, so the
// continuous (unbounded) gyro angle never causes a mismatch.
void Navigator::beginTurn(Turn t) {
    float delta = 0.0f;
    switch (t) {
        case TURN_LEFT:   delta =  90.0f; break;   // anticlockwise-positive
        case TURN_RIGHT:  delta = -90.0f; break;
        case TURN_AROUND: delta = 180.0f; break;
        default:          delta =   0.0f; break;
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

    // Timeout guard: never spin forever if the turn can't settle.
    bool timedOut = (HAL_GetTick() - turnStartMs_) > TURN_TIMEOUT_MS;
    return settled || timedOut;
}

void Navigator::beginDrive() {
    cellStartDistance_ = avgDistanceMm();
    ctrl_->driveStraight(searchSpeed_, headingRef_);
    phase_ = DRIVING;
}

bool Navigator::driveComplete() {
    return (avgDistanceMm() - cellStartDistance_) >= maze::CELL_MM;
}

void Navigator::update() {
    if (state_ == IDLE || state_ == DONE) return;

    switch (phase_) {
    case SENSE:
        senseWalls();
        phase_ = DECIDE;
        break;

    case DECIDE:
        if (!decideNextMove()) { goalOrReturnTransition(); break; }
        if (pendingTurn_ == TURN_NONE) beginDrive();
        else                          beginTurn(pendingTurn_);
        break;

    case TURNING:
        if (turnComplete()) {
            Planner::applyTurn(pose_, pendingTurn_);
            beginDrive();
        }
        break;

    case DRIVING:
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
