#include "Navigator.hpp"
#include "Config.h"
#include "MazeConfig.h"
#include <cmath>

namespace {
    constexpr float SEARCH_SPEED_MMPS  = 1000.0f;
    constexpr float SPEED_SPEED_MMPS   = 1200.0f;
    constexpr float TURN_TOLERANCE_DEG = 5.0f;
    constexpr float TURN_SETTLE_DPS    = 25.0f;
    constexpr uint32_t TURN_TIMEOUT_MS = 3000;

    // Wall-centering gating — thresholds fitted from live ToF logs.
    constexpr float WC_CORRIDOR_WIDTH_MM = 98.0f;   // LEFT+RIGHT when centred (~98)
    constexpr float WC_WIDTH_TOL_MM      = 15.0f;   // reject if the sum strays this far
    constexpr float WC_VALID_MAX_MM      = 120.0f;  // a side reading above this isn't a near wall
    constexpr float WC_DIAG_MAX_MM       = 150.0f;  // diagonal below this => corridor continues
    constexpr float WC_MIN_RUN_MM        = 40.0f;   // walls must persist this far before trusting
    constexpr float WC_ENTRY_MM          = 25.0f;   // skip the cell-entry boundary
    constexpr float WC_EXIT_MM           = 45.0f;   // skip the cell-exit boundary
    constexpr float WC_DEADBAND_MM       = 2.0f;    // ignore sub-noise offsets

    constexpr float FRONT_STOP_MM    = 25.0f;   // MEASURE: TOF_FRONT when correctly
                                                // stopped, centred, wall ahead
    constexpr float FRONT_REF_MAX_MM = 120.0f;  // only trust the front stop within this
    constexpr float FRONT_SLOW_MM    = 120.0f;  // begin easing off here
    constexpr float FRONT_MIN_MMPS   = 90.0f;   // creep speed for the final approach
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
    const float traveled = avgDistanceMm() - cellStartDistance_;

    // Front-wall referencing: absolute, so a slipped cell can't drift the stop.
    // Require >0.6 cell travelled so a wall seen at cell entry can't trigger it.
    if (walls_->ok(cfg::TOF_FRONT) && traveled > maze::CELL_MM * 0.6f) {
        float fd = walls_->distanceMm(cfg::TOF_FRONT);
        if (fd < FRONT_REF_MAX_MM) return fd <= FRONT_STOP_MM;
    }
    return traveled >= maze::CELL_MM;            // open ahead: dead reckon
}

void Navigator::update() {
    if (state_ == IDLE || state_ == DONE) return;

    updateWallCentering();

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

void Navigator::updateWallCentering() {
    const float posInCell = avgDistanceMm() - cellStartDistance_;

    // 1) Only mid-cell while driving, clear of the entry/exit boundaries.
    const bool inWindow = (phase_ == DRIVING) &&
                          (posInCell > WC_ENTRY_MM) &&
                          (posInCell < maze::CELL_MM - WC_EXIT_MM);

    // 2) Both side walls actually beside us now.
    const float ld = walls_->distanceMm(cfg::TOF_LEFT);
    const float rd = walls_->distanceMm(cfg::TOF_RIGHT);
    const bool sidesSeen = walls_->ok(cfg::TOF_LEFT)  && ld < WC_VALID_MAX_MM &&
                           walls_->ok(cfg::TOF_RIGHT) && rd < WC_VALID_MAX_MM;

    // 3) Width invariant: two real parallel walls sum to a fixed gap, regardless
    //    of how off-centre we are. A stub or an angled opening breaks it.
    const bool widthOk =
        std::fabs((ld + rd) - WC_CORRIDOR_WIDTH_MM) < WC_WIDTH_TOL_MM;

    // 4) Look-ahead: the angled sensors see into the next cell. Both must still
    //    see wall for the corridor to continue ("walls in the next cell too").
    const bool aheadContinues =
        walls_->ok(cfg::TOF_LEFTFRONT)  &&
        walls_->distanceMm(cfg::TOF_LEFTFRONT)  < WC_DIAG_MAX_MM &&
        walls_->ok(cfg::TOF_RIGHTFRONT) &&
        walls_->distanceMm(cfg::TOF_RIGHTFRONT) < WC_DIAG_MAX_MM;

    const bool qualified = inWindow && sidesSeen && widthOk && aheadContinues;

    if (!qualified) {
        wcRunStartDist_ = -1.0f;                 // reset dwell
        wallErrDbg_ = 0.0f; wallValidDbg_ = false;
        ctrl_->setWallError(0.0f, false);
        return;
    }

    // 5) Dwell: hold everything above for WC_MIN_RUN_MM of travel first.
    if (wcRunStartDist_ < 0.0f) wcRunStartDist_ = avgDistanceMm();
    if ((avgDistanceMm() - wcRunStartDist_) < WC_MIN_RUN_MM) {
        wallValidDbg_ = false;
        ctrl_->setWallError(0.0f, false);        // still qualifying — don't steer yet
        return;
    }

    // 6) +ve error = nearer the right wall => steer left to centre.
    float err = ld - rd;
    if (std::fabs(err) < WC_DEADBAND_MM) err = 0.0f;

    wallErrDbg_ = err; wallValidDbg_ = true;
    ctrl_->setWallError(err, true);
}

void Navigator::approachSlowdown() {
    if (!walls_->ok(cfg::TOF_FRONT)) return;
    float fd = walls_->distanceMm(cfg::TOF_FRONT);
    if (fd > FRONT_SLOW_MM) return;             // far: full speed
    float k = (fd - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM);
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    float v = FRONT_MIN_MMPS + k * (searchSpeed_ - FRONT_MIN_MMPS);
    ctrl_->driveStraight(v, headingRef_);       // re-issue at the tapered speed
}
