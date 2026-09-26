#include "Navigator.hpp"
#include "Config.h"
#include "MazeConfig.h"
#include "NavConfig.h"
#include "MotionProfile.hpp"
#include <cmath>

/* ---- lifecycle ------------------------------------------------------------ */

void Navigator::init(ControlLoop* ctrl, WallSensorArray* walls,
                     Encoder* encL, Encoder* encR) {
    ctrl_ = ctrl; walls_ = walls; encL_ = encL; encR_ = encR;
    clearMap();
    state_ = IDLE;
    phase_ = FINISHED;
}

void Navigator::clearMap() {
    map_.reset();
    mapReady_ = false;
    otherSide_ = false;
    startX_    = maze::START_X;
    tel.otherSide = 0;
}

float Navigator::axleMm() const {
    return 0.5f * (encL_->distanceMm() + encR_->distanceMm());
}

void Navigator::begin(State s, float cruise) {
    state_  = s;
    cruise_ = cruise;
    pose_   = GridPose{ startX_, maze::START_Y, NORTH };

    ctrl_->resetForRun();          // heading 0 == NORTH
    ctrl_->enable(true);
    runStartMm_ = axleMm();

    // First target is the start cell's own centre: the robot is parked with
    // its tail on the back wall, START_OFFSET_MM short of it. From here on
    // the start cell is handled exactly like every other cell.
    segmentEndMm_ = navcfg::START_OFFSET_MM;
    decided_ = braking_ = homing_ = false;
    returnKnown_ = false;
    cacheKey_ = -1;
    pidWall_.reset();
    phase_ = DRIVE;
}

void Navigator::startSearch() {
    clearMap();                    // search always starts from the assumption
    begin(SEARCH, cfg::SEARCH_SPEED_MMPS);
}

bool Navigator::startSpeed() {
    if (!mapReady_) return false;
    flood_.toGoal(map_, true);     // straight-ahead lookahead needs it from tick one
    begin(SPEED, cfg::SPEED_RUN_MMPS);
    return true;
}

void Navigator::abort() {
    ctrl_->enable(false);
    state_ = IDLE;
    phase_ = FINISHED;
    homing_ = false;
}

void Navigator::finish(State s) {
    ctrl_->setForwardSpeed(0.0f);
    ctrl_->setHeadingTrim(0.0f);
    ctrl_->enable(false);
    state_  = s;
    phase_  = FINISHED;
    homing_ = false;
}

void Navigator::update() {
    if (running()) {
        switch (phase_) {
            case DRIVE:  updateDrive();  break;
            case SETTLE: updateSettle(); break;
            case TURN:   updateTurn();   break;
            case PARK:   updatePark();   break;
            default: break;
        }
    }
    tel.state     = state_;
    tel.phase     = phase_;
    tel.x         = static_cast<uint8_t>(pose_.x);
    tel.y         = static_cast<uint8_t>(pose_.y);
    tel.facing    = pose_.facing;
    tel.decisions = decisions_;
}

/* ---- driving ------------------------------------------------------------- */

void Navigator::updateDrive() {
    const float traveled = traveledMm();
    float remaining = segmentEndMm_ - traveled;

    // Stopping in this cell: take the stop point from the front wall if one
    // is in range. Absolute beats relative (DECISIONS.md #25).
    if (braking_) remaining = frontReferenced(traveled, remaining);

    // Decision point: side sensors are now over the target cell's centre.
    if (!decided_ && remaining <= navcfg::SENSE_LOOKAHEAD_MM) {
        if (state_ != SPEED) senseWalls();     // speed run trusts the map
        pending_ = decide();
        ++decisions_;
        tel.lastAction = pending_;

        if (pending_ == ACT_STRAIGHT) {
            segmentEndMm_ += cfg::CELL_TRAVEL_MM;   // exact pitch: no drift
            remaining     += cfg::CELL_TRAVEL_MM;
            Planner::stepForward(pose_);
        } else {
            decided_ = braking_ = true;
        }
    }

    float v;
    if (braking_) {
        v = approachSpeed(remaining, cruise_, navcfg::BRAKE_DECEL_MMPS2,
                          navcfg::CREEP_MMPS, navcfg::CREEP_ZONE_MM,
                          navcfg::STOP_TOL_MM);
        if (v == 0.0f) {
            stopThen(pending_);
            publish(remaining, 0.0f);
            return;
        }
    } else {
        v = cruise_;
        // Speed run: the map says how far the straight goes, so accelerate
        // along it and start braking early enough for the turn at its end.
        if (state_ == SPEED) {
            const float stopDist = remaining + straightAheadMm() - navcfg::CREEP_ZONE_MM;
            const float vBrake = std::sqrt(2.0f * navcfg::BRAKE_DECEL_MMPS2 *
                                           (stopDist > 0.0f ? stopDist : 0.0f));
            if (vBrake < v) v = vBrake;
        }
    }

    ctrl_->setForwardSpeed(v);
    ctrl_->setHeadingTrim(wallTrimDeg(remaining));
    publish(remaining, v);
}

// Stop on the centre, wait for the robot to actually be still, then act.
void Navigator::stopThen(Action a) {
    ctrl_->setForwardSpeed(0.0f);
    ctrl_->setHeadingTrim(0.0f);
    pending_       = a;
    settleStartMs_ = HAL_GetTick();
    phase_         = SETTLE;
}

void Navigator::updateSettle() {
    const bool still   = std::fabs(ctrl_->pvX()) < navcfg::SETTLE_MMPS &&
                         ctrl_->speedRef() == 0.0f;
    const bool expired = (HAL_GetTick() - settleStartMs_) > navcfg::SETTLE_TIMEOUT_MS;
    if (still || expired) act(pending_);
}

float Navigator::frontReferenced(float traveled, float remaining) {
    tel.frontRef = 0;
    if (!walls_->ok(cfg::TOF_FRONT)) return remaining;

    const float fd = walls_->distanceMm(cfg::TOF_FRONT);
    if (fd > navcfg::FRONT_REF_MAX_MM) return remaining;

    // Compensate for sensor + filter lag at the current speed.
    const float r = fd - ctrl_->speedRef() * navcfg::FRONT_LATENCY_S - navcfg::FRONT_STOP_MM;
    segmentEndMm_ = traveled + r;           // re-anchor: erases encoder drift
    tel.frontRef  = 1;
    return r;
}

/* ---- sensing and deciding -------------------------------------------------- */

// True when `sensor` sees a clear opening on `side`, but the map says that
// side is the maze's outer wall. Only a valid, clearly-far reading counts.
bool Navigator::boundaryOpen(Dir side, int sensor) const {
    int nx, ny;
    MazeMap::neighbour(pose_.x, pose_.y, side, nx, ny);
    if (map_.inBounds(nx, ny)) return false;               // not the boundary
    return walls_->ok(sensor) && walls_->distanceMm(sensor) > navcfg::OUTER_OPEN_MM;
}

// The maze is on the other side of the start than assumed. Left and right are
// physical, so nothing flips: only the start column was wrong. Before this is
// detected the robot can only have driven straight along its start column (it
// cannot turn into the real outer wall, and an opening the other way triggers
// detection), so that column is everything learned. Move it across, keeping
// every wall exactly as seen; the current cell is re-sensed right after.
void Navigator::moveStartToOtherSide() {
    const int from = maze::START_X;
    const int to   = maze::WIDTH - 1 - maze::START_X;
    MazeMap old = map_;
    map_.reset();                               // outer walls in the new frame
    for (int y = 0; y < maze::HEIGHT; ++y) {
        if (!old.isVisited(from, y)) continue;
        for (int d = 0; d < 4; ++d)
            if (old.hasWall(from, y, static_cast<Dir>(d))) map_.setWall(to, y, static_cast<Dir>(d));
        map_.markVisited(to, y);
    }
    pose_.x    = to + (pose_.x - from);
    startX_    = to;
    otherSide_ = true;
    cacheKey_  = -1;
    tel.otherSide = 1;
}

void Navigator::senseWalls() {
    // Start-corner check first, so the walls below land in the right frame.
    if (!otherSide_ && state_ == SEARCH) {
        const Dir f = pose_.facing;
        if (boundaryOpen(static_cast<Dir>((f + 3) % 4), cfg::TOF_LEFT) ||
            boundaryOpen(static_cast<Dir>((f + 1) % 4), cfg::TOF_RIGHT))
            moveStartToOtherSide();
    }

    const Dir f     = pose_.facing;
    const Dir left  = static_cast<Dir>((f + 3) % 4);
    const Dir right = static_cast<Dir>((f + 1) % 4);

    const bool wl = walls_->seen(cfg::TOF_LEFT,  navcfg::SIDE_WALL_PRESENT_MM);
    const bool wf = walls_->seen(cfg::TOF_FRONT, navcfg::FRONT_WALL_PRESENT_MM);
    const bool wr = walls_->seen(cfg::TOF_RIGHT, navcfg::SIDE_WALL_PRESENT_MM);

    if (wf) map_.setWall(pose_.x, pose_.y, f);
    if (wl) map_.setWall(pose_.x, pose_.y, left);
    if (wr) map_.setWall(pose_.x, pose_.y, right);
    map_.markVisited(pose_.x, pose_.y);

    tel.wallL = wl; tel.wallF = wf; tel.wallR = wr;
}

Navigator::Action Navigator::decide() {
    if ((state_ == SEARCH || state_ == SPEED) && map_.isGoal(pose_.x, pose_.y))
        return ACT_GOAL;
    if (state_ == RETURN && pose_.x == startX_ && pose_.y == maze::START_Y)
        return ACT_HOME;

    if (state_ == RETURN) flood_.toCell(map_, startX_, maze::START_Y, returnKnown_);
    else                  flood_.toGoal(map_, state_ == SPEED);

    Dir target;
    if (!flood_.nextDir(map_, pose_.x, pose_.y, pose_.facing, target)) return ACT_STUCK;

    switch (Planner::turnFor(pose_.facing, target)) {
        case TURN_LEFT:   return ACT_LEFT;
        case TURN_RIGHT:  return ACT_RIGHT;
        case TURN_AROUND: return ACT_AROUND;
        default:          return ACT_STRAIGHT;
    }
}

void Navigator::act(Action a) {
    switch (a) {
    case ACT_LEFT:     startTurn(TURN_LEFT);   break;
    case ACT_RIGHT:    startTurn(TURN_RIGHT);  break;
    case ACT_AROUND:   startTurn(TURN_AROUND); break;   // single 180 pivot
    case ACT_STRAIGHT: nextCell();             break;

    case ACT_GOAL:
        // Stopped in the goal. Both runs drive home: a lifted robot costs
        // +20s (rule 2.4.6.2). After a speed run the map is complete enough,
        // so go home on explored cells only; after a search keep exploring.
        returnKnown_ = (state_ == SPEED);
        state_   = RETURN;
        cruise_  = cfg::SEARCH_SPEED_MMPS;
        pending_ = decide();
        ++decisions_;
        tel.lastAction = pending_;
        act(pending_);
        break;

    case ACT_HOME:   startHoming();                      break;
    case ACT_PARKED: mapReady_ = true; finish(DONE);     break;
    case ACT_STUCK:  finish(STUCK);                      break;
    }
}

/* ---- turning --------------------------------------------------------------- */

void Navigator::startTurn(Turn t) {
    turn_ = t;
    ctrl_->setForwardSpeed(0.0f);
    ctrl_->setHeadingTrim(0.0f);
    ctrl_->turnBy(Planner::turnDegrees(t));
    phase_ = TURN;
}

void Navigator::updateTurn() {
    if (!ctrl_->turnDone()) return;
    Planner::applyTurn(pose_, turn_);
    if (homing_) {
        parkEndMm_ = traveledMm() - (navcfg::START_OFFSET_MM - navcfg::PARK_GAP_MM);
        phase_ = PARK;
    } else {
        nextCell();
    }
}

// Axle is on the current cell's centre: target the next one.
void Navigator::nextCell() {
    segmentEndMm_ = traveledMm() + cfg::CELL_TRAVEL_MM;
    Planner::stepForward(pose_);
    decided_ = braking_ = false;
    pidWall_.reset();
    phase_ = DRIVE;
}

/* ---- return home and park -------------------------------------------------- */

// On the start cell's centre: face NORTH, then back into the start position.
void Navigator::startHoming() {
    homing_ = true;
    const Turn t = Planner::turnFor(pose_.facing, NORTH);
    if (t == TURN_NONE) {
        parkEndMm_ = traveledMm() - (navcfg::START_OFFSET_MM - navcfg::PARK_GAP_MM);
        phase_ = PARK;
    } else {
        startTurn(t);
    }
}

void Navigator::updatePark() {
    const float remaining = traveledMm() - parkEndMm_;   // still to reverse
    const float v = approachSpeed(remaining, navcfg::PARK_SPEED_MMPS,
                                  navcfg::BRAKE_DECEL_MMPS2,
                                  navcfg::CREEP_MMPS, navcfg::CREEP_ZONE_MM,
                          navcfg::STOP_TOL_MM);
    if (v == 0.0f) { stopThen(ACT_PARKED); return; }
    ctrl_->setForwardSpeed(-v);
    ctrl_->setHeadingTrim(0.0f);
    publish(remaining, -v);
}

/* ---- speed-run lookahead --------------------------------------------------- */

// How far the known path runs straight beyond the target cell. The flood
// is fixed during a speed run, so the walk is cached per target cell.
float Navigator::straightAheadMm() {
    const int key = (pose_.x << 8) | (pose_.y << 2) | pose_.facing;
    if (key != cacheKey_) {
        cacheKey_ = key;
        straightCache_ = 0;
        int x = pose_.x, y = pose_.y;
        for (int i = 0; i < maze::MAX_DIM; ++i) {
            if (map_.isGoal(x, y)) break;
            Dir d;
            if (!flood_.nextDir(map_, x, y, pose_.facing, d) || d != pose_.facing) break;
            MazeMap::neighbour(x, y, d, x, y);
            ++straightCache_;
        }
        tel.straightAhead = straightCache_;
    }
    return straightCache_ * cfg::CELL_TRAVEL_MM;
}

/* ---- lateral centring ------------------------------------------------------ */

// Returns a small heading lean; the heading loop does the rest (#28).
float Navigator::wallTrimDeg(float remaining) {
    tel.wallMode = 0;
    tel.wallErrMm = tel.wallTrimDeg = 0.0f;

    // Side-sensor position along the track relative to the target centre.
    // Corner posts sit on cell boundaries; keep clear of them.
    const float pitch = cfg::CELL_PITCH_MM;
    const float p = cfg::TOF_SIDE_AHEAD_MM - remaining;
    float q = std::fmod(p + 0.5f * pitch, pitch);
    if (q < 0.0f) q += pitch;
    const float fromPost = (q < pitch - q) ? q : pitch - q;

    if (fromPost < navcfg::WC_POST_GATE_MM || ctrl_->speedRef() < 1.0f) {
        pidWall_.reset();
        return 0.0f;
    }

    const float ld = walls_->distanceMm(cfg::TOF_LEFT);
    const float rd = walls_->distanceMm(cfg::TOF_RIGHT);
    const bool  lSeen = walls_->seen(cfg::TOF_LEFT,  navcfg::SIDE_WALL_PRESENT_MM);
    const bool  rSeen = walls_->seen(cfg::TOF_RIGHT, navcfg::SIDE_WALL_PRESENT_MM);

    float err = 0.0f, gain = 0.0f;
    if (lSeen && rSeen) {
        // Two real parallel walls always sum to the same width, however far
        // off-centre we are; a stub or an opening breaks that.
        if (std::fabs((ld + rd) - navcfg::WC_SUM_CENTERED_MM) < navcfg::WC_WIDTH_TOL_MM) {
            err = (ld - rd) - navcfg::WC_CENTER_TRIM_MM;   // + = drifted right
            gain = navcfg::WC_GAIN_BOTH_WALLS;
            tel.wallMode = 1;
        }
    } else if (lSeen) {
        // Hold the distance this side reads when truly centred (includes the
        // measured left/right asymmetry). x2 keeps one PID tuning valid.
        err = 2.0f * (ld - navcfg::WC_LEFT_CENTERED_MM);
        gain = navcfg::WC_GAIN_ONE_WALL;
        tel.wallMode = 2;
    } else if (rSeen) {
        err = 2.0f * (navcfg::WC_RIGHT_CENTERED_MM - rd);
        gain = navcfg::WC_GAIN_ONE_WALL;
        tel.wallMode = 3;
    }

    if (gain == 0.0f) { pidWall_.reset(); tel.wallMode = 0; return 0.0f; }
    if (std::fabs(err) < navcfg::WC_DEADBAND_MM) err = 0.0f;

    const float trim = gain * pidWall_.compute(err);   // + = lean left (CCW)
    tel.wallErrMm = err;
    tel.wallTrimDeg = trim;
    return trim;                                       // clamped in ControlLoop
}

/* ---- telemetry ----------------------------------------------------------- */

void Navigator::publish(float remaining, float v) {
    tel.remainingMm = remaining;
    tel.speedCmd    = v;
}
