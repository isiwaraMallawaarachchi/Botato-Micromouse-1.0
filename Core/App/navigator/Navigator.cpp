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
    arcLenMm_ = arcRadiusMm() * ArcShape::lengthPerRadius(ctrlcfg::ARC_RAMP_FRAC);
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
    // its tail on the back wall, START_OFFSET_MM short of it — or PARK_GAP_MM
    // less if it parked itself there after the previous run. From here on the
    // start cell is handled exactly like every other cell.
    segmentEndMm_ = navcfg::START_OFFSET_MM - (selfParked_ ? navcfg::PARK_GAP_MM : 0.0f);
    selfParked_ = false;
    decided_ = braking_ = homing_ = false;
    arcPending_ = false;
    returnKnown_ = false;
    cacheKey_ = -1;
    sideCellKey_ = -1;
    lastWallMode_ = 0;
    lastTrim_ = 0.0f;
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
            case ARC:    updateArc();    break;
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
    if (arcPending_) remaining = frontArcReferenced(traveled, remaining);
    // Normal-mode speed run (and its drive home): the map knows the target
    // cell's front wall, so take the distance to it from the front ToF as soon
    // as it is in range — braking is then planned from the real distance, not
    // from encoder distance that may have drifted since the last wall.
    else if (knownRun() && !arcsOn() && !braking_) remaining = frontArcReferenced(traveled, remaining);

    // Decision point. Search: side sensors over the target cell's centre.
    // Known-path runs with curves: early enough to start a curve on time.
    const float decideAt = arcsOn() ? arcStartMm() + navcfg::ARC_DECIDE_MARGIN_MM
                                    : navcfg::SENSE_LOOKAHEAD_MM;
    if (!decided_ && remaining <= decideAt) {
        // Speed run and its return trust the map: no sensing, so a glitch can
        // never add a false wall to the map the next speed run relies on.
        if (state_ != SPEED && !returnKnown_) senseWalls();
        pending_ = decide();
        ++decisions_;
        tel.lastAction = pending_;

        if (pending_ == ACT_STRAIGHT) {
            segmentEndMm_ += cfg::CELL_TRAVEL_MM;   // exact pitch: no drift
            remaining     += cfg::CELL_TRAVEL_MM;
            Planner::stepForward(pose_);
        } else if (arcsOn() && (pending_ == ACT_LEFT || pending_ == ACT_RIGHT)) {
            decided_    = true;                     // curve: no stop in this cell
            arcPending_ = true;
            turn_       = (pending_ == ACT_LEFT) ? TURN_LEFT : TURN_RIGHT;
        } else {
            decided_ = braking_ = true;
            stallSinceMs_ = 0;
        }
    }

    // Curve start: half a cell before the turn cell's centre (+ ARC_START_ADVANCE_MM).
    if (arcPending_ && remaining <= arcStartMm()) {
        startArc(traveled);
        publish(remaining, ctrl_->speedCmd());
        return;
    }

    float v;
    if (braking_) {
        v = approachSpeed(remaining, cruise_, navcfg::BRAKE_DECEL_MMPS2,
                          navcfg::CREEP_MMPS, navcfg::CREEP_ZONE_MM,
                          navcfg::STOP_TOL_MM);
        // Stalled in the last few mm: close enough, carry on instead of hanging.
        if (v != 0.0f && remaining < navcfg::STALL_ARRIVE_MM && stalled(remaining)) {
            ++tel.stallStops;
            v = 0.0f;
        }
        if (v == 0.0f) {
            stopThen(pending_);
            publish(remaining, 0.0f);
            return;
        }
    } else {
        v = cruise_;
        // Speed run (and the return after it): the map says how far the
        // straight goes, so accelerate along it and brake early for what ends
        // it — down to CURVE_SPEED for a curve, to a stop for anything else.
        if (knownRun()) {
            const float vc = (cfg::CURVE_SPEED_MMPS < cruise_) ? cfg::CURVE_SPEED_MMPS : cruise_;
            float vBrake;
            if (arcPending_) {
                const float d = remaining - arcStartMm();
                vBrake = std::sqrt(vc * vc + 2.0f * navcfg::BRAKE_DECEL_MMPS2 * (d > 0.0f ? d : 0.0f));
            } else {
                const float ahead = remaining + straightAheadMm();
                if (eventArc_) {
                    const float d = ahead - arcStartMm();
                    vBrake = std::sqrt(vc * vc + 2.0f * navcfg::BRAKE_DECEL_MMPS2 * (d > 0.0f ? d : 0.0f));
                } else {
                    const float d = ahead - navcfg::CREEP_ZONE_MM;
                    vBrake = std::sqrt(2.0f * navcfg::BRAKE_DECEL_MMPS2 * (d > 0.0f ? d : 0.0f));
                }
            }
            if (vBrake < v) v = vBrake;
        }
    }

    ctrl_->setForwardSpeed(v);
    ctrl_->setHeadingTrim(wallTrimDeg(remaining));
    publish(remaining, v);
}

// Stop on the centre, wait for the robot to actually be still, then act.
void Navigator::stopThen(Action a) {
    stallSinceMs_ = 0;
    ctrl_->setForwardSpeed(0.0f);
    ctrl_->setHeadingTrim(0.0f);
    pending_       = a;
    settleStartMs_ = HAL_GetTick();
    phase_         = SETTLE;
}

// True once the robot has made no real progress towards its stop point for
// STALL_MS. The window restarts whenever it gains STALL_PROGRESS_MM.
bool Navigator::stalled(float remaining) {
    const uint32_t now = HAL_GetTick();
    if (stallSinceMs_ == 0 || (stallRefMm_ - remaining) > navcfg::STALL_PROGRESS_MM) {
        stallRefMm_   = remaining;
        stallSinceMs_ = now;
        return false;
    }
    return (now - stallSinceMs_) > navcfg::STALL_MS;
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
        // A search that reaches the goal has driven and mapped a full path
        // start->goal: the map is ready for a speed run NOW, even if the robot
        // is then lifted out (the +20s reset) or its drive home is stopped.
        if (state_ == SEARCH) mapReady_ = true;
        returnKnown_ = (state_ == SPEED);
        state_   = RETURN;
        cacheKey_ = -1;              // lookahead must re-walk the path home
        // After a speed run the way home is known: go back at speed-run pace
        // (saves maze time; only the run TO the goal is scored).
        cruise_  = returnKnown_ ? cfg::SPEED_RUN_MMPS : cfg::SEARCH_SPEED_MMPS;
        pending_ = decide();
        ++decisions_;
        tel.lastAction = pending_;
        act(pending_);
        break;

    case ACT_HOME:   startHoming();                      break;
    case ACT_PARKED: mapReady_ = true; selfParked_ = true; finish(DONE); break;
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
        parkStartMs_ = HAL_GetTick();
        stallSinceMs_ = 0;
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
        parkStartMs_ = HAL_GetTick();
        stallSinceMs_ = 0;
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
    // Parking backs towards a wall: stopping early (friction, tail touching)
    // is fine, waiting forever is not — it would leave the run unfinished.
    const bool timedOut = (HAL_GetTick() - parkStartMs_) > navcfg::PARK_TIMEOUT_MS;
    if (v != 0.0f && (stalled(remaining) || timedOut)) ++tel.stallStops;
    if (v == 0.0f || stalled(remaining) || timedOut) { stopThen(ACT_PARKED); return; }
    ctrl_->setForwardSpeed(-v);
    ctrl_->setHeadingTrim(0.0f);
    publish(remaining, -v);
}

/* ---- curved turns ---------------------------------------------------------- */

void Navigator::startArc(float traveled) {
    arcPending_ = false;
    arcStartMm_ = traveled;


    const float vc = (cfg::CURVE_SPEED_MMPS < cruise_) ? cfg::CURVE_SPEED_MMPS : cruise_;
    ctrl_->setForwardSpeed(vc);
    ctrl_->setHeadingTrim(0.0f);
    ctrl_->arcTurn(Planner::turnDegrees(turn_), arcLenMm_);
    ++tel.arcs;
    phase_ = ARC;
}

void Navigator::updateArc() {
    const float vc = (cfg::CURVE_SPEED_MMPS < cruise_) ? cfg::CURVE_SPEED_MMPS : cruise_;
    ctrl_->setForwardSpeed(vc);
    if (ctrl_->arcActive()) { publish(0.0f, vc); return; }

    // Curve done: the axle is half a cell past the turn cell's centre, on the
    // new corridor's centreline. Target the next cell's centre from there,
    // measured from where the curve began so nothing is lost to timing.
    Planner::applyTurn(pose_, turn_);
    Planner::stepForward(pose_);
    segmentEndMm_ = arcStartMm_ + arcLenMm_ + (cfg::CELL_TRAVEL_MM - arcRadiusMm());
    decided_ = braking_ = false;
    pidWall_.reset();
    phase_ = DRIVE;
}

// Known-path run, target cell with a known front wall: take the position
// along the corridor from that wall (so curves start on time). The ToF gives
// real mm; segment distances are encoder mm (CELL_TRAVEL_MM per real
// CELL_PITCH_MM), so convert.
float Navigator::frontArcReferenced(float traveled, float remaining) {
    tel.frontRef = 0;
    if (!map_.isVisited(pose_.x, pose_.y) || !map_.hasWall(pose_.x, pose_.y, pose_.facing))
        return remaining;
    if (!walls_->ok(cfg::TOF_FRONT)) return remaining;
    const float fd = walls_->distanceMm(cfg::TOF_FRONT);
    if (fd > navcfg::ARC_FRONT_REF_MAX_MM) return remaining;
    const float realToCentre = fd - ctrl_->speedRef() * navcfg::FRONT_LATENCY_S - navcfg::FRONT_STOP_MM;
    const float r = realToCentre * (cfg::CELL_TRAVEL_MM / cfg::CELL_PITCH_MM);
    if (std::fabs(r - remaining) > navcfg::ARC_FRONT_REF_GATE_MM) return remaining;   // not that wall
    segmentEndMm_ = traveled + r;
    tel.frontRef  = 1;
    return r;
}

/* ---- speed-run lookahead --------------------------------------------------- */

// How far the known path runs straight beyond the target cell. The flood
// is fixed during a speed run, so the walk is cached per target cell.
float Navigator::straightAheadMm() {
    const int key = (pose_.x << 8) | (pose_.y << 2) | pose_.facing;
    if (key != cacheKey_) {
        cacheKey_ = key;
        straightCache_ = 0;
        eventArc_ = false;
        int x = pose_.x, y = pose_.y;
        for (int i = 0; i < maze::MAX_DIM; ++i) {
            if (state_ == SPEED && map_.isGoal(x, y)) break;                    // stop
            if (state_ == RETURN && x == startX_ && y == maze::START_Y) break;  // stop
            Dir d;
            if (!flood_.nextDir(map_, x, y, pose_.facing, d)) break;            // stop
            if (d != pose_.facing) {
                const Turn t = Planner::turnFor(pose_.facing, d);
                eventArc_ = arcsOn() && (t == TURN_LEFT || t == TURN_RIGHT);    // else stop
                break;
            }
            MazeMap::neighbour(x, y, d, x, y);
            ++straightCache_;
        }
        tel.straightAhead = straightCache_;
    }
    return straightCache_ * cfg::CELL_TRAVEL_MM;
}

/* ---- lateral centring ------------------------------------------------------ */

// Is this side's reading a wall that really continues beside the sensor?
// s: 0 left, 1 right. known/mapWall: what the map says about that wall.
bool Navigator::sideUsable(int s, bool known, bool mapWall, float& mm) {
    SideTrack& t = side_[s];
    const int sensor = (s == 0) ? cfg::TOF_LEFT : cfg::TOF_RIGHT;

    if (known && !mapWall) return false;            // map: opening here, never follow
    if (t.latched)         return false;            // wall already seen to end
    if (!walls_->ok(sensor)) return false;

    mm = walls_->distanceMm(sensor);
    const float centred = (s == 0) ? navcfg::WC_LEFT_CENTERED_MM : navcfg::WC_RIGHT_CENTERED_MM;

    if (mm > centred + navcfg::WC_BAND_MM) {        // too far to be this corridor's wall
        if (!known) t.latched = true;               // unexplored: the wall has ended
        return false;
    }
    if (mm < centred - navcfg::WC_BAND_MM) return false;   // implausibly close: skip sample

    if (!known && t.hasLast && (mm - t.lastMm) > navcfg::WC_JUMP_MM) {
        t.latched = true;                           // sudden step longer: wall end
        return false;
    }
    t.lastMm  = mm;
    t.hasLast = true;
    return true;
}

// Returns a small heading lean; the heading loop does the rest (#28).
float Navigator::wallTrimDeg(float remaining) {
    tel.wallMode = 0;
    tel.wallErrMm = tel.wallTrimDeg = 0.0f;

    // Side-sensor position along the track relative to the target centre, in
    // the robot's own cell length (CELL_TRAVEL_MM): posts sit half a driven
    // cell either side of a centre, whatever CELL_TRAVEL_ADJUST_MM is.
    const float pitch = cfg::CELL_TRAVEL_MM;
    const float p = cfg::TOF_SIDE_AHEAD_MM - remaining;

    // Which cell are the side sensors beside? The target cell once they have
    // crossed its boundary, the cell before it until then.
    int cx = pose_.x, cy = pose_.y;
    if (p < -0.5f * pitch)
        MazeMap::neighbour(pose_.x, pose_.y, MazeMap::opposite(pose_.facing), cx, cy);
    const int key = (cx << 8) | (cy << 2) | pose_.facing;
    if (key != sideCellKey_) {                      // new cell beside us: re-arm both sides
        sideCellKey_ = key;
        side_[0] = SideTrack{};
        side_[1] = SideTrack{};
    }

    // Corner posts sit on cell boundaries; keep clear of them.
    float q = std::fmod(p + 0.5f * pitch, pitch);
    if (q < 0.0f) q += pitch;
    const float fromPost = (q < pitch - q) ? q : pitch - q;

    const Dir  leftDir  = static_cast<Dir>((pose_.facing + 3) % 4);
    const Dir  rightDir = static_cast<Dir>((pose_.facing + 1) % 4);
    const bool known    = map_.isVisited(cx, cy);

    float ld = 0.0f, rd = 0.0f;
    const bool gated = fromPost < navcfg::WC_POST_GATE_MM || ctrl_->speedRef() < 1.0f;
    const bool lUse  = !gated && sideUsable(0, known, map_.hasWall(cx, cy, leftDir),  ld);
    const bool rUse  = !gated && sideUsable(1, known, map_.hasWall(cx, cy, rightDir), rd);

    tel.sideState = static_cast<uint8_t>((lUse ? 1 : 0) | (rUse ? 2 : 0) |
                                         (side_[0].latched ? 4 : 0) | (side_[1].latched ? 8 : 0) |
                                         (known ? 16 : 0));

    float err = 0.0f, gain = 0.0f;
    uint8_t mode = 0;
    if (lUse && rUse) {
        // Two real parallel walls always sum to the same width, however far
        // off-centre we are; a stub or an opening breaks that.
        if (std::fabs((ld + rd) - navcfg::WC_SUM_CENTERED_MM) < navcfg::WC_WIDTH_TOL_MM) {
            err = (ld - rd) - navcfg::WC_CENTER_TRIM_MM;   // + = drifted right
            gain = navcfg::WC_GAIN_BOTH_WALLS;
            mode = 1;
        }
    } else if (lUse) {
        // Hold the distance this side reads when truly centred (includes the
        // measured left/right asymmetry). x2 keeps one PID tuning valid.
        err = 2.0f * (ld - navcfg::WC_LEFT_CENTERED_MM);
        gain = navcfg::WC_GAIN_ONE_WALL;
        mode = 2;
    } else if (rUse) {
        err = 2.0f * (navcfg::WC_RIGHT_CENTERED_MM - rd);
        gain = navcfg::WC_GAIN_ONE_WALL;
        mode = 3;
    }

    // Switching reference (both / left / right / none) must not carry the old
    // integral and derivative into the new error.
    const bool modeChanged = (mode != lastWallMode_);
    if (modeChanged) { pidWall_.reset(); lastWallMode_ = mode; }
    tel.wallMode = mode;
    if (mode == 0) { lastTrim_ = 0.0f; return 0.0f; }
    if (std::fabs(err) < navcfg::WC_DEADBAND_MM) err = 0.0f;

    // Run the PID once per NEW side reading (~45Hz per sensor), not once per
    // main-loop pass. The main loop is fast and irregular; stepping the PID on
    // it made Ki depend on loop speed and reduced Kd to a one-pass blip. Held
    // between samples, WALL_PID's gains act at a steady rate and can be tuned.
    const uint32_t seq = walls_->samples(cfg::TOF_LEFT) + walls_->samples(cfg::TOF_RIGHT);
    if (seq == wallSeq_ && !modeChanged) {
        tel.wallErrMm = err;
        tel.wallTrimDeg = lastTrim_;
        return lastTrim_;
    }
    wallSeq_ = seq;

    const float trim = gain * pidWall_.compute(err);   // + = lean left (CCW)
    lastTrim_ = trim;
    tel.wallErrMm = err;
    tel.wallTrimDeg = trim;
    return trim;                                       // clamped in ControlLoop
}

/* ---- telemetry ----------------------------------------------------------- */

void Navigator::publish(float remaining, float v) {
    tel.remainingMm = remaining;
    tel.speedCmd    = v;
}
