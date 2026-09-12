#ifndef APP_NAVIGATOR_HPP
#define APP_NAVIGATOR_HPP

#include "MazeMap.hpp"
#include "FloodFill.hpp"
#include "Planner.hpp"
#include "ControlLoop.hpp"
#include "PIDController.hpp"
#include "WallSensorArray.hpp"
#include "Encoder.hpp"

class Navigator {
public:
    enum State { IDLE, SEARCH, RETURN, SPEED, DONE };

    void init(ControlLoop* ctrl, WallSensorArray* walls,
              Encoder* encL, Encoder* encR);

    void startSearch();
    void startSpeed();
    void abort();
    void update();

    State state() const { return state_; }
    int cellX() const { return pose_.x; }
    int cellY() const { return pose_.y; }

    // exposed for debugging
    GridPose pose_;
    float headingRef_ = 0.0f;      // accumulated commanded heading
    float turnTargetHeading_ = 0.0f;
    float wallErrDbg_ = 0.0f;      // lateral error fed to the centring PID
    float wallOffsetDbg_ = 0.0f;   // heading lean it produced (deg)
    int   wallModeDbg_ = 0;        // 0 = off, 1 = both walls, 2 = left, 3 = right

private:
    ControlLoop*     ctrl_  = nullptr;
    WallSensorArray* walls_ = nullptr;
    Encoder*         encL_  = nullptr;
    Encoder*         encR_  = nullptr;

    MazeMap   map_;
    FloodFill flood_;

    State state_ = IDLE;
    enum Phase { SENSE, DECIDE, TURNING, DEAD_END, DRIVING, ARRIVE };
    Phase phase_ = SENSE;

    Turn  pendingTurn_ = TURN_NONE;
    float cellStartDistance_ = 0.0f;
    float searchSpeed_ = 0.0f;
    uint32_t turnStartMs_ = 0;
    bool  driveFollowsTurn_ = false;

    // Overshoot carried into the next cell so distance error can't compound
    // over a long straight run (guide 15).
    float cellCarryMm_ = 0.0f;

    // ---- Dead-end multi-point (K) turn — always turns LEFT ----
    // The reversal is two 90 steps, never one 180: a 180 command lands exactly
    // on the +/-180 wrap boundary, where gyro noise decides the direction.
    enum DeadEndStep {
        DE_IDLE,
        DE_SWING_OUT_A,
        DE_REVERSE_A,
        DE_STRAIGHTEN_A,
        DE_ROTATE_90A,
        DE_ROTATE_90B,
        DE_SWING_OUT_B,
        DE_REVERSE_B,
        DE_STRAIGHTEN_B,
        DE_SETTLE,
        DE_FINISHED
    };

    DeadEndStep deStep_     = DE_IDLE;
    float deSwingTargetDeg_ = 0.0f;
    float deRefDistance_    = 0.0f;

    // Lateral centring: mm of offset -> deg of heading lean.
    // Ki is small with a tight clamp (guide 14.2: integral must be clamped).
    PIDController pidWall_{0.35f, 0.004f, 0.10f, 600.0f};

    void beginCellSequence();
    void senseWalls();
    bool decideNextMove();
    void beginTurn(Turn t);
    bool turnComplete();
    void beginDrive(bool afterTurn);
    bool driveComplete();
    void driveUpdate();
    float wallCenterOffsetDeg();
    float avgDistanceMm() const;
    void goalOrReturnTransition();

    void beginDeadEndTurn();
    void updateDeadEndTurn();
    void beginRelativeTurn(float deltaDeg);
    void beginWheelSwing(bool pivotOnLeftWheel, float deltaDeg);
    bool swingComplete() const;
    void beginDeadEndReverse(float holdHeadingDeg);
    bool deadEndReverseComplete(float distanceMm) const;
};

#endif
