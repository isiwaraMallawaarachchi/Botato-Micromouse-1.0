#ifndef APP_NAVIGATOR_HPP
#define APP_NAVIGATOR_HPP

#include "MazeMap.hpp"
#include "FloodFill.hpp"
#include "Planner.hpp"
#include "ControlLoop.hpp"
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

    // ---- Dead-end multi-point (K) turn ----
    enum DeadEndStep {
        DE_IDLE,
        DE_SWING_OUT_A,     // angle away from the turn side
        DE_REVERSE_A,       // reverse while angled -> gains clearance
        DE_STRAIGHTEN_A,    // back to entry heading, now offset
        DE_ROTATE_180,      // the reversal itself
        DE_SWING_OUT_B,     // repeat to re-align after the 180
        DE_REVERSE_B,
        DE_STRAIGHTEN_B,
        DE_SETTLE,          // final reverse to sit in the cell
        DE_FINISHED
    };

    DeadEndStep deStep_     = DE_IDLE;
    bool  deTurnLeft_       = true;    // alternates each dead end (error cancelling)
    float deSwingTargetDeg_ = 0.0f;
    float deRefDistance_    = 0.0f;

    void beginCellSequence();
    void senseWalls();
    bool decideNextMove();
    void beginTurn(Turn t);
    bool turnComplete();
    void beginDrive(bool afterTurn);
    bool driveComplete();
    void approachSlowdown();
    float avgDistanceMm() const;
    void goalOrReturnTransition();

    void beginDeadEndTurn();
    void updateDeadEndTurn();
    void beginWheelSwing(bool pivotOnLeftWheel, float deltaDeg);
    bool swingComplete() const;
    void beginDeadEndReverse(float holdHeadingDeg);
    bool deadEndReverseComplete(float distanceMm) const;
};

#endif
