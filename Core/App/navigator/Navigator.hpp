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
    float wallErrDbg_ = 0.0f;
    bool  wallValidDbg_ = false;

private:
    ControlLoop*     ctrl_  = nullptr;
    WallSensorArray* walls_ = nullptr;
    Encoder*         encL_  = nullptr;
    Encoder*         encR_  = nullptr;

    MazeMap   map_;
    FloodFill flood_;

    State state_ = IDLE;
    enum Phase { SENSE, DECIDE, TURNING, DRIVING, ARRIVE };
    Phase phase_ = SENSE;

    Turn  pendingTurn_ = TURN_NONE;
    Dir   pendingDir_  = NORTH;
    float cellStartDistance_ = 0.0f;
    float searchSpeed_ = 0.0f;
    uint32_t turnStartMs_ = 0;
    float wcRunStartDist_ = -1.0f;

    void beginCellSequence();
    void senseWalls();
    bool decideNextMove();
    void beginTurn(Turn t);
    bool turnComplete();
    void beginDrive();
    bool driveComplete();
    float avgDistanceMm() const;
    void  goalOrReturnTransition();
    void updateWallCentering();
    void approachSlowdown();
};

#endif
