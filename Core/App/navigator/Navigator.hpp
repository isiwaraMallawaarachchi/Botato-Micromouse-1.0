#ifndef APP_NAVIGATOR_HPP
#define APP_NAVIGATOR_HPP

#include <cstdint>
#include "MazeMap.hpp"
#include "FloodFill.hpp"
#include "Planner.hpp"
#include "ControlLoop.hpp"
#include "PIDController.hpp"
#include "WallSensorArray.hpp"
#include "Encoder.hpp"
#include "NavConfig.h"

/*
 * Navigator — maze policy. Speaks only intentions to ControlLoop
 * (setForwardSpeed, turnBy, setHeadingTrim); never touches hardware.
 *
 * Segment model. The robot drives toward the centre of a TARGET cell (pose_).
 * segmentEndMm_ is the axle distance, since the run started, at which the axle
 * is on that centre. When the axle is SENSE_LOOKAHEAD_MM short of it:
 *
 *   sense the target cell's walls and decide the next move, then
 *     straight -> extend the segment by one cell and keep going, no stop
 *     turn     -> brake to a stop on the centre, pivot, drive on
 *     goal     -> brake, stop; search turns around for the return leg
 *
 * Position error cannot compound: segment ends advance by exactly one
 * CELL_TRAVEL_MM, and every stop on a front wall re-anchors to the wall.
 *
 * Dead ends use a single 180-degree pivot, the same as any other turn.
 * After the return leg the robot faces NORTH and backs into the start
 * position, so every run begins from the same pose.
 */
class Navigator {
public:
    enum State  : uint8_t { IDLE, SEARCH, RETURN, SPEED, DONE, STUCK };
    enum Phase  : uint8_t { DRIVE, SETTLE, TURN, PARK, FINISHED };
    enum Action : uint8_t { ACT_STRAIGHT, ACT_LEFT, ACT_RIGHT, ACT_AROUND,
                            ACT_GOAL, ACT_HOME, ACT_PARKED, ACT_STUCK };

    void init(ControlLoop* ctrl, WallSensorArray* walls, Encoder* encL, Encoder* encR);

    void startSearch();
    bool startSpeed();            // false if no completed search map exists
    void abort();
    void clearMap();
    void update();                // main loop

    State    state()     const { return state_; }
    bool     running()   const { return phase_ != FINISHED; }
    bool     ended()     const { return state_ == DONE || state_ == STUCK; }
    bool     mapReady()  const { return mapReady_; }
    uint32_t decisions() const { return decisions_; }

    // Live Expressions: robot.navigator_.tel
    struct Telemetry {
        uint8_t  state, phase, x, y, facing, lastAction;
        uint8_t  wallL, wallF, wallR;       // last sensed at a decision point
        uint8_t  frontRef;                  // 1 = stop point taken from front ToF
        uint8_t  wallMode;                  // 0 off, 1 both, 2 left, 3 right
        float    remainingMm, speedCmd, wallErrMm, wallTrimDeg;
        uint32_t decisions;
        int32_t  straightAhead;             // speed run: straight cells ahead
    } tel = {};

private:
    ControlLoop*     ctrl_  = nullptr;
    WallSensorArray* walls_ = nullptr;
    Encoder*         encL_  = nullptr;
    Encoder*         encR_  = nullptr;

    MazeMap   map_;
    FloodFill flood_;
    GridPose  pose_;

    State  state_   = IDLE;
    Phase  phase_   = FINISHED;
    Action pending_ = ACT_STRAIGHT;
    Turn   turn_    = TURN_NONE;

    float    cruise_        = 0.0f;
    float    runStartMm_    = 0.0f;
    float    segmentEndMm_  = 0.0f;
    float    parkEndMm_     = 0.0f;
    bool     decided_       = false;
    bool     braking_       = false;
    bool     homing_        = false;
    bool     mapReady_      = false;
    uint32_t decisions_     = 0;
    uint32_t settleStartMs_ = 0;

    int straightCache_ = 0;
    int cacheKey_      = -1;

    PIDController pidWall_{navcfg::WALL_PID};

    float axleMm() const;
    float traveledMm() const { return axleMm() - runStartMm_; }

    void   begin(State s, float cruise);
    void   updateDrive();
    void   updateSettle();
    void   updateTurn();
    void   updatePark();
    void   senseWalls();
    Action decide();
    void   act(Action a);
    void   startTurn(Turn t);
    void   nextCell();
    void   startHoming();
    void   stopThen(Action a);
    void   finish(State s);
    float  frontReferenced(float traveled, float remaining);
    float  straightAheadMm();
    float  wallTrimDeg(float remaining);
    void   publish(float remaining, float v);
};

#endif // APP_NAVIGATOR_HPP
