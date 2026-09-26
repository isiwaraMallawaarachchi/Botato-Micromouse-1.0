#ifndef APP_PLANNER_HPP
#define APP_PLANNER_HPP

#include "MazeMap.hpp"

/*
 * Planner — grid pose and absolute-to-relative turn translation. Pure logic.
 *
 * turnFor() is purely geometric: Dir is clockwise (N=0,E=1,S=2,W=3), so
 * N->E is always TURN_RIGHT, whatever the gyro's sign convention.
 * turnDegrees() is the ONE place that maps a Turn onto the gyro convention
 * (anticlockwise-positive). DECISIONS.md #21.
 */
enum Turn : uint8_t { TURN_NONE, TURN_LEFT, TURN_RIGHT, TURN_AROUND };

struct GridPose {
    int x = 0;
    int y = 0;
    Dir facing = NORTH;
};

class Planner {
public:
    static Turn  turnFor(Dir facing, Dir target);
    static float turnDegrees(Turn t);         // +90 left, -90 right, +180 around
    static void  stepForward(GridPose& p);
    static void  applyTurn(GridPose& p, Turn t);
};

#endif // APP_PLANNER_HPP
