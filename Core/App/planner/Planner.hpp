#ifndef APP_PLANNER_HPP
#define APP_PLANNER_HPP

#include "MazeMap.hpp"

/*
 * Planner — small helpers translating absolute directions into robot-relative
 * turns, and tracking the robot's grid pose (cell x,y + facing direction).
 * Pure logic, no hardware. The Navigator uses this to decide "given I'm
 * facing EAST and flood-fill says go NORTH, what turn is that?"
 */

enum Turn : uint8_t { TURN_NONE, TURN_LEFT, TURN_RIGHT, TURN_AROUND };

struct GridPose {
    int x, y;
    Dir facing;
};

class Planner {
public:
    // What relative turn takes you from `facing` to `target`?
    static Turn turnFor(Dir facing, Dir target);

    // The heading in degrees (0=N,90=E,...) for an absolute direction.
    static float headingOf(Dir d);

    // Apply a completed forward step to the pose (moves one cell in facing).
    static void stepForward(GridPose& p);

    // Apply a completed turn to the pose's facing.
    static void applyTurn(GridPose& p, Turn t);
};

#endif // APP_PLANNER_HPP
