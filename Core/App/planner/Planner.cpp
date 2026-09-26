#include "Planner.hpp"

Turn Planner::turnFor(Dir facing, Dir target) {
    switch ((target - facing + 4) % 4) {
        case 0:  return TURN_NONE;
        case 1:  return TURN_RIGHT;
        case 2:  return TURN_AROUND;
        default: return TURN_LEFT;
    }
}

float Planner::turnDegrees(Turn t) {
    switch (t) {
        case TURN_LEFT:   return  90.0f;
        case TURN_RIGHT:  return -90.0f;
        case TURN_AROUND: return 180.0f;   // one continuous half turn, anticlockwise
        default:          return   0.0f;
    }
}

void Planner::stepForward(GridPose& p) {
    MazeMap::neighbour(p.x, p.y, p.facing, p.x, p.y);
}

void Planner::applyTurn(GridPose& p, Turn t) {
    int f = p.facing;
    switch (t) {
        case TURN_RIGHT:  f = (f + 1) % 4; break;
        case TURN_LEFT:   f = (f + 3) % 4; break;
        case TURN_AROUND: f = (f + 2) % 4; break;
        default: break;
    }
    p.facing = static_cast<Dir>(f);
}
