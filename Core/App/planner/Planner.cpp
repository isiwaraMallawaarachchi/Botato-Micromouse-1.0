#include "Planner.hpp"

Turn Planner::turnFor(Dir facing, Dir target) {
    int diff = ((int)target - (int)facing + 4) % 4;
    switch (diff) {
        case 0: return TURN_NONE;
        case 1: return TURN_RIGHT;   // N->E is physically a RIGHT turn
        case 2: return TURN_AROUND;
        default: return TURN_LEFT;   // diff == 3
    }
}

void Planner::stepForward(GridPose& p) {
    switch (p.facing) {
        case NORTH: p.y += 1; break;
        case EAST:  p.x += 1; break;
        case SOUTH: p.y -= 1; break;
        case WEST:  p.x -= 1; break;
    }
}

void Planner::applyTurn(GridPose& p, Turn t) {
    int f = (int)p.facing;
    switch (t) {
        case TURN_RIGHT:  f = (f + 1) % 4; break;
        case TURN_LEFT:   f = (f + 3) % 4; break;
        case TURN_AROUND: f = (f + 2) % 4; break;
        default: break;
    }
    p.facing = (Dir)f;
}
