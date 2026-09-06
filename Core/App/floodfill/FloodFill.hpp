#ifndef APP_FLOODFILL_HPP
#define APP_FLOODFILL_HPP

#include <cstdint>
#include "MazeMap.hpp"

/*
 * FloodFill — classic BFS distance-to-goal over the known map. recompute()
 * fills distance_[][] where each cell holds its step-count to the nearest
 * goal cell, respecting known walls. nextDir() returns the neighbouring
 * direction with the lowest distance (the gradient to follow). Pure
 * algorithm, no hardware.
 */

class FloodFill {
public:
    // Recompute distances toward the goal cells defined in MazeConfig.
    void recompute(const MazeMap& map);
    // Recompute toward an arbitrary single target (used for return-to-start).
    void recomputeTo(const MazeMap& map, int tx, int ty);

    uint16_t distance(int x, int y) const { return distance_[x][y]; }

    // Best direction to step from (x,y): the open neighbour with the
    // lowest distance. Returns false if boxed in (no open neighbour).
    bool nextDir(const MazeMap& map, int x, int y, Dir& out) const;

private:
    uint16_t distance_[maze::MAX_DIM][maze::MAX_DIM];

    void floodFrom(const MazeMap& map, const int (*targets)[2], int count);
};

#endif // APP_FLOODFILL_HPP
