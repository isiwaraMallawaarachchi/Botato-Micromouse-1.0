#ifndef APP_MAZEMAP_HPP
#define APP_MAZEMAP_HPP

#include <cstdint>
#include "MazeConfig.h"

/*
 * MazeMap — wall storage for the maze. One byte per cell (4 wall bits +
 * visited). Walls are stored per absolute direction (N/E/S/W). Setting a
 * wall also sets the matching wall on the neighbouring cell, so the map
 * stays consistent from either side.
 */

enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

class MazeMap {
public:
    void reset();

    void setWall(int x, int y, Dir d);          // marks wall on both cells
    bool hasWall(int x, int y, Dir d) const;

    void markVisited(int x, int y);
    bool isVisited(int x, int y) const;

    bool isGoal(int x, int y) const;
    bool inBounds(int x, int y) const;

private:
    // bit0..3 = N,E,S,W wall ; bit4 = visited
    uint8_t cell_[maze::MAX_DIM][maze::MAX_DIM];

    static Dir opposite(Dir d);
    static void neighbour(int x, int y, Dir d, int& nx, int& ny);
};

#endif // APP_MAZEMAP_HPP
