#ifndef APP_MAZEMAP_HPP
#define APP_MAZEMAP_HPP

#include <cstdint>
#include "MazeConfig.h"

/*
 * MazeMap — one byte per cell: 4 wall bits (N,E,S,W) + visited. Setting a
 * wall also sets the matching wall on the neighbour, so the map is consistent
 * from both sides. The outer boundary is always walled.
 */
enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

class MazeMap {
public:
    void reset();

    void setWall(int x, int y, Dir d);
    bool hasWall(int x, int y, Dir d) const;
    void markVisited(int x, int y);
    bool isVisited(int x, int y) const;
    bool isGoal(int x, int y) const;
    bool inBounds(int x, int y) const;

    static Dir  opposite(Dir d);
    static void neighbour(int x, int y, Dir d, int& nx, int& ny);

private:
    uint8_t cell_[maze::MAX_DIM][maze::MAX_DIM] = {};
};

#endif // APP_MAZEMAP_HPP
