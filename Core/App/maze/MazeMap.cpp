#include "MazeMap.hpp"

void MazeMap::reset() {
    for (int x = 0; x < maze::WIDTH; ++x)
        for (int y = 0; y < maze::HEIGHT; ++y)
            cell_[x][y] = 0;

    // Outer boundary walls are always present.
    for (int x = 0; x < maze::WIDTH; ++x) {
        setWall(x, 0, SOUTH);
        setWall(x, maze::HEIGHT - 1, NORTH);
    }
    for (int y = 0; y < maze::HEIGHT; ++y) {
        setWall(0, y, WEST);
        setWall(maze::WIDTH - 1, y, EAST);
    }
}

Dir MazeMap::opposite(Dir d) {
    switch (d) {
        case NORTH: return SOUTH;
        case EAST:  return WEST;
        case SOUTH: return NORTH;
        default:    return EAST;
    }
}

void MazeMap::neighbour(int x, int y, Dir d, int& nx, int& ny) {
    nx = x; ny = y;
    switch (d) {
        case NORTH: ny = y + 1; break;
        case EAST:  nx = x + 1; break;
        case SOUTH: ny = y - 1; break;
        case WEST:  nx = x - 1; break;
    }
}

bool MazeMap::inBounds(int x, int y) const {
    return x >= 0 && x < maze::WIDTH && y >= 0 && y < maze::HEIGHT;
}

void MazeMap::setWall(int x, int y, Dir d) {
    if (!inBounds(x, y)) return;
    cell_[x][y] |= (1 << d);
    int nx, ny;
    neighbour(x, y, d, nx, ny);
    if (inBounds(nx, ny)) cell_[nx][ny] |= (1 << opposite(d));
}

bool MazeMap::hasWall(int x, int y, Dir d) const {
    if (!inBounds(x, y)) return true;   // outside = wall
    return (cell_[x][y] >> d) & 0x1;
}

void MazeMap::markVisited(int x, int y) {
    if (inBounds(x, y)) cell_[x][y] |= (1 << 4);
}

bool MazeMap::isVisited(int x, int y) const {
    if (!inBounds(x, y)) return false;
    return (cell_[x][y] >> 4) & 0x1;
}

bool MazeMap::isGoal(int x, int y) const {
    return x >= maze::GOAL_X_MIN && x <= maze::GOAL_X_MAX &&
           y >= maze::GOAL_Y_MIN && y <= maze::GOAL_Y_MAX;
}
