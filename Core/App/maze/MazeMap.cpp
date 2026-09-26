#include "MazeMap.hpp"

void MazeMap::reset() {
    for (int x = 0; x < maze::MAX_DIM; ++x)
        for (int y = 0; y < maze::MAX_DIM; ++y)
            cell_[x][y] = 0;

    for (int x = 0; x < maze::WIDTH; ++x) {
        setWall(x, 0, SOUTH);
        setWall(x, maze::HEIGHT - 1, NORTH);
    }
    for (int y = 0; y < maze::HEIGHT; ++y) {
        setWall(0, y, WEST);
        setWall(maze::WIDTH - 1, y, EAST);
    }
}

Dir MazeMap::opposite(Dir d) { return static_cast<Dir>((d + 2) % 4); }

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
    cell_[x][y] |= static_cast<uint8_t>(1u << d);
    int nx, ny;
    neighbour(x, y, d, nx, ny);
    if (inBounds(nx, ny)) cell_[nx][ny] |= static_cast<uint8_t>(1u << opposite(d));
}

bool MazeMap::hasWall(int x, int y, Dir d) const {
    if (!inBounds(x, y)) return true;
    return (cell_[x][y] >> d) & 0x1u;
}

void MazeMap::markVisited(int x, int y) {
    if (inBounds(x, y)) cell_[x][y] |= 0x10u;
}

bool MazeMap::isVisited(int x, int y) const {
    return inBounds(x, y) && ((cell_[x][y] >> 4) & 0x1u);
}

bool MazeMap::isGoal(int x, int y) const {
    return x >= maze::GOAL_X_MIN && x <= maze::GOAL_X_MAX &&
           y >= maze::GOAL_Y_MIN && y <= maze::GOAL_Y_MAX;
}
