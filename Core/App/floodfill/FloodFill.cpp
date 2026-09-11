#include "FloodFill.hpp"

namespace {
    constexpr uint16_t INF = 0xFFFF;

    // Simple fixed-size FIFO queue for BFS (max cells = 16*16 = 256).
    struct Queue {
        uint8_t xs[maze::MAX_DIM * maze::MAX_DIM];
        uint8_t ys[maze::MAX_DIM * maze::MAX_DIM];
        int head = 0, tail = 0;
        void push(int x, int y) { xs[tail] = (uint8_t)x; ys[tail] = (uint8_t)y; ++tail; }
        bool empty() const { return head == tail; }
        void pop(int& x, int& y) { x = xs[head]; y = ys[head]; ++head; }
    };

    void step(int x, int y, Dir d, int& nx, int& ny) {
        nx = x; ny = y;
        switch (d) {
            case NORTH: ny = y + 1; break;
            case EAST:  nx = x + 1; break;
            case SOUTH: ny = y - 1; break;
            case WEST:  nx = x - 1; break;
        }
    }
}

void FloodFill::floodFrom(const MazeMap& map, const int (*targets)[2], int count) {
    for (int x = 0; x < maze::WIDTH; ++x)
        for (int y = 0; y < maze::HEIGHT; ++y)
            distance_[x][y] = INF;

    Queue q;
    for (int i = 0; i < count; ++i) {
        int gx = targets[i][0], gy = targets[i][1];
        distance_[gx][gy] = 0;
        q.push(gx, gy);
    }

    while (!q.empty()) {
        int x, y; q.pop(x, y);
        uint16_t d = distance_[x][y];
        for (int di = 0; di < 4; ++di) {
            Dir dir = (Dir)di;
            if (map.hasWall(x, y, dir)) continue;      // can't cross a wall
            int nx, ny; step(x, y, dir, nx, ny);
            if (!map.inBounds(nx, ny)) continue;
            if (distance_[nx][ny] > d + 1) {
                distance_[nx][ny] = d + 1;
                q.push(nx, ny);
            }
        }
    }
}

void FloodFill::recompute(const MazeMap& map) {
    constexpr int GW = maze::GOAL_X_MAX - maze::GOAL_X_MIN + 1;
    constexpr int GH = maze::GOAL_Y_MAX - maze::GOAL_Y_MIN + 1;
    int cells[GW * GH][2];
    int n = 0;
    for (int x = maze::GOAL_X_MIN; x <= maze::GOAL_X_MAX; ++x)
        for (int y = maze::GOAL_Y_MIN; y <= maze::GOAL_Y_MAX; ++y) {
            cells[n][0] = x; cells[n][1] = y; ++n;
        }
    floodFrom(map, cells, n);
}

void FloodFill::recomputeTo(const MazeMap& map, int tx, int ty) {
    int one[1][2] = { { tx, ty } };
    floodFrom(map, one, 1);
}

bool FloodFill::nextDir(const MazeMap& map, int x, int y, Dir& out) const {
    uint16_t best = 0xFFFF;
    bool found = false;
    for (int di = 0; di < 4; ++di) {
        Dir dir = (Dir)di;
        if (map.hasWall(x, y, dir)) continue;
        int nx, ny; step(x, y, dir, nx, ny);
        if (!map.inBounds(nx, ny)) continue;
        if (distance_[nx][ny] < best) {
            best = distance_[nx][ny];
            out = dir;
            found = true;
        }
    }
    return found;
}
