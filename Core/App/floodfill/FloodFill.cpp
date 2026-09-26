#include "FloodFill.hpp"

namespace {
    constexpr int CAP = maze::MAX_DIM * maze::MAX_DIM;

    // Fixed FIFO. BFS pushes each cell at most once, so CAP always suffices;
    // the guard still refuses to write past the end if that ever changes.
    struct Queue {
        uint8_t xs[CAP], ys[CAP];
        int head = 0, tail = 0;
        bool push(int x, int y) {
            if (tail >= CAP) return false;
            xs[tail] = static_cast<uint8_t>(x);
            ys[tail] = static_cast<uint8_t>(y);
            ++tail;
            return true;
        }
        bool empty() const { return head == tail; }
        void pop(int& x, int& y) { x = xs[head]; y = ys[head]; ++head; }
    };
}

void FloodFill::flood(const MazeMap& map, const int (*seeds)[2], int count, bool knownOnly) {
    for (int x = 0; x < maze::MAX_DIM; ++x)
        for (int y = 0; y < maze::MAX_DIM; ++y)
            distance_[x][y] = INF;

    Queue q;
    for (int i = 0; i < count; ++i) {
        distance_[seeds[i][0]][seeds[i][1]] = 0;
        q.push(seeds[i][0], seeds[i][1]);
    }

    while (!q.empty()) {
        int x, y;
        q.pop(x, y);
        const uint16_t d = distance_[x][y];
        for (int di = 0; di < 4; ++di) {
            const Dir dir = static_cast<Dir>(di);
            if (map.hasWall(x, y, dir)) continue;
            int nx, ny;
            MazeMap::neighbour(x, y, dir, nx, ny);
            if (!map.inBounds(nx, ny)) continue;
            if (knownOnly && !map.isVisited(nx, ny)) continue;
            if (distance_[nx][ny] > d + 1) {
                distance_[nx][ny] = static_cast<uint16_t>(d + 1);
                q.push(nx, ny);
            }
        }
    }
}

void FloodFill::toGoal(const MazeMap& map, bool knownOnly) {
    constexpr int N = (maze::GOAL_X_MAX - maze::GOAL_X_MIN + 1) *
                      (maze::GOAL_Y_MAX - maze::GOAL_Y_MIN + 1);
    int seeds[N][2];
    int n = 0;
    for (int x = maze::GOAL_X_MIN; x <= maze::GOAL_X_MAX; ++x)
        for (int y = maze::GOAL_Y_MIN; y <= maze::GOAL_Y_MAX; ++y) {
            seeds[n][0] = x; seeds[n][1] = y; ++n;
        }
    flood(map, seeds, n, knownOnly);
}

void FloodFill::toCell(const MazeMap& map, int tx, int ty, bool knownOnly) {
    const int one[1][2] = { { tx, ty } };
    flood(map, one, 1, knownOnly);
}

bool FloodFill::nextDir(const MazeMap& map, int x, int y, Dir prefer, Dir& out) const {
    uint16_t best = INF;
    bool found = false;
    for (int k = 0; k < 4; ++k) {
        const Dir dir = static_cast<Dir>((prefer + k) % 4);   // prefer tried first
        if (map.hasWall(x, y, dir)) continue;
        int nx, ny;
        MazeMap::neighbour(x, y, dir, nx, ny);
        if (!map.inBounds(nx, ny)) continue;
        if (distance_[nx][ny] < best) {                      // strict: ties keep prefer
            best  = distance_[nx][ny];
            out   = dir;
            found = true;
        }
    }
    return found;
}
