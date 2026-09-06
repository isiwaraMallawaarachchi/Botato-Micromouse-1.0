#ifndef APP_MAZECONFIG_H
#define APP_MAZECONFIG_H

#include <cstdint>

/*
 * MazeConfig — EDIT THIS FILE to match your physical maze.
 * Size, start cell + start heading, and the goal cells all live here.
 * Walls are NOT set here; the robot discovers them by sensing.
 *
 * Coordinate system:
 *   x = column (0 = left),  y = row (0 = bottom / start row)
 *   Heading: 0 = NORTH (+y), 90 = EAST (+x), 180 = SOUTH (-y), 270 = WEST (-x)
 */

namespace maze {

// ---- Maze size ---- (competition = 16x16; use smaller for a test maze)
constexpr int WIDTH  = 8;
constexpr int HEIGHT = 8;

// Max supported size for static arrays. Keep >= WIDTH/HEIGHT. 16 is safe.
constexpr int MAX_DIM = 16;

// ---- Start ----
constexpr int   START_X = 0;
constexpr int   START_Y = 0;
constexpr float START_HEADING_DEG = 0.0f;   // facing NORTH out of the start cell

// ---- Goal cells ----
// List every cell that counts as "reached the goal". For a standard 16x16
// the centre is a 2x2 block; for a small test maze, one cell is fine.
constexpr int GOAL_COUNT = 4;
constexpr int GOAL_CELLS[GOAL_COUNT][2] = {
    {3, 3}, {3, 4}, {4, 3}, {4, 4}    // centre 2x2 of a 16x16 maze
};

// ---- Physical cell size (mm) ---- (mirror of cfg::CELL_SIZE_MM; kept here
// so the maze module is self-contained. Change both if your cell differs.)
constexpr float CELL_MM = 180.0f;

} // namespace maze

#endif // APP_MAZECONFIG_H
