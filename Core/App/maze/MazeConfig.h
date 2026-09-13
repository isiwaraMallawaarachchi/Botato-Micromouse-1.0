#ifndef APP_MAZECONFIG_H
#define APP_MAZECONFIG_H

#include <cstdint>

/*
 * MazeConfig — EDIT THIS FILE to match your physical maze.
 * Size, start cell + start heading, and the goal region all live here.
 * Walls are NOT set here; the robot discovers them by sensing.
 *
 * Coordinate system:
 *   x = column (0 = left),  y = row (0 = bottom / start row)
 *   Heading: 0 = NORTH (+y), 90 = EAST (+x), 180 = SOUTH (-y), 270 = WEST (-x)
 */

namespace maze {

// ---- Maze size ---- (competition = 16x16; use smaller for a test maze)
constexpr int WIDTH  = 16;
constexpr int HEIGHT = 16;

// Max supported size for static arrays. Keep >= WIDTH/HEIGHT. 16 is safe.
constexpr int MAX_DIM = 16;

// ---- Start ----
constexpr int   START_X = 0;
constexpr int   START_Y = 0;
constexpr float START_HEADING_DEG = 0.0f;   // facing NORTH out of the start cell

// ---- Goal region ----
// Inclusive bounding box. ANY cell inside counts as "goal reached", so the bot
// stops at the first mouth it enters and turns back. Edit these 4 numbers to
// retarget any maze; for a 1x1 goal set min == max on both axes.
constexpr int GOAL_X_MIN = 7;
constexpr int GOAL_X_MAX = 8;
constexpr int GOAL_Y_MIN = 7;
constexpr int GOAL_Y_MAX = 8;

// ---- Physical cell size (mm) ----
constexpr float CELL_MM = 187.5f;

} // namespace maze

#endif // APP_MAZECONFIG_H
