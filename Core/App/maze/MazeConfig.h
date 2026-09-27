#ifndef APP_MAZECONFIG_H
#define APP_MAZECONFIG_H

/*
 * MazeConfig — the logical maze: size, start and goal. Edit to retarget a
 * test maze. Physical cell size is in config/Config.h (CELL_PITCH_MM,
 * CELL_TRAVEL_MM), because the navigator and the drive tests both use it.
 *
 * Coordinates: x = column (0 = left), y = row (0 = BOTTOM, the start row).
 * Directions: NORTH = +y. The robot starts facing NORTH out of the start cell.
 *
 * Start corner. Stand in the start cell facing its opening:
 *   maze to the robot's RIGHT  ->  START_X = 0
 *   maze to the robot's LEFT   ->  START_X = WIDTH - 1
 * If the real maze turns out to be the other way, the navigator detects it
 * from the first side opening where the map has the outer wall, moves what it
 * has learned to the other edge and carries on — no reprogramming needed.
 *
 * ROBOFEST 2026 preliminary maze: all four green starts (A1, P1, A16, P16)
 * have the maze on the robot's LEFT, so START_X = WIDTH - 1. The goal
 * (H8-I9) is the centre 2x2 from every corner, so the goal box is the same.
 */
namespace maze {

constexpr int WIDTH   = 8;
constexpr int HEIGHT  = 8;
constexpr int MAX_DIM = 16;          // static array bound, keep >= WIDTH/HEIGHT

constexpr int START_X = WIDTH - 1;   // maze on the robot's LEFT (ROBOFEST 2026 prelim)
constexpr int START_Y = 0;

// Inclusive goal box. Entering ANY cell inside counts as reached.
constexpr int GOAL_X_MIN = 3;
constexpr int GOAL_X_MAX = 4;
constexpr int GOAL_Y_MIN = 3;
constexpr int GOAL_Y_MAX = 4;

static_assert(WIDTH <= MAX_DIM && HEIGHT <= MAX_DIM, "maze larger than MAX_DIM");
static_assert(START_X == 0 || START_X == WIDTH - 1, "start must be in a corner column");
static_assert(GOAL_X_MIN <= GOAL_X_MAX && GOAL_Y_MIN <= GOAL_Y_MAX, "bad goal box");

} // namespace maze

#endif // APP_MAZECONFIG_H
