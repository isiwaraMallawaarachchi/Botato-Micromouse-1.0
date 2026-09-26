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
 * Start corner: the robot ASSUMES the maze extends to its right (start cell at
 * START_X = 0). If it is actually to the left, the navigator detects it from
 * the first side opening where the map has the outer wall, moves what it has
 * learned to column WIDTH-1 and carries on — no reprogramming needed after
 * the maze is revealed.
 */
namespace maze {

constexpr int WIDTH   = 8;
constexpr int HEIGHT  = 8;
constexpr int MAX_DIM = 16;          // static array bound, keep >= WIDTH/HEIGHT

constexpr int START_X = 0;
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
