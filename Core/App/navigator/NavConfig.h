#ifndef APP_NAVCONFIG_H
#define APP_NAVCONFIG_H

#include <cstdint>
#include "Config.h"
#include "ControlConfig.h"

/*
 * NavConfig — how the navigator interprets the maze.
 *
 * Position model: every decision and every turn happens with the WHEEL AXLE
 * on a cell centre. The robot pivots about its axle, so a turn does not move
 * it off the centre and no post-turn offset is needed. All distances below
 * are derived from the measured geometry in Config.h; values marked MEASURE
 * should be checked on the real maze with TEST_TOF_LIVE.
 */
namespace navcfg {

/* ---- Start --------------------------------------------------------------
 * Robot is parked in the start cell with its TAIL AGAINST THE BACK WALL,
 * facing the opening. The axle is then this far short of the cell centre. */
constexpr float START_OFFSET_MM = cfg::CORRIDOR_MM * 0.5f - cfg::AXLE_TO_REAR_MM;   // 60.5

/* ---- When to decide -----------------------------------------------------
 * Walls are read, and the next move chosen, while the axle is this far
 * short of the target cell centre. At that point the side sensors (30mm ahead
 * of the axle) sit exactly over the cell centre — the cleanest place to see
 * side walls, clear of the corner posts.                                    */
constexpr float SENSE_LOOKAHEAD_MM = cfg::TOF_SIDE_AHEAD_MM;                        // 30

/* ===========================================================================
 * >>> WALL DETECTION — tune the thresholds here <<<
 *
 * All values are CALIBRATED mm from each sensor's lens (after ToFConfig.h).
 * Measured with the robot centred in a cell, in a 180mm corridor:
 *
 *   side wall   Left ~59.7, Right ~63.7     no wall: ~250 or no target
 *   front wall  44 centred in the cell,     no wall: next wall ~266
 *               ~74 at the decision point (SENSE_LOOKAHEAD_MM earlier)
 *
 * A wall counts as PRESENT when the reading is BELOW its threshold. Keep each
 * threshold well above the with-wall reading (the robot is never exactly
 * centred: ~25mm of sideways error moves a side reading by 25mm) and well
 * below the no-wall reading. Check the result live with TEST_TOF_LIVE
 * (tofLive.wallLeft / wallFront / wallRight).
 * =========================================================================== */
constexpr float SIDE_WALL_PRESENT_MM   = 120.0f;   // wall ~60, no wall ~250
constexpr float FRONT_STOP_MM          = 44.0f;    // front reading, centred in cell (measured)
constexpr float FRONT_WALL_EXPECTED_MM = FRONT_STOP_MM + SENSE_LOOKAHEAD_MM;   // ~74
constexpr float FRONT_WALL_PRESENT_MM  = 150.0f;   // wall ~74, no wall ~266

// Guard rails: a tuning slip that would misread walls fails the build.
static_assert(SIDE_WALL_PRESENT_MM > 90.0f && SIDE_WALL_PRESENT_MM < 220.0f,
              "SIDE_WALL_PRESENT_MM must sit between a side wall (~60) and no wall (~250)");
static_assert(FRONT_WALL_PRESENT_MM > FRONT_WALL_EXPECTED_MM + 30.0f &&
              FRONT_WALL_PRESENT_MM < FRONT_WALL_EXPECTED_MM + cfg::CELL_PITCH_MM - 30.0f,
              "FRONT_WALL_PRESENT_MM must sit between a front wall and the next wall back");

/* ---- Start-corner detection ---------------------------------------------
 * A side opening where the map has the outer wall proves the maze is on the
 * other side of the start. Must be a VALID reading this far: a real wall
 * reads ~60mm, the next wall through an opening ~250mm or no target.      */
constexpr float OUTER_OPEN_MM = 200.0f;

/* ---- Front-wall stop -----------------------------------------------------
 * Stopping on a front wall uses the front ToF (absolute) instead of the
 * encoders, and re-anchors position — every front wall erases drift. The
 * robot stops when the front reading reaches FRONT_STOP_MM (tuning block).  */
constexpr float FRONT_REF_MAX_MM  = 120.0f;   // only trust the front wall this close
static_assert(FRONT_REF_MAX_MM > FRONT_WALL_EXPECTED_MM + 20.0f,
              "FRONT_REF_MAX_MM must cover the front wall seen from the decision point");
constexpr float FRONT_LATENCY_S   = 0.030f;   // sensor + filter lag, compensated

/* ---- Braking -------------------------------------------------------------
 * Braking follows v = sqrt(2 * decel * remaining), so the robot arrives at the
 * stop point at zero speed. Slightly below the control-loop DECEL so the
 * reference never demands more than the ramp can deliver.                  */
constexpr float    BRAKE_DECEL_MMPS2 = 0.8f * cfg::DECEL_MMPS2;
constexpr float    CREEP_MMPS        = 40.0f;
constexpr float    CREEP_ZONE_MM     = 6.0f;    // last mm covered at creep speed
constexpr float    STOP_TOL_MM       = 1.0f;
constexpr float    SETTLE_MMPS       = 15.0f;
constexpr uint32_t SETTLE_TIMEOUT_MS = 400;
constexpr float    PARK_SPEED_MMPS   = 150.0f;

// Stall guard. The last millimetres of every stop run at CREEP_MMPS; under load
// a small gear motor can stop turning there (static friction, a tail touching
// the wall). Without a guard the robot waits forever for the last 1mm — and a
// search that never finishes parking never marks itself done. No progress of
// STALL_PROGRESS_MM for STALL_MS counts as arrived:
//   stops:   only within STALL_ARRIVE_MM of the stop point
//   parking: anywhere (it is backing towards a wall), plus a hard time limit
constexpr uint32_t STALL_MS          = 300;
constexpr float    STALL_PROGRESS_MM = 0.5f;
constexpr float    STALL_ARRIVE_MM   = 10.0f;
constexpr uint32_t PARK_TIMEOUT_MS   = 4000;
// Parking stops this short of the back wall. Reversing lands within ~3mm and
// the robot can sit ~0.5 deg off square, which swings a tail corner ~0.4mm:
// 8mm keeps clear contact-free. The next run starting from this parked pose
// accounts for the gap (Navigator::begin).
constexpr float    PARK_GAP_MM       = 8.0f;

// A search run only learns it must stop when it reaches the decision point,
// SENSE_LOOKAHEAD_MM before the centre. It must be able to stop in that room.
static_assert(cfg::SEARCH_SPEED_MMPS * cfg::SEARCH_SPEED_MMPS
              <= 2.0f * BRAKE_DECEL_MMPS2 * (SENSE_LOOKAHEAD_MM - CREEP_ZONE_MM),
              "SEARCH_SPEED_MMPS too high to brake within SENSE_LOOKAHEAD_MM: "
              "lower the speed or raise DECEL_MMPS2 in Config.h");

/* ---- Lateral wall centring ------------------------------------------------
 * Trims the heading setpoint (DECISIONS.md #28), never the rate.           */
// Measured with TEST_TOF_LIVE, robot exactly centred (tofLive.sideSum / sideDiff).
constexpr float WC_SUM_CENTERED_MM = 122.4f;  // left + right when centred
constexpr float WC_CENTER_TRIM_MM  = -4.9f;   // left - right when centred
// What each side reads when centred, used when only ONE wall is visible.
constexpr float WC_LEFT_CENTERED_MM  = 0.5f * (WC_SUM_CENTERED_MM + WC_CENTER_TRIM_MM);  // ~58.8
constexpr float WC_RIGHT_CENTERED_MM = 0.5f * (WC_SUM_CENTERED_MM - WC_CENTER_TRIM_MM);  // ~63.7
constexpr float WC_WIDTH_TOL_MM    = 15.0f;   // both-wall sanity window
constexpr float WC_DEADBAND_MM     = 2.0f;
constexpr float WC_POST_GATE_MM    = 25.0f;   // ignore side walls near a post
// Wall-end protection. A side is only followed while its wall really
// continues beside the sensor:
//  - cell walls already in the map: a side the map marks OPEN is never used;
//  - cell not yet sensed: the reading must stay within WC_BAND_MM of that
//    side's centred value and must not jump longer by more than WC_JUMP_MM
//    between samples. Either one latches that side OFF until the next cell.
// When a wall ends, the filtered reading steps up by 100+mm in one sample, far
// above any real sideways drift (< 1mm per sample at search speed).
constexpr float WC_BAND_MM         = 25.0f;
constexpr float WC_JUMP_MM         = 10.0f;
constexpr float WC_GAIN_BOTH_WALLS = 1.00f;
constexpr float WC_GAIN_ONE_WALL   = 0.75f;
constexpr ctrlcfg::Gains WALL_PID  = { 0.75f, 0.004f, 0.55f, 600.0f };    // mm -> deg

} // namespace navcfg

#endif // APP_NAVCONFIG_H
