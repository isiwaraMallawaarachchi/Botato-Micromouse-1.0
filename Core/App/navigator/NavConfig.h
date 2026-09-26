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

/* ---- Wall detection at the decision point (calibrated mm from lens) ------ */
constexpr float SIDE_WALL_EXPECTED_MM  = (cfg::CORRIDOR_MM - cfg::BODY_WIDTH_MM) * 0.5f; // ~47
constexpr float SIDE_WALL_PRESENT_MM   = 100.0f;   // no wall reads >200
constexpr float FRONT_WALL_EXPECTED_MM = cfg::CORRIDOR_MM * 0.5f + SENSE_LOOKAHEAD_MM
                                       - cfg::TOF_FRONT_AHEAD_MM;                  // 64
constexpr float FRONT_WALL_PRESENT_MM  = 130.0f;   // next wall back reads ~256

/* ---- Start-corner detection ---------------------------------------------
 * A side opening where the map has the outer wall proves the maze is on the
 * other side of the start. Must be a VALID reading this far: a real wall
 * reads ~47mm, the next wall through an opening ~240mm or no target.      */
constexpr float OUTER_OPEN_MM = 200.0f;

/* ---- Front-wall stop -----------------------------------------------------
 * Stopping on a front wall uses the front ToF (absolute) instead of the
 * encoders, and re-anchors position — every front wall erases drift.
 * With the axle on the cell centre the front lens is this far from the wall.
 * Note: this sits near the bottom of the VL53L0X's range, so the 30mm point
 * in TEST_TOF_CAL matters for the Front sensor.                            */
constexpr float FRONT_STOP_MM     = cfg::CORRIDOR_MM * 0.5f - cfg::TOF_FRONT_AHEAD_MM;  // 34
constexpr float FRONT_REF_MAX_MM  = 120.0f;   // only trust the front wall this close
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
constexpr float    PARK_GAP_MM       = 3.0f;    // parking stops this short of the back wall

// A search run only learns it must stop when it reaches the decision point,
// SENSE_LOOKAHEAD_MM before the centre. It must be able to stop in that room.
static_assert(cfg::SEARCH_SPEED_MMPS * cfg::SEARCH_SPEED_MMPS
              <= 2.0f * BRAKE_DECEL_MMPS2 * (SENSE_LOOKAHEAD_MM - CREEP_ZONE_MM),
              "SEARCH_SPEED_MMPS too high to brake within SENSE_LOOKAHEAD_MM: "
              "lower the speed or raise DECEL_MMPS2 in Config.h");

/* ---- Lateral wall centring ------------------------------------------------
 * Trims the heading setpoint (DECISIONS.md #28), never the rate.           */
constexpr float WC_SUM_CENTERED_MM = cfg::CORRIDOR_MM - cfg::BODY_WIDTH_MM;  // MEASURE: left+right, centred
constexpr float WC_CENTER_TRIM_MM  = 0.0f;    // MEASURE: left-right, centred
constexpr float WC_WIDTH_TOL_MM    = 15.0f;   // both-wall sanity window
constexpr float WC_DEADBAND_MM     = 2.0f;
constexpr float WC_POST_GATE_MM    = 25.0f;   // ignore side walls near a post
constexpr float WC_GAIN_BOTH_WALLS = 1.00f;
constexpr float WC_GAIN_ONE_WALL   = 0.75f;
constexpr ctrlcfg::Gains WALL_PID  = { 0.35f, 0.004f, 0.10f, 600.0f };    // mm -> deg

} // namespace navcfg

#endif // APP_NAVCONFIG_H
