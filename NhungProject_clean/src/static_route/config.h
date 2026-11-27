#pragma once
#include <stdint.h>
#include "map_config.h"

namespace StaticRoute {
using StaticMap::Waypoint;
using StaticMap::ROUTE;
using StaticMap::ROUTE_COUNT;
using StaticMap::START_X;
using StaticMap::START_Y;
using StaticMap::START_HEADING_DEG;
using StaticMap::GOAL_X;
using StaticMap::GOAL_Y;
static_assert(ROUTE_COUNT == 5, "This test measures four legs");
static_assert(ROUTE[0].x == START_X && ROUTE[0].y == START_Y, "Start mismatch");
static_assert(ROUTE[ROUTE_COUNT-1].x == GOAL_X && ROUTE[ROUTE_COUNT-1].y == GOAL_Y, "Goal mismatch");

constexpr float CRUISE_MM_S = 150.0f;
constexpr float NEAR_SPEED_MM_S = 38.0f, NEAR_DISTANCE_MM = 65.0f;
constexpr float TURN_RAD_S = 0.85f, ANG_ACCEL_RAD_S2 = 2.4f;
constexpr float ACCEL_MM_S2 = 170.0f, DECEL_MM_S2 = 320.0f;
constexpr float POSITION_TOL_MM = 12.0f, ANGLE_TOL_DEG = 3.0f;
constexpr float BRAKE_AT_MM = 0.5f*POSITION_TOL_MM;
constexpr float STILL_MM_S = 6.0f, STILL_DEG_S = 3.0f;
constexpr uint32_t SETTLE_MS = 150, POLL_MS = 10, LOG_MS = 100;
constexpr uint32_t IMU_STALE_MS = 35, CONTROL_TIMEOUT_MS = 65;
}
