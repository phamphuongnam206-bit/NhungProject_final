#pragma once

#if defined(MAP_NAM_DIAMETER_L_MM) || defined(MAP_NAM_DIAMETER_R_MM) || defined(MAP_NAM_CPR_L) || defined(MAP_NAM_CPR_R)
constexpr float CountsPerMm[] = {MAP_NAM_CPR_L/(Pi*MAP_NAM_DIAMETER_L_MM),
                                 MAP_NAM_CPR_R/(Pi*MAP_NAM_DIAMETER_R_MM)};
#elif defined(MAP_NAM_COUNTS_PER_MM)  // measured encoder counts per mm of travel, both wheels
constexpr float CountsPerMm[] = {MAP_NAM_COUNTS_PER_MM, MAP_NAM_COUNTS_PER_MM};
#else
constexpr float CountsPerMm[] = {4.5f,4.5f};
#endif
#ifndef MAP_NAM_TRACK_MM
#define MAP_NAM_TRACK_MM 90.0f
#endif
constexpr float Track=MAP_NAM_TRACK_MM;
static_assert(Track>0 && CountsPerMm[0]>0 && CountsPerMm[1]>0,"Invalid wheel calibration");
constexpr int EncoderSign[] = {-1, 1};
constexpr float GyroSign = 1, GyroScale = 1;
