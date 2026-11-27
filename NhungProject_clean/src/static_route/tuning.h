#pragma once
// Runtime-tunable parameters. Defaults come from build flags (-DSTATIC_ROUTE_...);
// the web tool (tools/web/sa_ban.html) changes them over UART or HC-05 Bluetooth with
// "$SET,NAME,VALUE" while the car is not running. RAM only: a reboot restores defaults.
// Included inside namespace StaticRoute after wheel_calibration.h.

#ifndef STATIC_ROUTE_CRUISE_CPS
#define STATIC_ROUTE_CRUISE_CPS 400.0f  // First floor test. Change to 700 only after validation.
#endif
#ifndef STATIC_ROUTE_APPROACH_CPS
#define STATIC_ROUTE_APPROACH_CPS 200.0f
#endif
#ifndef STATIC_ROUTE_AVOID_CPS
#define STATIC_ROUTE_AVOID_CPS 300.0f
#endif
#ifndef STATIC_ROUTE_LAT_ACCEL
#define STATIC_ROUTE_LAT_ACCEL 300.0f
#endif
#ifndef STATIC_ROUTE_LATERAL_GAIN
#define STATIC_ROUTE_LATERAL_GAIN 2.8e-4f
#endif
#ifndef STATIC_ROUTE_HEADING_GAIN
#define STATIC_ROUTE_HEADING_GAIN 0.027f
#endif
#ifndef STATIC_ROUTE_FILLET_MM
#define STATIC_ROUTE_FILLET_MM 150.0f
#endif
#ifndef STATIC_ROUTE_SAFE_MARGIN_MM
#define STATIC_ROUTE_SAFE_MARGIN_MM float(StaticMap::SAFE_MARGIN)
#endif
#ifndef STATIC_ROUTE_DETECT_MM
#define STATIC_ROUTE_DETECT_MM 650.0f
#endif

#ifndef STATIC_ROUTE_GYRO_SCALE
#define STATIC_ROUTE_GYRO_SCALE 1.0f  // web: calibration page, "xoay tay 360 do"
#endif
#ifndef STATIC_ROUTE_SYNC_GAIN
#define STATIC_ROUTE_SYNC_GAIN 3.0f   // 1/s: cross-coupled wheel sync (0 = off)
#endif
#ifndef STATIC_ROUTE_START_FIX
#define STATIC_ROUTE_START_FIX 1.0f   // 1: fit the hand-placed START pose from the VL53 beams
#endif
#ifndef STATIC_ROUTE_RANGE_FIX
#define STATIC_ROUTE_RANGE_FIX 1.0f   // 1: correct the position from beams on fixed boxes
#endif

struct Tuning {
    float cruiseCps, approachCps, avoidCps;  // per-wheel CPS limits: cruise, near GOAL, in a detour
    float latAccel;                          // mm/s^2 allowed in curves
    float lateralGain, headingGain;          // tracking feedback (1/mm^2, 1/mm)
    float filletMm;                          // corner radius at N1..N3 (applies at the next START)
    float safeMarginMm;                      // extra gap beside the car body when planning detours
    float detectMm;                          // ignore range echoes farther than this
    float gyroScale;                         // multiplies the gyro rate (360 / measured turn)
    float startFix, rangeFix;                // 0/1 switches, see localize.h
    float syncGain;                          // wheel sync: mm/s of correction per mm of lag
};
constexpr Tuning DefaultTuning = {STATIC_ROUTE_CRUISE_CPS, STATIC_ROUTE_APPROACH_CPS, STATIC_ROUTE_AVOID_CPS,
                                  STATIC_ROUTE_LAT_ACCEL, STATIC_ROUTE_LATERAL_GAIN, STATIC_ROUTE_HEADING_GAIN,
                                  STATIC_ROUTE_FILLET_MM, STATIC_ROUTE_SAFE_MARGIN_MM, STATIC_ROUTE_DETECT_MM,
                                  STATIC_ROUTE_GYRO_SCALE, STATIC_ROUTE_START_FIX, STATIC_ROUTE_RANGE_FIX,
                                  STATIC_ROUTE_SYNC_GAIN};
Tuning tune = DefaultTuning;

struct TuneParam { const char *name; float Tuning::*field; float lo, hi; };
constexpr TuneParam TuneTable[] = {
    {"CRUISE_CPS",   &Tuning::cruiseCps,    200.0f, 700.0f},
    {"APPROACH_CPS", &Tuning::approachCps,  100.0f, 300.0f},
    {"AVOID_CPS",    &Tuning::avoidCps,     150.0f, 700.0f},
    {"LAT_ACCEL",    &Tuning::latAccel,     100.0f, 800.0f},
    {"LATERAL_GAIN", &Tuning::lateralGain,  0.5e-4f, 10e-4f},
    {"HEADING_GAIN", &Tuning::headingGain,  0.005f, 0.08f},
    {"FILLET_MM",    &Tuning::filletMm,     60.0f, 300.0f},
    {"SAFE_MARGIN",  &Tuning::safeMarginMm, 0.0f,  50.0f},
    {"DETECT_MM",    &Tuning::detectMm,     250.0f, 1200.0f},
    {"GYRO_SCALE",   &Tuning::gyroScale,    0.9f,  1.1f},
    {"START_FIX",    &Tuning::startFix,     0.0f,  1.0f},
    {"RANGE_FIX",    &Tuning::rangeFix,     0.0f,  1.0f},
    {"SYNC_GAIN",    &Tuning::syncGain,     0.0f,  8.0f},
};
constexpr unsigned TuneCount = sizeof(TuneTable)/sizeof(TuneTable[0]);
constexpr bool defaultsInRange(unsigned i = 0) {
    return i >= TuneCount || (DefaultTuning.*(TuneTable[i].field) >= TuneTable[i].lo &&
                              DefaultTuning.*(TuneTable[i].field) <= TuneTable[i].hi && defaultsInRange(i+1));
}
static_assert(defaultsInRange(), "A -DSTATIC_ROUTE_... default is outside its allowed range");

// Old names kept so the control code reads the same as before.
float &CruiseCps = tune.cruiseCps, &ApproachCps = tune.approachCps, &AvoidCps = tune.avoidCps;
float &LatAccelMm = tune.latAccel, &LateralGain = tune.lateralGain, &HeadingGain = tune.headingGain;
float &DetectMm = tune.detectMm;
