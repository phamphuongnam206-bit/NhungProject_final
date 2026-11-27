// 5-node route with smooth (filleted) corners + measured-feedforward PI wheels.
// The car rounds corners while moving and sidesteps blocking boxes seen by the
// four front VL53L0X sensors with a smooth lateral offset (no stop-and-pivot).
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "static_route/config.h"
#include "static_route/telemetry.h"
#include "navigation/range_sensors.h"
#ifndef STATIC_ROUTE_HOST_TEST
#include <Arduino.h>
#include <Wire.h>
#endif
#ifndef STATIC_ROUTE_BAUD
#define STATIC_ROUTE_BAUD 115200
#endif

namespace StaticRoute {
constexpr float Pi = 3.14159265f, Rad = Pi/180.0f;
#include "navigation/wheel_calibration.h"
float bound(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }
float wrap(float a) {
    while (a > Pi) a -= 2*Pi;
    while (a < -Pi) a += 2*Pi;
    return a;
}
struct Pose { float x, y, h; };
struct Motion { float v, w; };
#include "static_route/tuning.h"
#include "static_route/path.h"
#include "static_route/avoid.h"
constexpr Pose StartPose{float(START_X), float(START_Y), START_HEADING_DEG*Rad};
Pose pose = StartPose;
Motion request{};
float gyroBias = 0, yawRate = 0;
int32_t encoderTotal[2]{};
// Route-only motor control.
// CPS limits are per wheel, not a command to force both wheels to equal CPS.
// CruiseCps/ApproachCps/AvoidCps live in tuning.h (runtime adjustable, 200..700 cruise).
constexpr float TurnCps = 200.0f;
constexpr float MaxCountsPerMm = CountsPerMm[0] > CountsPerMm[1] ? CountsPerMm[0] : CountsPerMm[1];
constexpr float RouteAccel = 700.0f / MaxCountsPerMm;
constexpr float RouteDecel = 900.0f / MaxCountsPerMm;
constexpr float RouteKp[2] = {0.018f, 0.018f};
constexpr float RouteKi[2] = {0.08f, 0.08f};
constexpr float RoutePwmMax = 85.0f, RouteIntegralMax = 30.0f;
static_assert(EncoderSign[0] == -1 && EncoderSign[1] == 1, "Check encoder polarity");

// Smooth tracking (Kanayama-style, gains per mm so the response is speed independent).
// curvature = path curvature - LateralGain*lateralError + HeadingGain*sin(headingError)
// LateralGain, HeadingGain, LatAccelMm and AvoidCps are in tuning.h.
constexpr float LateralSatMm = 60.0f;
constexpr float MaxCurv = 1.0f/60.0f;          // inner wheel never reverses (Track/2 < 60 mm)
constexpr float PlanDecel = 0.8f*RouteDecel;  // planner brakes a bit softer than the slew allows
constexpr float AlignStartDeg = 35.0f;         // pivot in place only if the start heading is this far off
constexpr float OffPathMm = 100.0f;

// VL53L0X mounts from the competition car (J8..J11): forward/left of the centre, yaw (+ left).
struct Mount { float forward, left, yaw; };
constexpr Mount Mounts[RangeSensors::Count] = {
    {45, 25, 35.43f*Rad}, {45, 25, 0}, {45, -25, 0}, {45, -25, -35.43f*Rad}};
constexpr float HardStopMm = 90.0f;  // straight beam this close = stop now (DetectMm: tuning.h)
constexpr float LookAheadMm = 900.0f;

// Forward feedforward: measured steady-state PWM, unloaded bench data.
// Reverse: only the 200 CPS point was measured; use linear scaling below it.
// No claim that these values overcome floor load or static friction.
float routeFeedforward(unsigned i, int dir, float magnitude) {
    const float cps[] = {0, 200, 400, 700, 2000};
    const float pwmL[] = {0, 12.5f, 16.5f, 25.5f, 63.5f};
    const float pwmR[] = {0, 15.5f, 20.5f, 29.5f, 67.5f};
    if (dir < 0) return (i ? 17.5f : 12.0f) * bound(magnitude, 0, 200) / 200.0f;
    const float *pwm = i ? pwmR : pwmL;
    for (unsigned k=1; k<5; ++k) {
        if (magnitude <= cps[k])
            return pwm[k-1] + (pwm[k]-pwm[k-1]) * (magnitude-cps[k-1])/(cps[k]-cps[k-1]);
    }
    return pwm[4];
}

struct Wheel {
    float cps = 0, integral = 0, duty = 0, targetCps = 0;
    uint32_t lastPulse = 0, stoppedAt = 0, startedAt = 0;
    int direction = 0, wrong = 0, applied = 0;
    bool watching = false;
    void stop(uint32_t now) {
        if (applied || duty > 0) stoppedAt = now;
        duty = integral = targetCps = 0;
        wrong = applied = 0; watching = false;
        // Keep filtered cps: SETTLE must observe actual coast, not zeroed feedback.
    }
    bool observe(int ticks, float dt, uint32_t now) {
        const float tau = fabsf(targetCps) <= 400.0f ? 0.065f : 0.040f;
        cps += dt/(tau+dt) * (ticks/dt-cps);
        if (!watching) return true;
        const int forward = ticks*direction;
        if (forward > 0) { lastPulse = now; wrong = 0; }
        if (forward < 0) wrong -= forward;
        // Keep the original 750 ms no-pulse and wrong-direction protection.
        return now-lastPulse < 750 && (now-startedAt < 125 || wrong < 24);
    }
    int output(float ref, unsigned index, float dt, uint32_t now) {
        if (fabsf(ref) < 5) { stop(now); return 0; }
        const int sign = ref > 0 ? 1 : -1;
        if (direction && direction != sign) {
            if (applied || duty > 0) { stop(now); return 0; }
            if (now-stoppedAt < 50 || fabsf(cps) > 30) return 0;
            integral = 0; wrong = 0; watching = false;
        }
        if (direction != sign) { direction = sign; watching = false; }
        targetCps = ref;
        const float magnitude = fabsf(ref);
        // Start monitoring every commanded motion, including very low CPS.
        // Do not reset the no-pulse timer when the target changes magnitude.
        if (!watching) { watching = true; lastPulse = startedAt = now; wrong = 0; }
        const float error = magnitude - sign*cps;
        const float ff = routeFeedforward(index, sign, magnitude);
        const float nextI = bound(integral + RouteKi[index]*error*dt, -RouteIntegralMax, RouteIntegralMax);
        // Anti-windup for saturation and duty slew. No forced PWM 42/78.
        const float lo = fmaxf(0, duty-150.0f*dt);
        const float hi = fminf(RoutePwmMax, duty+90.0f*dt);
        const float proposed = ff + RouteKp[index]*error + nextI;
        if ((proposed >= lo && proposed <= hi) || (proposed > hi && error < 0) || (proposed < lo && error > 0))
            integral = nextI;
        duty = bound(ff + RouteKp[index]*error + integral, lo, hi);
        applied = sign*int(lroundf(duty));
        return applied;
    }
} wheels[2];

// Original odometry and gyro-bias code (route-only copy from the supplied ZIP).
#define STATIC_ROUTE_ODOMETRY_ONLY
struct BiasEstimator {
    unsigned n=0;
    float sum=0, minimum=0, maximum=0;
    bool sample(float raw, bool moved) {
        if (moved || fabsf(raw)>8) { n=0; return false; }
        if (!n) { sum=0; minimum=maximum=raw; }
        minimum=fminf(minimum,raw); maximum=fmaxf(maximum,raw);
        if (maximum-minimum>1.8f) { n=0; return false; }
        sum+=raw;
        if (++n<180) return false;
        gyroBias=sum/n;
        return true;
    }
} bias;

#ifndef STATIC_ROUTE_ODOMETRY_ONLY
struct Ray { float x, y, dx, dy; };
Ray rayFrom(const Pose &p, int i) {
    const Mount &m=Mounts[i];
    const float u=cosf(p.h), v=sinf(p.h);
    return {p.x+m.forward*u-m.left*v,p.y+m.forward*v+m.left*u,
            cosf(p.h+m.yaw),sinf(p.h+m.yaw)};
}
#endif
float separation(const Pose &a, const Pose &b) {
    return hypotf(a.x-b.x,a.y-b.y);
}
float travelled=0, positionError=5, headingError=2*Rad, slipTime=0, lastRate=0;
struct TimedPose { Pose value; uint32_t at; };
TimedPose history[12]{};
uint8_t historyNext=0, historySize=0;
void rememberPose(uint32_t now) {
    history[historyNext]={pose,now};
    historyNext=(historyNext+1)%12;
    if (historySize<12) ++historySize;
}
Pose sampleOrigin(uint32_t at) {
    Pose result=pose;
    for (int i=0;i<historySize;++i) {
        const TimedPose &p=history[(historyNext+11-i)%12];
        result=p.value;
        if (int32_t(at-p.at)>=0) break;
    }
    return result;
}
void resetOdometry(Pose origin, uint32_t now) {
    pose=origin; travelled=0; encoderTotal[0]=encoderTotal[1]=0;
    slipTime=0; positionError=5; headingError=2*Rad; lastRate=yawRate;
    historySize=historyNext=0; rememberPose(now);
}
#ifndef STATIC_ROUTE_ODOMETRY_ONLY
bool predicting() {
    return phase==Phase::MANUAL_MAPPING || phase==Phase::TRACKING ||
           phase==Phase::OBSTACLE_AVOIDANCE;
}
#endif
void predict(int left, int right, float dt, uint32_t now) {
    const float dl=left/CountsPerMm[0], dr=right/CountsPerMm[1];
    const float ds=(dl+dr)*0.5f, encTurn=(dr-dl)/Track;
    const float gyroTurn=(yawRate+lastRate)*0.5f*dt;
    lastRate=yawRate;
    const float mismatch=fabsf(encTurn-gyroTurn);
    const bool slipping=mismatch>0.012f+0.55f*dt;
    slipTime=bound(slipTime+(slipping?dt:-0.45f*dt),0,1);
    const bool still=!left && !right && fabsf(yawRate)<0.45f*Rad;
    const float gyroWeight=slipping?0.96f:0.82f;
    const float dh=still?0:gyroWeight*gyroTurn+(1-gyroWeight)*encTurn;
    pose.x+=ds*cosf(pose.h+dh*0.5f);
    pose.y+=ds*sinf(pose.h+dh*0.5f);
    pose.h=wrap(pose.h+dh);
    travelled+=fabsf(ds);
    headingError=fminf(0.6f,headingError+0.003f*fabsf(dh)+0.00018f*dt+0.027f*mismatch);
    positionError=fminf(150,positionError+0.0035f*fabsf(ds)+0.12f*mismatch+0.012f*slipping);
    encoderTotal[0]+=left; encoderTotal[1]+=right;
    rememberPose(now);
}

#undef STATIC_ROUTE_ODOMETRY_ONLY

enum class RobotState { INITIALIZATION, WAIT_START, RUN_ROUTE, FINISH_BLINK, FAULT };
// ALIGN: pivot only when the start heading is far off. TRACK: continuous smooth
// following (corners and avoidance included). SETTLE: final stop at GOAL.
enum class LegPhase { ALIGN, TRACK, SETTLE };
enum class ImuRead { NewSample, NotReady, BusError };
RobotState state = RobotState::INITIALIZATION;
LegPhase legPhase = LegPhase::ALIGN;
uint32_t stateAt = 0, runAt = 0, legPhaseAt = 0, lastGoodAt = 0, lastLogAt = 0;
uint32_t lastProgressAt = 0, stillSince = 0, compareAt = 0;
uint16_t previousCount[2]{};
unsigned target = 1, runNumber = 0;  // target = next MAP node to pass
float bestProgress = 0, encoderTurn = 0, gyroTurn = 0, maxCross = 0;
float remaining = 0, cross = 0, angleError = 0, targetDistance = 0;
bool settlingStill = false;
const char *stopReason = "BOOT";
TxQueue tx;
#include "static_route/localize.h"

Path path;
unsigned pathSeg = 0;
float station = 0, offsetNow = 0, refCurv = 0;
uint32_t fixedSeen = 0;     // bit per fixed MAP box id already reported this run
uint32_t maxControlUs = 0;  // worst control() duration this run, measured on the target
constexpr float CoastDecelMm = 300.0f;  // assumed coast deceleration after a cut (measure it)
ObstacleMap obstacles;
Avoidance avoid[2];
uint32_t rangeSeenAt[RangeSensors::Count]{};
Shift shiftAt(float s) {
    const Shift a = avoid[0].at(s), b = avoid[1].at(s);  // zones never overlap
    return {a.d+b.d, a.slope+b.slope, a.curv+b.curv};
}
bool avoiding(float s) {
    for (const auto &a : avoid) if (a.active && s > a.s0 && s < a.s3) return true;
    return false;
}

struct Button {
    bool raw = false, armed = false;
    uint32_t changed = 0;
    bool pressed(bool down, uint32_t now) {
        if (down != raw) { raw = down; changed = now; }
        if (now-changed < 35) return false;
        if (!raw) { armed = true; return false; }
        if (!armed) return false;
        armed = false;
        return true;
    }
    void requireRelease(uint32_t now) { armed = false; changed = now; }
} button;

// Target adapter. The host tests substitute encoder/gyro data and a motor sink.
void cutDrive();
void writeMotors(int leftPwm, int rightPwm, uint32_t sampledAt);
// Cross-coupled wheel sync. lag[i] = counts the wheel should have made (nominal target,
// before the sync correction) minus counts it really made. If one motor is weaker (same PWM,
// fewer pulses) its lag grows relative to the other; the difference in mm is fed back to
// both targets (+ to the slow wheel, - to the fast one) BEFORE the car turns off its line.
struct WheelSync { float lag[2]{}, nominal[2]{}, maxMm = 0, peakMm = 0; uint32_t logAt = 0; } sync;
float syncErrorMm() { return sync.lag[0]/CountsPerMm[0] - sync.lag[1]/CountsPerMm[1]; }
void stopDrive(uint32_t now) {
    cutDrive();
    request = {};
    for (auto &wheel : wheels) wheel.stop(now);
    sync.lag[0] = sync.lag[1] = sync.nominal[0] = sync.nominal[1] = 0;
}
float speed(unsigned i) { return wheels[i].cps/CountsPerMm[i]; }

int rangeLog(unsigned i, uint32_t now) {
    const RangeSensors::Reading &r = RangeSensors::get(i);
    return RangeSensors::fresh(i, now) && r.mm != RangeSensors::NoTarget ? int(r.mm) : -1;
}
const char *phaseName() {
    if (legPhase == LegPhase::ALIGN) return "ALIGN";
    if (legPhase == LegPhase::SETTLE) return "SETTLE";
    return avoiding(station) ? "AVOID" : "TRACK";
}
constexpr char CsvColumns[] =
    "TIME_MS,X_MM,Y_MM,YAW_DEG,WAYPOINT,LEFT_SPEED,RIGHT_SPEED,HEADING_ERROR,CROSS_TRACK_ERROR,"
    "TARGET_CPS_L,TARGET_CPS_R,PWM_L,PWM_R,PHASE,ACTUAL_CPS_L,ACTUAL_CPS_R,"
    "S_MM,OFFSET_MM,RANGE0,RANGE1,RANGE2,RANGE3\r\n";
// Hand calibration ($CAL,START / $CAL,STOP): encoder ticks (sign-corrected) and gyro angle.
struct Calibration { bool active = false; int32_t ticks[2]{}; float yaw = 0; uint32_t startedAt = 0, logAt = 0; } cal;
uint32_t lastControlAt = 0;  // newest control time, used as "now" by commands
uint32_t millis32() { return lastControlAt; }
void logCal(const char *tag, uint32_t now) {
    char line[80];
    snprintf(line, sizeof(line), "#%s,%lu,%ld,%ld,%s\r\n", tag, (unsigned long)(now-cal.startedAt),
             (long)cal.ticks[0], (long)cal.ticks[1], Decimal(cal.yaw/Rad).text);
    tx.push(line);
}
void logPose(uint32_t now, bool periodic) {
    char line[260];
    snprintf(line, sizeof(line), "%lu,%s,%s,%s,%u,%s,%s,%s,%s,%s,%s,%d,%d,%s,%s,%s,%s,%s,%d,%d,%d,%d\r\n",
             (unsigned long)(now-runAt), Decimal(pose.x).text, Decimal(pose.y).text,
             Decimal(pose.h/Rad).text, target, Decimal(speed(0)).text,
             Decimal(speed(1)).text, Decimal(angleError/Rad).text, Decimal(cross).text,
             Decimal(wheels[0].targetCps).text, Decimal(wheels[1].targetCps).text,
             wheels[0].applied, wheels[1].applied, phaseName(),
             Decimal(wheels[0].cps).text, Decimal(wheels[1].cps).text,
             Decimal(station).text, Decimal(offsetNow).text,
             rangeLog(0, now), rangeLog(1, now), rangeLog(2, now), rangeLog(3, now));
    tx.push(line, periodic);
}
void logPath() {
    char line[96];
    for (unsigned i = 0; i < path.count; ++i) {
        const Seg &g = path.seg[i];
        snprintf(line, sizeof(line), "#PATH_SEG,%u,%s,%s,%s,%s,%s\r\n", i, Decimal(g.x0).text,
                 Decimal(g.y0).text, Decimal(g.h0/Rad).text, Decimal(g.len).text,
                 Decimal(g.k ? 1/g.k : 0).text);
        tx.push(line);
    }
}
void logObstacle(const Obstacle &o, float s, float lat) {
    char line[96];
    snprintf(line, sizeof(line), "#OBSTACLE,%u,%s,%s,%s,%s,%s\r\n", unsigned(o.id),
             o.kind == 'F' ? "FIXED" : o.kind == 'R' ? "RANDOM" : "UNKNOWN",
             Decimal(o.x).text, Decimal(o.y).text, Decimal(s).text, Decimal(lat).text);
    tx.push(line);
}
void logAvoid(const Avoidance &a) {
    char line[110];
    snprintf(line, sizeof(line), "#AVOID,%u,%s,%s,%s,%s,%s,%s\r\n", unsigned(a.id),
             a.offset > 0 ? "LEFT" : "RIGHT", Decimal(a.offset).text, Decimal(a.s0).text,
             Decimal(a.s1).text, Decimal(a.s2).text, Decimal(a.s3).text);
    tx.push(line);
}
void waypointEvent(const char *name, unsigned node, uint32_t now) {
    char line[120];
    snprintf(line, sizeof(line), "#WAYPOINT_%s,%u,%lu,%d,%d,%s,%s,%s,%s\r\n",
             name, node, (unsigned long)(now-runAt), int(ROUTE[node].x), int(ROUTE[node].y),
             Decimal(pose.x).text, Decimal(pose.y).text, Decimal(pose.h/Rad).text,
             Decimal(travelled).text);
    tx.push(line);
}
float goalError() { return hypotf(GOAL_X-pose.x, GOAL_Y-pose.y); }
void endReport(const char *event, const char *why, uint32_t now) {
    char line[420];
    logPose(now, false);
    snprintf(line, sizeof(line),
             "#%s\r\n#TOTAL_TIME_MS,%lu\r\n#TOTAL_DISTANCE_MM,%s\r\n"
             "#FINAL_X_MM,%s\r\n#FINAL_Y_MM,%s\r\n#FINAL_YAW_DEG,%s\r\n"
             "#GOAL_ERROR_MM,%s\r\n#MAX_CROSS_TRACK_ERROR_MM,%s\r\n"
             "#DROPPED_SAMPLES,%lu\r\n#DROPPED_EVENTS,%lu\r\n#MAX_CONTROL_US,%lu\r\n#MAX_SYNC_MM,%s\r\n#STOP_REASON,%s\r\n",
             event, (unsigned long)(now-runAt), Decimal(travelled).text,
             Decimal(pose.x).text, Decimal(pose.y).text, Decimal(pose.h/Rad).text,
             Decimal(goalError()).text, Decimal(maxCross).text, (unsigned long)tx.droppedSamples,
             (unsigned long)tx.droppedEvents, (unsigned long)maxControlUs, Decimal(sync.maxMm).text, why);
    tx.push(line);
}
void waitStart(uint32_t now) {
    state = RobotState::WAIT_START;
    stateAt = now;
    button.requireRelease(now);
    tx.push("#WAIT_START\r\n");
}
void dumpParams();
void logState();
void fault(const char *why, uint32_t now) {
    if (state == RobotState::FAULT) return;
    stopDrive(now);
    const bool wasRunning = state == RobotState::RUN_ROUTE;
    state = RobotState::FAULT;
    stateAt = now;
    stopReason = why;
    if (wasRunning) endReport("ROUTE_FAULT", why, now);
    else {
        char line[85];
        snprintf(line, sizeof(line), "#FAULT\r\n#STOP_REASON,%s\r\n", why);
        tx.push(line);
    }
}

void setLegPhase(LegPhase next, uint32_t now) {
    legPhase = next;
    legPhaseAt = lastProgressAt = now;
    bestProgress = 1.0e6f;
    settlingStill = false;
}
// Errors against the (possibly shifted) smooth reference at the car's station.
void updateErrors() {
    float lateral = 0;
    station = path.track(pathSeg, pose.x, pose.y, lateral);
    for (auto &a : avoid) {
        // Commit once the car enters the ramp or the box is DecideMm away: no flip-flopping
        // near it. The committed (final) plan is logged again for the UART observer.
        Obstacle &o = obstacles.items[a.obstacle];
        if (a.active && !o.frozen && (station >= a.s0 || 0.5f*(a.s1+a.s2) - station <= DecideMm)) {
            o.frozen = true;
            logAvoid(a);
        }
        if (a.active && station > a.s3) a.active = false;
    }
    const PathPoint ref = path.at(station);
    const Shift sh = shiftAt(station);
    offsetNow = sh.d;
    cross = lateral - sh.d;  // positive: car is left of the reference
    refCurv = shiftedCurv(ref.k, sh);  // exact, also on arcs (see avoid.h)
    angleError = wrap(shiftedHeading(ref.h, ref.k, sh) - pose.h);
    remaining = path.length - station;
    targetDistance = hypotf(pose.x-GOAL_X, pose.y-GOAL_Y);
    maxCross = fmaxf(maxCross, fabsf(cross));
}
// Nodes are passed on the move: log END/START when the station crosses each arc midpoint.
void passNodes(uint32_t now) {
    while (target+1 < ROUTE_COUNT && station >= path.nodeS[target]) {
        waypointEvent("END", target, now);
        ++target;
        waypointEvent("START", target, now);
    }
}
void startRun(uint32_t now, uint16_t left, uint16_t right) {
    stopDrive(now);
    for (auto &wheel : wheels) wheel = Wheel{};
    previousCount[0] = left; previousCount[1] = right;
    const StartFix place = tune.startFix > 0.5f ? fitStart() : StartFix{StartPose, 0, 0, "OFF"};
    resetOdometry(place.pose, now);
    rangeFix = RangeFix{};
    sync = WheelSync{}; sync.logAt = now;
    path.build();
    pathSeg = 0; obstacles.clear(); fixedSeen = 0; maxControlUs = 0;
    for (auto &a : avoid) a = Avoidance{};
    for (unsigned i = 0; i < RangeSensors::Count; ++i) rangeSeenAt[i] = RangeSensors::get(i).at;
    target = 1; maxCross = encoderTurn = gyroTurn = 0;
    runAt = stateAt = lastGoodAt = compareAt = lastLogAt = now;
    tx.droppedSamples = tx.droppedEvents = 0;
    cal.active = false;
    state = RobotState::RUN_ROUTE; stopReason = "RUNNING";
    char line[48];
    snprintf(line, sizeof(line), "#ROUTE_STARTED,%u\r\n", ++runNumber);
    tx.push(line);
    dumpParams();  // the log records the parameters this run used
    logStartFix("START_POSE", place);
    logPath();
    tx.push(CsvColumns);
    waypointEvent("START", 0, now); waypointEvent("END", 0, now); waypointEvent("START", 1, now);
    updateErrors();
    setLegPhase(fabsf(angleError) > AlignStartDeg*Rad ? LegPhase::ALIGN : LegPhase::TRACK, now);
    logPose(now, false);
}
// ---- Commands from the web tool or a terminal (UART or HC-05 Bluetooth, same USART1) ----
// One ASCII line each, '\n' or '\r' terminated. Replies start with '#'.
//   $PING                -> #PONG,<state>
//   $GET                 -> #PARAM,NAME,VALUE,MIN,MAX for every parameter, then #STATE
//   $SET,NAME,VALUE      -> #PARAM,... (refused while the route runs: #ERR,BUSY)
//   $DEFAULTS            -> restore build defaults, then the #PARAM list
//   $START / $STOP       -> same as pressing SW1 to start / stop (#ACK or #ERR,STATE)
//   $REBOOT              -> software reset (not while running)
//   $CAL,START / $CAL,STOP -> hand calibration while waiting: #CAL / #CAL_DONE,ms,ticksL,ticksR,yawDeg
enum class Remote : uint8_t { None, Start, Stop };
Remote remote = Remote::None;
bool rebootRequested = false;
const char *stateName() {
    switch (state) {
        case RobotState::INITIALIZATION: return "INIT";
        case RobotState::WAIT_START: return "WAIT_START";
        case RobotState::RUN_ROUTE: return "RUN_ROUTE";
        case RobotState::FINISH_BLINK: return "FINISH";
        default: return "FAULT";
    }
}
void logState() {
    char line[64];
    snprintf(line, sizeof(line), "#STATE,%s,%s\r\n", stateName(), stopReason);
    tx.push(line);
}
void logParam(const TuneParam &p) {
    char line[80];
    snprintf(line, sizeof(line), "#PARAM,%s,%s,%s,%s\r\n", p.name, Number(tune.*p.field).text,
             Number(p.lo).text, Number(p.hi).text);
    tx.push(line);
}
void dumpParams() { for (const auto &p : TuneTable) logParam(p); }
void reply(const char *tag, const char *what) {
    char line[64];
    snprintf(line, sizeof(line), "#%s,%s\r\n", tag, what);
    tx.push(line);
}
// Small decimal parser ("-12.5", "0.00028", "2.8e-4"). newlib's strtof costs ~16 KB of flash.
bool parseNumber(const char *s, float &out) {
    if (!s) return false;
    bool negative = *s == '-';
    if (*s == '-' || *s == '+') ++s;
    float v = 0, scale = 1;
    bool digits = false, fraction = false;
    for (; *s; ++s) {
        if (*s >= '0' && *s <= '9') {
            digits = true;
            if (fraction) { scale *= 0.1f; v += (*s-'0')*scale; } else v = v*10 + (*s-'0');
        } else if (*s == '.' && !fraction) fraction = true;
        else break;
    }
    if (digits && (*s == 'e' || *s == 'E')) {
        ++s;
        const bool down = *s == '-';
        if (*s == '-' || *s == '+') ++s;
        int e = 0;
        if (*s < '0' || *s > '9') return false;
        while (*s >= '0' && *s <= '9' && e < 40) e = e*10 + (*s++ - '0');
        while (e--) v = down ? v*0.1f : v*10;
    }
    while (*s == ' ') ++s;
    if (!digits || *s) return false;
    out = negative ? -v : v;
    return true;
}
void handleCommand(char *line) {
    if (line[0] != '$') { reply("ERR", "FORMAT"); return; }
    char *cmd = line+1, *name = strchr(cmd, ','), *value = nullptr;
    if (name) { *name++ = 0; value = strchr(name, ','); if (value) *value++ = 0; }
    for (char *c = cmd; *c; ++c) if (*c >= 'a' && *c <= 'z') *c = char(*c-'a'+'A');
    const bool running = state == RobotState::RUN_ROUTE;
    if (!strcmp(cmd, "PING")) reply("PONG", stateName());
    else if (!strcmp(cmd, "GET")) { dumpParams(); logState(); }
    else if (!strcmp(cmd, "SET")) {
        if (running) { reply("ERR", "BUSY"); return; }
        const TuneParam *p = nullptr;
        for (const auto &t : TuneTable) if (name && !strcmp(name, t.name)) p = &t;
        if (!p) { reply("ERR", "UNKNOWN_PARAM"); return; }
        float v = 0;
        if (!parseNumber(value, v) || !isfinite(v) || v < p->lo || v > p->hi) { reply("ERR", "RANGE"); return; }
        tune.*(p->field) = v;
        updateClearances();
        logParam(*p);
    } else if (!strcmp(cmd, "DEFAULTS")) {
        if (running) { reply("ERR", "BUSY"); return; }
        tune = DefaultTuning; updateClearances(); dumpParams();
    } else if (!strcmp(cmd, "START")) {
        if (state != RobotState::WAIT_START) { reply("ERR", "STATE"); return; }
        remote = Remote::Start; reply("ACK", "START");
    } else if (!strcmp(cmd, "STOP")) {
        if (!running) { reply("ERR", "STATE"); return; }
        remote = Remote::Stop; reply("ACK", "STOP");
    } else if (!strcmp(cmd, "CAL")) {
        // Wheel calibration with the motors off: push the car straight by hand over a
        // measured distance, or turn it in place by hand. Only while waiting for START.
        if (name && !strcmp(name, "START")) {
            if (state != RobotState::WAIT_START) { reply("ERR", "STATE"); return; }
            cal = Calibration{}; cal.active = true; cal.startedAt = cal.logAt = millis32();
            reply("ACK", "CAL_START");
        } else if (name && !strcmp(name, "STOP")) {
            if (!cal.active) { reply("ERR", "STATE"); return; }
            cal.active = false; logCal("CAL_DONE", millis32());
        } else reply("ERR", "FORMAT");
    } else if (!strcmp(cmd, "REBOOT")) {
        if (running) { reply("ERR", "BUSY"); return; }
        rebootRequested = true; reply("ACK", "REBOOT");
    } else reply("ERR", "UNKNOWN_COMMAND");
}
constexpr uint32_t FinishHoldMs = 3000;
uint32_t placeLogAt = 0;
void serviceUi(bool down, uint32_t now, uint16_t left, uint16_t right) {
    const bool press = button.pressed(down, now);
    const Remote r = remote;
    remote = Remote::None;
    if (state == RobotState::FINISH_BLINK && now-stateAt >= FinishHoldMs) {
        waitStart(now);
        return;
    }
    const bool start = (press || r == Remote::Start) && state == RobotState::WAIT_START;
    const bool stop = (press || r == Remote::Stop) && state == RobotState::RUN_ROUTE;
    if (start) startRun(now, left, right);
    else if (stop) {
        stopDrive(now);
        stopReason = press ? "MANUAL_SW1" : "REMOTE_STOP";
        endReport("ROUTE_STOPPED", stopReason, now);
        waitStart(now);
    }
}
// LED: blinks while waiting for SW1, solid while running, stays solid FinishHoldMs
// after the route ends, then the car resets to WAIT_START (blinking again).
bool ledOn(uint32_t now) {
    const uint32_t age = now-stateAt;
    switch (state) {
        case RobotState::WAIT_START: return (age/250)%2 == 0;
        case RobotState::RUN_ROUTE: return true;
        case RobotState::FINISH_BLINK: return true;
        case RobotState::FAULT: return (age/100)%2 == 0;
        default: return false;
    }
}
float slew(float old, float next, float rise, float fall, float dt) {
    return old + bound(next-old, -fall*dt, rise*dt);
}
bool advancing(float metric, float step, uint32_t now) {
    if (metric < bestProgress-step) { bestProgress = metric; lastProgressAt = now; }
    return now-lastProgressAt < 4500;
}
// Fastest speed allowed at station s with curvature k (lateral accel + per-wheel CPS cap).
float curveSpeed(float k, float s) {
    const float a = fabsf(k);
    const float cps = avoiding(s) ? fminf(AvoidCps, CruiseCps) : CruiseCps;
    const float wheelCap = cps/MaxCountsPerMm/(1 + 0.5f*Track*a);
    return a < 1.0e-5f ? wheelCap : fminf(wheelCap, sqrtf(LatAccelMm/a));
}
// Look ahead along the path so the car slows BEFORE a corner or an avoidance bend.
float plannedSpeed() {
    float v = curveSpeed(refCurv, station);
    for (float ds = 25; ds <= 350 && station+ds < path.length; ds += 25) {
        const float s = station+ds;
        const float cap = curveSpeed(shiftedCurv(path.curvature(s), shiftAt(s)), s);
        v = fminf(v, sqrtf(cap*cap + 2*PlanDecel*ds));
    }
    // GOAL: same approach/brake profile as the original stop-and-go test.
    const float brake = sqrtf(2*RouteDecel*fmaxf(0, remaining - BRAKE_AT_MM*0.5f));
    const float approach = fminf(NEAR_SPEED_MM_S, ApproachCps/MaxCountsPerMm) + fmaxf(0, remaining-NEAR_DISTANCE_MM)*0.9f;
    v = fminf(v, fminf(approach, brake));
    return v*bound(1-fabsf(angleError)/(60*Rad), 0.25f, 1);
}
void follow(float dt, uint32_t now) {
    updateErrors();
    if (legPhase == LegPhase::SETTLE) {
        request = {};
        const bool still = fabsf(speed(0)) <= STILL_MM_S && fabsf(speed(1)) <= STILL_MM_S
                           && fabsf(yawRate) <= STILL_DEG_S*Rad;
        if (!still) settlingStill = false;
        else if (!settlingStill) { stillSince = now; settlingStill = true; }
        if (now-legPhaseAt > 1800) { fault("NOT_SETTLED", now); return; }
        if (!settlingStill || now-stillSince < SETTLE_MS) return;
        if (targetDistance > POSITION_TOL_MM) { fault("MISSED_WAYPOINT", now); return; }
        waypointEvent("END", target, now);
        stopDrive(now); stopReason = "GOAL_REACHED";
        state = RobotState::FINISH_BLINK; stateAt = now;
        endReport("ROUTE_FINISHED", stopReason, now);
        return;
    }
    if (legPhase == LegPhase::ALIGN) {  // only used when the start heading is far off
        const float turnCap = fminf(TURN_RAD_S, 2.0f*TurnCps/(Track*MaxCountsPerMm));
        const float turn = bound(3.4f*angleError - 0.15f*yawRate, -turnCap, turnCap);
        if (!advancing(fabsf(angleError), Rad, now) || now-legPhaseAt > 10000) {
            fault("TURN_NO_PROGRESS", now); return;
        }
        request.v = 0;
        const bool aligned = fabsf(angleError) <= ANGLE_TOL_DEG*Rad;
        request.w = slew(request.w, aligned ? 0 : turn, ANG_ACCEL_RAD_S2, ANG_ACCEL_RAD_S2, dt);
        if (aligned && fabsf(yawRate) <= STILL_DEG_S*Rad && fabsf(request.w) < 0.001f
            && fabsf(speed(0)) <= STILL_MM_S && fabsf(speed(1)) <= STILL_MM_S)
            setLegPhase(LegPhase::TRACK, now);
        return;
    }
    passNodes(now);
    // Aim inside the GOAL acceptance disk to leave room for wheel coast.
    if (remaining <= BRAKE_AT_MM && targetDistance <= POSITION_TOL_MM) {
        stopDrive(now);
        setLegPhase(LegPhase::SETTLE, now);
        return;
    }
    if (remaining <= 0) { fault("MISSED_WAYPOINT", now); return; }
    if (fabsf(cross) > OffPathMm || fabsf(angleError) > 75*Rad) { fault("OFF_PATH", now); return; }
    if (!advancing(remaining, 3.0f, now)) { fault("NO_TRACK_PROGRESS", now); return; }
    request.v = slew(request.v, plannedSpeed(), RouteAccel, RouteDecel, dt);
    // Curvature command: path + avoidance feedforward, lateral and heading feedback.
    const float curv = bound(refCurv - LateralGain*bound(cross, -LateralSatMm, LateralSatMm)
                             + HeadingGain*sinf(angleError), -MaxCurv, MaxCurv);
    // Limit curvature during translation: neither wheel receives a reverse setpoint.
    const float wLimit = 1.7f*request.v/Track;
    request.w = bound(slew(request.w, request.v*curv, ANG_ACCEL_RAD_S2, ANG_ACCEL_RAD_S2, dt),
                      -wLimit, wLimit);
}
void actuate(uint32_t now, float dt) {
    if (state != RobotState::RUN_ROUTE || (!request.v && !request.w)) { stopDrive(now); return; }
    const float cap = legPhase == LegPhase::ALIGN ? TurnCps :
        ((remaining <= NEAR_DISTANCE_MM || targetDistance <= NEAR_DISTANCE_MM) ? ApproachCps :
         avoiding(station) ? fminf(AvoidCps, CruiseCps) : CruiseCps);
    // Scale both targets together: preserve curvature and avoid exceeding per-wheel CPS limit.
    const float halfDiff = 0.5f*Track*request.w;
    const float peak = fmaxf(fabsf((request.v-halfDiff)*CountsPerMm[0]),
                             fabsf((request.v+halfDiff)*CountsPerMm[1]));
    if (peak > cap) { request.v *= cap/peak; request.w *= cap/peak; }
    const float diff = 0.5f*Track*request.w;
    sync.nominal[0] = (request.v-diff)*CountsPerMm[0];
    sync.nominal[1] = (request.v+diff)*CountsPerMm[1];
    // Correction only while both wheels drive the same way (not while pivoting in ALIGN).
    // Limited to 30 % of the slower wheel's speed: a stalled wheel must not drag the other
    // one to a halt (the stall guard has to blame the right wheel).
    const float lim = fminf(60.0f, 0.3f*fminf(fabsf(request.v-diff), fabsf(request.v+diff)));
    const float corr = legPhase == LegPhase::TRACK ? bound(tune.syncGain*syncErrorMm(), -lim, lim) : 0;
    const int left = wheels[0].output(sync.nominal[0] + corr*CountsPerMm[0], 0, dt, now);
    const int right = wheels[1].output(sync.nominal[1] - corr*CountsPerMm[1], 1, dt, now);
    writeMotors(left, right, now);
}
int delta16(uint16_t current, uint16_t old) {
    const unsigned d = uint16_t(current-old);
    return d < 32768 ? int(d) : int(d)-65536;
}

// A newly confirmed obstacle: report where it is relative to the path.
void announce(const Obstacle &o) {
    float s = -1, lat = 0;
    const bool near = path.nearest(o.x, o.y, station, station+LookAheadMm, s, lat);
    logObstacle(o, near ? s : -1, near ? lat : 0);
}
// Every confirmed box that blocks the reference ahead gets a smooth sidestep. An early,
// noisy estimate may not fit yet: retry as it firms up, give up only at DecideMm.
// Until the car enters a planned detour, the plan follows the refined estimate (the
// reference is unchanged at the car, so replacing it is seamless).
// Fixed boxes never block: the filleted path and every detour are checked against them.
// At most one plan is computed per call (every 50 ms) to bound the control step.
uint32_t reviewAt = 0;
void reviewObstacles(uint32_t now) {
    if (now-reviewAt < 50) return;
    reviewAt = now;
    for (unsigned i = 0; i < obstacles.count && state == RobotState::RUN_ROUTE; ++i) {
        Obstacle &o = obstacles.items[i];
        if (!o.confirmed || o.frozen || (o.kind != 'U' && o.kind != 'R')) continue;
        Avoidance *own = nullptr;
        for (auto &a : avoid) if (a.active && a.obstacle == i) own = &a;
        if (own && station >= own->s0) { o.frozen = true; continue; }  // already inside it
        float s = -1, lat = 0;
        if (!path.nearest(o.x, o.y, station, station+LookAheadMm, s, lat) || s <= station+1.0f)
            continue;                                     // behind the car / outside look-ahead
        // Zones never overlap, so without its own plan the reference at s is unshifted.
        if (fabsf(lat - (own ? 0 : shiftAt(s).d)) >= BlockMm) {
            if (own) { own->active = false; o.planned = false; }  // refined: it no longer blocks
            continue;
        }
        Avoidance *slot = own ? own : !avoid[0].active ? &avoid[0] : !avoid[1].active ? &avoid[1] : nullptr;
        // A re-plan keeps its ramp start (still ahead of the car), so replacing it is seamless.
        float from = own ? own->s0 : station + 10.0f;
        for (const auto &a : avoid) if (a.active && &a != own) from = fmaxf(from, a.s3);
        Avoidance plan;
        const PlanResult result = slot ? planAvoid(path, obstacles, o.id, from, s, lat, plan)
                                       : PlanResult::NoRoom;
        if (result == PlanResult::Ok) {
            plan.obstacle = uint8_t(i);
            *slot = plan; o.planned = true;
            if (!own) logAvoid(plan);  // first plan; the committed one is logged on freeze
            return;
        }
        // A refined estimate that would need a longer ramp: keep the earlier plan only if
        // its remaining part still clears the box where it is now believed to be.
        float worst = 0;
        if (own && result == PlanResult::TooClose &&
            checkDetour(path, obstacles, *own, fmaxf(station, own->s0), own->s3, worst)) return;
        if (own) { own->active = false; o.planned = false; }
        if (result == PlanResult::NoRoom && s-station > DecideMm) return;  // not yet: re-estimate
        const bool late = result == PlanResult::TooClose;
        char line[48];
        snprintf(line, sizeof(line), "#AVOID_FAIL,%u,%s\r\n", unsigned(o.id), late ? "TOO_CLOSE" : "NO_ROOM");
        tx.push(line);
        fault(late ? "OBSTACLE_TOO_CLOSE" : "NO_AVOID_PATH", now);
        return;
    }
}
// Turn each new VL53L0X sample into a world point using the pose at the sample time.
void scanRanges(uint32_t now) {
#ifndef STATIC_ROUTE_NO_RANGE
    for (unsigned i = 0; i < RangeSensors::Count && state == RobotState::RUN_ROUTE; ++i) {
        if (!RangeSensors::fresh(i, now)) { fault("RANGE_STALE", now); return; }
        const RangeSensors::Reading &r = RangeSensors::get(i);
        if (r.at == rangeSeenAt[i]) continue;
        rangeSeenAt[i] = r.at;
        if (r.mm == RangeSensors::NoTarget || r.mm > DetectMm) continue;
        const Pose p = sampleOrigin(r.at);
        const Mount &m = Mounts[i];
        const float c = cosf(p.h), sn = sinf(p.h), dx = cosf(p.h+m.yaw), dy = sinf(p.h+m.yaw);
        const float hx = p.x + m.forward*c - m.left*sn + r.mm*dx;
        const float hy = p.y + m.forward*sn + m.left*c + r.mm*dy;
        const bool inside = hx >= -30 && hy >= -30 && hx <= StaticMap::MAP_WIDTH+30 && hy <= StaticMap::MAP_HEIGHT+30;
        const Obstacle seen = obstacles.classify(hx, hy, dx, dy, now);
        // Last-resort stop for the straight beams. For anything not on the MAP the distance
        // grows with speed, since the car coasts after the cut (CoastDecelMm: measure it).
        // MAP boxes keep the fixed 90 mm: the path is verified clear of them, and the beams
        // sweep past them at short range while rounding a corner.
        const float v = fmaxf(0, 0.5f*(speed(0)+speed(1)));
        const bool known = inside && (seen.kind == 'F' || seen.kind == 'R');
        const float stopMm = known ? HardStopMm : HardStopMm + v*v/(2*CoastDecelMm) + v*0.06f;
        if ((i == 1 || i == 2) && r.mm < stopMm) { fault("OBSTACLE_STOP", now); return; }
        if (!inside) continue;  // something outside the taped map
        if (seen.kind == 'F') {  // fixed MAP box: reported once per run, never stored
            if (tune.rangeFix > 0.5f) fixFromBeam(p, i, r.mm, seen.id);
            const uint32_t bit = 1UL << (seen.id & 31);
            if (!(fixedSeen & bit)) { fixedSeen |= bit; announce(seen); }
            continue;
        }
        if (Obstacle *o = obstacles.note(seen, hx, hy, now)) announce(*o);
        if (obstacles.overflow) { tx.push("#AVOID_FAIL,0,MAP_FULL\r\n"); fault("OBSTACLE_MAP_FULL", now); return; }
    }
    reviewObstacles(now);
#else
    (void)now;
#endif
}
void control(uint32_t now, uint16_t left, uint16_t right, ImuRead imu, float rawDps, bool tickSeen) {
    if (state == RobotState::FAULT) return;
    const uint32_t elapsed = now-lastGoodAt;
    if (state == RobotState::RUN_ROUTE && elapsed > CONTROL_TIMEOUT_MS) {
        fault("CONTROL_TIMEOUT", now); return;
    }
    if (imu == ImuRead::BusError) { fault("MPU_READ", now); return; }
    if (elapsed > IMU_STALE_MS) { fault("MPU_STALE", now); return; }
    if (imu == ImuRead::NotReady || !elapsed) return;
    if (!isfinite(rawDps) || fabsf(rawDps) >= 245.0f) { fault("MPU_INVALID", now); return; }
    lastGoodAt = lastControlAt = now;
    const float dt = elapsed*0.001f;
    const int ticks[] = {EncoderSign[0]*delta16(left, previousCount[0]),
                         EncoderSign[1]*delta16(right, previousCount[1])};
    previousCount[0] = left; previousCount[1] = right;
    yawRate = (rawDps-gyroBias)*Rad*GyroSign*GyroScale*tune.gyroScale;
    if (state == RobotState::INITIALIZATION) {
        startBeams.sample(now);
        if (bias.sample(rawDps, ticks[0] || ticks[1])) {
            if (!tickSeen) fault("SYSTICK_GUARD", now);
            else waitStart(now);
        } else if (now-stateAt >= 12000) fault("GYRO_NOT_STILL", now);
        return;
    }
    if (state == RobotState::WAIT_START) {
        // Car must be still while the START beams are averaged (hand moves it: start over).
        if (ticks[0] || ticks[1] || fabsf(yawRate) > 2*Rad || cal.active) startBeams.clear();
        else startBeams.sample(now);
        if (!cal.active && tune.startFix > 0.5f && now-placeLogAt >= 1000) {
            placeLogAt = now;
            logStartFix("PLACE", fitStart());  // live placement error while waiting for SW1
        }
    }
    if (state == RobotState::WAIT_START && cal.active) {  // hand-push / hand-turn calibration
        cal.ticks[0] += ticks[0]; cal.ticks[1] += ticks[1];
        cal.yaw += yawRate*dt;
        if (now-cal.logAt >= 100) { cal.logAt = now; logCal("CAL", now); }
    }
    if (state != RobotState::RUN_ROUTE) return;
    for (unsigned i=0; i<2; ++i) {
        if (fabsf(float(ticks[i])) > 8.0f + 8000.0f*dt) { fault("ENCODER_JUMP", now); return; }
        if (!wheels[i].observe(ticks[i], dt, now)) {
            fault(i ? "WHEEL_R_STALL_DIR" : "WHEEL_L_STALL_DIR", now); return;
        }
    }
    for (unsigned i=0; i<2; ++i)
        sync.lag[i] = legPhase != LegPhase::TRACK ? 0 : bound(sync.lag[i] + sync.nominal[i]*dt - ticks[i], -400.0f, 400.0f);
    sync.peakMm = fmaxf(sync.peakMm, fabsf(syncErrorMm()));
    sync.maxMm = fmaxf(sync.maxMm, sync.peakMm);
    if (now-sync.logAt >= 1000) {  // worst wheel lag difference of the last second
        char line[40];
        snprintf(line, sizeof(line), "#SYNC,%s\r\n", Decimal(sync.peakMm).text);
        tx.push(line);
        sync.logAt = now; sync.peakMm = 0;
    }
    encoderTurn += (ticks[1]/CountsPerMm[1]-ticks[0]/CountsPerMm[0])/Track;
    gyroTurn += (yawRate+lastRate)*0.5f*dt;
    predict(ticks[0], ticks[1], dt, now);
    if (slipTime > 0.22f) { fault("WHEEL_SLIP", now); return; }
    if (now-compareAt >= 450) {
        if (fabsf(encoderTurn-gyroTurn) > 8*Rad + 0.20f*fabsf(encoderTurn)) {
            fault("GYRO_ENCODER_DISAGREE", now); return;
        }
        compareAt = now; encoderTurn = gyroTurn = 0;
    }
    updateErrors();  // station from this cycle's pose before obstacles are reviewed
    scanRanges(now);
    if (state != RobotState::RUN_ROUTE) return;
    follow(dt, now);
    actuate(now, dt);
    logRangeFix(now);
    if (state == RobotState::RUN_ROUTE && now-lastLogAt >= LOG_MS) {
        logPose(now, true); lastLogAt = now;
    }
}

#ifndef STATIC_ROUTE_HOST_TEST
#include "navigation/board_pins.h"
uint8_t mpu = 0;
#include "navigation/encoder_imu.h"
volatile bool armed = false, tripped = false, tickSeen = false;
volatile uint32_t guardAt = 0;
bool pwmWritten = false;
int lastMotorSign[2]{};
uint32_t lastPollAt = 0;
extern "C" void HAL_SYSTICK_Callback(void) {
    tickSeen = true;
    if (armed && HAL_GetTick()-guardAt > CONTROL_TIMEOUT_MS) {
        GPIOB->BRR = 1U; // TB6612 STBY on PB0; works even if loop is stuck in I2C.
        armed = false;
        tripped = true;
    }
}
void cutDrive() {
    GPIOB->BRR = 1U;
    armed = false;
    if (pwmWritten) {
        analogWrite(Pwm[0], 0); analogWrite(Pwm[1], 0);
        pwmWritten = false;
    }
    lastMotorSign[0] = lastMotorSign[1] = 0;
}
void writeMotors(int leftPwm, int rightPwm, uint32_t sampledAt) {
    const int duty[] = {leftPwm, rightPwm};
    if (tripped || (!leftPwm && !rightPwm)) { cutDrive(); return; }
    for (int i=0; i<2; ++i) {
        const int sign = (duty[i] > 0) - (duty[i] < 0);
        if (sign != lastMotorSign[i]) analogWrite(Pwm[i], 0);
        lastMotorSign[i] = sign;
        const bool high1 = duty[i] && ((duty[i] > 0) == (i == 1));
        digitalWrite(In1[i], high1 ? HIGH : LOW);
        digitalWrite(In2[i], duty[i] && !high1 ? HIGH : LOW);
        analogWrite(Pwm[i], abs(duty[i]));
    }
    pwmWritten = true;
    const uint32_t flags = __get_PRIMASK();
    __disable_irq();
    if (HAL_GetTick()-sampledAt > CONTROL_TIMEOUT_MS) tripped = true;
    guardAt = sampledAt; armed = !tripped;
    digitalWrite(PB0, armed ? HIGH : LOW);
    __set_PRIMASK(flags);
}
ImuRead readGyro(float &rawDps) {
    // INT_STATUS then the 14-byte accel/temperature/gyro burst. One trailing
    // byte keeps GYRO_ZOUT_L out of the last-byte position on the STM32F1.
    uint8_t data[16]{};
    if (!readRegs(Wire, mpu, 0x3a, data, sizeof(data))) return ImuRead::BusError;
    if (!(data[0] & 1U)) return ImuRead::NotReady;
    const uint16_t word = (uint16_t(data[13]) << 8) | data[14];
    rawDps = int16_t(word)/131.0f;
    return ImuRead::NewSample;
}
void lowOutput(uint32_t pin) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
void beginHardware() {
    lowOutput(PB0);
    digitalWrite(PC13, HIGH); pinMode(PC13, OUTPUT);
    pinMode(SwitchPin, INPUT_PULLUP);
    analogWriteResolution(8);
    for (unsigned i=0; i<2; ++i) {
        lowOutput(In1[i]); lowOutput(In2[i]); lowOutput(Pwm[i]); analogWrite(Pwm[i], 0);
    }
    for (uint32_t pin : Shut) lowOutput(pin);
    // USART1 PA9/PA10: USB-UART adapter (115200) or HC-05 Bluetooth (build with -DSTATIC_ROUTE_BAUD=38400).
    Serial.setRx(PA10); Serial.setTx(PA9); Serial.begin(STATIC_ROUTE_BAUD, SERIAL_8N1);
    tx.push("#HELLO,STATIC_ROUTE,2\r\n");
    Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin(); Wire.setClock(100000);
    if (!initEncoders()) { fault("ENCODER_INIT", millis()); return; }
    if (!(TIM4->CR1 & TIM_CR1_CEN) ||
        (TIM4->CCER & (TIM_CCER_CC3E|TIM_CCER_CC4E)) != (TIM_CCER_CC3E|TIM_CCER_CC4E)) {
        fault("PWM_TIMER_INIT", millis()); return;
    }
    // Enable the data-ready status bit; no external interrupt or extra PCB pin.
    if (!initImu() || !writeReg(Wire, mpu, 0x38, 1)) { fault("MPU_INIT", millis()); return; }
#ifndef STATIC_ROUTE_NO_RANGE
    if (!RangeSensors::begin()) {
        char line[40];
        snprintf(line, sizeof(line), "#RANGE_READY_MASK,%u\r\n", unsigned(RangeSensors::readyMask()));
        tx.push(line);
        fault("RANGE_INIT", millis()); return;
    }
#endif
    previousCount[0] = uint16_t(TIM3->CNT); previousCount[1] = uint16_t(TIM2->CNT);
    stateAt = lastGoodAt = lastPollAt = millis();
    tx.push("#CALIBRATING_GYRO\r\n");
}
char rxLine[64];
unsigned rxLen = 0;
uint32_t rebootAt = 0;
// Bounded per loop: at most 24 received bytes, so a flood cannot stall the control loop.
void serviceCommands(uint32_t now) {
    for (int n = 0; n < 24 && Serial.available() > 0; ++n) {
        const int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rxLen) { rxLine[rxLen] = 0; handleCommand(rxLine); }
            rxLen = 0;
        } else if (rxLen < sizeof(rxLine)-1) rxLine[rxLen++] = char(c);
        else rxLen = 0;  // overlong line: drop it
    }
    if (rebootRequested && state != RobotState::RUN_ROUTE) {
        if (!rebootAt) { rebootAt = now; cutDrive(); }
        if (now-rebootAt > 150) NVIC_SystemReset();  // let the #ACK leave first
    }
}
void service() {
    uint32_t now = millis();
    if (tripped) fault("CONTROL_TIMEOUT", now);
    serviceCommands(now);
#ifndef STATIC_ROUTE_NO_RANGE
    RangeSensors::poll(now);  // one bounded I2C step on the sensor bus
#endif
    serviceUi(digitalRead(SwitchPin)==LOW, now, uint16_t(TIM3->CNT), uint16_t(TIM2->CNT));
    if (state != RobotState::FAULT && now-lastPollAt >= POLL_MS) {
        lastPollAt = now;
        float rate = 0;
        const ImuRead result = readGyro(rate);
        now = millis();
        if (tripped) fault("CONTROL_TIMEOUT", now);
        const uint32_t startedUs = micros();
        control(now, uint16_t(TIM3->CNT), uint16_t(TIM2->CNT), result, rate, tickSeen);
        const uint32_t tookUs = micros()-startedUs;
        if (state == RobotState::RUN_ROUTE && tookUs > maxControlUs) maxControlUs = tookUs;
    }
    digitalWrite(PC13, ledOn(millis()) ? LOW : HIGH);
    const int room = Serial.availableForWrite();
    if (room > 0 && tx.count) {
        unsigned n = tx.contiguous();
        if (n > unsigned(room)) n = unsigned(room);
        if (n > 32) n = 32;
        tx.consume(Serial.write((const uint8_t*)tx.data + tx.head, n));
    }
}
#endif
} // namespace StaticRoute

#ifndef STATIC_ROUTE_HOST_TEST
void setup() { StaticRoute::beginHardware(); }
void loop() { StaticRoute::service(); }
#endif
