// g++ -std=c++17 -Iinclude -Isrc -O2 test/static_route_host.cpp -o route-test
// Ideal host simulation: wheels follow the setpoint, VL53L0X readings are ray-cast
// from the estimated pose against the MAP boxes. No PWM/friction/real-position claim.
#define STATIC_ROUTE_HOST_TEST
#include "../src/test_static_route.cpp"
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace StaticRoute;
namespace StaticRoute {
int driven[2]{};
void cutDrive() { driven[0] = driven[1] = 0; }
void writeMotors(int l, int r, uint32_t) { driven[0] = l; driven[1] = r; }
}

struct Box { float x, y; };
float rayBox(float ox, float oy, float dx, float dy, const Box &b) {
    const float o[2] = {ox, oy}, d[2] = {dx, dy};
    const float lo[2] = {b.x-BoxHalf, b.y-BoxHalf}, hi[2] = {b.x+BoxHalf, b.y+BoxHalf};
    float t0 = 0, t1 = 1.0e9f;
    for (int a = 0; a < 2; ++a) {
        if (fabsf(d[a]) < 1.0e-9f) { if (o[a] < lo[a] || o[a] > hi[a]) return -1; continue; }
        float ta = (lo[a]-o[a])/d[a], tb = (hi[a]-o[a])/d[a];
        if (ta > tb) std::swap(ta, tb);
        t0 = fmaxf(t0, ta); t1 = fminf(t1, tb);
        if (t0 > t1) return -1;
    }
    return t0;
}

// Motor model (motor scenarios): steady CPS for a PWM = inverse of the firmware's
// feedforward table, times a per-motor gain (0.85 = that motor is 15 % weaker).
float cpsFromPwm(unsigned i, int pwm) {
    const float cps[] = {0, 200, 400, 700, 2000};
    const float pwmL[] = {0, 12.5f, 16.5f, 25.5f, 63.5f}, pwmR[] = {0, 15.5f, 20.5f, 29.5f, 67.5f};
    const float *p = i ? pwmR : pwmL, m = fabsf(float(pwm));
    float c = cps[4];
    for (unsigned k=1; k<5; ++k) if (m <= p[k]) { c = cps[k-1] + (cps[k]-cps[k-1])*(m-p[k-1])/(p[k]-p[k-1]); break; }
    return pwm < 0 ? -c : c;
}
struct Rig {
    uint32_t now = 0;
    uint16_t count[2] = {8, 65520};
    float residual[2]{}, actual[2]{};
    std::string log;
    std::vector<Box> world;          // boxes the simulated sensors can see
    bool rangesOn = true;
    float minClear = 1.0e9f;         // robot centre to nearest box edge while running
    float minExtra = 1.0e9f;         // ... to boxes added by the scenario (not the fixed MAP)
    float minFoot = 1.0e9f;          // 130 x 100 mm body to any box (0 = collision)
    float minFootExtra = 1.0e9f;     // body to the scenario's boxes
    // Separate truth (place/drift scenarios): the beams are cast from the TRUE pose, the
    // encoders count with the firmware's CountsPerMm but the wheels really move ds/trueScale,
    // and the gyro reads trueGyro x the true rate. Otherwise truth follows the estimate.
    bool separateTruth = false;
    Pose truth = StartPose;
    float trueScale[2] = {1, 1}, trueGyro = 1;
    bool motorModel = false;         // wheels follow the PWM through cpsFromPwm, not the request
    float motorGain[2] = {1, 1};
    Rig() {
        for (const auto &o : StaticMap::FIXED_OBSTACLES) world.push_back({float(o.x), float(o.y)});
        RangeSensors::begin();
    }
    void flush() {
        while (tx.count) {
            const unsigned n = tx.contiguous();
            log.append(tx.data+tx.head, n); tx.consume(n);
        }
    }
    void injectRanges() {
        for (unsigned i = 0; i < RangeSensors::Count; ++i) {
            if (((now/10) + i) % 3) continue;  // ~33 Hz per sensor, staggered
            float ox, oy, dx, dy;
            beamFrom(truth, i, ox, oy, dx, dy);
            float best = 1.0e9f;
            for (const Box &b : world) {
                const float t = rayBox(ox, oy, dx, dy, b);
                if (t >= 0) best = fminf(best, t);
            }
            const uint16_t mm = best <= RangeSensors::MaxUsefulMm ? uint16_t(lroundf(fmaxf(best, 1)))
                                                                   : RangeSensors::NoTarget;
            RangeSensors::inject(i, mm, now);
        }
    }
    void tick(bool down=false, bool freezeYaw=false, int lockedWheel=-1, bool reversed=false) {
        constexpr float dt = 0.010f;
        now += 10;
        for (int i=0; i<2; ++i) {
            float demand = request.v+(i ? 0.5f : -0.5f)*Track*request.w;
            if (motorModel) demand = motorGain[i]*cpsFromPwm(i, driven[i])/CountsPerMm[i];
            if (!driven[i] || lockedWheel==i) demand = 0;
            actual[i] += (demand-actual[i])*0.25f;
            residual[i] += actual[i]*dt*CountsPerMm[i];
            const int ticks = lroundf(residual[i]); residual[i] -= ticks;
            count[i] += (reversed && i==0 ? -1 : 1)*EncoderSign[i]*ticks;
        }
        const float vl = actual[0]/trueScale[0], vr = actual[1]/trueScale[1];
        const float tw = (vr-vl)/Track, tv = 0.5f*(vl+vr);
        truth.x += tv*dt*cosf(truth.h+tw*dt*0.5f); truth.y += tv*dt*sinf(truth.h+tw*dt*0.5f);
        truth.h = wrap(truth.h+tw*dt);
        const float raw = 0.65f + (freezeYaw ? 0 : trueGyro*tw/Rad);
        if (rangesOn) injectRanges();
        serviceUi(down, now, count[0], count[1]);
        control(now, count[0], count[1], ImuRead::NewSample, raw, true);
        if (!separateTruth) truth = pose;  // ideal rig: the estimate IS the truth
        assert(abs(driven[0]) <= RoutePwmMax && abs(driven[1]) <= RoutePwmMax);
        if (state != RobotState::RUN_ROUTE) assert(!driven[0] && !driven[1]);
        if (state == RobotState::RUN_ROUTE)
            for (size_t i = 0; i < world.size(); ++i) {
                const float c = boxDistance(truth.x, truth.y, world[i].x, world[i].y, BoxHalf);
                const float f = footprintGap(truth.x, truth.y, cosf(truth.h), sinf(truth.h),
                                             world[i].x, world[i].y, BoxHalf, BoxHalf);
                minClear = fminf(minClear, c); minFoot = fminf(minFoot, f);
                if (i >= size_t(StaticMap::FIXED_COUNT)) { minExtra = fminf(minExtra, c); minFootExtra = fminf(minFootExtra, f); }
            }
        flush();
    }
    void ready() {
        previousCount[0]=count[0]; previousCount[1]=count[1];
        for (int i=0; i<220; ++i) tick(true);  // held through boot
        assert(state==RobotState::WAIT_START && runNumber==0 && fabsf(gyroBias-0.65f)<0.001f);
    }
    void press() {
        for (int i=0; i<6; ++i) tick(false);
        tick(true); tick(false); tick(true);    // contact bounce
        for (int i=0; i<4; ++i) tick(true);
    }
    void start() {
        press();
        assert(state==RobotState::RUN_ROUTE && target==1);
        if (!separateTruth) {  // beams cast from the nominal START: the fit must not move it
            if (fabsf(pose.x-START_X)>1 || fabsf(pose.y-START_Y)>1 || fabsf(pose.h-StartPose.h)>0.3f*Rad)
                std::cerr << "start fit " << pose.x << "," << pose.y << "," << pose.h/Rad << std::endl;
            assert(fabsf(pose.x-START_X)<=1 && fabsf(pose.y-START_Y)<=1 && fabsf(pose.h-StartPose.h)<=0.3f*Rad);
        }
        assert(!encoderTotal[0] && !encoderTotal[1]);
        assert(travelled==0 && !wheels[0].integral && !wheels[1].integral);
        assert(legPhase==LegPhase::TRACK);  // 21.6 deg start error: no pivot, it curves in
    }
    void finish(float minTravel=2800, float maxTravel=3020) {
        float peak = 0;
        for (unsigned i=0; i<9000 && state==RobotState::RUN_ROUTE; ++i) {
            tick(true); // holding the start press may never stop or restart a run
            assert(request.v>=0);
            if (legPhase==LegPhase::TRACK && request.v>0)
                assert(request.v-0.5f*Track*fabsf(request.w)>=-0.001f);  // inner wheel never reverses
            peak = fmaxf(peak, request.v);
        }
        if (state != RobotState::FINISH_BLINK) {
            std::cerr << "node " << target << " " << stopReason << " at s=" << station << '\n';
            for (size_t p = 0; (p = log.find('#', p)) != std::string::npos; ++p)
                if (log.compare(p, 9, "#OBSTACLE")==0 || log.compare(p, 6, "#AVOID")==0)
                    std::cerr << log.substr(p, log.find('\n', p)-p) << '\n';
            for (unsigned i = 0; i < obstacles.count; ++i) {
                const Obstacle &o = obstacles.items[i];
                if (o.kind == 'F') continue;
                float s = -1, lat = 0; path.nearest(o.x, o.y, 0, path.length, s, lat);
                std::cerr << "  obstacle[" << i << "] " << o.kind << int(o.id) << " (" << o.x << "," << o.y
                          << ") s=" << s << " lat=" << lat << " conf=" << o.confirmed << " plan=" << o.planned
                          << " frozen=" << o.frozen << " samples=" << int(o.samples) << '\n';
            }
            for (const auto &a : avoid)
                std::cerr << "  avoid active=" << a.active << " obstacle=" << int(a.obstacle) << " offset="
                          << a.offset << " s=" << a.s0 << ".." << a.s3 << '\n';
        }
        assert(state==RobotState::FINISH_BLINK && peak>=0.95f*CruiseCps/MaxCountsPerMm && goalError()<=POSITION_TOL_MM);
        if (travelled<minTravel || travelled>maxTravel) std::cerr << "travelled " << travelled << '\n';
        assert(travelled>minTravel && travelled<maxTravel && target==4);
        assert(!driven[0] && !driven[1] && std::string(stopReason)=="GOAL_REACHED");
        // LED stays solid for FinishHoldMs after the goal, then WAIT_START blinks again.
        for (unsigned t=0; t<FinishHoldMs; t+=10) assert(ledOn(now+t));
    }
    void checkFault(const char *reason) {
        if (state!=RobotState::FAULT || std::string(stopReason)!=reason)
            std::cerr << "expected " << reason << " got " << stopReason << '\n';
        assert(state==RobotState::FAULT && std::string(stopReason)==reason);
        assert(!request.v && !request.w && !driven[0] && !driven[1]);
        press(); assert(state==RobotState::FAULT);
        assert(log.find(std::string("#STOP_REASON,")+reason)!=std::string::npos);
    }
    bool has(const std::string &text) const { return log.find(text)!=std::string::npos; }
};

// The filleted reference itself must clear every fixed box, the forbidden zone and the
// random candidate spots (exactly one of them appears, so the route must clear all 4).
void checkPathClearance() {
    Path p; p.build();
    ObstacleMap none;
    float worst = 1.0e9f, worstS = 0;
    for (float s = 0; s <= p.length; s += 2) {
        const PathPoint q = p.at(s);
        const float c = none.clearance(q.x, q.y, q.h);  // full footprint along the path
        if (c < worst) { worst = c; worstS = s; }
    }
    std::cout << "path " << p.length << " mm, " << p.count << " segments, min body gap "
              << worst << " mm at s=" << worstS << '\n';
    assert(p.count==7 && worst>=FootMarginMm);
    // Footprint gap sanity: side-on 20 mm, overlapping 0, corner-on diagonal.
    assert(fabsf(footprintGap(0, 0, 1, 0, 0, BodyHalfWidth+BoxHalf+20, BoxHalf, BoxHalf)-20)<0.01f);
    assert(footprintGap(0, 0, 1, 0, BodyHalfLength+BoxHalf-1, 0, BoxHalf, BoxHalf)==0);
    const float r = sqrtf(2.0f)/2;  // car at 45 deg: its front corner reaches 81.3 mm along x
    assert(fabsf(footprintGap(0, 0, r, r, 81.317f+BoxHalf+10, 0, BoxHalf, BoxHalf)-10)<0.05f);
    // Exact shifted geometry: an offset d toward the centre of a radius-R arc has radius R-d.
    assert(fabsf(shiftedCurv(-1/150.0f, {-75, 0, 0}) - (-1/75.0f)) < 1e-6f);
    assert(fabsf(shiftedCurv(0, {0, 1, 0})) < 1e-9f && fabsf(shiftedCurv(0, {10, 0, 0.01f})-0.01f) < 1e-7f);
    for (int i=1; i<ROUTE_COUNT; ++i) assert(p.nodeS[i]>p.nodeS[i-1]);
    // Segments join without gaps or heading jumps.
    for (unsigned i=1; i<p.count; ++i) {
        const PathPoint e = Path::eval(p.seg[i-1], p.seg[i-1].len);
        assert(hypotf(e.x-p.seg[i].x0, e.y-p.seg[i].y0)<0.5f && fabsf(wrap(e.h-p.seg[i].h0))<0.002f);
    }
    const PathPoint end = p.at(p.length);
    assert(hypotf(end.x-GOAL_X, end.y-GOAL_Y)<0.5f);
}

int main(int argc, char **argv) {
    const std::string scenario = argc>1 ? argv[1] : "route";
    Rig rig;
    if (scenario=="clearance") { checkPathClearance(); std::cout << "PASS " << scenario << '\n'; return 0; }
    if (scenario=="wrap") rig.now=stateAt=lastGoodAt=UINT32_MAX-2400;
    if (scenario=="init-bias" || scenario=="init-systick") {
        for (uint32_t i=10; i<=12010 && state!=RobotState::FAULT; i+=10) {
            control(i,0,0,ImuRead::NewSample,scenario=="init-bias"?12:0,false);
        }
        rig.checkFault(scenario=="init-bias" ? "GYRO_NOT_STILL" : "SYSTICK_GUARD");
    } else if (scenario=="commands") {
        // Web tool / Bluetooth protocol: parameters, refusals, remote start and stop.
        auto cmd = [&](const char *s) { char b[64]; snprintf(b, sizeof(b), "%s", s); handleCommand(b); rig.flush(); };
        auto num = [](float v) { return std::string(Number(v).text); };
        assert(num(400)=="400" && num(0.027f)=="0.027" && num(2.8e-4f)=="0.00028" && num(-12.5f)=="-12.5"
               && num(0)=="0" && num(1200)=="1200" && num(0.5e-4f)=="0.00005");
        float pv = 0;
        assert(parseNumber("-12.5", pv) && pv==-12.5f && parseNumber("2.8e-4", pv) && fabsf(pv-2.8e-4f)<1e-9f);
        assert(parseNumber("400", pv) && pv==400 && parseNumber("0.027", pv) && fabsf(pv-0.027f)<1e-7f);
        assert(!parseNumber("abc", pv) && !parseNumber("1.2.3", pv) && !parseNumber("", pv) && !parseNumber("5e", pv));
        rig.ready();
        cmd("$GET");
        size_t params = 0;
        for (size_t p = 0; (p = rig.log.find("#PARAM,", p)) != std::string::npos; ++p) ++params;
        assert(params==TuneCount && rig.has("#PARAM,LATERAL_GAIN,0.00028,0.00005,0.001") && rig.has("#STATE,WAIT_START"));
        cmd("$set,CRUISE_CPS,500"); assert(CruiseCps==500 && rig.has("#PARAM,CRUISE_CPS,500,200,700"));
        cmd("$SET,CRUISE_CPS,900"); assert(CruiseCps==500 && rig.has("#ERR,RANGE"));
        cmd("$SET,CRUISE_CPS,abc"); assert(CruiseCps==500);
        cmd("$SET,NOPE,1"); assert(rig.has("#ERR,UNKNOWN_PARAM"));
        cmd("$SET,SAFE_MARGIN,30"); assert(ClearMm==80 && PassMm==140);
        cmd("$DEFAULTS"); assert(ClearMm==BodyHalfWidth+StaticMap::SAFE_MARGIN && CruiseCps==DefaultTuning.cruiseCps);
        cmd("$STOP"); assert(rig.has("#ERR,STATE"));
        cmd("FOO"); assert(rig.has("#ERR,FORMAT"));
        cmd("$XYZ"); assert(rig.has("#ERR,UNKNOWN_COMMAND"));
        cmd("$START"); assert(rig.has("#ACK,START"));
        rig.tick(false); assert(state==RobotState::RUN_ROUTE);
        assert(rig.log.find("#PARAM,CRUISE_CPS,", rig.log.find("#ROUTE_STARTED"))!=std::string::npos);
        cmd("$SET,CRUISE_CPS,300"); assert(rig.has("#ERR,BUSY") && CruiseCps==DefaultTuning.cruiseCps);
        cmd("$CAL,START"); assert(!cal.active);  // refused while the route runs
        cmd("$REBOOT"); assert(!rebootRequested);
        for (int i=0; i<30; ++i) rig.tick(false);
        cmd("$STOP"); assert(rig.has("#ACK,STOP"));
        rig.tick(false);
        assert(state==RobotState::WAIT_START && rig.has("#STOP_REASON,REMOTE_STOP") && !driven[0] && !driven[1]);
        cmd("$PING"); assert(rig.has("#PONG,WAIT_START"));
        // Hand calibration: push 1000 mm, the encoders count, motors stay off.
        cmd("$CAL,STOP"); assert(rig.has("#ERR,STATE"));
        for (int i=0; i<60; ++i) rig.tick(false);  // let the stopped car finish coasting
        cmd("$CAL,START"); assert(rig.has("#ACK,CAL_START") && cal.active);
        rig.count[0] += EncoderSign[0]*4500; rig.count[1] += EncoderSign[1]*4510;
        for (int i=0; i<15; ++i) rig.tick(false);
        assert(rig.has("#CAL,") && !driven[0] && !driven[1] && state==RobotState::WAIT_START);
        cmd("$CAL,STOP");
        assert(!cal.active && rig.log.find(",4500,4510,", rig.log.find("#CAL_DONE,"))!=std::string::npos);
        cmd("$CAL,XYZ"); assert(rig.has("#ERR,FORMAT"));
        cmd("$REBOOT"); assert(rebootRequested && rig.has("#ACK,REBOOT"));
        RangeSensors::inject(0, 100, 0);  // a sample taken exactly at millis()==0 is valid
        assert(RangeSensors::fresh(0, 50) && !RangeSensors::fresh(0, 101));
    } else if (scenario=="obstacle-map") {
        ObstacleMap m;
        auto box = [&](float x, float y, uint32_t t) {
            Obstacle *c = nullptr;
            for (int k = 0; k < 3; ++k) {  // three hits confirm it
                const Obstacle seen = m.classify(x, y-BoxHalf, 0, 1, t);
                if (Obstacle *o = m.note(seen, x, y-BoxHalf, t)) c = o;
            }
            return c;
        };
        // Far outside the MAP so no hit snaps to a fixed or candidate box.
        for (unsigned i = 0; i < MaxObstacles; ++i) assert(box(3000.0f+200*(i%8), 3000.0f+250*(i/8), 1000));
        assert(!m.overflow && m.count==MaxObstacles);
        assert(!box(5000, 5000, 1500) && m.overflow);   // all recent and confirmed: must report full
        m.overflow = false;
        assert(box(5000, 5000, 1000+ForgetMs+1) && !m.overflow);  // stale entries are recycled
        assert(m.classify(50, 150-BoxHalf, 0, 1, 0).kind=='F' && !m.note(m.classify(50, 100, 0, 1, 0), 50, 100, 0));
    } else if (scenario=="uart") {
        const std::string line(100,'s');
        for (int i=0; i<30; ++i) tx.push(line.c_str(),true);
        assert(tx.droppedSamples && tx.count<=TxQueue::Capacity-TxQueue::EventReserve);
        const std::string report(500,'e'); tx.push(report.c_str());
        assert(!tx.droppedEvents); rig.flush();
        assert(rig.log.substr(rig.log.size()-report.size())==report);
    } else {
        int randomId = 0;
        if (scenario.rfind("random",0)==0) {
            randomId = std::stoi(scenario.substr(6));
            for (const auto &o : StaticMap::POSSIBLE_OBSTACLES)
                if (o.id==randomId) rig.world.push_back({float(o.x), float(o.y)});
            assert(rig.world.size()==size_t(StaticMap::FIXED_COUNT)+1);
        }
        // Placement / calibration errors (truth separate from the estimate).
        const bool place = scenario=="place", drift = scenario.rfind("drift",0)==0;
        const bool motor = scenario.rfind("motor",0)==0;
        if (motor) { rig.motorModel = true; rig.motorGain[1] = 0.85f; }
        if (scenario=="motor-nosync") tune.syncGain = 0;
        if (place || drift) rig.separateTruth = true;
        if (place) rig.truth = {StartPose.x+20, StartPose.y-15, StartPose.h+4*Rad};
        if (drift) { rig.trueScale[0] = rig.trueScale[1] = 1.02f; rig.trueGyro = 1.02f; }
        if (scenario=="drift-nofix") tune.rangeFix = 0;
        // Unknown boxes: one partly intruding on the last leg (room to sidestep), one
        // dead centre in the N1->N2 corridor (no 100 mm gap either side: must stop safely).
        if (scenario=="block") rig.world.push_back({1510, 307});
        if (scenario=="block-center") rig.world.push_back({362, 652});
        rig.ready();
        assert(ledOn(stateAt) && !ledOn(stateAt+250) && ledOn(stateAt+500));
        rig.start();
        assert(rig.has("#PATH_SEG,6,"));
        if (scenario=="route" || scenario=="wrap" || randomId) {
            rig.finish();
            std::string firstLog=rig.log;
            size_t pos=0;
            for (unsigned n=0; n<5; ++n) {
                for (const char *e: {"START","END"}) {
                    pos=firstLog.find(std::string("#WAYPOINT_")+e+","+std::to_string(n)+",",pos);
                    assert(pos!=std::string::npos); ++pos;
                }
            }
            assert(firstLog.find("#ROUTE_FINISHED")!=std::string::npos);
            assert(firstLog.find("#DROPPED_SAMPLES,0")!=std::string::npos);
            assert(firstLog.find("#AVOID")==std::string::npos);  // nothing on the route: no detour
            assert(firstLog.find(",STRAIGHT,")==std::string::npos && firstLog.find(",ALIGN,")==std::string::npos);
            std::cout << "min clearance " << rig.minClear << " mm, body gap " << rig.minFoot << " mm";
            assert(rig.minFoot>=FootMarginMm*0.5f);  // tracking error eats at most half the margin
            if (randomId) std::cout << ", random " << randomId << (firstLog.find("#OBSTACLE,"+std::to_string(randomId)
                                     +",RANDOM")!=std::string::npos ? " seen" : " not seen");
            std::cout << '\n';
            assert(rig.minClear>=ClearMm);
            if (argc>2) {
                std::ofstream out(argv[2],std::ios::binary);
                out << "#SOURCE,HOST_SIMULATION\r\n" << firstLog; assert(out.good());
            }
            if (scenario!="wrap" && !randomId) {
                const float error1=goalError(), dist1=travelled;
                const uint32_t time1=rig.now-runAt;
                for (int i=0; i<230; ++i) rig.tick(true);
                assert(state==RobotState::FINISH_BLINK && ledOn(rig.now));  // still solid at 2.3 s
                for (int i=0; i<100; ++i) rig.tick(true);
                assert(state==RobotState::WAIT_START && runNumber==1);
                rig.count[0]+=3100; rig.count[1]-=1300;  // lift and replace between runs
                rig.start(); assert(runNumber==2 && maxCross==0);
                rig.finish(); std::cout << "repeat " << goalError() << " mm goal error, distance " << travelled << " mm\n";
                assert(fabsf(goalError()-error1)<POSITION_TOL_MM && fabsf(travelled-dist1)<POSITION_TOL_MM);
                std::cout << "simulation " << time1 << " ms, " << dist1 << " mm, goal error " << error1 << " mm, max cross " << maxCross << " mm\n";
            }
        } else if (motor) {
            float worstHead = 0;
            rig.finish(2700, 3150);
            (void)worstHead;
            std::cout << "right motor 15% weak: max sync " << sync.maxMm << " mm, max cross " << maxCross
                      << " mm, goal " << goalError() << " mm, body gap " << rig.minFoot << " mm" << std::endl;
            assert(rig.has("#MAX_SYNC_MM,") && rig.minFoot>0);
            if (scenario=="motor") assert(sync.maxMm<8);
        } else if (place || drift) {
            const Pose &t = rig.truth;
            std::cout << "start estimate error " << hypotf(pose.x-t.x, pose.y-t.y) << " mm, "
                      << fabsf(wrap(pose.h-t.h))/Rad << " deg; ";
            assert(rig.has(place ? "#START_POSE,OK," : "#START_POSE,OK,"));
            std::cout << "(est " << pose.x << "," << pose.y << "," << pose.h/Rad << " true " << t.x << "," << t.y << "," << t.h/Rad << ") ";
            if (place) assert(hypotf(pose.x-t.x, pose.y-t.y)<8 && fabsf(wrap(pose.h-t.h))<2*Rad);
            rig.finish(2650, 3150);
            const float trueGoal = hypotf(t.x-GOAL_X, t.y-GOAL_Y);
            std::cout << "true GOAL error " << trueGoal << " mm, body gap " << rig.minFoot << " mm" << std::endl;
            // No fixed box is in view on the last leg: only GYRO_SCALE / $CAL remove the rest.
            if (scenario!="drift-nofix") assert(trueGoal<(place ? 12 : 40) && rig.minFoot>(place ? FootMarginMm*0.5f : 10));
        } else if (scenario=="block") {
            rig.finish(2800, 3100);
            assert(rig.has("#OBSTACLE,0,UNKNOWN") && rig.has("#AVOID,0,"));
            const size_t a = rig.log.find("#AVOID,0,");
            std::cout << rig.log.substr(a, rig.log.find('\n', a)-a) << "\nclearance to box " << rig.minExtra
                      << " mm (body " << rig.minFootExtra << "), to MAP body " << rig.minFoot << " mm, max cross " << maxCross << " mm\n";
            assert(rig.minFootExtra>=FootMarginMm*0.5f && rig.minFoot>=FootMarginMm*0.5f);  // no contact
            if (argc>2) {
                std::ofstream out(argv[2],std::ios::binary);
                out << "#SOURCE,HOST_SIMULATION\r\n" << rig.log; assert(out.good());
            }
        } else if (scenario=="block-center") {
            for (int i=0; i<9000 && state==RobotState::RUN_ROUTE; ++i) rig.tick(false);
            for (int i=0; i<60; ++i) rig.tick(false);  // coast after the cut
            std::cout << "stop " << stopReason << " at s=" << station << ", clearance to box " << rig.minExtra << " mm\n";
            assert(rig.has("#AVOID_FAIL,0,NO_ROOM"));
            assert(rig.minExtra>150);  // decided well before the box
            rig.checkFault("NO_AVOID_PATH");
        } else if (scenario=="late") {
            // A box appears 230 mm ahead on leg N2->N3: too close for a smooth detour, must stop.
            for (int i=0; i<9000 && station<1500; ++i) rig.tick(false);
            assert(state==RobotState::RUN_ROUTE);
            rig.world.push_back({pose.x+230*cosf(pose.h), pose.y+230*sinf(pose.h)});
            for (int i=0; i<300 && state==RobotState::RUN_ROUTE; ++i) rig.tick(false);
            for (int i=0; i<60; ++i) rig.tick(false);  // coast after the cut
            std::cout << "stop " << stopReason << ", clearance to box " << rig.minExtra << " mm\n";
            const std::string why = stopReason;  // centre of the corridor: no room either way
            assert(why=="OBSTACLE_TOO_CLOSE" || why=="OBSTACLE_STOP" || why=="NO_AVOID_PATH");
            assert(rig.minExtra>BodyHalfLength);  // the nose never reaches the box
            rig.checkFault(stopReason);
        } else if (scenario=="range-stale") {
            for (int i=0; i<50; ++i) rig.tick(false);
            rig.rangesOn = false;
            for (int i=0; i<20 && state==RobotState::RUN_ROUTE; ++i) rig.tick(false);
            rig.checkFault("RANGE_STALE");
        } else if (scenario=="manual") {
            for (int i=0; i<80; ++i) rig.tick(false);
            rig.press(); assert(state==RobotState::WAIT_START && runNumber==1 && !driven[0] && !driven[1]);
            for (int i=0;i<100;++i) rig.tick(true);
            assert(runNumber==1 && rig.log.find("#ROUTE_FINISHED")==std::string::npos);
            assert(rig.log.find("#STOP_REASON,MANUAL_SW1")!=std::string::npos);
            rig.start();
        } else if (scenario=="stall-left" || scenario=="stall-right" || scenario=="wrong-dir") {
            // Hold both encoder and gyro stationary for stall, or inject reversed left feedback.
            for (int i=0; i<140 && state==RobotState::RUN_ROUTE; ++i) {
                if (scenario=="wrong-dir") rig.tick(false,false,-1,true);
                else {
                    const int locked = scenario=="stall-left"?0:1;
                    // Match the gyro to the unlocked wheel motion to isolate the wheel guard.
                    rig.tick(false,false,locked);
                }
            }
            rig.checkFault(scenario=="stall-right"?"WHEEL_R_STALL_DIR":"WHEEL_L_STALL_DIR");
        } else if (scenario=="mpu-read" || scenario=="mpu-stale" || scenario=="mpu-invalid" || scenario=="timeout") {
            const char *why = "MPU_READ";
            auto status=ImuRead::BusError;
            uint32_t gap=10; float raw=0;
            if (scenario=="mpu-stale") { status=ImuRead::NotReady; gap=40; why="MPU_STALE"; }
            if (scenario=="mpu-invalid") { status=ImuRead::NewSample; raw=249; why="MPU_INVALID"; }
            if (scenario=="timeout") { status=ImuRead::NewSample; gap=70; why="CONTROL_TIMEOUT"; }
            control(rig.now+gap,rig.count[0],rig.count[1],status,raw,true);
            rig.checkFault(why);
        } else if (scenario=="mpu-frozen") {
            for (int i=0; i<250 && state==RobotState::RUN_ROUTE; ++i) rig.tick(false,true);
            rig.checkFault("GYRO_ENCODER_DISAGREE");
        } else if (scenario=="encoder-jump") {
            control(rig.now+10,rig.count[0]+3000,rig.count[1],ImuRead::NewSample,0,true);
            rig.checkFault("ENCODER_JUMP");
        } else if (scenario=="miss") {
            // Knocked 120 mm sideways: far outside the tracking corridor.
            pose.x += 120; follow(.01f, rig.now);
            rig.checkFault("OFF_PATH");
        } else if (scenario=="goal" || scenario=="coast") {
            // On the last straight: cross the GOAL plane off the disk, or coast out after entry.
            pathSeg = path.count-1; legPhase = LegPhase::TRACK; pose.h = -90*Rad;
            const float offset = scenario=="coast" ? 0.0f : 18.0f;
            pose.x = GOAL_X + offset; pose.y = GOAL_Y + (scenario=="coast" ? 1.0f : -1.0f);
            follow(.01f, rig.now);
            if (scenario=="coast") {
                assert(legPhase==LegPhase::SETTLE);
                pose.x += 2*POSITION_TOL_MM;
                follow(.01f,rig.now+10); follow(.01f,rig.now+200);
            }
            rig.checkFault("MISSED_WAYPOINT");
            assert(rig.log.find("#ROUTE_FINISHED")==std::string::npos);
        } else { std::cerr << "Unknown test\n"; return 2; }
    }
    std::cout << "PASS " << scenario << '\n';
}
