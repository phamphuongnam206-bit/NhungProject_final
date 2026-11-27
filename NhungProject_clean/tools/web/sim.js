// JavaScript port of src/test_static_route.cpp (+ static_route/path.h, avoid.h, tuning.h)
// and of the host rig in test/static_route_host.cpp. Same algorithm, step for step, so
// the web tool predicts what the firmware does. Checked against the C++ simulation by
// tools/web/check_sim.js (node). Units: mm, s, rad; map origin bottom-left.
(function (root) {
'use strict';
const Pi = 3.14159265, Rad = Pi/180;
const bound = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const wrap = a => { while (a > Pi) a -= 2*Pi; while (a < -Pi) a += 2*Pi; return a; };
const lround = x => x < 0 ? -Math.round(-x) : Math.round(x);
const hypot = (a, b) => Math.sqrt(a*a + b*b);

// ---- include/map_config.h ----
const BOX = 100;
const MAP = {
    width: 1800, height: 1200, box: BOX, robotLength: 130, robotWidth: 100, safeMargin: 15,
    forbidden: {xmin: 600, ymin: 0, xmax: 1200, ymax: 600},
    start: {x: 300, y: -65, headingDeg: 90}, goal: {x: 1600, y: -100},
    fixed: [[1,50,150],[2,550,250],[3,300,350],[4,550,450],[5,50,550],[6,50,750],[7,350,900],
            [8,450,1150],[9,650,750],[10,750,650],[11,850,1000],[12,950,650],[13,1150,1150]]
        .map(([id, x, y]) => ({id, x, y})),
    possible: [[14,1250,850],[15,1450,450],[16,1450,150],[17,1750,250]].map(([id, x, y]) => ({id, x, y})),
    route: [{x: 300, y: -65}, {x: 120, y: 390}, {x: 605, y: 915}, {x: 1600, y: 570}, {x: 1600, y: -100}],
};
// ---- src/static_route/config.h ----
const NEAR_SPEED_MM_S = 38, NEAR_DISTANCE_MM = 65, TURN_RAD_S = 0.85, ANG_ACCEL_RAD_S2 = 2.4;
const POSITION_TOL_MM = 12, ANGLE_TOL_DEG = 3, BRAKE_AT_MM = 0.5*POSITION_TOL_MM;
const STILL_MM_S = 6, STILL_DEG_S = 3, SETTLE_MS = 150, LOG_MS = 100;
// ---- src/static_route/tuning.h (runtime parameters; same names as the $SET protocol) ----
const TUNE = [
    {name: 'CRUISE_CPS',   key: 'cruiseCps',    def: 400,    lo: 200,    hi: 700},
    {name: 'APPROACH_CPS', key: 'approachCps',  def: 200,    lo: 100,    hi: 300},
    {name: 'AVOID_CPS',    key: 'avoidCps',     def: 300,    lo: 150,    hi: 700},
    {name: 'LAT_ACCEL',    key: 'latAccel',     def: 300,    lo: 100,    hi: 800},
    {name: 'LATERAL_GAIN', key: 'lateralGain',  def: 2.8e-4, lo: 0.5e-4, hi: 10e-4},
    {name: 'HEADING_GAIN', key: 'headingGain',  def: 0.027,  lo: 0.005,  hi: 0.08},
    {name: 'FILLET_MM',    key: 'filletMm',     def: 150,    lo: 60,     hi: 300},
    {name: 'SAFE_MARGIN',  key: 'safeMarginMm', def: 15,     lo: 0,      hi: 50},
    {name: 'DETECT_MM',    key: 'detectMm',     def: 650,    lo: 250,    hi: 1200},
    {name: 'GYRO_SCALE',   key: 'gyroScale',    def: 1,      lo: 0.9,    hi: 1.1},
    {name: 'START_FIX',    key: 'startFix',     def: 1,      lo: 0,      hi: 1},
    {name: 'RANGE_FIX',    key: 'rangeFix',     def: 1,      lo: 0,      hi: 1},
    {name: 'SYNC_GAIN',    key: 'syncGain',     def: 3,      lo: 0,      hi: 8},
];
const defaultTune = () => Object.fromEntries(TUNE.map(p => [p.key, p.def]));

// ---- src/static_route/path.h ----
class Path {
    constructor() { this.seg = []; this.length = 0; this.nodeS = [0, 0, 0, 0, 0]; }
    add(x, y, h, len, k) {
        if (len < 0.5) return;
        this.seg.push({x0: x, y0: y, h0: h, len, k, s0: this.length});
        this.length += len;
    }
    build(fillet) {
        this.seg = []; this.length = 0;
        const R = MAP.route, n = R.length;
        let px = R[0].x, py = R[0].y;
        for (let i = 1; i < n; ++i) {
            const inX = R[i].x-R[i-1].x, inY = R[i].y-R[i-1].y, hin = Math.atan2(inY, inX);
            let tangent = 0, turn = 0, hout = hin;
            if (i+1 < n) {
                const outX = R[i+1].x-R[i].x, outY = R[i+1].y-R[i].y;
                hout = Math.atan2(outY, outX);
                turn = wrap(hout-hin);
                const room = 0.45*Math.min(hypot(inX, inY), hypot(outX, outY));
                tangent = Math.min(fillet*Math.tan(0.5*Math.abs(turn)), room);
            }
            const ex = R[i].x - tangent*Math.cos(hin), ey = R[i].y - tangent*Math.sin(hin);
            this.add(px, py, hin, hypot(ex-px, ey-py), 0);
            if (tangent > 0.5 && Math.abs(turn) > 0.01) {
                const r = tangent/Math.tan(0.5*Math.abs(turn));
                this.add(ex, ey, hin, Math.abs(turn)*r, (turn > 0 ? 1 : -1)/r);
                this.nodeS[i] = this.length - 0.5*Math.abs(turn)*r;
                px = R[i].x + tangent*Math.cos(hout); py = R[i].y + tangent*Math.sin(hout);
            } else { this.nodeS[i] = this.length; px = R[i].x; py = R[i].y; }
        }
        return this;
    }
    find(s) { let i = 0; while (i+1 < this.seg.length && s >= this.seg[i+1].s0) ++i; return i; }
    static eval(g, u) {
        if (!g.k) return {x: g.x0+u*Math.cos(g.h0), y: g.y0+u*Math.sin(g.h0), h: g.h0, k: 0};
        const h = g.h0 + g.k*u;
        return {x: g.x0+(Math.sin(h)-Math.sin(g.h0))/g.k, y: g.y0-(Math.cos(h)-Math.cos(g.h0))/g.k, h: wrap(h), k: g.k};
    }
    at(s) { const g = this.seg[this.find(s)]; return Path.eval(g, bound(s-g.s0, 0, g.len)); }
    curvature(s) { return (s < 0 || s > this.length) ? 0 : this.seg[this.find(s)].k; }
    static local(g, x, y) {
        if (!g.k) {
            const c = Math.cos(g.h0), s = Math.sin(g.h0), dx = x-g.x0, dy = y-g.y0;
            return {u: dx*c + dy*s, e: dy*c - dx*s};
        }
        const r = 1/g.k, cx = g.x0 - r*Math.sin(g.h0), cy = g.y0 + r*Math.cos(g.h0);
        const ax = g.x0-cx, ay = g.y0-cy, bx = x-cx, by = y-cy;
        return {u: Math.atan2(ax*by-ay*bx, ax*bx+ay*by)*r, e: (Math.abs(r) - hypot(bx, by))*(g.k > 0 ? 1 : -1)};
    }
    track(ref, x, y) {  // ref.i: segment index, only moves forward
        let l;
        for (;;) {
            l = Path.local(this.seg[ref.i], x, y);
            if (l.u <= this.seg[ref.i].len || ref.i+1 >= this.seg.length) break;
            ++ref.i;
        }
        return {s: this.seg[ref.i].s0 + l.u, lateral: l.e};
    }
    nearest(x, y, from, to) {
        let best = 1e9, out = null;
        for (let i = this.find(from); i < this.seg.length && this.seg[i].s0 <= to; ++i) {
            const g = this.seg[i], l = Path.local(g, x, y), uc = bound(l.u, 0, g.len), p = Path.eval(g, uc);
            const d = hypot(x-p.x, y-p.y);
            if (d < best) { best = d; out = {s: g.s0+uc, lat: l.e < 0 ? -d : d}; }
        }
        return out;
    }
}

// ---- src/static_route/avoid.h ----
const BoxHalf = BOX/2, BodyHalfWidth = MAP.robotWidth/2, BodyHalfLength = MAP.robotLength/2;
const HoldBeforeMm = BoxHalf + BodyHalfLength - 15, HoldAfterMm = HoldBeforeMm;
const RampMm = 300, MinRampMm = 100, MaxOffsetMm = 260, RampCurv = 0.01, PlanCurvMax = 0.75/60, PlanStepMm = 25;
const SnapMm = 45, MergeMm = 90, HitMergeMm = 30, DecideMm = HoldBeforeMm + MinRampMm + 120;
const ConfirmHits = 3, ForgetMs = 2000, MaxObstacles = 16;
const rampFor = d => bound(Math.sqrt(Math.abs(d)*Pi*Pi/(2*RampCurv)), MinRampMm, RampMm);
const ZERO = {d: 0, slope: 0, curv: 0};

class Avoidance {
    constructor(o) { Object.assign(this, {offset: 0, s0: 0, s1: 0, s2: 0, s3: 0, id: 0, obstacle: 0, active: false}, o); }
    at(s) {
        if (!this.active || s <= this.s0 || s >= this.s3) return ZERO;
        if (s >= this.s1 && s <= this.s2) return {d: this.offset, slope: 0, curv: 0};
        const inn = s < this.s1, len = inn ? this.s1-this.s0 : this.s3-this.s2, a = Pi/len, sign = inn ? 1 : -1;
        const t = (inn ? s-this.s0 : s-this.s2)/len, c = Math.cos(Pi*t), sn = Math.sin(Pi*t);
        return {d: this.offset*0.5*(1-sign*c), slope: sign*this.offset*0.5*a*sn, curv: sign*this.offset*0.5*a*a*c};
    }
}
function shiftedCurv(k, sh) {
    if (!sh.d && !sh.slope && !sh.curv) return k;
    const A = 1 - k*sh.d, B = sh.slope, q = A*A + B*B;
    return (k*A*A + A*sh.curv + 2*k*B*B)/(q*Math.sqrt(q));
}
const shiftedHeading = (h, k, sh) => (!sh.d && !sh.slope) ? h : wrap(h + Math.atan2(sh.slope, 1 - k*sh.d));
const boxDistance = (x, y, cx, cy, half) => hypot(Math.max(0, Math.abs(x-cx)-half), Math.max(0, Math.abs(y-cy)-half));
function footprintGap(x, y, c, s, cx, cy, hx, hy) {
    const L = BodyHalfLength, W = BodyHalfWidth;
    const dx = cx-x, dy = cy-y, ac = Math.abs(c), as = Math.abs(s);
    const reachX = L*ac + W*as, reachY = L*as + W*ac;
    if (Math.abs(dx) > hx + reachX + 100 || Math.abs(dy) > hy + reachY + 100) return 100;
    const u = dx*c + dy*s, v = dy*c - dx*s;
    if (Math.abs(dx) <= hx + reachX && Math.abs(dy) <= hy + reachY &&
        Math.abs(u) <= L + hx*ac + hy*as && Math.abs(v) <= W + hx*as + hy*ac) return 0;
    let best = 1e18;
    for (let i = 0; i < 4; ++i) {
        const a = (i & 1) ? L : -L, b = (i & 2) ? W : -W;
        const ex = Math.max(0, Math.abs(a*c - b*s - dx) - hx), ey = Math.max(0, Math.abs(a*s + b*c - dy) - hy);
        best = Math.min(best, ex*ex + ey*ey);
        const px = ((i & 1) ? hx : -hx) + dx, py = ((i & 2) ? hy : -hy) + dy;
        const fu = Math.max(0, Math.abs(px*c + py*s) - L), fv = Math.max(0, Math.abs(py*c - px*s) - W);
        best = Math.min(best, fu*fu + fv*fv);
    }
    return Math.sqrt(best);
}

class ObstacleMap {
    constructor(clear) { this.items = []; this.overflow = false; this.c = clear; }
    absorbTwins(slot) {
        for (const o of this.items) {
            if (o === slot || o.kind !== 'U' || Math.abs(o.x-slot.x) >= BOX || Math.abs(o.y-slot.y) >= BOX) continue;
            if ((o.planned || o.frozen) && (slot.planned || slot.frozen)) continue;
            const keep = (o.planned || o.frozen) ? o : slot, drop = keep === slot ? o : slot;
            if (!keep.frozen) {
                const w = drop.samples/Math.max(1, keep.samples+drop.samples);
                keep.x += w*(drop.x-keep.x); keep.y += w*(drop.y-keep.y);
            }
            keep.samples = Math.min(255, keep.samples+drop.samples);
            keep.hits = Math.max(keep.hits, drop.hits);
            keep.confirmed = keep.confirmed || drop.confirmed;
            keep.lastAt = slot.lastAt;
            drop.kind = 'X'; drop.confirmed = drop.planned = false;
            slot = keep;
        }
        return slot;
    }
    classify(hx, hy, dx, dy, now) {
        const mk = (x, y, id, kind) => ({x, y, lastAt: now, id, hits: 0, kind, confirmed: false, planned: false, frozen: false, samples: 0});
        for (const o of MAP.fixed) if (boxDistance(hx, hy, o.x, o.y, BoxHalf) < SnapMm) return mk(o.x, o.y, o.id, 'F');
        for (const o of MAP.possible) if (boxDistance(hx, hy, o.x, o.y, BoxHalf) < SnapMm) return mk(o.x, o.y, o.id, 'R');
        if (Math.abs(dx) > Math.abs(dy)) return mk(hx + (dx > 0 ? BoxHalf : -BoxHalf), hy, 0, 'U');
        return mk(hx, hy + (dy > 0 ? BoxHalf : -BoxHalf), 0, 'U');
    }
    freeSlot(now) {
        if (this.items.length < MaxObstacles) { const o = {}; this.items.push(o); return o; }
        let slot = null;
        for (const o of this.items) if ((o.kind === 'X' || !o.confirmed) && (!slot || o.lastAt-slot.lastAt < 0)) slot = o;
        if (slot) return slot;
        for (const o of this.items)
            if (!o.planned && !o.frozen && now-o.lastAt > ForgetMs && (!slot || o.lastAt-slot.lastAt < 0)) slot = o;
        return slot;
    }
    note(seen, hx, hy, now) {
        if (seen.kind === 'F') return null;
        let slot = null;
        for (const o of this.items) {
            const same = seen.id ? o.id === seen.id
                : o.kind === 'U' && (boxDistance(hx, hy, o.x, o.y, BoxHalf) < HitMergeMm || hypot(o.x-seen.x, o.y-seen.y) < MergeMm);
            if (same) { slot = o; break; }
        }
        if (!slot) {
            slot = this.freeSlot(now);
            if (!slot) { this.overflow = true; return null; }
            for (const k of Object.keys(slot)) delete slot[k];
            Object.assign(slot, seen);
        } else if (slot.kind === 'U' && !slot.frozen) {
            const w = 1/Math.min(slot.samples+1, 8);
            slot.x += w*(seen.x-slot.x); slot.y += w*(seen.y-slot.y);
        }
        if (slot.samples < 255) ++slot.samples;
        if (slot.kind === 'U') slot = this.absorbTwins(slot);
        if (now-slot.lastAt > 300) slot.hits = 0;
        slot.lastAt = now;
        if (slot.hits < 255) ++slot.hits;
        if (slot.confirmed || slot.hits < ConfirmHits) return null;
        slot.confirmed = true;
        return slot;
    }
    clearance(x, y, h) {
        const c = Math.cos(h), s = Math.sin(h);
        let gap = 1e9, randomFound = false;
        for (const o of MAP.fixed) gap = Math.min(gap, footprintGap(x, y, c, s, o.x, o.y, BoxHalf, BoxHalf));
        for (const o of this.items) {
            if (!o.confirmed) continue;
            randomFound = randomFound || o.kind === 'R';
            gap = Math.min(gap, footprintGap(x, y, c, s, o.x, o.y, BoxHalf, BoxHalf));
        }
        if (!randomFound) for (const o of MAP.possible) gap = Math.min(gap, footprintGap(x, y, c, s, o.x, o.y, BoxHalf, BoxHalf));
        const f = MAP.forbidden;
        gap = Math.min(gap, footprintGap(x, y, c, s, 0.5*(f.xmin+f.xmax), 0.5*(f.ymin+f.ymax), 0.5*(f.xmax-f.xmin), 0.5*(f.ymax-f.ymin)));
        const reachX = BodyHalfLength*Math.abs(c) + BodyHalfWidth*Math.abs(s);
        const reachY = BodyHalfLength*Math.abs(s) + BodyHalfWidth*Math.abs(c);
        gap = Math.min(gap, x - reachX, MAP.width - x - reachX);
        return Math.min(gap, MAP.height - y - reachY);
    }
}
function checkDetour(path, map, a, from, to, margin) {
    let worst = 1e9;
    for (let s = from; s <= to; s += PlanStepMm) {
        const p = path.at(s), sh = a.at(s);
        if (1 - p.k*sh.d < 0.5 || Math.abs(shiftedCurv(p.k, sh)) > PlanCurvMax) return {ok: false, worst};
        const h = shiftedHeading(p.h, p.k, sh);
        worst = Math.min(worst, map.clearance(p.x - Math.sin(p.h)*sh.d, p.y + Math.cos(p.h)*sh.d, h));
        if (worst < margin) return {ok: false, worst};
    }
    return {ok: true, worst};
}
function planAvoid(path, map, id, from, sObs, latObs, passMm, margin) {
    const s1 = sObs - HoldBeforeMm, s2 = sObs + HoldAfterMm;
    const room = s1 - from, roomAfter = path.length - 40 - s2;
    let best = 1e9, out = null, tooClose = false;
    for (let side = 0; side < 2; ++side) {
        const offset = side ? latObs - passMm : latObs + passMm;
        if (Math.abs(offset) > MaxOffsetMm) continue;
        const ramp = rampFor(offset);
        if (roomAfter < ramp) continue;
        const a = new Avoidance({offset, id, active: true, s1, s2,
            s0: s1 - Math.max(ramp, Math.min(RampMm, room)), s3: s2 + Math.min(RampMm, roomAfter)});
        const chk = checkDetour(path, map, a, a.s0, a.s3, margin);
        if (!chk.ok) continue;
        if (room < ramp) { tooClose = true; continue; }
        const score = Math.abs(offset) - 0.2*chk.worst;
        if (score < best) { best = score; out = a; }
    }
    return out ? {result: 'Ok', plan: out} : {result: tooClose ? 'TooClose' : 'NoRoom'};
}

// ---- src/test_static_route.cpp: wheels ----
const RouteKp = [0.018, 0.018], RouteKi = [0.08, 0.08], RoutePwmMax = 85, RouteIntegralMax = 30;
function routeFeedforward(i, dir, magnitude) {
    const cps = [0, 200, 400, 700, 2000], pwmL = [0, 12.5, 16.5, 25.5, 63.5], pwmR = [0, 15.5, 20.5, 29.5, 67.5];
    if (dir < 0) return (i ? 17.5 : 12.0)*bound(magnitude, 0, 200)/200;
    const pwm = i ? pwmR : pwmL;
    for (let k = 1; k < 5; ++k)
        if (magnitude <= cps[k]) return pwm[k-1] + (pwm[k]-pwm[k-1])*(magnitude-cps[k-1])/(cps[k]-cps[k-1]);
    return pwm[4];
}
class Wheel {
    constructor() { Object.assign(this, {cps: 0, integral: 0, duty: 0, targetCps: 0, lastPulse: 0, stoppedAt: 0, startedAt: 0, direction: 0, wrong: 0, applied: 0, watching: false}); }
    stop(now) {
        if (this.applied || this.duty > 0) this.stoppedAt = now;
        this.duty = this.integral = this.targetCps = 0;
        this.wrong = this.applied = 0; this.watching = false;
    }
    observe(ticks, dt, now) {
        const tau = Math.abs(this.targetCps) <= 400 ? 0.065 : 0.040;
        this.cps += dt/(tau+dt)*(ticks/dt-this.cps);
        if (!this.watching) return true;
        const forward = ticks*this.direction;
        if (forward > 0) { this.lastPulse = now; this.wrong = 0; }
        if (forward < 0) this.wrong -= forward;
        return now-this.lastPulse < 750 && (now-this.startedAt < 125 || this.wrong < 24);
    }
    output(ref, index, dt, now) {
        if (Math.abs(ref) < 5) { this.stop(now); return 0; }
        const sign = ref > 0 ? 1 : -1;
        if (this.direction && this.direction !== sign) {
            if (this.applied || this.duty > 0) { this.stop(now); return 0; }
            if (now-this.stoppedAt < 50 || Math.abs(this.cps) > 30) return 0;
            this.integral = 0; this.wrong = 0; this.watching = false;
        }
        if (this.direction !== sign) { this.direction = sign; this.watching = false; }
        this.targetCps = ref;
        const magnitude = Math.abs(ref);
        if (!this.watching) { this.watching = true; this.lastPulse = this.startedAt = now; this.wrong = 0; }
        const error = magnitude - sign*this.cps;
        const ff = routeFeedforward(index, sign, magnitude);
        const nextI = bound(this.integral + RouteKi[index]*error*dt, -RouteIntegralMax, RouteIntegralMax);
        const lo = Math.max(0, this.duty-150*dt), hi = Math.min(RoutePwmMax, this.duty+90*dt);
        const proposed = ff + RouteKp[index]*error + nextI;
        if ((proposed >= lo && proposed <= hi) || (proposed > hi && error < 0) || (proposed < lo && error > 0)) this.integral = nextI;
        this.duty = bound(ff + RouteKp[index]*error + this.integral, lo, hi);
        this.applied = sign*lround(this.duty);
        return this.applied;
    }
}

// VL53L0X mounts (J8..J11): forward/left of the centre, yaw (+ left).
const MOUNTS = [{forward: 45, left: 25, yaw: 35.43*Rad}, {forward: 45, left: 25, yaw: 0},
                {forward: 45, left: -25, yaw: 0}, {forward: 45, left: -25, yaw: -35.43*Rad}];
function rayBox(ox, oy, dx, dy, b) {
    const o = [ox, oy], d = [dx, dy], lo = [b.x-BoxHalf, b.y-BoxHalf], hi = [b.x+BoxHalf, b.y+BoxHalf];
    let t0 = 0, t1 = 1e9;
    for (let a = 0; a < 2; ++a) {
        if (Math.abs(d[a]) < 1e-9) { if (o[a] < lo[a] || o[a] > hi[a]) return -1; continue; }
        let ta = (lo[a]-o[a])/d[a], tb = (hi[a]-o[a])/d[a];
        if (ta > tb) [ta, tb] = [tb, ta];
        t0 = Math.max(t0, ta); t1 = Math.min(t1, tb);
        if (t0 > t1) return -1;
    }
    return t0;
}
function rng(seed) {  // mulberry32 + Box-Muller, deterministic sensor noise
    let a = seed >>> 0;
    const u = () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0)/4294967296; };
    return () => Math.sqrt(-2*Math.log(u() || 1e-12))*Math.cos(2*Pi*u());
}

// ---- src/static_route/localize.h ----
function beamFrom(p, i) {
    const m = MOUNTS[i], c = Math.cos(p.h), s = Math.sin(p.h);
    return {ox: p.x + m.forward*c - m.left*s, oy: p.y + m.forward*s + m.left*c, dx: Math.cos(p.h+m.yaw), dy: Math.sin(p.h+m.yaw)};
}
function castFixed(ox, oy, dx, dy) {
    let best = {t: 1e9, nx: 0, ny: 0, box: -1};
    MAP.fixed.forEach((o, b) => {
        const org = [ox, oy], d = [dx, dy], lo = [o.x-BOX/2, o.y-BOX/2], hi = [o.x+BOX/2, o.y+BOX/2];
        let t0 = 0, t1 = 1e9, axis = -1, ok = true;
        for (let a = 0; a < 2 && ok; ++a) {
            if (Math.abs(d[a]) < 1e-6) { ok = org[a] >= lo[a] && org[a] <= hi[a]; continue; }
            let ta = (lo[a]-org[a])/d[a], tb = (hi[a]-org[a])/d[a];
            if (ta > tb) [ta, tb] = [tb, ta];
            if (ta > t0) { t0 = ta; axis = a; }
            t1 = Math.min(t1, tb); ok = t0 <= t1;
        }
        if (ok && axis >= 0 && t0 < best.t) best = {t: t0, nx: axis === 0 ? (dx > 0 ? -1 : 1) : 0, ny: axis === 1 ? (dy > 0 ? -1 : 1) : 0, box: b};
    });
    return best;
}
function expectBeam(p, i, detectMm) {
    const r = beamFrom(p, i), h = castFixed(r.ox, r.oy, r.dx, r.dy), c = -(r.dx*h.nx + r.dy*h.ny);
    if (h.box < 0 || h.t > detectMm || c < 0.35) return null;
    const m = MOUNTS[i], cs = Math.cos(p.h), sn = Math.sin(p.h);
    const dox = -m.forward*sn - m.left*cs, doy = m.forward*cs - m.left*sn;
    return {t: h.t, box: h.box, J: [h.nx/c, h.ny/c, (dox*h.nx + doy*h.ny + h.t*(-r.dy*h.nx + r.dx*h.ny))/c]};
}
function nearestFixed(qx, qy, mask) {
    let best = {d: 1e9, nx: 0, ny: 0};
    MAP.fixed.forEach((o, b) => {
        if (!mask[b]) return;
        const ex = qx-o.x, ey = qy-o.y, ox = Math.abs(ex)-BOX/2, oy = Math.abs(ey)-BOX/2;
        let d, mx, my;
        if (ox > 0 || oy > 0) { const cx = Math.max(ox, 0), cy = Math.max(oy, 0); d = hypot(cx, cy); mx = cx/d*(ex < 0 ? -1 : 1); my = cy/d*(ey < 0 ? -1 : 1); }
        else if (ox > oy) { d = ox; mx = ex < 0 ? -1 : 1; my = 0; } else { d = oy; mx = 0; my = ey < 0 ? -1 : 1; }
        if (Math.abs(d) < Math.abs(best.d)) best = {d, nx: mx, ny: my};
    });
    return best;
}
const det3 = M => M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1]) - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0]) + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
// avg: averaged beam per sensor (NaN = fewer than 6 samples). Returns {pose, status, used, rms}.
function fitStart(avg, start, detectMm) {
    const wr = 1/36, wp = 1/1600, wh = 1/((8*Rad)*(8*Rad));
    const mask = MAP.fixed.map(o => hypot(o.x-start.x, o.y-start.y) < detectMm+200);
    let best = 1e9, bestP = null;
    const f = {pose: Object.assign({}, start), status: 'NO_DATA', used: 0, rms: 0};
    for (let s = 0; s < 3 && best > 2; ++s) {
        const p = {x: start.x, y: start.y, h: wrap(start.h + [0, 6, -6][s]*Rad)};
        let cost = 0, used = 0, rms = 0;
        for (let it = 0; it < 6; ++it) {
            const A = [[wp, 0, 0], [0, wp, 0], [0, 0, wh]];
            const g = [-(p.x-start.x)*wp, -(p.y-start.y)*wp, -wrap(p.h-start.h)*wh];
            let sq = 0, bad = 0; used = 0;
            const cs = Math.cos(p.h), sn = Math.sin(p.h);
            for (let i = 0; i < 4; ++i) {
                if (!Number.isFinite(avg[i])) continue;
                const m = avg[i], mt = MOUNTS[i], r = beamFrom(p, i), n = nearestFixed(r.ox + m*r.dx, r.oy + m*r.dy, mask);
                if (Math.abs(n.d) > 60) { bad++; continue; }
                const J = [n.nx, n.ny, n.nx*(-mt.forward*sn - mt.left*cs - m*r.dy) + n.ny*(mt.forward*cs - mt.left*sn + m*r.dx)];
                for (let a = 0; a < 3; ++a) { g[a] -= wr*J[a]*n.d; for (let b = 0; b < 3; ++b) A[a][b] += wr*J[a]*J[b]; }
                sq += n.d*n.d; used++;
            }
            const ex = p.x-start.x, ey = p.y-start.y, eh = wrap(p.h-start.h);
            cost = wr*sq + wp*(ex*ex+ey*ey) + wh*eh*eh + 25*bad; rms = used ? Math.sqrt(sq/used) : 0;
            if (it === 5) break;
            const D = det3(A), st = [0, 1, 2].map(k => det3(A.map((row, a) => row.map((v, b) => b === k ? g[a] : v)))/D);
            p.x += bound(st[0], -30, 30); p.y += bound(st[1], -30, 30); p.h = wrap(p.h + bound(st[2], -0.1, 0.1));
        }
        if (used >= 2 && cost < best) { best = cost; bestP = p; f.used = used; f.rms = rms; }
    }
    if (f.used < 2) return f;
    const far = Math.abs(bestP.x-start.x) > 50 || Math.abs(bestP.y-start.y) > 50 || Math.abs(wrap(bestP.h-start.h)) > 15*Rad || f.rms > 25;
    f.status = far ? 'FAR' : 'OK';
    if (!far) f.pose = bestP;
    return f;
}

// One complete run. opts: {tune, countsPerMm:[l,r], track, random:0|14..17, unknown:[{x,y}],
// trueCpmErr:[%l,%r], trueTrackErr:%, rangeNoise:mm, seed, maxMs, raycastFrom:'true'|'estimate',
// t0, events:[{atStation, ahead}] (boxes that appear ahead of the car),
// trueGyroErr:% (gyro reads this much high), place:{dx, dy, dhDeg} (hand-placement error)}.
function simulate(opts = {}) {
    const tune = Object.assign(defaultTune(), opts.tune || {});
    const CountsPerMm = opts.countsPerMm || [4.5, 4.5], Track = opts.track || 90;
    const MaxCountsPerMm = Math.max(CountsPerMm[0], CountsPerMm[1]);
    const RouteAccel = 700/MaxCountsPerMm, RouteDecel = 900/MaxCountsPerMm, PlanDecel = 0.8*RouteDecel;
    const TurnCps = 200, LateralSatMm = 60, MaxCurv = 1/60, AlignStartDeg = 35, OffPathMm = 100;
    const HardStopMm = 90, LookAheadMm = 900, CoastDecelMm = 300;
    const FootMarginMm = tune.safeMarginMm, ClearMm = BodyHalfWidth + tune.safeMarginMm;
    const PassMm = BoxHalf*1.2 + ClearMm, BlockMm = PassMm - 10;
    const cpmErr = opts.trueCpmErr || [0, 0];
    const trueCpm = [CountsPerMm[0]*(1+cpmErr[0]/100), CountsPerMm[1]*(1+cpmErr[1]/100)];
    const trueTrack = Track*(1+(opts.trueTrackErr || 0)/100);
    const noise = opts.rangeNoise ? rng(opts.seed || 1) : null;
    const fromEstimate = opts.raycastFrom === 'estimate';

    // World: fixed MAP boxes, the random candidate that appeared, unknown boxes.
    const world = MAP.fixed.map(o => ({x: o.x, y: o.y, kind: 'F', id: o.id}));
    for (const o of MAP.possible) if (o.id === opts.random) world.push({x: o.x, y: o.y, kind: 'R', id: o.id});
    for (const u of opts.unknown || []) world.push({x: u.x, y: u.y, kind: 'U', id: 0});
    const pending = (opts.events || []).map(e => Object.assign({done: false}, e));

    // Firmware state
    const StartPose = {x: MAP.start.x, y: MAP.start.y, h: MAP.start.headingDeg*Rad};
    let pose = Object.assign({}, StartPose), truth = Object.assign({}, StartPose);
    const pl = opts.place || {};
    truth = {x: StartPose.x + (pl.dx || 0), y: StartPose.y + (pl.dy || 0), h: wrap(StartPose.h + (pl.dhDeg || 0)*Rad)};
    let positionError = 5, headingError = 2*Rad;
    let request = {v: 0, w: 0}, yawRate = 0, lastRate = 0;
    const wheels = [new Wheel(), new Wheel()];
    let travelled = 0, slipTime = 0, history = [], state = 'RUN_ROUTE', legPhase = 'TRACK', stopReason = 'RUNNING';
    let legPhaseAt = 0, lastProgressAt = 0, stillSince = 0, compareAt = 0, lastLogAt = 0, runAt = 0;
    let target = 1, bestProgress = 1e6, encoderTurn = 0, gyroTurn = 0, maxCross = 0, settlingStill = false;
    let remaining = 0, cross = 0, angleError = 0, targetDistance = 0, station = 0, offsetNow = 0, refCurv = 0;
    const path = new Path().build(tune.filletMm), segRef = {i: 0};
    const obstacles = new ObstacleMap();
    let avoid = [new Avoidance(), new Avoidance()];
    let fixedSeen = new Set(), reviewAt = 0;
    const ranges = [0, 1, 2, 3].map(() => ({mm: 0xFFFF, at: 0, sampled: false}));
    const rangeSeenAt = [0, 0, 0, 0];
    // Rig state
    let driven = [0, 0], actualCps = [0, 0], residual = [0, 0], trueV = [0, 0];
    const events = [], trace = [];
    let minGap = 1e9, minGapAt = null, collision = null;

    const ev = (t, type, data) => events.push(Object.assign({t: t-runAt, type}, data));
    const speed = i => wheels[i].cps/CountsPerMm[i];
    const shiftAt = s => { const a = avoid[0].at(s), b = avoid[1].at(s); return {d: a.d+b.d, slope: a.slope+b.slope, curv: a.curv+b.curv}; };
    const avoiding = s => avoid.some(a => a.active && s > a.s0 && s < a.s3);
    function cutDrive() { driven = [0, 0]; }
    function stopDrive(now) { cutDrive(); request = {v: 0, w: 0}; for (const w of wheels) w.stop(now); }
    function fault(why, now) {
        if (state === 'FAULT') return;
        stopDrive(now); state = 'FAULT'; stopReason = why;
        ev(now, 'END', {event: 'ROUTE_FAULT', reason: why});
    }
    function setLegPhase(next, now) { legPhase = next; legPhaseAt = lastProgressAt = now; bestProgress = 1e6; settlingStill = false; }
    function rememberPose(now) { history.unshift({p: Object.assign({}, pose), at: now}); if (history.length > 12) history.pop(); }
    function sampleOrigin(at) {
        let r = pose;
        for (const h of history) { r = h.p; if (at-h.at >= 0) break; }
        return r;
    }
    function predict(left, right, dt, now) {
        const dl = left/CountsPerMm[0], dr = right/CountsPerMm[1];
        const ds = (dl+dr)*0.5, encTurn = (dr-dl)/Track;
        const gTurn = (yawRate+lastRate)*0.5*dt;
        lastRate = yawRate;
        const mismatch = Math.abs(encTurn-gTurn), slipping = mismatch > 0.012+0.55*dt;
        slipTime = bound(slipTime+(slipping ? dt : -0.45*dt), 0, 1);
        const still = !left && !right && Math.abs(yawRate) < 0.45*Rad;
        const gw = slipping ? 0.96 : 0.82;
        const dh = still ? 0 : gw*gTurn+(1-gw)*encTurn;
        pose.x += ds*Math.cos(pose.h+dh*0.5); pose.y += ds*Math.sin(pose.h+dh*0.5); pose.h = wrap(pose.h+dh);
        travelled += Math.abs(ds);
        headingError = Math.min(0.6, headingError+0.003*Math.abs(dh)+0.00018*dt+0.027*mismatch);
        positionError = Math.min(150, positionError+0.0035*Math.abs(ds)+0.12*mismatch+0.012*slipping);
        rememberPose(now);
    }
    function logAvoid(a, now, final) { ev(now, 'AVOID', {id: a.id, side: a.offset > 0 ? 'LEFT' : 'RIGHT', offset: a.offset, s0: a.s0, s1: a.s1, s2: a.s2, s3: a.s3, final}); }
    function updateErrors(now) {
        const tr = path.track(segRef, pose.x, pose.y);
        station = tr.s;
        for (const a of avoid) {
            const o = obstacles.items[a.obstacle];
            if (a.active && o && !o.frozen && (station >= a.s0 || 0.5*(a.s1+a.s2) - station <= DecideMm)) { o.frozen = true; logAvoid(a, now, true); }
            if (a.active && station > a.s3) a.active = false;
        }
        const ref = path.at(station), sh = shiftAt(station);
        offsetNow = sh.d; cross = tr.lateral - sh.d;
        refCurv = shiftedCurv(ref.k, sh);
        angleError = wrap(shiftedHeading(ref.h, ref.k, sh) - pose.h);
        remaining = path.length - station;
        targetDistance = hypot(pose.x-MAP.goal.x, pose.y-MAP.goal.y);
        maxCross = Math.max(maxCross, Math.abs(cross));
    }
    function passNodes(now) {
        while (target+1 < MAP.route.length && station >= path.nodeS[target]) { ev(now, 'WAYPOINT', {node: target}); ++target; }
    }
    const advancing = (metric, step, now) => { if (metric < bestProgress-step) { bestProgress = metric; lastProgressAt = now; } return now-lastProgressAt < 4500; };
    const slew = (old, next, rise, fall, dt) => old + bound(next-old, -fall*dt, rise*dt);
    function curveSpeed(k, s) {
        const a = Math.abs(k), cps = avoiding(s) ? Math.min(tune.avoidCps, tune.cruiseCps) : tune.cruiseCps;
        const wheelCap = cps/MaxCountsPerMm/(1 + 0.5*Track*a);
        return a < 1e-5 ? wheelCap : Math.min(wheelCap, Math.sqrt(tune.latAccel/a));
    }
    function plannedSpeed() {
        let v = curveSpeed(refCurv, station);
        for (let ds = 25; ds <= 350 && station+ds < path.length; ds += 25) {
            const s = station+ds, cap = curveSpeed(shiftedCurv(path.curvature(s), shiftAt(s)), s);
            v = Math.min(v, Math.sqrt(cap*cap + 2*PlanDecel*ds));
        }
        const brake = Math.sqrt(2*RouteDecel*Math.max(0, remaining - BRAKE_AT_MM*0.5));
        const approach = Math.min(NEAR_SPEED_MM_S, tune.approachCps/MaxCountsPerMm) + Math.max(0, remaining-NEAR_DISTANCE_MM)*0.9;
        v = Math.min(v, approach, brake);
        return v*bound(1-Math.abs(angleError)/(60*Rad), 0.25, 1);
    }
    function follow(dt, now) {
        updateErrors(now);
        if (legPhase === 'SETTLE') {
            request = {v: 0, w: 0};
            const still = Math.abs(speed(0)) <= STILL_MM_S && Math.abs(speed(1)) <= STILL_MM_S && Math.abs(yawRate) <= STILL_DEG_S*Rad;
            if (!still) settlingStill = false; else if (!settlingStill) { stillSince = now; settlingStill = true; }
            if (now-legPhaseAt > 1800) { fault('NOT_SETTLED', now); return; }
            if (!settlingStill || now-stillSince < SETTLE_MS) return;
            if (targetDistance > POSITION_TOL_MM) { fault('MISSED_WAYPOINT', now); return; }
            ev(now, 'WAYPOINT', {node: target});
            stopDrive(now); stopReason = 'GOAL_REACHED'; state = 'FINISH';
            ev(now, 'END', {event: 'ROUTE_FINISHED', reason: stopReason});
            return;
        }
        if (legPhase === 'ALIGN') {
            const turnCap = Math.min(TURN_RAD_S, 2*TurnCps/(Track*MaxCountsPerMm));
            const turn = bound(3.4*angleError - 0.15*yawRate, -turnCap, turnCap);
            if (!advancing(Math.abs(angleError), Rad, now) || now-legPhaseAt > 10000) { fault('TURN_NO_PROGRESS', now); return; }
            request.v = 0;
            const aligned = Math.abs(angleError) <= ANGLE_TOL_DEG*Rad;
            request.w = slew(request.w, aligned ? 0 : turn, ANG_ACCEL_RAD_S2, ANG_ACCEL_RAD_S2, dt);
            if (aligned && Math.abs(yawRate) <= STILL_DEG_S*Rad && Math.abs(request.w) < 0.001 &&
                Math.abs(speed(0)) <= STILL_MM_S && Math.abs(speed(1)) <= STILL_MM_S) setLegPhase('TRACK', now);
            return;
        }
        passNodes(now);
        if (remaining <= BRAKE_AT_MM && targetDistance <= POSITION_TOL_MM) { stopDrive(now); setLegPhase('SETTLE', now); return; }
        if (remaining <= 0) { fault('MISSED_WAYPOINT', now); return; }
        if (Math.abs(cross) > OffPathMm || Math.abs(angleError) > 75*Rad) { fault('OFF_PATH', now); return; }
        if (!advancing(remaining, 3, now)) { fault('NO_TRACK_PROGRESS', now); return; }
        request.v = slew(request.v, plannedSpeed(), RouteAccel, RouteDecel, dt);
        const curv = bound(refCurv - tune.lateralGain*bound(cross, -LateralSatMm, LateralSatMm) + tune.headingGain*Math.sin(angleError), -MaxCurv, MaxCurv);
        const wLimit = 1.7*request.v/Track;
        request.w = bound(slew(request.w, request.v*curv, ANG_ACCEL_RAD_S2, ANG_ACCEL_RAD_S2, dt), -wLimit, wLimit);
    }
    function actuate(now, dt) {
        if (state !== 'RUN_ROUTE' || (!request.v && !request.w)) { stopDrive(now); return; }
        const cap = legPhase === 'ALIGN' ? TurnCps :
            ((remaining <= NEAR_DISTANCE_MM || targetDistance <= NEAR_DISTANCE_MM) ? tune.approachCps :
             avoiding(station) ? Math.min(tune.avoidCps, tune.cruiseCps) : tune.cruiseCps);
        const halfDiff = 0.5*Track*request.w;
        const peak = Math.max(Math.abs((request.v-halfDiff)*CountsPerMm[0]), Math.abs((request.v+halfDiff)*CountsPerMm[1]));
        if (peak > cap) { request.v *= cap/peak; request.w *= cap/peak; }
        const diff = 0.5*Track*request.w;
        const left = wheels[0].output((request.v-diff)*CountsPerMm[0], 0, dt, now);
        const right = wheels[1].output((request.v+diff)*CountsPerMm[1], 1, dt, now);
        driven = (!left && !right) ? [0, 0] : [left, right];
    }
    function announce(o, now) {
        const near = path.nearest(o.x, o.y, station, station+LookAheadMm);
        ev(now, 'OBSTACLE', {id: o.id, kind: o.kind === 'F' ? 'FIXED' : o.kind === 'R' ? 'RANDOM' : 'UNKNOWN', x: o.x, y: o.y, s: near ? near.s : -1, lat: near ? near.lat : 0});
    }
    function reviewObstacles(now) {
        if (now-reviewAt < 50) return;
        reviewAt = now;
        const items = obstacles.items;
        for (let i = 0; i < items.length && state === 'RUN_ROUTE'; ++i) {
            const o = items[i];
            if (!o.confirmed || o.frozen || (o.kind !== 'U' && o.kind !== 'R')) continue;
            let own = null;
            for (const a of avoid) if (a.active && a.obstacle === i) own = a;
            if (own && station >= own.s0) { o.frozen = true; continue; }
            const near = path.nearest(o.x, o.y, station, station+LookAheadMm);
            if (!near || near.s <= station+1) continue;
            const {s, lat} = near;
            if (Math.abs(lat - (own ? 0 : shiftAt(s).d)) >= BlockMm) { if (own) { own.active = false; o.planned = false; } continue; }
            const slotIdx = own ? avoid.indexOf(own) : !avoid[0].active ? 0 : !avoid[1].active ? 1 : -1;
            let from = own ? own.s0 : station + 10;
            for (const a of avoid) if (a.active && a !== own) from = Math.max(from, a.s3);
            const r = slotIdx >= 0 ? planAvoid(path, obstacles, o.id, from, s, lat, PassMm, FootMarginMm) : {result: 'NoRoom'};
            if (r.result === 'Ok') {
                r.plan.obstacle = i; avoid[slotIdx] = r.plan; o.planned = true;
                if (!own) logAvoid(r.plan, now, false);
                return;
            }
            if (own && r.result === 'TooClose' && checkDetour(path, obstacles, own, Math.max(station, own.s0), own.s3, FootMarginMm).ok) return;
            if (own) { own.active = false; o.planned = false; }
            if (r.result === 'NoRoom' && s-station > DecideMm) return;
            const late = r.result === 'TooClose';
            ev(now, 'AVOID_FAIL', {id: o.id, reason: late ? 'TOO_CLOSE' : 'NO_ROOM'});
            fault(late ? 'OBSTACLE_TOO_CLOSE' : 'NO_AVOID_PATH', now);
            return;
        }
    }
    function scanRanges(now) {
        for (let i = 0; i < 4 && state === 'RUN_ROUTE'; ++i) {
            const r = ranges[i];
            if (!(r.sampled && now-r.at <= 100)) { fault('RANGE_STALE', now); return; }
            if (r.at === rangeSeenAt[i]) continue;
            rangeSeenAt[i] = r.at;
            if (r.mm === 0xFFFF || r.mm > tune.detectMm) continue;
            const p = sampleOrigin(r.at), m = MOUNTS[i];
            const c = Math.cos(p.h), sn = Math.sin(p.h), dx = Math.cos(p.h+m.yaw), dy = Math.sin(p.h+m.yaw);
            const hx = p.x + m.forward*c - m.left*sn + r.mm*dx, hy = p.y + m.forward*sn + m.left*c + r.mm*dy;
            const inside = hx >= -30 && hy >= -30 && hx <= MAP.width+30 && hy <= MAP.height+30;
            const seen = obstacles.classify(hx, hy, dx, dy, now);
            const v = Math.max(0, 0.5*(speed(0)+speed(1)));
            const known = inside && (seen.kind === 'F' || seen.kind === 'R');
            const stopMm = known ? HardStopMm : HardStopMm + v*v/(2*CoastDecelMm) + v*0.06;
            if ((i === 1 || i === 2) && r.mm < stopMm) { fault('OBSTACLE_STOP', now); return; }
            if (!inside) continue;
            if (seen.kind === 'F') { if (tune.rangeFix > 0.5) fixFromBeam(p, i, r.mm, seen.id); if (!fixedSeen.has(seen.id)) { fixedSeen.add(seen.id); announce(seen, now); } continue; }
            const o = obstacles.note(seen, hx, hy, now);
            if (o) announce(o, now);
            if (obstacles.overflow) { ev(now, 'AVOID_FAIL', {id: 0, reason: 'MAP_FULL'}); fault('OBSTACLE_MAP_FULL', now); return; }
        }
        reviewObstacles(now);
    }
    function control(now, ticks, dt) {
        if (state !== 'RUN_ROUTE') return;
        for (let i = 0; i < 2; ++i) {
            if (Math.abs(ticks[i]) > 8 + 8000*dt) { fault('ENCODER_JUMP', now); return; }
            if (!wheels[i].observe(ticks[i], dt, now)) { fault(i ? 'WHEEL_R_STALL_DIR' : 'WHEEL_L_STALL_DIR', now); return; }
        }
        encoderTurn += (ticks[1]/CountsPerMm[1]-ticks[0]/CountsPerMm[0])/Track;
        gyroTurn += (yawRate+lastRate)*0.5*dt;
        predict(ticks[0], ticks[1], dt, now);
        if (slipTime > 0.22) { fault('WHEEL_SLIP', now); return; }
        if (now-compareAt >= 450) {
            if (Math.abs(encoderTurn-gyroTurn) > 8*Rad + 0.20*Math.abs(encoderTurn)) { fault('GYRO_ENCODER_DISAGREE', now); return; }
            compareAt = now; encoderTurn = gyroTurn = 0;
        }
        updateErrors(now);
        scanRanges(now);
        if (state !== 'RUN_ROUTE') return;
        follow(dt, now);
        actuate(now, dt);
    }
    function fixFromBeam(p, i, mm, seenId) {  // localize.h: scalar Kalman update
        const x = expectBeam(p, i, tune.detectMm);
        if (!x || MAP.fixed[x.box].id !== seenId) return;
        const e = mm - x.t, J = x.J;
        if (Math.abs(e) > 30) return;
        const P = [positionError**2, positionError**2, headingError**2];
        const S = P[0]*J[0]*J[0] + P[1]*J[1]*J[1] + P[2]*J[2]*J[2] + 100;
        const sx = bound(P[0]*J[0]*e/S, -4, 4), sy = bound(P[1]*J[1]*e/S, -4, 4), sh = bound(P[2]*J[2]*e/S, -0.5*Rad, 0.5*Rad);
        pose.x += sx; pose.y += sy; pose.h = wrap(pose.h + sh);
        for (const hh of history) { if (hh.p === pose) continue; hh.p.x += sx; hh.p.y += sy; hh.p.h = wrap(hh.p.h + sh); }
        positionError = Math.max(3, positionError*Math.sqrt(1 - Math.min(0.9, P[0]*(J[0]*J[0]+J[1]*J[1])/S)));
        headingError = Math.max(0.5*Rad, headingError*Math.sqrt(1 - Math.min(0.9, P[2]*J[2]*J[2]/S)));
        fixes.n++; fixes.sum += hypot(sx, sy);
    }
    const fixes = {n: 0, sum: 0};
    function injectRanges(now) {
        const src = truth;
        for (let i = 0; i < 4; ++i) {
            if ((Math.floor(now/10) + i) % 3) continue;
            const m = MOUNTS[i], c = Math.cos(src.h), s = Math.sin(src.h), dx = Math.cos(src.h+m.yaw), dy = Math.sin(src.h+m.yaw);
            const ox = src.x + m.forward*c - m.left*s, oy = src.y + m.forward*s + m.left*c;
            let best = 1e9;
            for (const b of world) { const t = rayBox(ox, oy, dx, dy, b); if (t >= 0) best = Math.min(best, t); }
            if (noise && best < 1e9) best += noise()*opts.rangeNoise;
            ranges[i] = {mm: best <= 1200 ? lround(Math.max(best, 1)) : 0xFFFF, at: now, sampled: true};
        }
    }
    function bodyCheck(now) {
        const src = truth, c = Math.cos(src.h), s = Math.sin(src.h);
        for (const b of world) {
            const g = footprintGap(src.x, src.y, c, s, b.x, b.y, BoxHalf, BoxHalf);
            if (g < minGap) { minGap = g; minGapAt = {t: now-runAt, box: b}; }
            if (g <= 0 && !collision) collision = {t: now-runAt, box: b};
        }
        const f = MAP.forbidden;
        const gf = footprintGap(src.x, src.y, c, s, 0.5*(f.xmin+f.xmax), 0.5*(f.ymin+f.ymax), 0.5*(f.xmax-f.xmin), 0.5*(f.ymax-f.ymin));
        if (gf <= 0 && !collision) collision = {t: now-runAt, box: {kind: 'FORBIDDEN'}};
    }

    // startRun(). The sensors were already sampling while the car waited for SW1.
    // t0 = 2330 ms reproduces the C++ rig's start time (sensor stagger phase).
    const t0 = opts.t0 === undefined ? 2330 : opts.t0;
    let now = t0;
    // While waiting for SW1 the beams are averaged (localize.h StartBeams), then fitted.
    const sum = [0, 0, 0, 0], cnt = [0, 0, 0, 0];
    for (let t = t0-1500; t <= t0; t += 10) {
        injectRanges(t);
        for (let i = 0; i < 4; ++i) if (ranges[i].at === t && ranges[i].mm !== 0xFFFF && ranges[i].mm <= tune.detectMm) {
            sum[i] += ranges[i].mm; if (++cnt[i] >= 32) { sum[i] *= 0.5; cnt[i] = 16; }
        }
    }
    const place = tune.startFix > 0.5 ? fitStart(sum.map((v, i) => cnt[i] >= 6 ? v/cnt[i] : NaN), StartPose, tune.detectMm)
                                      : {pose: StartPose, status: 'OFF', used: 0, rms: 0};
    pose = Object.assign({}, place.pose);
    ev(t0, 'START_POSE', {status: place.status, dx: pose.x-StartPose.x, dy: pose.y-StartPose.y, dhDeg: wrap(pose.h-StartPose.h)/Rad, used: place.used, rms: place.rms});
    for (let i = 0; i < 4; ++i) rangeSeenAt[i] = ranges[i].at;
    runAt = legPhaseAt = lastProgressAt = compareAt = lastLogAt = reviewAt = now;
    rememberPose(now);
    updateErrors(now);
    setLegPhase(Math.abs(angleError) > AlignStartDeg*Rad ? 'ALIGN' : 'TRACK', now);
    const maxMs = opts.maxMs || 120000;
    const PH = {ALIGN: 0, TRACK: 1, AVOID: 2, SETTLE: 3};
    while (now - runAt < maxMs) {
        const dt = 0.010;
        now += 10;
        // Rig: wheels follow the commanded CPS; true motion uses the true calibration.
        for (let i = 0; i < 2; ++i) {
            let demand = request.v + (i ? 0.5 : -0.5)*Track*request.w;
            if (!driven[i]) demand = 0;
            actualCps[i] += (demand*CountsPerMm[i] - actualCps[i])*0.25;
            residual[i] += actualCps[i]*dt;
            trueV[i] = actualCps[i]/trueCpm[i];
        }
        const ticks = [0, 1].map(i => { const t = lround(residual[i]); residual[i] -= t; return t; });
        const tw = (trueV[1]-trueV[0])/trueTrack, tv = 0.5*(trueV[0]+trueV[1]);
        truth.x += tv*dt*Math.cos(truth.h+tw*dt*0.5); truth.y += tv*dt*Math.sin(truth.h+tw*dt*0.5); truth.h = wrap(truth.h+tw*dt);
        yawRate = tw*(1 + (opts.trueGyroErr || 0)/100)*tune.gyroScale;  // gyro scale error, then the firmware's correction
        for (const e of pending) if (!e.done && station >= e.atStation) {
            e.done = true;
            world.push({x: truth.x + e.ahead*Math.cos(truth.h), y: truth.y + e.ahead*Math.sin(truth.h), kind: 'U', id: 0, late: true});
        }
        injectRanges(now);
        const wasRunning = state === 'RUN_ROUTE';
        control(now, ticks, dt);
        if (fromEstimate) truth = Object.assign({}, pose);  // ideal rig: the estimate IS the truth
        if (wasRunning || Math.abs(tv) > 0.5) bodyCheck(now);
        const ph = legPhase === 'TRACK' && avoiding(station) ? 'AVOID' : legPhase;
        trace.push([now-runAt, truth.x, truth.y, truth.h, pose.x, pose.y, pose.h, request.v, station, offsetNow, cross,
                    angleError, wheels[0].targetCps, wheels[1].targetCps, wheels[0].cps, wheels[1].cps, PH[ph],
                    ...ranges.map(r => r.mm === 0xFFFF ? -1 : r.mm)]);
        if (state !== 'RUN_ROUTE' && Math.abs(tv) < 0.5) break;  // finished or faulted, and stopped
    }
    if (state === 'RUN_ROUTE') fault('SIM_TIMEOUT', now);
    const endT = events.filter(e => e.type === 'END').map(e => e.t)[0];
    return {
        path: path.seg.map(g => Object.assign({}, g)), nodeS: path.nodeS.slice(), pathLength: path.length,
        world, trace, events,
        result: {
            reason: stopReason, ok: stopReason === 'GOAL_REACHED', timeMs: endT === undefined ? now-runAt : endT,
            distance: travelled, goalErrorEst: hypot(pose.x-MAP.goal.x, pose.y-MAP.goal.y),
            goalErrorTrue: hypot(truth.x-MAP.goal.x, truth.y-MAP.goal.y), maxCross, minBodyGap: minGap,
            minBodyGapAt: minGapAt, collision, startPose: place, rangeFixes: fixes.n, rangeFixMm: fixes.sum, finalPose: Object.assign({}, pose), finalTruth: Object.assign({}, truth),
        },
    };
}

const api = {MAP, TUNE, defaultTune, simulate, Path, footprintGap, shiftedCurv, MOUNTS, Rad, BodyHalfLength, BodyHalfWidth, BoxHalf};
if (typeof module !== 'undefined' && module.exports) module.exports = api;
root.SaBanSim = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
