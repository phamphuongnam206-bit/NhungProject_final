// Parser for the firmware's UART/Bluetooth output (src/test_static_route.cpp).
// Used for the live connection (line by line) and for saved logs. Units: mm, deg in
// the text, converted to rad here. Checked by tools/web/check_sim.js on C++ sim logs.
(function (root) {
'use strict';
const Rad = Math.PI/180;
const num = v => { const x = parseFloat(v); return Number.isFinite(x) ? x : NaN; };

function newRun(n) {
    return {number: n, params: {}, path: [], samples: [], events: [], obstacles: [], avoids: [],
            fails: [], meta: {}, end: null, simulated: false, columns: null};
}
// Feed lines one at a time; `onUpdate(kind, data)` is called for things a UI shows live.
class LineParser {
    constructor(onUpdate) {
        this.onUpdate = onUpdate || (() => {});
        this.runs = []; this.run = null; this.params = {}; this.state = null; this.columns = null;
        this.nextSimulated = false; this.lastT = 0;
    }
    current() {
        if (!this.run) { this.run = newRun(0); this.runs.push(this.run); }
        return this.run;
    }
    push(raw) {
        const line = String(raw).replace(/\r$/, '').trim();
        if (!line) return;
        const f = line.split(',');
        const tag = f[0];
        if (tag === '#SOURCE') { this.nextSimulated = (f[1] || '') === 'HOST_SIMULATION'; return; }
        if (tag === '#ROUTE_STARTED') {
            this.run = newRun(parseInt(f[1], 10) || this.runs.length+1);
            this.run.simulated = this.nextSimulated; this.nextSimulated = false;
            this.runs.push(this.run); this.lastT = 0;
            this.onUpdate('run', this.run);
            return;
        }
        if (tag === 'TIME_MS') { this.columns = f.map(s => s.trim()); return; }
        if (/^\d+$/.test(tag)) { this.sample(f); return; }
        if (!tag.startsWith('#')) { this.onUpdate('text', line); return; }
        const r = this.current();
        switch (tag) {
        case '#PARAM': {
            const p = {name: f[1], value: num(f[2]), lo: num(f[3]), hi: num(f[4])};
            if (p.name && Number.isFinite(p.value)) { this.params[p.name] = p; r.params[p.name] = p.value; this.onUpdate('param', p); }
            return;
        }
        case '#PATH_SEG': {
            const [i, x0, y0, h0, len, radius] = f.slice(1).map(num);
            if ([i, x0, y0, h0, len, radius].every(Number.isFinite)) {
                r.path[i] = {x0, y0, h0: h0*Rad, len, k: radius ? 1/radius : 0};
                this.onUpdate('path', r.path);
            }
            return;
        }
        case '#OBSTACLE': {
            const o = {t: this.lastT, id: parseInt(f[1], 10), kind: f[2], x: num(f[3]), y: num(f[4]), s: num(f[5]), lat: num(f[6])};
            if (Number.isFinite(o.x) && Number.isFinite(o.y)) { r.obstacles.push(o); r.events.push(Object.assign({type: 'OBSTACLE'}, o)); this.onUpdate('obstacle', o); }
            return;
        }
        case '#AVOID': {
            const [offset, s0, s1, s2, s3] = f.slice(3, 8).map(num);
            if ([offset, s0, s1, s2, s3].every(Number.isFinite)) {
                const a = {t: this.lastT, id: parseInt(f[1], 10), side: f[2], offset, s0, s1, s2, s3};
                const prev = r.avoids.findIndex(b => b.id === a.id && Math.abs(b.s0-a.s0) < 1 && b.side === a.side);
                a.final = prev >= 0;  // the firmware prints a plan when made and again when committed
                if (prev >= 0) r.avoids[prev] = a; else r.avoids.push(a);
                r.events.push(Object.assign({type: 'AVOID'}, a));
                this.onUpdate('avoid', a);
            }
            return;
        }
        case '#AVOID_FAIL': {
            const e = {t: this.lastT, type: 'AVOID_FAIL', id: parseInt(f[1], 10), reason: f[2]};
            r.fails.push(e); r.events.push(e); this.onUpdate('event', e);
            return;
        }
        case '#WAYPOINT_START': case '#WAYPOINT_END': {
            const e = {type: tag === '#WAYPOINT_END' ? 'WAYPOINT' : 'WAYPOINT_START', node: parseInt(f[1], 10),
                       t: num(f[2]), x: num(f[5]), y: num(f[6]), h: num(f[7])*Rad, travelled: num(f[8])};
            if (Number.isFinite(e.t)) { r.events.push(e); this.onUpdate('event', e); }
            return;
        }
        case '#ROUTE_FINISHED': case '#ROUTE_FAULT': case '#ROUTE_STOPPED':
            r.end = tag.slice(1); r.events.push({t: this.lastT, type: 'END', event: r.end});
            this.onUpdate('end', r);
            return;
        case '#CAL': case '#CAL_DONE': {
            const c = {done: tag === '#CAL_DONE', ms: num(f[1]), left: num(f[2]), right: num(f[3]), yawDeg: num(f[4])};
            if ([c.ms, c.left, c.right, c.yawDeg].every(Number.isFinite)) this.onUpdate('cal', c);
            return;
        }
        case '#PLACE': case '#START_POSE': {  // localize.h: status,dx,dy,dhDeg,beams,rms
            const p = {start: tag === '#START_POSE', status: f[1], dx: num(f[2]), dy: num(f[3]), dhDeg: num(f[4]), used: num(f[5]), rms: num(f[6])};
            if (p.start) r.meta.START_POSE = p;
            this.onUpdate('place', p);
            return;
        }
        case '#POSE_FIX': this.onUpdate('posefix', {n: num(f[1]), dx: num(f[2]), dy: num(f[3])}); return;
        case '#STATE': this.state = {state: f[1], reason: f[2]}; this.onUpdate('state', this.state); return;
        case '#PONG': this.state = Object.assign({}, this.state, {state: f[1]}); this.onUpdate('state', this.state); return;
        case '#WAIT_START': this.state = {state: 'WAIT_START'}; this.onUpdate('state', this.state); return;
        case '#CALIBRATING_GYRO': this.state = {state: 'INIT'}; this.onUpdate('state', this.state); return;
        case '#HELLO': this.onUpdate('hello', f.slice(1)); return;
        case '#ACK': case '#ERR': this.onUpdate(tag.slice(1).toLowerCase(), f.slice(1).join(',')); return;
        default:
            if (f.length === 2) {
                r.meta[tag.slice(1)] = f[1];
                if (tag === '#STOP_REASON') {
                    r.meta.STOP_REASON = f[1];
                    if (!r.end) { this.state = {state: 'FAULT', reason: f[1]}; this.onUpdate('state', this.state); }
                    this.onUpdate('end', r);
                }
            }
        }
    }
    sample(f) {
        const cols = this.columns;
        if (!cols || f.length !== cols.length) return;  // partial UART line
        const v = {};
        cols.forEach((c, i) => { v[c] = c === 'PHASE' ? f[i] : num(f[i]); });
        const s = {t: v.TIME_MS, x: v.X_MM, y: v.Y_MM, h: v.YAW_DEG*Rad, node: v.WAYPOINT,
                   v: 0.5*((v.LEFT_SPEED || 0)+(v.RIGHT_SPEED || 0)), spdL: v.LEFT_SPEED, spdR: v.RIGHT_SPEED,
                   angleErr: (v.HEADING_ERROR || 0)*Rad, cross: v.CROSS_TRACK_ERROR,
                   tl: v.TARGET_CPS_L, tr: v.TARGET_CPS_R, pwmL: v.PWM_L, pwmR: v.PWM_R, phase: v.PHASE || '',
                   al: v.ACTUAL_CPS_L, ar: v.ACTUAL_CPS_R, s: v.S_MM, off: v.OFFSET_MM,
                   r: [v.RANGE0, v.RANGE1, v.RANGE2, v.RANGE3].map(x => x === undefined ? NaN : x)};
        if (![s.t, s.x, s.y, s.h].every(Number.isFinite)) return;
        const r = this.current();
        if (r.samples.length && s.t < r.samples[r.samples.length-1].t) {  // bare CSV restarted
            this.run = newRun(this.runs.length+1); this.runs.push(this.run);
        }
        this.current().samples.push(s);
        this.lastT = s.t;
        this.onUpdate('sample', s);
    }
}
function parseText(text) {
    const p = new LineParser();
    for (const line of String(text).split(/\n/)) p.push(line);
    return {runs: p.runs.filter(r => r.samples.length || r.path.length), params: p.params};
}
const api = {LineParser, parseText};
if (typeof module !== 'undefined' && module.exports) module.exports = api;
root.SaBanUart = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);
