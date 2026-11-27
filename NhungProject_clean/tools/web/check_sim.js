// Compare the JavaScript simulator (sim.js) with the C++ host simulation
// (test/static_route_host.cpp). Run: node tools/web/check_sim.js
// The C++ rig ray-casts from the estimated pose, so these checks do the same.
'use strict';
const S = require('./sim.js');
let failed = 0;
function check(name, cond, detail) {
    console.log(`${cond ? 'PASS' : 'FAIL'} ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failed++;
}
const near = (a, b, tol) => Math.abs(a-b) <= tol;
const run = (o) => S.simulate(Object.assign({raycastFrom: 'estimate'}, o));

// Expected values printed by the C++ simulation (VERIFICATION.md).
for (const [cps, ms, mm, goal] of [[400, 36370, 2858.72, 4.576], [700, 23220, 2876.86, 4.652]]) {
    const r = run({tune: {cruiseCps: cps}});
    const x = r.result;
    check(`route ${cps} CPS`, x.ok && near(x.timeMs, ms, ms*0.01) && near(x.distance, mm, 5) && near(x.goalErrorEst, goal, 1.5),
          `${x.reason} ${x.timeMs} ms (C++ ${ms}), ${x.distance.toFixed(1)} mm (C++ ${mm}), goal ${x.goalErrorEst.toFixed(2)} mm (C++ ${goal})`);
    check(`route ${cps} no detour`, !r.events.some(e => e.type === 'AVOID'));
    check(`route ${cps} body gap`, x.minBodyGap >= 7.5 && !x.collision, `${x.minBodyGap.toFixed(1)} mm`);
}
for (const id of [14, 15, 16, 17]) {
    const r = run({random: id});
    check(`random ${id}`, r.result.ok && !r.events.some(e => e.type === 'AVOID') &&
          r.events.some(e => e.type === 'OBSTACLE' && e.id === id), `${r.result.reason}, gap ${r.result.minBodyGap.toFixed(1)} mm`);
}
{
    const r = run({unknown: [{x: 1510, y: 307}]});
    const av = r.events.filter(e => e.type === 'AVOID');
    check('block (1510,307) detours and finishes', r.result.ok && av.length >= 2 && av[0].side === 'LEFT' && !r.result.collision,
          `${r.result.reason}, first ${av[0] && av[0].offset.toFixed(1)} (C++ 17.6), final ${av.length && av[av.length-1].offset.toFixed(1)}`);
}
{
    const r = run({unknown: [{x: 362, y: 652}]});
    const end = r.trace[r.trace.length-1];
    check('block-center stops safely', r.result.reason === 'NO_AVOID_PATH' && !r.result.collision,
          `${r.result.reason} at s=${end[8].toFixed(0)} (C++ ~518)`);
}
{
    const r = run({events: [{atStation: 1500, ahead: 230}]});
    check('late box: stops without contact', ['NO_AVOID_PATH', 'OBSTACLE_TOO_CLOSE', 'OBSTACLE_STOP'].includes(r.result.reason) && !r.result.collision,
          r.result.reason);
}
{   // Ray-cast from the true pose with a calibration error: the web tool's default mode.
    const r = S.simulate({trueCpmErr: [2, 2]});
    check('2% wheel calibration error still reaches GOAL (estimate)', r.result.reason === 'GOAL_REACHED',
          `true GOAL error ${r.result.goalErrorTrue.toFixed(1)} mm`);
}
{   // Hand placement 20 mm / -15 mm / 4 deg off (C++ "place": 7.1 mm estimate error, GOAL 4.8 mm)
    const r = S.simulate({place: {dx: 20, dy: -15, dhDeg: 4}});
    const sp = r.result.startPose;
    check('place: START_POSE fit + reaches GOAL', sp.status === 'OK' && r.result.ok && r.result.goalErrorTrue < 12 && !r.result.collision,
          `fit ${sp.pose.x.toFixed(1)},${sp.pose.y.toFixed(1)},${(sp.pose.h/S.Rad).toFixed(2)} (C++ 313.0,-80.7,92.69), true GOAL ${r.result.goalErrorTrue.toFixed(1)} mm`);
}
{   // 2% wheel + 2% gyro error: range fixes vs none (C++ "drift": 30.7 vs 54.9 mm)
    const a = S.simulate({trueCpmErr: [2, 2], trueGyroErr: 2});
    const b = S.simulate({trueCpmErr: [2, 2], trueGyroErr: 2, tune: {rangeFix: 0}});
    check('drift: range fixes reduce the true GOAL error', a.result.ok && a.result.goalErrorTrue < b.result.goalErrorTrue && !a.result.collision,
          `${a.result.goalErrorTrue.toFixed(1)} mm with, ${b.result.goalErrorTrue.toFixed(1)} mm without, ${a.result.rangeFixes} fixes`);
}
console.log(failed ? `${failed} check(s) failed` : 'all checks passed');
process.exit(failed ? 1 : 0);
