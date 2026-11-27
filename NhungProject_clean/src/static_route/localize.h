#pragma once
// Pose fixes from the VL53L0X beams against the FIXED MAP boxes (never random/unknown ones).
// Included inside namespace StaticRoute after avoid.h (uses Pose, Mounts, RangeSensors, tune).
//  1. Start: while the car waits for SW1 each beam is averaged. On START a small weighted
//     least-squares fit (prior = nominal START pose) corrects a hand-placement error.
//     #PLACE (1 Hz while waiting) and #START_POSE report it. Far off / no data -> nominal pose.
//  2. Run: a beam that hits a fixed box face nudges the position along that face's normal,
//     weighted by the odometry uncertainty (positionError). #POSE_FIX reports the total.

struct BeamHit { float t, nx, ny; int8_t box; };  // range, outward face normal, FIXED index
inline void beamFrom(const Pose &p, unsigned i, float &ox, float &oy, float &dx, float &dy) {
    const Mount &m = Mounts[i];
    const float c = cosf(p.h), s = sinf(p.h);
    ox = p.x + m.forward*c - m.left*s; oy = p.y + m.forward*s + m.left*c;
    dx = cosf(p.h+m.yaw); dy = sinf(p.h+m.yaw);
}
inline BeamHit castFixed(float ox, float oy, float dx, float dy) {
    BeamHit best{1.0e9f, 0, 0, -1};
    for (int b = 0; b < StaticMap::FIXED_COUNT; ++b) {
        const StaticMap::Obstacle &o = StaticMap::FIXED_OBSTACLES[b];
        const float org[2] = {ox, oy}, d[2] = {dx, dy};
        const float lo[2] = {o.x-0.5f*o.width, o.y-0.5f*o.length}, hi[2] = {o.x+0.5f*o.width, o.y+0.5f*o.length};
        float t0 = 0, t1 = 1.0e9f; int axis = -1; bool ok = true;
        for (int a = 0; a < 2 && ok; ++a) {
            if (fabsf(d[a]) < 1.0e-6f) { ok = org[a] >= lo[a] && org[a] <= hi[a]; continue; }
            float ta = (lo[a]-org[a])/d[a], tb = (hi[a]-org[a])/d[a];
            if (ta > tb) { const float k = ta; ta = tb; tb = k; }
            if (ta > t0) { t0 = ta; axis = a; }
            t1 = fminf(t1, tb);
            ok = t0 <= t1;
        }
        if (!ok || axis < 0 || t0 >= best.t) continue;  // axis < 0: origin inside the box
        best = {t0, axis == 0 ? (dx > 0 ? -1.0f : 1.0f) : 0.0f, axis == 1 ? (dy > 0 ? -1.0f : 1.0f) : 0.0f, int8_t(b)};
    }
    return best;
}
// Range to a fixed box and its gradient wrt (x, y, h). False: no usable face hit.
inline bool expectBeam(const Pose &p, unsigned i, float &t, float J[3], int8_t *box = nullptr) {
    float ox, oy, dx, dy;
    beamFrom(p, i, ox, oy, dx, dy);
    const BeamHit h = castFixed(ox, oy, dx, dy);
    const float c = -(dx*h.nx + dy*h.ny);  // cos of incidence
    if (h.box < 0 || h.t > tune.detectMm || c < 0.35f) return false;
    const Mount &m = Mounts[i];
    const float cs = cosf(p.h), sn = sinf(p.h);
    const float dox = -m.forward*sn - m.left*cs, doy = m.forward*cs - m.left*sn;  // d(origin)/dh
    t = h.t;
    J[0] = h.nx/c; J[1] = h.ny/c;
    J[2] = (dox*h.nx + doy*h.ny + h.t*(-dy*h.nx + dx*h.ny))/c;
    if (box) *box = h.box;
    return true;
}

// ---- 1. Start placement ----
struct StartBeams {
    float sum[RangeSensors::Count]{};
    uint8_t n[RangeSensors::Count]{};
    uint32_t seen[RangeSensors::Count]{};
    void clear() { for (unsigned i = 0; i < RangeSensors::Count; ++i) { sum[i] = 0; n[i] = 0; } }
    void sample(uint32_t now) {
        for (unsigned i = 0; i < RangeSensors::Count; ++i) {
            if (!RangeSensors::fresh(i, now)) continue;
            const RangeSensors::Reading &r = RangeSensors::get(i);
            if (r.at == seen[i]) continue;
            seen[i] = r.at;
            if (r.mm == RangeSensors::NoTarget || r.mm > tune.detectMm) continue;
            sum[i] += r.mm;
            if (++n[i] >= 32) { sum[i] *= 0.5f; n[i] = 16; }  // keep the average recent
        }
    }
} startBeams;
struct StartFix { Pose pose; float rms; uint8_t used; const char *status; };
constexpr float StartSigmaRange = 6, StartSigmaXY = 40, StartSigmaH = 8*Rad;
// Signed distance from point q to the nearest fixed box (inside < 0) and its unit normal.
// Only boxes near START are tried (the fit runs on a 72 MHz core without an FPU).
uint32_t startBoxes() {
    uint32_t mask = 0;
    for (int b = 0; b < StaticMap::FIXED_COUNT; ++b)
        if (hypotf(StaticMap::FIXED_OBSTACLES[b].x-StartPose.x, StaticMap::FIXED_OBSTACLES[b].y-StartPose.y) < tune.detectMm+200)
            mask |= 1UL << b;
    return mask;
}
float nearestFixed(float qx, float qy, float &nx, float &ny, uint32_t mask) {
    float best = 1.0e9f;
    for (int b = 0; b < StaticMap::FIXED_COUNT; ++b) {
        if (!(mask & (1UL << b))) continue;
        const StaticMap::Obstacle &o = StaticMap::FIXED_OBSTACLES[b];
        const float hx = 0.5f*o.width, hy = 0.5f*o.length, ex = qx-o.x, ey = qy-o.y;
        const float ox = fabsf(ex)-hx, oy = fabsf(ey)-hy;
        float d, mx, my;
        if (ox > 0 || oy > 0) {
            const float cx = fmaxf(ox, 0), cy = fmaxf(oy, 0);
            d = hypotf(cx, cy); mx = cx/d*(ex < 0 ? -1 : 1); my = cy/d*(ey < 0 ? -1 : 1);
        } else if (ox > oy) { d = ox; mx = ex < 0 ? -1 : 1; my = 0; }
        else { d = oy; mx = 0; my = ey < 0 ? -1 : 1; }
        if (fabsf(d) < fabsf(best)) { best = d; nx = mx; ny = my; }
    }
    return best;
}
// Gauss-Newton (point-to-box, like ICP) from one seed. Each averaged beam end point should
// lie on a fixed box surface. cost: weighted squares + prior + penalty per unmatched beam.
Pose fitFrom(Pose p, uint32_t mask, float &cost, uint8_t &used, float &rms) {
    constexpr float wr = 1/(StartSigmaRange*StartSigmaRange);
    constexpr float wp = 1/(StartSigmaXY*StartSigmaXY), wh = 1/(StartSigmaH*StartSigmaH);
    for (int it = 0; it < 6; ++it) {
        float A[3][3] = {{wp, 0, 0}, {0, wp, 0}, {0, 0, wh}};
        float g[3] = {-(p.x-StartPose.x)*wp, -(p.y-StartPose.y)*wp, -wrap(p.h-StartPose.h)*wh};
        float sq = 0, bad = 0; used = 0;
        const float cs = cosf(p.h), sn = sinf(p.h);
        for (unsigned i = 0; i < RangeSensors::Count; ++i) {
            if (startBeams.n[i] < 6) continue;
            const float m = startBeams.sum[i]/startBeams.n[i];
            const Mount &mt = Mounts[i];
            float ox, oy, dx, dy, nx = 0, ny = 0;
            beamFrom(p, i, ox, oy, dx, dy);
            const float d = nearestFixed(ox + m*dx, oy + m*dy, nx, ny, mask);
            if (fabsf(d) > 60) { bad += 1; continue; }
            const float qhx = -mt.forward*sn - mt.left*cs - m*dy, qhy = mt.forward*cs - mt.left*sn + m*dx;
            const float J[3] = {nx, ny, nx*qhx + ny*qhy};
            for (int a = 0; a < 3; ++a) { g[a] -= wr*J[a]*d; for (int b = 0; b < 3; ++b) A[a][b] += wr*J[a]*J[b]; }
            sq += d*d; ++used;
        }
        const float ex = p.x-StartPose.x, ey = p.y-StartPose.y, eh = wrap(p.h-StartPose.h);
        cost = wr*sq + wp*(ex*ex+ey*ey) + wh*eh*eh + 25*bad;
        rms = used ? sqrtf(sq/used) : 0;
        if (it == 5) break;
        const float det = A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1]) - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
                        + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
        float st[3];
        for (int k = 0; k < 3; ++k) {  // Cramer's rule
            float M[3][3];
            for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) M[a][b] = b == k ? g[a] : A[a][b];
            st[k] = (M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1]) - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
                   + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]))/det;
        }
        p.x += bound(st[0], -30, 30); p.y += bound(st[1], -30, 30); p.h = wrap(p.h + bound(st[2], -0.1f, 0.1f));
    }
    return p;
}
StartFix fitStart() {
    StartFix f{StartPose, 0, 0, "NO_DATA"};
    float best = 1.0e9f;
    Pose p = StartPose;
    const uint32_t mask = startBoxes();
    for (int s = 0; s < 3 && best > 2; ++s) {  // seeds: heading 0 / +6 / -6 deg (if still poor)
        float cost, rms; uint8_t used;
        const Pose q = fitFrom({StartPose.x, StartPose.y, wrap(StartPose.h + (s == 0 ? 0 : s == 1 ? 6 : -6)*Rad)}, mask, cost, used, rms);
        if (used >= 2 && cost < best) { best = cost; p = q; f.used = used; f.rms = rms; }
    }
    if (f.used < 2) return f;
    const bool far = fabsf(p.x-StartPose.x) > StaticMap::START_SEARCH_MM || fabsf(p.y-StartPose.y) > StaticMap::START_SEARCH_MM
                     || fabsf(wrap(p.h-StartPose.h)) > StaticMap::START_SEARCH_DEG*Rad || f.rms > 25;
    f.status = far ? "FAR" : "OK";
    if (!far) f.pose = p;
    return f;
}
void logStartFix(const char *tag, const StartFix &f) {
    char line[110];
    snprintf(line, sizeof(line), "#%s,%s,%s,%s,%s,%u,%s\r\n", tag, f.status,
             Decimal(f.pose.x-StartPose.x).text, Decimal(f.pose.y-StartPose.y).text,
             Decimal(wrap(f.pose.h-StartPose.h)/Rad).text, unsigned(f.used), Decimal(f.rms).text);
    tx.push(line);
}

// ---- 2. Corrections while running ----
struct RangeFix { uint16_t count; float dx, dy, total; uint32_t logAt; } rangeFix{};
// p: pose when the beam was sampled. Returns the correction applied (0 if rejected).
float fixFromBeam(const Pose &p, unsigned i, float mm, uint8_t seenId) {
    float t, J[3]; int8_t box;
    if (!expectBeam(p, i, t, J, &box) || StaticMap::FIXED_OBSTACLES[box].id != seenId) return 0;
    const float e = mm - t;
    if (fabsf(e) > 30) return 0;                        // wrong box / grazing hit / noise
    // Scalar Kalman update, diagonal covariance from the odometry error bounds.
    constexpr float R = 10.0f;                          // one raw VL53 sample, mm
    const float P[3] = {positionError*positionError, positionError*positionError, headingError*headingError};
    const float S = P[0]*J[0]*J[0] + P[1]*J[1]*J[1] + P[2]*J[2]*J[2] + R*R;
    const float sx = bound(P[0]*J[0]*e/S, -4.0f, 4.0f);
    const float sy = bound(P[1]*J[1]*e/S, -4.0f, 4.0f);
    const float sh = bound(P[2]*J[2]*e/S, -0.5f*Rad, 0.5f*Rad);
    pose.x += sx; pose.y += sy; pose.h = wrap(pose.h + sh);
    for (uint8_t k = 0; k < historySize; ++k) {
        history[k].value.x += sx; history[k].value.y += sy; history[k].value.h = wrap(history[k].value.h + sh);
    }
    positionError = fmaxf(3.0f, positionError*sqrtf(1 - fminf(0.9f, P[0]*(J[0]*J[0]+J[1]*J[1])/S)));
    headingError = fmaxf(0.5f*Rad, headingError*sqrtf(1 - fminf(0.9f, P[2]*J[2]*J[2]/S)));
    const float step = hypotf(sx, sy);
    rangeFix.count++; rangeFix.dx += sx; rangeFix.dy += sy; rangeFix.total += fabsf(step);
    return step;
}
void logRangeFix(uint32_t now) {
    if (now-rangeFix.logAt < 1000 || !rangeFix.count) return;
    rangeFix.logAt = now;
    char line[80];
    snprintf(line, sizeof(line), "#POSE_FIX,%u,%s,%s\r\n", unsigned(rangeFix.count),
             Decimal(rangeFix.dx).text, Decimal(rangeFix.dy).text);
    tx.push(line);
    rangeFix.count = 0; rangeFix.dx = rangeFix.dy = 0;
}
