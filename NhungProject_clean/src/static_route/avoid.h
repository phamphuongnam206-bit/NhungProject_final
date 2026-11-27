#pragma once
// Obstacle memory + smooth lateral-offset avoidance on top of the reference path.
// The car keeps moving: the reference is shifted sideways with a raised-cosine
// ramp, held while passing the box, then ramped back. No stop-and-pivot.
// Collision checks use the real 130 x 100 mm footprint at the shifted heading.
// Included inside namespace StaticRoute after path.h.

constexpr float BoxHalf = StaticMap::BOX_SIZE*0.5f;
constexpr float BodyHalfWidth = StaticMap::ROBOT_WIDTH*0.5f;
constexpr float BodyHalfLength = StaticMap::ROBOT_LENGTH*0.5f;
// Derived from tune.safeMarginMm; refreshed by updateClearances() after a $SET.
float FootMarginMm = DefaultTuning.safeMarginMm;              // car body to any box, required
float ClearMm = BodyHalfWidth + DefaultTuning.safeMarginMm;  // the same, as centre to box edge side-on
float PassMm = BoxHalf*1.2f + ClearMm;                        // centre-to-centre lateral to pass a box
float BlockMm = PassMm - 10.0f;                               // closer than this = blocking
void updateClearances() {
    FootMarginMm = tune.safeMarginMm;
    ClearMm = BodyHalfWidth + tune.safeMarginMm;
    PassMm = BoxHalf*1.2f + ClearMm;
    BlockMm = PassMm - 10.0f;
}
// Full offset is held from 100 mm before to 100 mm after the box centre. The raised
// cosine is flat at its ends (97% of the offset 30 mm before the hold); the planner
// checks the footprint along the whole detour anyway.
constexpr float HoldBeforeMm = BoxHalf + BodyHalfLength - 15.0f, HoldAfterMm = HoldBeforeMm;
constexpr float RampMm = 300.0f, MinRampMm = 100.0f, MaxOffsetMm = 260.0f;
constexpr float RampCurv = 0.01f;            // peak curvature of a ramp (radius >= 100 mm)
constexpr float PlanCurvMax = 0.75f/60.0f;   // detour curvature limit: 75% of the controller's 1/60
constexpr float PlanStepMm = 25.0f;          // footprint check spacing along a detour
// Snap a hit to a MAP box within 45 mm: covers odometry drift (~1.5% over 3 m) so the
// fixed boxes are never mistaken for new ones. Trade-off: an unknown box standing
// < 45 mm from a MAP box is labelled as that box (not possible under the rules).
constexpr float SnapMm = 45.0f;
constexpr float MergeMm = 90.0f, HitMergeMm = 30.0f;  // boxes never overlap: centres >= 100 apart
// Last gap (station) to commit: shortest ramp + hold must still fit, plus braking margin.
constexpr float DecideMm = HoldBeforeMm + MinRampMm + 120.0f;
// Shortest raised-cosine ramp for an offset d: peak curvature d*pi^2/(2L^2) <= RampCurv.
float rampFor(float d) { return bound(sqrtf(fabsf(d)*Pi*Pi/(2*RampCurv)), MinRampMm, RampMm); }
constexpr uint8_t ConfirmHits = 3;
constexpr uint32_t ForgetMs = 2000;  // a confirmed box unseen this long (and unplanned) may be recycled

struct Shift { float d, slope, curv; };  // offset (+ left), d', d'' along the base station
struct Avoidance {
    float offset = 0, s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    uint8_t id = 0, obstacle = 0;  // MAP id (0 = unknown) and ObstacleMap index
    bool active = false;
    Shift at(float s) const {
        if (!active || s <= s0 || s >= s3) return {0, 0, 0};
        if (s >= s1 && s <= s2) return {offset, 0, 0};
        const bool in = s < s1;
        const float len = in ? s1-s0 : s3-s2, a = Pi/len, sign = in ? 1.0f : -1.0f;
        const float t = (in ? s-s0 : s-s2)/len, c = cosf(Pi*t), sn = sinf(Pi*t);
        return {offset*0.5f*(1-sign*c), sign*offset*0.5f*a*sn, sign*offset*0.5f*a*a*c};
    }
};
// Exact geometry of the reference shifted by d(s) from a base segment of curvature k:
// r' = A t + d' n with A = 1 - k d, so heading = h + atan2(d', A) and
// curvature = (k A^2 + A d'' + 2 k d'^2) / (A^2 + d'^2)^(3/2), per shifted arc length.
float shiftedCurv(float k, const Shift &sh) {
    if (!sh.d && !sh.slope && !sh.curv) return k;
    const float A = 1 - k*sh.d, B = sh.slope, q = A*A + B*B;
    return (k*A*A + A*sh.curv + 2*k*B*B)/(q*sqrtf(q));
}
float shiftedHeading(float h, float k, const Shift &sh) {
    return (!sh.d && !sh.slope) ? h : wrap(h + atan2f(sh.slope, 1 - k*sh.d));
}

struct Obstacle {
    float x, y;
    uint32_t lastAt;
    uint8_t id, hits;
    char kind;  // 'F' fixed map box (never stored), 'R' random candidate, 'U' unknown, 'X' retired
    bool confirmed;
    bool planned;  // a detour is planned for it (re-planned while the car has not reached it)
    bool frozen;   // the car entered that detour, or is close: estimate and plan are final
    uint8_t samples;
};
constexpr unsigned MaxObstacles = 16;

float boxDistance(float x, float y, float cx, float cy, float half) {
    return hypotf(fmaxf(0, fabsf(x-cx)-half), fmaxf(0, fabsf(y-cy)-half));
}
// Gap between the car rectangle (centre x,y; heading cos c, sin s) and an axis-aligned
// rectangle (centre cx,cy; half sizes hx,hy). 0 when they overlap. Far pairs return 100
// early: every caller only compares against margins well below that.
float footprintGap(float x, float y, float c, float s, float cx, float cy, float hx, float hy) {
    constexpr float L = BodyHalfLength, W = BodyHalfWidth;
    const float dx = cx-x, dy = cy-y, ac = fabsf(c), as = fabsf(s);
    const float reachX = L*ac + W*as, reachY = L*as + W*ac;  // the car's own AABB half sizes
    if (fabsf(dx) > hx + reachX + 100.0f || fabsf(dy) > hy + reachY + 100.0f) return 100.0f;
    const float u = dx*c + dy*s, v = dy*c - dx*s;  // rectangle centre in the car frame
    if (fabsf(dx) <= hx + reachX && fabsf(dy) <= hy + reachY &&
        fabsf(u) <= L + hx*ac + hy*as && fabsf(v) <= W + hx*as + hy*ac) return 0;  // SAT: overlap
    // Separated convex shapes: the gap is reached at a corner of one of them.
    float best = 1.0e18f;
    for (int i = 0; i < 4; ++i) {
        const float a = (i & 1) ? L : -L, b = (i & 2) ? W : -W;
        const float ex = fmaxf(0, fabsf(a*c - b*s - dx) - hx), ey = fmaxf(0, fabsf(a*s + b*c - dy) - hy);
        best = fminf(best, ex*ex + ey*ey);
        const float px = ((i & 1) ? hx : -hx) + dx, py = ((i & 2) ? hy : -hy) + dy;
        const float fu = fmaxf(0, fabsf(px*c + py*s) - L), fv = fmaxf(0, fabsf(py*c - px*s) - W);
        best = fminf(best, fu*fu + fv*fv);
    }
    return sqrtf(best);
}

struct ObstacleMap {
    Obstacle items[MaxObstacles]{};
    unsigned count = 0;
    bool overflow = false;  // a new box could not be stored: the caller must stop the car

    void clear() { count = 0; overflow = false; }
    // Boxes cannot overlap: two unknown estimates closer than one box in both axes are
    // the same box seen from different faces. Keep the one with a plan, average if
    // allowed, and retire the other in place ('X', so indices used by plans stay valid).
    Obstacle *absorbTwins(Obstacle *slot) {
        for (unsigned j = 0; j < count; ++j) {
            Obstacle &o = items[j];
            if (&o == slot || o.kind != 'U' || fabsf(o.x-slot->x) >= StaticMap::BOX_SIZE ||
                fabsf(o.y-slot->y) >= StaticMap::BOX_SIZE) continue;
            if ((o.planned || o.frozen) && (slot->planned || slot->frozen)) continue;  // both committed
            Obstacle *keep = (o.planned || o.frozen) ? &o : slot, *drop = keep == slot ? &o : slot;
            if (!keep->frozen) {
                const float w = float(drop->samples)/fmaxf(1.0f, float(keep->samples+drop->samples));
                keep->x += w*(drop->x-keep->x); keep->y += w*(drop->y-keep->y);
            }
            keep->samples = uint8_t(fminf(255.0f, float(keep->samples+drop->samples)));
            keep->hits = keep->hits > drop->hits ? keep->hits : drop->hits;
            keep->confirmed |= drop->confirmed;
            keep->lastAt = slot->lastAt;
            drop->kind = 'X'; drop->confirmed = drop->planned = false;
            slot = keep;
        }
        return slot;
    }
    // Snap a range hit to a known MAP box. Otherwise assume an axis-aligned 100 mm box
    // whose face was hit: its centre lies half a box behind that face. The position
    // along the face is unknown; the running mean of later hits refines it.
    Obstacle classify(float hx, float hy, float dx, float dy, uint32_t now) const {
        for (const auto &o : StaticMap::FIXED_OBSTACLES)
            if (boxDistance(hx, hy, o.x, o.y, BoxHalf) < SnapMm)
                return {float(o.x), float(o.y), now, o.id, 0, 'F', false, false, false, 0};
        for (const auto &o : StaticMap::POSSIBLE_OBSTACLES)
            if (boxDistance(hx, hy, o.x, o.y, BoxHalf) < SnapMm)
                return {float(o.x), float(o.y), now, o.id, 0, 'R', false, false, false, 0};
        if (fabsf(dx) > fabsf(dy)) return {hx + (dx > 0 ? BoxHalf : -BoxHalf), hy, now, 0, 0, 'U', false, false, false, 0};
        return {hx, hy + (dy > 0 ? BoxHalf : -BoxHalf), now, 0, 0, 'U', false, false, false, 0};
    }
    // A slot for a new box: unused, else the oldest unconfirmed (noise), else a confirmed
    // box nobody plans around that has not been seen for ForgetMs (it is behind the car).
    Obstacle *freeSlot(uint32_t now) {
        if (count < MaxObstacles) return &items[count++];
        Obstacle *slot = nullptr;
        for (unsigned i = 0; i < count; ++i)
            if ((items[i].kind == 'X' || !items[i].confirmed) &&
                (!slot || int32_t(items[i].lastAt-slot->lastAt) < 0)) slot = &items[i];
        if (slot) return slot;
        for (unsigned i = 0; i < count; ++i) {
            const Obstacle &o = items[i];
            if (!o.planned && !o.frozen && now-o.lastAt > ForgetMs &&
                (!slot || int32_t(o.lastAt-slot->lastAt) < 0)) slot = &items[i];
        }
        return slot;
    }
    // (hx,hy) is the raw hit. A hit on or next to a box already known is the same box.
    // Returns the entry that just became confirmed, or nullptr. Fixed boxes are not stored.
    Obstacle *note(const Obstacle &seen, float hx, float hy, uint32_t now) {
        if (seen.kind == 'F') return nullptr;
        Obstacle *slot = nullptr;
        for (unsigned i = 0; i < count; ++i) {
            Obstacle &o = items[i];
            const bool same = seen.id ? o.id == seen.id
                : o.kind == 'U' && (boxDistance(hx, hy, o.x, o.y, BoxHalf) < HitMergeMm ||
                                    hypotf(o.x-seen.x, o.y-seen.y) < MergeMm);
            if (same) { slot = &o; break; }
        }
        if (!slot) {
            slot = freeSlot(now);
            if (!slot) { overflow = true; return nullptr; }
            *slot = seen;
        } else if (slot->kind == 'U' && !slot->frozen) {
            // Running mean: early corner hits from the angled beams are refined by later ones.
            const float w = 1.0f/fminf(float(slot->samples)+1.0f, 8.0f);
            slot->x += w*(seen.x-slot->x); slot->y += w*(seen.y-slot->y);
        }
        if (slot->samples < 255) ++slot->samples;
        if (slot->kind == 'U') slot = absorbTwins(slot);
        // Hits must keep coming: a gap of 300 ms restarts the count (noise rejection).
        if (now-slot->lastAt > 300) slot->hits = 0;
        slot->lastAt = now;
        if (slot->hits < 255) ++slot->hits;
        if (slot->confirmed || slot->hits < ConfirmHits) return nullptr;
        slot->confirmed = true;
        return slot;
    }
    // Footprint gap of the car at (x,y,heading) to fixed boxes, confirmed boxes, the
    // forbidden zone and the map sides (the bottom edge is open: START/GOAL lie below it).
    // Exactly one of the 4 random candidates appears: until one is confirmed, a detour
    // must not cut through any candidate spot.
    float clearance(float x, float y, float h) const {
        const float c = cosf(h), s = sinf(h);
        float gap = 1.0e9f;
        bool randomFound = false;
        for (const auto &o : StaticMap::FIXED_OBSTACLES)
            gap = fminf(gap, footprintGap(x, y, c, s, o.x, o.y, BoxHalf, BoxHalf));
        for (unsigned i = 0; i < count; ++i) {
            const Obstacle &o = items[i];
            if (!o.confirmed) continue;
            randomFound |= o.kind == 'R';
            gap = fminf(gap, footprintGap(x, y, c, s, o.x, o.y, BoxHalf, BoxHalf));
        }
        if (!randomFound)
            for (const auto &o : StaticMap::POSSIBLE_OBSTACLES)
                gap = fminf(gap, footprintGap(x, y, c, s, o.x, o.y, BoxHalf, BoxHalf));
        const auto &f = StaticMap::FORBIDDEN;
        gap = fminf(gap, footprintGap(x, y, c, s, 0.5f*(f.xmin+f.xmax), 0.5f*(f.ymin+f.ymax),
                                      0.5f*(f.xmax-f.xmin), 0.5f*(f.ymax-f.ymin)));
        const float reachX = BodyHalfLength*fabsf(c) + BodyHalfWidth*fabsf(s);
        const float reachY = BodyHalfLength*fabsf(s) + BodyHalfWidth*fabsf(c);
        gap = fminf(gap, fminf(x - reachX, float(StaticMap::MAP_WIDTH) - x - reachX));
        return fminf(gap, float(StaticMap::MAP_HEIGHT) - y - reachY);
    }
};

// Footprint check of a detour on [from, to]: true when its shape is trackable (no offset
// deep toward an arc centre, curvature within PlanCurvMax) and the body keeps FootMarginMm.
bool checkDetour(const Path &path, const ObstacleMap &map, const Avoidance &a, float from, float to, float &worst) {
    worst = 1.0e9f;
    for (float s = from; s <= to; s += PlanStepMm) {
        const PathPoint p = path.at(s);
        const Shift sh = a.at(s);
        if (1 - p.k*sh.d < 0.5f || fabsf(shiftedCurv(p.k, sh)) > PlanCurvMax) return false;
        const float h = shiftedHeading(p.h, p.k, sh);
        worst = fminf(worst, map.clearance(p.x - sinf(p.h)*sh.d, p.y + cosf(p.h)*sh.d, h));
        if (worst < FootMarginMm) return false;
    }
    return true;
}

enum class PlanResult { Ok, NoRoom, TooClose };
// Plan an offset around a box centred at station sObs, lateral latObs. The ramp
// starts no earlier than `from` (robot station, or the end of an earlier avoidance).
PlanResult planAvoid(const Path &path, const ObstacleMap &map, uint8_t id, float from,
                     float sObs, float latObs, Avoidance &out) {
    const float s1 = sObs - HoldBeforeMm, s2 = sObs + HoldAfterMm;
    const float room = s1 - from, roomAfter = path.length - 40.0f - s2;
    float best = 1.0e9f;
    bool found = false, tooClose = false;
    for (int side = 0; side < 2; ++side) {
        Avoidance a;
        a.offset = side ? latObs - PassMm : latObs + PassMm;
        if (fabsf(a.offset) > MaxOffsetMm) continue;
        const float ramp = rampFor(a.offset);
        if (roomAfter < ramp) continue;
        a.active = true; a.id = id;
        // Geometry is judged even when the car is already too close, so a dead-end
        // corridor reports NoRoom and a feasible-but-late detour reports TooClose.
        a.s0 = s1 - fmaxf(ramp, fminf(RampMm, room)); a.s1 = s1; a.s2 = s2; a.s3 = s2 + fminf(RampMm, roomAfter);
        float worst = 0;
        if (!checkDetour(path, map, a, a.s0, a.s3, worst)) continue;
        if (room < ramp) { tooClose = true; continue; }
        const float score = fabsf(a.offset) - 0.2f*worst;
        if (score < best) { best = score; out = a; found = true; }
    }
    return found ? PlanResult::Ok : tooClose ? PlanResult::TooClose : PlanResult::NoRoom;
}
