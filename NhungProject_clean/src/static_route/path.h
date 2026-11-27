#pragma once
// Smooth reference path: the 5-node MAP polyline with a circular fillet at N1..N3,
// so the car rounds each corner while moving instead of stop-turn-go.
// Included inside namespace StaticRoute after bound()/wrap().

// Corner radius at N1..N3 is tune.filletMm (tuning.h). The host test checks the
// default filleted path clears every fixed box, the forbidden zone and the 4 candidates.

struct Seg { float x0, y0, h0, len, k, s0; };  // constant curvature: k=0 line, k>0 left turn
struct PathPoint { float x, y, h, k; };

struct Path {
    static constexpr unsigned Max = 2*ROUTE_COUNT;
    Seg seg[Max]{};
    unsigned count = 0;
    float length = 0;
    float nodeS[ROUTE_COUNT]{};  // station where each node is passed (arc midpoint)

    void add(float x, float y, float h, float len, float k) {
        if (len < 0.5f || count >= Max) return;
        seg[count++] = {x, y, h, len, k, length};
        length += len;
    }
    void build() {
        count = 0; length = 0;
        float px = ROUTE[0].x, py = ROUTE[0].y;
        for (int i = 1; i < ROUTE_COUNT; ++i) {
            const float inX = ROUTE[i].x-ROUTE[i-1].x, inY = ROUTE[i].y-ROUTE[i-1].y;
            const float hin = atan2f(inY, inX);
            float tangent = 0, turn = 0, hout = hin;
            if (i+1 < ROUTE_COUNT) {
                const float outX = ROUTE[i+1].x-ROUTE[i].x, outY = ROUTE[i+1].y-ROUTE[i].y;
                hout = atan2f(outY, outX);
                turn = wrap(hout-hin);
                // Keep each fillet inside the shorter neighbouring leg.
                const float room = 0.45f*fminf(hypotf(inX, inY), hypotf(outX, outY));
                tangent = fminf(tune.filletMm*tanf(0.5f*fabsf(turn)), room);
            }
            const float ex = ROUTE[i].x - tangent*cosf(hin), ey = ROUTE[i].y - tangent*sinf(hin);
            add(px, py, hin, hypotf(ex-px, ey-py), 0);
            if (tangent > 0.5f && fabsf(turn) > 0.01f) {
                const float r = tangent/tanf(0.5f*fabsf(turn));
                add(ex, ey, hin, fabsf(turn)*r, (turn > 0 ? 1.0f : -1.0f)/r);
                nodeS[i] = length - 0.5f*fabsf(turn)*r;
                px = ROUTE[i].x + tangent*cosf(hout); py = ROUTE[i].y + tangent*sinf(hout);
            } else {
                nodeS[i] = length; px = ROUTE[i].x; py = ROUTE[i].y;
            }
        }
    }
    unsigned find(float s) const {
        unsigned i = 0;
        while (i+1 < count && s >= seg[i+1].s0) ++i;
        return i;
    }
    static PathPoint eval(const Seg &g, float u) {
        if (!g.k) return {g.x0+u*cosf(g.h0), g.y0+u*sinf(g.h0), g.h0, 0};
        const float h = g.h0 + g.k*u;
        return {g.x0+(sinf(h)-sinf(g.h0))/g.k, g.y0-(cosf(h)-cosf(g.h0))/g.k, wrap(h), g.k};
    }
    PathPoint at(float s) const {
        const Seg &g = seg[find(s)];
        return eval(g, bound(s-g.s0, 0, g.len));
    }
    float curvature(float s) const { return (s < 0 || s > length) ? 0 : seg[find(s)].k; }
    // Along-segment distance u (unclamped) and lateral offset e (+ = left of the path).
    static void local(const Seg &g, float x, float y, float &u, float &e) {
        if (!g.k) {
            const float c = cosf(g.h0), s = sinf(g.h0), dx = x-g.x0, dy = y-g.y0;
            u = dx*c + dy*s; e = dy*c - dx*s;
            return;
        }
        const float r = 1/g.k;  // signed: centre on the left for k>0
        const float cx = g.x0 - r*sinf(g.h0), cy = g.y0 + r*cosf(g.h0);
        const float ax = g.x0-cx, ay = g.y0-cy, bx = x-cx, by = y-cy;
        u = atan2f(ax*by-ay*bx, ax*bx+ay*by)*r;
        e = (fabsf(r) - hypotf(bx, by))*(g.k > 0 ? 1 : -1);
    }
    // Station of the robot. The segment index only moves forward.
    float track(unsigned &i, float x, float y, float &lateral) const {
        float u = 0;
        for (;;) {
            local(seg[i], x, y, u, lateral);
            if (u <= seg[i].len || i+1 >= count) break;
            ++i;
        }
        return seg[i].s0 + u;
    }
    // Closest path point to (x,y) among segments overlapping [from,to].
    bool nearest(float x, float y, float from, float to, float &s, float &lat) const {
        float best = 1.0e9f;
        bool found = false;
        for (unsigned i = find(from); i < count && seg[i].s0 <= to; ++i) {
            float u, e;
            local(seg[i], x, y, u, e);
            const float uc = bound(u, 0, seg[i].len);
            const PathPoint p = eval(seg[i], uc);
            const float d = hypotf(x-p.x, y-p.y);
            if (d < best) { best = d; s = seg[i].s0+uc; lat = e < 0 ? -d : d; found = true; }
        }
        return found;
    }
};
