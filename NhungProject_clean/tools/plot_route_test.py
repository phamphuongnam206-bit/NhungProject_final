#!/usr/bin/env python3
"""Plot a test_static_route UART capture. Units in the plot are centimetres."""
import argparse
import csv
from dataclasses import dataclass, field
from math import atan2, ceil, cos, degrees, hypot, isfinite, radians, sin
from pathlib import Path
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch, Patch, Rectangle
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

ROUTE_FILE = Path(__file__).resolve().parents[1] / "include/map_config.h"
COLUMNS = "TIME_MS X_MM Y_MM YAW_DEG WAYPOINT LEFT_SPEED RIGHT_SPEED HEADING_ERROR CROSS_TRACK_ERROR".split()
HALF_LENGTH_CM, HALF_WIDTH_CM = 6.5, 5.0


@dataclass
class Run:
    samples: list = field(default_factory=list)  # (ms, x cm, y cm, yaw rad)
    meta: dict = field(default_factory=dict)
    events: list = field(default_factory=list)
    path_segments: list = field(default_factory=list)
    obstacles: list = field(default_factory=list)
    avoids: list = field(default_factory=list)
    phases: set = field(default_factory=set)
    simulated: bool = False


def map_config():
    text = ROUTE_FILE.read_text(encoding="utf-8")
    def literal(name):
        return int(re.search(r"constexpr int\s+" + name + r"\s*=\s*(-?\d+)\s*;", text).group(1))
    values = {name: literal(name) for name in ("START_X", "GOAL_X", "GOAL_Y", "ROBOT_LENGTH")}
    # The map defines START_Y as -ROBOT_LENGTH / 2.
    start_y = re.search(r"constexpr int\s+START_Y\s*=\s*([^;]+);", text).group(1).strip()
    values["START_Y"] = (-(values["ROBOT_LENGTH"] // 2) if start_y == "-ROBOT_LENGTH / 2" else int(start_y))
    route = re.search(r"constexpr Waypoint ROUTE\[\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
    pairs = re.findall(r"\{\s*([A-Z_]+|-?\d+)\s*,\s*([A-Z_]+|-?\d+)\s*\}", route)
    points = [tuple((values[v] if v in values else int(v))/10 for v in pair) for pair in pairs]
    if len(points) != 5:
        raise ValueError("MAP cần có 5 waypoint.")
    box_size = literal("BOX_SIZE")

    def obstacle_array(name):
        block = re.search(r"constexpr Obstacle\s+" + name + r"\[\]\s*=\s*\{(.*?)\};", text, re.S)
        if not block:
            raise ValueError(f"Không đọc được {name} từ map_config.h.")
        entries = re.findall(
            r"\{\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*"
            r"(BOX_SIZE|-?\d+)\s*,\s*(BOX_SIZE|-?\d+)\s*\}", block.group(1))
        return [(int(i), int(x)/10, int(y)/10,
                 (box_size if w == "BOX_SIZE" else int(w))/10,
                 (box_size if h == "BOX_SIZE" else int(h))/10)
                for i, x, y, w, h in entries]

    forbidden_match = re.search(
        r"constexpr Rectangle\s+FORBIDDEN\s*=\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*,"
        r"\s*(-?\d+)\s*,\s*(-?\d+)\s*\}", text)
    if not forbidden_match:
        raise ValueError("Không đọc được vùng cấm từ map_config.h.")
    forbidden = tuple(int(v)/10 for v in forbidden_match.groups())
    return points, obstacle_array("FIXED_OBSTACLES"), obstacle_array("POSSIBLE_OBSTACLES"), forbidden


def route_cm():
    return map_config()[0]


def read_runs(filename):
    runs = []
    run = Run()
    next_simulated = False
    columns = COLUMNS
    with open(filename, newline="", encoding="utf-8-sig", errors="replace") as stream:
        for row in csv.reader(stream):
            if not row:
                continue
            tag = row[0].strip()
            if tag == "#SOURCE":
                next_simulated = len(row)>1 and row[1].strip()=="HOST_SIMULATION"
                if not run.samples:
                    run.simulated = next_simulated
            elif tag == "#ROUTE_STARTED":
                if run.samples:
                    runs.append(run)
                run = Run(simulated=next_simulated)
                next_simulated = False
                columns = COLUMNS
            elif tag.startswith("#"):
                if tag in ("#ROUTE_FINISHED", "#ROUTE_FAULT", "#ROUTE_STOPPED"):
                    run.meta["event"] = tag[1:]
                elif tag == "#WAYPOINT_END" and len(row) >= 8:
                    try:
                        n, time, x, y, yaw = int(row[1]), int(row[2]), float(row[5]), float(row[6]), float(row[7])
                        if 0 <= n < 5 and all(map(isfinite, (x, y, yaw))):
                            run.events.append((time, x/10, y/10, radians(yaw)))
                    except ValueError:
                        pass
                elif tag == "#PATH_SEG" and len(row) >= 7:
                    try:
                        segment = (int(float(row[1])), *(float(v) for v in row[2:7]))
                        if all(isfinite(v) for v in segment[1:]):
                            run.path_segments.append(segment)
                    except (ValueError, IndexError):
                        pass
                elif tag == "#OBSTACLE" and len(row) >= 7:
                    try:
                        obstacle = (int(float(row[1])), row[2].strip(), *(float(v) for v in row[3:7]))
                        if all(isfinite(v) for v in obstacle[2:]):
                            run.obstacles.append(obstacle)
                    except (ValueError, IndexError):
                        pass
                elif tag == "#AVOID" and len(row) >= 8:
                    try:
                        avoid = (int(float(row[1])), row[2].strip(), *(float(v) for v in row[3:8]))
                        if all(isfinite(v) for v in avoid[2:]):
                            run.avoids.append(avoid)
                    except (ValueError, IndexError):
                        pass
                elif tag == "#AVOID_FAIL" and len(row) >= 3:
                    try:
                        run.meta.setdefault("AVOID_FAIL", []).append((int(float(row[1])), row[2].strip()))
                    except (ValueError, IndexError):
                        pass
                elif len(row) == 2:
                    run.meta[tag[1:]] = row[1].strip()
            elif tag == "TIME_MS":
                columns = [s.strip() for s in row]
                if any(c not in columns for c in COLUMNS):
                    raise ValueError("Header CSV thiếu trường của firmware test.")
            elif tag.isdecimal() and len(row) == len(columns):
                try:
                    fields = dict(zip(columns, row))
                    t = int(fields["TIME_MS"])
                    x, y, yaw = (float(fields[c]) for c in ("X_MM", "Y_MM", "YAW_DEG"))
                    if not all(map(isfinite, (x, y, yaw))):
                        continue
                except ValueError:
                    continue  # partial UART line
                phase = fields.get("PHASE", "").strip()
                if phase:
                    run.phases.add(phase)
                if run.samples and t < run.samples[-1][0]:
                    runs.append(run)  # bare CSV with elapsed time restarting
                    run = Run()
                run.samples.append((t, x/10, y/10, radians(yaw)))
    if run.samples:
        runs.append(run)
    if not runs:
        raise ValueError("File chưa có mẫu Odometry hợp lệ.")
    return runs


def sample_path(segments, start=None, end=None, step=5.0):
    """Sample PATH_SEG geometry in mm; optional limits are global stations."""
    samples = []
    station = 0.0
    for _, x0, y0, h0_deg, length, radius in sorted(segments, key=lambda item: item[0]):
        lo = 0.0 if start is None else max(0.0, start-station)
        hi = length if end is None else min(length, end-station)
        overlaps = length >= 0 and lo <= hi
        overlaps = overlaps and (end is None or end >= station)
        overlaps = overlaps and (start is None or start <= station+length)
        if overlaps:
            count = max(1, ceil((hi-lo)/step))
            for n in range(count+1):
                u = lo + (hi-lo)*n/count
                h0 = radians(h0_deg)
                if abs(radius) < 1e-9:
                    heading = h0
                    x, y = x0 + u*cos(h0), y0 + u*sin(h0)
                else:
                    curvature = 1.0/radius
                    heading = h0 + curvature*u
                    x = x0 + (sin(heading)-sin(h0))/curvature
                    y = y0 - (cos(heading)-cos(h0))/curvature
                if not samples or abs(samples[-1][0]-(station+u)) > 1e-7:
                    samples.append((station+u, x, y, heading))
        station += max(0.0, length)
    return samples


def avoidance_path(segments, avoid):
    _, _, offset, s0, s1, s2, s3 = avoid
    result = []
    for station, x, y, heading in sample_path(segments, s0, s3):
        if station < s1 and s1 > s0:
            lateral = offset*.5*(1-cos(3.141592653589793*(station-s0)/(s1-s0)))
        elif station <= s2:
            lateral = offset
        elif s3 > s2:
            lateral = offset*.5*(1+cos(3.141592653589793*(station-s2)/(s3-s2)))
        else:
            lateral = 0.0
        result.append((x-lateral*sin(heading), y+lateral*cos(heading)))
    return result


def poses(run):
    # Endpoint events and the summary can survive a dropped periodic sample.
    by_time = {p[0]: p for p in run.samples + run.events}
    if all(k in run.meta for k in ("TOTAL_TIME_MS", "FINAL_X_MM", "FINAL_Y_MM", "FINAL_YAW_DEG")):
        t = int(run.meta["TOTAL_TIME_MS"])
        x, y, yaw = (float(run.meta[k]) for k in ("FINAL_X_MM", "FINAL_Y_MM", "FINAL_YAW_DEG"))
        if not all(map(isfinite, (x, y, yaw))):
            raise ValueError("Pose cuối không hợp lệ.")
        by_time[t] = t, x/10, y/10, radians(yaw)
    return [by_time[t] for t in sorted(by_time)]


def body(x, y, yaw):
    u, v = cos(yaw), sin(yaw)
    return Polygon([(x+u*a-v*b, y+v*a+u*b)
                    for a, b in ((HALF_LENGTH_CM, HALF_WIDTH_CM), (-HALF_LENGTH_CM, HALF_WIDTH_CM),
                                 (-HALF_LENGTH_CM, -HALF_WIDTH_CM), (HALF_LENGTH_CM, -HALF_WIDTH_CM))])


def swept_area(path):
    # Interpolate by the shortest angular arc, including stationary pivots.
    # Hulls of adjacent rectangles fill the translation between samples.
    _, x, y, yaw = path[0]
    previous = body(x, y, yaw)
    strips = [previous]
    for _, nx, ny, nyaw in path[1:]:
        angle = atan2(sin(nyaw-yaw), cos(nyaw-yaw))
        steps = max(1, ceil(abs(degrees(angle))/2), ceil(hypot(nx-x, ny-y)/0.4))
        for i in range(1, steps+1):
            f = i/steps
            polygon = body(x+f*(nx-x), y+f*(ny-y), yaw+f*angle)
            strips.append(previous.union(polygon).convex_hull)
            previous = polygon
        x, y, yaw = nx, ny, nyaw
    return unary_union(strips)


def fill_polygon(ax, geometry, color, opacity):
    polys = [geometry] if geometry.geom_type=="Polygon" else geometry.geoms
    for poly in polys:
        poly = orient(poly, sign=1)
        rings = [MplPath(list(r.coords), closed=True) for r in (poly.exterior, *poly.interiors)]
        patch = PathPatch(MplPath.make_compound_path(*rings), linewidth=0,
                          facecolor=color, alpha=opacity, zorder=1)
        ax.add_patch(patch)
        ax.update_datalim(list(poly.exterior.coords))


def plot(run, out, margin, dpi, number):
    route, fixed_obstacles, possible_obstacles, forbidden = map_config()
    path = poses(run)
    area = swept_area(path)
    fig, ax = plt.subplots(figsize=(10, 7.7))
    fig.subplots_adjust(left=.105, right=.96, top=.84, bottom=.20)
    blue, red, amber = "#225dad", "#b02338", "#edc85a"
    title = "Bám lộ trình ôm cua" if run.phases.intersection(("TRACK", "AVOID")) else "Bám lộ trình tĩnh"
    if run.simulated:
        title += "  ·  Mô phỏng trên máy tính"
    fig.text(.105,.95,title,fontsize=20,weight="bold",color="#20344b")
    _, x, y, ang = path[-1]
    error = hypot(x-route[-1][0], y-route[-1][1])
    reason = run.meta.get("STOP_REASON", "LOG_CHƯA_KẾT_THÚC")
    complete = run.meta.get("event")=="ROUTE_FINISHED" and reason=="GOAL_REACHED"
    label = "Sai số cuối đến GOAL" if complete else "Khoảng cách từ mẫu cuối đến GOAL"
    fig.text(.105,.9, f"Lượt {number}   ·   {label} {error:.2f} cm   ·   {reason}",
             fontsize=11, color="#44566b")
    xmin, ymin, xmax, ymax = forbidden
    ax.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, facecolor="#efe7d5",
                           edgecolor="#b9a780", hatch="///", linewidth=.8, alpha=.5, zorder=0))
    for obstacle_id, ox, oy, width, height in fixed_obstacles:
        ax.add_patch(Rectangle((ox-width/2, oy-height/2), width, height,
                               facecolor="#9da4aa", edgecolor="#6e757b", linewidth=.7,
                               alpha=.65, zorder=.4))
        ax.text(ox, oy, str(obstacle_id), ha="center", va="center", fontsize=7,
                color="#34393d", zorder=.6)
    for obstacle_id, ox, oy, width, height in possible_obstacles:
        ax.add_patch(Rectangle((ox-width/2, oy-height/2), width, height,
                               facecolor="none", edgecolor="#747b82", linestyle="--",
                               linewidth=1, zorder=.5))
        ax.text(ox, oy, str(obstacle_id), ha="center", va="center", fontsize=7,
                color="#555d64", zorder=.6)
    if margin:
        fill_polygon(ax, area.buffer(margin), amber, .43)
    fill_polygon(ax, area, red, .19)
    # A slightly offset route must remain visible where it coincides with odometry.
    odom, = ax.plot([p[1] for p in path], [p[2] for p in path], color=red, lw=2.1,
                    zorder=3, label="Odometry ước lượng")
    rx, ry = zip(*route)
    ref, = ax.plot(rx, ry, color=blue, ls=(0,(5,4)), lw=1.6, zorder=4, label="Lộ trình theo prompt")
    smooth_ref = None
    if run.path_segments:
        smooth = sample_path(run.path_segments)
        if smooth:
            smooth_ref, = ax.plot([p[1]/10 for p in smooth], [p[2]/10 for p in smooth],
                                  color=blue, lw=2, zorder=4.2,
                                  label="Đường tham chiếu (ôm cua)")
    avoid_handle = None
    for avoid in run.avoids:
        detour = avoidance_path(run.path_segments, avoid)
        if detour:
            line, = ax.plot([p[0]/10 for p in detour], [p[1]/10 for p in detour],
                            color="#16864a", lw=2.2, zorder=4.4, label="Đường né")
            if avoid_handle is None:
                avoid_handle = line
    for obstacle_id, kind, ox, oy, _, _ in run.obstacles:
        size = 10.0
        ax.add_patch(Rectangle((ox/10-size/2, oy/10-size/2), size, size,
                               facecolor="#ee8b2d", edgecolor="#a84e0a", linewidth=1,
                               alpha=.78, zorder=4.5))
        ax.annotate(f"{kind}{obstacle_id}", (ox/10, oy/10), xytext=(0, 2),
                    textcoords="offset points", ha="center", va="bottom", fontsize=8,
                    weight="bold", color="#773500", zorder=5)
    for i, (px, py) in enumerate(route):
        color = "#177448" if i==0 else ("#703b9c" if i==len(route)-1 else blue)
        name = "START" if i==0 else ("GOAL" if i==len(route)-1 else f"WP{i}")
        ax.scatter(px, py, s=48, facecolor=color, edgecolor="white", linewidth=.9, zorder=5)
        ax.annotate(name, (px, py), xytext=(7,6), textcoords="offset points", zorder=6,
                    fontsize=9, weight="bold", color=color)
    ax.plot([x,route[-1][0]],[y,route[-1][1]], color="#54334f", ls=":", lw=1.2)
    ax.scatter(x,y,marker="x",s=45,color="#391b2b",zorder=6)
    ax.set(aspect="equal", xlabel="X (cm)", ylabel="Y (cm)")
    ax.set_axisbelow(True); ax.grid(color="#dce2e8", lw=.65)
    ax.margins(x=.08, y=.10)
    for spine in ax.spines.values():
        spine.set_color("#a9b5c0")
    handles = [ref]
    if smooth_ref is not None:
        handles.append(smooth_ref)
    if avoid_handle is not None:
        handles.append(avoid_handle)
    handles.extend([odom, Patch(facecolor=red, alpha=.19, label="Vệt thân xe 13 × 10 cm")])
    if margin:
        handles.append(Patch(facecolor=amber, alpha=.43, label=f"Biên an toàn {margin:g} cm"))
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5,.08), frameon=False,
               ncol=2, fontsize=10)
    fig.text(.5,.045,"Tâm vệt quét đặt tại pose. Quỹ đạo vật lý cần phép đo độc lập.",
             ha="center", fontsize=9, color="#57697b")
    fig.savefig(out,dpi=dpi,facecolor="white")
    plt.close(fig)
    return error


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("-o","--output",type=Path,default=Path("route_test.png"))
    ap.add_argument("--run",type=int, help="Số lượt tính từ 1. Mặc định lấy lượt cuối.")
    ap.add_argument("--safety-cm", type=float, default=0, help="Biên bổ sung, ví dụ 1.5")
    ap.add_argument("--dpi", type=int, default=300)
    args = ap.parse_args()
    if args.safety_cm < 0 or not isfinite(args.safety_cm) or args.dpi < 1:
        ap.error("Biên an toàn phải hữu hạn, không âm và DPI phải dương.")
    try:
        runs = read_runs(args.csv)
        number = len(runs) if args.run is None else args.run
        if not 1 <= number <= len(runs):
            ap.error(f"Log có {len(runs)} lượt. Chọn từ 1 đến {len(runs)}.")
        gap = plot(runs[number-1], args.output, args.safety_cm, args.dpi, number)
    except (OSError, ValueError) as err:
        sys.exit(str(err))
    print(f"{args.output} | lượt {number}/{len(runs)} | từ pose cuối đến GOAL {gap:.2f} cm")


if __name__ == "__main__":
    main()
