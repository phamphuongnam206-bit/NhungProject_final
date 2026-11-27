#!/usr/bin/env python3
"""Build and run the static-route host simulation checks."""

import argparse
import glob
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


SCENARIOS = """commands obstacle-map clearance route wrap random14 random15 random16
random17 block block-center late range-stale manual stall-left stall-right wrong-dir
mpu-read mpu-stale mpu-invalid timeout mpu-frozen encoder-jump miss goal coast
init-bias init-systick uart place drift drift-nofix motor motor-nosync""".split()


def find_gpp():
    found = shutil.which("g++")
    if found:
        return Path(found)
    if os.name == "nt":
        base = os.environ.get("LOCALAPPDATA", "")
        pattern = os.path.join(base, "Microsoft", "WinGet", "Packages",
                               "BrechtSanders.WinLibs.POSIX.UCRT_*", "mingw64",
                               "bin", "g++.exe")
        matches = glob.glob(pattern)
        if matches:
            return Path(sorted(matches)[-1])
    return None


def run(command, cwd, env, timeout=120):
    try:
        result = subprocess.run(command, cwd=str(cwd), env=env, timeout=timeout,
                                capture_output=True, text=True, encoding="utf-8",
                                errors="replace")
        output = "\n".join(part.rstrip() for part in
                           (result.stdout, result.stderr) if part.strip())
        return result.returncode, output
    except subprocess.TimeoutExpired as exc:
        parts = [p for p in (exc.stdout, exc.stderr) if p]
        output = "\n".join(p.decode("utf-8", "replace") if isinstance(p, bytes)
                           else p for p in parts)
        return -1, output + "\nHET THOI GIAN (120 giay)"


def show_failure(name, output):
    print("  FAIL {}:".format(name))
    for line in output.splitlines()[-8:]:
        print("    " + line)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="Chay bo kiem thu mo phong tren PC")
    parser.add_argument("--cps", type=int, choices=(400, 700))
    parser.add_argument("--log", type=Path, metavar="DIR")
    parser.add_argument("--no-web", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    gpp = find_gpp()
    if not gpp:
        print("Khong tim thay g++. Hay chay: winget install --id BrechtSanders.WinLibs.POSIX.UCRT -e")
        return 2

    env = os.environ.copy()
    if os.name == "nt":
        env["PATH"] = str(gpp.parent) + os.pathsep + env.get("PATH", "")
    if args.log:
        args.log.mkdir(parents=True, exist_ok=True)

    cps_values = (args.cps,) if args.cps else (400, 700)
    failures = 0
    with tempfile.TemporaryDirectory(prefix="static_route_tests_") as tmp:
        for cps in cps_values:
            exe = Path(tmp) / ("static_route_{}{}".format(cps, ".exe" if os.name == "nt" else ""))
            command = [str(gpp), "-std=c++17", "-O2", "-Wall", "-Iinclude", "-Isrc",
                       "-DSTATIC_ROUTE_CRUISE_CPS={}.0f".format(cps),
                       "test/static_route_host.cpp", "-o", str(exe)]
            code, output = run(command, root, env)
            if code or "warning:" in output.lower():
                failures += len(SCENARIOS)
                show_failure("bien dich {} CPS".format(cps), output)
                print("{} CPS: 0/{} PASS".format(cps, len(SCENARIOS)))
                continue

            passed = 0
            for scenario in SCENARIOS:
                scenario_cmd = [str(exe), scenario]
                if args.log and scenario in ("route", "block", "random15"):
                    scenario_cmd.append(str(args.log / "sim_{}_{}.csv".format(cps, scenario)))
                code, output = run(scenario_cmd, root, env)
                ok = code == 0 and any(line.startswith("PASS") for line in output.splitlines())
                if ok:
                    passed += 1
                else:
                    failures += 1
                    show_failure(scenario, output)
                if scenario in ("route", "block"):
                    for line in output.splitlines():
                        if line and not line.startswith("PASS"):
                            print("  {}: {}".format(scenario, line))
            print("{} CPS: {}/{} PASS".format(cps, passed, len(SCENARIOS)))

    if not args.no_web:
        node = shutil.which("node")
        fallback = Path("C:/Program Files/nodejs/node.exe")
        if not node and fallback.is_file():
            node = str(fallback)
        if node:
            code, output = run([node, "tools/web/check_sim.js"], root, env)
            print(output or "Web check: khong co dau ra")
            if code:
                failures += 1
        else:
            print("Bo qua web check vi chua co Node.js. Cai bang: winget install --id OpenJS.NodeJS.LTS -e")

    print("TONG KET: {} loi".format(failures) if failures else "TONG KET: tat ca dat")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
