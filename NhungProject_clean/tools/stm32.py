"""Build/upload/monitor the firmware .cpp that is currently open (Ctrl+Shift+B).

- File registered in platformio.ini (build_src_filter) -> use that env.
- Any other .cpp in src/ that has setup() -> build it alone with [env:auto],
  no need to edit platformio.ini.
- Serial baud always comes from monitor_speed in platformio.ini.
"""
from configparser import ConfigParser
from pathlib import Path
import json
import os
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
AUTO_ENV = "auto"
DEFAULT_BAUD = "115200"


def load_config():
    config = ConfigParser(interpolation=None)
    config.read(ROOT / "platformio.ini", encoding="utf-8")
    return config


def env_baud(config, env):
    return config.get(f"env:{env}", "monitor_speed", fallback=config.get("env", "monitor_speed", fallback=DEFAULT_BAUD))


def detect_baud(source):
    """Baud in Serial.begin(...) (number or #define/const in the file); None if unknown."""
    text = source.read_text(encoding="utf-8", errors="ignore")
    m = re.search(r"\bSerial\w*\.begin\s*\(\s*([A-Za-z_]\w*|\d+)", text)
    if not m:
        return None
    value = m.group(1)
    if value.isdigit():
        return value
    d = re.search(rf"#define\s+{value}\s+(\d+)|\b{value}\s*=\s*(\d+)", text)
    return (d.group(1) or d.group(2)) if d else None


def select_target(source):
    """Return (env, baud, display_name, src_filter_override_or_None)."""
    config = load_config()
    default = config.get("platformio", "default_envs", fallback="").split(",")[0].strip()

    def fallback(reason):
        if not default:
            raise ValueError(reason)
        print(f"{reason} -> dung firmware mac dinh: {default}", flush=True)
        return default, env_baud(config, default), env_sources(config, default), None

    try:
        source = Path(source).resolve()
        relative = source.relative_to(ROOT / "src").as_posix()
    except ValueError:
        return fallback("File dang mo khong nam trong src/")
    if not source.is_file() or source.suffix != ".cpp":
        return fallback("File dang mo khong phai file .cpp")

    for section in config.sections():
        if not section.startswith("env:") or section == f"env:{AUTO_ENV}":
            continue
        filters = config.get(section, "build_src_filter", fallback="").split()
        includes = [f for f in filters if f.startswith("+<")]
        if includes and includes[0] == f"+<{relative}>":
            env = section[4:]
            return env, env_baud(config, env), env_sources(config, env), None

    text = source.read_text(encoding="utf-8", errors="ignore")
    if not re.search(r"\bvoid\s+setup\s*\(", text):
        return fallback(f"src/{relative} khong co setup() (file phu)")
    return AUTO_ENV, env_baud(config, AUTO_ENV), f"src/{relative}", f"-<*> +<{relative}>"


def warn_baud_mismatch(name, baud):
    """Baud always comes from platformio.ini; warn if the code's Serial.begin() disagrees."""
    for path in name.split(", "):
        source = ROOT / path
        if not source.is_file():
            continue
        code_baud = detect_baud(source)
        if code_baud and code_baud != baud:
            print(f"CANH BAO: {path} dung Serial.begin({code_baud}) nhung platformio.ini monitor_speed = {baud}.\n"
                  f"          Sua monitor_speed trong platformio.ini cho khop neu Serial ra ky tu la.", flush=True)


def env_sources(config, env):
    filters = config.get(f"env:{env}", "build_src_filter", fallback="").split()
    files = [f"src/{f[2:-1]}" for f in filters if f.startswith("+<")]
    return ", ".join(files) or env


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ("upload", "monitor", "build"):
        print("Usage: stm32.py upload|monitor|build path/to/firmware.cpp")
        return 2
    try:
        env, baud, name, src_filter = select_target(sys.argv[2])
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2
    sibling = Path(sys.executable).with_name("pio.exe" if os.name == "nt" else "pio")
    pio = str(sibling) if sibling.is_file() else shutil.which("pio")
    if not pio:
        print("PlatformIO not found.", file=sys.stderr)
        return 2
    action = sys.argv[1]
    child_env = os.environ.copy()
    child_env.pop("PLATFORMIO_BUILD_SRC_FILTER", None)
    if src_filter:
        child_env["PLATFORMIO_BUILD_SRC_FILTER"] = src_filter

    if action != "monitor":
        verb = "NAP CODE" if action == "upload" else "BUILD"
        bar = "=" * 60
        print(f"{bar}\n {verb}: {name}  (env: {env})\n{bar}", flush=True)
        cmd = [pio, "run", "-e", env]
        if action == "upload":
            cmd += ["-t", "upload"]
        rc = subprocess.call(cmd, cwd=ROOT, env=child_env)
        status = "THANH CONG" if rc == 0 else f"THAT BAI (ma loi {rc})"
        print(f"\n{bar}\n {verb} {name}: {status}\n{bar}", flush=True)
        return rc

    print(f"Doc Serial: {name}  (baud {baud}, lay tu platformio.ini)", flush=True)
    warn_baud_mismatch(name, baud)
    raw = subprocess.check_output([pio, "device", "list", "--json-output"], cwd=ROOT, env=child_env, text=True)
    ports = json.loads(raw)
    if not ports:
        print("Khong tim thay cong serial nao.")
        return 1
    if len(ports) == 1:
        port = ports[0]["port"]
        print(f"Cong serial: {port} - {ports[0].get('description', '')}")
    else:
        print("Cac cong COM dang co:")
        for p in ports:
            print(f"  {p['port']} - {p.get('description', '')}")
        port = ask_port(ports)
        if port is None:
            return 0
    return subprocess.call([pio, "device", "monitor", "-p", port, "-b", baud], cwd=ROOT, env=child_env)


def ask_port(ports):
    """Ask for the COM number only: typing 4 selects COM4. Returns None on quit."""
    names = {p["port"].upper(): p["port"] for p in ports}
    while True:
        try:
            choice = input("Nhap so COM (vd: 4 = COM4, q de thoat): ").strip().upper()
        except EOFError:
            return None
        if choice == "Q":
            return None
        if choice.isdigit():
            choice = f"COM{int(choice)}"
        if choice in names:
            return names[choice]
        print(f"Khong co {choice}. Chon mot trong: {', '.join(names.values())}")


if __name__ == "__main__":
    sys.exit(main())
