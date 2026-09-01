# -*- coding: utf-8 -*-
"""Robust Windows bootstrap for TankTrouble autonomous combat (auto-aim).

Keeps startup errors visible and records them in startup_error.log.
This file is intentionally small and independent from the game runtime.
"""
from __future__ import annotations

import importlib
import os
import subprocess
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent
LOG = ROOT / "startup_error.log"
REQ = ROOT / "requirements_runtime.txt"


def log(text: str = "") -> None:
    print(text, flush=True)
    with LOG.open("a", encoding="utf-8") as f:
        f.write(text + "\n")


def run(cmd: list[str]) -> int:
    log("RUN: " + " ".join(cmd))
    p = subprocess.run(cmd, cwd=str(ROOT))
    return int(p.returncode)


def ensure_pip() -> None:
    try:
        import pip  # noqa: F401
        return
    except Exception:
        pass
    log("pip not found; trying Python ensurepip...")
    rc = run([sys.executable, "-m", "ensurepip", "--upgrade"])
    if rc != 0:
        raise RuntimeError("pip is unavailable and ensurepip failed")


def missing_runtime_modules() -> list[str]:
    checks = [("numpy", "numpy"), ("PIL", "Pillow")]
    missing: list[str] = []
    for module_name, package_name in checks:
        try:
            importlib.import_module(module_name)
        except Exception:
            missing.append(package_name)
    return missing


def ensure_dependencies() -> None:
    missing = missing_runtime_modules()
    if not missing:
        log("Runtime dependencies: OK")
        return

    log("Missing Python packages: " + ", ".join(missing))
    ensure_pip()
    if not REQ.exists():
        raise FileNotFoundError(str(REQ))

    log("Installing runtime dependencies. Internet access may be required the first time...")
    rc = run([sys.executable, "-m", "pip", "install", "-r", str(REQ)])
    if rc != 0:
        raise RuntimeError("pip install failed with exit code %d" % rc)

    still_missing = missing_runtime_modules()
    if still_missing:
        raise RuntimeError("dependencies still missing after install: " + ", ".join(still_missing))
    log("Runtime dependencies installed successfully.")


def validate_files() -> None:
    required = [
        ROOT / "launcher.py",
        ROOT / "launcher_autonomous.py",
        ROOT / "python" / "navigation_service.py",
        ROOT / "python" / "navigation_bot.py",
        ROOT / "python" / "navigation_mvp.py",
        ROOT / "python" / "combat_ai.py",
        ROOT / "data log" / "ws_server.py",
        ROOT / "src" / "ai" / "ai-bridge.js",
        ROOT / "src" / "ai" / "ai-bridge-autonomous.js",
        ROOT / "src" / "tanktrouble" / "index.html",
        ROOT / "src" / "tanktrouble" / "data.js.base",
        ROOT / "assets" / "mazes.json",
    ]
    absent = [str(p.relative_to(ROOT)) for p in required if not p.exists()]
    if absent:
        raise FileNotFoundError("Required project files are missing: " + ", ".join(absent))
    log("Project files: OK")


def main() -> int:
    try:
        LOG.write_text("", encoding="utf-8")
        log("TankTrouble Autonomous AI startup")
        log("Python: " + sys.version.replace("\n", " "))
        log("Project: " + str(ROOT))
        validate_files()
        ensure_dependencies()
        log("Starting game + autonomous combat AI (auto-aim, no pathfinding)...")
        log("A browser window should open automatically.")
        return run([sys.executable, str(ROOT / "launcher_autonomous.py")])
    except KeyboardInterrupt:
        log("Stopped by user.")
        return 0
    except Exception:
        tb = traceback.format_exc()
        log("STARTUP FAILED")
        log(tb)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
