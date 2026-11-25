"""Host-Simulator mit C++17 bauen, auch ohne make im Windows-Entwicklerterminal.

Aufruf: python firmware/host_sim/build.py
Windows: Developer Command Prompt mit MSVC cl; alternativ CXX auf clang++/g++ setzen.
"""
from __future__ import annotations

import os
import shlex
import shutil
import subprocess
from pathlib import Path


def build() -> Path:
    root = Path(__file__).resolve().parent
    target = root / ("mcu_sim.exe" if os.name == "nt" else "mcu_sim")
    configured = os.environ.get("CXX")
    compiler = shlex.split(configured) if configured else next(
        ([path] for name in ("cl", "clang++", "g++") if (path := shutil.which(name))), None)
    if compiler is None:
        raise RuntimeError("C++17-Compiler fehlt. Windows: Developer Command Prompt mit cl verwenden. "
                           "macOS: Xcode Command Line Tools. Linux: g++ installieren.")
    sources = ["mcu_sim.cpp", *[f"../followme_mcu/src/{name}.cpp" for name in ("proto", "supervisor", "wheel_ctrl", "app")]]
    if Path(compiler[0]).stem.lower() == "cl":
        flags = ["/nologo", "/std:c++17", "/O2", "/EHsc", "/D_CRT_SECURE_NO_WARNINGS", f"/Fe:{target}"]
    else:
        flags = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-o", str(target)]
    subprocess.run([*compiler, *flags, *sources], cwd=root, check=True)
    return target


if __name__ == "__main__":
    print(build())
