"""Gebündelten Host-Simulator nur bei passender Plattform und Quellprüfsumme nutzen."""
from __future__ import annotations

import hashlib
import json
import platform
import sys
from pathlib import Path


def platform_key() -> str:
    machine = platform.machine().lower()
    machine = {"amd64": "x86_64", "aarch64": "arm64"}.get(machine, machine)
    return f"{sys.platform}-{machine}"


def source_hash(firmware: Path) -> str:
    files = [firmware / "host_sim/mcu_sim.cpp", firmware / "host_sim/wheel_plant.h", firmware / "followme_mcu/config.h"]
    files.extend(p for p in (firmware / "followme_mcu/src").iterdir() if p.suffix in {".h", ".cpp"})
    digest = hashlib.sha256()
    for path in sorted(files):
        digest.update(path.relative_to(firmware).as_posix().encode() + b"\0" + path.read_bytes() + b"\0")
    return digest.hexdigest()


def bundled_binary(host: Path) -> Path | None:
    folder = host / "prebuilt" / platform_key()
    manifest_path = folder / "manifest.json"
    if not manifest_path.exists():
        return None
    manifest = json.loads(manifest_path.read_text())
    binary = folder / ("mcu_sim.exe" if sys.platform == "win32" else "mcu_sim")
    if manifest.get("platform") != platform_key() or manifest.get("sources_sha256") != source_hash(host.parent):
        raise ValueError("Gebündelter Simulator passt nicht zu Plattform/Quellstand. Team-Paket neu erzeugen.")
    if not binary.is_file() or hashlib.sha256(binary.read_bytes()).hexdigest() != manifest.get("binary_sha256"):
        raise ValueError("Prüfsumme des gebündelten Simulators stimmt nicht. Team-Paket neu entpacken.")
    if sys.platform != "win32":
        binary.chmod(binary.stat().st_mode | 0o100)  # zipfile erhält das Ausführungsbit beim Entpacken nicht immer.
    return binary
