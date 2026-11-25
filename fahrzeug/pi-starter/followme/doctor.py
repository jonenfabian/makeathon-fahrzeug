"""Diagnose vor der Inbetriebnahme. Keine ARM- oder Fahrbefehle, keine Freigabe."""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import importlib
import importlib.util
import json
import os
import platform
import sys
from dataclasses import asdict
from pathlib import Path

from .config import camera_model, load_config
from .coral import default_library


def diagnose(values: dict, *, target_pi: bool = False, hardware: bool = False, sim: bool = False) -> dict:
    checks = []

    def add(name, status, detail):
        checks.append({"name": name, "status": status, "detail": detail})

    add("python", "ok" if sys.version_info >= (3, 10) else "error", platform.python_version())
    add("architektur", "error" if target_pi and platform.machine() != "aarch64" else "ok", platform.machine())
    release = Path("/etc/os-release")
    os_text = release.read_text() if release.exists() else platform.platform()
    add("betriebssystem", "ok" if not target_pi or "trixie" in os_text.lower() else "warning", os_text)
    for name in ("numpy", "cv2", "serial"):
        try:
            module = importlib.import_module(name)
            add(name, "ok", f"Import erfolgreich: {getattr(module, '__version__', 'ohne Versionsangabe')}")
        except (OSError, ImportError) as exc:
            add(name, "error", str(exc))
    if values["quelle"] == "picam":
        if target_pi:
            found = importlib.util.find_spec("picamera2") is not None
            add("picamera2", "ok" if found else "error", "Systempaket python3-picamera2 und venv --system-site-packages")
        else:
            add("picamera2", "skip", "Nicht auf Ziel-Pi geprüft")
    if values["erkenner"] in ("person", "coral"):
        module = "mediapipe" if values["erkenner"] == "person" else "ai_edge_litert"
        add("nn_runtime", "ok" if importlib.util.find_spec(module) else "error", module)
        model = Path(values["modell"] if values["erkenner"] == "person" else values["coral_modell"])
        if model.is_file():
            data = model.read_bytes()
            valid = data[4:8] == b"TFL3"
            if values["erkenner"] == "coral":
                valid = valid and b"edgetpu-custom-op" in data
            add("modell", "ok" if valid else "error",
                {"path": str(model), "sha256": hashlib.sha256(data).hexdigest(), "flatbuffer": valid})
        else:
            add("modell", "error", str(model))
    if values["erkenner"] == "coral":
        library = values["coral_bibliothek"] or default_library()
        try:
            ctypes.CDLL(library)
            add("coral_bibliothek", "ok", library)
        except OSError as exc:
            add("coral_bibliothek", "error", f"{exc}; Einrichtung siehe CORAL.md")
        if target_pi:
            import grp

            rule = Path("/etc/udev/rules.d/99-followme-coral.rules")
            add("coral_udev", "ok" if rule.is_file() else "error", str(rule))
            try:
                group = grp.getgrnam("followme-coral")
                member = group.gr_gid in {*os.getgroups(), os.getegid()}
                add("coral_gruppe", "ok" if member else "error", "followme-coral; ggf. neu anmelden")
            except KeyError:
                add("coral_gruppe", "error", "Gruppe followme-coral fehlt; siehe CORAL.md")
    if hardware:
        from .camera import Camera

        cam = camera_model(values)
        camera = None
        detector = None
        try:
            from .follow import make_detector

            detector = make_detector(values["erkenner"], cam, argparse.Namespace(**values))
            camera = Camera(values["quelle"], cam.width_px, cam.height_px)
            frame = camera.read_timed()
            if frame is None:
                raise RuntimeError("Keine Kamerabilder")
            targets = detector.detect(frame.image)
            add("kamera_erkennung", "ok", {"timestamp_source": frame.timestamp_source, "targets": len(targets)})
        except (OSError, RuntimeError, ValueError, ImportError) as exc:
            add("kamera_erkennung", "error", str(exc))
        finally:
            if camera:
                camera.close()
            if detector and hasattr(detector, "close"):
                detector.close()
    else:
        add("kamera_erkennung", "skip", "Nur mit --hardware öffnen; kein Kameratest behauptet")
    if hardware or sim:
        from .link import open_serial, open_sim

        try:
            with (open_sim() if sim else open_serial(values["port"])) as link:
                status = link.wait_status(timeout_s=3)
                add("pico_sim" if sim else "pico", "ok", asdict(status))
                add("pico_standby", "ok" if status.state == "STANDBY" else "warning", status.state)
                # Absichtlich weder SetpointSender noch ARM, CMD oder STOP.
        except (OSError, RuntimeError, ValueError, TimeoutError) as exc:
            add("pico_sim" if sim else "pico", "error", str(exc))
    else:
        add("pico", "skip", "Serielle Schnittstelle bleibt geschlossen")
    return {"format": 1, "vehicle": values["fahrzeug"], "hardware_requested": hardware,
            "simulated": sim, "ready_for_drive": False,
            "software_ok": not any(c["status"] == "error" for c in checks), "checks": checks,
            "notice": "Keine elektrische Abnahme, kein Not-Aus-Test und keine Freigabe für Fahrversuche"}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--konfiguration", type=Path)
    ap.add_argument("--ziel-pi", action="store_true", help="ARM64/Trixie/Picamera2 auf dem Zielsystem prüfen")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--hardware", action="store_true", help="Kamera/Erkenner und Pico öffnen; Fahrzeug aufbocken")
    mode.add_argument("--sim", action="store_true", help="Nur echten Firmwarekern im Host-Simulator verbinden")
    ap.add_argument("--ausgabe", type=Path)
    args = ap.parse_args(argv)
    try:
        values = load_config(args.konfiguration)
        report = diagnose(values, target_pi=args.ziel_pi, hardware=args.hardware, sim=args.sim)
    except (OSError, ValueError) as exc:
        ap.error(str(exc))
    data = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    print(data, end="")
    if args.ausgabe:
        with args.ausgabe.open("x", encoding="utf-8") as file:
            file.write(data)
    return int(not report["software_ok"])


if __name__ == "__main__":
    raise SystemExit(main())
