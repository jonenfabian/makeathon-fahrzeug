"""Pico-Status nur lesen: keine Kamera und keine ARM-/CMD-/STOP-Befehle."""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict
from pathlib import Path

from .link import STATUS_MAX_AGE_S, McuLink, open_serial, open_sim
from .protocol import TOF_NO_TARGET, supported_protocol
from .sensors import side_views

_SIDE_TEXT = {"too_close": "zu nah", "no_target": "kein Ziel", "fault": "Fehler", "stale": "veraltet",
              "unavailable": "-"}


def _side_text(view: dict) -> str:
    """Ultraschall kurz: Abstand oder Zustand (nur Messwert, keine Freigabe)."""
    return f"{view['distance_mm']} mm" if view["state"] == "distance" else _SIDE_TEXT.get(view["state"], view["state"])


def monitor(link: McuLink, duration_s: float, *, emit=print) -> dict:
    """Begrenzte Leseprobe; ESTOP ist ein gültiger Messzustand, keine Fahrfreigabe."""
    if not math.isfinite(duration_s) or not 0 < duration_s <= 600:
        raise ValueError("Dauer muss > 0 und <= 600 Sekunden sein")
    report = {"format": 1, "mode": "read_only", "commands_sent": 0, "ready_for_drive": False,
              "complete": False, "samples": [], "info": None,
              "notice": "Motorspannung hinter Not-Aus; keine Akku-Ladezustandsmessung. ToF-Einzelmesszeit unbekannt."}
    try:
        link.wait_status(timeout_s=3)
        start_s = time.monotonic()
        while time.monotonic() - start_s < duration_s:
            status, age_s, error = link.status_snapshot()
            if error or status is None or age_s is None or age_s > STATUS_MAX_AGE_S:
                raise ConnectionError(error or "Keine frische STAT-Meldung; alte Werte nicht weiterverwenden")
            info = link.info
            if info:
                report["info"] = asdict(info)
                if not supported_protocol(info.proto_version):
                    raise ConnectionError(f"Nicht unterstützte Protokollversion: {info.proto_version}")
            sides = side_views(link.ultrasonic_snapshot(), time.monotonic())
            report["samples"].append({"elapsed_s": time.monotonic() - start_s,
                                      "status_age_s": age_s, "status": asdict(status), "ultrasonic": sides})
            tof = "kein Ziel" if status.tof_mm == TOF_NO_TARGET else f"{status.tof_mm} mm"
            emit(f"{status.state:7s} | Motor {status.vmot_mv / 1000:5.2f} V | ToF {tof:10s} | "
                 f"US L {_side_text(sides['links']):10s} R {_side_text(sides['rechts']):10s} | "
                 f"Encoder L {status.ticks_l:+8d} R {status.ticks_r:+8d} | "
                 f"Rad L {status.v_l_mm_s:+5d} R {status.v_r_mm_s:+5d} mm/s | "
                 f"Flags {' '.join(status.flag_names()) or '-'}")
            time.sleep(min(.5, max(0, duration_s - (time.monotonic() - start_s))))
        if report["info"] is None:
            raise ConnectionError("Keine INFO-Meldung; Firmware/Protokollversion noch unbekannt")
        report["complete"] = True
    except KeyboardInterrupt:
        report["error"] = "Leseprobe unterbrochen"
    except (OSError, RuntimeError, ValueError, TimeoutError) as exc:
        report["error"] = str(exc)
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--sekunden", type=float, default=30)
    ap.add_argument("--ausgabe", type=Path)
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args(argv)
    if not math.isfinite(args.sekunden) or not 0 < args.sekunden <= 600:
        ap.error("Dauer muss > 0 und <= 600 Sekunden sein")
    if args.ausgabe and args.ausgabe.exists():
        ap.error("Ausgabedatei existiert bereits; anderen Namen wählen")
    print("Nur lesen. Motoranzeige hinter Not-Aus; keine Akku-Ladezustandsanzeige.", flush=True)
    try:
        with (open_sim() if args.sim else open_serial(args.port)) as link:
            report = monitor(link, args.sekunden)
    except (OSError, RuntimeError, ValueError, TimeoutError) as exc:
        report = {"format": 1, "mode": "read_only", "complete": False, "commands_sent": 0,
                  "ready_for_drive": False, "error": str(exc)}
    report["simulated"] = args.sim
    if args.ausgabe:
        with args.ausgabe.open("x", encoding="utf-8") as file:
            json.dump(report, file, indent=2, ensure_ascii=False, allow_nan=False)
            file.write("\n")
        print(f"Protokoll: {args.ausgabe}")
    if not report["complete"]:
        print(f"Leseprobe unvollständig: {report['error']}")
    return int(not report["complete"])


if __name__ == "__main__":
    raise SystemExit(main())
