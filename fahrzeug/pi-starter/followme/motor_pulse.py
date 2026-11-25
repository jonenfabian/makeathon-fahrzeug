"""Kurzer Einrichtversuch, 0,1 m/s für höchstens 0,3 s Sollwertzeit, danach STOP.

Nur aufgebockt (Räder frei). Keine unabhängige Sicherheitsfunktion: Schutz,
Watchdog und Motoransteuerung bleiben im Pico. Kein automatischer Wiederholversuch.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict
from pathlib import Path

from .link import McuLink, open_serial, open_sim
from .protocol import supported_protocol


def pulse(link: McuLink, *, duration_s: float = .3, speed_mps: float = .1) -> dict:
    if (not math.isfinite(duration_s) or not 0 < duration_s <= .3
            or not math.isfinite(speed_mps) or not 0 < speed_mps <= .1):
        raise ValueError("Einrichtversuch: höchstens 0,3 s und 0,1 m/s vorwärts")
    report = {"complete": False, "stop_verified": False, "ready_for_drive": False,
              "requested_speed_mps": speed_mps, "requested_duration_s": duration_s,
              "errors": [], "before": None, "after": None, "samples": []}
    started_s = None
    try:
        link.wait_status(timeout_s=3)
        deadline_s = time.monotonic() + 2.5
        while link.info is None and time.monotonic() < deadline_s:
            time.sleep(.02)
        if link.info is None or not supported_protocol(link.info.proto_version):
            raise RuntimeError("Keine unterstützte INFO/Protokollversion")
        report["info"] = asdict(link.info)
        before = link.wait_status(timeout_s=3)
        report["before"] = asdict(before)
        if (before.state != "STANDBY" or before.flags
                or max(abs(before.v_l_mm_s), abs(before.v_r_mm_s)) > 5):
            raise RuntimeError("Erwartet: stehende Räder, STANDBY ohne Fehlerflags")
        arm_seq = link.arm()
        link.wait_status(lambda st: st.state == "RUN" and st.last_seq == arm_seq, timeout_s=.15)
        started_s = time.monotonic()
        while time.monotonic() - started_s < duration_s:
            status = link.wait_status(timeout_s=.1)
            report["samples"].append(asdict(status))
            if status.state != "RUN" or status.flags:
                raise RuntimeError(f"Versuch abgebrochen: {status.state}, Flags {status.flags:#x}")
            if min(status.v_l_mm_s, status.v_r_mm_s) < -15:
                raise RuntimeError("Versuch abgebrochen: gemessene Drehrichtung widerspricht Vorwärtsbefehl")
            link.send_cmd(speed_mps, 0)
            remaining_s = duration_s - (time.monotonic() - started_s)
            if remaining_s > 0:
                time.sleep(min(.05, remaining_s))
    except (OSError, RuntimeError, ValueError, TimeoutError, KeyboardInterrupt) as exc:
        report["errors"].append(str(exc) or "Versuch unterbrochen")
    finally:
        # Ausdrücklicher STOP statt weiterlaufender Null-Sollwert-Regelung.
        if started_s is not None:
            report["pulse_elapsed_s"] = time.monotonic() - started_s
        try:
            stop_seq = link.stop()
            after = link.wait_status(lambda st: st.last_seq == stop_seq and st.state != "RUN"
                                    and max(abs(st.v_l_mm_s), abs(st.v_r_mm_s)) <= 5, timeout_s=1)
            report["after"] = asdict(after)
            report["stop_verified"] = True
        except (OSError, RuntimeError, ValueError, TimeoutError) as exc:
            report["errors"].append(f"STOP nicht bestätigt: {exc}")
    report["complete"] = started_s is not None and not report["errors"] and report["stop_verified"]
    return report


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--ausgabe", required=True, type=Path)
    args = ap.parse_args(argv)
    # Vor der Geräteöffnung sicherstellen, dass die Messdatei neu angelegt werden kann.
    with args.ausgabe.open("x", encoding="utf-8") as file:
        json.dump({"complete": False, "phase": "prepared", "ready_for_drive": False}, file)
    events = []
    try:
        with (open_sim() if args.sim else open_serial(args.port)) as link:
            link.on_event = events.append
            report = pulse(link)
    except (OSError, RuntimeError, ValueError, TimeoutError) as exc:
        report = {"complete": False, "stop_verified": False, "ready_for_drive": False, "errors": [str(exc)]}
    report.update(simulated=args.sim, events=events)
    args.ausgabe.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "events"}, indent=2, ensure_ascii=False))
    return int(not report["complete"])


if __name__ == "__main__":
    raise SystemExit(main())
