"""Wiederholbare Fehlerketten ausschließlich am Firmware-Host-Simulator.

Keine Schnittstelle zu echter Hardware. Die Motorbefehle testen Firmwarelogik
mit angenommenem Radmodell, keine realen Bremswege oder Laufzeitgarantien.
"""

from __future__ import annotations

import argparse
import json
import random
import subprocess
import time
from dataclasses import asdict
from pathlib import Path

from . import protocol
from .link import SetpointSender, open_sim


def observe(link, predicate, label, observations, timeout_s=2.0):
    previous = link.status
    baseline_ms = previous.t_ms if previous else -1
    # Keine alte passende Meldung als Beweis für einen neuen Prüfschritt nehmen.
    status = link.wait_status(lambda s: s.t_ms > baseline_ms and link.status_age_s < .3 and predicate(s),
                              timeout_s=timeout_s)
    observations.append({"phase": label, **asdict(status)})
    return status


def hold(link, v_mps, duration_s):
    until_s = time.monotonic() + duration_s
    while time.monotonic() < until_s:
        link.send_cmd(v_mps, 0)
        time.sleep(.04)


def run_case(name, observations):
    with open_sim() as link:
        observe(link, lambda s: s.state == "STANDBY", "start", observations)
        link.send_sim("TOF", 3000)
        link.arm()
        hold(link, .5, .4)
        observe(link, lambda s: s.state == "RUN" and s.v_l_mm_s > 100 and s.v_r_mm_s > 100,
                "fahrt", observations)
        if name == "bildalter_sensor_notaus":
            sender = SetpointSender(link, max_age_s=.15).start()
            try:
                sender.set(.5, 0)
                # Keine neue Beobachtung: der unabhängige Sender läuft weiter.
                observe(link, lambda s: s.state == "RUN" and abs(s.v_l_mm_s) < 30
                        and abs(s.v_r_mm_s) < 30, "beobachtung_veraltet", observations)
                link.send_sim("TOFFAIL", 1)
                observe(link, lambda s: s.flags & protocol.FLAG_TOF_FAIL, "sensor_ausfall", observations)
                link.send_sim("ESTOP", 1)
                observe(link, lambda s: s.state == "ESTOP", "notaus", observations)
                sender.set(.5, 0)
                link.arm()
                observe(link, lambda s: s.state == "ESTOP" and s.flags & protocol.FLAG_ARM_REJECTED,
                        "arm_abgelehnt", observations)
                link.send_sim("TOFFAIL", 0)
                link.send_sim("ESTOP", 0)
                observe(link, lambda s: s.state == "STANDBY", "entriegelt", observations)
                sender.set(.5, 0)
                time.sleep(.12)
                observe(link, lambda s: s.state == "STANDBY" and abs(s.v_l_mm_s) < 30,
                        "kein_wiederanlauf", observations)
            finally:
                sender.stop()
        elif name == "muelldaten_watchdog":
            # Verfälschte, unvollständige und überlange Frames sind keine CMD.
            errors = link.status.rx_errors
            with link._write_lock:
                link._writer.write(b"$CMD,500,500,0*00\n" + b"?" * 400 + b"\n$CMD,1\n")
                link._writer.flush()
            observe(link, lambda s: s.state == "STANDBY" and s.flags & protocol.FLAG_WDG
                    and s.rx_errors > errors, "muelldaten_stoppen_watchdog_nicht", observations)
            hold(link, .5, .2)
            observe(link, lambda s: s.state == "STANDBY", "gueltiges_cmd_ohne_arm", observations)
        elif name == "stop_arm_folge":
            for index in range(6):
                link.stop()
                observe(link, lambda s: s.state == "STANDBY", f"stop_{index}", observations)
                hold(link, .5, .08)
                observe(link, lambda s: s.state == "STANDBY", f"cmd_gesperrt_{index}", observations)
                link.arm()
                hold(link, 0, .08)
                observe(link, lambda s: s.state == "RUN", f"arm_{index}", observations)
        else:
            raise ValueError(f"Unbekannter Testfall: {name}")
        link.stop()


def run_campaign(cycles=2, seed=17):
    if not 1 <= cycles <= 1000:
        raise ValueError("Zyklen müssen zwischen 1 und 1000 liegen")
    rng = random.Random(seed)
    started_s = time.monotonic()
    cases = []
    for cycle in range(cycles):
        names = ["bildalter_sensor_notaus", "muelldaten_watchdog", "stop_arm_folge"]
        rng.shuffle(names)
        for name in names:
            result = {"cycle": cycle + 1, "name": name, "status": "ok", "observations": []}
            cases.append(result)
            try:
                run_case(name, result["observations"])
            except (OSError, RuntimeError, ValueError, TimeoutError, subprocess.SubprocessError) as exc:
                result.update(status="error", error=str(exc))
    # Jeder Fall startet einen neuen Firmwareprozess: auch nach vorheriger
    # Fahrt darf ein Pico-Neustart nie RUN übernehmen.
    return {"format": 1, "simulated": True, "ready_for_drive": False, "seed": seed,
            "cycles": cycles, "duration_s": time.monotonic() - started_s,
            "ok": all(case["status"] == "ok" for case in cases), "cases": cases}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--zyklen", type=int, default=2)
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--ausgabe", type=Path)
    args = ap.parse_args(argv)
    if not 1 <= args.zyklen <= 1000:
        ap.error("Zyklen müssen zwischen 1 und 1000 liegen")
    # Vor langen Prüfungen das Ziel exklusiv reservieren, nie überschreiben.
    if args.ausgabe:
        with args.ausgabe.open("x", encoding="utf-8") as file:
            report = run_campaign(args.zyklen, args.seed)
            file.write(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    else:
        report = run_campaign(args.zyklen, args.seed)
    if args.ausgabe:
        print(f"{len(report['cases'])} Fälle · {report['duration_s']:.1f} s · "
              f"{'bestanden' if report['ok'] else 'FEHLER'} · {args.ausgabe}")
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return int(not report["ok"])


if __name__ == "__main__":
    raise SystemExit(main())
