"""Optionale, rein beobachtende POSIX-Hostprobe und kontrollierte GIL-Gegenprobe.

Der Kindprozess startet isoliert, importiert weder NN noch Fahrzeugsoftware und
teilt keinen GIL mit der Anwendung. Auch er unterliegt Scheduling und Datei-I/O.
Gleichzeitige Verzögerungen beweisen deshalb keinen bestimmten Kernel-Verursacher.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import select
import subprocess
import sys
import time
from pathlib import Path


class ProcessProbe:
    def __init__(self, path):
        self.path = Path(path)
        self.process = None

    def __enter__(self):
        if os.name != "posix":
            raise RuntimeError("Prozessprobe unterstützt nur POSIX-Hosts")
        if self.path.exists() or self.path.is_symlink():
            raise FileExistsError(self.path)
        launched_s = time.monotonic()
        self.process = subprocess.Popen(
            [sys.executable, "-I", str(Path(__file__).resolve()), "--child", str(self.path.resolve())],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        try:
            if not select.select([self.process.stdout], [], [], 5)[0]:
                raise TimeoutError("Prozessprobe meldet sich nicht startbereit")
            ready = json.loads(self.process.stdout.readline())
            if ready.get("ready") is not True or not launched_s <= ready["time_s"] <= time.monotonic():
                raise RuntimeError("Prozessprobe hat keine vergleichbare monotone Uhr")
            return self
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.process is None:
            return
        proc, self.process = self.process, None
        proc.stdin.close()  # EOF beendet nur unseren Kindprozess geordnet
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            raise TimeoutError("Prozessprobe musste beendet werden; Diagnose unvollständig") from None
        finally:
            proc.stdout.close()
        if proc.returncode:
            raise RuntimeError(f"Prozessprobe fehlgeschlagen (Exitcode {proc.returncode})")

    def __exit__(self, *_):
        self.close()


def child_probe(path, period_s=.02):
    """Nur Standalone-Kindprozess: EOF auf stdin ist der einzige Steuerbefehl."""
    with Path(path).open("x", encoding="utf-8") as file:
        def write(event):
            file.write(json.dumps(event, allow_nan=False) + "\n")
            file.flush()

        write({"kind": "probe", "format": 1, "period_s": period_s, "time_s": time.monotonic(),
               "python": platform.python_version(), "pid": os.getpid(), "isolated": bool(sys.flags.isolated)})
        ready = False
        while True:
            planned_s = time.monotonic() + period_s
            closing = False
            if select.select([sys.stdin], [], [], max(0, planned_s - time.monotonic()))[0]:
                if not sys.stdin.buffer.read(1):
                    closing = True
                else:
                    raise ValueError("Prozessprobe erwartet nur EOF, keine Befehle")
            stamp_s = time.monotonic()
            # EOF kann vor dem nächsten Solltakt kommen oder erst nach einer Pause.
            # Abschließende Beobachtung immer sichern; keine negative Verspätung.
            write({"kind": "tick", "planned_s": min(planned_s, stamp_s), "time_s": stamp_s,
                   "closing": closing,
                   "process_cpu_s": time.process_time()})
            if closing:
                break
            if not ready:
                print(json.dumps({"ready": True, "time_s": stamp_s}), flush=True)
                ready = True
        write({"kind": "end", "time_s": time.monotonic()})


def compare_process(path, windows, *, start_s, end_s):
    """Streaming-Auswertung, keine automatische Gleichsetzung von Prozesspause und GIL."""
    if not math.isfinite(start_s) or not math.isfinite(end_s) or not 0 <= start_s < end_s or len(windows) > 5:
        raise ValueError("Ungültiges Profilfenster oder mehr als fünf Vergleichsfenster")
    if any(not all(math.isfinite(w[k]) for k in ("started_s", "time_s"))
           or not start_s <= w["started_s"] < w["time_s"] <= end_s for w in windows):
        raise ValueError("Vergleichsfenster liegt nicht vollständig im Profilfenster")
    rows = [{"started_s": w["started_s"], "time_s": w["time_s"], "samples": 0,
             "max_process_lateness_s": 0.0, "max_heartbeat_gap_s": 0.0} for w in windows]
    count, max_late, max_gap = 0, 0.0, 0.0
    previous_s = first_s = ended_s = None
    with Path(path).open() as file:
        header = json.loads(next(file))
        if header.get("kind") != "probe" or header.get("format") != 1 or header.get("isolated") is not True:
            raise ValueError("Keine isolierte Prozessprobe im Format 1")
        for line in file:
            event = json.loads(line)
            stamp = event["time_s"]
            if ended_s is not None or not math.isfinite(stamp) or (previous_s is not None and stamp < previous_s):
                raise ValueError("Ungültige Reihenfolge der Prozessprobe")
            if event["kind"] == "end":
                ended_s = stamp
                continue
            planned = event["planned_s"]
            if event["kind"] != "tick" or not math.isfinite(planned) or planned > stamp:
                raise ValueError("Ungültige Prozessprobe")
            if first_s is None:
                first_s = stamp
            lateness = stamp - planned
            gap = stamp - previous_s if previous_s is not None else 0
            if start_s <= stamp <= end_s:
                count += 1
            # Ganze beobachtete Intervalle: auch eine Pause über das Profilende zählt.
            if previous_s is not None and previous_s < end_s and stamp > start_s:
                max_late, max_gap = max(max_late, lateness), max(max_gap, gap)
            for row in rows:
                if previous_s is not None and previous_s < row["time_s"] and stamp > row["started_s"]:
                    row["samples"] += 1
                    row["max_process_lateness_s"] = max(row["max_process_lateness_s"], lateness)
                    row["max_heartbeat_gap_s"] = max(row["max_heartbeat_gap_s"], gap)
            previous_s = stamp
    for row in rows:
        row["covered"] = first_s is not None and first_s <= row["started_s"] and previous_s >= row["time_s"]
    return {"scope": "Unabhängiger Python-Prozess, inklusive eigenem Scheduling/I/O; keine GIL-Messung",
            "complete": first_s is not None and first_s <= start_s and ended_s is not None and ended_s >= end_s
                        and previous_s >= end_s and count > 0 and all(row["covered"] for row in rows),
            "samples": count, "max_lateness_s": max_late, "max_heartbeat_gap_s": max_gap,
            "windows": rows, "causality_proven": False, "hardware_verified": False}


def control_case(directory, *, hold_gil):
    """200 ms C-Schlaf mit/ohne GIL-Halten; ausschließlich dieser Diagnoseprozess."""
    import ctypes

    from .recording import FlightRecorder
    from .soak_telemetry import HostProbe

    library = ctypes.PyDLL(None) if hold_gil else ctypes.CDLL(None)
    sleep_us = library.usleep
    sleep_us.argtypes, sleep_us.restype = [ctypes.c_uint], ctypes.c_int
    with (FlightRecorder(directory, {"synthetic": True, "sim": True, "hold_gil": hold_gil}) as recorder,
          ProcessProbe(directory / "process.jsonl"), HostProbe(recorder) as probe):
        start_s = time.monotonic()
        time.sleep(.25)
        pause_start_s = time.monotonic()
        if sleep_us(200000) != 0:
            raise OSError("Kontrollierter C-Schlaf unterbrochen")
        pause_end_s = time.monotonic()
        time.sleep(.25)
        end_s = time.monotonic()
    return {"hold_gil": hold_gil, "synthetic": True, "no_device_commands": True,
            "injected_s": pause_end_s - pause_start_s, "application_probe": probe.summary(),
            "independent_process": compare_process(directory / "process.jsonl",
                                                   [{"started_s": pause_start_s, "time_s": pause_end_s}],
                                                   start_s=start_s, end_s=end_s)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--child", type=Path, help=argparse.SUPPRESS)
    ap.add_argument("--ausgabe", type=Path)
    args = ap.parse_args(argv)
    if args.child is not None:
        child_probe(args.child)
        return 0
    if args.ausgabe is None or os.name != "posix" or sys.implementation.name != "cpython":
        ap.error("Neuer Ausgabeordner und CPython auf POSIX erforderlich")
    args.ausgabe.mkdir(parents=True, exist_ok=False)
    cases = [control_case(args.ausgabe / name, hold_gil=hold)
             for name, hold in (("gil_frei", False), ("gil_gehalten", True))]
    result = {"format": 1, "scope": "Künstliche Kontrollfälle, kein NN-/Langlauf-/Hardware-Nachweis",
              "hardware_verified": False, "causality_proven_for_application": False, "cases": cases}
    with (args.ausgabe / "report.json").open("x") as file:
        json.dump(result, file, indent=2, allow_nan=False)
        file.write("\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
