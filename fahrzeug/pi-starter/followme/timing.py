"""Laufzeitbudgets eines Fahrtenschreibers auswerten, ausschließlich offline.

Keine Kamera-/NN-/Pico-Imports, keine Fahrfreigabe. Die Grenzen sind
Auswerteannahmen, nicht konfigurierbare Sicherheitsgrenzen des Fahrzeugs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from collections import Counter
from dataclasses import asdict, dataclass
from itertools import pairwise
from pathlib import Path

from . import protocol


@dataclass(frozen=True)
class TimingLimits:
    # ANNAHMEN: am Prototyp anhand längerer Aufzeichnungen überprüfen.
    max_frame_age_s: float = .6
    max_frame_gap_s: float = .6
    max_cmd_gap_s: float = .15
    max_status_gap_s: float = .5
    min_samples: int = 3
    min_duration_s: float = 5.0

    def __post_init__(self):
        for name, value in asdict(self).items():
            if name == "min_samples":
                if type(value) is not int or value < 2:
                    raise ValueError("min_samples muss eine ganze Zahl ≥ 2 sein")
            elif _number(value) <= 0:
                raise ValueError(f"{name} muss endlich und positiv sein")


def _number(value) -> float:
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and value >= 0
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError("Zeitstempel muss endlich, numerisch und nichtnegativ sein")
    return float(value)


DEFAULT_LIMITS = TimingLimits()


def _count(value) -> int:
    if type(value) is not int or value < 0:
        raise ValueError("Zähler muss eine nichtnegative ganze Zahl sein")
    return value


def _over(value, limit):
    return value > limit + 1e-9  # Rundungsreste an der sekundengenauen Grenze.


def _distribution(values_s):
    values = sorted(value * 1000 for value in values_s)
    return {"samples": len(values), "median_ms": statistics.median(values) if values else None,
            "p95_ms": values[math.ceil(.95 * len(values)) - 1] if values else None,
            "max_ms": values[-1] if values else None}


def _cadence(times, start_s, end_s, limit_s):
    deltas = [right - left for left, right in pairwise(times)]
    tail_s = max(0, end_s - max(times)) if times else None
    lead_s = max(0, times[0] - start_s) if times else None
    span_s = times[-1] - times[0] if len(times) >= 2 else 0
    return {"events": len(times), "span_s": max(0, span_s),
            "gaps": _distribution([d for d in deltas if d >= 0]),
            "tail_ms": tail_s * 1000 if tail_s is not None else None,
            "lead_ms": lead_s * 1000 if lead_s is not None else None,
            "gaps_over_limit": sum(_over(d, limit_s) for d in deltas),
            "tail_over_limit": tail_s is not None and _over(tail_s, limit_s),
            "lead_over_limit": lead_s is not None and _over(lead_s, limit_s),
            "time_regressions": sum(d < 0 for d in deltas),
            "observed_hz": (len(times) - 1) / span_s if span_s > 0 and all(d >= 0 for d in deltas) else None}


def _wire(event):
    line = event.get("line")
    if not isinstance(line, str) or not line.isascii() or len(line) > 512:
        return None
    return protocol.unframe(line)


def analyze_events(events, limits: TimingLimits = DEFAULT_LIMITS) -> dict:
    """Analysiert Format 1; Dateireihenfolge darf zwischen Threads abweichen.

    Die Reihenfolge innerhalb jeder Quelle bleibt erhalten. Pico-Zeit wird
    niemals von Pi-Zeit abgezogen; last_seq ist kein Motorwirkungsnachweis.
    """
    events = iter(events)
    header = next(events, None)
    if not isinstance(header, dict) or header.get("kind") != "run" or type(header.get("format")) is not int:
        raise ValueError("Erstes Ereignis muss run im Format 1 sein")
    if header["format"] != 1:
        raise ValueError("Unbekanntes Protokollformat")
    for key in ("sim", "synthetic"):
        if key in header and type(header[key]) is not bool:
            raise ValueError(f"{key} muss boolesch sein")
    synthetic = header.get("synthetic", False)
    source_kind = ("synthetic" if synthetic else "simulation" if header.get("sim") is True
                   else "hardware_log_unverified" if header.get("sim") is False else "unknown")
    captures, finished, cmd, status_times, ages, event_times = [], [], [], [], [], []
    sources, discarded, flags, tx_types = Counter(), Counter(), Counter(), Counter()
    bad_packets = status_duplicates = status_regressions = source_errors = frame_clock_errors = 0
    previous_status = None
    ended, dropped, end_s = False, 0, None
    for event in events:
        if ended:
            raise ValueError("Daten nach Protokollende")
        if not isinstance(event, dict):
            raise TypeError("Ungültiges Ereignis (erwartet: JSON-Objekt)")
        kind = event.get("kind")
        if kind == "frame":
            captured_s, finished_s = _number(event.get("captured_s")), _number(event.get("finished_s"))
            if captured_s > finished_s or (captures and captured_s <= captures[-1]):
                frame_clock_errors += 1
            if captured_s <= finished_s:
                ages.append(finished_s - captured_s)
            captures.append(captured_s)
            finished.append(finished_s)
            event_times.append(finished_s)
            source = event.get("timestamp_source", "unknown")
            reason = event.get("discarded")
            if not isinstance(source, str) or (reason is not None and not isinstance(reason, str)):
                raise ValueError("Zeitstempelquelle/Verwerfungsgrund muss Text sein")
            sources[source] += 1
            if reason is not None:
                discarded[reason] += 1
        elif kind in ("tx", "rx"):
            stamp_s = _number(event.get("time_s"))
            event_times.append(stamp_s)
            fields = _wire(event)
            if fields is None:
                bad_packets += 1
                continue
            if kind == "tx":
                try:
                    name = fields[0]
                    if name not in ("CMD", "ARM", "STOP") or len(fields) != (4 if name == "CMD" else 2):
                        raise ValueError("Unbekannte Sendung")
                    seq = int(fields[1])
                    if not 0 <= seq <= 65535 or type(event.get("seq")) is not int or event["seq"] != seq:
                        raise ValueError("Abweichende Folgenummer")
                    if name == "CMD" and any(abs(int(v)) > 100000 for v in fields[2:]):
                        raise ValueError("Ungültiger CMD-Wert")
                except ValueError:
                    bad_packets += 1
                    continue
                tx_types[name] += 1
                if name == "CMD":
                    cmd.append(stamp_s)
            elif fields[0] == "STAT":
                st = protocol.parse_status(fields)
                if st is None or not 0 <= st.t_ms <= 0xFFFFFFFF or st.state not in ("RUN", "STANDBY", "ESTOP"):
                    bad_packets += 1
                    continue
                flags.update(st.flag_names())
                if previous_status is not None:
                    delta_ms = (st.t_ms - previous_status) & 0xFFFFFFFF
                    if delta_ms == 0:
                        status_duplicates += 1
                        continue
                    if delta_ms >= 0x80000000:
                        status_regressions += 1
                        continue
                previous_status = st.t_ms
                status_times.append(stamp_s)
            elif fields[0] == "US":
                # Ab Protokoll 2: Ultraschall links/rechts, nur auf Gültigkeit geprüft (keine Taktprobe).
                if protocol.parse_ultrasonic(fields) is None:
                    bad_packets += 1
            elif fields[0] != "INFO" or protocol.parse_info(fields) is None:
                bad_packets += 1
        elif kind in ("sender_tick", "gc_pause", "thread_late"):
            # Optionale Host-Spannen sind keine CMD/STAT/Frame-Proben.
            stamp_s = _number(event.get("time_s"))
            start_s = _number(event.get("planned_s" if kind == "thread_late" else "started_s"))
            if start_s > stamp_s:
                raise ValueError("Rückwärts laufende Diagnosezeit")
            if kind == "sender_tick" and _number(event.get("planned_s")) > start_s:
                raise ValueError("Sender vor geplantem Termin")
            event_times.append(stamp_s)
        elif kind in ("control", "error", "end"):
            stamp_s = _number(event.get("time_s"))
            event_times.append(stamp_s)
            if kind == "error":
                source_errors += 1
            if kind == "end":
                ended, dropped, end_s = True, _count(event.get("dropped")), stamp_s
        else:
            raise ValueError(f"Unbekanntes Ereignis: {kind!r}")
    if end_s is not None and any(stamp > end_s for stamp in event_times):
        raise ValueError("Zeitstempel nach dem Ende der Aufzeichnung")
    horizon_s = end_s if end_s is not None else max(event_times, default=0)
    start_s = min(event_times, default=horizon_s)
    duration_s = horizon_s - start_s
    streams = {"frame_capture": _cadence(captures, start_s, horizon_s, limits.max_frame_gap_s),
               "frame_finished": _cadence(finished, start_s, horizon_s, limits.max_frame_gap_s),
               "cmd": _cadence(cmd, start_s, horizon_s, limits.max_cmd_gap_s),
               "status": _cadence(status_times, start_s, horizon_s, limits.max_status_gap_s)}
    failures, missing = [], []
    age_over = sum(_over(age, limits.max_frame_age_s) for age in ages)
    if age_over:
        failures.append("frame_age_limit")
    for name, stream in streams.items():
        if stream["gaps_over_limit"] or stream["tail_over_limit"] or stream["lead_over_limit"]:
            failures.append(f"{name}_gap_limit")
        if stream["time_regressions"]:
            failures.append(f"{name}_time_regression")
        if stream["events"] < limits.min_samples:
            missing.append(f"{name}_samples")
        if stream["span_s"] < limits.min_duration_s:
            missing.append(f"{name}_observation_too_short")
    for reason, count in (("invalid_protocol_events", bad_packets), ("frame_clock_errors", frame_clock_errors),
                          ("pico_time_regression", status_regressions), ("source_errors", source_errors),
                          ("discarded_frames", sum(n for reason, n in discarded.items() if reason != "bedienung"))):
        if count:
            failures.append(reason)
    unknown_age = sum(n for source, n in sources.items()
                      if source != "sensor-boottime" and not (synthetic and source == "synthetic"))
    if unknown_age:
        missing.append("camera_buffer_age_unknown")
    if not ended or dropped:
        missing.append("incomplete_recording")
    if duration_s < limits.min_duration_s:
        missing.append("duration_too_short")
    if source_kind == "unknown":
        missing.append("source_kind_unknown")
    status = "limits_exceeded" if failures else "inconclusive" if missing else "within_recorded_limits"
    runtime = header.get("runtime", {})
    if not isinstance(runtime, dict):
        raise TypeError("Ungültige Laufzeitmetadaten (erwartet: JSON-Objekt)")
    return {"format": 1, "status": status, "ready_for_drive": False, "hardware_verified": False,
            "source_kind": source_kind,
            "runtime": {k: v for k, v in runtime.items()
                        if k in ("platform", "python", "git_commit", "model_sha256") and isinstance(v, str)},
            "limits": asdict(limits), "limits_are_assumptions": True,
            "duration_s": duration_s, "complete": ended and dropped == 0, "dropped": dropped,
            "failures": failures, "insufficient_evidence": missing,
            "frame_age": {**_distribution(ages), "over_limit": age_over,
                          "unknown_buffer_age_frames": unknown_age},
            "timestamp_sources": dict(sources), "discarded": dict(discarded), "streams": streams,
            "status_duplicates": status_duplicates, "status_regressions": status_regressions,
            "flag_observations": dict(flags), "tx_types": dict(tx_types),
            "invalid_protocol_events": bad_packets, "frame_clock_errors": frame_clock_errors,
            "source_errors": source_errors, "capture_to_motor_ms": None, "command_to_ack_ms": None}


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Doppelter JSON-Schlüssel: {key}")
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError(f"Nichtendliche JSON-Zahl: {value}")


def analyze(path: Path, limits: TimingLimits = DEFAULT_LIMITS) -> dict:
    """Liest nur events.jsonl (oder die direkt angegebene Datei), nie Bilder."""
    path = Path(path)
    if path.is_dir():
        path = path / "events.jsonl"
    digest = hashlib.sha256()

    def events():
        with path.open("rb") as file:
            for number, raw in enumerate(file, 1):
                digest.update(raw)
                try:
                    yield json.loads(raw, object_pairs_hook=_unique, parse_constant=_invalid_constant)
                except (ValueError, UnicodeError) as exc:
                    raise ValueError(f"Zeile {number}: {exc}") from exc

    report = analyze_events(events(), limits)
    report["source_sha256"] = digest.hexdigest()
    return report


DEMOS = ("normal", "bild_veraltet", "sendepause", "statusverlust")


def synthetic_events(scenario: str = "normal") -> list[dict]:
    """Erfundene sechs Sekunden; erzeugt nur Daten, sendet keine Befehle."""
    if scenario not in DEMOS:
        raise ValueError("Unbekannte Rechenprobe")
    events = []
    for i in range(20):
        captured_s = 100 + i * .3
        events.append({"kind": "frame", "captured_s": captured_s - (.75 if scenario == "bild_veraltet" else 0),
                       "finished_s": captured_s + .08, "timestamp_source": "synthetic", "discarded": None})
    for i in range(120):
        stamp_s = 100 + i * .05
        if scenario != "sendepause" or not 102 <= stamp_s < 102.5:
            events.append({"kind": "tx", "time_s": stamp_s, "seq": i,
                           "line": protocol.cmd_line(i, .4, 0).strip()})
    for i in range(150):
        stamp_s = 100 + i * .04
        if scenario != "statusverlust" or stamp_s < 102:
            events.append({"kind": "rx", "time_s": stamp_s,
                           "line": protocol.frame(f"STAT,{i * 40},RUN,0,0,0,0,400,400,3000,12000,0").strip()})
    events.sort(key=lambda e: e.get("finished_s", e.get("time_s")))
    return [{"kind": "run", "format": 1, "sim": True, "synthetic": True},
            *events, {"kind": "end", "time_s": 106.0, "dropped": 0}]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("protokoll", type=Path, nargs="?", help="Fahrtordner oder events.jsonl")
    ap.add_argument("--demo", choices=DEMOS, help="Erfundene Rechenprobe statt einer Messdatei")
    ap.add_argument("--ausgabe", type=Path, help="Neue JSON-Ausgabedatei, wird nie überschrieben")
    ap.add_argument("--streng", action="store_true", help="Exitcode 1, wenn Grenzen überschritten oder Daten unklar")
    for name in ("frame_age", "frame_gap", "cmd_gap", "status_gap"):
        ap.add_argument("--max-" + name.replace("_", "-"), type=float, help="Auswertefrist in Sekunden")
    args = ap.parse_args(argv)
    if (args.protokoll is None) == (args.demo is None):
        ap.error("Genau ein Protokoll oder --demo angeben")
    try:
        limits = TimingLimits(**{f"max_{name}_s": value for name in ("frame_age", "frame_gap", "cmd_gap", "status_gap")
                                 if (value := getattr(args, f"max_{name}")) is not None})
        report = analyze(args.protokoll, limits) if args.protokoll else analyze_events(synthetic_events(args.demo), limits)
        data = json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
        if args.ausgabe:
            with args.ausgabe.open("x", encoding="utf-8") as file:
                file.write(data)
    except (OSError, ValueError, TypeError) as exc:
        ap.error(str(exc))
    print(data, end="")
    return int(args.streng and report["status"] != "within_recorded_limits")


if __name__ == "__main__":
    raise SystemExit(main())
