"""Direkte Host-Simulatorzeit gegen Python-TX, kein USB-/Pico-Zeitnachweis.

Datei-I/O ausschließlich nach dem Versuch. Zuordnung nach Reihenfolge UND
CMD-Inhalt, damit Folgenummernüberlauf und verzögerte TX-Callbacks klar bleiben.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from .protocol import unframe

CAPACITY = 32768


def records(path):
    path = Path(path)
    if path.stat().st_size > 256 * 1024 * 1024:
        raise ValueError("Diagnosedatei zu groß")
    with path.open() as file:
        while line := file.readline(16385):
            if len(line) > 16384:
                raise ValueError("Diagnosezeile zu groß")
            row = json.loads(line)
            if not isinstance(row, dict):
                raise TypeError("Diagnoseeintrag ist kein Objekt")
            yield row


def integer(row, key, lower=0, upper=2**64 - 1):
    value = row.get(key)
    if type(value) is not int or not lower <= value <= upper:
        raise ValueError(f"Ungültiger Diagnosewert: {key}")
    return value


def read_trace(path):
    source = iter(records(path))
    header = next(source, {})
    if (header.get("kind") != "sim_trace" or header.get("format") != 1
            or header.get("clock") not in ("CLOCK_MONOTONIC", "mach_absolute_time")):
        raise ValueError("Keine Host-Simulatortrace im Format 1")
    started = integer(header, "started_ns", 1)
    capacity = integer(header, "capacity", 1, CAPACITY)
    commands, footer = [], None
    previous = started
    for row in source:
        if footer is not None:
            raise ValueError("Daten nach Trace-Ende")
        if row.get("kind") == "end":
            ended = integer(row, "ended_ns", previous)
            written = integer(row, "written", 0, capacity)
            dropped = integer(row, "dropped")
            received = integer(row, "received")
            if written != len(commands) or received != written + dropped or ended < started:
                raise ValueError("Widersprüchlicher Trace-Endzähler")
            footer = row
        elif row.get("kind") == "cmd":
            if len(commands) >= capacity:
                raise ValueError("Trace-Kapazität überschritten")
            read_ns = integer(row, "read_ns", started)
            previous = integer(row, "accepted_ns", max(previous, read_ns))
            integer(row, "firmware_ms", 0, 2**32 - 1)
            integer(row, "seq", 0, 65535)
            integer(row, "v_mm_s", -100000, 100000)
            integer(row, "omega_mrad_s", -100000, 100000)
            commands.append(row)
        else:
            raise ValueError("Unbekannter Trace-Eintrag")
    return header, commands, footer


def compare_trace(trace_path, events_path):
    header, commands, footer = read_trace(trace_path)
    tx, markers, recorded_end = [], {}, None
    for event in records(events_path):
        if recorded_end is not None:
            raise ValueError("Daten nach Recorder-Ende")
        if event.get("kind") == "control" and event.get("key") in ("load_start", "load_end"):
            key = event["key"]
            if key in markers:
                raise ValueError("Doppelter Lastmarker")
            markers[key] = event["time_s"]
        elif event.get("kind") == "end":
            if recorded_end is not None:
                raise ValueError("Doppeltes Recorder-Ende")
            recorded_end = event
        elif event.get("kind") == "tx":
            fields = unframe(event["line"])
            if fields and fields[0] == "CMD":
                if len(fields) != 4:
                    raise ValueError("Ungültige Python-CMD-Zeile")
                if len(tx) >= CAPACITY:
                    raise ValueError("Zu viele Python-CMD für begrenzte Trace")
                stamp, started = event["time_s"], event.get("write_started_s")
                if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
                        or not isinstance(started, (int, float)) or not math.isfinite(started) or started > stamp):
                    raise ValueError("Python-TX-Schreibzeit fehlt oder ist ungültig")
                if tx and started < tx[-1]["time_s"]:
                    raise ValueError("Python-TX-Zeit läuft rückwärts")
                tx.append({"seq": int(fields[1]), "v_mm_s": int(fields[2]), "omega_mrad_s": int(fields[3]),
                           "time_s": stamp, "write_started_s": started})
    reasons = []
    if footer is None:
        reasons.append("trace_end_missing")
    elif footer["dropped"]:
        reasons.append("trace_overflow")
    if recorded_end is None or recorded_end.get("dropped") != 0:
        reasons.append("recorder_incomplete")
    if len(tx) != len(commands) or not commands:
        reasons.append("command_count_mismatch_or_empty")
    matched = len(tx) == len(commands) and bool(commands) and all(
        all(a[key] == b[key] for key in ("seq", "v_mm_s", "omega_mrad_s")) for a, b in zip(tx, commands))
    if not matched:
        reasons.append("command_order_or_content_unverified")
    start, end = markers.get("load_start"), markers.get("load_end")
    if (not isinstance(start, (int, float)) or not isinstance(end, (int, float))
            or not math.isfinite(start) or not math.isfinite(end) or not 0 <= start < end):
        reasons.append("load_window_missing_or_invalid")
    parent_end_s = recorded_end.get("time_s") if recorded_end else None
    clock_verified = (matched and footer is not None and isinstance(parent_end_s, (int, float))
                      and math.isfinite(parent_end_s)
                      and header["started_ns"] / 1e9 <= tx[0]["write_started_s"] + .000001
                      and footer["ended_ns"] / 1e9 <= parent_end_s + .000001 and all(
        # 1 µs nur für Float-/Nanosekundenrundung, kein Timingbudget.
        a["accepted_ns"] / 1e9 >= b["write_started_s"] - .000001
        and a["read_ns"] / 1e9 >= b["write_started_s"] - .000001
        for a, b in zip(commands, tx)))
    if not clock_verified:
        reasons.append("shared_clock_or_causal_order_unverified")
    if ("load_window_missing_or_invalid" not in reasons and footer is not None
            and not header["started_ns"] / 1e9 <= start < end <= footer["ended_ns"] / 1e9):
        reasons.append("trace_lifetime_does_not_cover_load_window")
    pairs = []
    if matched and clock_verified and "load_window_missing_or_invalid" not in reasons:
        for index in range(1, len(commands)):
            before, after = commands[index - 1], commands[index]
            # Ganze überlappende Intervalle; keine Pause am Fensterende wegschneiden.
            if before["accepted_ns"] / 1e9 < end and after["accepted_ns"] / 1e9 > start:
                pairs.append({"ordinal": index + 1, "seq": after["seq"],
                              "started_s": before["accepted_ns"] / 1e9, "time_s": after["accepted_ns"] / 1e9,
                              "native_gap_s": (after["accepted_ns"] - before["accepted_ns"]) / 1e9,
                              "python_tx_gap_s": tx[index]["time_s"] - tx[index - 1]["time_s"],
                              "python_callback_after_accept_s": tx[index]["time_s"] - after["accepted_ns"] / 1e9,
                              "write_start_to_accept_s": after["accepted_ns"] / 1e9 - tx[index]["write_started_s"],
                              "read_to_accept_s": (after["accepted_ns"] - after["read_ns"]) / 1e9})
    if not pairs:
        reasons.append("no_complete_cmd_interval_in_load_window")
    return {"scope": "Anwendungsannahme im nativen Host-Simulator nach Parser/Supervisor, nicht USB/Pico",
            "complete": not reasons, "incomplete_reasons": reasons, "commands": len(commands),
            "python_commands": len(tx), "trace_dropped": footer["dropped"] if footer else None,
            "shared_clock_causal_order_verified": clock_verified, "cmd_gap_budget_s": .15,
            "max_native_gap_s": max((p["native_gap_s"] for p in pairs), default=None),
            "max_python_tx_gap_s": max((p["python_tx_gap_s"] for p in pairs), default=None),
            "native_gaps_over_budget": sum(p["native_gap_s"] > .15 for p in pairs),
            "native_gap_budget_passed": None if reasons else all(p["native_gap_s"] <= .15 for p in pairs),
            "max_callback_after_accept_s": max((p["python_callback_after_accept_s"] for p in pairs), default=None),
            "max_write_start_to_accept_s": max((p["write_start_to_accept_s"] for p in pairs), default=None),
            "worst_native_windows": sorted(pairs, key=lambda p: p["native_gap_s"], reverse=True)[:5],
            "started_ns": header["started_ns"], "observer_effect": True,
            "causality_proven": False, "hardware_verified": False}
