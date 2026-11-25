"""Begrenzte Host-Diagnose: Überlappungen sind Hinweise, keine Ursachenbeweise.

GC-Hooks greifen nie auf Recorder-Locks zu (GC kann innerhalb dieser Locks
beginnen). Nur der zusätzliche Diagnose-Thread leert den begrenzten Puffer.
Die Instrumentierung verändert selbst Last/Timing; keine Echtzeitzusage.
"""

from __future__ import annotations

import gc
import heapq
import json
import threading
import time
from collections import deque
from pathlib import Path

from .protocol import unframe


class HostProbe:
    PERIOD_S = .02
    REPORT_LATENESS_S = .01

    def __init__(self, recorder):
        self.recorder = recorder
        self._pending = deque(maxlen=256)
        self._gc_started = {}
        self.gc_dropped = 0
        self.samples = 0
        self.max_lateness_s = 0.0
        self._stop = threading.Event()
        self._thread = None
        self._callback = self._gc_event

    def _gc_event(self, phase, info):
        stamp = time.monotonic()
        generation = info["generation"]
        if phase == "start":
            self._gc_started[generation] = stamp
        elif phase == "stop":
            start = self._gc_started.pop(generation, None)
            if start is not None:
                if len(self._pending) == self._pending.maxlen:
                    self.gc_dropped += 1
                self._pending.append((start, stamp, generation))

    def _drain(self):
        while self._pending:
            start, end, generation = self._pending.popleft()
            self.recorder.record({"kind": "gc_pause", "started_s": start,
                                  "time_s": end, "generation": generation})

    def _loop(self):
        previous_wall_s = time.monotonic()
        previous_cpu_s = time.process_time()
        planned_s = time.monotonic() + self.PERIOD_S
        while not self._stop.wait(max(0, planned_s - time.monotonic())):
            now_s = time.monotonic()
            cpu_s = time.process_time()
            lateness_s = max(0, now_s - planned_s)
            self.samples += 1
            self.max_lateness_s = max(self.max_lateness_s, lateness_s)
            if lateness_s >= self.REPORT_LATENESS_S:
                self.recorder.record({"kind": "thread_late", "planned_s": planned_s, "time_s": now_s,
                                      "previous_sample_s": previous_wall_s,
                                      "process_cpu_delta_s": cpu_s - previous_cpu_s})
            self._drain()
            previous_wall_s, previous_cpu_s = now_s, cpu_s
            planned_s = time.monotonic() + self.PERIOD_S  # keine Diagnose-Nachholbursts
        self._drain()

    def __enter__(self):
        gc.callbacks.append(self._callback)
        self._thread = threading.Thread(target=self._loop, name="soak-diagnose", daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *_):
        gc.callbacks.remove(self._callback)  # fremde Hooks unverändert erhalten
        self._stop.set()
        self._thread.join(2)
        if self._thread.is_alive():
            raise TimeoutError("Diagnose-Thread beendet sich nicht")

    def summary(self):
        return {"period_s": self.PERIOD_S, "report_lateness_s": self.REPORT_LATENESS_S,
                "samples": self.samples, "max_lateness_s": self.max_lateness_s,
                "gc_buffer_dropped": self.gc_dropped,
                "observer_effect": True, "causality_proven": False}


def diagnose_timing(path, *, start_s, end_s, gc_buffer_dropped=0):
    """Zwei Streaming-Durchgänge, höchstens fünf CMD-Fenster mit je fünf Hinweisen.

    Kein Resampling, keine Grenzwertanhebung und keine nachträgliche Fensterwahl.
    Schreibspanne enthält write/flush plus mögliche Unterbrechungen, aber weder
    vorherige Lock-Wartezeit noch den nachfolgenden Recorder-Callback.
    """
    if not 0 <= start_s < end_s:
        raise ValueError("Ungültiges Diagnosefenster")
    worst = []
    last_s = None
    gaps_over_budget = 0
    dropped = None
    counts = {"sender_tick": 0, "gc_pause": 0, "thread_late": 0, "writes": 0}
    maxima = {"sender_lateness_s": 0.0, "send_span_s": 0.0, "write_span_s": 0.0,
              "gc_span_s": 0.0, "thread_lateness_s": 0.0}

    def events():
        with Path(path).open() as file:
            for line in file:
                yield json.loads(line)

    for event in events():
        kind = event["kind"]
        if kind == "end":
            dropped = event["dropped"]
        stamp = event.get("time_s", -1)
        if not start_s <= stamp <= end_s:
            continue
        if kind in counts:
            counts[kind] += 1
        if kind == "sender_tick":
            maxima["sender_lateness_s"] = max(maxima["sender_lateness_s"], event["started_s"] - event["planned_s"])
            maxima["send_span_s"] = max(maxima["send_span_s"], stamp - event["started_s"])
        elif kind == "gc_pause":
            maxima["gc_span_s"] = max(maxima["gc_span_s"], stamp - event["started_s"])
        elif kind == "thread_late":
            maxima["thread_lateness_s"] = max(maxima["thread_lateness_s"], stamp - event["planned_s"])
        elif kind == "tx" and (fields := unframe(event["line"])) and fields[0] == "CMD":
            if event.get("write_started_s") is not None:
                counts["writes"] += 1
                maxima["write_span_s"] = max(maxima["write_span_s"], stamp - event["write_started_s"])
            if last_s is not None:
                gap = stamp - last_s
                gaps_over_budget += gap > .15
                heapq.heappush(worst, (gap, last_s, stamp))
                if len(worst) > 5:
                    heapq.heappop(worst)
            last_s = stamp
    windows = [{"gap_s": gap, "started_s": start, "time_s": end, "overlap": {}}
               for gap, start, end in sorted(worst, reverse=True)]
    for event in events():
        kind = event["kind"]
        intervals = []
        if kind == "sender_tick":
            intervals = [("sender_lateness", event["planned_s"], event["started_s"]),
                         ("send_span", event["started_s"], event["time_s"])]
        elif kind == "tx" and event.get("write_started_s") is not None:
            intervals = [("write_span", event["write_started_s"], event["time_s"])]
        elif kind in ("gc_pause", "thread_late"):
            intervals = [(kind, event.get("started_s", event.get("planned_s")), event["time_s"])]
        for label, start, end in intervals:
            if not start_s <= end <= end_s:
                continue
            for window in windows:
                if start < window["time_s"] and end > window["started_s"]:
                    old = window["overlap"].get(label)
                    if old is None or end - start > old["duration_s"]:
                        window["overlap"][label] = {"started_s": start, "time_s": end, "duration_s": end - start}
                        if kind == "thread_late" and "process_cpu_delta_s" in event:
                            window["overlap"][label].update(
                                process_cpu_delta_s=event["process_cpu_delta_s"],
                                cpu_interval_started_s=event["previous_sample_s"])
    return {"scope": "Host-Zeitspannen, keine Kernel-/USB-/GIL-Ursachenzuordnung",
            "cmd_gap_budget_s": .15, "gaps_over_budget": gaps_over_budget,
            "recorder_dropped": dropped, "gc_buffer_dropped": gc_buffer_dropped,
            "complete": dropped == 0 and gc_buffer_dropped == 0 and counts["sender_tick"] > 0
                        and counts["writes"] == counts["sender_tick"] and last_s is not None,
            "causality_proven": False, "counts": counts, "maxima": maxima, "worst_cmd_windows": windows}
