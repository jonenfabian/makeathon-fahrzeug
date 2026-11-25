"""Gemeinsamer Software-Dauerlauf, ausschließlich Host-Simulator und Loopback.

Echtes CPU-MobileDet auf wiederholten CC0-Prüfbildern, PerceptionLoop,
20-Hz-Sender, Firmwarekern, MJPEG und Fahrtenschreiber. Kein ARM, keine
serielle Schnittstelle, keine Kamera, keine Aussage über thermische Pi-Last.
"""

from __future__ import annotations

import argparse
import errno
import gc
import json
import math
import platform
import resource
import socket
import subprocess
import threading
import time
import tracemalloc
from contextlib import ExitStack
from dataclasses import asdict
from pathlib import Path

from .coral import CPU_MODEL_NAME, CoralPersonDetector
from .geometry import CameraModel
from .host_profile import ProcessProbe, compare_process
from .link import SetpointSender, open_sim
from .person_quality import case_images, load_annotations, sha256
from .pipeline import PerceptionLoop
from .protocol import unframe
from .recording import FlightRecorder
from .sim_trace import compare_trace
from .soak_telemetry import HostProbe, diagnose_timing
from .stream import FrameStream
from .timing import analyze_events


class FaultRecorder(FlightRecorder):
    """Nur im Testprozess: langsamen Datenträger / ENOSPC gezielt einspeisen."""

    def __init__(self, *args, fault="none", **kwargs):
        self.fault = fault
        self.writes = 0
        self.inject_after_s = math.inf
        super().__init__(*args, **kwargs)

    def _write(self, event):
        if event["kind"] != "run":
            self.writes += 1
            if self.fault == "slow":
                time.sleep(.1)
            if self.fault == "enospc" and time.monotonic() >= self.inject_after_s:
                raise OSError(errno.ENOSPC, "Simulierter voller Datenträger; kein echter Datenträger gefüllt")
        super()._write(event)


class WireStats:
    """Konstanter Speicher auch bei ausgefallenem Fahrtenschreiber."""

    def __init__(self, recorder):
        self.recorder = recorder
        self.commands = self.arms = 0
        self.max_gap_s = 0.0
        self.last_s = None
        self._lock = threading.Lock()

    def __call__(self, event):
        if event["kind"] == "tx":
            fields = unframe(event["line"])
            with self._lock:
                if fields and fields[0] == "ARM":
                    self.arms += 1
                if fields and fields[0] == "CMD":
                    stamp = event["time_s"]
                    if self.last_s is not None:
                        self.max_gap_s = max(self.max_gap_s, stamp - self.last_s)
                    self.last_s = stamp
                    self.commands += 1
        self.recorder.record(event)


def reader_client(port, *, slow=False):
    client = socket.create_connection(("127.0.0.1", port), timeout=1)
    client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 if slow else 65536)
    client.sendall(b"GET /stream HTTP/1.0\r\nHost: localhost\r\n\r\n")
    counts = {"bytes": 0, "error": None}

    def read():
        try:
            while data := client.recv(65536):
                counts["bytes"] += len(data)
        except OSError as exc:
            counts["error"] = str(exc)

    thread = None
    if not slow:
        thread = threading.Thread(target=read, name="soak-zuschauer", daemon=True)
        thread.start()
    return client, thread, counts


def close_client(client, thread):
    try:
        client.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    client.close()
    if thread:
        thread.join(2)
        if thread.is_alive():
            raise TimeoutError("Testzuschauer beendet sich nicht")


def memory_trend(samples, warmup_s=60):
    """Vorab festes Diagnosebudget, kein Beweis für Leak-Freiheit oder aktuellen nativen RSS."""
    settled = [sample for sample in samples if sample["elapsed_s"] >= warmup_s]
    span = settled[-1]["elapsed_s"] - settled[0]["elapsed_s"] if len(settled) >= 2 else 0
    result = {"warmup_s": warmup_s, "min_span_s": 300, "max_growth_bytes": 2 * 1024 * 1024,
              "max_slope_bytes_s": 1024, "samples": len(settled), "span_s": span,
              "sufficient_evidence": len(settled) >= 6 and span >= 300, "passed": None,
              "leak_free_proven": False, "current_native_rss_measured": False}
    if not result["sufficient_evidence"]:
        return result
    xs = [sample["elapsed_s"] - settled[0]["elapsed_s"] for sample in settled]
    ys = [sample["python_heap_bytes"] for sample in settled]
    mean_x, mean_y = sum(xs) / len(xs), sum(ys) / len(ys)
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / sum((x - mean_x) ** 2 for x in xs)
    growth = ys[-1] - ys[0]
    result.update(heap_growth_bytes=growth, heap_slope_bytes_s=slope,
                  process_peak_rss_growth_bytes=settled[-1]["process_peak_rss_bytes"] - settled[0]["process_peak_rss_bytes"],
                  passed=growth <= result["max_growth_bytes"] and slope <= result["max_slope_bytes_s"])
    return result


def run_phase(directory, detector, images, duration_s, *, fault="none", memory_warmup_s=0, on_progress=None,
              process_probe=False, sim_trace=False):
    if not math.isfinite(duration_s) or duration_s < 2 or fault not in ("none", "slow", "enospc"):
        raise ValueError("Mindestens zwei Sekunden und bekannte Fehlerart erforderlich")
    if not images:
        raise ValueError("Mindestens ein Prüfbild erforderlich")
    if not math.isfinite(memory_warmup_s) or not 0 <= memory_warmup_s < duration_s:
        raise ValueError("Speicher-Aufwärmzeit muss endlich und kleiner als die Phasendauer sein")
    import cv2

    cv2.imencode(".jpg", images[0])  # Import/Codec-Kaltstart vor der Zeitmessung
    memory = []  # höchstens ca. 202 Proben, keine Frames/Ereignisse sammeln
    sample_interval_s = max(10, duration_s / 200)
    first_snapshot = None
    allocation_diff = []
    last_progress_s = 0
    frames = discarded = 0
    detected_error = None
    close_error_detected = False
    pipeline = PerceptionLoop(detector)
    recorder = FaultRecorder(directory, {"sim": True, "synthetic": False, "fault": fault,
                                        "source": "wiederholte CC0-Standbilder, CPU, kein ARM"},
                             fault=fault, capacity=8 if fault == "slow" else 128)
    stats = WireStats(recorder)
    tracemalloc.start()
    try:
        with ExitStack() as stack:
            link = stack.enter_context(open_sim(**({"cmd_trace": Path(directory) / "sim-cmd.jsonl"} if sim_trace else {})))
            link.on_event = stats
            link.wait_status(lambda _: link.info is not None, timeout_s=3)
            stream = FrameStream(port=0, host="127.0.0.1")
            stack.callback(stream.close)
            for slow in (False, True):
                client, thread, counts = reader_client(stream._server.server_port, slow=slow)
                stack.callback(close_client, client, thread)
                if not slow:
                    fast_counts = counts
            probe = stack.enter_context(HostProbe(recorder))
            if process_probe:
                stack.enter_context(ProcessProbe(Path(directory) / "process.jsonl"))
            sender = SetpointSender(link, on_tick=recorder.record)
            stack.callback(link.stop)
            stack.callback(sender.stop)
            start = time.monotonic()
            recorder.record({"kind": "control", "key": "load_start", "time_s": start})
            sender.start()
            recorder.inject_after_s = start + 1
            next_frame = start
            while time.monotonic() - start < duration_s:
                sender.check()
                try:
                    recorder.check()
                except RuntimeError as exc:
                    if fault != "enospc":
                        raise
                    detected_error = str(exc)
                    break
                if link.status_age_s > .5 or link.status.state == "RUN":
                    raise RuntimeError("Kein frischer passiver Simulatorstatus")
                captured_s = time.monotonic()
                step = pipeline.process(images[frames % len(images)], captured_s)
                sender.set(step.v_mps, step.omega_rad_s, observed_at_s=captured_s)
                recorder.record({"kind": "frame", **asdict(step), "timestamp_source": "host_fixture"})
                stream.publish(images[frames % len(images)])
                frames += 1
                discarded += step.discarded is not None
                elapsed = time.monotonic() - start
                if first_snapshot is None and elapsed >= memory_warmup_s:
                    first_snapshot = tracemalloc.take_snapshot()
                if not memory or elapsed - memory[-1]["elapsed_s"] >= sample_interval_s:
                    heap, heap_peak = tracemalloc.get_traced_memory()
                    rss_peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
                    memory.append({"elapsed_s": elapsed, "python_heap_bytes": heap, "python_peak_bytes": heap_peak,
                                   "process_peak_rss_bytes": rss_peak if platform.system() == "Darwin" else rss_peak * 1024})
                if on_progress and elapsed - last_progress_s >= 60:
                    last_progress_s = elapsed
                    on_progress({"fault": fault, "elapsed_s": elapsed, "frames": frames,
                                 "commands": stats.commands, "max_cmd_gap_s": stats.max_gap_s,
                                 "dropped": recorder.dropped, "memory": memory[-1]})
                next_frame += .2  # Lastprofil fünf Erkennungen pro Sekunde, kein Pi-FPS-Versprechen
                time.sleep(max(0, next_frame - time.monotonic()))
            elapsed = time.monotonic() - start
            recorder.record({"kind": "control", "key": "load_end", "time_s": start + elapsed})
    finally:
        try:
            recorder.close()
        except RuntimeError as exc:
            if fault != "enospc" or "[Errno 28]" not in str(exc):
                raise
            if detected_error is None:
                detected_error = str(exc)
                close_error_detected = True
        finally:
            # Erst nachdem Sender/Clients/Simulator beendet sind: kein erzwungenes
            # globales GC während des gemessenen 20-Hz-Betriebs.
            if first_snapshot is not None:
                gc.collect()
                final_snapshot = tracemalloc.take_snapshot()
                for stat in final_snapshot.compare_to(first_snapshot, "lineno")[:10]:
                    frame = stat.traceback[-1]
                    allocation_diff.append({"file": Path(frame.filename).name, "line": frame.lineno,
                                            "size_diff_bytes": stat.size_diff, "count_diff": stat.count_diff})
            tracemalloc.stop()
    checks = {"no_arm": stats.arms == 0, "commands_present": stats.commands >= 10,
              "cmd_gap_budget": stats.max_gap_s <= .15,
              "frames_present": frames >= 1, "fast_reader_received": fast_counts["bytes"] > 0}
    if fault == "none":
        checks.update(no_drops=recorder.dropped == 0, no_recorder_error=recorder.error is None,
                      fresh_frames=discarded == 0)
    elif fault == "slow":
        checks["overflow_visible"] = recorder.dropped > 0
    else:
        checks["write_error_visible"] = detected_error is not None and "Errno 28" in detected_error
    trend = memory_trend(memory, memory_warmup_s)
    if fault == "none" and trend["sufficient_evidence"]:
        checks["python_memory_budget"] = trend["passed"]
    trace_analysis = compare_trace(Path(directory) / "sim-cmd.jsonl", Path(directory) / "events.jsonl") if sim_trace else None
    if sim_trace and fault == "none":
        checks["sim_trace_complete"] = trace_analysis["complete"]
    return {"fault": fault, "elapsed_s": elapsed, "frames": frames, "discarded_frames": discarded,
            "commands": stats.commands, "max_cmd_gap_s": stats.max_gap_s,
            "dropped": recorder.dropped, "recorder_error": detected_error,
            "recorder_error_detected_during_close": close_error_detected,
            "fast_reader_bytes": fast_counts["bytes"], "memory": memory, "memory_trend": trend,
            "allocation_diff_after_gc": allocation_diff,
            "allocation_diff_scope": "warmer Lauf gegen beendete Worker nach GC, nicht stationärer Live-Heap",
            "host_probe": probe.summary(),
            "independent_process_enabled": process_probe,
            "sim_cmd_trace": trace_analysis,
            "checks": checks, "passed": all(checks.values())}


def active_timing(path):
    """Vorab markiertes Lastfenster, nicht nachträglich aus günstigen Ereignissen gewählt.

    Der vollständige Lebenszyklus bleibt separat unverändert ausgewertet.
    Ohne beide expliziten Marker keine eingeschränkte Auswertung.
    """
    markers = {}
    dropped = None
    with Path(path).open() as file:
        for line in file:
            event = json.loads(line)
            if event["kind"] == "control" and event.get("key") in ("load_start", "load_end"):
                if event["key"] in markers:
                    raise ValueError("Doppelter Lastfenster-Marker")
                markers[event["key"]] = event["time_s"]
            if event["kind"] == "end":
                dropped = event["dropped"]
    if set(markers) != {"load_start", "load_end"} or dropped is None:
        raise ValueError("Lastfenster oder vollständiges Protokollende fehlt")
    start, end = markers["load_start"], markers["load_end"]
    if not 0 <= start < end:
        raise ValueError("Ungültiges Lastfenster")

    def events():
        with Path(path).open() as file:
            yield json.loads(next(file))
            yield {"kind": "control", "key": "load_start", "time_s": start}
            for line in file:
                event = json.loads(line)
                if event["kind"] == "frame":
                    if start <= event["captured_s"] <= event["finished_s"] <= end:
                        yield event
                elif event["kind"] in ("tx", "rx", "error") and start <= event["time_s"] <= end:
                    yield event
        yield {"kind": "end", "time_s": end, "dropped": dropped}

    return {"recorded_window": markers, "analysis": analyze_events(events())}


def write_report(path, report):
    """Exklusiver Diagnosebeleg; nie vorhandene Versuche überschreiben."""
    with Path(path).open("x", encoding="utf-8") as file:
        file.write(json.dumps(report, indent=2, allow_nan=False) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("video", type=Path)
    ap.add_argument("--annotation", type=Path, required=True)
    ap.add_argument("--modell", type=Path, default=Path("models") / CPU_MODEL_NAME)
    ap.add_argument("--dauer", type=float, default=600)
    ap.add_argument("--speicher-aufwaermen", type=float, default=60, help="Nur Speicheranalyse, Timing behält ganzes Lastfenster")
    ap.add_argument("--ausgabe", type=Path, required=True, help="Neuer Ordner")
    ap.add_argument("--prozessprobe", action="store_true", help="Separate POSIX-Hostprobe; zusätzliche Diagnoselast")
    ap.add_argument("--sim-trace", action="store_true", help="Direkte CMD-Zeiten nur im Host-Simulator; Beobachtereffekt")
    args = ap.parse_args(argv)
    if not math.isfinite(args.dauer) or args.dauer < 2:
        ap.error("Dauer muss endlich und mindestens zwei Sekunden sein")
    if not math.isfinite(args.speicher_aufwaermen) or not 0 <= args.speicher_aufwaermen < args.dauer:
        ap.error("Speicher-Aufwärmzeit muss endlich, nicht negativ und kürzer als die Dauer sein")
    fixture = load_annotations(args.annotation)
    images = [image for _, image in case_images(args.video, fixture)]
    args.ausgabe.mkdir(parents=True, exist_ok=False)
    detector = CoralPersonDetector(CameraModel(width_px=fixture["width_px"], height_px=fixture["height_px"]),
                                   args.modell, edge_tpu=False)
    try:
        phases = []
        for fault, duration in (("none", args.dauer), ("slow", 4), ("enospc", 4)):
            try:
                phase = run_phase(args.ausgabe / fault, detector, images, duration, fault=fault,
                                  memory_warmup_s=args.speicher_aufwaermen if fault == "none" else 0,
                                  process_probe=args.prozessprobe,
                                  sim_trace=args.sim_trace,
                                  on_progress=lambda row: print(json.dumps({"progress": row}), flush=True))
            except (OSError, RuntimeError, ValueError, TypeError, KeyError, subprocess.SubprocessError) as exc:
                phase = {"fault": fault, "passed": False, "execution_error": f"{type(exc).__name__}: {exc}"}
            phases.append(phase)
            # Vor der nächsten Phase sichern: spätere Fehler verlieren keine warme Speicheranalyse.
            (args.ausgabe / fault).mkdir(exist_ok=True)
            write_report(args.ausgabe / fault / "phase-report.json", phase)
            print(json.dumps({k: phase[k] for k in ("fault", "elapsed_s", "frames", "max_cmd_gap_s", "passed",
                                                   "execution_error") if k in phase}), flush=True)
            if "execution_error" in phase:
                break  # Weitere Fehlerphasen nicht als ausgeführt vortäuschen.
        timing = active = diagnostic = independent = None
        analysis_error = None
        if "execution_error" not in phases[0]:
            try:
                with (args.ausgabe / "none/events.jsonl").open() as file:
                    timing = analyze_events(json.loads(line) for line in file)
                active = active_timing(args.ausgabe / "none/events.jsonl")
                diagnostic = diagnose_timing(args.ausgabe / "none/events.jsonl",
                                            start_s=active["recorded_window"]["load_start"],
                                            end_s=active["recorded_window"]["load_end"],
                                            gc_buffer_dropped=phases[0]["host_probe"]["gc_buffer_dropped"])
                if args.prozessprobe:
                    independent = compare_process(args.ausgabe / "none/process.jsonl", diagnostic["worst_cmd_windows"],
                                                  start_s=active["recorded_window"]["load_start"],
                                                  end_s=active["recorded_window"]["load_end"])
            except (OSError, RuntimeError, ValueError, TypeError, KeyError, StopIteration) as exc:
                analysis_error = f"{type(exc).__name__}: {exc}"
        report = {"format": 1, "hardware_verified": False, "quality_approved": False,
                  "scope": "Host-Software, keine Kamera/USB/Temperatur, wiederholte Standbilder, ohne ARM",
                  "model_sha256": sha256(args.modell), "video_sha256": sha256(args.video),
                  "annotation_sha256": sha256(args.annotation), "phases": phases, "timing": timing,
                  "active_timing": active,
                  "host_diagnostics": diagnostic,
                  "independent_process": independent,
                  "analysis_error": analysis_error,
                  "phases_not_run": [f for f in ("none", "slow", "enospc") if f not in {p["fault"] for p in phases}],
                  "passed": len(phases) == 3 and all(p["passed"] for p in phases) and analysis_error is None
                            and active is not None and not active["analysis"]["failures"] and diagnostic["complete"]
                            and (not args.prozessprobe or independent is not None and independent["complete"])}
        write_report(args.ausgabe / "report.json", report)
        return int(not report["passed"])
    finally:
        detector.close()


if __name__ == "__main__":
    raise SystemExit(main())
