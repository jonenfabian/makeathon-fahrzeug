"""Hauptprogramm: Kamera → Erkennung → Folgeregler → Pico.

Aufruf auf dem Fahrzeug:
    python -m followme.follow --erkenner aruco --stream 8000
Am Laptop mit Webcam und Host-Simulator statt Pico:
    python -m followme.follow --erkenner aruco --quelle 0 --sim
Standard: Personenmarker tag36h11 ID 300, schwarze Kante 0,168 m.
Andere IDs/Wörterbücher nur bewusst konfigurieren; Fahrzeugfolgen ist nicht abgenommen.

Bedienung im Terminal (Eingabe + Enter):
    r   scharf schalten (ARM) — erst dann fährt das Fahrzeug
    s   anhalten (STOP)
    q   beenden
Ctrl-C beendet ebenfalls. Stürzt dieses Programm ab, hält der Pico das
Fahrzeug spätestens nach 300 ms an (Watchdog).

Raspberry Pi 3B: Die Bildschleife läuft so schnell, wie die Erkennung es
zulässt (ArUco ≈ 10 Bilder/s, Personen-Netz nur wenige Bilder/s — am
Fahrzeug mit tools/benchmark.py messen). Gesendet wird unabhängig davon im
20-Hz-Takt vom SetpointSender (link.py); ist der letzte Sollwert älter als
0,6 s, sendet er 0/0.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
import traceback
from contextlib import ExitStack
from dataclasses import asdict
from pathlib import Path

from .camera import Camera
from .config import DEFAULTS, camera_model, load_config, validate
from .controller import FollowParams
from .coral import DEFAULT_MODEL, CoralPersonDetector
from .dashboard_data import DashboardData
from .detect import WOERTERBUECHER, ArucoDetector, ColorDetector, Detector, PersonDetector, Target
from .drive_control import DriveControl, TerminalControl
from .geometry import CameraModel
from .link import McuLink, SetpointSender, open_serial, open_sim
from .pipeline import PerceptionLoop, Step
from .recording import FlightRecorder, runtime_metadata
from .sensors import side_views
from .tag_tracking import OdometryHistory, TagTracker
from .task import Task, TaskError, apply_task, load_task

MAX_LOOP_HZ = 20.0  # Obergrenze der Bildschleife (spart CPU, wenn die Erkennung schneller ist)


def make_detector(name: str, cam: CameraModel, args: argparse.Namespace) -> Detector:
    if name == "aruco":
        return ArucoDetector(cam, target_id=args.marker_id, marker_size_m=args.marker_groesse,
                             woerterbuch=args.woerterbuch)
    if name == "farbe":
        return ColorDetector(cam)
    if name == "person":
        return PersonDetector(cam, model_path=args.modell)
    if name == "coral":
        return CoralPersonDetector(cam, model_path=args.coral_modell, library=args.coral_bibliothek)
    raise ValueError(name)


def start_keyboard(on_control=None) -> queue.Queue[str]:
    """Liest Terminal-Eingaben in einem eigenen Thread (blockiert die Schleife nicht)."""
    q: queue.Queue[str] = queue.Queue()

    def reader() -> None:
        for line in sys.stdin:
            key = line.strip().lower()
            if on_control is not None:
                on_control(key)
            else:
                q.put(key)

    threading.Thread(target=reader, name="tastatur", daemon=True).start()
    return q


def draw_overlay(frame, targets: list[Target], chosen: Target | None, text: str) -> None:
    import cv2

    for t in targets:
        x, y, w, h = (int(v) for v in t.box)
        color = (0, 220, 0) if t is chosen else (160, 160, 160)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, f"{t.distance_m:.1f} m", (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    cv2.putText(frame, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def main(argv: list[str] | None = None) -> int:
    preliminary = argparse.ArgumentParser(add_help=False)
    preliminary.add_argument("--konfiguration", type=Path)
    selected, _ = preliminary.parse_known_args(argv)
    ap = argparse.ArgumentParser(description="Follow-me: einer Person folgen", parents=[preliminary])
    ap.add_argument("--erkenner", choices=["aruco", "farbe", "person", "coral"], default="aruco")
    ap.add_argument("--quelle", default="picam", help="picam | Webcam-Nummer | Videodatei")
    ap.add_argument("--port", default="/dev/ttyACM0", help="serielle Schnittstelle des Pico")
    ap.add_argument("--sim", action="store_true", help="Host-Simulator statt Pico")
    ap.add_argument("--stream", type=int, default=0, help="Port für Live-Bild im Browser (0 = aus)")
    ap.add_argument("--abstand", type=float, default=1.5, help="Wunschabstand [m]")
    ap.add_argument("--marker-id", type=int, default=300)
    ap.add_argument("--marker-groesse", type=float, default=0.168, help="Kantenlänge [m]")
    ap.add_argument("--woerterbuch", choices=sorted(WOERTERBUECHER), default="apriltag36h11",
                    help="Markerfamilie; Standard tag36h11 für Personenmarker ID 300")
    ap.add_argument("--modell", default="models/efficientdet_lite0.tflite")
    ap.add_argument("--coral-modell", type=Path, default=DEFAULT_MODEL)
    ap.add_argument("--coral-bibliothek", help="Pfad zu selbst gebautem libedgetpu.so.1 (siehe CORAL.md)")
    ap.add_argument("--fahrzeug")
    ap.add_argument("--aufgabe", type=Path, help="Eigene Aufgabe mit create_task(); dieselbe Datei wie im Lernlabor")
    for key in ("width_px", "height_px"):
        ap.add_argument("--" + key.replace("_", "-"), type=int)
    for key in ("hfov_deg", "vfov_deg"):
        ap.add_argument("--" + key.replace("_", "-"), type=float)
    ap.add_argument("--protokoll", type=Path, help="Neuer Ordner für JSONL-Fahrtenschreiber; keine Zugangsdaten")
    ap.add_argument("--bilder", action="store_true", help="Zusätzlich Kamerabilder speichern (Datenschutz beachten)")
    ap.add_argument("--ohne-verfolgung", action="store_true",
                    help="AprilTag nur per Erkennung, ohne kurze Verfolgung (Encoder + Bildvergleich) in Lücken")
    ap.add_argument("--belichtung-us", type=int,
                    help="Feste Belichtungszeit der Pi-Kamera in µs (100–33000), Verstärkung bleibt automatisch. "
                         "Ohne Angabe regelt die Kamera selbst")
    try:
        ap.set_defaults(**load_config(selected.konfiguration))
    except (OSError, ValueError) as exc:
        ap.error(str(exc))
    args = ap.parse_args(argv)
    try:
        values = validate({k: str(v) if isinstance(v := getattr(args, k), Path) else v for k in DEFAULTS})
    except ValueError as exc:
        ap.error(str(exc))
    if args.bilder and not args.protokoll:
        ap.error("--bilder erfordert --protokoll")
    if args.belichtung_us is not None and not 100 <= args.belichtung_us <= 33000:
        ap.error("--belichtung-us muss zwischen 100 und 33000 liegen")
    cam_model = camera_model(values)
    params = FollowParams(d_target_m=args.abstand)
    try:
        task, task_error = load_task(args.aufgabe), None
    except TaskError as exc:
        # Programm trotzdem starten: Start lädt die korrigierte Datei neu, das Dashboard zeigt den Fehler.
        task, task_error = Task(), str(exc)
        print("Fehler in der Aufgabendatei: " + task_error, flush=True)
    with ExitStack() as stack:
        recorder = None
        if args.protokoll:
            recorder = stack.enter_context(FlightRecorder(args.protokoll,
                {"config": values, "camera": asdict(cam_model), "params": asdict(params), "sim": args.sim,
                 "runtime": runtime_metadata(values), "belichtung_us": args.belichtung_us},
                images=args.bilder))
            stack.enter_context(recorder.capture_errors())
        detector = make_detector(args.erkenner, cam_model, args)
        if hasattr(detector, "close"):
            stack.callback(detector.close)
        camera = Camera(args.quelle, cam_model.width_px, cam_model.height_px, exposure_us=args.belichtung_us)
        stack.callback(camera.close)
        link: McuLink = stack.enter_context(open_sim() if args.sim else open_serial(args.port))
        if recorder:
            link.on_event = recorder.record
        pipeline = PerceptionLoop(detector, params)
        sender = SetpointSender(link)
        # Lücken der AprilTag-Erkennung bei Erschütterung kurz überbrücken (höchstens 0,8 s, tag_tracking.py).
        tracker = TagTracker(cam_model) if isinstance(detector, ArucoDetector) and not args.ohne_verfolgung else None
        odometry: OdometryHistory | None = None
        tracker_token = None

        def stop() -> None:
            sender.invalidate()
            sender.stop()
            link.stop()

        stack.callback(stop)  # auch bei Fehlern während wait_status/Stream-Aufbau
        stream, dashboard = None, None
        if args.stream:
            dashboard = DashboardData(args.fahrzeug, args.erkenner,
                                      args.marker_id if args.erkenner == "aruco" else None, link.status_snapshot,
                                      info_provider=lambda: link.info,
                                      ultrasonic_provider=link.ultrasonic_snapshot)
        quitting = threading.Event()
        control_errors = []

        def prepare():
            return PerceptionLoop(detector, params), load_task(args.aufgabe)

        st = link.wait_status(timeout_s=3)
        if st.state == "RUN":
            raise RuntimeError("Pico ist bereits RUN. STOP wird versucht; Zustand prüfen und bewusst neu starten.")
        sender.start()
        controls = DriveControl(link, sender, prepare, (pipeline, task),
                                clear=dashboard.clear if dashboard is not None else lambda: None,
                                task_error=task_error)
        stack.callback(controls.close)
        if args.stream:
            from .stream import FrameStream

            stream = FrameStream(args.stream, status_provider=dashboard.snapshot, control=controls,
                                 simulated=args.sim, task_name=str(args.aufgabe) if args.aufgabe else "Folgeregler")
            stack.callback(stream.close)
            print(f"Dashboard: http://<pi-name>.local:{args.stream} (Start/Stopp, Kamera und Messwerte)", flush=True)

        def fatal(message: str) -> None:
            control_errors.append(message)
            quitting.set()

        terminal = TerminalControl(controls, on_quit=quitting.set, on_fatal=fatal,
                                   say=lambda text: print(text, flush=True))

        def control(key: str) -> None:
            if key not in ("r", "s", "q"):
                return
            terminal.handle(key)  # s und q sofort, r lädt die Aufgabe in einem eigenen Thread
            if recorder:
                recorder.record({"kind": "control", "key": key, "time_s": time.monotonic()})

        print(f"Pico verbunden: Zustand {st.state}, Flags {st.flag_names() or '-'}")
        print("Eingabe: r = ARM, s = STOP, q = Ende (jeweils Enter); niemals automatisches ARM")
        start_keyboard(control)
        period = 1 / MAX_LOOP_HZ
        fps, last_print = 0.0, 0.0
        try:
            while not quitting.is_set():
                sender.check()
                t0 = time.monotonic()
                token, (pipeline, task) = controls.capture()
                sample = camera.read_timed()
                if sample is None:
                    print("Bildquelle zu Ende.")
                    break
                if (isinstance(detector, ArucoDetector)
                        and (detector.woerterbuch, detector.target_id, detector.marker_size_m)
                        == ("apriltag36h11", 300, .168)):
                    targets, markers = detector.detect_scene(sample.image, sample.captured_s)
                else:
                    targets, markers = detector.detect(sample.image), []
                gray = pose = None
                if tracker is not None:
                    import cv2

                    st_now, age_now, _ = link.status_snapshot()
                    if odometry is None and link.info is not None:
                        odometry = OdometryHistory(link.info.counts_per_rev, link.info.wheel_diameter_mm,
                                                   link.info.track_mm)
                    if odometry is not None and st_now is not None and age_now is not None:
                        odometry.add(time.monotonic() - age_now, st_now.ticks_l, st_now.ticks_r)
                    if token != tracker_token:  # Start/Stopp: kein Anker aus der vorigen Bedienung
                        tracker.reset()
                        tracker_token = token
                    gray = cv2.cvtColor(sample.image, cv2.COLOR_BGR2GRAY)
                    pose = odometry.pose_at(sample.captured_s) if odometry is not None else None
                    if not targets and (tracked := tracker.track(gray, sample.captured_s, pose)) is not None:
                        targets = [tracked]
                sender.check()  # Auch ein während der Erkennung verstummter Sender ist ein Fehler.
                finished_s = time.monotonic()
                step = pipeline.evaluate(targets, sample.captured_s, finished_s)
                if (tracker is not None and step.chosen is not None and not step.discarded
                        and not step.chosen.label.endswith("-verfolgt")):
                    tracker.confirm(gray, sample.captured_s, step.chosen, pose)
                try:
                    step = apply_task(task, step, markers, link.status, status_fresh=link.status_age_s <= 0.5,
                                      sides=side_views(link.ultrasonic_snapshot(), time.monotonic()))
                except TaskError as exc:
                    # Fehler im Teamcode: Fahrt stoppen und weiterlaufen. Start lädt die Datei neu.
                    print("Fehler in der Aufgabendatei: " + str(exc), flush=True)
                    traceback.print_exception(exc.__cause__ or exc)
                    controls.task_failed(str(exc), (pipeline, Task()))
                    if recorder:
                        recorder.record({"kind": "task_error", "message": str(exc), "time_s": time.monotonic()})
                    step = Step(sample.captured_s, time.monotonic(), [], None, 0, 0, step.state, "aufgabe")
                with controls.lock:
                    finished_s = time.monotonic()
                    if token != controls.generation:
                        step = Step(sample.captured_s, finished_s, [], None, 0, 0, "SUCHE", "bedienung")
                    if step.discarded or not controls.active:
                        sender.invalidate()
                    else:
                        sender.set(step.v_mps, step.omega_rad_s, observed_at_s=sample.captured_s)
                    if step.discarded:
                        markers = []
                    if dashboard is not None:
                        dashboard.update(step, markers, sample.timestamp_source)
                controls.frame_done()
                if step.discarded:
                    markers = []  # verworfene oder vor STOP/ARM aufgenommene Daten nicht anzeigen
                if recorder:
                    recorder.record({"kind": "frame", **asdict(step), "markers": [asdict(m) for m in markers],
                                     "timestamp_source": sample.timestamp_source,
                                     "exposure_us": sample.exposure_us, "gain": sample.gain},
                                    sample.image)
                    recorder.check()
                st = link.status
                dt = time.monotonic() - t0
                fps = 0.9 * fps + 0.1 / max(dt, period)
                text = (f"{st.state if st else '?'} {step.state} v={step.v_mps:.2f} "
                        f"w={step.omega_rad_s:.2f} {fps:.0f} B/s {step.discarded or ''}")
                if stream:
                    draw_overlay(sample.image, step.targets, step.chosen, text)
                    stream.publish(sample.image)
                if t0 - last_print > .5:
                    last_print = t0
                    print(f"{text} | Bildalter {(step.finished_s - step.captured_s) * 1000:.0f} ms "
                          f"| Pico {st.flag_names() if st else '-'}", flush=True)
                time.sleep(max(0, period - (time.monotonic() - t0)))
            if control_errors:
                raise RuntimeError(control_errors[-1])
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
