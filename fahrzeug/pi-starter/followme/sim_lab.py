"""Lokales Lernlabor: Produktions-AprilTags → Python-Aufgabe → echter Firmwarekern.

Start: python -m followme.sim_lab
Es gibt hier ausschließlich den Host-Simulator, keinen seriellen Gerätepfad.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import mimetypes
import secrets
import threading
import time
import traceback
from dataclasses import asdict, replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

if __name__ == "__main__":
    print("Lernlabor startet … (beim ersten Start werden NumPy und OpenCV geladen)", flush=True)

from .controller import FollowParams
from .detect import ArucoDetector
from .drive_control import LOOP_TIMEOUT_S
from .link import SetpointSender, open_sim
from .pipeline import PerceptionLoop
from .sensors import side_views
from .sim_world import CAMERA, SCENARIOS, Box, World, collision, front_distance, render, side_distance, tag_image
from .task import Task, TaskError, apply_task, load_task
from .virtual_drive import Pose

STATIC = Path(__file__).with_name("sim_static")
LEASE_S = 1.2


class Lab:
    def __init__(self, scenario="folgen", task_path: Path | None = None):
        self.lock = threading.RLock()
        self.world = World.create(scenario)
        self.pose = Pose()
        self.profile = json.loads((STATIC / "profile.json").read_text())
        self.task_path = task_path
        self.task_error = None
        try:
            self.task = load_task(task_path)
        except TaskError as exc:
            # Lernlabor trotzdem starten: die Seite zeigt den Fehler, Start lädt die korrigierte Datei.
            self.task, self.task_error = Task(), str(exc)
            print("Fehler in der Aufgabendatei: " + self.task_error, flush=True)
        self.pipeline = PerceptionLoop(ArucoDetector(CAMERA, 300, .168, "apriltag36h11"),
                                       FollowParams(v_max_mps=.3, omega_max_rad_s=.7))
        self.revision = 0
        self.lease = None
        self.deadline_s = 0.
        self.elapsed_s = 0.
        self.base_elapsed_s = 0.
        self.started_s = 0.
        self.last_step = None
        self.markers = []
        self.jpeg = b""
        self.reason = ("Fehler in der Aufgabendatei. Datei korrigieren, dann Versuch starten." if self.task_error
                       else "Bereit. Startet euren ersten Versuch.")
        self.error = None
        self.trail = []
        self.alive = threading.Event()
        self.link = open_sim()
        try:
            self.previous = self.link.wait_status(lambda _: self.link.info is not None, timeout_s=3)
            self.info = self.link.info
            self.sender = SetpointSender(self.link).start()
        except BaseException:
            self.link.close()
            raise
        self.alive.set()
        self.worker = threading.Thread(target=self._loop, name="lernwelt", daemon=True)
        self.monitor = threading.Thread(target=self._monitor, name="bedienfreigabe", daemon=True)
        self.worker.start()
        self.monitor.start()

    def _new_pipeline(self):
        self.pipeline = PerceptionLoop(self.pipeline.detector, self.pipeline.params)
        try:
            self.task = load_task(self.task_path)
        except TaskError as exc:
            self.task, self.task_error = Task(), str(exc)
            self.reason = "Fehler in der Aufgabendatei. Datei korrigieren, dann Versuch starten."
            print("Fehler in der Aufgabendatei: " + self.task_error, flush=True)
            raise
        self.task_error = None
        self.last_step, self.markers = None, []

    def _task_failed(self, exc: TaskError):
        """Fehler im Teamcode: Versuch stoppen, Meldung zeigen, Lernlabor läuft weiter."""
        print("Fehler in der Aufgabendatei: " + str(exc), flush=True)
        traceback.print_exception(exc.__cause__ or exc)
        with self.lock:
            self.task, self.task_error = Task(), str(exc)  # fehlerhafte Aufgabe nicht in jedem Bild erneut aufrufen
            if self.lease:
                self._stop("Versuch gestoppt: Fehler in der Aufgabendatei. Datei korrigieren, dann neu starten.")
            else:
                self.revision += 1
                self.reason = "Fehler in der Aufgabendatei. Datei korrigieren, dann Versuch starten."

    def start(self, revision: int):
        with self.lock:
            if type(revision) is not int or revision != self.revision:
                raise ValueError("Ansicht ist veraltet. Zustand neu laden und bewusst starten.")
            if self.lease is not None:
                raise ValueError("Ein Browser bedient bereits diesen Versuch.")
            if self.error:
                raise ValueError("Lernlabor neu starten: " + self.error)
            self._new_pipeline()
            self.sender.invalidate()
            self._send_distances()
            # Neue ToF-Probe muss vor ARM durch den Firmwarekern gegangen sein.
            old_t = self.link.status.t_ms
            self.link.wait_status(lambda s: s.t_ms >= old_t + 100, timeout_s=1)
            try:
                self.link.arm()
                self.link.wait_status(lambda s: s.state == "RUN", timeout_s=1)
            except (OSError, TimeoutError):
                self._stop("Start vom Pico abgelehnt. Frontabstand und Status prüfen.")
                raise ValueError(self.reason) from None
            self.revision += 1
            self.lease = secrets.token_urlsafe(24)
            self.deadline_s = time.monotonic() + LEASE_S
            self.base_elapsed_s, self.started_s = self.elapsed_s, time.monotonic()
            self.reason = "Versuch läuft. Beobachtet Bild, Sollwert und Fahrt."
            return {"lease": self.lease, "revision": self.revision}

    def heartbeat(self, lease):
        with self.lock:
            if not self.lease or not secrets.compare_digest(str(lease), self.lease):
                raise ValueError("Bedienfreigabe abgelaufen. Neu starten.")
            if time.monotonic() >= self.deadline_s:
                self._stop("Bedienverbindung unterbrochen. Bewusster neuer Start nötig.")
                raise ValueError(self.reason)
            self.deadline_s = time.monotonic() + LEASE_S

    def _stop(self, reason):
        self.revision += 1
        self.lease = None
        self.sender.invalidate()
        self.last_step, self.markers = None, []
        self.link.stop()
        try:
            self.link.wait_status(lambda s: s.state == "STANDBY", timeout_s=1)
            self.reason = reason
        except TimeoutError:
            self.reason = "STOPP unbestätigt. Lernlabor neu starten."
            self.error = self.reason

    def stop(self):
        with self.lock:
            self._stop("Angehalten. Mit Start beginnt eine neue Bedienfreigabe.")

    def reset(self, scenario):
        world = World.create(scenario)
        with self.lock:
            self._stop("Szene zurückgesetzt. Bereit für einen neuen Versuch.")
            if self.error:
                raise ValueError(self.error)
            self.previous = self.link.wait_status(lambda s: abs(s.v_l_mm_s) <= 2 and abs(s.v_r_mm_s) <= 2,
                                                  timeout_s=2)
            self.world, self.pose, self.trail = world, Pose(), []
            self.elapsed_s, self.base_elapsed_s = 0., 0.
            self._new_pipeline()

    def add_box(self, x_m, y_m):
        if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or abs(v) > 8
               for v in (x_m, y_m)):
            raise ValueError("Kistenposition muss zwischen −8 und 8 m liegen.")
        with self.lock:
            if self.lease or len(self.world.boxes) >= 15:
                raise ValueError("Erst stoppen; höchstens 15 Kisten pro Szene.")
            candidate = copy.deepcopy(self.world)
            candidate.boxes.append(Box(f"Eigene Kiste {len(candidate.boxes) + 1}", x_m, y_m))
            if collision(candidate, self.pose, self.profile["footprint"]):
                raise ValueError("Die Kiste würde das Fahrzeug berühren.")
            self.world = candidate
            self.revision += 1

    def hide_target(self, hidden):
        if type(hidden) is not bool:
            raise ValueError("Verdeckung muss true oder false sein")
        with self.lock:
            self.world.target_hidden = hidden

    def _monitor(self):
        while self.alive.is_set():
            try:
                with self.lock:
                    if self.lease and time.monotonic() >= self.deadline_s:
                        self._stop("Bedienverbindung unterbrochen. Bewusster neuer Start nötig.")
                    self.sender.check()
            except (OSError, ValueError) as exc:
                with self.lock:
                    self.lease = None
                    self.error = self.reason = str(exc)
                return
            time.sleep(.04)

    def _send_distances(self):
        """Abstände aus der Szene an den Firmware-Simulator: Front-ToF und Ultraschall links/rechts."""
        self.link.send_sim("TOF", front_distance(self.world, self.pose))
        self.link.send_sim("US_L", side_distance(self.world, self.pose, "links"))
        self.link.send_sim("US_R", side_distance(self.world, self.pose, "rechts"))

    def _advance(self):
        status = self.link.status
        if status is None or self.link.status_age_s > .5:
            raise ConnectionError("Kein frischer Pico-Status")
        before = replace(self.pose)
        self.pose.advance(self.previous, status, self.info)
        self.previous = status
        length = math.hypot(self.pose.x_m - before.x_m, self.pose.y_m - before.y_m)
        steps = max(1, math.ceil(length / .01), math.ceil(abs(self.pose.yaw_rad - before.yaw_rad) / .03))
        for i in range(1, steps + 1):
            f = i / steps
            sample = Pose(before.x_m + (self.pose.x_m - before.x_m) * f,
                          before.y_m + (self.pose.y_m - before.y_m) * f,
                          before.yaw_rad + (self.pose.yaw_rad - before.yaw_rad) * f)
            hit = collision(self.world, sample, self.profile["footprint"])
            if hit and self.lease:
                self._stop(f"Berührung mit {hit}. Szene zurücksetzen und Versuch ändern.")
                break
        if self.lease:
            self.elapsed_s = self.base_elapsed_s + time.monotonic() - self.started_s
            self.world.update_target(self.elapsed_s)
        self._send_distances()
        if not self.trail or length > .002:
            self.trail.append([self.pose.x_m, self.pose.y_m])
            self.trail = self.trail[-1500:]

    def _loop(self):
        import cv2

        next_frame = 0.
        try:
            while self.alive.is_set():
                with self.lock:
                    self._advance()
                    scene, pose = copy.deepcopy(self.world), replace(self.pose)
                    revision, pipeline, task = self.revision, self.pipeline, self.task
                if time.monotonic() >= next_frame:
                    captured_s = time.monotonic()
                    frame = render(scene, pose)
                    targets, markers = pipeline.detector.detect_scene(frame, captured_s)
                    step = pipeline.evaluate(targets, captured_s, time.monotonic())
                    try:
                        step = apply_task(task, step, markers, self.link.status,
                                          status_fresh=self.link.status_age_s <= .5,
                                          sides=side_views(self.link.ultrasonic_snapshot(), time.monotonic()))
                    except TaskError as exc:
                        self._task_failed(exc)
                        next_frame = captured_s + .10
                        continue
                    if time.monotonic() - captured_s > LOOP_TIMEOUT_S:
                        with self.lock:
                            if self.lease and revision == self.revision:
                                # Nach einem Hänger nicht mit altem Ergebnis von selbst weiterfahren.
                                self._stop("Versuch gestoppt: Bildverarbeitung oder Aufgabe hängt. Neu starten.")
                    display = frame.copy()
                    for marker in markers:
                        x, y, w, h = map(round, marker.box)
                        cv2.rectangle(display, (x, y), (x + w, y + h), (57, 189, 46), 2)
                        cv2.putText(display, f"ID {marker.marker_id} / {marker.distance_m:.2f} m", (x, max(18, y - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .45, (32, 110, 20), 1)
                    ok, encoded = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 86])
                    with self.lock:
                        if revision == self.revision:
                            self.last_step, self.markers = step, markers if not step.discarded else []
                            if ok:
                                self.jpeg = encoded.tobytes()
                            if self.lease and time.monotonic() < self.deadline_s and not step.discarded:
                                self.sender.set(max(-.3, min(.3, step.v_mps)), max(-.7, min(.7, step.omega_rad_s)),
                                                observed_at_s=step.captured_s)
                            else:
                                self.sender.invalidate()
                    next_frame = captured_s + .10
                time.sleep(.015)
        except Exception as exc:  # noqa: BLE001 - jeder andere Fehler der Simulation muss STOP auslösen.
            with self.lock:
                self.error = str(exc)
                self._stop(f"Versuch abgebrochen: {exc}")

    def snapshot(self):
        with self.lock:
            step = self.last_step
            fresh = step is not None and time.monotonic() - step.captured_s <= .6 and not step.discarded
            status = self.link.status
            return {"mode": "simulation", "revision": self.revision, "running": self.lease is not None,
                    "elapsed_s": self.elapsed_s, "reason": self.reason, "error": self.error,
                    "task_error": self.task_error,
                    "pose": asdict(self.pose), "world": self.world.view(), "trail": list(self.trail),
                    "mcu": None if status is None else {**asdict(status), "flag_names": status.flag_names(),
                                                       "fresh": self.link.status_age_s <= .5},
                    "perception": {"fresh": bool(fresh), "age_s": None if step is None else time.monotonic() - step.captured_s,
                                   "state": step.state if fresh else "KEIN FRISCHES BILD",
                                   "target": asdict(step.chosen) if fresh and step.chosen else None,
                                   "markers": [asdict(m) for m in self.markers] if fresh else []},
                    "command": list(self.sender.current()[:2]), "task": self.task_path.name if self.task_path else "Folgeregler",
                    "scenarios": {key: {"name": value[0], "exercise": value[1]} for key, value in SCENARIOS.items()}}

    def close(self):
        self.alive.clear()
        self.worker.join(timeout=3)
        self.monitor.join(timeout=2)
        self.sender.stop()
        try:
            self.link.stop()
        finally:
            self.link.close()


def make_server(lab: Lab, port: int = 8000):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(2)

        def log_message(self, *args):
            pass

        def respond(self, code, payload, content_type="application/json; charset=utf-8"):
            data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode() if isinstance(payload, dict) else payload
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def local_request(self):
            actual = self.server.server_port
            hosts = {f"127.0.0.1:{actual}", f"localhost:{actual}"}
            origin = self.headers.get("Origin")
            if self.headers.get("Host") not in hosts or origin is not None and origin not in {f"http://{h}" for h in hosts}:
                self.respond(403, {"error": "Nur lokale Anfragen von dieser Lernlabor-Seite erlaubt"})
                return False
            return True

        def do_GET(self):
            if not self.local_request():
                return
            path = urlsplit(self.path).path
            if path == "/api/state":
                return self.respond(200, lab.snapshot())
            if path == "/api/frame.jpg":
                with lab.lock:
                    data = lab.jpeg
                return self.respond(200 if data else 503, data, "image/jpeg")
            if path.startswith("/api/tag/") and path.endswith(".png"):
                import cv2
                try:
                    data = cv2.imencode(".png", tag_image(int(path[9:-4])))[1].tobytes()
                    return self.respond(200, data, "image/png")
                except ValueError:
                    return self.respond(404, {"error": "Unbekannter Marker"})
            relative = "index.html" if path == "/" else path.lstrip("/")
            file = (STATIC / relative).resolve()
            if not file.is_relative_to(STATIC.resolve()) or not file.is_file():
                return self.respond(404, {"error": "Datei nicht gefunden"})
            mime = "text/javascript" if file.suffix == ".js" else mimetypes.guess_type(str(file))[0]
            self.respond(200, file.read_bytes(), mime or "application/octet-stream")

        def do_POST(self):
            if not self.local_request():
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096 or self.headers.get_content_type() != "application/json":
                    raise ValueError("Kleines JSON-Objekt erwartet")
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise TypeError("JSON-Objekt erwartet")
                path = urlsplit(self.path).path
                result = {}
                if path == "/api/start":
                    result = lab.start(body.get("revision"))
                elif path == "/api/heartbeat":
                    lab.heartbeat(body.get("lease"))
                elif path == "/api/stop":
                    lab.stop()
                elif path == "/api/reset":
                    lab.reset(body.get("scenario", "folgen"))
                elif path == "/api/box":
                    lab.add_box(body.get("x_m"), body.get("y_m"))
                elif path == "/api/hide":
                    lab.hide_target(body.get("hidden"))
                else:
                    return self.respond(404, {"error": "Unbekannter Befehl"})
                self.respond(200, {"ok": True, **result})
            except (ValueError, KeyError, TypeError, TimeoutError) as exc:
                self.respond(409, {"error": str(exc)})

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description="Follow-Me-Lernlabor mit virtueller Kamera und Pico-Firmware")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--szenario", choices=SCENARIOS, default="folgen")
    parser.add_argument("--aufgabe", type=Path, help="Eigene Python-Datei mit create_task()")
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("Port muss zwischen 0 und 65535 liegen")
    lab = Lab(args.szenario, args.aufgabe)
    try:
        server = make_server(lab, args.port)
        print(f"Lernlabor bereit: http://127.0.0.1:{server.server_port} (Strg-C beendet)", flush=True)
        try:
            server.serve_forever(poll_interval=.1)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
    finally:
        lab.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
