"""Live-Bild im Browser (MJPEG): http://<pi-name>.local:8000

MJPEG: Der Browser bekommt eine Folge von JPEG-Bildern in einer einzigen
HTTP-Antwort ("multipart/x-mixed-replace") — braucht keine Zusatzsoftware,
läuft in jedem Browser. Nur Standardbibliothek + OpenCV.
"""

from __future__ import annotations

import ipaddress
import json
import math
import mimetypes
import re
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np

STATIC = Path(__file__).with_name("hardware_static")
_VEHICLE_HOST = re.compile(r"fm\d{2}(\.local)?\Z", re.IGNORECASE)


def host_allowed(header: str | None) -> bool:
    """Host-Kopfzeile gegen feste Namen prüfen: fmNN, fmNN.local, localhost, private IP-Literale.

    Schutz gegen DNS-Rebinding: Eine fremde Webseite kann ihren eigenen Namen auf die Adresse des Pi
    zeigen lassen. Der Browser sendet dann weiter den fremden Namen als Host, und der Vergleich
    „Origin gleich Host“ allein ließe die Anfrage durch. Port ist erlaubt, wird aber nicht geprüft.
    Kein Ersatz für Not-Aus und Pico-Schutzfunktionen.
    """
    if not header:
        return False
    host = header.strip()
    if host.startswith("["):  # IPv6-Literal, optional mit Port
        literal, bracket, rest = host[1:].partition("]")
        if not bracket or (rest and not re.fullmatch(r":\d{1,5}", rest)):
            return False
        host = literal
    else:
        name, colon, port = host.rpartition(":")
        if colon:
            if not re.fullmatch(r"\d{1,5}", port):
                return False
            host = name
    if host.lower() == "localhost" or _VEHICLE_HOST.match(host):
        return True
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        return False
    return address.is_loopback or address.is_link_local or (address.is_private and not address.is_unspecified)


class FrameStream:
    def __init__(self, port: int = 8000, quality: int = 70, *, host: str = "0.0.0.0", status_provider=None,
                 control=None, simulated=False, task_name=None):
        self._status_provider = status_provider
        self.control = control
        self._jpeg: bytes | None = None
        self._cond = threading.Condition()
        self._quality = quality
        self._closed = False
        self._version = 0
        self._clients: set[socket.socket] = set()
        self._close_lock = threading.Lock()
        stream = self

        class Handler(BaseHTTPRequestHandler):
            def setup(self) -> None:
                self.request.settimeout(1.0)  # langsame Leser halten keinen Thread endlos fest
                super().setup()
                with stream._cond:
                    stream._clients.add(self.connection)

            def finish(self) -> None:
                try:
                    super().finish()
                finally:
                    with stream._cond:
                        stream._clients.discard(self.connection)

            def log_message(self, *args: object) -> None:  # kein Log je Bild
                pass

            def host_ok(self) -> bool:
                if host_allowed(self.headers.get("Host")):
                    return True
                self.json(403, {"error": "Dashboard nur über fmNN, fmNN.local, localhost oder die "
                                         "IP-Adresse des Fahrzeugs öffnen."})
                return False

            def do_GET(self) -> None:
                try:
                    if not self.host_ok():
                        return
                    self.send_frames()
                except (OSError, TimeoutError):
                    return  # Browser weg, WLAN weg oder Leser zu langsam

            def json(self, status, body):
                payload = json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                self.wfile.write(payload)

            def do_POST(self):
                if not self.host_ok():
                    return
                if stream.control is None:
                    self.send_error(501, "Dieser Stream hat keine Fahrbedienung.")
                    return
                # Keine fremden Webseiten als versteckte Fernbedienung; keine Rollen/Passwörter.
                origin = self.headers.get("Origin")
                site = self.headers.get("Sec-Fetch-Site")
                if ((origin is not None and origin != "http://" + self.headers.get("Host", ""))
                        or site not in (None, "same-origin", "none")):
                    self.json(403, {"error": "Bedienung nur von dieser Fahrzeugseite."})
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= 2048 or self.headers.get_content_type() != "application/json":
                        raise ValueError("Kleines JSON-Objekt erforderlich.")
                    body = json.loads(self.rfile.read(size))
                    if not isinstance(body, dict):
                        raise TypeError("JSON-Objekt erforderlich.")
                    path = urlsplit(self.path).path
                    result = {}
                    if path == "/api/start":
                        result = stream.control.start(instance=body.get("instance"), revision=body.get("revision"))
                    elif path == "/api/stop":
                        stream.control.stop()
                    elif path == "/api/heartbeat":
                        stream.control.heartbeat(body.get("lease"))
                    elif path == "/api/release":
                        stream.control.release(body.get("lease"))
                    else:
                        self.json(404, {"error": "Unbekannter Befehl."})
                        return
                    self.json(200, {"ok": True, **result})
                except (ValueError, TypeError, OSError) as exc:
                    self.json(409, {"error": str(exc)})

            def send_frames(self) -> None:
                # Abfrageteil ignorieren: Das Dashboard hängt eine Zahl an /stream, um den Bildstrom
                # nach Neustart, Verbindungsverlust oder verborgenem Tab neu zu öffnen.
                self.path = urlsplit(self.path).path
                if self.path == "/status":
                    if stream._status_provider is None:
                        self.send_error(503, "Status nicht verfügbar")
                        return
                    try:
                        data = stream._status_provider()
                        data["control"] = (stream.control.snapshot() if stream.control is not None
                                           else {"enabled": False})
                        data["runtime"] = {"simulated": simulated, "task": task_name}
                        self.json(200, data)
                    except (TypeError, ValueError):
                        self.send_error(503, "Status nicht verfügbar")
                    return
                if self.path in ("/", "/dashboard.css", "/dashboard.js"):
                    name = "index.html" if self.path == "/" else self.path[1:]
                    data = (STATIC / name).read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", mimetypes.guess_type(name)[0] + "; charset=utf-8")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.send_header("Content-Security-Policy", "default-src 'self'; frame-ancestors 'none'")
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if self.path not in ("/", "/stream"):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                version = -1
                while True:
                    with stream._cond:
                        stream._cond.wait_for(lambda version=version: stream._closed or
                                              (stream._jpeg is not None and stream._version != version))
                        if stream._closed:
                            return
                        jpeg = stream._jpeg
                        version = stream._version
                    header = f"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n"
                    self.wfile.write(header.encode("ascii") + jpeg + b"\r\n")

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=lambda: self._server.serve_forever(poll_interval=.05),
                                        name="mjpeg", daemon=True)
        self._thread.start()
        self._last = -math.inf

    def publish(self, frame_bgr: np.ndarray, max_fps: float = 10.0) -> None:
        """Neues Bild bereitstellen (gedrosselt, damit das WLAN nicht leidet)."""
        if not math.isfinite(max_fps) or max_fps <= 0:
            raise ValueError("Bildrate muss endlich und positiv sein")
        with self._cond:
            if self._closed:
                return
        now = time.monotonic()
        if now - self._last < 1.0 / max_fps:
            return
        self._last = now
        import cv2

        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if ok:
            with self._cond:
                if self._closed:
                    return
                self._jpeg = buf.tobytes()
                self._version += 1
                self._cond.notify_all()

    def close(self) -> None:
        with self._close_lock:
            with self._cond:
                if self._closed:
                    return
                self._closed = True
                clients = list(self._clients)
                self._cond.notify_all()
            # Auch einen gerade im Socket-Schreiben wartenden Client lösen.
            for client in clients:
                try:
                    client.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
            self._server.shutdown()
            self._server.server_close()
            self._thread.join(timeout=1.0)
