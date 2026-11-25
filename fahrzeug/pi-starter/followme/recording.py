"""Begrenzter, asynchroner Fahrtenschreiber. Keine Bilder ohne ausdrückliche Wahl."""

from __future__ import annotations

import hashlib
import json
import platform
import queue
import subprocess
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path


def runtime_metadata(config: dict) -> dict:
    metadata = {"python": platform.python_version(), "platform": platform.platform()}
    try:
        commit = subprocess.run(["git", "-C", str(Path(__file__).parent), "rev-parse", "HEAD"],
                                capture_output=True, text=True, check=False, timeout=2)
        metadata["git_commit"] = commit.stdout.strip() if commit.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        metadata["git_commit"] = None
    if config["erkenner"] in ("person", "coral"):
        path = Path(config["modell"] if config["erkenner"] == "person" else config["coral_modell"])
        if path.is_file():
            metadata["model_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    return metadata


class FlightRecorder:
    def __init__(self, directory: Path, metadata: dict, *, images: bool = False, capacity: int = 128):
        if capacity < 1:
            raise ValueError("Warteschlange muss begrenzt und positiv sein")
        self.directory = Path(directory)
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=False)
        self.images = images
        if images:
            (self.directory / "bilder").mkdir(mode=0o700)
        self._file = (self.directory / "events.jsonl").open("x", encoding="utf-8")
        self._queue: queue.Queue = queue.Queue(maxsize=capacity)
        self._stopping = threading.Event()
        self.dropped = 0
        self.error: str | None = None
        self._count = 0
        try:
            self._write({"kind": "run", "format": 1, "utc": datetime.now(timezone.utc).isoformat(), **metadata})
        except Exception:
            self._file.close()
            raise
        self._thread = threading.Thread(target=self._loop, name="fahrtenschreiber", daemon=True)
        self._thread.start()

    def _write(self, event: dict) -> None:
        self._file.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
        self._file.flush()

    def record(self, event: dict, image=None) -> None:
        """Auch aus dem Sender-Thread: niemals auf Datenträger oder freien Platz warten."""
        if self._stopping.is_set() or self.error:
            self.dropped += 1
            return
        try:
            self._queue.put_nowait((dict(event), image.copy() if self.images and image is not None else None))
        except queue.Full:
            self.dropped += 1

    def _loop(self) -> None:
        try:
            while not self._stopping.is_set() or not self._queue.empty():
                try:
                    event, image = self._queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                if image is not None:
                    import cv2

                    self._count += 1
                    name = f"bilder/{self._count:07d}.jpg"
                    if not cv2.imwrite(str(self.directory / name), image):
                        raise OSError("Bild konnte nicht gespeichert werden")
                    event["image"] = name
                self._write(event)
            self._write({"kind": "end", "time_s": time.monotonic(), "dropped": self.dropped})
        except (OSError, ValueError, TypeError) as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self._file.close()

    def check(self) -> None:
        if self.error:
            raise RuntimeError(f"Fahrtenschreiber: {self.error}")

    @contextmanager
    def capture_errors(self):
        try:
            yield
        except Exception as exc:
            self.record({"kind": "error", "message": f"{type(exc).__name__}: {exc}", "time_s": time.monotonic()})
            raise

    def close(self) -> None:
        self._stopping.set()
        self._thread.join(timeout=2)
        if self._thread.is_alive():
            raise TimeoutError("Fahrtenschreiber beendet sich nicht; Protokoll ist unvollständig")
        self.check()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
