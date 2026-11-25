"""Bildquelle: Pi-Kamera (Picamera2), USB-Webcam oder Videodatei.

Auf dem Fahrzeug: Raspberry Pi Camera Module 3 Wide über Picamera2
(BSD-2-Clause, unter Raspberry Pi OS als Paket python3-picamera2
vorinstalliert). Am Laptop: Webcam oder ein aufgenommenes Video — so kann
ein Team an der Erkennung arbeiten, während ein anderes am Fahrzeug schraubt.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Capture:
    image: np.ndarray
    captured_s: float
    timestamp_source: str
    exposure_us: int | None = None  # tatsächliche Belichtungszeit laut libcamera-Metadaten
    gain: float | None = None  # analoge × digitale Verstärkung laut Metadaten


def manual_exposure_controls(exposure_us: int) -> dict:
    """Feste Belichtungszeit, die Verstärkung regelt die Automatik weiter.

    Kürzere Belichtung verringert Verwacklung bei Erschütterungen (fm01 19.11.: Automatik
    belichtet bei etwa 90 lx die volle Bilddauer von 33 ms). Braucht libcamera mit
    ExposureTimeMode/AnalogueGainMode (ab 0.5, auf fm01 0.8).
    """
    if type(exposure_us) is not int or not 100 <= exposure_us <= 33000:
        raise ValueError("Belichtungszeit muss zwischen 100 und 33000 µs liegen")
    from libcamera import controls

    if not hasattr(controls, "ExposureTimeModeEnum") or not hasattr(controls, "AnalogueGainModeEnum"):
        raise RuntimeError("libcamera kennt keine feste Belichtungszeit mit automatischer Verstärkung")
    return {"ExposureTimeMode": controls.ExposureTimeModeEnum.Manual, "ExposureTime": exposure_us,
            "AnalogueGainMode": controls.AnalogueGainModeEnum.Auto}


def sensor_time_s(timestamp_ns: int) -> float:
    """libcamera SensorTimestamp: CLOCK_BOOTTIME -> Python CLOCK_MONOTONIC.

    Quelle: libcamera controls/SensorTimestamp. Nach Suspend unterscheiden sich
    diese Uhren; deshalb nicht einfach Nanosekunden durch 1e9 teilen.
    """
    if type(timestamp_ns) is not int or timestamp_ns < 0:
        raise ValueError("Ungültiger SensorTimestamp")
    now_s = time.monotonic()
    return now_s - (time.clock_gettime(time.CLOCK_BOOTTIME) - timestamp_ns / 1e9)


class Camera:
    def __init__(self, source: str | int = "picam", width: int = 640, height: int = 360, loop: bool = False,
                 exposure_us: int | None = None):
        self.width = width
        self.height = height
        self.exposure_us = exposure_us
        if exposure_us is not None and source != "picam":
            raise ValueError("Feste Belichtungszeit gibt es nur für die Pi-Kamera")
        self._loop = loop  # Videodatei am Ende von vorn abspielen (Benchmark, Tests)
        self._picam = None
        self._cap = None
        if source == "picam":
            self._open_picam()
        else:
            import cv2

            src = int(source) if isinstance(source, str) and source.isdigit() else source
            self._cap = cv2.VideoCapture(src)
            if not self._cap.isOpened():
                self._cap.release()
                raise RuntimeError(f"Bildquelle {source!r} lässt sich nicht öffnen")

    def _open_picam(self) -> None:
        from picamera2 import Picamera2

        cam = Picamera2()
        # ANNAHME: Sensormodus 2304×1296 ist beim IMX708 (Camera Module 3)
        # der volle Bildausschnitt mit 2×2-Binning — kleinere Modi schneiden
        # den Bildwinkel ab. Prüfen mit: rpicam-hello --list-cameras
        # "RGB888" liefert in Picamera2 die Byte-Reihenfolge BGR, also genau
        # das, was OpenCV erwartet.
        main = {"size": (self.width, self.height), "format": "RGB888"}
        try:
            try:
                cfg = cam.create_video_configuration(main=main, sensor={"output_size": (2304, 1296)})
            except TypeError:  # ältere Picamera2-Versionen kennen sensor= nicht
                cfg = cam.create_video_configuration(main=main, raw={"size": (2304, 1296)})
            cam.configure(cfg)
            if self.exposure_us is not None:
                cam.set_controls(manual_exposure_controls(self.exposure_us))
            cam.start()
        except Exception:
            cam.close()
            raise
        self._picam = cam

    def read(self) -> np.ndarray | None:
        """Nächstes Bild (BGR, height×width×3) oder None bei Videoende."""
        sample = self.read_timed()
        return None if sample is None else sample.image

    def read_timed(self) -> Capture | None:
        requested_s = time.monotonic()
        if self._picam is not None:
            request = self._picam.capture_request()
            try:
                image = request.make_array("main").copy()
                meta = request.get_metadata()
                stamp = meta.get("SensorTimestamp")
                exposure = meta.get("ExposureTime")
                gain = (meta["AnalogueGain"] * meta.get("DigitalGain", 1.0)) if "AnalogueGain" in meta else None
                if stamp is not None:
                    return Capture(image, sensor_time_s(stamp), "sensor-boottime", exposure, gain)
                # Kein behaupteter Belichtungszeitpunkt: Pufferalter unbekannt.
                return Capture(image, requested_s, "request-start-ohne-sensorzeit", exposure, gain)
            finally:
                request.release()
        import cv2

        ok, frame = self._cap.read()  # type: ignore[union-attr]
        if not ok and self._loop:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # type: ignore[union-attr]
            ok, frame = self._cap.read()  # type: ignore[union-attr]
        if not ok:
            return None
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        return Capture(frame, requested_s, "request-start-pufferalter-unbekannt")

    def close(self) -> None:
        if self._picam is not None:
            cam, self._picam = self._picam, None
            try:
                cam.stop()
            finally:
                cam.close()
        if self._cap is not None:
            self._cap.release()
