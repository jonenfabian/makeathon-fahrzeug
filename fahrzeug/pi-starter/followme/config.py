"""Kleine JSON-Fahrzeugkonfiguration; keine Pico-Sicherheitsgrenzen auf dem Pi."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

from .coral import DEFAULT_MODEL
from .detect import WOERTERBUECHER
from .geometry import CameraModel
from .markers import DEFAULT_MARKERS

DEFAULTS = {
    "fahrzeug": "fm01", "quelle": "picam", "port": "/dev/ttyACM0", "erkenner": "aruco",
    "abstand": 1.5, "marker_id": 300, "marker_groesse": 0.168, "woerterbuch": "apriltag36h11",
    "modell": "models/efficientdet_lite0.tflite", "coral_modell": str(DEFAULT_MODEL),
    "coral_bibliothek": None, "width_px": 640, "height_px": 360, "hfov_deg": 102.0, "vfov_deg": 67.0,
}


def validate(data: dict) -> dict:
    if not isinstance(data, dict) or set(data) - DEFAULTS.keys():
        raise ValueError("Konfiguration enthält unbekannte Schlüssel oder ist kein Objekt")
    values = {**DEFAULTS, **data}
    if not isinstance(values["fahrzeug"], str) or not re.fullmatch(r"fm[0-9]{2}", values["fahrzeug"]):
        raise ValueError("Fahrzeugkennung muss fm01 … fm99 sein")
    if values["erkenner"] not in ("aruco", "farbe", "person", "coral"):
        raise ValueError("Unbekannter Erkenner")
    if not isinstance(values["woerterbuch"], str) or values["woerterbuch"] not in WOERTERBUECHER:
        raise ValueError("Unbekanntes Markerwörterbuch")
    for key in ("abstand", "marker_groesse", "hfov_deg", "vfov_deg"):
        value = values[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} muss endlich und positiv sein")
    if max(values["hfov_deg"], values["vfov_deg"]) >= 180:
        raise ValueError("Bildwinkel müssen kleiner als 180° sein")
    for key in ("width_px", "height_px", "marker_id"):
        value = values[key]
        if type(value) is not int or value < (0 if key == "marker_id" else 1):
            raise ValueError(f"{key}: ungültige ganze Zahl")
    maximum = {"4x4_50": 49, "apriltag36h11": 586}[values["woerterbuch"]]
    if values["marker_id"] > maximum:
        raise ValueError("marker_id liegt außerhalb des Markerwörterbuchs")
    if values["erkenner"] == "aruco" and values["woerterbuch"] == "apriltag36h11":
        spec = DEFAULT_MARKERS.get(values["marker_id"])
        if spec and not math.isclose(values["marker_groesse"], spec.size_m, rel_tol=0, abs_tol=1e-9):
            raise ValueError(f"marker_groesse passt nicht zur registrierten ID {spec.marker_id}")
    for key in ("quelle", "port", "modell", "coral_modell"):
        if not isinstance(values[key], str) or not values[key].strip():
            raise ValueError(f"{key} muss eine nichtleere Zeichenkette sein")
    if values["coral_bibliothek"] is not None and not isinstance(values["coral_bibliothek"], str):
        raise ValueError("coral_bibliothek muss ein Pfad oder null sein")
    return values


def load_config(path: Path | None = None) -> dict:
    original = json.loads(path.read_text(encoding="utf-8")) if path else {}
    values = validate(original)
    # Nur in Dateien gesetzte Pfade sind relativ zur Konfigurationsdatei.
    if path:
        for key in ("modell", "coral_modell", "coral_bibliothek", "quelle"):
            value = original.get(key)
            if value and not Path(value).is_absolute() and not (key == "quelle" and (value == "picam" or value.isdigit())):
                values[key] = str((path.resolve().parent / value).resolve())
    return values


def camera_model(values: dict) -> CameraModel:
    return CameraModel(**{key: values[key] for key in ("width_px", "height_px", "hfov_deg", "vfov_deg")})
