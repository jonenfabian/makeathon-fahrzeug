"""Zielerkennung: Wo ist die Person, der wir folgen?

Drei Erkenner mit derselben Schnittstelle (Bild rein, Liste von Zielen raus):

  ArucoDetector   Zielperson hält einen AprilTag (Schwarz-Weiß-Quadrat
                  mit ID) flach zur Kamera — die Grundstufe.
  ColorDetector   Zielperson trägt eine Warnweste in einer bestimmten Farbe.
                  Einfach, aber lichtempfindlich.
  PersonDetector  Neuronales Netz (MediaPipe, EfficientDet-Lite0, Apache-2.0)
                  erkennt beliebige Personen. Bei mehreren Personen muss man
                  die richtige auswählen — das ist die Kür (TargetSelector).

ArUco: quadratische Marker aus einem festen "Wörterbuch" von Mustern; jedes
Muster hat eine ID. OpenCV (Apache-2.0) findet sie schnell und genau.
Derselbe Erkenner liest auch AprilTags (Wörterbuch tag36h11): Jedes Fahrzeug
trägt vorn und hinten einen 50-mm-Tag am Kameramast (fahrzeug_tag_id).

Bewusst NICHT verwendet: Ultralytics-YOLO (AGPL-3.0 — die Lizenz würde
jeden Code, der es nutzt, mit erfassen).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import numpy as np

from .geometry import CameraModel, person_distance
from .markers import DEFAULT_MARKERS, MarkerObservation


@dataclass(frozen=True)
class Target:
    """Ein erkanntes Ziel im Fahrzeugsystem."""

    bearing_rad: float  # Richtung, positiv = links
    distance_m: float  # Entfernung zur Kamera
    score: float  # Konfidenz 0..1
    box: tuple[float, float, float, float]  # Begrenzungsrahmen (x, y, w, h) in Pixeln
    label: str = "person"


class Detector(Protocol):
    def detect(self, frame_bgr: np.ndarray) -> list[Target]: ...


# ---------------------------------------------------------------------------
# ArUco-Marker und AprilTags
# ---------------------------------------------------------------------------
# Wörterbücher: Name auf der Kommandozeile → Konstante in cv2.aruco
WOERTERBUECHER = {"4x4_50": "DICT_4X4_50", "apriltag36h11": "DICT_APRILTAG_36h11"}

# Bestellte Tags: je Fahrzeug dieselbe gerade ID auf Vorder- und Rückseite.
FAHRZEUG_TAG_M = 0.05
SEITEN = ("vorn", "hinten")


def fahrzeug_tag_id(fahrzeug: int, seite: str) -> int:
    """ID für die reservierten Fahrzeuge 1 bis 10; die Seite ist nicht kodiert."""
    if type(fahrzeug) is not int or not 1 <= fahrzeug <= 10 or seite not in SEITEN:
        raise ValueError(f"Fahrzeug {fahrzeug}, Seite {seite!r}")
    return 2 * fahrzeug


def tag_fahrzeug(tag_id: int) -> int | None:
    """Fahrzeugnummer nur für reservierte gerade IDs; die Seite bleibt unbekannt."""
    if type(tag_id) is not int or tag_id < 2 or tag_id > 20 or tag_id % 2:
        return None
    return tag_id // 2


@dataclass
class ArucoDetector:
    """Findet ArUco-Marker; Ziel ist der Marker mit target_id (None = alle).

    Die Klasse behält ihre alten ArUco-Defaults für explizite Aufrufer.
    Die Fahranwendung konfiguriert tag36h11 ID 300 und 0,168 m.

    Mit woerterbuch="apriltag36h11" und marker_size_m=0.05 erkennt er die Tags
    der anderen Fahrzeuge. ANNAHME (synthetisch geprüft, am Fahrzeug messen):
    Der Tag braucht ≥ 20 px Kantenlänge, mit 640 px Bildbreite und 102°-Optik
    also ≤ 0,6 m Abstand, mit 1280 px ≤ 1,2 m.
    """

    cam: CameraModel
    target_id: int | None = 7
    marker_size_m: float = 0.15
    woerterbuch: str = "4x4_50"
    _detector: object = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        import cv2  # OpenCV, Apache-2.0

        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(getattr(aruco, WOERTERBUECHER[self.woerterbuch]))
        if hasattr(aruco, "ArucoDetector"):  # OpenCV ≥ 4.7
            self._detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        else:  # OpenCV 4.6 (Raspberry Pi OS Bookworm, Paket python3-opencv)
            params = aruco.DetectorParameters_create()
            self._detector = lambda img: aruco.detectMarkers(img, dictionary, parameters=params)

    def _scan(self, frame_bgr: np.ndarray):
        import cv2

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if callable(self._detector):
            return self._detector(gray)[:2]
        return self._detector.detectMarkers(gray)[:2]  # type: ignore[attr-defined]

    @staticmethod
    def _geometry(c):
        pts = c.reshape(4, 2)
        side_px = float(np.mean([np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]))
        cx = float(pts[:, 0].mean())
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        return side_px, cx, (float(x0), float(y0), float(x1 - x0), float(y1 - y0))

    def detect(self, frame_bgr: np.ndarray) -> list[Target]:
        corners, ids = self._scan(frame_bgr)
        if ids is None:
            return []
        targets = []
        for c, marker_id in zip(corners, ids.flatten()):
            if self.target_id is not None and int(marker_id) != self.target_id:
                continue
            side_px, cx, box = self._geometry(c)
            targets.append(Target(self.cam.bearing_rad(cx), self.cam.distance_from_width(side_px, self.marker_size_m),
                                  1.0, box, self._label(int(marker_id))))
        return targets

    def detect_scene(self, frame_bgr: np.ndarray, captured_s: float) -> tuple[list[Target], list[MarkerObservation]]:
        """Personenziel und bekannte Schilder aus derselben Aufnahme getrennt liefern.

        Ausschließlich für den festgelegten Personenmodus: andere Modi verwenden detect().
        Unbekannte IDs und andere Familien gelangen nie in die Beobachtungsliste.
        """
        if (self.woerterbuch, self.target_id, self.marker_size_m) != ("apriltag36h11", 300, .168):
            raise ValueError("Szenenerkennung erfordert den konfigurierten Personenmarker")
        if not math.isfinite(captured_s):
            raise ValueError("Ungültige Aufnahmezeit")
        corners, ids = self._scan(frame_bgr)
        targets: list[Target] = []
        observations: list[MarkerObservation] = []
        if ids is None:
            return targets, observations
        for c, raw_id in zip(corners, ids.flatten()):
            spec = DEFAULT_MARKERS.get(int(raw_id))
            if spec is None:
                continue
            side_px, cx, box = self._geometry(c)
            bearing = self.cam.bearing_rad(cx)
            distance = self.cam.distance_from_width(side_px, spec.size_m)
            observations.append(MarkerObservation(spec.marker_id, spec.role, spec.size_m,
                                                  bearing, distance, box, captured_s))
            if spec.role == "person" and spec.marker_id == self.target_id:
                targets.append(Target(bearing, distance, 1.0, box, self._label(spec.marker_id)))
        return targets, observations

    def _label(self, marker_id: int) -> str:
        if self.woerterbuch == "apriltag36h11" and (fz := tag_fahrzeug(marker_id)):
            return f"fahrzeug{fz}"
        return f"aruco{marker_id}"


# ---------------------------------------------------------------------------
# Farbige Weste
# ---------------------------------------------------------------------------
@dataclass
class ColorDetector:
    """Findet die größte Fläche einer Farbe (HSV-Schwellwerte).

    HSV: Farbton (Hue), Sättigung, Helligkeit — trennt "welche Farbe" von
    "wie hell", deshalb robuster als RGB-Schwellwerte. OpenCV-Hue läuft
    0..179. Voreinstellung: Neon-Gelb/-Grün einer Warnweste.
    ANNAHME: Werte unter Hallenlicht bestimmen (tools/farbe_kalibrieren.py
    ist eine Teamaufgabe).
    """

    cam: CameraModel
    hsv_low: tuple[int, int, int] = (25, 120, 120)
    hsv_high: tuple[int, int, int] = (45, 255, 255)
    vest_width_m: float = 0.45
    min_area_px: int = 400

    def detect(self, frame_bgr: np.ndarray) -> list[Target]:
        import cv2

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.hsv_low), np.array(self.hsv_high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < self.min_area_px:
            return []
        x, y, w, h = cv2.boundingRect(c)
        return [
            Target(
                bearing_rad=self.cam.bearing_rad(x + w / 2.0),
                distance_m=self.cam.distance_from_width(w, self.vest_width_m),
                score=0.8,
                box=(float(x), float(y), float(w), float(h)),
                label="weste",
            )
        ]


# ---------------------------------------------------------------------------
# Personen per neuronalem Netz
# ---------------------------------------------------------------------------
MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/object_detector/"
    "efficientdet_lite0/int8/latest/efficientdet_lite0.tflite"
)


@dataclass
class PersonDetector:
    """Personenerkennung mit MediaPipe Object Detector (EfficientDet-Lite0).

    Modell einmalig herunterladen (siehe docs/06-software.md §4):
        wget -O models/efficientdet_lite0.tflite <MODEL_URL>
    ANNAHME: Auf der CPU des Pi 3B+ reichen 640×360 Pixel nicht für flüssiges
    Folgen; die tatsächliche Bildrate wird am Prototyp gemessen.
    Am Prototyp messen (follow.py zeigt die Bildrate an).
    """

    cam: CameraModel
    model_path: Path = Path("models/efficientdet_lite0.tflite")
    min_score: float = 0.45
    _det: object = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        from mediapipe.tasks import python as mp_tasks  # Apache-2.0
        from mediapipe.tasks.python import vision

        opts = vision.ObjectDetectorOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=str(self.model_path)),
            score_threshold=self.min_score,
            category_allowlist=["person"],
            max_results=5,
        )
        self._det = vision.ObjectDetector.create_from_options(opts)

    def detect(self, frame_bgr: np.ndarray) -> list[Target]:
        import cv2
        import mediapipe as mp

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        result = self._det.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))  # type: ignore[attr-defined]
        targets = []
        for d in result.detections:
            b = d.bounding_box
            box = (float(b.origin_x), float(b.origin_y), float(b.width), float(b.height))
            targets.append(
                Target(
                    bearing_rad=self.cam.bearing_rad(box[0] + box[2] / 2.0),
                    distance_m=person_distance(self.cam, box),
                    score=float(d.categories[0].score),
                    box=box,
                )
            )
        return targets

    def close(self) -> None:
        """Modell freigeben (sonst meldet MediaPipe beim Beenden einen Fehler)."""
        if self._det is not None:
            self._det.close()  # type: ignore[attr-defined]
            self._det = None


# ---------------------------------------------------------------------------
# Auswahl der Zielperson
# ---------------------------------------------------------------------------
@dataclass
class TargetSelector:
    """Wählt aus mehreren Kandidaten die Person, der wir schon folgen.

    Einfachste Verfolgung (Tracking): Nimm den Kandidaten, der der letzten
    bekannten Position am nächsten liegt; ist er weiter als max_jump_m
    entfernt, ist es vermutlich jemand anderes ⇒ verwerfen.
    Ohne Vorgeschichte: den nächstgelegenen Kandidaten nahe der Bildmitte.

    Das ist bewusst schlicht. Kreuzt eine zweite Person, springt die
    Auswahl leicht über — das zu verbessern (z. B. Farbhistogramm der
    Kleidung, Bewegungsvorhersage) ist eine Kür-Aufgabe der Teams.
    """

    max_jump_m: float = 0.8
    _last: Target | None = None

    def select(self, candidates: Sequence[Target]) -> Target | None:
        if not candidates:
            return None
        if self._last is None:
            # Start: nur Kandidaten innerhalb ±20° berücksichtigen.
            central = [t for t in candidates if abs(t.bearing_rad) < math.radians(20)]
            pick = min(central or candidates, key=lambda t: t.distance_m)
        else:
            pick = min(candidates, key=lambda t: _planar_distance(t, self._last))  # type: ignore[arg-type]
            if _planar_distance(pick, self._last) > self.max_jump_m:  # type: ignore[arg-type]
                return None
        self._last = pick
        return pick

    def reset(self) -> None:
        self._last = None


def _planar_distance(a: Target, b: Target) -> float:
    """Abstand zweier Ziele in der Ebene (Polarkoordinaten → kartesisch)."""
    ax, ay = a.distance_m * math.cos(a.bearing_rad), a.distance_m * math.sin(a.bearing_rad)
    bx, by = b.distance_m * math.cos(b.bearing_rad), b.distance_m * math.sin(b.bearing_rad)
    return math.hypot(ax - bx, ay - by)
