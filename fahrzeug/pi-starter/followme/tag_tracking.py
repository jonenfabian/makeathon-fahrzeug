"""Marker zwischen zwei AprilTag-Erkennungen kurz weiterverfolgen.

Erschütterungen beim Geradeausfahren verwackeln das Bild, dann liest die AprilTag-Dekodierung den
Marker nicht mehr (fm01 19.11.2026: ab 0,15 m/s in 2–16 % der Bilder statt etwa 75 % im Stand).
Der Tracker überbrückt solche Lücken in drei Schritten:

1. **Vorhersage aus den Encodern:** Wie weit sich das Fahrzeug seit der letzten sicheren Erkennung
   gedreht hat und gefahren ist, wissen die Räder. Unter der Annahme, dass das Ziel kurz still steht,
   folgen daraus Richtung, Abstand und Größe im aktuellen Bild.
2. **Bildvergleich:** In einem Suchfenster um die Vorhersage sucht ein normierter Mustervergleich
   (Template Matching) den Bildausschnitt der letzten Erkennung. Unschärfe stört diesen Vergleich
   weit weniger als die Dekodierung der Tag-ID.
3. **Zeitliche Grenze:** Höchstens ``MAX_BRIDGE_S`` nach der letzten bestätigten ID. Danach gilt das
   Ziel als verloren und der Regler entscheidet wie bisher (Aussetzer, Suche).

Nur Wahrnehmung: Motorgrenzen, Watchdogs und Not-Aus bleiben im Pico.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .detect import Target
from .geometry import CameraModel

MAX_BRIDGE_S = 0.8  # wie FollowParams.dropout_s; länger ohne bestätigte ID keine Zielangabe
MIN_SCORE = 0.6  # normierte Korrelation (TM_CCOEFF_NORMED); ANNAHME, mit Aufnahmen vom 19.11. abgestimmt
TARGET_SPEED_MPS = 1.0  # so schnell darf sich das Ziel seitlich bewegen; bestimmt das Suchfenster
MAX_REACH_PX = (120, 60)  # Suchfenster höchstens so weit um die Vorhersage (Rechenzeit auf dem Pi 3B+)
MATCH_SIDE_PX = 24  # größere Muster verkleinert vergleichen; Feinheiten sind bei Unschärfe ohnehin weg
# ANNAHME: Kamera 0,136 m vor der Radachse (labor/wandgeometrie.py, CAD MAST_X); am Prototyp messen.
CAMERA_X_M = 0.136


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    yaw_rad: float


class OdometryHistory:
    """Fahrzeugpose aus Encoderständen, zeitlich interpolierbar.

    Zählerstände kommen mit dem Pico-Status (25 Hz). Ein Sprung rückwärts oder ein unplausibel großer
    Sprung (Neustart des Pico) beginnt einen neuen Verlauf statt einer falschen Bewegung.
    """

    def __init__(self, counts_per_rev: float, wheel_diameter_mm: float, track_mm: float, horizon_s: float = 3.0):
        if not all(math.isfinite(v) and v > 0 for v in (counts_per_rev, wheel_diameter_mm, track_mm, horizon_s)):
            raise ValueError("Fahrzeugparameter müssen positiv sein")
        self.m_per_count = math.pi * wheel_diameter_mm / 1000 / counts_per_rev
        self.track_m = track_mm / 1000
        self.horizon_s = horizon_s
        self._samples: deque[tuple[float, Pose]] = deque()
        self._last: tuple[float, int, int] | None = None
        self._pose = Pose(0., 0., 0.)

    def add(self, time_s: float, ticks_l: int, ticks_r: int) -> None:
        if self._last is not None:
            t0, l0, r0 = self._last
            if time_s <= t0:
                return
            dl, dr = (ticks_l - l0) * self.m_per_count, (ticks_r - r0) * self.m_per_count
            # Mehr als 3 m/s Radgeschwindigkeit ist kein Fahren, sondern ein Zählerneustart.
            if max(abs(dl), abs(dr)) > 3.0 * (time_s - t0) + .05:
                self._samples.clear()
            else:
                ds, dyaw = (dl + dr) / 2, (dr - dl) / self.track_m
                mid = self._pose.yaw_rad + dyaw / 2
                self._pose = Pose(self._pose.x_m + ds * math.cos(mid), self._pose.y_m + ds * math.sin(mid),
                                  self._pose.yaw_rad + dyaw)
        self._last = (time_s, ticks_l, ticks_r)
        self._samples.append((time_s, self._pose))
        while self._samples and self._samples[0][0] < time_s - self.horizon_s:
            self._samples.popleft()

    def pose_at(self, time_s: float) -> Pose | None:
        """Pose zum Zeitpunkt (linear zwischen zwei Statusmeldungen), None außerhalb des Verlaufs."""
        samples = self._samples
        if not samples or time_s < samples[0][0] or time_s > samples[-1][0] + .1:
            return None
        if time_s >= samples[-1][0]:
            return samples[-1][1]
        for (t0, p0), (t1, p1) in zip(samples, list(samples)[1:]):
            if t0 <= time_s <= t1:
                f = (time_s - t0) / (t1 - t0)
                return Pose(p0.x_m + f * (p1.x_m - p0.x_m), p0.y_m + f * (p1.y_m - p0.y_m),
                            p0.yaw_rad + f * (p1.yaw_rad - p0.yaw_rad))
        return None


def predict(anchor: Target, anchor_pose: Pose, pose: Pose, camera_x_m: float = CAMERA_X_M) -> tuple[float, float]:
    """Richtung und Abstand eines still stehenden Ziels nach Eigenbewegung (Kamera vor der Achse)."""
    def camera(p: Pose) -> tuple[float, float]:
        return p.x_m + camera_x_m * math.cos(p.yaw_rad), p.y_m + camera_x_m * math.sin(p.yaw_rad)

    cx, cy = camera(anchor_pose)
    heading = anchor_pose.yaw_rad + anchor.bearing_rad
    tx, ty = cx + anchor.distance_m * math.cos(heading), cy + anchor.distance_m * math.sin(heading)
    nx, ny = camera(pose)
    bearing = math.atan2(ty - ny, tx - nx) - pose.yaw_rad
    return math.atan2(math.sin(bearing), math.cos(bearing)), math.hypot(tx - nx, ty - ny)


class TagTracker:
    """Verfolgt das zuletzt bestätigte Ziel höchstens ``max_bridge_s`` lang weiter."""

    def __init__(self, cam: CameraModel, *, max_bridge_s: float = MAX_BRIDGE_S, min_score: float = MIN_SCORE,
                 target_speed_mps: float = TARGET_SPEED_MPS, camera_x_m: float = CAMERA_X_M):
        self.cam, self.max_bridge_s, self.min_score = cam, max_bridge_s, min_score
        self.target_speed_mps, self.camera_x_m = target_speed_mps, camera_x_m
        self.reset()

    def reset(self) -> None:
        self._anchor: tuple[float, Target, Pose | None, np.ndarray, tuple[float, float, float]] | None = None
        self.last_score: float | None = None

    @staticmethod
    def _prepare(gray: np.ndarray) -> np.ndarray:
        import cv2

        # Leichte Glättung gegen Sensorrauschen bei hoher Verstärkung (8-ms-Belichtung: etwa 16-fach).
        return cv2.GaussianBlur(gray, (3, 3), 0)

    def confirm(self, gray: np.ndarray, captured_s: float, target: Target, pose: Pose | None) -> None:
        """Bestätigte Erkennung: Bildausschnitt und Pose als neuen Anker merken."""
        x, y, w, h = target.box
        side = max(w, h)
        margin = .25 * side  # weißer Rand um den Tag gehört zum Muster
        x0, y0 = int(max(0, x - margin)), int(max(0, y - margin))
        x1, y1 = int(min(gray.shape[1], x + w + margin)), int(min(gray.shape[0], y + h + margin))
        if x1 - x0 < 8 or y1 - y0 < 8:
            self.reset()
            return
        patch = self._prepare(gray)[y0:y1, x0:x1].copy()
        # Lage der Ankermitte im Ausschnitt und Kantenlänge, für Skalierung und Rückrechnung.
        self._anchor = (captured_s, target, pose, patch, (x + w / 2 - x0, y + h / 2 - y0, side))

    def track(self, gray: np.ndarray, captured_s: float, pose: Pose | None) -> Target | None:
        """Ziel ohne neue Erkennung: Vorhersage, Bildvergleich, Zeitgrenze. None = kein sicheres Ziel."""
        import cv2

        self.last_score = None
        if self._anchor is None:
            return None
        anchor_s, anchor, anchor_pose, patch, (ox, oy, side) = self._anchor
        age_s = captured_s - anchor_s
        if not 0 <= age_s <= self.max_bridge_s:
            return None
        if anchor_pose is not None and pose is not None:
            bearing, distance = predict(anchor, anchor_pose, pose, self.camera_x_m)
        else:  # ohne Encoder: Ziel am alten Ort annehmen, Suchfenster trägt die Unsicherheit
            bearing, distance = anchor.bearing_rad, anchor.distance_m
        if not (distance > .15 and abs(bearing) < math.radians(80)):
            return None
        width, height = self.cam.width_px, self.cam.height_px
        anchor_cy = anchor.box[1] + anchor.box[3] / 2
        scale = anchor.distance_m / distance
        if not .5 <= scale <= 2.:
            return None
        px = width / 2 - self.cam.fx * math.tan(bearing)
        py = height / 2 + (anchor_cy - height / 2) * scale  # gleiche Zielhöhe, näher ⇒ weiter vom Bildmittelpunkt
        size = side * scale
        # Großes Muster verkleinert vergleichen: Rechenzeit wächst mit Fenster- mal Musterfläche.
        down = min(1., MATCH_SIDE_PX / max(size, 1.))
        template = cv2.resize(patch, None, fx=scale * down, fy=scale * down, interpolation=cv2.INTER_AREA)
        th, tw = template.shape[:2]
        tw_full, th_full = tw / down, th / down
        # Suchfenster: Ziel darf sich seitlich bewegen, Erschütterung verschiebt das Bild vor allem senkrecht.
        reach_x = min(1.5 * size + self.cam.fx * self.target_speed_mps * age_s / max(distance, .3), MAX_REACH_PX[0])
        reach_y = min(1.5 * size + 25, MAX_REACH_PX[1])
        x0, x1 = int(max(0, px - reach_x - tw_full / 2)), int(min(width, px + reach_x + tw_full / 2))
        y0, y1 = int(max(0, py - reach_y - th_full / 2)), int(min(height, py + reach_y + th_full / 2))
        window = self._prepare(gray)[y0:y1, x0:x1]
        if down < 1.:
            window = cv2.resize(window, None, fx=down, fy=down, interpolation=cv2.INTER_AREA)
        if window.shape[1] <= tw or window.shape[0] <= th:
            return None
        result = cv2.matchTemplate(window, template, cv2.TM_CCOEFF_NORMED)
        _, score, _, loc = cv2.minMaxLoc(result)
        self.last_score = float(score)
        if score < self.min_score:
            return None
        cx = x0 + loc[0] / down + ox * scale
        cy = y0 + loc[1] / down + oy * scale
        box = (cx - size / 2, cy - size / 2, size, size)
        # Richtung aus dem Bild, Abstand aus der Encoder-Vorhersage (bewegt sich das Ziel weg, fährt der
        # Regler dadurch eher zu langsam als zu schnell).
        return Target(self.cam.bearing_rad(cx), distance, float(score), box, anchor.label + "-verfolgt")
