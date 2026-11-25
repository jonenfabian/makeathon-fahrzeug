"""Gemeinsamer Wahrnehmungsschritt für Fahrzeug und Offline-Prüfung.

Keine Sicherheitsfunktion: Verwirft alte Anwendungsdaten. Die unabhängigen
Grenzen, Sensoren und der Watchdog des Pico bleiben unverändert zuständig.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .controller import FollowController, FollowParams
from .detect import Detector, Target, TargetSelector


@dataclass
class Step:
    captured_s: float
    finished_s: float
    targets: list[Target]
    chosen: Target | None
    v_mps: float
    omega_rad_s: float
    state: str
    discarded: str | None = None


class PerceptionLoop:
    def __init__(self, detector: Detector, params: FollowParams | None = None, max_age_s: float = 0.6):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("Bildalter muss endlich und positiv sein")
        self.detector = detector
        self.params = params or FollowParams()
        self.max_age_s = max_age_s
        self._last_capture_s = -math.inf
        self.reset()

    def reset(self) -> None:
        self.selector = TargetSelector()
        self.controller = FollowController(self.params)

    def evaluate(self, targets: list[Target], captured_s: float, finished_s: float) -> Step:
        """Deterministischer Teil; Replay verwendet die aufgezeichneten Zeitpunkte."""
        reason = None
        if not all(math.isfinite(t) for t in (captured_s, finished_s)) or captured_s > finished_s:
            reason = "zeitstempel"
        elif captured_s <= self._last_capture_s:
            reason = "reihenfolge"
        elif finished_s - captured_s > self.max_age_s:
            reason = "bild_veraltet"
        else:
            for target in targets:
                values = (target.bearing_rad, target.distance_m, target.score, *target.box)
                if (not all(math.isfinite(v) for v in values) or target.distance_m <= 0
                        or not 0 <= target.score <= 1 or target.box[2] <= 0 or target.box[3] <= 0):
                    reason = "ungueltiges_ziel"
                    break
        if math.isfinite(captured_s):
            self._last_capture_s = max(self._last_capture_s, captured_s)
        if reason:
            self.reset()
            return Step(captured_s, finished_s, [], None, 0, 0, self.controller.state, reason)
        if self.controller.target_lost_long:
            self.selector.reset()
        chosen = self.selector.select(targets)
        v, w = self.controller.update(chosen, captured_s)
        return Step(captured_s, finished_s, targets, chosen, v, w, self.controller.state)

    def process(self, image, captured_s: float) -> Step:
        try:
            targets = self.detector.detect(image)
            return self.evaluate(targets, captured_s, time.monotonic())
        except Exception:
            self.reset()
            raise
