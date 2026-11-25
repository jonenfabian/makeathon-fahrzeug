"""Kurze Sichtlücken durch eine begrenzte Positionsvorhersage überbrücken.

Alle Positionen müssen im selben ruhenden Koordinatensystem liegen. Ein Aufrufer
mit bewegter Kamera rechnet deshalb zuerst seine Eigenbewegung heraus. Dieser
Baustein verlängert keine Kommunikations-/Hardware-Watchdogs und ist keine
Sicherheitsfunktion. Fehlende/alte Kamera- oder Encoderdaten behandelt der Aufrufer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TargetPrediction:
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    observed_at_s: float
    predicted_at_s: float
    age_s: float
    speed_scale: float
    target_id: int | None


class VelocityBridge:
    """Konstante Zielgeschwindigkeit, begrenzt auf zwei Sekunden ohne Sicht.

    In den letzten 0,5 s fällt die freizugebende Robotergeschwindigkeit auf null.
    Ein fester Wandmarker wird mit stationary=True beobachtet: Geschwindigkeit null.
    Bewegte Ziele erhalten eine geglättete Geschwindigkeit aus echten Beobachtungen.
    """

    def __init__(self, horizon_s: float = 2., fade_s: float = .5,
                 velocity_alpha: float = .35, max_target_speed_mps: float = 2.):
        values = (horizon_s, fade_s, velocity_alpha, max_target_speed_mps)
        if not all(math.isfinite(v) for v in values) or not (
            horizon_s > 0 and 0 < fade_s <= horizon_s and 0 < velocity_alpha <= 1
            and max_target_speed_mps > 0
        ):
            raise ValueError("Ungültige Bridge-Parameter")
        self.horizon_s = horizon_s
        self.fade_s = fade_s
        self.velocity_alpha = velocity_alpha
        self.max_target_speed_mps = max_target_speed_mps
        self._last: tuple[float, float, float, int | None] | None = None
        self._velocity = (0., 0.)

    def observe(self, x_m: float, y_m: float, observed_at_s: float,
                target_id: int | None = None, *, stationary: bool = False) -> bool:
        """Echte Beobachtung übernehmen; alte/duplizierte Zeiten erneuern nichts."""
        if not all(math.isfinite(v) for v in (x_m, y_m, observed_at_s)):
            raise ValueError("Ungültige Zielbeobachtung")
        previous = self._last
        if previous is not None and observed_at_s <= previous[2]:
            return False
        if stationary or previous is None or previous[3] != target_id:
            self._velocity = (0., 0.)
        else:
            dt_s = observed_at_s - previous[2]
            vx = (x_m - previous[0]) / dt_s
            vy = (y_m - previous[1]) / dt_s
            if dt_s >= self.horizon_s or math.hypot(vx, vy) > self.max_target_speed_mps:
                # Keine Geschwindigkeit aus einer langen Lücke oder einem Positionssprung ableiten.
                self._velocity = (0., 0.)
            else:
                a = self.velocity_alpha
                self._velocity = (self._velocity[0] + a * (vx - self._velocity[0]),
                                  self._velocity[1] + a * (vy - self._velocity[1]))
        self._last = (x_m, y_m, observed_at_s, target_id)
        return True

    def predict(self, now_s: float) -> TargetPrediction | None:
        if not math.isfinite(now_s):
            raise ValueError("Ungültige Vorhersagezeit")
        if self._last is None:
            return None
        x, y, observed, target_id = self._last
        age = now_s - observed
        if age < 0:
            raise ValueError("Vorhersage liegt vor der Beobachtung")
        if age >= self.horizon_s:
            return None
        vx, vy = self._velocity
        scale = min(1., (self.horizon_s - age) / self.fade_s)
        return TargetPrediction(x + vx * age, y + vy * age, vx, vy,
                                observed, now_s, age, scale, target_id)
