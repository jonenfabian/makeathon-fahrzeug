"""Folgeregler: aus Richtung und Entfernung der Zielperson werden v und ω.

Reine Rechenlogik ohne Kamera und ohne Pico ⇒ mit dem 2D-Simulator
(sim2d.py) und Unit-Tests ohne Hardware prüfbar.

Zustände:
  SUCHE     noch kein Ziel gesehen (Start) — steht still
  FOLGEN    Ziel sichtbar — regelt Abstand und Richtung
  AUSSETZER Ziel kurz weg (Erkennung hat ein paar Bilder verpasst) —
            fährt den letzten Befehl abklingend weiter
  DREHEN    Ziel länger weg — dreht langsam in die Richtung, in der es
            zuletzt war
  VERLOREN  Ziel zu lange weg — steht still, bis es wieder auftaucht

Sicherheit: Dieser Regler ist NICHT die Sicherheitsfunktion. Wird die
Person verwechselt oder der Abstand falsch geschätzt, sperrt der Pico die
Vorwärtsfahrt über den Abstandssensor (40 cm) und hält bei Absturz dieses
Programms über den Watchdog an.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .detect import Target

SEARCH_PATTERNS = ("einseitig", "pendeln")


@dataclass
class FollowParams:
    d_target_m: float = 1.5  # Wunschabstand zur Person
    k_v: float = 1.0  # [1/s] Geschwindigkeit je Meter Abstandsfehler
    k_omega: float = 2.2  # [1/s] Gierrate je rad Richtungsfehler
    v_max_mps: float = 1.2  # oberhalb begrenzt ohnehin der Pico
    omega_max_rad_s: float = 2.0
    slowdown_bearing_rad: float = 0.7  # ab ~40° seitlich: erst drehen, dann fahren
    filter_alpha: float = 0.5  # Glättung der Messwerte (1 = keine)
    dropout_s: float = 0.8  # so lange gilt ein fehlendes Ziel als Aussetzer
    # (0,8 s: bei 3 Bildern/s auf dem Pi 3B sind das zwei verpasste Bilder)
    dropout_decay: float = 0.85  # Abklingfaktor je Schritt im Aussetzer
    search_after_s: float = 2.0  # danach drehen
    search_omega_rad_s: float = 0.8
    search_max_s: float = 8.0  # danach VERLOREN
    search_pattern: str = "einseitig"  # Kandidat erst nach festem A/B-Vergleich übernehmen


class FollowController:
    def __init__(self, params: FollowParams | None = None):
        self.p = params or FollowParams()
        if self.p.search_pattern not in SEARCH_PATTERNS:
            raise ValueError("Unbekannte Suchstrategie")
        self.state = "SUCHE"
        self._d_f: float | None = None  # gefilterte Entfernung
        self._b_f: float | None = None  # gefilterte Richtung
        self._last_seen_s: float | None = None
        self._last_bearing = 0.0
        self._v = 0.0
        self._omega = 0.0

    def update(self, target: Target | None, now_s: float) -> tuple[float, float]:
        """Ein Reglerschritt. Rückgabe (v [m/s], ω [rad/s])."""
        if target is not None:
            self._track(target, now_s)
            self.state = "FOLGEN"
            self._v, self._omega = self.control_law(self._d_f, self._b_f)  # type: ignore[arg-type]
            return self._v, self._omega
        return self._without_target(now_s)

    def _track(self, target: Target, now_s: float) -> None:
        # Exponentielle Glättung: neuer Wert = alter + α · (Messung − alter).
        # Nach einem Verlust wird neu gestartet, statt alte Werte zu mischen.
        a = self.p.filter_alpha
        if self._d_f is None or self.state not in ("FOLGEN", "AUSSETZER"):
            self._d_f, self._b_f = target.distance_m, target.bearing_rad
        else:
            self._d_f += a * (target.distance_m - self._d_f)
            self._b_f += a * (target.bearing_rad - self._b_f)  # type: ignore[operator]
        self._last_seen_s = now_s
        self._last_bearing = self._b_f  # type: ignore[assignment]

    def control_law(self, distance_m: float, bearing_rad: float) -> tuple[float, float]:
        """Regelgesetz: (Entfernung [m], Richtung [rad, links positiv]) → (v [m/s], ω [rad/s]).

        Hinweise für die Teams:
          * Parameter stehen in self.p (FollowParams): d_target_m, k_v, k_omega,
            v_max_mps, omega_max_rad_s, slowdown_bearing_rad.
          * Proportionalregler: Stellgröße = Verstärkung · Abweichung.
          * Nie rückwärts fahren (hinter dem Fahrzeug gibt es keinen Sensor).
          * Steht die Person weit seitlich: erst drehen, dann fahren.
          * Messlatte: tests/test_controller_sim.py.
        """
        p = self.p
        # Proportionalregler auf den Abstandsfehler; nie rückwärts, denn
        # hinter dem Fahrzeug ist kein Sensor (Rückwärtsfahren in Personen
        # wäre gefährlicher als Stehenbleiben).
        v = min(max(p.k_v * (distance_m - p.d_target_m), 0.0), p.v_max_mps)
        # Steht die Person weit seitlich, erst eindrehen: v sinkt linear auf
        # 0 bei slowdown_bearing_rad. Sonst fährt das Fahrzeug weite Bögen
        # und verliert die Person aus dem Bild.
        v *= max(0.0, 1.0 - abs(bearing_rad) / p.slowdown_bearing_rad)
        omega = min(max(p.k_omega * bearing_rad, -p.omega_max_rad_s), p.omega_max_rad_s)
        return v, omega

    def _without_target(self, now_s: float) -> tuple[float, float]:
        if self._last_seen_s is None:
            self.state = "SUCHE"
            return 0.0, 0.0
        gone_s = now_s - self._last_seen_s
        p = self.p
        if gone_s < p.dropout_s:
            # Aussetzer: kurz weiterfahren, aber abklingend — Personen
            # verschwinden selten wirklich in 0,4 s, Erkenner verpassen
            # dagegen oft einzelne Bilder.
            self.state = "AUSSETZER"
            self._v *= p.dropout_decay
            self._omega *= p.dropout_decay
            return self._v, self._omega
        self._v = 0.0
        if gone_s < p.search_after_s:
            self.state = "AUSSETZER"
            self._omega = 0.0
        elif gone_s < p.search_after_s + p.search_max_s:
            self.state = "DREHEN"
            self._omega = math.copysign(p.search_omega_rad_s, self._last_bearing or 1.0)
            if p.search_pattern == "pendeln":
                # Fester Kandidat: erste Strecke 1 s, danach je 2 s zurück/vor.
                # Bei konstanter Ist-Drehrate ideal ±0,8 rad um den Suchbeginn.
                # Zeitbasierter Befehl, KEINE gemessene Encoder-Winkelbegrenzung.
                leg = int((gone_s - p.search_after_s + 1.0) // 2.0)
                if leg % 2:
                    self._omega = -self._omega
        else:
            self.state = "VERLOREN"
            self._omega = 0.0
        return self._v, self._omega

    @property
    def target_lost_long(self) -> bool:
        """True, wenn das Ziel so lange weg ist, dass die Auswahl neu starten sollte."""
        return self.state in ("DREHEN", "VERLOREN")
