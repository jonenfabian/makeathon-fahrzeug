"""Zeit- und Gültigkeitsvertrag für Abstandsdaten, keine Schutzfunktion.

V1-STAT überträgt weder Einzelmesszeit noch vollständigen Sensorstatus. Aus dem
Empfangszeitpunkt darf deshalb keine Messzeit abgeleitet werden.

Ab Protokoll 2 meldet der Pico die Ultraschallsensoren links/rechts in $US
mit Zustand und Alter je Sensor (Quelle "pico"). Messzeit = Empfang − Alter;
die USB-Laufzeit fehlt darin. ANNAHME: Sie liegt unter 5 ms, die Messung ist
also höchstens so viel älter als angegeben. Am Prototyp mit Zeitstempeln prüfen.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .protocol import FLAG_TOF_FAIL, TOF_NO_TARGET, Status, Ultrasonic, UltrasonicReading

SENSOR_NAMES = ("front", "links", "rechts")
# Je Seite kommt alle 2 · US_SLOT_MS = 120 ms ein neuer Ultraschallwert.
# ANNAHME: 0,5 s Altersgrenze lässt drei Ausfälle in Folge zu; am Prototyp prüfen.
SIDE_MAX_AGE_S = 0.5
SAMPLE_STATES = ("distance", "too_close", "no_target", "fault", "unavailable", "legacy_unknown")


@dataclass(frozen=True)
class DistanceSample:
    sensor: str
    state: str
    distance_mm: int | None
    captured_s: float | None
    source: str

    def __post_init__(self) -> None:
        if self.sensor not in SENSOR_NAMES or self.state not in SAMPLE_STATES:
            raise ValueError("Unbekannter Sensor oder Messzustand")
        if self.source not in ("sim", "v1", "none", "pico"):
            raise ValueError("Unbekannte Datenquelle")
        if self.distance_mm is not None and (type(self.distance_mm) is not int or not 0 <= self.distance_mm < 65535):
            raise ValueError("Ungültiger Abstand")
        if self.state == "distance" and (self.distance_mm is None or self.distance_mm == 0):
            raise ValueError("Abstand benötigt einen positiven Messwert")
        if self.state == "too_close" and self.distance_mm != 0:
            raise ValueError("Zu nah benötigt Abstand 0")
        if self.state in ("no_target", "fault", "unavailable") and self.distance_mm is not None:
            raise ValueError("Dieser Zustand darf keinen Abstand behaupten")
        if self.captured_s is not None and (isinstance(self.captured_s, bool)
                or not isinstance(self.captured_s, (float, int)) or not math.isfinite(self.captured_s)
                or self.captured_s < 0):
            raise ValueError("Ungültige Messzeit")
        if self.state == "legacy_unknown" and self.source != "v1":
            raise ValueError("Unqualifizierter Altwert gehört ausschließlich zu V1")
        if self.source == "v1" and self.state not in ("legacy_unknown", "fault", "unavailable"):
            raise ValueError("V1 liefert keinen qualifizierten Einzelzustand")
        if self.source == "none" and (self.state != "unavailable" or self.captured_s is not None):
            raise ValueError("Fehlende Quelle kann nur nicht verfügbare Daten liefern")
        if self.source == "v1" and self.captured_s is not None:
            raise ValueError("V1 liefert keine Einzelmesszeit")
        if self.source == "sim" and self.state != "unavailable" and self.captured_s is None:
            raise ValueError("Simulierte Messung benötigt eine Messzeit")
        if self.source == "pico" and (self.state == "unavailable") != (self.captured_s is None):
            raise ValueError("Pico-Messung braucht genau dann eine Messzeit, wenn sie vorliegt")

    def view(self, now_s: float, max_age_s: float) -> dict:
        """Leseansicht mit pro Sensor berechnetem Alter, keine Fahrfreigabe."""
        if not math.isfinite(now_s) or not math.isfinite(max_age_s) or now_s < 0 or max_age_s <= 0:
            raise ValueError("Ungültiger Abfragezeitpunkt oder Altersgrenze")
        if self.captured_s is not None and self.captured_s > now_s:
            raise ValueError("Messung liegt in der Zukunft")
        age = None if self.captured_s is None else now_s - self.captured_s
        stale = None if age is None else age > max_age_s
        return {"sensor": self.sensor, "state": "stale" if stale else self.state,
                "measurement_state": self.state, "distance_mm": None if stale else self.distance_mm,
                "captured_s": self.captured_s, "age_s": age, "stale": stale, "source": self.source}


def v1_front(status: Status | None) -> DistanceSample:
    """V1 zeigt einen Rohwert, aber kein qualifiziertes Alter oder Sensoreinzelergebnis."""
    if status is None:
        return DistanceSample("front", "unavailable", None, None, "v1")
    if status.flags & FLAG_TOF_FAIL:
        return DistanceSample("front", "fault", None, None, "v1")
    raw = status.tof_mm
    value = raw if type(raw) is int and 0 <= raw < TOF_NO_TARGET else None
    return DistanceSample("front", "legacy_unknown", value, None, "v1")


def v2_side(sensor: str, reading: UltrasonicReading | None, received_s: float | None) -> DistanceSample:
    """Ultraschallwert aus $US (Protokoll 2) mit Messzeit im Zeitbezug von received_s."""
    if sensor not in ("links", "rechts"):
        raise ValueError("Ultraschall gibt es nur links und rechts")
    if reading is None or received_s is None:
        return DistanceSample(sensor, "unavailable", None, None, "none")
    if reading.state == "unavailable" or reading.age_ms is None:
        return DistanceSample(sensor, "unavailable", None, None, "pico")
    captured = max(0.0, received_s - reading.age_ms / 1000.0)
    return DistanceSample(sensor, reading.state, reading.distance_mm, captured, "pico")


def side_views(snapshot: tuple[Ultrasonic | None, float | None], now_s: float,
               max_age_s: float = SIDE_MAX_AGE_S) -> dict[str, dict]:
    """Leseansicht links/rechts aus McuLink.ultrasonic_snapshot(); keine Fahrfreigabe."""
    us, received = snapshot
    if received is not None:
        received = min(received, now_s)  # Meldung kam erst nach dem Abfragezeitpunkt an
    return {name: v2_side(name, None if us is None else getattr(us, attr), received).view(now_s, max_age_s)
            for name, attr in (("links", "left"), ("rechts", "right"))}
