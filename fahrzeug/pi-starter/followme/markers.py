"""Rollen und Maße der im Follow-Me-Modus verwendeten AprilTags.

Nur Anwendungsdaten; diese Zuordnung steuert weder Motoren noch Schutzgrenzen.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MarkerSpec:
    marker_id: int
    role: str
    size_m: float
    family: str = "apriltag36h11"

    def __post_init__(self) -> None:
        if type(self.marker_id) is not int or not 0 <= self.marker_id < 587:
            raise ValueError("Ungültige tag36h11-ID")
        if self.family != "apriltag36h11":
            raise ValueError("Unbekannte Markerfamilie")
        if self.role not in ("person", "schild", "fahrzeug"):
            raise ValueError("Unbekannte Markerrolle")
        if (isinstance(self.size_m, bool) or not isinstance(self.size_m, (int, float))
                or not math.isfinite(self.size_m) or self.size_m <= 0):
            raise ValueError("Ungültige schwarze Kantenlänge")


def marker_registry(specs: list[MarkerSpec]) -> dict[int, MarkerSpec]:
    """Eindeutiges Register erstellen; doppelte IDs niemals still überschreiben."""
    result = {}
    for spec in specs:
        if spec.marker_id in result:
            raise ValueError(f"Marker-ID {spec.marker_id} doppelt belegt")
        result[spec.marker_id] = spec
    return result


DEFAULT_MARKERS = marker_registry(
    [MarkerSpec(300, "person", .168)]
    + [MarkerSpec(n, "schild", .15) for n in range(100, 109)]
    + [MarkerSpec(n, "fahrzeug", .05) for n in (2, 4, 6, 8)]
)


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    role: str
    size_m: float
    bearing_rad: float
    distance_m: float
    box: tuple[float, float, float, float]
    captured_s: float
    family: str = "apriltag36h11"
