"""Kamerageometrie: Bildposition → Richtung und Entfernung.

Lochkameramodell (pinhole model): Ein Punkt in Richtung α zur optischen
Achse landet im Bild bei x = cx + f · tan(α). f ist die Brennweite in
Pixeln; aus dem horizontalen Bildwinkel HFOV und der Bildbreite W folgt
    f = (W / 2) / tan(HFOV / 2).
Ein Objekt der echten Höhe H in Entfernung d erscheint h Pixel hoch:
    h = f · H / d   ⇒   d = f · H / h.

ANNAHMEN (am Prototyp zu prüfen):
  * Camera Module 3 Wide: 102° horizontal, 67° vertikal (Herstellerangabe).
    Die Weitwinkeloptik verzeichnet zum Rand hin; das Lochkameramodell ist
    dort ungenau (einige Grad). Für das Folgen reicht das — wer genauer
    will, kalibriert mit cv2.calibrateCamera (Kür-Aufgabe).
  * Mittlere Personengröße 1,70 m, Schulterbreite 0,45 m.
Verifizieren: Person in 2,0 m Abstand mittig vor die Kamera stellen;
distance_from_height() muss 2,0 m ± 15 % liefern.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraModel:
    width_px: int = 640
    height_px: int = 360
    hfov_deg: float = 102.0
    vfov_deg: float = 67.0

    @property
    def fx(self) -> float:
        """Horizontale Brennweite in Pixeln."""
        return (self.width_px / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def fy(self) -> float:
        """Vertikale Brennweite in Pixeln."""
        return (self.height_px / 2.0) / math.tan(math.radians(self.vfov_deg) / 2.0)

    def bearing_rad(self, x_px: float) -> float:
        """Richtung zu Bildspalte x. Positiv = links (wie die Gierrate ω).

        Bildspalten wachsen nach rechts, Winkel nach links ⇒ Minuszeichen.
        """
        return -math.atan((x_px - self.width_px / 2.0) / self.fx)

    def distance_from_height(self, h_px: float, real_height_m: float = 1.70) -> float:
        """Entfernung aus der Pixelhöhe eines Objekts bekannter Höhe."""
        return self.fy * real_height_m / max(h_px, 1.0)

    def distance_from_width(self, w_px: float, real_width_m: float) -> float:
        """Entfernung aus der Pixelbreite eines Objekts bekannter Breite."""
        return self.fx * real_width_m / max(w_px, 1.0)


def person_distance(cam: CameraModel, box: tuple[float, float, float, float]) -> float:
    """Entfernung zu einer Person aus ihrem Begrenzungsrahmen (x, y, w, h).

    Sicherheitsrelevante Feinheit: Ist die Person so nah, dass der Rahmen
    oben oder unten am Bildrand abgeschnitten ist, wirkt sie KLEINER und
    damit FERNER, als sie ist — das Fahrzeug würde beschleunigen. Darum wird
    dann zusätzlich die Breite ausgewertet und der kleinere (vorsichtigere)
    Wert genommen. Den letzten Schutz bildet ohnehin der Abstandssensor am Pico.
    """
    _, y, w, h = box
    d_h = cam.distance_from_height(h)
    touches_edge = y <= 2 or (y + h) >= cam.height_px - 2
    if touches_edge:
        return min(d_h, cam.distance_from_width(w, real_width_m=0.45))
    return d_h
