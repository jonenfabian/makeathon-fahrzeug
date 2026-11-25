"""RGB-LEDs (WS2812B) am Pi: zeigen, dass sich Fahrzeuge gerade „unterhalten“.

Zwei Streifen mit je 6 LEDs liegen oben auf dem Oberdeck, links und rechts. Beide hängen über je 470 Ω
direkt am selben Datenausgang (SPI MOSI, GPIO10, Pin 19) und zeigen dasselbe Muster. Ohne Pegelwandler
(Fabian 15.10.2026): Das 3,3-V-High liegt unter V_IH = 0,7 · VDD ≈ 3,6 V der WS2812B, also außerhalb der
Spezifikation. ANNAHME: läuft trotzdem; am Prototyp prüfen (kein Flackern, richtige Farben). Abhilfe: eine
Diode 1N4001 in Reihe zur Streifenversorgung (VDD ≈ 4,4 V ⇒ V_IH ≈ 3,1 V).

Nicht sicherheitsrelevant: Die LEDs zeigen nur etwas an. Ob das Fahrzeug fahren darf, entscheidet allein
der Pico.

WS2812B über SPI: Jedes Datenbit der LED wird ein SPI-Byte. Bei 6,4 MHz dauert ein SPI-Bit 0,156 µs:
  0 → 0b11000000 (0,31 µs high), 1 → 0b11111000 (0,78 µs high), Bitdauer 1,25 µs.
Das liegt in den WS2812B-Zeiten (0: 0,22–0,38 µs, 1: 0,58–1,0 µs high). Weil jedes LED-Bit ein ganzes Byte
ist, stört eine kurze Pause zwischen zwei SPI-Bytes nicht: Sie fällt immer in die Low-Phase.
Farbreihenfolge auf dem Draht: Grün, Rot, Blau. Übernahme, wenn die Leitung ≥ 280 µs low bleibt; das
passiert zwischen zwei Bildern von selbst.

Raspberry Pi 3B+ einrichten (docs/06-software.md): `dtparam=spi=on` und, damit der SPI-Takt nicht mit dem
Core-Takt schwankt, `core_freq=250` in /boot/firmware/config.txt (rpi_ws281x-README, Abschnitt SPI).

Probieren:  python -m followme.leds --fahrzeug 3 --partner 5
"""

from __future__ import annotations

import argparse
import colorsys
import math
import time
from collections.abc import Sequence
from dataclasses import dataclass, field

Farbe = tuple[int, int, int]  # (Rot, Grün, Blau), je 0..255

SPI_HZ = 6_400_000
BIT_0, BIT_1 = 0b11000000, 0b11111000
LED_ANZAHL = 6  # je Streifen; beide Streifen zeigen dasselbe
# 12 LEDs × 60 mA bei Weiß = 0,72 A. Mit 0,4 bleiben es ≈ 0,3 A, und die LEDs blenden nicht.
MAX_HELLIGKEIT = 0.4
AUS: Farbe = (0, 0, 0)


def kodieren(farben: Sequence[Farbe], helligkeit: float = MAX_HELLIGKEIT) -> bytes:
    """Farben → SPI-Bytes (24 Bytes je LED, Reihenfolge G, R, B, höchstes Bit zuerst)."""
    k = min(max(helligkeit, 0.0), 1.0)
    out = bytearray()
    for r, g, b in farben:
        for c in (g, r, b):
            v = min(max(round(c * k), 0), 255)
            out += bytes(BIT_1 if v >> i & 1 else BIT_0 for i in range(7, -1, -1))
    return bytes(out)


def fahrzeug_farbe(nr: int) -> Farbe:
    """Feste Farbe je Fahrzeug 1 … 10 (Farbkreis in Zehntelschritten), z. B. für den Partner im Gespräch."""
    r, g, b = colorsys.hsv_to_rgb(((nr - 1) % 10) / 10.0, 1.0, 1.0)
    return (round(r * 255), round(g * 255), round(b * 255))


def gedimmt(f: Farbe, k: float) -> Farbe:
    return (round(f[0] * k), round(f[1] * k), round(f[2] * k))


def ruhe(eigene: Farbe, t: float, anzahl: int = LED_ANZAHL) -> list[Farbe]:
    """Langsames Atmen in der eigenen Farbe (Periode 3 s): „bin da, rede gerade mit niemandem“."""
    k = 0.15 + 0.35 * (0.5 - 0.5 * math.cos(2 * math.pi * t / 3.0))
    return [gedimmt(eigene, k)] * anzahl


def unterhalten(eigene: Farbe, andere: Farbe, t: float, anzahl: int = LED_ANZAHL) -> list[Farbe]:
    """Zwei Lichtpunkte laufen gegeneinander: die eigene Farbe nach vorn, die des Partners nach hinten.

    So sieht man von außen, dass zwei Fahrzeuge Daten austauschen und mit wem. 8 Schritte je Sekunde.
    """
    schritt = int(t * 8)
    frame = [gedimmt(eigene, 0.08)] * anzahl
    frame[schritt % anzahl] = eigene
    frame[anzahl - 1 - schritt % anzahl] = andere
    return frame


@dataclass
class LedStreifen:
    """Ausgabe an die WS2812B-Streifen über /dev/spidev<bus>.<geraet>.

    spi: für Tests ein Objekt mit writebytes2(); sonst wird spidev (MIT-Lizenz, Paket python3-spidev)
    geöffnet.
    """

    anzahl: int = LED_ANZAHL
    helligkeit: float = MAX_HELLIGKEIT
    bus: int = 0
    geraet: int = 0
    spi: object = field(default=None, repr=False)

    def __post_init__(self) -> None:
        if self.spi is None:
            import spidev

            dev = spidev.SpiDev()
            dev.open(self.bus, self.geraet)
            dev.max_speed_hz = SPI_HZ
            dev.mode = 0
            self.spi = dev

    def zeigen(self, farben: Sequence[Farbe]) -> None:
        if len(farben) != self.anzahl:
            raise ValueError(f"{len(farben)} Farben für {self.anzahl} LEDs")
        self.spi.writebytes2(kodieren(farben, self.helligkeit))  # type: ignore[attr-defined]

    def aus(self) -> None:
        self.zeigen([AUS] * self.anzahl)

    def close(self) -> None:
        self.aus()
        close = getattr(self.spi, "close", None)
        if close:
            close()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="LED-Muster zeigen (Ctrl-C beendet)")
    ap.add_argument("--fahrzeug", type=int, default=1, help="eigene Nummer 1 … 10")
    ap.add_argument("--partner", type=int, default=0, help="Nummer des Gesprächspartners (0 = niemand)")
    ap.add_argument("--dauer", type=float, default=0.0, help="Sekunden (0 = bis Ctrl-C)")
    args = ap.parse_args(argv)
    leds = LedStreifen()
    eigene = fahrzeug_farbe(args.fahrzeug)
    t0 = time.monotonic()
    try:
        while not args.dauer or time.monotonic() - t0 < args.dauer:
            t = time.monotonic() - t0
            if args.partner:
                leds.zeigen(unterhalten(eigene, fahrzeug_farbe(args.partner), t))
            else:
                leds.zeigen(ruhe(eigene, t))
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        pass
    finally:
        leds.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
