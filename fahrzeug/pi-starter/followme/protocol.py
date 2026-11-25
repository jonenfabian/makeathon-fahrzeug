"""Zeilenprotokoll Pi ⇄ Pico (Python-Seite).

Spezifikation: docs/06-software.md §2. Gegenstück in C++:
firmware/followme_mcu/src/proto.cpp und app.cpp. Ändert sich das Format,
müssen beide Seiten UND die Tests beider Seiten angepasst werden — die
Integrationstests (tests/test_link_sim.py) laufen gegen die echte
Firmwarelogik im Host-Simulator und brechen dann sofort.

Format (angelehnt an NMEA 0183, das Satzformat von GPS-Empfängern):
    $TYP,feld1,feld2,...*HH\\n
HH = XOR aller Zeichen zwischen '$' und '*' als zwei Hex-Ziffern.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Bitmaske der Flags in STAT (identisch zu sup::Flag in supervisor.h).
FLAG_WDG = 0x01  # letzter Stopp durch Watchdog
FLAG_TOF_BLOCK = 0x02  # Vorwärtsfahrt durch Abstandssensor gesperrt
FLAG_TOF_FAIL = 0x04  # Abstandssensor liefert keine Werte
FLAG_ESTOP = 0x08  # Motorspannung fehlt (Not-Aus)
FLAG_LOW_BATT = 0x10  # Akku unter Warnschwelle
FLAG_BUMPER = 0x20  # letzter Stopp durch Stoßfänger
FLAG_ARM_REJECTED = 0x40  # letztes ARM abgelehnt
FLAG_RELAY_FAULT = 0x80  # Not-Aus gedrückt, Motorspannung blieb: Relais klebt
FLAG_ENCODER_FAULT = 0x100  # Stellwert ohne Encoderimpulse: Leitung ab oder Rad blockiert

FLAG_NAMES = {
    FLAG_WDG: "WATCHDOG",
    FLAG_TOF_BLOCK: "ABSTAND",
    FLAG_TOF_FAIL: "ABSTANDSSENSOR-FEHLT",
    FLAG_ESTOP: "NOT-AUS",
    FLAG_LOW_BATT: "AKKU-NIEDRIG",
    FLAG_BUMPER: "STOSSFAENGER",
    FLAG_ARM_REJECTED: "ARM-ABGELEHNT",
    FLAG_RELAY_FAULT: "RELAIS-FEHLER",
    FLAG_ENCODER_FAULT: "ENCODER-FEHLER",
}

TOF_NO_TARGET = 0xFFFF

# Unterstützte Protokollversionen (INFO). 2 = zusätzlich $US mit den
# Ultraschallsensoren, STAT/INFO unverändert. Ein Pi mit dieser Software
# arbeitet auch mit einer Firmware der Version 1 (dann ohne Seitenwerte).
PROTO_VERSIONS = (1, 2)

# Zustandszeichen in $US → Messzustand des Sensorvertrags (sensors.py).
US_STATES = {"U": "unavailable", "D": "distance", "C": "too_close", "N": "no_target", "F": "fault"}


def supported_protocol(version: int) -> bool:
    """True, wenn diese Pi-Software die gemeldete Protokollversion versteht."""
    return version in PROTO_VERSIONS


def checksum(body: str) -> int:
    """XOR-Prüfsumme über alle Zeichen von body."""
    cs = 0
    for ch in body.encode("ascii"):
        cs ^= ch
    return cs


def frame(body: str) -> str:
    """Baut die vollständige Zeile '$<body>*HH\\n'."""
    return f"${body}*{checksum(body):02X}\n"


def unframe(line: str) -> list[str] | None:
    """Prüft Rahmen und Prüfsumme; gibt die Felder zurück oder None."""
    line = line.strip()
    if len(line) < 5 or not line.startswith("$") or line[-3] != "*":
        return None
    body = line[1:-3]
    try:
        expected = int(line[-2:], 16)
    except ValueError:
        return None
    if checksum(body) != expected:
        return None
    return body.split(",")


@dataclass(frozen=True)
class Status:
    """Inhalt einer STAT-Meldung des Pico (25 Hz)."""

    t_ms: int  # Pico-Zeit seit Start
    state: str  # "STANDBY" | "RUN" | "ESTOP"
    flags: int  # Bitmaske FLAG_*
    last_seq: int  # zuletzt vom Pico empfangene Folgenummer
    ticks_l: int  # Encoder-Zählerstände
    ticks_r: int
    v_l_mm_s: int  # gemessene Radgeschwindigkeiten
    v_r_mm_s: int
    tof_mm: int  # Abstand nach vorn (65535 = kein Ziel)
    vmot_mv: int  # Motorspannung hinter dem Not-Aus
    rx_errors: int  # vom Pico verworfene Zeilen

    def flag_names(self) -> list[str]:
        """Gesetzte Flags als lesbare Namen (für Statusanzeigen)."""
        return [name for bit, name in FLAG_NAMES.items() if self.flags & bit]


@dataclass(frozen=True)
class Info:
    """Inhalt einer INFO-Meldung (Version und Fahrzeugparameter, alle 2 s)."""

    proto_version: int
    fw_version: str
    v_max_mm_s: int
    omega_max_mrad_s: int
    track_mm: int
    wheel_diameter_mm: int
    counts_per_rev: int


@dataclass(frozen=True)
class UltrasonicReading:
    """Ein Sensor aus $US. Nur Messwert, keine Fahrfreigabe."""

    state: str  # Wert aus US_STATES
    distance_mm: int | None  # nur bei "distance" > 0, bei "too_close" 0
    age_ms: int | None  # Alter beim Senden in Pico-Zeit; None ohne Messung


@dataclass(frozen=True)
class Ultrasonic:
    """Inhalt einer US-Meldung (ab Protokoll 2, mit jeder STAT-Meldung).

    HC-SR04 schräg vorn links und rechts. Der Pico misst, sperrt damit aber
    nichts: Die Abstandssperre bleibt beim Front-ToF.
    """

    t_ms: int  # Pico-Zeit beim Senden
    left: UltrasonicReading
    right: UltrasonicReading


def parse_status(fields: list[str]) -> Status | None:
    """Wandelt die Felder einer STAT-Zeile in ein Status-Objekt."""
    if len(fields) != 12 or fields[0] != "STAT":
        return None
    try:
        return Status(
            t_ms=int(fields[1]),
            state=fields[2],
            flags=int(fields[3], 16),
            last_seq=int(fields[4]),
            ticks_l=int(fields[5]),
            ticks_r=int(fields[6]),
            v_l_mm_s=int(fields[7]),
            v_r_mm_s=int(fields[8]),
            tof_mm=int(fields[9]),
            vmot_mv=int(fields[10]),
            rx_errors=int(fields[11]),
        )
    except ValueError:
        return None


def parse_info(fields: list[str]) -> Info | None:
    """Wandelt die Felder einer INFO-Zeile in ein Info-Objekt."""
    if len(fields) != 8 or fields[0] != "INFO":
        return None
    try:
        return Info(
            proto_version=int(fields[1]),
            fw_version=fields[2],
            v_max_mm_s=int(fields[3]),
            omega_max_mrad_s=int(fields[4]),
            track_mm=int(fields[5]),
            wheel_diameter_mm=int(fields[6]),
            counts_per_rev=int(fields[7]),
        )
    except ValueError:
        return None


def _us_reading(code: str, mm_field: str, age_field: str) -> UltrasonicReading | None:
    state = US_STATES.get(code)
    mm, age = int(mm_field), int(age_field)
    if state is None or not 0 <= mm < TOF_NO_TARGET or not 0 <= age <= 0xFFFFFFFF:
        return None
    if state == "distance":
        return UltrasonicReading(state, mm, age) if mm > 0 else None
    if mm != 0:
        return None  # nur ein gültiger Abstand trägt einen Wert
    if state == "unavailable":
        return UltrasonicReading(state, None, None) if age == 0 else None
    return UltrasonicReading(state, 0 if state == "too_close" else None, age)


def parse_ultrasonic(fields: list[str]) -> Ultrasonic | None:
    """Wandelt die Felder einer US-Zeile in ein Ultrasonic-Objekt."""
    if len(fields) != 8 or fields[0] != "US":
        return None
    try:
        t_ms = int(fields[1])
        left = _us_reading(fields[2], fields[3], fields[4])
        right = _us_reading(fields[5], fields[6], fields[7])
    except ValueError:
        return None
    if left is None or right is None or not 0 <= t_ms <= 0xFFFFFFFF:
        return None
    return Ultrasonic(t_ms, left, right)


def cmd_line(seq: int, v_mps: float, omega_rad_s: float) -> str:
    """CMD-Zeile: Fahrzeuggeschwindigkeit [m/s] und Gierrate [rad/s].

    Auf dem Draht stehen ganze Zahlen in mm/s und mrad/s. Begrenzt wird
    hier NICHT — das macht der Pico (Python ist nie sicherheitskritisch).
    Es wird nur auf den Wertebereich des Parsers (±100000) gekappt, damit
    ein Rechenfehler (z. B. inf) nicht zu einer verworfenen Zeile und damit
    zu einem Watchdog-Stopp führt. NaN (z. B. 0/0 im Regler) wird zu 0.
    """
    return frame(f"CMD,{seq & 0xFFFF},{_milli(v_mps)},{_milli(omega_rad_s)}")


def _milli(x: float) -> int:
    """SI-Wert → ganze Tausendstel, gekappt auf ±100000, NaN → 0."""
    if math.isnan(x):
        return 0
    return int(max(-100000.0, min(100000.0, x * 1000.0)) + (0.5 if x >= 0 else -0.5))


def arm_line(seq: int) -> str:
    return frame(f"ARM,{seq & 0xFFFF}")


def stop_line(seq: int) -> str:
    return frame(f"STOP,{seq & 0xFFFF}")


def sim_line(key: str, value: int) -> str:
    """Nur für den Host-Simulator (firmware/host_sim)."""
    return frame(f"SIM,{key},{int(value)}")
