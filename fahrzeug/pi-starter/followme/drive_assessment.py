"""Getrennte dynamische Diagnose virtueller Fahrten, keine Hardwareabnahme.

ANNAHMEN vor Szenarienauswertung: 1,2 m Nahbereich, 0,6 s Reaktionszeit,
40 mm/s Stillstandstoleranz, höchstens 100 ms Abtastlücke. Am Prototyp mit
Kamera-/Bremsmessungen prüfen. Alte Prüfungen und CLI-Exitcodes bleiben erhalten.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from itertools import pairwise
from pathlib import Path

from .protocol import unframe

CAMERA_OFFSET_M = .08  # dieselbe Geometrieannahme wie in der virtuellen Fahrt
MODES = ("beobachten", "halt", "rueckkehr")


@dataclass(frozen=True)
class Criteria:
    near_m: float = 1.2
    min_distance_m: float = .6  # bestehende Simulations-Untergrenze nicht senken
    reaction_s: float = .6
    stopped_mm_s: float = 40.
    max_sample_gap_s: float = .1
    observation_s: float = 2.
    tail_s: float = 1.
    hold_max_m: float = 2.3


CRITERIA = Criteria()


def distance_parts(c0, c1, p0, p1):
    """Exakte symmetrische Aufteilung ΔAbstand: Person, Fahrzeug/Kamerabewegung.

    Mittel beider Reihenfolgen (erst Person/erst Kamera bewegen). Summe ist
    exakt ΔAbstand, aber keine eindeutige physikalische Kausalitätszuweisung.
    Kamera-Drehhebel und beide bewegte Akteure werden nicht wegdefiniert.
    """
    d00, d10 = math.dist(c0, p0), math.dist(c0, p1)
    d01, d11 = math.dist(c1, p0), math.dist(c1, p1)
    return ((d10 - d00 + d11 - d01) / 2, (d01 - d00 + d11 - d10) / 2)


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("Endlicher numerischer Simulationswert erforderlich")
    return value


def assess(source, *, mode="beobachten"):
    """Nur gespeicherte Stichproben auswerten; keine Verbindung, keine Mutation.

    Fehlende Beobachtung: passed=None, niemals als Erfolg auslegen. Modus
    'beobachten' liefert grundsätzlich kein Gesamturteil. CMD sind letzte
    gesendete Befehle je Stichprobe, kein vollständiger TX-/Empfangsnachweis.
    """
    if mode not in MODES:
        raise ValueError("Unbekannter Diagnosemodus")
    rows = source["samples"]
    if not isinstance(rows, list) or len(rows) > 100000:
        raise ValueError("Höchstens 100.000 Simulationsstichproben")
    offset = number(source.get("camera_offset_m", CAMERA_OFFSET_M))
    if not 0 <= offset <= 1:
        raise ValueError("Ungültiger Kameraabstand zur Achse")
    c = CRITERIA
    reasons, parsed = [], []
    for row in rows:
        values = {key: number(row[key]) for key in ("t_s", "x_m", "y_m", "yaw_rad", "target_x_m",
                                                    "target_y_m", "distance_m", "v_l_mm_s", "v_r_mm_s")}
        if values["t_s"] < 0 or (parsed and values["t_s"] <= parsed[-1]["t_s"]):
            raise ValueError("Simulationszeit muss streng aufsteigen")
        camera = (values["x_m"] + offset * math.cos(values["yaw_rad"]),
                  values["y_m"] + offset * math.sin(values["yaw_rad"]))
        person = (values["target_x_m"], values["target_y_m"])
        if abs(math.dist(camera, person) - values["distance_m"]) > 1e-6:
            raise ValueError("Abstand passt nicht zur gespeicherten Kamerageometrie")
        if type(row["seen"]) is not bool:
            raise ValueError("Erkennungsstichprobe muss boolesch sein")
        command = row.get("cmd_line")
        v_cmd = None
        if command is not None:
            fields = unframe(command)
            if fields is None or len(fields) != 4 or fields[0] != "CMD":
                raise ValueError("Ungültiger CMD-Rahmen")
            seq, v_cmd, omega = map(int, fields[1:])
            if not 0 <= seq <= 65535 or max(abs(v_cmd), abs(omega)) > 100000:
                raise ValueError("CMD außerhalb des Protokollbereichs")
        parsed.append({**values, "camera": camera, "person": person, "v_cmd": v_cmd, "seen": row["seen"]})
    if source.get("fault") != "keine":
        reasons.append("normalfahrt_erforderlich")
    if len(parsed) < 2 or parsed[-1]["t_s"] - parsed[0]["t_s"] < c.observation_s - 1e-9:
        reasons.append("beobachtungszeit_zu_kurz")
    if any(b["t_s"] - a["t_s"] > c.max_sample_gap_s for a, b in pairwise(parsed)):
        reasons.append("abtastluecke")
    if not parsed or any(r["v_cmd"] is None for r in parsed):
        reasons.append("cmd_stichprobe_fehlt")
    if sum(r["seen"] for r in parsed) < 3:
        reasons.append("markerbeobachtung_fehlt")

    totals = {"person_delta_m": 0., "vehicle_delta_m": 0., "near_person_closing_m": 0.,
              "near_vehicle_closing_m": 0.}
    for a, b in pairwise(parsed):
        person_delta, vehicle_delta = distance_parts(a["camera"], b["camera"], a["person"], b["person"])
        totals["person_delta_m"] += person_delta
        totals["vehicle_delta_m"] += vehicle_delta
        if min(a["distance_m"], b["distance_m"]) < c.near_m:
            # Ganze Intervalle, die den Nahbereich berühren, nicht passend kürzen.
            totals["near_person_closing_m"] += max(-person_delta, 0.)
            totals["near_vehicle_closing_m"] += max(-vehicle_delta, 0.)

    mature, near_since = [], None
    for row in parsed:
        if row["distance_m"] < c.near_m:
            if near_since is None:
                near_since = row["t_s"]
            if row["t_s"] - near_since >= c.reaction_s - 1e-9:
                mature.append(row)
        else:
            near_since = None
    checks = {
        "mindestabstand": bool(parsed) and all(r["distance_m"] >= c.min_distance_m for r in parsed),
        "keine_rueckwaertssollwerte": bool(parsed) and all(r["v_cmd"] is not None and r["v_cmd"] >= 0 for r in parsed),
    }
    if mode == "rueckkehr":
        if not mature:
            reasons.append("kein_ausreichend_langer_nahbereich")
        if sum(r["seen"] for r in mature) < 3:
            reasons.append("nahbereich_nicht_ausreichend_beobachtet")
        checks["nahe_vorwaerts_beendet"] = bool(mature) and all(
            r["v_cmd"] == 0 and abs(r["v_l_mm_s"] + r["v_r_mm_s"]) / 2 < c.stopped_mm_s for r in mature)
    if mode == "halt":
        tail = [r for r in parsed if r["t_s"] >= parsed[-1]["t_s"] - c.tail_s]
        if not tail or any(math.dist(r["person"], tail[0]["person"]) > 1e-6 for r in tail):
            reasons.append("kein_beobachtetes_ruheziel_im_endfenster")
        if not any(r["seen"] for r in tail):
            reasons.append("marker_im_endfenster_nicht_beobachtet")
        checks["halt_beide_raeder"] = bool(tail) and all(
            r["v_cmd"] == 0 and max(abs(r["v_l_mm_s"]), abs(r["v_r_mm_s"])) < c.stopped_mm_s for r in tail)
        checks["halt_folgeabstand"] = bool(tail) and all(c.near_m <= r["distance_m"] <= c.hold_max_m for r in tail)
    return {"format": 1, "mode": mode, "criteria": asdict(c), "provisional_simulation_criteria": True,
            "scope": "Offline-Stichprobendiagnose; alte Fahrprüfungen bleiben unverändert",
            "complete": not reasons, "incomplete_reasons": reasons,
            "passed": None if reasons or mode == "beobachten" else all(checks.values()),
            "checks": checks, "legacy_checks": dict(source.get("checks", {})),
            "samples": len(parsed), "mature_near_samples": len(mature), "motion": totals,
            "final_distance_m": parsed[-1]["distance_m"] if parsed else None,
            "camera_offset_m": offset, "camera_offset_assumed": "camera_offset_m" not in source,
            "hardware_verified": False, "person_recognition_verified": False, "ready_for_drive": False}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("fahrt", type=Path, help="Gespeicherte virtual_drive-JSON, keine Geräteverbindung")
    ap.add_argument("--modus", choices=(*MODES, "phasen"), default="beobachten")
    ap.add_argument("--ausgabe", type=Path, required=True, help="Neue Ergebnisdatei")
    args = ap.parse_args(argv)
    if args.fahrt.stat().st_size > 64 * 1024 * 1024:
        ap.error("Fahrtdatei größer als 64 MiB")
    source = json.loads(args.fahrt.read_text())
    if args.modus == "phasen":
        from .chaotic_assessment import assess_phases

        result = assess_phases(source)
    else:
        result = assess(source, mode=args.modus)
    with args.ausgabe.open("x", encoding="utf-8") as file:
        json.dump(result, file, indent=2, ensure_ascii=False, allow_nan=False)
        file.write("\n")
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    # Beobachtungsmodus ist nicht qualifizierend, kein stillschweigender Pass.
    return 0 if result["passed"] is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
