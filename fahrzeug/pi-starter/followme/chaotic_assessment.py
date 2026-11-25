"""Vorab festgelegte Phasendiagnose des gesamten virtuellen Personenwegs.

ANNAHMEN, keine Sicherheitsabnahme: 26 s einschließlich 4 s Nachlauf;
0,6 s Nah-/Fernreaktion, 1,2 s Verlust-Stopp, 3 s Wiedererkennung.
Weder Regler noch bisherige Fahrt-Checks werden hier verändert.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from itertools import pairwise

from .drive_assessment import CRITERIA, assess, number
from .protocol import unframe

# Unveränderter bisheriger Zeitplan, jetzt gemeinsame Quelle für Fahrt/Prüfung.
CHAOTIC_PATH = (
    (0.0, 2.0, 0.2), (2.5, 3.0, 0.5), (5.0, 3.2, -0.7),
    (6.2, 3.2, -0.7), (8.5, 2.6, 0.4), (11.0, 3.7, 1.0),
    (13.0, 4.3, -0.1), (15.5, 4.0, -1.0), (16.5, 4.0, -1.0),
    (19.5, 5.3, 0.4), (22.0, 5.8, -0.4),
)
PHASES = (
    ("anfahrt", 0., 2.5), ("seitenwechsel_1", 2.5, 5.), ("pause_1", 5., 6.2),
    ("rueckkehr_1", 6.2, 8.5), ("wieder_entfernen", 8.5, 11.),
    ("seitenwechsel_2", 11., 13.), ("rueckweg_2", 13., 15.5),
    ("pause_2", 15.5, 16.5), ("weitergehen", 16.5, 19.5),
    ("seitenwechsel_3", 19.5, 22.), ("nachlauf", 22., 26.),
)


@dataclass(frozen=True)
class PhaseCriteria:
    startup_s: float = 1.
    detection_fraction: float = .8
    loss_stop_after_s: float = 1.2
    reacquire_s: float = 3.
    min_target_updates: int = 3
    far_m: float = 1.8
    far_bearing_rad: float = .35
    recent_seen_s: float = .2
    far_reaction_s: float = .6
    mask_settle_s: float = .4


POLICY = PhaseCriteria()


def planned_position(t):
    for (t0, x0, y0), (t1, x1, y1) in pairwise(CHAOTIC_PATH):
        if t <= t1:
            f = max(0., (t - t0) / (t1 - t0))
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
    return CHAOTIC_PATH[-1][1:]


def assess_phases(source):
    """Vollständigkeit vor Bewertung; Fristen laufen über Phasengrenzen weiter.

    'seen' allein zählt nicht als Bildrate. Neue Updatezähler unterscheiden
    Bildverarbeitung ohne Ziel von Schleifendurchläufen ohne neues Ergebnis.
    """
    base = assess(source)
    reasons = list(base["incomplete_reasons"])
    if source.get("scenario") != "chaotisch" or source.get("profile") != "pi3b_aruco":
        reasons.append("falscher_szenario_oder_markerprofil_plan")
    if number(source.get("duration_s", 0)) < 26:
        reasons.append("angeforderte_dauer_zu_kurz")
    window = source.get("occlusion_window_s")
    hidden = isinstance(window, (list, tuple)) and list(window) == [9., 12.]
    if window is not None and not hidden:
        reasons.append("unbekannter_verdeckungsplan")
    rows = [r for r in source["samples"] if r["t_s"] <= 26.]
    if not rows or rows[0]["t_s"] > CRITERIA.max_sample_gap_s or rows[-1]["t_s"] < 26 - CRITERIA.max_sample_gap_s:
        reasons.append("plan_nicht_vollstaendig_abgedeckt")
    offset = base["camera_offset_m"]
    mirrored = source.get("mirrored", False)
    if type(mirrored) is not bool:
        raise ValueError("Ungültige Spiegelungsangabe")
    last_seen = near_since = far_since = None
    entries = []
    for row in rows:
        t = row["t_s"]
        expected_x, expected_y = planned_position(t)
        if mirrored:
            expected_y = -expected_y
        if math.dist((row["target_x_m"], row["target_y_m"]), (expected_x, expected_y)) > 1e-6:
            reasons.append("personenweg_passt_nicht_zum_plan")
        updates, targets = row.get("perception_updates"), row.get("target_updates")
        if updates is None or targets is None:
            reasons.append("updatezaehler_fehlen")
            updates = targets = 0
        elif (type(updates) is not int or type(targets) is not int or not 0 <= targets <= updates <= 100
              or row["seen"] != (targets > 0)):
            raise ValueError("Widersprüchliche Wahrnehmungszähler")
        if type(row["flags"]) is not int or not isinstance(row["mcu"], str):
            raise ValueError("Ungültiger Firmwarezustand")
        if targets:
            last_seen = t
        seen_age = t - last_seen if last_seen is not None else None
        if row["distance_m"] < CRITERIA.near_m:
            if near_since is None:
                near_since = t
        else:
            near_since = None
        cx = row["x_m"] + offset * math.cos(row["yaw_rad"])
        cy = row["y_m"] + offset * math.sin(row["yaw_rad"])
        angle = math.atan2(row["target_y_m"] - cy, row["target_x_m"] - cx) - row["yaw_rad"]
        bearing = math.atan2(math.sin(angle), math.cos(angle))
        eligible = (row["distance_m"] >= POLICY.far_m and abs(bearing) <= POLICY.far_bearing_rad
                    and seen_age is not None and seen_age <= POLICY.recent_seen_s)
        if eligible:
            if far_since is None:
                far_since = t
        else:
            far_since = None
        v_cmd = int(unframe(row["cmd_line"])[2]) if row.get("cmd_line") is not None else None
        forward = (row["v_l_mm_s"] + row["v_r_mm_s"]) / 2
        entries.append({**row, "updates": updates, "targets": targets, "v_cmd": v_cmd, "forward": forward,
                        "seen_age": seen_age,
                        "near_due": near_since is not None and t - near_since >= CRITERIA.reaction_s - 1e-9,
                        "loss_due": seen_age is not None and seen_age >= POLICY.loss_stop_after_s,
                        "far_due": far_since is not None and t - far_since >= POLICY.far_reaction_s - 1e-9})

    def stopped(r):
        return r["v_cmd"] == 0 and abs(r["forward"]) < CRITERIA.stopped_mm_s

    phases = []
    for name, start, end in PHASES:
        part = [r for r in entries if start <= r["t_s"] < end or end == 26 and r["t_s"] == end]
        # Anlauf-Abzug vorab festgelegt, keine günstig ausgesuchten Frames.
        observations = [r for r in part if r["t_s"] >= POLICY.startup_s]
        count = sum(r["updates"] for r in observations)
        seen = sum(r["targets"] for r in observations)
        if not count:
            reasons.append(f"keine_bildverarbeitung:{name}")
        near, loss, far = ([r for r in part if r[key]] for key in ("near_due", "loss_due", "far_due"))
        checks = {
            "mindestabstand": bool(part) and all(r["distance_m"] >= CRITERIA.min_distance_m for r in part),
            "keine_rueckwaertssollwerte": bool(part) and all(r["v_cmd"] is not None and r["v_cmd"] >= 0 for r in part),
            "firmware_laeuft_ohne_flags": bool(part) and all(r["mcu"] == "RUN" and r["flags"] == 0 for r in part),
        }
        intentionally_hidden = hidden and start < 12 + POLICY.mask_settle_s and end > 9
        if not intentionally_hidden:
            checks["markerquote"] = bool(count) and seen / count >= POLICY.detection_fraction
        if near:
            checks["nahreaktion"] = all(stopped(r) for r in near)
        if loss:
            checks["verlustreaktion"] = all(stopped(r) for r in loss)
        if far:
            checks["folgt_bei_geeignetem_fernziel"] = all(
                r["v_cmd"] is not None and r["v_cmd"] > 0 and r["forward"] > CRITERIA.stopped_mm_s for r in far)
        # Ganze Abtastintervalle über Phasengrenzen bleiben sichtbar.
        gaps = [b["t_s"] - a["t_s"] for a, b in pairwise(entries) if a["t_s"] < end and b["t_s"] > start]
        phases.append({"name": name, "start_s": start, "end_s": end, "samples": len(part),
                       "processed_updates": count, "target_updates": seen,
                       "detection_fraction": seen / count if count else None,
                       "max_sample_gap_s": max(gaps, default=None),
                       "min_distance_m": min((r["distance_m"] for r in part), default=None),
                       "near_response_applicable": bool(near), "loss_response_applicable": bool(loss),
                       "far_response_applicable": bool(far), "intentional_occlusion_overlap": intentionally_hidden,
                       "checks": checks, "passed": all(checks.values())})
    control_checks = {}
    if hidden:
        exposed = [r for r in entries if 9 <= r["t_s"] < 12 and r["loss_due"]]
        recovered = [r for r in entries if 12 <= r["t_s"] <= 12 + POLICY.reacquire_s]
        control_checks = {
            "verlust_tatsaechlich_beobachtet": bool(exposed),
            "verlust_vorwaerts_beendet": bool(exposed) and all(stopped(r) for r in exposed),
            "wiedererkannt": sum(r["targets"] for r in recovered if r.get("controller") == "FOLGEN") >= POLICY.min_target_updates,
        }
    final = [r for r in entries if 25 <= r["t_s"] <= 26]
    final_checks = {
        "ziel_im_nachlauf_beobachtet": sum(r["targets"] for r in final) >= POLICY.min_target_updates,
        "beide_raeder_und_vorwaertsbefehl_beendet": bool(final) and all(
            r["v_cmd"] == 0 and max(abs(r["v_l_mm_s"]), abs(r["v_r_mm_s"])) < CRITERIA.stopped_mm_s for r in final),
        "ruhe_folgeabstand": bool(final) and all(CRITERIA.near_m <= r["distance_m"] <= CRITERIA.hold_max_m for r in final),
    }
    reasons = sorted(set(reasons))
    passed = all(p["passed"] for p in phases) and all(control_checks.values()) and all(final_checks.values())
    # Einzelne grüne Zeilen dürfen einen unvollständigen Gesamtverlauf nicht kaschieren.
    for phase in phases:
        phase["complete"] = not reasons
        if reasons:
            phase["passed"] = None
    return {"format": 1, "scope": "Vorläufige 26-s-Phasendiagnose, keine Umwertung alter Fahrt-Checks",
            "phase_criteria": asdict(POLICY), "base_criteria": asdict(CRITERIA),
            "complete": not reasons, "incomplete_reasons": reasons, "passed": None if reasons else passed,
            "phases": phases, "loss_control": {"applicable": hidden, "checks": control_checks},
            "final_checks": final_checks, "legacy_checks": dict(source.get("checks", {})),
            "observed_loss_samples": sum(r["loss_due"] for r in entries),
            "hardware_verified": False, "person_recognition_verified": False, "ready_for_drive": False}
