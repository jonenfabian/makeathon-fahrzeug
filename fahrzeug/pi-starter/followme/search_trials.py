"""Vorab begrenzter Suchvergleich, nur Markerbilder und Firmware-Host-Simulator.

Keine Parameteroptimierung: feste Wege, beide Spiegelrichtungen, eine Ausführung
pro Fall/Strategie. Übernahmeentscheidung getrennt von Sicherheits-/Pi-Freigabe.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from dataclasses import asdict
from itertools import pairwise
from pathlib import Path

from .chaotic_assessment import assess_phases, planned_position
from .controller import SEARCH_PATTERNS, FollowParams
from .drive_assessment import CRITERIA, assess
from .protocol import unframe

SEARCH_PATHS = {
    "suche_stand_gegen": ((0., 2., .2), (3., 2., .2), (6., 2., -1.2), (11., 4.5, -1.2)),
    "suche_stand_gleich": ((0., 2., .2), (3., 2., .2), (6., 2., 1.2), (11., 4.5, 1.2)),
    "suche_fahrt_gegen": ((0., 2., .2), (2.5, 3.25, .5), (5.5, 4.75, -1.2), (11., 7.5, -1.2)),
    "suche_fahrt_gleich": ((0., 2., .2), (2.5, 3.25, .5), (5.5, 4.75, 1.2), (11., 7.5, 1.2)),
}
# Wie §25; flächentreuer Renderer nicht abgenommen (docs/11 §28).
RENDERER = "ganzzahlig"
CASES = [
    {"name": f"{initial}_{direction}_{'spiegel' if mirror else 'original'}",
     "scenario": f"suche_{initial}_{direction}", "mirror": mirror, "initial": initial,
     "window": (3., 6.) if initial == "stand" else (2.5, 5.5), "duration_s": 11.}
    for initial in ("stand", "fahrt") for direction in ("gegen", "gleich") for mirror in (False, True)
] + [
    {"name": f"chaos_verdeckt_{'spiegel' if mirror else 'original'}", "scenario": "chaotisch",
     "mirror": mirror, "initial": "beliebig", "window": (9., 12.), "duration_s": 26.}
    for mirror in (False, True)
] + [{"name": "chaos_normal", "scenario": "chaotisch", "mirror": False, "initial": "beliebig",
      "window": None, "duration_s": 26.}]


def planned_trial_position(path, t):
    for (t0, x0, y0), (t1, x1, y1) in pairwise(path):
        if t <= t1:
            f = max(0., (t - t0) / (t1 - t0))
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
    return path[-1][1:]


def analyze_trial(source, case):
    base = assess(source)
    reasons = list(base["incomplete_reasons"])
    rows = source["samples"]
    duration = case["duration_s"]
    if (not rows or rows[0]["t_s"] > .1 or rows[-1]["t_s"] < duration - .1
            or source.get("duration_s", 0) < duration):
        reasons.append("gesamtabdeckung_fehlt")
    for key, source_key in (("scenario", "scenario"), ("mirror", "mirrored"), ("window", "occlusion_window_s")):
        if key not in case:
            continue
        actual = source.get(source_key)
        same = source_key in source and (actual == case[key] or key == "window" and actual is not None
                                        and case[key] is not None and list(actual) == list(case[key]))
        if not same:
            reasons.append("abweichender_versuchsplan")
    last_seen, exposed, reacquired, commands = None, [], [], []
    mask_loss_observed = False
    initial, window = case["initial"], case["window"]
    pre = [r for r in rows if window is not None and window[0] - .5 <= r["t_s"] < window[0]]
    initial_verified = initial == "beliebig" or bool(pre) and all(
        (max(abs(r["v_l_mm_s"]), abs(r["v_r_mm_s"])) < 40 if initial == "stand"
         else (r["v_l_mm_s"] + r["v_r_mm_s"]) / 2 >= 100) for r in pre)
    for row in rows:
        scenario = case.get("scenario")
        if scenario in SEARCH_PATHS or scenario == "chaotisch":
            expected_x, expected_y = (planned_position(row["t_s"]) if scenario == "chaotisch"
                                      else planned_trial_position(SEARCH_PATHS[scenario], row["t_s"]))
            if case["mirror"]:
                expected_y = -expected_y
            if math.dist((row["target_x_m"], row["target_y_m"]), (expected_x, expected_y)) > 1e-6:
                reasons.append("abweichender_personenweg")
        updates, targets = row.get("perception_updates"), row.get("target_updates")
        if updates is None or targets is None:
            reasons.append("wahrnehmungszaehler_fehlen")
            targets = 0
        elif (type(updates) is not int or type(targets) is not int or not 0 <= targets <= updates <= 100
              or row["seen"] != (targets > 0)):
            raise ValueError("Widersprüchliche Wahrnehmungszähler")
        if targets:
            last_seen = row["t_s"]
        command = int(unframe(row["cmd_line"])[2]) if row.get("cmd_line") is not None else None
        speed = (row["v_l_mm_s"] + row["v_r_mm_s"]) / 2
        commands.append(command)
        if type(row["flags"]) is not int or not isinstance(row["mcu"], str):
            raise ValueError("Ungültiger Firmwarestatus")
        if window is not None:
            if last_seen is not None and row["t_s"] - last_seen >= 1.2:
                exposed.append(command == 0 and abs(speed) < 40)
                mask_loss_observed = mask_loss_observed or window[0] <= row["t_s"] < window[1]
            if row["t_s"] >= window[1] and targets and row["controller"] == "FOLGEN":
                reacquired.extend([row["t_s"] - window[1]] * targets)
    checks = {"ausgangslage": initial_verified,
              "mindestabstand": bool(rows) and min(r["distance_m"] for r in rows) >= CRITERIA.min_distance_m,
              "keine_rueckwaertssollwerte": bool(commands) and all(v is not None and v >= 0 for v in commands),
              "firmware_run_ohne_flags": bool(rows) and all(r["mcu"] == "RUN" and r["flags"] == 0 for r in rows),
              "protokoll_fehlerfrei": source.get("checks", {}).get("protokoll_fehlerfrei", False)}
    if window is not None:
        checks.update(verlust_beobachtet=mask_loss_observed, verlust_vorwaerts_beendet=bool(exposed) and all(exposed),
                      wiedererkannt_in_3s=len(reacquired) >= 3 and reacquired[2] <= 3.)
    phase_result = assess_phases(source) if case.get("scenario") == "chaotisch" else None
    if phase_result is not None:
        if not phase_result["complete"]:
            reasons.append("phasennachweis_unvollstaendig")
        checks["chaos_phasen"] = phase_result["passed"] is True
    return {"complete": not reasons, "incomplete_reasons": sorted(set(reasons)),
            "passed": None if reasons else all(checks.values()), "checks": checks,
            "initial_verified": initial_verified,
            "first_reacquire_delay_s": reacquired[0] if reacquired else None,
            "reacquire_delay_s": reacquired[2] if len(reacquired) >= 3 else None,
            "observed_until_after_occlusion_s": rows[-1]["t_s"] - window[1] if rows and window is not None else None,
            "min_distance_m": min((r["distance_m"] for r in rows), default=None),
            "braking_from_motion_exercised": initial == "fahrt" and initial_verified and bool(exposed) and all(exposed),
            "hardware_verified": False, "ready_for_drive": False}


def adoption_gate(records):
    expected = {(c["name"], s) for c in CASES for s in SEARCH_PATTERNS}
    by_key = {(r["case"], r["strategy"]): r for r in records}
    reasons, improvements = [], []
    if set(by_key) != expected or len(records) != len(expected):
        reasons.append("vollstaendige_eindeutige_paare_erforderlich")
    for case in CASES:
        old, new = (by_key.get((case["name"], s)) for s in SEARCH_PATTERNS)
        if old is None or new is None:
            continue
        if not all(r.get("complete") is True and r.get("initial_verified") is True for r in (old, new)):
            reasons.append(f"daten_oder_ausgangslage:{case['name']}")
        if new.get("passed") is not True:
            reasons.append(f"kandidat_nicht_gruen:{case['name']}")
        if case["window"] is None:
            continue
        before, after = old.get("reacquire_delay_s"), new.get("reacquire_delay_s")
        censored = before is None and old.get("passed") is False
        if censored:
            before = old.get("observed_until_after_occlusion_s")  # tatsächlich letzter Messpunkt, nicht geplantes Ende
        if (type(before) not in (int, float) or type(after) not in (int, float)
                or not math.isfinite(before) or not math.isfinite(after) or before < 0 or after < 0):
            reasons.append(f"zeitvergleich_unvollstaendig:{case['name']}")
            continue
        improvements.append(before - after)
        if after > before + .25:
            prefix = "verschlechterung_nicht_ausgeschlossen" if censored else "wiederaufnahme_schlechter"
            reasons.append(f"{prefix}:{case['name']}")
    if not improvements or max(improvements) < 1.:
        reasons.append("keine_verbesserung_um_mindestens_1s")
    return {"adopt": not reasons, "reasons": reasons, "max_improvement_s": max(improvements, default=None),
            "improvement_is_lower_bound": True, "comparison_tolerance_s": .25,
            "reacquisition_budget_s": 3., "hardware_verified": False}


def save(path, data):
    with path.open("x", encoding="utf-8") as file:
        json.dump(data, file, indent=2, ensure_ascii=False, allow_nan=False)
        file.write("\n")


def main(argv=None):
    from .virtual_drive import run

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ausgabe", type=Path, required=True, help="Neuer Versuchsordner")
    args = ap.parse_args(argv)
    args.ausgabe.mkdir(exist_ok=False)
    save(args.ausgabe / "plan.json", {"cases": CASES, "strategies": SEARCH_PATTERNS, "runs_per_pair": 1,
                                      "candidate_first_leg_s": 1., "candidate_next_legs_s": 2.,
                                      "reacquisition_budget_s": 3., "relative_comparison_tolerance_s": .25,
                                      "minimum_improvement_s": 1., "renderer": RENDERER,
                                      "hardware_verified": False})
    records = []
    for index, case in enumerate(CASES):
        for strategy in SEARCH_PATTERNS[::1 if index % 2 == 0 else -1]:
            name = f"{index:02}-{case['name']}-{strategy}"
            try:
                drive = run(duration_s=case["duration_s"], scenario=case["scenario"], mirror=case["mirror"],
                            occlusion_window_s=case["window"], params=FollowParams(search_pattern=strategy),
                            renderer=RENDERER)
                data = asdict(drive)
                save(args.ausgabe / f"{name}-fahrt.json", data)
                if data["search_pattern"] != strategy or data["renderer"] != RENDERER:
                    raise ValueError("Tatsächliche Suchstrategie/Renderer passt nicht zum Plan")
                result = analyze_trial(data, case)
            except (OSError, RuntimeError, ValueError, TypeError, KeyError, subprocess.SubprocessError) as exc:
                result = {"complete": False, "passed": None, "execution_error": f"{type(exc).__name__}: {exc}"}
            result.update(case=case["name"], strategy=strategy)
            records.append(result)
            save(args.ausgabe / f"{name}-bericht.json", result)
            print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    gate = adoption_gate(records)
    save(args.ausgabe / "report.json", {"records": records, "gate": gate, "reference_changed": False,
                                        "hardware_verified": False, "ready_for_drive": False})
    return 0 if gate["adopt"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
