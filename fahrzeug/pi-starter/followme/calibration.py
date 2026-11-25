"""Reine Offline-Auswertung von Encoder-, Weg-, Winkel- und Marker-Messungen.

Keine Verbindung zu Kamera/Pico, keine Firmware-/Konfigurationsänderung.
Vorschläge sind keine Hardwarevalidierung oder Sicherheitsfreigabe.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import statistics
from pathlib import Path

DEFAULT_POLICY = {"min_samples": 3, "max_spread_fraction": .03, "max_correction_fraction": .25,
                  "camera_min_hit_rate": .8, "camera_max_error_fraction": .15}
COUNTERS = {"left_start_ticks", "left_end_ticks", "right_start_ticks", "right_end_ticks"}
FIELDS = {"encoder": {"wheel", "turns", "start_ticks", "end_ticks"},
          "straight": {"distance_mm", *COUNTERS}, "turns": {"measured_deg", *COUNTERS},
          "camera": {"distance_m", "estimate_m", "frames", "hits"}}


def number(value, name, *, positive=False, nonzero=False):
    if (type(value) not in (int, float) or abs(value) > 1e9 or not math.isfinite(value)
            or positive and value < 1e-9 or nonzero and abs(value) < 1e-9):
        raise ValueError(f"{name}: endliche Zahl mit gültigem Vorzeichen/Betrag erforderlich")
    return value


def tick_delta(start, end):
    """Vorzeichenbehaftete 32-Bit-Zähler, höchstens ein kurzer Überlauf.

    Die tatsächliche Änderung muss strikt kleiner als 2**31 Zählschritte
    sein. Längere Messungen sind aus zwei Endständen nicht rekonstruierbar.
    """
    if any(type(v) is not int or not -(2**31) <= v < 2**31 for v in (start, end)):
        raise ValueError("Zählerstände müssen vorzeichenbehaftete 32-Bit-Ganzzahlen sein")
    delta = (end - start + 2**31) % 2**32 - 2**31
    if delta == -(2**31):
        raise ValueError("Zählerdifferenz ist mehrdeutig")
    return delta


def validate(data):
    keys = {"format", "kind", "vehicle", "source_note", "camera_note", "reference", "measurements", "policy"}
    if not isinstance(data, dict) or set(data) - keys or type(data.get("format")) is not int or data["format"] != 1:
        raise ValueError("Unbekanntes Kalibrierformat oder unbekannte Felder")
    if data.get("kind") not in ("synthetic", "measured"):
        raise ValueError("kind muss ausdrücklich synthetic oder measured sein")
    if not isinstance(data.get("vehicle"), str) or not re.fullmatch("fm[0-9]{2}", data["vehicle"]):
        raise ValueError("vehicle muss fm01 … fm99 sein")
    for key in ("source_note", "camera_note"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"{key}: Herkunft/Versuchsbedingungen dokumentieren")
    ref = data.get("reference")
    if not isinstance(ref, dict) or set(ref) != {"counts_per_rev", "wheel_diameter_mm", "track_width_mm"}:
        raise ValueError("Verwendete Firmware-Referenzwerte mit Einheiten fehlen")
    for key, value in ref.items():
        number(value, key, positive=True)
    override = data.get("policy", {})
    if not isinstance(override, dict) or set(override) - DEFAULT_POLICY.keys():
        raise ValueError("Unbekannte Auswerteregel")
    policy = {**DEFAULT_POLICY, **override}
    if type(policy["min_samples"]) is not int or not 3 <= policy["min_samples"] <= 100:
        raise ValueError("Mindestens drei Wiederholungen pro Messreihe vorsehen")
    for key in DEFAULT_POLICY.keys() - {"min_samples"}:
        number(policy[key], key, positive=True)
        if policy[key] > 1:
            raise ValueError(f"{key}: Anteil muss höchstens 1 sein")
    measurements = data.get("measurements")
    if not isinstance(measurements, dict) or set(measurements) != FIELDS.keys():
        raise ValueError("measurements braucht encoder, straight, turns und camera (ggf. leere Listen)")
    for section, expected in FIELDS.items():
        rows = measurements[section]
        if not isinstance(rows, list) or len(rows) > 10000:
            raise ValueError(f"{section}: begrenzte Messwertliste erforderlich")
        for index, row in enumerate(rows):
            if not isinstance(row, dict) or set(row) != expected:
                raise ValueError(f"{section}[{index}]: erwartet {sorted(expected)}")
            for key in expected & (COUNTERS | {"start_ticks", "end_ticks"}):
                tick_delta(row[key], row[key])
            if section == "encoder":
                if row["wheel"] not in ("left", "right"):
                    raise ValueError("Encoder-Rad muss left oder right sein")
                number(row["turns"], "Radumdrehungen", nonzero=True)
            elif section == "camera":
                number(row["distance_m"], "Maßbandabstand_m", positive=True)
                if (any(type(row[k]) is not int or not 0 <= row[k] <= 1e9 for k in ("frames", "hits"))
                        or row["hits"] > row["frames"]):
                    raise ValueError("Kamera: 0 <= hits <= frames als ganze Zahlen erforderlich")
                if row["hits"]:
                    number(row["estimate_m"], "Marker-Schätzung_m", positive=True)
                elif row["estimate_m"] is not None:
                    raise ValueError("Ohne Treffer muss estimate_m null sein, nicht null Meter")
            else:
                number(row["distance_mm"] if section == "straight" else row["measured_deg"], section, nonzero=True)
    return policy


def summary(values, policy):
    median = statistics.median(values) if values else None
    spread = (max(values) - min(values)) / median if median is not None and median > 0 else None
    return {"samples": len(values), "median": median, "spread_fraction": spread,
            "repeatable": len(values) >= policy["min_samples"] and spread is not None
                          and spread <= policy["max_spread_fraction"]}


def correction_ok(value, old, policy):
    return value is not None and abs(value / old - 1) <= policy["max_correction_fraction"]


def status(rows, consistent):
    return "missing" if not rows else "consistent" if consistent else "needs_review"


def encoder_section(rows, ref, policy):
    observations = []
    wheels = {}
    for row in rows:
        delta = tick_delta(row["start_ticks"], row["end_ticks"])
        observations.append({**row, "delta_ticks": delta, "counts_per_rev": abs(delta / row["turns"]),
                             "direction_ok": delta * row["turns"] > 0})
    for wheel in ("left", "right"):
        group = [row for row in observations if row["wheel"] == wheel]
        result = summary([row["counts_per_rev"] for row in group], policy)
        result["both_directions"] = any(r["turns"] > 0 for r in group) and any(r["turns"] < 0 for r in group)
        result["direction_ok"] = bool(group) and all(r["direction_ok"] for r in group)
        wheels[wheel] = result
    medians = [r["median"] for r in wheels.values() if r["median"] is not None]
    common = statistics.mean(medians) if len(medians) == 2 else None
    compatible = common is not None and common > 0 and (max(medians) - min(medians)) / common <= policy["max_spread_fraction"]
    checks = {"repeatable": all(r["repeatable"] for r in wheels.values()),
              "both_directions": all(r["both_directions"] for r in wheels.values()),
              "direction_ok": all(r["direction_ok"] for r in wheels.values()), "wheels_compatible": compatible,
              "correction_reasonable": correction_ok(common, ref["counts_per_rev"], policy)}
    consistent = all(checks.values())
    return {"status": status(rows, consistent), "checks": checks, "wheels": wheels, "candidate_counts_per_rev": common,
            "suggestion": common if consistent else None, "observations": observations,
            "notice": "Drehrichtung zuerst korrigieren; Vorwärts-/Rückwärtsmessungen und beide Räder erforderlich"}


def wheel_deltas(row):
    return (tick_delta(row["left_start_ticks"], row["left_end_ticks"]),
            tick_delta(row["right_start_ticks"], row["right_end_ticks"]))


def straight_section(rows, ref, policy, encoder):
    cpr = encoder["suggestion"] or ref["counts_per_rev"]
    observations = []
    for row in rows:
        left, right = wheel_deltas(row)
        distance = row["distance_mm"]
        direction_ok = left * distance > 0 and right * distance > 0
        balanced = bool(left and right) and abs(left - right) / max(abs(left), abs(right)) <= policy["max_spread_fraction"]
        observations.append({**row, "delta_left_ticks": left, "delta_right_ticks": right,
                             "direction_ok": direction_ok, "straight_ticks_balanced": balanced,
                             "reference_odometry_mm": (left + right) / 2 * math.pi * ref["wheel_diameter_mm"] / ref["counts_per_rev"],
                             "diameter_left_mm": abs(distance * cpr / (math.pi * left)) if left else None,
                             "diameter_right_mm": abs(distance * cpr / (math.pi * right)) if right else None})
    wheels = {wheel: summary([r[f"diameter_{wheel}_mm"] for r in observations if r[f"diameter_{wheel}_mm"] is not None], policy)
              for wheel in ("left", "right")}
    medians = [r["median"] for r in wheels.values() if r["median"] is not None]
    common = statistics.mean(medians) if len(medians) == 2 else None
    compatible = common is not None and common > 0 and (max(medians) - min(medians)) / common <= policy["max_spread_fraction"]
    checks = {"encoder_ready": encoder["suggestion"] is not None, "wheels_compatible": compatible,
              "repeatable": all(r["repeatable"] for r in wheels.values()),
              "direction_ok": all(r["direction_ok"] for r in observations),
              "straight_ticks_balanced": bool(rows) and all(r["straight_ticks_balanced"] for r in observations),
              "correction_reasonable": correction_ok(common, ref["wheel_diameter_mm"], policy)}
    consistent = all(checks.values())
    return {"status": status(rows, consistent), "checks": checks, "counts_per_rev_used": cpr,
            "encoder_calibrated": encoder["suggestion"] is not None,
            "wheels": wheels, "candidate_diameter_mm": common, "suggestion": common if consistent else None,
            "observations": observations, "formula": "D = Maßbandweg_mm * Zählschritte_pro_Umdrehung / (pi * DeltaTicks)",
            "notice": "Gerade, schlupffreie Fahrt; Ist-Zähler statt v*t, erst Encoder-Auflösung klären"}


def turn_section(rows, ref, policy, encoder, straight):
    cpr = encoder["suggestion"] or ref["counts_per_rev"]
    diameter = straight["suggestion"] or ref["wheel_diameter_mm"]
    observations = []
    for row in rows:
        left, right = wheel_deltas(row)
        path_difference = (right - left) * math.pi * diameter / cpr
        angle_rad = math.radians(row["measured_deg"])
        centered = left * right < 0 and abs(left + right) / (abs(left) + abs(right)) <= policy["max_spread_fraction"]
        observations.append({**row, "delta_left_ticks": left, "delta_right_ticks": right,
                             "direction_ok": path_difference * angle_rad > 0, "on_spot": centered,
                             "odometry_deg_with_updated_wheel_scale": math.degrees(path_difference / ref["track_width_mm"]),
                             "track_width_mm": abs(path_difference / angle_rad)})
    result = summary([r["track_width_mm"] for r in observations], policy)
    both = any(r["measured_deg"] > 0 for r in rows) and any(r["measured_deg"] < 0 for r in rows)
    checks = {"encoder_ready": encoder["suggestion"] is not None, "wheel_scale_ready": straight["suggestion"] is not None,
              "repeatable": result["repeatable"], "both_directions": both,
              "direction_ok": all(r["direction_ok"] for r in observations),
              "on_spot": bool(rows) and all(r["on_spot"] for r in observations),
              "correction_reasonable": correction_ok(result["median"], ref["track_width_mm"], policy)}
    consistent = all(checks.values())
    return {"status": status(rows, consistent), "checks": checks, "summary": result, "both_directions": both,
            "suggestion": result["median"] if consistent else None, "observations": observations,
            "counts_per_rev_used": cpr, "wheel_diameter_mm_used": diameter,
            "formula": "Spurweite_neu = (Weg_rechts_neu - Weg_links_neu) / Istwinkel_rad",
            "notice": "Unaufgewickelten Gesamtwinkel inkl. voller Umdrehungen messen; erst Wegskala, dann Spurweite"}


def camera_section(rows, policy):
    observations = []
    for row in rows:
        rate = row["hits"] / row["frames"] if row["frames"] else 0
        estimate = row["estimate_m"]
        observations.append({**row, "hit_rate": rate, "usable": estimate is not None and rate >= policy["camera_min_hit_rate"],
                             "error_m": estimate - row["distance_m"] if estimate is not None else None,
                             "relative_error": estimate / row["distance_m"] - 1 if estimate is not None else None,
                             "distance_scale": row["distance_m"] / estimate if estimate is not None else None})
    usable = [r for r in observations if r["usable"]]
    span = len({r["distance_m"] for r in usable}) >= 2
    maximum = max((abs(r["relative_error"]) for r in usable), default=None)
    checks = {"enough_usable": len(usable) >= policy["min_samples"], "all_rows_usable": len(usable) == len(rows),
              "different_distances": span, "error_in_range": maximum is not None and maximum <= policy["camera_max_error_fraction"]}
    consistent = all(checks.values())
    return {"status": status(rows, consistent), "checks": checks, "usable_samples": len(usable), "different_distances": span,
            "max_abs_relative_error": maximum,
            "median_relative_error": statistics.median(r["relative_error"] for r in usable) if usable else None,
            "diagnostic_distance_scale": statistics.median(r["distance_scale"] for r in usable) if usable else None,
            "observations": observations, "suggestion": None,
            "notice": "Nur Marker-Abstandsprüfung. Keine Personenhöhen-, Objektiv- oder Intrinsik-Kalibrierung; keine FOV-Änderung"}


def analyze(data):
    policy = validate(data)
    ref = data["reference"]
    rows = data["measurements"]
    encoder = encoder_section(rows["encoder"], ref, policy)
    straight = straight_section(rows["straight"], ref, policy, encoder)
    turns = turn_section(rows["turns"], ref, policy, encoder, straight)
    camera = camera_section(rows["camera"], policy)
    sections = {"encoder": encoder, "straight": straight, "turns": turns, "camera": camera}
    suggestions = {key: value for key, value in (("ENCODER_COUNTS_PER_REV", encoder["suggestion"]),
                  ("WHEEL_DIAMETER_MM", straight["suggestion"]), ("TRACK_WIDTH_MM", turns["suggestion"])) if value is not None}
    return {"format": 1, "kind": data["kind"], "vehicle": data["vehicle"], "source_note": data["source_note"],
            "camera_note": data["camera_note"], "synthetic": data["kind"] == "synthetic", "reference": dict(ref),
            "policy": policy, "analysis_ok": True, "data_consistent": all(s["status"] == "consistent" for s in sections.values()),
            "hardware_verified": False, "ready_for_drive": False, "parameters_applied": False,
            "suggestions_for_manual_review_only": suggestions, "sections": sections,
            "notice": "Rechenvorschläge, keine Abnahme. Auch kind=measured ist nur eine Herkunftsangabe. "
                      "Vor Übernahme Firmwarequelle prüfen, beide UF2 neu bauen und physische Abnahme wiederholen."}


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Doppelter JSON-Schlüssel: {key}")
        result[key] = value
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("measurement", type=Path)
    ap.add_argument("--ausgabe", type=Path, help="Neue JSON-Datei, wird nicht überschrieben")
    ap.add_argument("--streng", action="store_true", help="Exitcode 1 bei offenen/inkonsistenten Messreihen")
    args = ap.parse_args(argv)
    if args.ausgabe and args.ausgabe.resolve() == args.measurement.resolve():
        ap.error("Messdatei darf niemals Ausgabeziel sein")
    output = args.ausgabe.open("x", encoding="utf-8") if args.ausgabe else None
    report = {"format": 1, "analysis_ok": False, "ready_for_drive": False, "parameters_applied": False}
    try:
        raw = args.measurement.read_bytes()
        report = analyze(json.loads(raw, object_pairs_hook=unique_object))
        report["measurement_sha256"] = hashlib.sha256(raw).hexdigest()
        report["measurement_path"] = str(args.measurement)
    except (OSError, ValueError, TypeError) as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if output:
            with output:
                output.write(text)
    if output:
        print(f"Bericht: {args.ausgabe}")
        if report["analysis_ok"]:
            print(f"Daten: {report['kind']} · konsistent: {report['data_consistent']} · keine Fahrfreigabe")
            for key, section, label in (("ENCODER_COUNTS_PER_REV", "encoder", "Encoder"),
                                        ("WHEEL_DIAMETER_MM", "straight", "Raddurchmesser [mm]"),
                                        ("TRACK_WIDTH_MM", "turns", "Spurweite [mm]")):
                value = report["suggestions_for_manual_review_only"].get(key)
                print(f"{label}: {value:.6g}" if value is not None else f"{label}: kein Vorschlag ({report['sections'][section]['status']})")
            print("Vorschläge nur zur manuellen Prüfung:",
                  ", ".join(f"{k}={v:.6g}" for k, v in report["suggestions_for_manual_review_only"].items()) or "keine")
            bias = report["sections"]["camera"]["median_relative_error"]
            print(f"Kamera-Abstandsfehler (Median): {bias:+.1%}" if bias is not None else "Kamera: keine verwertbaren Abstände")
        else:
            print(report["error"])
    else:
        print(text, end="")
    return int(not report["analysis_ok"] or args.streng and not report.get("data_consistent", False))


if __name__ == "__main__":
    raise SystemExit(main())
