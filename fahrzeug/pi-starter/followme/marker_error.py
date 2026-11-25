"""Offline-Diagnose: Messfehler kleiner synthetischer ArUco-Marker.

Trennt am gezeichneten Marker Rendererrundung (ideal → gezeichnete Pixel) und
Detektoranteil (gezeichnet → gemessen) und zerlegt im Suchübergang die
Messdifferenz exakt in wahre Zielbewegung und Messfehler. Wahre Werte und
Weltposen sind privilegiertes Simulatorwissen: KEIN Tracker, keine
Kompensation, keine Grenz- oder Fahrfreigabe. Gilt nur für den synthetischen
Renderer und die lokal installierte OpenCV-Version, nicht für echte Kamera,
Verzeichnung, Unschärfe oder NN.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .detect import ArucoDetector, Target, TargetSelector, _planar_distance
from .drive_assessment import CAMERA_OFFSET_M
from .geometry import CameraModel

MARKER_SIZE_M = 0.15
BEARINGS_A_DEG = (0, 10, -10, 20, -20, 30, -30, 40, -40, 45, -45, 48, -48, 50, -50, 52, -52)
# Vorab festgelegt (dist/marker-error-20261022-J4bz02/vorabplan.md), nicht nach Ergebnissen ändern.
PLAN = {
    "format": 1,
    "raster_a": [(round(.6 + .1 * i, 2), b) for i in range(44) for b in BEARINGS_A_DEG],
    "raster_b": [(round(2 + .01 * i, 2), 0) for i in range(201)],
    "raster_c": [(3., sign * .25 * i) for i in range(209) for sign in ((1,) if i == 0 else (1, -1))],
    "gate_rule": "unveraendertes TargetSelector.max_jump_m",
}


def marker_layout(cam: CameraModel, distance_m: float, bearing_rad: float, renderer: str = "ganzzahlig") -> dict:
    """Gezeichnete Geometrie wie in virtual_drive.render_marker, ohne zu zeichnen.

    Mitte im OpenCV-Eckenmaß (Pixelmitte = ganze Zahl). ganzzahlig: Rechteck ab
    x0 mit side_px Pixeln hat dort die Mitte x0 + (side_px − 1) / 2.
    flaechentreu: ungerundete Kante/Mitte; visible_px umfasst alle Pixel mit
    Flächenanteil > 0 (nach 8-bit-Rundung können Randpixel weiß bleiben).
    """
    if distance_m <= 0 or abs(bearing_rad) >= math.pi / 2:
        return {"drawn": False, "reason": "hinter_kamera"}
    side_px = round(cam.fx * MARKER_SIZE_M / distance_m)
    if side_px < 8:
        return {"drawn": False, "reason": "zu_klein", "side_px": side_px}
    if side_px > 4 * cam.width_px:
        return {"drawn": False, "reason": "zu_gross", "side_px": side_px}
    if renderer == "flaechentreu":
        side = cam.fx * MARKER_SIZE_M / distance_m
        center_x = cam.width_px / 2 - cam.fx * math.tan(bearing_rad)
        x0, y0 = center_x - side / 2, cam.height_px / 2 - side / 2  # Kante, nicht Pixelmitte
        # Erstes Pixel mit Flächenanteil > 0: rechte Pixelkante px + 0,5 liegt rechts von x0.
        left, top = max(0, math.floor(x0 + .5)), max(0, math.floor(y0 + .5))
        right = min(cam.width_px, math.ceil(x0 + side + .5))
        bottom = min(cam.height_px, math.ceil(y0 + side + .5))
        if left >= right or top >= bottom:
            return {"drawn": False, "reason": "ausserhalb_bild", "side_px": side}
        return {"drawn": True, "reason": None, "side_px": side, "x0_px": x0, "y0_px": y0,
                "visible_px": [left, top, right, bottom],
                "clipped": x0 < -.5 or y0 < -.5 or x0 + side > cam.width_px - .5 or y0 + side > cam.height_px - .5,
                "center_px": center_x}
    center_px = round(cam.width_px / 2 - cam.fx * math.tan(bearing_rad))
    x0, y0 = center_px - side_px // 2, (cam.height_px - side_px) // 2
    left, top = max(0, x0), max(0, y0)
    right, bottom = min(cam.width_px, x0 + side_px), min(cam.height_px, y0 + side_px)
    if left >= right or top >= bottom:
        return {"drawn": False, "reason": "ausserhalb_bild", "side_px": side_px}
    return {"drawn": True, "reason": None, "side_px": side_px, "x0_px": x0, "y0_px": y0,
            "visible_px": [left, top, right, bottom],
            "clipped": (left, top, right, bottom) != (x0, y0, x0 + side_px, y0 + side_px),
            "center_px": x0 + (side_px - 1) / 2}


def _point(distance_m: float, bearing_rad: float) -> np.ndarray:
    return np.array([distance_m * math.cos(bearing_rad), distance_m * math.sin(bearing_rad)])


def _layout_matches(painted, layout: dict, renderer: str) -> bool:
    expected = layout.get("visible_px")
    if renderer == "ganzzahlig" or painted is None or expected is None:
        return painted == expected
    # Flächentreu: gemalte Pixel innerhalb der Hülle, höchstens ein weißes Randpixel je Seite.
    return all(abs(a - b) <= 1 for a, b in zip(painted, expected)) and (
        painted[0] >= expected[0] and painted[1] >= expected[1]
        and painted[2] <= expected[2] and painted[3] <= expected[3])


def measure(cam: CameraModel, detector, distance_m: float, bearing_rad: float,
            renderer: str = "ganzzahlig") -> dict:
    """Ein Marker: wahr → gezeichnet → gemessen, Fehler im Kamerarahmen."""
    from .virtual_drive import render_marker

    layout = marker_layout(cam, distance_m, bearing_rad, renderer)
    image = render_marker(cam, distance_m, bearing_rad, renderer=renderer)
    dark = np.argwhere(image.min(axis=2) < 255)
    painted = None if not len(dark) else [int(dark[:, 1].min()), int(dark[:, 0].min()),
                                          int(dark[:, 1].max()) + 1, int(dark[:, 0].max()) + 1]
    raw = detector.detect(image)
    row = {"distance_m": distance_m, "bearing_rad": bearing_rad, "layout": layout, "raw_targets": len(raw),
           "layout_matches_image": _layout_matches(painted, layout, renderer)}
    if not layout["drawn"]:
        row["status"] = "nicht_gezeichnet_erkannt" if raw else "nicht_gezeichnet"
        return row
    clipped = "abgeschnitten_" if layout["clipped"] else ""
    if len(raw) != 1:
        row["status"] = f"{clipped}{'nicht_erkannt' if not raw else 'mehrdeutig'}"
        return row
    hit = raw[0]
    drawn_d = cam.fx * MARKER_SIZE_M / layout["side_px"]
    drawn_b = cam.bearing_rad(layout["center_px"])
    true_p, drawn_p, hit_p = (_point(distance_m, bearing_rad), _point(drawn_d, drawn_b),
                              _point(hit.distance_m, hit.bearing_rad))
    row.update({
        "status": f"{clipped}erkannt", "measured_distance_m": hit.distance_m,
        "measured_bearing_rad": hit.bearing_rad,
        "render_distance_error_m": drawn_d - distance_m, "detector_distance_error_m": hit.distance_m - drawn_d,
        "render_bearing_error_rad": drawn_b - bearing_rad, "detector_bearing_error_rad": hit.bearing_rad - drawn_b,
        "detector_side_error_px": cam.fx * MARKER_SIZE_M / hit.distance_m - layout["side_px"],
        "detector_center_error_px": cam.width_px / 2 - cam.fx * math.tan(hit.bearing_rad) - layout["center_px"],
        "render_position_error_m": float(np.linalg.norm(drawn_p - true_p)),
        "detector_position_error_m": float(np.linalg.norm(hit_p - drawn_p)),
        "position_error_m": float(np.linalg.norm(hit_p - true_p)),
        "lateral_error_m": distance_m * (hit.bearing_rad - bearing_rad),
        # Vorab festgelegt: auffällig, wenn betraglich mehr als eine Pixelkante.
        "detector_distance_conspicuous": abs(hit.distance_m - drawn_d) > drawn_d / layout["side_px"],
    })
    return row


def summarize(rows: list[dict]) -> dict:
    """Zählung je Status und Fehlerbeträge der vollständig gezeichneten Treffer."""
    status: dict[str, int] = {}
    for row in rows:
        status[row["status"]] = status.get(row["status"], 0) + 1
    hits = [row for row in rows if row["status"] == "erkannt"]

    def worst(key):
        return max((abs(row[key]) for row in hits), default=None)

    def mean(key):
        return sum(row[key] for row in hits) / len(hits) if hits else None

    bands = {}
    for row in hits:
        band = bands.setdefault(str(round(row["layout"]["side_px"])), {"n": 0, "max_abs_detector_side_error_px": 0.})
        band["n"] += 1
        band["max_abs_detector_side_error_px"] = max(band["max_abs_detector_side_error_px"],
                                                     abs(row["detector_side_error_px"]))
    return {"points": len(rows), "status": dict(sorted(status.items())), "detection_holes": detection_holes(rows),
            "layout_mismatches": sum(not row["layout_matches_image"] for row in rows),
            "detector_distance_conspicuous": sum(row["detector_distance_conspicuous"] for row in hits),
            **{f"max_abs_{key}": worst(key) for key in (
                "render_distance_error_m", "detector_distance_error_m", "render_bearing_error_rad",
                "detector_bearing_error_rad", "detector_side_error_px", "detector_center_error_px",
                "position_error_m", "lateral_error_m")},
            "mean_detector_side_error_px": mean("detector_side_error_px"),
            "mean_detector_center_error_px": mean("detector_center_error_px"),
            "side_px_bands": dict(sorted(bands.items(), key=lambda item: int(item[0])))}


def detection_holes(rows: list[dict], margin_px: int = 3) -> list[list[float]]:
    """Je Peilung: nicht erkannte Entfernung zwischen zwei erkannten (Randabstand ≥ margin_px)."""
    width = CameraModel().width_px
    columns: dict[float, list[dict]] = {}
    for row in rows:
        visible = row["layout"].get("visible_px")
        if (row["layout"]["drawn"] and not row["layout"]["clipped"] and visible[0] >= margin_px
                and visible[2] <= width - margin_px):
            columns.setdefault(row["bearing_rad"], []).append(row)
    holes = []
    for bearing, column in sorted(columns.items()):
        column.sort(key=lambda row: row["distance_m"])
        found = [i for i, row in enumerate(column) if row["status"] == "erkannt"]
        holes += [[row["distance_m"], bearing] for row in column[found[0]:found[-1] + 1]
                  if row["status"] != "erkannt"] if found else []
    return holes


def sweep(detector=None, renderer: str = "ganzzahlig") -> dict:
    cam = CameraModel()
    detector = detector if detector is not None else ArucoDetector(cam)
    result = {}
    for name in ("raster_a", "raster_b", "raster_c"):
        rows = [measure(cam, detector, d, math.radians(b), renderer) for d, b in PLAN[name]]
        result[name] = {"summary": summarize(rows), "rows": rows}
    return result


def _camera(row: dict) -> np.ndarray:
    yaw = row["yaw_rad"]
    return np.array([row["x_m"] + CAMERA_OFFSET_M * math.cos(yaw), row["y_m"] + CAMERA_OFFSET_M * math.sin(yaw)])


def _world(row: dict, distance_m: float, bearing_rad: float) -> np.ndarray:
    return _camera(row) + _point(distance_m, row["yaw_rad"] + bearing_rad)


def classify(gate_m: float, true_world_m: float, measured_world_m: float, true_relative_m: float) -> list[str]:
    """Vorab festgelegte Einordnung eines Abstands gegen das unveränderte Gate."""
    labels = []
    if true_world_m > gate_m:
        labels.append("zielbewegung_allein_ausreichend")
    if true_world_m <= gate_m < measured_world_m:
        labels.append("messfehler_entscheidend_im_weltmass")
    if true_relative_m > gate_m >= true_world_m:
        labels.append("kamerabewegung_entscheidend")
    return labels


def analyze_case(source: dict, detector=None) -> dict:
    """Festes Fenster: Verdeckungsende bis einschließlich dritter Auswahl danach."""
    from .search_transition import reconstruct
    from .virtual_drive import Pose

    cam = CameraModel()
    detector = detector if detector is not None else ArucoDetector(cam)
    window = source.get("occlusion_window_s")
    if window is None:
        raise ValueError("Gegenfall braucht eine Marker-Bildpause")
    replay = reconstruct(source, detector=detector)
    renderer = replay["generator_assumptions"]["renderer"]
    by_time = {row["t_s"]: row for row in source["samples"]}
    reasons = list(replay["incomplete_reasons"])
    events, reference, selected_after = [], None, 0
    for event in replay["events"]:
        if event["selector_reset"]:
            reference = None
        row = by_time[event["capture_s"]]
        distance, bearing = Pose(row["x_m"], row["y_m"], row["yaw_rad"]).target(row["target_x_m"],
                                                                               row["target_y_m"])
        in_window = event["processed_s"] >= window[1] and selected_after < 3
        current = None
        if in_window:
            hidden = window[0] <= row["t_s"] < window[1]
            current = {"distance_m": distance, "bearing_rad": bearing} if hidden else measure(
                cam, detector, distance, bearing, renderer)
            if hidden:
                current["status"] = "verdeckt"
            if current.get("raw_targets", 0) != event["raw_targets"]:
                reasons.append("messung_weicht_von_rekonstruktion_ab")
            entry = {"processed_s": event["processed_s"], "capture_s": event["capture_s"],
                     "kind": event["kind"], "gate_m": event["gate_m"], "selector_reset": event["selector_reset"],
                     "observation": current, "reference_capture_s": reference and reference["row"]["t_s"]}
            if (reference is not None and "measured_distance_m" in current
                    and "measured_distance_m" in reference["observation"]):
                entry["decomposition"] = _decompose(reference, row, current, event)
            elif event["raw_targets"] and reference is not None:
                reasons.append("bezug_oder_treffer_nicht_zerlegbar")
            events.append(entry)
            selected_after += event["kind"] == "gewaehlt"
        if event["kind"] == "gewaehlt":
            reference = {"row": row, "observation": current or measure(cam, detector, distance, bearing, renderer)}
    if selected_after < 3:
        reasons.append("dritte_auswahl_fehlt")
    decomposed = [event["decomposition"] for event in events if "decomposition" in event]
    labels: dict[str, int] = {}
    for item in decomposed:
        for label in item["classification"]:
            labels[label] = labels.get(label, 0) + 1
    misses: dict[str, int] = {}
    for event in events:
        if event["kind"] == "kein_rohtreffer":
            observation = event["observation"]
            reason = observation.get("layout", {}).get("reason") or observation["status"]
            misses[reason] = misses.get(reason, 0) + 1
    return {"format": 1, "scope": "Festes Gegenfall-Fenster, rekonstruierte synthetische Frames", "renderer": renderer,
            "complete": not reasons and replay["complete"], "incomplete_reasons": sorted(set(reasons)),
            "passed": None, "window_events": len(events), "decomposed_events": len(decomposed),
            "classification_counts": dict(sorted(labels.items())), "no_raw_reasons": dict(sorted(misses.items())),
            "max_identity_residual_m": max((item["identity_residual_m"] for item in decomposed), default=None),
            "events": events, "privileged_simulator_truth": True, "tracker_implemented": False,
            "target_selector_changed": False, "closed_loop_runs_added": 0, "hardware_verified": False,
            "ready_for_drive": False}


def _decompose(reference: dict, row: dict, current: dict, event: dict) -> dict:
    """Exakt: Messdifferenz = wahre Zielbewegung + (Messfehler neu − Messfehler Bezug)."""
    ref_row, ref = reference["row"], reference["observation"]

    def world(r, obs, key):
        return _world(r, obs[f"{key}distance_m"], obs[f"{key}bearing_rad"])

    def drawn(r, obs):
        cam = CameraModel()
        layout = obs["layout"]
        return _world(r, cam.fx * MARKER_SIZE_M / layout["side_px"], cam.bearing_rad(layout["center_px"]))

    true_shift = world(row, current, "") - world(ref_row, ref, "")
    measured_shift = world(row, current, "measured_") - world(ref_row, ref, "measured_")
    error_new = world(row, current, "measured_") - world(row, current, "")
    error_ref = world(ref_row, ref, "measured_") - world(ref_row, ref, "")
    render_diff = (drawn(row, current) - world(row, current, "")) - (drawn(ref_row, ref) - world(ref_row, ref, ""))
    error_diff = error_new - error_ref

    def target(obs, key):
        return Target(obs[f"{key}bearing_rad"], obs[f"{key}distance_m"], 1., (0, 0, 0, 0))

    measured_relative = _planar_distance(target(current, "measured_"), target(ref, "measured_"))
    true_relative = _planar_distance(target(current, ""), target(ref, ""))
    gate = event["gate_m"]
    if gate != TargetSelector().max_jump_m:
        raise ValueError("Gate weicht vom unveränderten Selektor ab")
    measured_world, true_world = float(np.linalg.norm(measured_shift)), float(np.linalg.norm(true_shift))
    return {
        "gate_distances_m": {"gemessen_kamerarelativ": measured_relative, "wahr_kamerarelativ": true_relative,
                             "gemessen_welt": measured_world, "wahr_welt": true_world},
        "camera_relative_matches_replay": math.isclose(measured_relative,
                                                       event["geometry"]["camera_relative_jump_m"], abs_tol=1e-9),
        "true_target_shift_vec_m": true_shift.tolist(), "measurement_error_diff_vec_m": error_diff.tolist(),
        "measurement_error_diff_m": float(np.linalg.norm(error_diff)),
        "render_part_of_error_diff_m": float(np.linalg.norm(render_diff)),
        "detector_part_of_error_diff_m": float(np.linalg.norm(error_diff - render_diff)),
        "error_new_m": float(np.linalg.norm(error_new)), "error_reference_m": float(np.linalg.norm(error_ref)),
        "identity_residual_m": float(np.linalg.norm(measured_shift - true_shift - error_diff)),
        "classification": classify(gate, true_world, measured_world, true_relative),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ausgabe", type=Path, required=True, help="Neuer Ordner, nie überschreiben")
    ap.add_argument("--fahrt", type=Path, help="Gespeicherte virtual_drive-JSON mit Bildpause (Gegenfall)")
    ap.add_argument("--renderer", choices=("ganzzahlig", "flaechentreu"), default="ganzzahlig",
                    help="Renderer der Raster; der Gegenfall nutzt den in der Fahrt gespeicherten")
    args = ap.parse_args(argv)
    args.ausgabe.mkdir(parents=False, exist_ok=False)

    def write(name, data):
        with (args.ausgabe / name).open("x", encoding="utf-8") as file:
            json.dump(data, file, indent=2, ensure_ascii=False, allow_nan=False)
            file.write("\n")

    import cv2

    write("plan.json", {**PLAN, "opencv": cv2.__version__, "renderer": args.renderer})  # vor jeder Messung
    raster = sweep(renderer=args.renderer)
    write("raster.json", raster)
    summary = {name: raster[name]["summary"] for name in raster}
    complete = all(item["layout_mismatches"] == 0 for item in summary.values())
    if args.fahrt:
        if args.fahrt.stat().st_size > 64 * 1024 * 1024:
            ap.error("Fahrtdatei größer als 64 MiB")
        raw = args.fahrt.read_bytes()
        case = analyze_case(json.loads(raw))
        case["source_sha256"] = hashlib.sha256(raw).hexdigest()
        write("gegenfall.json", case)
        summary["gegenfall"] = {key: value for key, value in case.items() if key != "events"}
        complete = complete and case["complete"]
    print(json.dumps(summary, ensure_ascii=False))
    # 0 heißt nur: Messung vollständig und Pixellage bestätigt, keine Freigabe.
    return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
