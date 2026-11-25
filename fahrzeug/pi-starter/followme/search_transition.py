"""Offline-Diagnose des Suchübergangs an eingefrorenen Simulatorposen.

Rekonstruiert synthetische Markerbilder, nicht die ursprünglichen Frames.
Reglerzustände/Posen bleiben Eingaben: KEIN alternativer geschlossener Fahrkreis,
kein Identitätstracker und keine Übernahme-/Sicherheitsentscheidung.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import deque
from pathlib import Path

from .detect import ArucoDetector, TargetSelector, _planar_distance
from .drive_assessment import CAMERA_OFFSET_M, assess, number
from .geometry import CameraModel
from .sim2d import PROFILES

STATES = {"SUCHE", "FOLGEN", "AUSSETZER", "DREHEN", "VERLOREN"}


def compare_geometry(before_target, before, after_target, after):
    """Beobachtungen im eigenen Kamerarahmen und im gemeinsamen Weltrahmen.

    Weltpose und wahre Zielposition sind privilegiertes Simulatorwissen, keine
    implementierte Kompensation. Die skalaren Abstände sind NICHT additiv.
    """
    def camera(row):
        yaw = row["yaw_rad"]
        return (row["x_m"] + CAMERA_OFFSET_M * math.cos(yaw),
                row["y_m"] + CAMERA_OFFSET_M * math.sin(yaw))

    def world(target, row):
        cx, cy = camera(row)
        angle = row["yaw_rad"] + target.bearing_rad
        return cx + target.distance_m * math.cos(angle), cy + target.distance_m * math.sin(angle)

    yaw_delta = after["yaw_rad"] - before["yaw_rad"]
    return {"camera_relative_jump_m": _planar_distance(before_target, after_target),
            "world_observation_shift_m": math.dist(world(before_target, before), world(after_target, after)),
            "true_target_shift_m": math.dist((before["target_x_m"], before["target_y_m"]),
                                             (after["target_x_m"], after["target_y_m"])),
            "camera_translation_m": math.dist(camera(before), camera(after)),
            "camera_yaw_change_rad": math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)),
            "since_selected_capture_s": after["t_s"] - before["t_s"]}


def reconstruct(source, *, detector=None):
    """Aktuellen Marker-/Latenzpfad gegen gespeicherte Zähler prüfen, ohne Geräte.

    Die Zählergleichheit prüft weder Reglerzustände noch die historische Version
    des Renderers. Abweichende/fehlende Zähler oder Abdeckung ⇒ complete=False.
    """
    from .virtual_drive import RENDERERS, Pose, render_marker

    if source.get("profile") != "pi3b_aruco" or source.get("fault") != "keine":
        raise ValueError("Nur ungestörten pi3b_aruco-Generator mit optionaler Bildpause rekonstruieren")
    if source.get("camera_offset_m") != CAMERA_OFFSET_M:
        raise ValueError("Explizite aktuelle Simulator-Kamerageometrie erforderlich")
    # Fahrten vor Einführung des Rendererfelds (22.10.2026) sind ganzzahlig gezeichnet.
    renderer = source.get("renderer", "ganzzahlig")
    if renderer not in RENDERERS:
        raise ValueError("Unbekannter Marker-Renderer")
    duration = number(source["duration_s"])
    if not 0 < duration <= 3600:
        raise ValueError("Ungültige Versuchsdauer")
    window = source.get("occlusion_window_s")
    if window is not None and (not isinstance(window, (list, tuple)) or len(window) != 2
                              or not 0 <= number(window[0]) < number(window[1]) <= duration):
        raise ValueError("Ungültige Marker-Bildpause")
    base = assess(source)
    reasons, rows = list(base["incomplete_reasons"]), source["samples"]
    if not rows or rows[0]["t_s"] > .1 or rows[-1]["t_s"] < duration - .1:
        reasons.append("gesamtabdeckung_fehlt")
    cam, selector = CameraModel(), TargetSelector()
    detector = detector if detector is not None else ArucoDetector(cam)
    config = PROFILES["pi3b_aruco"]
    next_frame, state, previous_capture = 0., "SUCHE", None
    pending, events = deque(), []
    mismatches, processed, rejected, selected_total = 0, 0, 0, 0
    raw_after, selected_after = [], []
    for row in rows:
        t, count, chosen_count = row["t_s"], 0, 0
        if row.get("controller") not in STATES:
            raise ValueError("Ungültiger gespeicherter Reglerzustand")
        updates, targets_count = row.get("perception_updates"), row.get("target_updates")
        if updates is None or targets_count is None:
            reasons.append("wahrnehmungszaehler_fehlen")
        elif (type(updates) is not int or type(targets_count) is not int
              or not 0 <= targets_count <= updates <= 100 or row["seen"] != (targets_count > 0)):
            raise ValueError("Ungültige Wahrnehmungszähler")
        if t >= next_frame:
            next_frame = t + 1 / config["frame_hz"]
            pose = Pose(row["x_m"], row["y_m"], row["yaw_rad"])
            distance, bearing = pose.target(row["target_x_m"], row["target_y_m"])
            hidden = window is not None and window[0] <= t < window[1]
            raw = detector.detect(render_marker(cam, distance, bearing, not hidden, renderer))
            pending.append((t + config["cam_latency_s"], row, raw))
        while pending and pending[0][0] <= t:
            _, captured, raw = pending.popleft()
            reset = state in ("DREHEN", "VERLOREN")
            if reset:
                selector.reset()
                previous_capture = None
            previous = selector._last
            geometry = None
            if raw and previous is not None and previous_capture is not None:
                nearest = min(raw, key=lambda target: _planar_distance(target, previous))
                geometry = compare_geometry(previous, previous_capture, nearest, captured)
            chosen = selector.select(raw)
            kind = "gewaehlt" if chosen is not None else "zuordnung_verworfen" if raw else "kein_rohtreffer"
            events.append({"processed_s": t, "capture_s": captured["t_s"], "raw_targets": len(raw),
                           "kind": kind, "recorded_state_before_row": state, "selector_reset": reset,
                           "gate_m": selector.max_jump_m, "geometry": geometry})
            processed += 1
            count += 1
            chosen_count += int(chosen is not None)
            selected_total += int(chosen is not None)
            rejected += int(bool(raw) and chosen is None)
            if chosen is not None:
                previous_capture = captured
            if window is not None and t >= window[1]:
                if raw:
                    raw_after.append(t - window[1])
                if chosen is not None:
                    selected_after.append(t - window[1])
        mismatches += int((count, chosen_count) != (updates, targets_count))
        # Gespeicherter Zustand ist erst im nächsten Schleifentakt Eingang.
        # Bei mehreren Ergebnissen pro Takt fehlt der Zwischenzustand im Log.
        if count > 1:
            reasons.append("zwischenzustaende_mehrerer_ergebnisse_fehlen")
        state = row["controller"]
    if mismatches:
        reasons.append("rekonstruktion_weicht_von_zaehlern_ab")
    return {"format": 1, "scope": "Rekonstruierte synthetische Frames auf eingefrorenen Posen/Zuständen",
            "complete": not reasons, "incomplete_reasons": sorted(set(reasons)), "passed": None,
            "samples": len(rows), "counter_mismatch_rows": mismatches, "frames_processed": processed,
            "selected_frames": selected_total, "rejected_frames": rejected,
            "first_raw_after_occlusion_s": raw_after[0] if raw_after else None,
            "third_raw_after_occlusion_s": raw_after[2] if len(raw_after) >= 3 else None,
            "first_selected_after_occlusion_s": selected_after[0] if selected_after else None,
            "third_selected_after_occlusion_s": selected_after[2] if len(selected_after) >= 3 else None,
            "generator_assumptions": {"frame_hz": config["frame_hz"], "latency_s": config["cam_latency_s"],
                                      "camera_offset_m": CAMERA_OFFSET_M, "marker_id": 7, "renderer": renderer},
            "events": events, "controller_states_verified": False, "original_frames_available": False,
            "closed_loop_runs_added": 0, "target_selector_changed": False,
            "hardware_verified": False, "person_recognition_verified": False, "ready_for_drive": False}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("fahrt", type=Path, help="Gespeicherte virtual_drive-JSON")
    ap.add_argument("--ausgabe", type=Path, required=True, help="Neue Ergebnisdatei, nie überschreiben")
    args = ap.parse_args(argv)
    if args.fahrt.stat().st_size > 64 * 1024 * 1024:
        ap.error("Fahrtdatei größer als 64 MiB")
    raw = args.fahrt.read_bytes()
    result = reconstruct(json.loads(raw))
    result["source_sha256"] = hashlib.sha256(raw).hexdigest()
    with args.ausgabe.open("x", encoding="utf-8") as file:
        json.dump(result, file, indent=2, ensure_ascii=False, allow_nan=False)
        file.write("\n")
    print(json.dumps({key: value for key, value in result.items() if key != "events"}, ensure_ascii=False))
    # 0 heißt ausschließlich: Rekonstruktion vollständig und zählergleich.
    return 0 if result["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
