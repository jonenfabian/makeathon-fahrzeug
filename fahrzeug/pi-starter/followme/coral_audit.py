"""Offline-Diagnose: Rohmodell, Adapter und Coral-Referenz getrennt prüfen.

Referenz: google-coral/pycoral, Commit 46f06c8dd9c5231e5cb27910d4e1fcbdcf1be8f0,
adapters/common.py und examples/detect_image.py. Eigenständige Umsetzung,
kein PyCoral-Import, kein USB, keine automatische Änderung der Live-Pipeline.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path

import numpy as np

from .coral import CPU_MODEL_NAME, MAX_TARGETS, CoralPersonDetector
from .geometry import CameraModel
from .person_quality import case_images, load_annotations, match_boxes, sha256

REFERENCE_COMMIT = "46f06c8dd9c5231e5cb27910d4e1fcbdcf1be8f0"
MODEL_SHA256 = "32c486140391eb4dc43fca7113ad392be632dc5366687f2731f73d740678693f"
MODES = ("application", "center_lanczos", "coral_reference")


def prepare(frame, width, height, mode):
    if mode not in MODES:
        raise ValueError("Unbekannte Vergleichsvariante")
    h, w = frame.shape[:2]
    scale = min(width / w, height / h)
    rw, rh = ((int(w * scale), int(h * scale)) if mode == "coral_reference"
              else (round(w * scale), round(h * scale)))
    rw, rh = max(1, rw), max(1, rh)
    left, top = (0, 0) if mode == "coral_reference" else ((width - rw) // 2, (height - rh) // 2)
    rgb = frame[:, :, ::-1]
    if mode == "application":
        import cv2

        resized = cv2.resize(rgb, (rw, rh))
    else:
        from PIL import Image  # optional; Pillow 12.0.0, MIT-CMU

        resized = np.asarray(Image.fromarray(rgb).resize((rw, rh), Image.Resampling.LANCZOS))
    tensor = np.zeros((height, width, 3), np.uint8)
    tensor[top:top + rh, left:left + rw] = resized
    return tensor, {"left": left, "top": top, "rw": rw, "rh": rh, "width": w, "height": h}


def project_people(boxes, classes, scores, count, transform, min_score=.45):
    """Unabhängige skalare Rückprojektion statt Wiederverwendung des Adapters."""
    people = []
    for i in range(count):
        if classes[i] != 0 or scores[i] < min_score:
            continue
        y0, x0, y1, x1 = boxes[i]
        coords = []
        for value, extent, offset, resized, source in (
            (x0, 320, transform["left"], transform["rw"], transform["width"]),
            (y0, 320, transform["top"], transform["rh"], transform["height"]),
            (x1, 320, transform["left"], transform["rw"], transform["width"]),
            (y1, 320, transform["top"], transform["rh"], transform["height"]),
        ):
            coords.append(max(0.0, min(float(source), (float(value) * extent - offset) * source / resized)))
        x0, y0, x1, y1 = coords
        if x1 > x0 and y1 > y0:
            people.append({"box": [x0, y0, x1 - x0, y1 - y0], "score": float(scores[i])})
    return sorted(people, key=lambda row: row["score"], reverse=True)[:MAX_TARGETS]


def audit(video, annotation, model):
    if sha256(model) != MODEL_SHA256:
        raise ValueError("Diagnose ist nur für das festgelegte CPU-MobileDet definiert")
    fixture = load_annotations(annotation)
    detector = CoralPersonDetector(CameraModel(width_px=fixture["width_px"], height_px=fixture["height_px"]),
                                   model, edge_tpu=False)
    try:
        interpreter = detector._interpreter
        inputs, outputs = interpreter.get_input_details(), interpreter.get_output_details()
        if (tuple(inputs[0]["shape"]) != (1, 320, 320, 3) or inputs[0]["dtype"] != np.uint8
                or tuple(inputs[0]["quantization"]) != (1 / 128, 128)
                or [o["name"] for o in outputs] != ["TFLite_Detection_PostProcess",
                    "TFLite_Detection_PostProcess:1", "TFLite_Detection_PostProcess:2", "TFLite_Detection_PostProcess:3"]):
            raise ValueError("Tensorvertrag weicht vom geprüften Modell ab")
        cases = []
        for case, image in case_images(video, fixture):
            targets = detector.detect(image)
            actual_input = interpreter.get_tensor(inputs[0]["index"])[0]
            modes = []
            for mode in MODES:
                tensor, transform = prepare(image, 320, 320, mode)
                if mode == "application":
                    input_equal = np.array_equal(tensor, actual_input)
                interpreter.set_tensor(inputs[0]["index"], tensor[np.newaxis])
                interpreter.invoke()
                boxes, classes, scores, count = [interpreter.get_tensor(o["index"]) for o in outputs]
                n = int(count.ravel()[0])
                boxes, classes, scores = boxes.reshape(-1, 4), classes.ravel(), scores.ravel()
                if not 0 <= n <= min(len(boxes), len(classes), len(scores)):
                    raise ValueError("Ungültige Rohmodell-Anzahl")
                predictions = project_people(boxes, classes, scores, n, transform)
                if mode == "application":
                    projection_equal = (len(predictions) == len(targets) and all(
                        np.allclose(row["box"], target.box, rtol=0, atol=.001)
                        and abs(row["score"] - target.score) < 1e-6 for row, target in zip(predictions, targets)))
                modes.append({"mode": mode, "input_sha256": hashlib.sha256(tensor.tobytes()).hexdigest(),
                              "transform": transform,
                              "raw_person_candidates": int(sum(classes[i] == 0 and scores[i] >= .45 for i in range(n))),
                              "raw": [{"class_id": int(classes[i]), "score": float(scores[i]),
                                       "box_yxyx": boxes[i].tolist()} for i in range(n) if scores[i] >= .45],
                              **match_boxes(case["people"], case["ignore"], predictions)})
            cases.append({"id": case["id"], "input_equal": input_equal, "projection_equal": projection_equal,
                          "modes": modes})
        return {"format": 1, "hardware_verified": False, "quality_approved": False, "production_changed": False,
                "model_sha256": sha256(model), "video_sha256": sha256(video), "annotation_sha256": sha256(annotation),
                "reference_commit": REFERENCE_COMMIT, "person_class_id": 0,
                "versions": {p: importlib.metadata.version(p) for p in ("ai-edge-litert", "numpy", "pillow")},
                "adapter_consistent": bool(cases) and all(c["input_equal"] and c["projection_equal"] for c in cases),
                "totals": {mode: {key: sum(m[key] for c in cases for m in c["modes"] if m["mode"] == mode)
                                  for key in ("tp", "fp", "fn")} for mode in MODES}, "cases": cases}
    finally:
        detector.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("video", type=Path)
    ap.add_argument("--annotation", type=Path, required=True)
    ap.add_argument("--modell", type=Path, default=Path("models") / CPU_MODEL_NAME)
    ap.add_argument("--ausgabe", type=Path, required=True)
    args = ap.parse_args(argv)
    with args.ausgabe.open("x", encoding="utf-8") as file:
        report = audit(args.video, args.annotation, args.modell)
        file.write(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
    print(json.dumps({"adapter_consistent": report["adapter_consistent"], "totals": report["totals"]}))
    return int(not report["adapter_consistent"])


if __name__ == "__main__":
    raise SystemExit(main())
