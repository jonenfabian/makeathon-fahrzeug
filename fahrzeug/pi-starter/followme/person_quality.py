"""Boxgenaue Offline-Regression mit vorläufigen visuellen Annotationen.

Nur feste Videobilder, kein Tracking, keine Kamera-/Pico-Verbindung und keine
Abnahme von Pi-FPS oder echter Coral-Inferenz. Keine COCO-mAP-Berechnung.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
import platform
import re
from pathlib import Path

from .coral import CPU_MODEL_NAME, MAX_TARGETS, CoralPersonDetector
from .geometry import CameraModel
from .video_check import transform

DEFAULT_ANNOTATIONS = Path(__file__).resolve().parents[1] / "testdata/personen-boxen.json"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def valid_box(box, width_px=None, height_px=None):
    if (not isinstance(box, (tuple, list)) or len(box) != 4
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in box)):
        raise ValueError("Box braucht vier endliche Zahlen (x, y, Breite, Höhe)")
    x, y, w, h = box
    if x < 0 or y < 0 or w <= 0 or h <= 0:
        raise ValueError("Box braucht positive Ausdehnung und nichtnegative Koordinaten")
    if width_px is not None and (x + w > width_px or y + h > height_px):
        raise ValueError("Box liegt außerhalb des Bildes")
    return box


def load_annotations(path=DEFAULT_ANNOTATIONS):
    fixture = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(fixture, dict) or fixture.get("format") != 1 or fixture.get("box_format") != "xywh":
        raise ValueError("Unbekanntes Annotationsformat")
    if not isinstance(fixture.get("video_sha256"), str) or not re.fullmatch("[a-f0-9]{64}", fixture["video_sha256"]):
        raise ValueError("Feste Video-SHA256 fehlt")
    width, height = fixture.get("width_px"), fixture.get("height_px")
    if any(type(v) is not int or not 1 <= v <= 4096 for v in (width, height)):
        raise ValueError("Ungültige Bildgröße")
    cases = fixture.get("cases")
    if not isinstance(cases, list) or not cases:
        raise ValueError("Keine annotierten Prüffälle")
    ids = set()
    for case in cases:
        if (not isinstance(case, dict) or not isinstance(case.get("id"), str)
                or not re.fullmatch("[a-z0-9-]+", case["id"]) or case["id"] in ids):
            raise ValueError("Prüffall-ID muss eindeutig und als Dateiname sicher sein")
        ids.add(case["id"])
        if type(case.get("frame")) is not int or case["frame"] < 0:
            raise ValueError("Frame muss eine nichtnegative ganze Zahl sein")
        if case.get("crop") is not None:
            crop = valid_box(case["crop"], width, height)
            if any(type(v) is not int for v in crop):
                raise ValueError("Ausschnitt muss ganze Pixel verwenden")
        for key in ("people", "ignore"):
            if not isinstance(case.get(key), list):
                raise TypeError(f"{key} muss eine Boxliste sein")
            for box in case[key]:
                valid_box(box, width, height)
    scope = fixture.get("evaluation_scope", "mixed")
    if scope not in ("mixed", "negative_only"):
        raise ValueError("Unbekannter Auswerteumfang")
    if scope == "negative_only":
        if any(c["people"] or c["ignore"] for c in cases):
            raise ValueError("Reiner Negativtest darf keine Personen oder Ignore-Zonen enthalten")
    elif not any(c["people"] for c in cases) or not any(not c["people"] and not c["ignore"] for c in cases):
        raise ValueError("Positive und eindeutig negative Fälle sind erforderlich")
    floors = fixture.get("regression_floors")
    required = {"negative_false_positives"} if scope == "negative_only" else {"precision", "recall", "negative_false_positives"}
    if (not isinstance(floors, dict) or set(floors) != required
            or any(type(floors.get(k)) not in (int, float) or not 0 <= floors[k] <= 1
                   for k in ("precision", "recall") if k in required)
            or type(floors.get("negative_false_positives")) is not int or floors["negative_false_positives"] < 0):
        raise ValueError("Ungültige Regression-Mindestwerte")
    if not isinstance(fixture.get("annotation_method"), str) or not fixture["annotation_method"].strip():
        raise ValueError("Annotationsherkunft fehlt")
    return fixture


def intersection(a, b):
    return max(0, min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0])) * max(
        0, min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1]))


def iou(a, b):
    overlap = intersection(a, b)
    return overlap / (a[2] * a[3] + b[2] * b[3] - overlap)


def match_boxes(people, ignored, predictions, *, iou_threshold=.5, min_score=.45):
    """Konfidenz-absteigend, je GT höchstens ein Treffer, bestes freies IoU.

    Duplikate zählen als FP, auch wenn sie in eine Ignore-Box fallen.
    Ignore-Zuordnung erst nach regulären Zielen, anhand eigener Vorhersagefläche.
    """
    if not 0 < iou_threshold <= 1 or not 0 <= min_score <= 1:
        raise ValueError("Ungültige IoU-/Konfidenzschwelle")
    for box in [*people, *ignored]:
        valid_box(box)
    for pred in predictions:
        valid_box(pred["box"])
        if not math.isfinite(pred["score"]) or not 0 <= pred["score"] <= 1:
            raise ValueError("Ungültiger Erkennungsscore")
    used = set()
    rows = []
    for pred in sorted(predictions, key=lambda p: p["score"], reverse=True):
        if pred["score"] < min_score:
            continue
        overlaps = [iou(pred["box"], box) for box in people]
        candidates = [i for i, overlap in enumerate(overlaps) if overlap >= iou_threshold and i not in used]
        row = {"box": list(pred["box"]), "score": pred["score"], "match": None, "iou": None}
        if candidates:
            index = max(candidates, key=lambda i: overlaps[i])
            used.add(index)
            row.update(result="tp", match=index, iou=overlaps[index])
        elif any(overlap >= iou_threshold for overlap in overlaps):
            row["result"] = "fp"  # zweite Box für dieselbe Person, nicht wegignorieren
        elif any(intersection(pred["box"], box) / (pred["box"][2] * pred["box"][3]) >= .5 for box in ignored):
            row["result"] = "ignored"
        else:
            row["result"] = "fp"
        rows.append(row)
    return {"tp": len(used), "fp": sum(row["result"] == "fp" for row in rows),
            "fn": len(people) - len(used), "ignored": sum(row["result"] == "ignored" for row in rows),
            "missed_indices": [i for i in range(len(people)) if i not in used], "predictions": rows}


def case_images(video, fixture):
    """Sequenziell decodieren: keine ungenauen Keyframe-Seeks und kein Vollvideo im RAM."""
    import cv2

    if sha256(video) != fixture["video_sha256"]:
        raise ValueError("Video-SHA256 passt nicht zu den Annotationen")
    capture = cv2.VideoCapture(str(video))
    try:
        if not capture.isOpened():
            raise ValueError("Video kann nicht geöffnet werden")
        last = max(case["frame"] for case in fixture["cases"])
        for frame in range(last + 1):
            ok, original = capture.read()
            if not ok:
                raise ValueError(f"Video endet vor annotiertem Frame {frame}")
            cases = [case for case in fixture["cases"] if case["frame"] == frame]
            if not cases:
                continue
            image = cv2.resize(original, (fixture["width_px"], fixture["height_px"]))
            for case in cases:
                sample = image.copy()
                if case.get("crop") is not None:
                    x, y, w, h = case["crop"]
                    sample = cv2.resize(sample[y:y + h, x:x + w], (fixture["width_px"], fixture["height_px"]))
                yield case, sample
    finally:
        capture.release()


def paint(image, case, result=None):
    import cv2

    image = image.copy()

    def box(rect, color, text):
        x, y, w, h = (round(v) for v in rect)
        cv2.rectangle(image, (x, y), (x + w - 1, y + h - 1), color, 2)
        cv2.putText(image, text, (x + 2, max(y + 12, 12)), cv2.FONT_HERSHEY_SIMPLEX, .35, color, 1)

    missed = result["missed_indices"] if result else []
    for i, rect in enumerate(case["people"]):
        box(rect, (0, 0, 255) if i in missed else (0, 210, 0), f"GT {i}" + (" FN" if i in missed else ""))
    for rect in case["ignore"]:
        box(rect, (160, 160, 160), "ignore")
    if result:
        for pred in result["predictions"]:
            color = (0, 150, 255) if pred["result"] == "fp" else (255, 180, 0)
            box(pred["box"], color, f"{pred['result']} {pred['score']:.2f}")
    return image


def evaluate_fixture(video, fixture, detector, *, min_score=.45, variant="original", preview=None):
    import cv2

    if variant not in ("original", "dunkel"):
        raise ValueError("Nur Original/Helligkeit erhalten die annotierte Geometrie")
    samples = []
    for case, image in case_images(video, fixture):
        image = transform(image, variant)
        predictions = [{"box": list(t.box), "score": t.score} for t in detector.detect(image)]
        result = match_boxes(case["people"], case["ignore"], predictions, min_score=min_score)
        samples.append({"id": case["id"], "frame": case["frame"], "negative": not case["people"] and not case["ignore"],
                        "annotated": len(case["people"]), **result})
        if preview and not cv2.imwrite(str(preview / f"{case['id']}.png"), paint(image, case, result)):
            raise OSError("Vorschaubild konnte nicht gespeichert werden")
    counts = {key: sum(sample[key] for sample in samples) for key in ("tp", "fp", "fn", "ignored")}
    precision = (counts["tp"] / (counts["tp"] + counts["fp"])
                 if counts["tp"] + counts["fn"] and counts["tp"] + counts["fp"] else None)
    recall = counts["tp"] / (counts["tp"] + counts["fn"]) if counts["tp"] + counts["fn"] else None
    return {"variant": variant, "min_score": min_score, "iou_threshold": .5, **counts,
            "precision": precision, "recall": recall,
            "negative_false_positives": sum(sample["fp"] for sample in samples if sample["negative"]),
            "samples": samples}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("video", type=Path)
    ap.add_argument("--annotation", type=Path, default=DEFAULT_ANNOTATIONS)
    ap.add_argument("--modell", type=Path, default=Path("models") / CPU_MODEL_NAME)
    ap.add_argument("--scores", nargs="+", type=float, default=[.45])
    ap.add_argument("--varianten", nargs="+", choices=("original", "dunkel"), default=["original"])
    ap.add_argument("--vorschau", type=Path, help="Neuer Ordner für Box-Overlays")
    ap.add_argument("--nur-annotation", action="store_true", help="Nur Soll-Boxen ansehen, kein Modell laden")
    ap.add_argument("--ausgabe", type=Path, help="Neuer JSON-Bericht; ohne Option JSON auf stdout")
    args = ap.parse_args(argv)
    if (len(set(args.scores)) != len(args.scores) or not all(0 <= v <= 1 for v in args.scores)
            or args.nur_annotation and args.vorschau is None):
        ap.error("Eindeutige Scores in [0,1] wählen; --nur-annotation erfordert --vorschau")
    fixture = load_annotations(args.annotation)
    annotated = sum(len(case["people"]) for case in fixture["cases"])
    scope = fixture.get("evaluation_scope", "mixed")
    try:
        litert_version = importlib.metadata.version("ai-edge-litert")
    except importlib.metadata.PackageNotFoundError:
        litert_version = None
    report = {"format": 1, "backend": "none" if args.nur_annotation else "litert-cpu", "measured": False, "passed": None,
              "platform": platform.platform(), "python_version": platform.python_version(),
              "litert_version": litert_version,
              "runtime": "litert",
              "quality_approved": False, "identity_evaluated": False, "distance_evaluated": False,
              "hardware_benchmark": False, "video_sha256": fixture["video_sha256"],
              "evaluation_scope": scope, "recall_evaluated": False,
              "annotation_sha256": sha256(args.annotation), "annotation_method": fixture["annotation_method"],
              "annotation_revision": fixture.get("annotation_revision"),
              "scope": "Anwendungs-Ausgabe inklusive Personenlimit; keine Roh-Netz-mAP und keine unabhängige Abnahme",
              "annotated_people": annotated, "negative_images": sum(not c["people"] and not c["ignore"]
                                                                      for c in fixture["cases"]),
              "recall_ceiling_from_output_cap": sum(min(MAX_TARGETS, len(c["people"]))
                                                     for c in fixture["cases"]) / annotated if annotated else None,
              "reports": []}
    output = args.ausgabe.open("x", encoding="utf-8") if args.ausgabe else None
    try:
        import cv2

        report["opencv_version"] = cv2.__version__
        if args.vorschau:
            args.vorschau.mkdir(parents=True, exist_ok=False)
        if args.nur_annotation:
            for case, image in case_images(args.video, fixture):
                if not cv2.imwrite(str(args.vorschau / f"{case['id']}.png"), paint(image, case)):
                    raise OSError("Vorschaubild konnte nicht gespeichert werden")
        else:
            report["model_sha256"] = sha256(args.modell)
            for score in args.scores:
                detector = CoralPersonDetector(CameraModel(width_px=fixture["width_px"], height_px=fixture["height_px"]),
                                                args.modell, min_score=score, edge_tpu=False)
                try:
                    for variant in args.varianten:
                        preview = args.vorschau / f"{variant}-{score:g}" if args.vorschau else None
                        if preview:
                            preview.mkdir()
                        report["reports"].append(evaluate_fixture(args.video, fixture, detector, min_score=score,
                                                                  variant=variant, preview=preview))
                finally:
                    detector.close()
            floors = fixture["regression_floors"]
            original = [r for r in report["reports"] if r["variant"] == "original" and r["min_score"] == .45]
            # Nur die vorab festgelegte Baseline bewerten, keinen günstigsten Sweep-Wert wählen.
            report["checks"] = []
            for result in original:
                check = {"negative": result["negative_false_positives"] <= floors["negative_false_positives"]}
                if scope == "mixed":
                    check.update(precision=result["precision"] is not None and result["precision"] >= floors["precision"],
                                 recall=result["recall"] is not None and result["recall"] >= floors["recall"])
                report["checks"].append(check)
            report.update(measured=True, recall_evaluated=annotated > 0, output_cap=MAX_TARGETS, regression_floors=floors,
                          passed=all(all(check.values()) for check in report["checks"]) if original else None)
    except (OSError, RuntimeError, ValueError, ImportError) as exc:
        report.update(passed=False, error=f"{type(exc).__name__}: {exc}")
    finally:
        data = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if output:
            with output:
                output.write(data)
    if args.ausgabe:
        print(json.dumps({"passed": report["passed"], "measured": report["measured"],
                          "evaluation_scope": report["evaluation_scope"], "recall_evaluated": report["recall_evaluated"],
                          "report": str(args.ausgabe), "error": report.get("error"),
                          "results": [{key: r[key] for key in ("variant", "min_score", "tp", "fp", "fn",
                                                               "precision", "recall", "negative_false_positives")}
                                      for r in report["reports"]]}, ensure_ascii=False))
    else:
        print(data, end="")
    return int(report["passed"] is False)


if __name__ == "__main__":
    raise SystemExit(main())
