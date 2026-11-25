"""Reale Personenvideos prüfen, ohne Kamera, Pico und ARM. CPU != Pi-Benchmark."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .coral import CPU_MODEL_NAME, CoralPersonDetector
from .detect import TargetSelector
from .geometry import CameraModel

VARIANTS = ("original", "dunkel", "verdeckt", "bildverlust")


def transform(image, variant: str):
    if variant == "original":
        return image.copy()
    if variant == "dunkel":
        return (image.astype(np.float32) * .25).astype(np.uint8)
    if variant == "verdeckt":
        image = image.copy()
        w = image.shape[1]
        image[:, w // 3:2 * w // 3] = 0
        return image
    if variant == "bildverlust":
        return np.zeros_like(image)
    raise ValueError(variant)


def check_video(path: Path, detector, *, step: int = 10, max_frames: int = 40, variant: str = "original") -> dict:
    import cv2

    if step < 1 or max_frames < 1:
        raise ValueError("Abtastschritt und Bildanzahl müssen positiv sein")
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        capture.release()
        raise ValueError(f"Video kann nicht geöffnet werden: {path}")
    results = []
    selector = TargetSelector()
    try:
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Video hat keine brauchbare Zeitbasis")
        index = 0
        previous_s = -1.0
        while len(results) < max_frames:
            ok, image = capture.read()
            if not ok:
                break
            # Bei diesem WebM meldet OpenCV teils 1000 "FPS" (Container-Zeitbasis),
            # während die tatsächlichen Präsentationszeitpunkte 40 ms auseinanderliegen.
            video_s = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000
            if not math.isfinite(video_s) or video_s < 0 or video_s <= previous_s:
                raise ValueError("Video liefert keine streng steigenden Präsentationszeitpunkte")
            previous_s = video_s
            if index % step == 0:
                image = cv2.resize(image, (640, 360))
                targets = detector.detect(transform(image, variant))
                chosen = selector.select(targets)
                results.append({"frame": index, "video_s": video_s, "people": len(targets),
                                "boxes": [list(t.box) for t in targets],
                                "chosen_box": list(chosen.box) if chosen else None})
            index += 1
    finally:
        capture.release()
    if not results:
        raise ValueError("Keine Bilder geprüft")
    hits = sum(row["people"] > 0 for row in results)
    return {"video": str(path), "variant": variant, "frames": len(results), "hit_rate": hits / len(results),
            "timebase": "presentation-timestamps", "container_fps": fps,
            "multi_person_frames": sum(r["people"] >= 2 for r in results),
            "identity_evaluated": False, "distance_evaluated": False, "hardware_benchmark": False,
            "transformed": variant != "original", "samples": results}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("video", type=Path)
    ap.add_argument("--modell", type=Path, default=Path("models") / CPU_MODEL_NAME)
    ap.add_argument("--varianten", nargs="+", choices=VARIANTS, default=list(VARIANTS))
    ap.add_argument("--schritt", type=int, default=10)
    ap.add_argument("--max-bilder", type=int, default=40)
    ap.add_argument("--ausgabe", type=Path)
    ap.add_argument("--min-trefferquote", type=float, default=.75, help="Nur Original; KEIN Recall ohne Boxannotation")
    args = ap.parse_args(argv)
    if not 0 <= args.min_trefferquote <= 1:
        ap.error("Trefferquote muss zwischen 0 und 1 liegen")
    detector = CoralPersonDetector(CameraModel(), args.modell, edge_tpu=False)
    try:
        reports = [check_video(args.video, detector, step=args.schritt, max_frames=args.max_bilder, variant=v)
                   for v in args.varianten]
    finally:
        detector.close()
    checks = {r["variant"]: r["hit_rate"] >= args.min_trefferquote if r["variant"] == "original"
              else r["hit_rate"] == 0 for r in reports if r["variant"] in ("original", "bildverlust")}
    report = {"format": 1, "backend": "litert-cpu", "checks": checks,
              "passed": all(checks.values()) if checks else None,
              "reports": reports, "limits": "Keine Identitäts-, Abstands- oder Pi-FPS-Abnahme"}
    data = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if args.ausgabe:
        with args.ausgabe.open("x", encoding="utf-8") as file:
            file.write(data)
        print(json.dumps({"passed": report["passed"], "checks": checks,
                          "observations": [{k: r[k] for k in ("variant", "frames", "hit_rate", "multi_person_frames")}
                                           for r in reports], "report": str(args.ausgabe)}, ensure_ascii=False))
    else:
        print(data, end="")
    return int(report["passed"] is False)


if __name__ == "__main__":
    raise SystemExit(main())
