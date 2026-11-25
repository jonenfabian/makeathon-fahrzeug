"""Fahrtenschreiber offline wiederholen. Öffnet niemals einen Pico oder Motorpfad."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path

from .controller import FollowParams
from .coral import CPU_MODEL_NAME, CoralPersonDetector
from .detect import Target
from .geometry import CameraModel
from .pipeline import PerceptionLoop


def replay(directory: Path, *, detector=None, delay_s: float = 0.0) -> dict:
    if not math.isfinite(delay_s) or delay_s < 0:
        raise ValueError("Zusätzliche Verzögerung muss endlich und nichtnegativ sein")
    directory = directory.resolve()
    with (directory / "events.jsonl").open(encoding="utf-8") as source:
        header = json.loads(next(source))
        if header.get("kind") != "run" or header.get("format") != 1:
            raise ValueError("Unbekanntes Protokollformat")
        pipeline = PerceptionLoop(detector, FollowParams(**header["params"]))
        frames, differences, discarded, tx, rx, errors = 0, 0, 0, 0, 0, 0
        ended, dropped = False, 0
        for line in source:
            if ended:
                raise ValueError("Daten nach Protokollende")
            event = json.loads(line)
            kind = event["kind"]
            if kind == "end":
                ended, dropped = True, event["dropped"]
            elif kind == "control":
                pipeline.reset()
            elif kind == "tx":
                tx += 1
            elif kind == "rx":
                rx += 1
            elif kind == "error":
                errors += 1
            elif kind == "frame":
                frames += 1
                if event.get("discarded") == "bedienung":
                    discarded += 1
                    continue  # Ergebnis aus einer vor STOP/ARM gestarteten Berechnung
                targets = [Target(**{**t, "box": tuple(t["box"])}) for t in event["targets"]]
                if detector is not None:
                    import cv2

                    if "image" not in event:
                        raise ValueError("Neu-Erkennung erfordert ein Protokoll mit --bilder")
                    image_path = (directory / event["image"]).resolve()
                    if not image_path.is_relative_to(directory):
                        raise ValueError("Bildpfad verlässt den Protokollordner")
                    image = cv2.imread(str(image_path))
                    if image is None:
                        raise ValueError(f"Bild fehlt oder ist ungültig: {image_path}")
                    targets = detector.detect(image)
                step = pipeline.evaluate(targets, event["captured_s"], event["finished_s"] + delay_s)
                discarded += step.discarded is not None
                differences += not (math.isclose(step.v_mps, event["v_mps"], abs_tol=1e-9)
                                    and math.isclose(step.omega_rad_s, event["omega_rad_s"], abs_tol=1e-9))
        return {"frames": frames, "command_differences": differences, "discarded": discarded,
                "tx_events": tx, "rx_events": rx, "complete": ended and dropped == 0,
                "dropped": dropped, "extra_delay_s": delay_s, "redetected": detector is not None,
                "source_errors": errors,
                "hardware": False, "params": asdict(pipeline.params)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("ordner", type=Path)
    ap.add_argument("--zusatzlatenz", type=float, default=0)
    ap.add_argument("--neu-erkennen", action="store_true", help="CPU-MobileDet auf gespeicherten JPEGs")
    ap.add_argument("--modell", type=Path, default=Path("models") / CPU_MODEL_NAME)
    ap.add_argument("--ausgabe", type=Path)
    args = ap.parse_args(argv)
    detector = None
    try:
        if args.neu_erkennen:
            with (args.ordner / "events.jsonl").open(encoding="utf-8") as source:
                header = json.loads(next(source))
            detector = CoralPersonDetector(CameraModel(**header["camera"]), args.modell, edge_tpu=False)
        report = replay(args.ordner, detector=detector, delay_s=args.zusatzlatenz)
    finally:
        if detector:
            detector.close()
    data = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    print(data, end="")
    if args.ausgabe:
        with args.ausgabe.open("x", encoding="utf-8") as file:
            file.write(data)
    return int(not report["complete"] or report["frames"] == 0 or report["source_errors"] > 0
               or (not args.neu_erkennen and args.zusatzlatenz == 0 and report["command_differences"] > 0))


if __name__ == "__main__":
    raise SystemExit(main())
