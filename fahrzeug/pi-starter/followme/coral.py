"""Optionaler Coral-USB-Erkenner: LiteRT direkt, ohne veraltetes PyCoral.

Vertrag: SSDLite MobileDet COCO QAT, uint8-RGB, SSD-Postprocessing,
COCO-Klasse 0 = Person. Einrichtung und Prüfgrenzen: ../CORAL.md.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .detect import Target
from .geometry import CameraModel, person_distance

MODEL_NAME = "ssdlite_mobiledet_coco_qat_postprocess_edgetpu.tflite"
CPU_MODEL_NAME = "ssdlite_mobiledet_coco_qat_postprocess.tflite"
DEFAULT_MODEL = Path("models") / MODEL_NAME
MAX_TARGETS = 5  # Anwendungs-Ausgabe begrenzen, nicht die Roh-Erkennung des Netzes


def default_library() -> str:
    local = Path.home() / ".local/share/followme/coral/libedgetpu.so.1"
    return str(local) if local.is_file() else "libedgetpu.so.1"


class CoralPersonDetector:
    """Gleiche Target-Schnittstelle wie PersonDetector, kein CPU-Fallback.

    edge_tpu=False ist ausschließlich für den expliziten CPU-Kompatibilitätstest
    mit dem unkompilierten Modell bestimmt, nicht für den Fahrbetrieb.
    """

    def __init__(self, cam: CameraModel, model_path: Path = DEFAULT_MODEL,
                 min_score: float = 0.45, library: str | None = None, *, edge_tpu: bool = True):
        self.cam = cam
        self.min_score = min_score
        self._interpreter = None
        self._delegate = None
        if not 0 <= min_score <= 1:
            raise ValueError("min_score muss zwischen 0 und 1 liegen")
        model_path = Path(model_path)
        data = model_path.read_bytes()  # fehlendes Modell früh und eindeutig melden
        if data[4:8] != b"TFL3" or (b"edgetpu-custom-op" in data) != edge_tpu:
            raise ValueError("Falsches TFLite-Modell: Coral braucht das Edge-TPU-Modell, CPU das Original")
        try:
            from ai_edge_litert.interpreter import Interpreter, load_delegate
        except ImportError as exc:
            raise RuntimeError("Offizielles LiteRT fehlt oder ist defekt; siehe CORAL.md. Kein Laufzeit-Fallback.") from exc
        try:
            delegates = []
            if edge_tpu:
                self._delegate = load_delegate(library or default_library(), {"device": "usb"})
                delegates.append(self._delegate)
            self._interpreter = Interpreter(model_path=str(model_path), num_threads=2,
                                            experimental_delegates=delegates)
            self._interpreter.allocate_tensors()
            inputs = self._interpreter.get_input_details()
            outputs = self._interpreter.get_output_details()
            if len(inputs) != 1 or inputs[0]["dtype"] != np.uint8:
                raise ValueError("MobileDet erwartet genau einen uint8-Eingang")
            shape = tuple(inputs[0]["shape"])
            if len(shape) != 4 or shape[0] != 1 or shape[3] != 3 or min(shape[1:3]) <= 0:
                raise ValueError(f"Unerwartete Eingabeform: {shape}")
            # Festes Modell, keine Vermutung anhand mehrdeutiger Tensorformen:
            # boxes, classes, scores, count (TFLite_Detection_PostProcess).
            if (len(outputs) != 4 or tuple(outputs[0]["shape"])[-1:] != (4,)
                    or tuple(outputs[1]["shape"]) != tuple(outputs[2]["shape"])
                    or np.prod(outputs[3]["shape"]) != 1):
                raise ValueError("Unerwartete SSD-Ausgänge; das festgelegte MobileDet-Modell verwenden")
            self._input = inputs[0]["index"]
            self._outputs = [o["index"] for o in outputs]
            self._height, self._width = int(shape[1]), int(shape[2])
        except Exception as exc:
            self.close()
            mode = "Coral-USB" if edge_tpu else "LiteRT-CPU-Test"
            raise RuntimeError(f"{mode} nicht verfügbar: {exc}. Siehe CORAL.md; kein CPU-Fallback.") from exc

    def detect(self, frame_bgr: np.ndarray) -> list[Target]:
        import cv2

        if self._interpreter is None:
            raise RuntimeError("Erkenner ist geschlossen")
        if frame_bgr.shape != (self.cam.height_px, self.cam.width_px, 3) or frame_bgr.dtype != np.uint8:
            raise ValueError("Bild muss uint8-BGR sein und zur Kamerageometrie passen")
        h, w = frame_bgr.shape[:2]
        scale = min(self._width / w, self._height / h)
        rw, rh = max(1, round(w * scale)), max(1, round(h * scale))
        left, top = (self._width - rw) // 2, (self._height - rh) // 2
        image = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        image[top:top + rh, left:left + rw] = cv2.resize(frame_bgr[:, :, ::-1], (rw, rh))
        self._interpreter.set_tensor(self._input, image[np.newaxis])
        self._interpreter.invoke()
        boxes, classes, scores, count = [self._interpreter.get_tensor(i) for i in self._outputs]
        boxes, classes, scores = boxes.reshape(-1, 4), classes.ravel(), scores.ravel()
        count_value = float(count.ravel()[0])
        if not np.isfinite(count_value) or count_value < 0 or count_value != int(count_value):
            raise ValueError("Ungültige Anzahl Erkennungen")
        n = min(int(count_value), len(boxes), len(classes), len(scores))
        targets = []
        for box, label, score in zip(boxes[:n], classes[:n], scores[:n]):
            if not np.isfinite([*box, label, score]).all() or label != 0 or not self.min_score <= score <= 1:
                continue
            y0, x0, y1, x1 = box
            x0, x1 = np.clip((np.array([x0, x1]) * self._width - left) * w / rw, 0, w)
            y0, y1 = np.clip((np.array([y0, y1]) * self._height - top) * h / rh, 0, h)
            if x1 <= x0 or y1 <= y0:
                continue
            rect = tuple(float(v) for v in (x0, y0, x1 - x0, y1 - y0))
            targets.append(Target(self.cam.bearing_rad((x0 + x1) / 2), person_distance(self.cam, rect),
                                  float(score), rect))
        return sorted(targets, key=lambda t: t.score, reverse=True)[:MAX_TARGETS]

    def close(self) -> None:
        # Interpreter vor Delegate freigeben: enthält dessen native Zeiger.
        self._interpreter = None
        self._delegate = None
