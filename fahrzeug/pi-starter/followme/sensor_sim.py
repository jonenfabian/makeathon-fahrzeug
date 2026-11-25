"""Deterministische Simulation von drei unabhängigen ToF-Messwerten.

Rein softwareseitig; kein Pico-Protokoll und keine Motor- oder Hardwareanbindung.
"""
from __future__ import annotations

import threading

from .sensors import SENSOR_NAMES, DistanceSample


class SimulatedDistances:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples = {name: DistanceSample(name, "unavailable", None, None, "sim") for name in SENSOR_NAMES}

    def update(self, sensor: str, state: str, distance_mm: int | None, captured_s: float) -> None:
        sample = DistanceSample(sensor, state, distance_mm, captured_s, "sim")
        with self._lock:
            previous = self._samples[sensor]
            if previous.captured_s is not None and captured_s <= previous.captured_s:
                raise ValueError("Messung ist nicht neuer als die letzte")
            self._samples[sensor] = sample

    def snapshot(self, now_s: float, max_age_s: float) -> dict[str, dict]:
        with self._lock:
            samples = self._samples.copy()
        return {name: sample.view(now_s, max_age_s) for name, sample in samples.items()}


def main() -> None:
    """Beispiel für unabhängige Abstände und Alter ohne Gerät oder Netzwerk."""
    import json

    sensors = SimulatedDistances()
    sensors.update("front", "distance", 800, 10.0)
    sensors.update("links", "too_close", 0, 10.1)
    sensors.update("rechts", "no_target", None, 10.2)
    print(json.dumps(sensors.snapshot(10.4, .5), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
