"""Lesende Datenbasis für Team-Dashboards; keine Steuer- oder Schutzfunktion."""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import asdict

from .link import STATUS_MAX_AGE_S
from .markers import MarkerObservation
from .pipeline import Step
from .protocol import Info, Status, Ultrasonic, supported_protocol
from .sensor_sim import SimulatedDistances
from .sensors import DistanceSample, side_views, v1_front

StatusProvider = Callable[[], tuple[Status | None, float | None, str | None]]
UltrasonicProvider = Callable[[], tuple[Ultrasonic | None, float | None]]


class DashboardData:
    """Ein letzter Wahrnehmungszustand; Zeitpunkt und Status beim Lesen bewerten."""

    def __init__(self, vehicle_id: str, detector: str, marker_id: int | None,
                 status_provider: StatusProvider | None = None, *, clock=time.monotonic,
                 max_image_age_s: float = .6, simulated_sensors: SimulatedDistances | None = None,
                 info_provider: Callable[[], Info | None] | None = None,
                 ultrasonic_provider: UltrasonicProvider | None = None):
        if not math.isfinite(max_image_age_s) or max_image_age_s <= 0:
            raise ValueError("Ungültige Bildaltersgrenze")
        self.vehicle_id = vehicle_id
        self.detector = detector
        self.marker_id = marker_id
        self.status_provider = status_provider
        self.info_provider = info_provider
        self.ultrasonic_provider = ultrasonic_provider  # McuLink.ultrasonic_snapshot, Zeitbezug wie clock
        self.clock = clock
        self.max_image_age_s = max_image_age_s
        self.simulated_sensors = simulated_sensors
        self._lock = threading.Lock()
        self._step: Step | None = None
        self._timestamp_source: str | None = None
        self._markers: tuple[MarkerObservation, ...] = ()

    def update(self, step: Step, markers: list[MarkerObservation], timestamp_source: str | None = None) -> None:
        """Nur bereits verarbeitete Daten übernehmen; niemals Motorbefehle senden."""
        with self._lock:
            self._step = step
            self._timestamp_source = timestamp_source
            self._markers = tuple(markers) if not step.discarded else ()

    def clear(self) -> None:
        """Sofortige Invalidierung bei STOP, ARM oder Quellwechsel."""
        with self._lock:
            self._step = None
            self._timestamp_source = None
            self._markers = ()

    def snapshot(self) -> dict:
        now = self.clock()
        with self._lock:
            step, markers, timestamp_source = self._step, self._markers, self._timestamp_source
        status, status_age, error = self.status_provider() if self.status_provider else (None, None, None)
        info = self.info_provider() if self.info_provider else None
        protocol_state = ("unknown" if info is None else f"v{info.proto_version}"
                          if supported_protocol(info.proto_version) else "unsupported")
        status_fresh = (protocol_state != "unsupported" and status is not None and error is None and status_age is not None
                        and math.isfinite(status_age) and 0 <= status_age <= STATUS_MAX_AGE_S)
        connection = ("error" if error else "protocol_unsupported" if protocol_state == "unsupported" else
                      "unknown" if status is None else "fresh" if status_fresh else "stale")
        pico = None if status is None else {
            "state": status.state, "flags": status.flag_names(), "t_ms": status.t_ms,
            "status_age_s": status_age if status_age is not None and math.isfinite(status_age) else None,
            "fresh": status_fresh,
            "v_l_mm_s": status.v_l_mm_s, "v_r_mm_s": status.v_r_mm_s,
            "ticks_l": status.ticks_l, "ticks_r": status.ticks_r, "vmot_mv": status.vmot_mv,
            "tof_raw_mm": status.tof_mm,
        }
        front = v1_front(status).view(now, 1.0)
        if status is not None and not status_fresh:
            front["state"] = ("connection_error" if error else "protocol_unsupported"
                              if protocol_state == "unsupported" else "status_stale")
            front["distance_mm"] = None
        sensors = {"front": front}
        if self.ultrasonic_provider is not None and protocol_state != "unsupported":
            sides = side_views(self.ultrasonic_provider(), now)
        else:
            sides = {name: DistanceSample(name, "unavailable", None, None, "none").view(now, 1.0)
                     for name in ("links", "rechts")}
        for name, view in sides.items():
            if error and view["source"] == "pico":
                view["state"], view["distance_mm"] = "connection_error", None
            sensors[name] = view
        if self.simulated_sensors is not None:
            # Nur ausdrücklich injizierte Simulatordaten, nie unbemerkt echte V1-Werte ersetzen.
            sensors.update({name: data for name, data in self.simulated_sensors.snapshot(now, 1.0).items()
                            if name in ("links", "rechts")})
        age = None if step is None or not math.isfinite(step.captured_s) else now - step.captured_s
        image_fresh = (step is not None and not step.discarded and age is not None
                       and 0 <= age <= self.max_image_age_s)
        target = asdict(step.chosen) if image_fresh and step.chosen is not None else None
        visible = [asdict(marker) for marker in markers if marker.captured_s == step.captured_s] if image_fresh else []
        return {"schema_version": 1, "vehicle_id": self.vehicle_id,
                "mode": {"name": "follow", "detector": self.detector, "marker_id": self.marker_id},
                "connection": {"state": connection, "error": error,
                               "protocol_version": None if info is None else info.proto_version,
                               "protocol_state": protocol_state}, "pico": pico,
                "perception": {"age_s": age if age is not None and math.isfinite(age) and age >= 0 else None,
                               "timestamp_source": timestamp_source,
                               "age_basis": "sensor_timestamp" if timestamp_source == "sensor-boottime"
                               else "request_start_or_unknown",
                               "fresh": image_fresh, "discarded": None if step is None else step.discarded,
                               "controller_state": step.state if image_fresh else None,
                               "target": target, "markers": visible},
                "sensors": sensors}
