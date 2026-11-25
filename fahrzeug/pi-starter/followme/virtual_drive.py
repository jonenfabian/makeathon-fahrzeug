"""Virtuelle Probefahrt mit Bildern und echter Pico-Firmwarelogik.

Ein gezeichneter ArUco-Marker wird vom normalen Erkenner gelesen. Regler,
Zielauswahl und Sollwert-Sender sprechen über das echte Zeilenprotokoll
mit dem Host-Simulator. Dessen Encoderstände bewegen die virtuelle Kamera.
Die Firmwareparameter kommen aus INFO, nicht aus einer zweiten Konfiguration.

ANNAHMEN: schlupffreier ebener Boden, Marker immer frontal zur Kamera,
scharf ohne Rauschen gezeichnet, Lochkameramodell ohne Verzeichnung, Kamera
8 cm vor der Achse. Motor-PT1 und Encoder stammen aus
host_sim/wheel_plant.h. Pi-Bildrate und Latenz kommen aus sim2d.PROFILES;
sie sind noch keine Hardwaremessung.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from itertools import pairwise
from pathlib import Path

import numpy as np

from .chaotic_assessment import CHAOTIC_PATH, assess_phases
from .controller import SEARCH_PATTERNS, FollowController, FollowParams
from .detect import ArucoDetector, Target, TargetSelector
from .drive_assessment import CAMERA_OFFSET_M, assess
from .geometry import CameraModel
from .link import McuLink, SetpointSender, open_sim
from .protocol import FLAG_ESTOP, FLAG_TOF_BLOCK, FLAG_TOF_FAIL, FLAG_WDG, TOF_NO_TARGET, Info, Status, cmd_line
from .search_trials import SEARCH_PATHS
from .sim2d import PROFILES

FAULTS = ("keine", "bildverlust", "bildschleife", "pi", "notaus", "abstand", "sensor")

PERSON_PATHS = {
    **SEARCH_PATHS,
    "chaotisch": CHAOTIC_PATH,
    "halt": ((0., 2., .2), (2.5, 3., .2)),
    "rueckkehr": ((0., 2., .2), (2.5, 3., .2), (4., 3., .2), (6.5, 2.2, .2)),
}
SCENARIOS = ("gerade", *PERSON_PATHS)


def person_position(now_s: float, scenario: str) -> tuple[float, float]:
    """Geplanter Personenweg; die Fahrzeugbewegung wird daraus nicht vorgegeben."""
    if scenario == "gerade":
        return 2.0 + 0.35 * now_s, 0.2
    if scenario not in PERSON_PATHS:
        raise ValueError(f"Unbekannter Personenweg: {scenario}")
    path = PERSON_PATHS[scenario]
    for (t0, x0, y0), (t1, x1, y1) in pairwise(path):
        if now_s <= t1:
            f = max(0.0, (now_s - t0) / (t1 - t0))
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
    return path[-1][1:]


class CommandRecorder:
    """Protokolliert erfolgreiche Sendungen mit deren echter Folgenummer."""

    def __init__(self, link: McuLink):
        self.link = link
        self._lock = threading.Lock()
        self._last: str | None = None

    def send_cmd(self, v_mps: float, omega_rad_s: float) -> int:
        seq = self.link.send_cmd(v_mps, omega_rad_s)
        with self._lock:
            self._last = cmd_line(seq, v_mps, omega_rad_s).strip()
        return seq

    def latest(self) -> str | None:
        with self._lock:
            return self._last


@dataclass
class Pose:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def advance(self, before: Status, after: Status, info: Info) -> None:
        """Differentialantrieb, mittlerer Kurs zwischen zwei Encoderständen."""
        m_per_tick = math.pi * info.wheel_diameter_mm / (1000 * info.counts_per_rev)
        left_m = (after.ticks_l - before.ticks_l) * m_per_tick
        right_m = (after.ticks_r - before.ticks_r) * m_per_tick
        angle_rad = (right_m - left_m) / (info.track_mm / 1000)
        distance_m = (left_m + right_m) / 2
        # Exakte Kreisbogensehne, stabil auch bei Geradeausfahrt.
        half = angle_rad / 2
        chord_m = distance_m * (math.sin(half) / half if abs(half) > 1e-9 else 1.0)
        self.x_m += chord_m * math.cos(self.yaw_rad + half)
        self.y_m += chord_m * math.sin(self.yaw_rad + half)
        self.yaw_rad += angle_rad

    def target(self, x_m: float, y_m: float) -> tuple[float, float]:
        cx = self.x_m + CAMERA_OFFSET_M * math.cos(self.yaw_rad)
        cy = self.y_m + CAMERA_OFFSET_M * math.sin(self.yaw_rad)
        rel = math.atan2(y_m - cy, x_m - cx) - self.yaw_rad
        return math.hypot(x_m - cx, y_m - cy), math.atan2(math.sin(rel), math.cos(rel))


RENDERERS = ("flaechentreu", "ganzzahlig")


def render_marker(cam: CameraModel, distance_m: float, bearing_rad: float, visible: bool = True,
                  renderer: str = "ganzzahlig") -> np.ndarray:
    """Frontaler 15-cm-Marker ID 7 auf hellem Grund; außerhalb des Bilds abgeschnitten.

    ganzzahlig (Standard): gerundete Kante/Mitte; bei nicht durch 6 teilbarer
    Kante ungleiches Zellraster, 8/11 px nie erkennbar (docs/11 §27).
    flaechentreu (nur Diagnose, nicht abgenommen, docs/11 §28): Kante und Mitte
    ungerundet, jedes Pixel erhält den exakten Flächenanteil der 6×6 Zellen
    (Pixelmitte = ganze Zahl wie bei den OpenCV-Ecken), danach 8-bit gerundet.
    """
    import cv2

    if renderer not in RENDERERS:
        raise ValueError(f"Unbekannter Marker-Renderer: {renderer}")
    image = np.full((cam.height_px, cam.width_px, 3), 255, dtype=np.uint8)
    if not visible or distance_m <= 0 or abs(bearing_rad) >= math.pi / 2:
        return image
    side_px = round(cam.fx * 0.15 / distance_m)
    if side_px < 8 or side_px > 4 * cam.width_px:
        return image
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    generate = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    if renderer == "flaechentreu":
        side = cam.fx * 0.15 / distance_m
        center_x, center_y = cam.width_px / 2 - cam.fx * math.tan(bearing_rad), cam.height_px / 2
        cols = np.arange(max(0, math.floor(center_x - side / 2 - .5)),
                         min(cam.width_px, math.ceil(center_x + side / 2 + .5)))
        rows = np.arange(max(0, math.floor(center_y - side / 2 - .5)),
                         min(cam.height_px, math.ceil(center_y + side / 2 + .5)))
        if not len(cols) or not len(rows):
            return image
        cells = generate(dictionary, 7, 6).astype(float) / 255  # eine Zelle je Pixel, 1 = weiß
        edges = np.arange(7) * side / 6

        def overlap(pixels, start):
            low = np.maximum(pixels[:, None] - .5, start + edges[None, :-1])
            return np.clip(np.minimum(pixels[:, None] + .5, start + edges[None, 1:]) - low, 0, None)

        ox, oy = overlap(cols, center_x - side / 2), overlap(rows, center_y - side / 2)
        gray = 1 - np.outer(oy.sum(axis=1), ox.sum(axis=1)) + oy @ cells @ ox.T
        image[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1] = np.rint(gray * 255).clip(0, 255).astype(np.uint8)[..., None]
        return image
    center_px = round(cam.width_px / 2 - cam.fx * math.tan(bearing_rad))
    x0, y0 = center_px - side_px // 2, (cam.height_px - side_px) // 2
    x1, y1 = x0 + side_px, y0 + side_px
    left, top = max(0, x0), max(0, y0)
    right, bottom = min(cam.width_px, x1), min(cam.height_px, y1)
    if left >= right or top >= bottom:
        return image
    marker = cv2.cvtColor(generate(dictionary, 7, side_px), cv2.COLOR_GRAY2BGR)
    image[top:bottom, left:right] = marker[top - y0:bottom - y0, left - x0:right - x0]
    return image


@dataclass
class DriveResult:
    fault: str
    profile: str
    firmware: dict
    scenario: str = "gerade"
    samples: list[dict] = field(default_factory=list)
    checks: dict[str, bool] = field(default_factory=dict)
    camera_offset_m: float = CAMERA_OFFSET_M
    assessment: dict = field(default_factory=dict)
    duration_s: float = 0.
    occlusion_window_s: tuple[float, float] | None = None
    phase_assessment: dict | None = None
    mirrored: bool = False
    search_pattern: str = "einseitig"
    renderer: str = "ganzzahlig"

    @property
    def passed(self) -> bool:
        return bool(self.checks) and all(self.checks.values())


def run(*, fault: str = "keine", duration_s: float = 5.0, fault_after_s: float = 2.0,
        profile_name: str = "pi3b_aruco", scenario: str = "gerade",
        occlusion_window_s: tuple[float, float] | None = None,
        params: FollowParams | None = None, mirror: bool = False,
        renderer: str = "ganzzahlig") -> DriveResult:
    """Echtzeitfahrt; Störung bleibt ab fault_after_s bis zum Ende aktiv.

    Nur Host-Simulator: Diese Funktion öffnet keine serielle Schnittstelle.
    Die Zielperson geht geradeaus oder folgt dem unregelmäßigen Zeitplan.
    """
    if fault not in FAULTS:
        raise ValueError(f"Unbekannte Störung: {fault}")
    if scenario not in SCENARIOS:
        raise ValueError(f"Unbekannter Personenweg: {scenario}")
    if not math.isfinite(duration_s) or not math.isfinite(fault_after_s) or not 1 <= fault_after_s <= duration_s - 2:
        raise ValueError("Mindestens 1 s Anfahrt und 2 s nach Störbeginn vorsehen.")
    # Nur das ArUco-Profil passt zum gezeichneten Marker; keine Behauptung,
    # das Personen-Netz oder die Farberkennung würden hier mitgeprüft.
    if profile_name != "pi3b_aruco":
        raise ValueError("Virtuelle Markerfahrt benötigt das Profil pi3b_aruco.")
    if type(mirror) is not bool:
        raise ValueError("Spiegelung muss boolesch sein")
    if renderer not in RENDERERS:
        raise ValueError(f"Unbekannter Marker-Renderer: {renderer}")
    if occlusion_window_s is not None:
        start, end = occlusion_window_s
        if not (math.isfinite(start) and math.isfinite(end) and 0 <= start < end <= duration_s):
            raise ValueError("Endliches, geordnetes Bildpausenfenster innerhalb der Fahrt erforderlich.")
    config = PROFILES[profile_name]
    cam = CameraModel()
    detector = ArucoDetector(cam)
    selector = TargetSelector()
    controller = FollowController(params)
    pose = Pose()
    pending: deque[tuple[float, list[Target]]] = deque()

    with open_sim() as link:
        link.wait_status(lambda _: link.info is not None, timeout_s=3.0)
        info = link.info
        previous = link.status
        assert info is not None and previous is not None
        result = DriveResult(fault, profile_name, asdict(info), scenario=scenario,
                             duration_s=duration_s, occlusion_window_s=occlusion_window_s,
                             mirrored=mirror, search_pattern=controller.p.search_pattern, renderer=renderer)
        recorder = CommandRecorder(link)
        sender = SetpointSender(recorder).start()
        try:
            link.send_sim("TOF", 3000)
            link.arm()
            link.wait_status(lambda s: s.state == "RUN")
            start_s = time.monotonic()
            next_frame_s = 0.0
            injected = False
            while (now_s := time.monotonic() - start_s) < duration_s:
                if link.status_age_s > 0.5:
                    raise TimeoutError("Virtueller Pico liefert keinen frischen Status.")
                status = link.status
                assert status is not None
                pose.advance(previous, status, info)
                previous = status
                target_x_m, target_y_m = person_position(now_s, scenario)
                if mirror:
                    target_y_m = -target_y_m
                distance_m, bearing_rad = pose.target(target_x_m, target_y_m)
                active = fault != "keine" and now_s >= fault_after_s
                if active and not injected:
                    if fault == "pi":
                        sender.stop()  # Ganzer Pi-Prozess schweigt, Status wird weiter beobachtet.
                    elif fault == "notaus":
                        link.send_sim("ESTOP", 1)
                    elif fault == "sensor":
                        link.send_sim("TOFFAIL", 1)
                    injected = True
                tof_mm = (round(distance_m * 1000) if abs(bearing_rad) < math.radians(13.5)
                          else TOF_NO_TARGET)
                link.send_sim("TOF", 300 if active and fault == "abstand" else min(tof_mm, TOF_NO_TARGET))
                seen = False
                perception_updates = target_updates = 0
                if not (active and fault == "bildschleife"):
                    if now_s >= next_frame_s:
                        next_frame_s = now_s + 1 / config["frame_hz"]
                        hidden = (active and fault == "bildverlust" or occlusion_window_s is not None
                                  and occlusion_window_s[0] <= now_s < occlusion_window_s[1])
                        frame = render_marker(cam, distance_m, bearing_rad, not hidden, renderer)
                        targets = detector.detect(frame)
                        pending.append((now_s + config["cam_latency_s"], targets))
                    while pending and pending[0][0] <= now_s:
                        targets = pending.popleft()[1]
                        if controller.target_lost_long:
                            selector.reset()
                        chosen = selector.select(targets)
                        perception_updates += 1
                        target_updates += int(chosen is not None)
                        v_mps, omega_rad_s = controller.update(chosen, time.monotonic())
                        sender.set(v_mps, omega_rad_s)
                        seen = seen or chosen is not None
                result.samples.append({
                    "t_s": now_s, **asdict(pose), "target_x_m": target_x_m, "target_y_m": target_y_m,
                    "distance_m": distance_m, "seen": seen, "controller": controller.state,
                    "perception_updates": perception_updates, "target_updates": target_updates,
                    "mcu": status.state, "flags": status.flags, "v_l_mm_s": status.v_l_mm_s,
                    "v_r_mm_s": status.v_r_mm_s, "stale_setpoint": sender.current()[2],
                    "cmd_line": recorder.latest(), "ack_seq": status.last_seq,
                })
                time.sleep(max(0, start_s + now_s + 0.02 - time.monotonic()))
            before = [s for s in result.samples if s["t_s"] < fault_after_s]
            tail = [s for s in result.samples if s["t_s"] >= duration_s - 0.4]
            result.checks = {
                "marker_erkannt": sum(s["seen"] for s in before) >= 3,
                "angefahren": any(min(s["v_l_mm_s"], s["v_r_mm_s"]) > 100 for s in before),
                "abstand_gehalten": min(s["distance_m"] for s in result.samples) >= 0.6,
                "protokoll_fehlerfrei": link.bad_lines == 0 and link.status.rx_errors == 0,
            }
            if fault == "keine":
                result.checks["folgt_weiter"] = all(s["mcu"] == "RUN" for s in tail) and tail[-1]["x_m"] > 0.5
                result.checks["folgeabstand"] = 1.2 <= tail[-1]["distance_m"] <= 2.3
            else:
                # Bei Bildverlust darf der Regler später auf der Stelle suchen.
                result.checks["vorwaertsfahrt_beendet"] = all(
                    abs(s["v_l_mm_s"] + s["v_r_mm_s"]) / 2 < 40 for s in tail)
                if fault != "bildverlust":
                    result.checks["beide_raeder_stehen"] = all(
                        max(abs(s["v_l_mm_s"]), abs(s["v_r_mm_s"])) < 40 for s in tail)
                expected = {"pi": ("STANDBY", FLAG_WDG), "notaus": ("ESTOP", FLAG_ESTOP),
                            "abstand": ("RUN", FLAG_TOF_BLOCK), "sensor": ("RUN", FLAG_TOF_FAIL)}
                if fault in expected:
                    state, flag = expected[fault]
                    result.checks["passende_reaktion"] = all(s["mcu"] == state and s["flags"] & flag for s in tail)
                elif fault == "bildschleife":
                    result.checks["sollwert_veraltet"] = all(s["stale_setpoint"] for s in tail)
                elif fault == "bildverlust":
                    result.checks["zielverlust_erkannt"] = all(s["controller"] != "FOLGEN" for s in tail)
                if fault in ("bildschleife", "bildverlust"):
                    result.checks["sender_haelt_verbindung"] = all(
                        s["mcu"] == "RUN" and not s["flags"] & FLAG_WDG for s in tail)
        finally:
            sender.stop()
            link.stop()
    # Erst nach Sender-/Simulatorende: keine Offline-Analyse im aktiven Fahrkreis.
    mode = scenario if scenario in ("halt", "rueckkehr") else "beobachten"
    result.assessment = assess({"samples": result.samples, "fault": fault, "checks": result.checks,
                                "camera_offset_m": result.camera_offset_m}, mode=mode)
    if scenario == "chaotisch":
        result.phase_assessment = assess_phases(asdict(result))
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description="Virtuelle Probefahrt: Markerbild → Erkenner → Regler → Pico-Firmware")
    ap.add_argument("--stoerung", choices=FAULTS, default="keine")
    ap.add_argument("--szenario", choices=SCENARIOS, default="gerade")
    ap.add_argument("--sekunden", type=float, default=5.0)
    ap.add_argument("--stoerung-ab", type=float, default=2.0, help="Sekunden nach Fahrtbeginn")
    ap.add_argument("--ausgabe", type=Path, help="JSON mit Firmwareparametern, Verlauf und Prüfergebnissen")
    ap.add_argument("--bildpause-von", type=float, help="Temporäre Marker-Verdeckung, nur Host-Simulator")
    ap.add_argument("--bildpause-bis", type=float)
    ap.add_argument("--suchstrategie", choices=SEARCH_PATTERNS, help="Expliziter A/B-Kandidat, sonst Referenzstandard")
    ap.add_argument("--spiegeln", action="store_true", help="Personenweg links/rechts spiegeln")
    args = ap.parse_args()
    if (args.bildpause_von is None) != (args.bildpause_bis is None):
        ap.error("Beide Bildpausen-Grenzen angeben")
    window = (args.bildpause_von, args.bildpause_bis) if args.bildpause_von is not None else None
    result = run(fault=args.stoerung, duration_s=args.sekunden, fault_after_s=args.stoerung_ab,
                 scenario=args.szenario, occlusion_window_s=window, mirror=args.spiegeln,
                 params=FollowParams(search_pattern=args.suchstrategie) if args.suchstrategie else None)
    for name, passed in result.checks.items():
        print(f"{'OK' if passed else 'FEHLER'}: {name}")
    if args.ausgabe:
        args.ausgabe.write_text(json.dumps(asdict(result), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"Verlauf: {args.ausgabe}")
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
