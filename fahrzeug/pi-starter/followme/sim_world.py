"""Ebene Fahrwelt mit räumlichen Kartons und echter perspektivischer Tagkamera.

ANNAHMEN: ebener Boden, starre Kartons, ideale Lochkamera ohne Verzeichnung,
waagerechte Kamera. Montagewerte im Profil sind keine Prototyp-Kalibrierung.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np

from .geometry import CameraModel
from .markers import DEFAULT_MARKERS
from .protocol import TOF_NO_TARGET
from .virtual_drive import Pose

CAMERA = CameraModel(width_px=960, height_px=540)
PROFILE = json.loads((Path(__file__).with_name("sim_static") / "profile.json").read_text())
CAM_X_M, CAM_Z_M = PROFILE["camera_x_m"], PROFILE["camera_z_m"]
TOF_X_M, TOF_Z_M = PROFILE["tof_x_m"], PROFILE["tof_z_m"]
US = PROFILE["ultrasonic"]  # HC-SR04 schräg vorn; Lage geplant (cad/params.py US_*), nicht gemessen
US_NO_TARGET, US_TOO_CLOSE = -1, 0  # SIM-Werte des Firmware-Simulators ($SIM,US_L/US_R)
SCENARIOS = {
    "folgen": ("Der Person folgen", "Folgt dem Marker. Die Person geht los, hält an und geht weiter."),
    "hindernis": ("Eine Kiste im Weg", "Beobachtet den Frontabstand. Der Pico sperrt die Vorwärtsfahrt."),
    "schilder": ("Person oder Schild?", "ID 100 ist ein Schild. Nur ID 300 darf das Personenziel sein."),
    "schilderweg": ("Schilder am Weg", ("ID 101 steht schon am Start, ID 102 und ID 100 folgen später am Weg. "
                                        "Verbindet jede ID mit einer eigenen Aktion.")),
}
# Schilder je Szene: (ID, x, y). „schilderweg“ dient den Aufgaben „andere Schild-ID“ und „Langsam-Schild“:
# 101 ist ab dem Start sichtbar (darf den Stopp von 100 nicht auslösen), 102 und 100 kommen erst in Fahrt
# ins Bild. ANNAHME: Die Lagen sind frei gewählt und entsprechen keinem realen Parcours.
SIGNS = {
    "schilder": ((100, 1.4, -.65),),
    "schilderweg": ((101, 1.4, -.65), (102, 3.0, .65), (100, 4.6, -.65)),
}


@dataclass
class Box:
    name: str
    x_m: float
    y_m: float
    length_m: float = .6
    width_m: float = .35
    height_m: float = .6

    def bounds(self):
        return (self.x_m - self.length_m / 2, self.y_m - self.width_m / 2, 0), (
            self.x_m + self.length_m / 2, self.y_m + self.width_m / 2, self.height_m)


@dataclass
class Tag:
    marker_id: int
    x_m: float
    y_m: float
    z_m: float = .45
    normal_rad: float = math.pi

    @property
    def size_m(self):
        return DEFAULT_MARKERS[self.marker_id].size_m


@dataclass
class World:
    scenario: str = "folgen"
    boxes: list[Box] = field(default_factory=list)
    signs: list[Tag] = field(default_factory=list)
    target_x_m: float = 1.8
    target_y_m: float = 0.0
    target_hidden: bool = False

    @classmethod
    def create(cls, scenario: str) -> World:
        if scenario not in SCENARIOS:
            raise ValueError("Unbekannte Szene")
        boxes = [Box("Kiste links", 2.9, 1.1), Box("Kiste rechts", 4.2, -.9)]
        if scenario == "hindernis":
            # Niedrige Kiste: Tag bleibt sichtbar, damit gezielt die Frontsperre untersucht wird.
            boxes = [Box("Kiste im Weg", 1.65, 0, height_m=.25), Box("Reserve", 3.5, 1.2)]
        if scenario == "schilderweg":
            # Sichtschutz: 102 und 100 erscheinen erst in Fahrt (etwa ab x = 1,8 m bzw. 3,0 m), nicht am Start.
            boxes = [Box("Sichtschutz links", 2.2, .48, .3, .35, .7), Box("Sichtschutz rechts", 3.8, -.54, .3, .35, .7)]
        signs = [Tag(*sign) for sign in SIGNS.get(scenario, ())]
        world = cls(scenario, boxes, signs)
        world.update_target(0)
        return world

    def update_target(self, elapsed_s: float):
        if self.scenario == "hindernis":
            self.target_x_m, self.target_y_m = 3.2, 0
            return
        # 0–3 s stehen, 3–13 s gehen, 13–17 s warten, danach eine weite Kurve.
        walking = max(0., min(10., elapsed_s - 3)) + max(0., min(10., elapsed_s - 17))
        self.target_x_m = 1.8 + .18 * walking
        self.target_y_m = .35 * math.sin(max(0., elapsed_s - 17) * .12)

    def tags(self) -> list[Tag]:
        person = [] if self.target_hidden else [Tag(300, self.target_x_m, self.target_y_m)]
        return [*self.signs, *person]

    def solids(self) -> list[Box]:
        return [*self.boxes, Box("Person", self.target_x_m + .045, self.target_y_m, .06, .4, 1.25)]

    def view(self):
        return {"scenario": self.scenario, "boxes": [asdict(b) for b in self.boxes],
                "tags": [{**asdict(t), "size_m": t.size_m} for t in self.tags()],
                "target": {"x_m": self.target_x_m, "y_m": self.target_y_m, "hidden": self.target_hidden}}


@lru_cache(maxsize=32)
def tag_image(marker_id: int) -> np.ndarray:
    """Produktionsfamilie, schwarzes Quadrat mit ungestörtem weißen Rand."""
    import cv2

    if marker_id not in DEFAULT_MARKERS:
        raise ValueError("Unbekannte Marker-ID")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    generate = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    return cv2.copyMakeBorder(generate(dictionary, marker_id, 240), 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)


def _rays(pose: Pose, cam: CameraModel):
    u, v = np.meshgrid(np.arange(cam.width_px), np.arange(cam.height_px))
    left = -(u - cam.width_px / 2) / cam.fx
    up = -(v - cam.height_px / 2) / cam.fy
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    return np.stack((c - left * s, s + left * c, up), axis=-1)


def _box_depth(origin, rays, box: Box):
    low, high = box.bounds()
    near = np.full(rays.shape[:2], -np.inf)
    far = np.full_like(near, np.inf)
    for axis in range(3):
        direction = rays[..., axis]
        parallel = np.abs(direction) < 1e-9
        safe = np.where(parallel, 1., direction)
        t0, t1 = (low[axis] - origin[axis]) / safe, (high[axis] - origin[axis]) / safe
        inside = low[axis] <= origin[axis] <= high[axis]
        near = np.maximum(near, np.where(parallel, -np.inf if inside else np.inf, np.minimum(t0, t1)))
        far = np.minimum(far, np.where(parallel, np.inf if inside else -np.inf, np.maximum(t0, t1)))
    return np.where((far >= near) & (near > .025), near, np.inf)


def render(world: World, pose: Pose, cam: CameraModel = CAMERA) -> np.ndarray:
    """RGB-Bild mit Tiefenverdeckung; derselbe Frame geht an den echten Detektor."""
    import cv2

    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    origin = np.array([pose.x_m + CAM_X_M * c, pose.y_m + CAM_X_M * s, CAM_Z_M])
    rays = _rays(pose, cam)
    frame = np.full((cam.height_px, cam.width_px, 3), (235, 229, 218), dtype=np.uint8)
    depth = np.full((cam.height_px, cam.width_px), np.inf)
    ground = rays[..., 2] < -1e-6
    floor_t = -origin[2] / np.where(ground, rays[..., 2], -1)
    floor = ground & (floor_t < 20)
    depth[floor] = floor_t[floor]
    frame[floor] = (215, 222, 226)
    gx = origin[0] + floor_t * rays[..., 0]
    gy = origin[1] + floor_t * rays[..., 1]
    grid = floor & ((np.abs(gx - np.rint(gx)) < .012) | (np.abs(gy - np.rint(gy)) < .012))
    frame[grid] = (186, 199, 205)
    for box in world.solids():
        candidate = _box_depth(origin, rays, box)
        mask = candidate < depth
        frame[mask] = (169, 185, 199) if box.name == "Person" else (118, 158, 188)
        depth[mask] = candidate[mask]
    for tag in world.tags():
        center = np.array([tag.x_m, tag.y_m, tag.z_m])
        normal = np.array([math.cos(tag.normal_rad), math.sin(tag.normal_rad), 0.])
        if np.dot(normal, origin - center) <= 0:
            continue
        tangent = np.array([math.sin(tag.normal_rad), -math.cos(tag.normal_rad), 0.])
        half = tag.size_m * (320 / 240) / 2
        vertices = np.array([center + tangent * half + [0, 0, half],
                             center - tangent * half + [0, 0, half],
                             center - tangent * half - [0, 0, half],
                             center + tangent * half - [0, 0, half]])
        rel = vertices - origin
        forward, left = rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c
        if np.any(forward < .025):
            continue
        points = np.column_stack((cam.width_px / 2 - cam.fx * left / forward,
                                  cam.height_px / 2 - cam.fy * rel[:, 2] / forward)).astype(np.float32)
        image = tag_image(tag.marker_id)
        matrix = cv2.getPerspectiveTransform(np.array([[0, 0], [319, 0], [319, 319], [0, 319]], np.float32), points)
        warped = cv2.warpPerspective(image, matrix, (cam.width_px, cam.height_px), flags=cv2.INTER_NEAREST)
        polygon = np.zeros(depth.shape, np.uint8)
        cv2.fillConvexPoly(polygon, np.rint(points).astype(np.int32), 1)
        divisor = rays @ normal
        plane_t = np.dot(center - origin, normal) / np.where(np.abs(divisor) > 1e-9, divisor, 1e-9)
        mask = (polygon != 0) & (plane_t > .025) & (plane_t <= depth + .001)
        frame[mask] = warped[mask, None]
        depth[mask] = plane_t[mask]
    return frame


def front_distance(world: World, pose: Pose) -> int:
    """Neun Strahlen im angenommenen 27°-Sichtkegel, höchstens vier Meter."""
    origin = (pose.x_m + TOF_X_M * math.cos(pose.yaw_rad), pose.y_m + TOF_X_M * math.sin(pose.yaw_rad), TOF_Z_M)
    angles = pose.yaw_rad + np.linspace(-math.radians(13.5), math.radians(13.5), 9)
    rays = np.stack((np.cos(angles), np.sin(angles), np.zeros_like(angles)), axis=-1)[None, ...]
    closest = min(float(np.min(_box_depth(origin, rays, box))) for box in world.solids())
    return max(0, round(closest * 1000)) if closest <= 4 else TOF_NO_TARGET


def side_distance(world: World, pose: Pose, side: str) -> int:
    """Ultraschall schräg vorn links/rechts: fünf Strahlen im angenommenen Öffnungswinkel.

    Rückgabe im Format von $SIM,US_L/US_R: Abstand in mm, 0 = zu nah, -1 = kein Ziel. ANNAHME: idealer
    Kegel; echte Ultraschallsensoren sehen schräge glatte Flächen und weiche Kleidung schlechter.
    """
    if side not in ("links", "rechts"):
        raise ValueError("Ultraschall gibt es nur links und rechts")
    sign = 1.0 if side == "links" else -1.0
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    lx, ly = US["x_m"], sign * US["y_m"]
    origin = (pose.x_m + lx * c - ly * s, pose.y_m + lx * s + ly * c, US["z_m"])
    half = math.radians(US["cone_deg"] / 2)
    angles = pose.yaw_rad + sign * math.radians(US["yaw_deg"]) + np.linspace(-half, half, 5)
    rays = np.stack((np.cos(angles), np.sin(angles), np.zeros_like(angles)), axis=-1)[None, ...]
    closest = min(float(np.min(_box_depth(origin, rays, box))) for box in world.solids())
    if closest > US["max_m"]:
        return US_NO_TARGET
    return US_TOO_CLOSE if closest < US["min_m"] else round(closest * 1000)


def collision(world: World, pose: Pose, footprint: dict) -> str | None:
    """Trennachsenprüfung der gesamten gedrehten Grundfläche gegen Kartons/Person."""
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    axes = np.array([[1., 0.], [0., 1.], [c, s], [-s, c]])
    corners = np.array([[-footprint["rear_m"], -footprint["half_width_m"]],
                        [footprint["front_m"], -footprint["half_width_m"]],
                        [footprint["front_m"], footprint["half_width_m"]],
                        [-footprint["rear_m"], footprint["half_width_m"]]])
    robot = corners @ np.array([[c, s], [-s, c]]) + [pose.x_m, pose.y_m]
    for box in world.solids():
        low, high = box.bounds()
        obstacle = np.array([[low[0], low[1]], [high[0], low[1]], [high[0], high[1]], [low[0], high[1]]])
        a, b = robot @ axes.T, obstacle @ axes.T
        if np.all((a.max(axis=0) >= b.min(axis=0)) & (b.max(axis=0) >= a.min(axis=0))):
            return box.name
    return None
