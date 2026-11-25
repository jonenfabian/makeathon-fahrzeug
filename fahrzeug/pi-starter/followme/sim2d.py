"""2D-Simulator: Person läuft einen Weg ab, das Fahrzeug folgt.

Zweck: Den Folgeregler (controller.py) ohne Hardware entwickeln und
vergleichen — auch während des Makeathons, solange der Akku lädt.

Was nachgebildet ist (und wo die Zahlen herkommen):
  * Fahrzeug: Einradmodell (Position x, y, Kurs θ) mit den Grenzen und
    Rampen der Pico-Firmware (config.h: 1,2 m/s, 1,5 m/s² / 3 m/s²) und
    einer Verzögerung 1. Ordnung (ANNAHME τ = 0,15 s für den Radregler).
  * Abstandssperre des Pico: Vorwärtsfahrt gesperrt, wenn die Person
    innerhalb ±13,5° (halber Sichtwinkel VL53L1X, 27°) näher als 0,40 m vor
    dem Sensor steht.
  * Kamera + Erkennung auf dem Raspberry Pi 3B: 102° Bildwinkel,
    Reichweite 8 m, Rauschen, verpasste Bilder, Bildrate und Verzögerung je
    Erkenner als Profil (PROFILES unten). Der Regler rechnet nur, wenn ein
    Bild fertig ist; dazwischen hält der Sollwert-Sender den letzten Wert
    (älter als 0,6 s ⇒ 0, wie SetpointSender in link.py).
    ANNAHMEN — Bildraten mit tools/benchmark.py am Fahrzeug messen und hier
    eintragen; Rauschen mit einem Mitschnitt prüfen.
Nicht nachgebildet: Verwechslung mit anderen Personen, Verdeckung.

Aufruf:
    python -m followme.sim2d --szenario rechteck --profil pi3b_person --plot sim.png
"""

from __future__ import annotations

import argparse
import math
import random
from collections import deque
from dataclasses import dataclass, field

from .controller import FollowController, FollowParams
from .detect import Target


@dataclass
class SimConfig:
    dt_s: float = 0.01  # Rechenschritt der Fahrzeugphysik
    frame_hz: float = 10.0  # fertig erkannte Bilder pro Sekunde
    cam_latency_s: float = 0.15  # Belichtung bis Sollwert
    bearing_noise_rad: float = 0.02
    dist_noise_rel: float = 0.08
    dropout_prob: float = 0.05
    hfov_deg: float = 102.0
    max_range_m: float = 8.0
    cam_offset_m: float = 0.08  # Kamera vor der Achsmitte
    setpoint_max_age_s: float = 0.6  # wie SetpointSender.max_age_s
    tau_s: float = 0.15
    v_max_mps: float = 1.2  # wie V_MAX_MM_S
    omega_max_rad_s: float = 4.0  # wie OMEGA_MAX_MRAD_S
    acc_mps2: float = 1.5  # wie ACC_MAX_MM_S2
    dec_mps2: float = 3.0  # wie DEC_MAX_MM_S2
    alpha_max_rad_s2: float = 12.0  # wie ALPHA_MAX_MRAD_S2
    tof_stop_m: float = 0.40  # wie TOF_STOP_MM
    tof_half_fov_rad: float = math.radians(13.5)
    seed: int = 1


# Erkenner-Profile auf dem Raspberry Pi 3B. ANNAHMEN (Schätzung aus der
# Rechenleistung, siehe docs/06-software.md §5) — durch Messwerte aus
# tools/benchmark.py ersetzen.
PROFILES: dict[str, dict] = {
    "pi3b_aruco": {"frame_hz": 10.0, "cam_latency_s": 0.15},
    "pi3b_farbe": {"frame_hz": 12.0, "cam_latency_s": 0.12, "dist_noise_rel": 0.15},
    "pi3b_person": {"frame_hz": 3.0, "cam_latency_s": 0.40, "dist_noise_rel": 0.12, "dropout_prob": 0.10},
}


def profile(name: str, **overrides: float) -> SimConfig:
    """SimConfig für ein Erkenner-Profil (plus Einzelwerte)."""
    return SimConfig(**{**PROFILES[name], **overrides})


# Wege der Person: Liste von (x, y) in m; Geschwindigkeit; Pausen (Index → s).
SCENARIOS: dict[str, dict] = {
    "gerade": {"path": [(2.0, 0.0), (12.0, 0.0)], "speed": 1.0, "pauses": {1: 3.0}},
    "rechteck": {
        "path": [(2.0, 0.0), (8.0, 0.0), (8.0, 4.0), (2.0, 4.0), (2.0, 0.5)],
        "speed": 1.0,
        "pauses": {2: 2.0, 4: 3.0},
    },
    "slalom": {
        "path": [(2.0 + 0.5 * i, 1.2 * math.sin(i * 0.6)) for i in range(25)],
        "speed": 1.1,
        "pauses": {},
    },
    # Person geht weg, bleibt stehen, tritt 1 m zur Seite und geht am
    # Fahrzeug vorbei zurück — prüft, dass das Fahrzeug ihr nicht in den Weg
    # fährt, sondern sich auf der Stelle nach ihr umdreht.
    "begegnung": {
        "path": [(2.0, 0.0), (6.0, 0.0), (6.0, 1.0), (-3.0, 1.0)],
        "speed": 1.0,
        "pauses": {1: 2.0},
    },
}


@dataclass
class Person:
    path: list[tuple[float, float]]
    speed: float
    pauses: dict[int, float] = field(default_factory=dict)

    def position(self, t: float) -> tuple[float, float]:
        """Ort der Person zur Zeit t (läuft die Polylinie mit Pausen ab)."""
        x, y = self.path[0]
        for i in range(1, len(self.path)):
            x1, y1 = self.path[i]
            seg = math.hypot(x1 - x, y1 - y)
            dur = seg / self.speed
            if t < dur:
                f = t / dur
                return x + f * (x1 - x), y + f * (y1 - y)
            t -= dur
            x, y = x1, y1
            pause = self.pauses.get(i, 0.0)
            if t < pause:
                return x, y
            t -= pause
        return x, y

    def duration(self) -> float:
        total = sum(self.pauses.values())
        for (x0, y0), (x1, y1) in zip(self.path, self.path[1:]):
            total += math.hypot(x1 - x0, y1 - y0) / self.speed
        return total


@dataclass
class SimResult:
    t: list[float] = field(default_factory=list)
    robot: list[tuple[float, float, float]] = field(default_factory=list)
    person: list[tuple[float, float]] = field(default_factory=list)
    dist: list[float] = field(default_factory=list)
    states: list[str] = field(default_factory=list)
    seen: list[bool] = field(default_factory=list)

    @property
    def min_distance_m(self) -> float:
        return min(self.dist)

    @property
    def mean_follow_distance_m(self) -> float:
        d = [x for x, s in zip(self.dist, self.states) if s == "FOLGEN"]
        return sum(d) / len(d) if d else float("nan")

    @property
    def lost_fraction(self) -> float:
        """Anteil der Zeit in DREHEN/VERLOREN (nach dem ersten Sehen)."""
        if "FOLGEN" not in self.states:
            return 1.0
        rest = self.states[self.states.index("FOLGEN") :]
        return sum(s in ("DREHEN", "VERLOREN") for s in rest) / len(rest)


def run(scenario: str, params: FollowParams | None = None, cfg: SimConfig | None = None,
        extra_time_s: float = 4.0) -> SimResult:
    """Simuliert ein Szenario mit dem Folgeregler und gibt den Verlauf zurück."""
    cfg = cfg or SimConfig()
    rng = random.Random(cfg.seed)
    sc = SCENARIOS[scenario]
    person = Person(sc["path"], sc["speed"], sc["pauses"])
    ctrl = FollowController(params)
    res = SimResult()

    x = y = th = 0.0  # Fahrzeug startet im Ursprung, Blick in +x
    v_ramp = w_ramp = 0.0  # Ausgang der Firmware-Rampe
    v = w = 0.0  # tatsächliche Geschwindigkeiten (nach PT1)
    pending: deque[tuple[float, Target | None]] = deque()  # (fertig um t, Messung)
    v_cmd = w_cmd = 0.0  # letzter Sollwert des Reglers
    t_cmd = -1e9  # Zeitpunkt des letzten Reglerschritts
    next_frame = 0.0
    t = 0.0
    t_end = person.duration() + extra_time_s
    while t < t_end:
        px, py = person.position(t)
        cx, cy = x + cfg.cam_offset_m * math.cos(th), y + cfg.cam_offset_m * math.sin(th)
        rel = math.atan2(py - cy, px - cx) - th
        bearing = math.atan2(math.sin(rel), math.cos(rel))
        d_cam = math.hypot(px - cx, py - cy)

        # --- Kamera: Bild aufnehmen; Ergebnis ist cam_latency_s später fertig --
        if t >= next_frame:
            next_frame += 1.0 / cfg.frame_hz
            visible = abs(bearing) < math.radians(cfg.hfov_deg / 2) and d_cam < cfg.max_range_m
            meas = None
            if visible and rng.random() > cfg.dropout_prob:
                meas = Target(
                    bearing_rad=bearing + rng.gauss(0.0, cfg.bearing_noise_rad),
                    distance_m=d_cam * (1.0 + rng.gauss(0.0, cfg.dist_noise_rel)),
                    score=1.0,
                    box=(0.0, 0.0, 0.0, 0.0),
                )
            pending.append((t + cfg.cam_latency_s, meas))

        # --- Pi: Regler rechnet, sobald eine Erkennung fertig ist ---------------
        seen_now = False
        while pending and pending[0][0] <= t + 1e-9:
            meas = pending.popleft()[1]
            v_cmd, w_cmd = ctrl.update(meas, t)
            t_cmd = t
            seen_now = seen_now or meas is not None
        # Sollwert-Sender: veralteter Sollwert ⇒ 0 (siehe SetpointSender).
        stale = t - t_cmd > cfg.setpoint_max_age_s
        v_tx, w_tx = (0.0, 0.0) if stale else (v_cmd, w_cmd)

        # --- Pico: Grenzen, Abstandssperre, Rampen -----------------------------
        v_set = max(-cfg.v_max_mps, min(cfg.v_max_mps, v_tx))
        w_set = max(-cfg.omega_max_rad_s, min(cfg.omega_max_rad_s, w_tx))
        d_front = d_cam  # Abstandssensor sitzt (vereinfacht) am Ort der Kamera
        if abs(bearing) < cfg.tof_half_fov_rad and d_front < cfg.tof_stop_m and v_set > 0:
            v_set = 0.0
        slowing = v_ramp != 0 and (v_set - v_ramp > 0) != (v_ramp > 0)
        a = cfg.dec_mps2 if slowing else cfg.acc_mps2
        v_ramp += max(-a * cfg.dt_s, min(a * cfg.dt_s, v_set - v_ramp))
        w_ramp += max(-cfg.alpha_max_rad_s2 * cfg.dt_s, min(cfg.alpha_max_rad_s2 * cfg.dt_s, w_set - w_ramp))

        # --- Fahrzeug: PT1 und Einradmodell -------------------------------------
        k = cfg.dt_s / cfg.tau_s
        v += k * (v_ramp - v)
        w += k * (w_ramp - w)
        x += v * math.cos(th) * cfg.dt_s
        y += v * math.sin(th) * cfg.dt_s
        th += w * cfg.dt_s

        res.t.append(t)
        res.robot.append((x, y, th))
        res.person.append((px, py))
        res.dist.append(math.hypot(px - x, py - y))
        res.states.append(ctrl.state)
        res.seen.append(seen_now)
        t += cfg.dt_s
    return res


def plot(res: SimResult, path: str, title: str = "") -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    ax1.plot([p[0] for p in res.person], [p[1] for p in res.person], label="Person")
    ax1.plot([r[0] for r in res.robot], [r[1] for r in res.robot], label="Fahrzeug")
    ax1.set_aspect("equal")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.legend()
    ax1.set_title(title or "Wege")
    ax2.plot(res.t, res.dist, label="Abstand Achsmitte–Person")
    ax2.axhline(0.4, linestyle="--", linewidth=1, label="Abstandssperre Pico (0,4 m)")
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel("Abstand [m]")
    ax2.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="2D-Simulation des Personenfolgens")
    ap.add_argument("--szenario", choices=sorted(SCENARIOS), default="rechteck")
    ap.add_argument("--plot", help="Bilddatei für Wege und Abstand (PNG)")
    ap.add_argument("--profil", choices=sorted(PROFILES), default="pi3b_aruco")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)
    res = run(args.szenario, cfg=profile(args.profil, seed=args.seed))
    print(f"Szenario {args.szenario} / {args.profil}: min. Abstand {res.min_distance_m:.2f} m, "
          f"mittlerer Folgeabstand {res.mean_follow_distance_m:.2f} m, "
          f"Ziel verloren {100 * res.lost_fraction:.1f} % der Zeit")
    if args.plot:
        plot(res, args.plot, title=args.szenario)
        print(f"Grafik: {args.plot}")


if __name__ == "__main__":
    main()
