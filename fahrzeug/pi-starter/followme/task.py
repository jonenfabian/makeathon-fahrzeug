"""Gemeinsamer Aufgabenvertrag für virtuelle Kamera und echtes Fahrzeug.

Eigener Python-Code wird ausschließlich über die lokale Kommandozeile geladen.
Er liefert Sollwerte; Motorsteuerung und Schutzfunktionen bleiben im Pico.
"""
from __future__ import annotations

import importlib.util
import math
import numbers
import sys
import sysconfig
import traceback
from dataclasses import dataclass, replace
from pathlib import Path

from .detect import Target
from .markers import MarkerObservation
from .pipeline import Step
from .protocol import TOF_NO_TARGET, Status

_PACKAGE_DIR = Path(__file__).resolve().parent
# Bibliotheken und Standardbibliothek: dort liegt nie die Fehlerstelle des Teams.
_LIBRARY_DIRS = tuple({Path(p).resolve() for p in (sys.prefix, sys.base_prefix, sys.exec_prefix,
                                                   *sysconfig.get_paths().values()) if p})


class TaskError(ValueError):
    """Fehler in der eigenen Aufgabendatei; Meldung enthält Datei und Zeile, wenn bekannt."""


def _is_team_file(filename: str) -> bool:
    try:
        path = Path(filename).resolve()
    except (OSError, ValueError):
        return False
    return not filename.startswith("<") and not any(
        path.is_relative_to(folder) for folder in (_PACKAGE_DIR, *_LIBRARY_DIRS))


def describe_error(exc: BaseException) -> str:
    """Kurze Meldung für Dashboard und Terminal: Fehlerart, Text und die Stelle im Teamcode."""
    if isinstance(exc, SyntaxError) and exc.filename:
        text, where = f"{type(exc).__name__}: {exc.msg}", (exc.filename, exc.lineno)
    else:
        text = f"{type(exc).__name__}: {exc}" if str(exc) else type(exc).__name__
        frames = [f for f in traceback.extract_tb(exc.__traceback__) if _is_team_file(f.filename)]
        where = (frames[-1].filename, frames[-1].lineno) if frames else None
    if where is None:
        return text
    return f"{text} ({Path(where[0]).name}, Zeile {where[1]})"


@dataclass(frozen=True)
class Observation:
    captured_s: float
    target: Target | None
    markers: tuple[MarkerObservation, ...]
    controller_state: str
    front_mm: int | None
    front_state: str
    suggested_v_mps: float
    suggested_omega_rad_s: float
    # Ultraschall schräg vorn (HC-SR04, ±45°, ab Firmware-Protokoll 2). Nur Messwerte, der Pico sperrt damit
    # nichts. Zustand: "distance", "too_close" (mm = 0), "no_target", "fault", "stale" oder "unavailable".
    left_mm: int | None = None
    left_state: str = "unavailable"
    right_mm: int | None = None
    right_state: str = "unavailable"


class Task:
    """Basis für eigene Aufgaben: vorhandene Folgeregelung unverändert verwenden."""

    def reset(self) -> None:
        """Eigenen Zustand zurücksetzen. Wird einmal je Start aufgerufen, nie zwischen zwei Bildern."""

    def decide(self, observation: Observation) -> tuple[float, float]:
        return observation.suggested_v_mps, observation.suggested_omega_rad_s


def load_task(path: Path | None) -> Task:
    """Aufgabendatei frisch laden. Jeder Fehler darin wird zu TaskError mit Datei und Zeile."""
    if path is None:
        return Task()
    try:
        path = path.resolve(strict=True)
    except OSError:
        raise TaskError(f"Aufgabendatei nicht gefunden: {path}") from None
    if path.suffix != ".py":
        raise TaskError("Aufgabe muss eine Python-Datei sein")
    spec = importlib.util.spec_from_file_location("followme_student_task", path)
    if spec is None or spec.loader is None:
        raise TaskError("Aufgabe kann nicht geladen werden")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # Dataclasses mit vorwärts referenzierten Typen benötigen den Moduleintrag.
    try:
        # Start lädt die aktuellen Bytes, auch bei gleich großer Änderung im selben Zeitstempel.
        exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)  # noqa: S102 - Bewusst per CLI gewählter Teamcode.
        if not callable(getattr(module, "create_task", None)):
            raise TaskError(f"Funktion create_task() fehlt in {path.name}")
        task = module.create_task()
    except TaskError:
        raise
    except KeyboardInterrupt:
        raise
    except BaseException as exc:  # auch sys.exit() oder eigene BaseException beenden nicht das Programm.
        raise TaskError(describe_error(exc)) from exc
    if not callable(getattr(task, "decide", None)) or not callable(getattr(task, "reset", None)):
        raise TaskError("create_task() muss ein Objekt mit decide() und reset() liefern")
    try:
        task.reset()  # einziger Aufruf: beim Laden, also bei jedem bewussten Start
    except KeyboardInterrupt:
        raise
    except BaseException as exc:
        raise TaskError(describe_error(exc)) from exc
    return task


def _speed(value) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real) or not math.isfinite(value):
        raise TaskError(f"Aufgabe liefert ungültige Geschwindigkeit: {value!r}")
    return float(value)


def apply_task(task: Task, step: Step, markers: list[MarkerObservation], status: Status | None,
               *, status_fresh: bool, sides: dict[str, dict] | None = None) -> Step:
    """Nur gültige Beobachtungen an Aufgabencode geben; Ergebnis endlich prüfen.

    Ein verworfenes Bild (veraltet, Lücke, Bedienung) geht nicht an decide() und setzt die Aufgabe
    nicht zurück. Sonst vergäße etwa ein gemerktes Stoppschild nach einem einzigen schlechten Bild
    seinen Zustand und die Folgeregelung führe wieder an. Die Sollwerte des verworfenen Schritts
    bleiben unverändert (die Bildkette liefert dort Stillstand).

    sides: Leseansicht aus sensors.side_views() (links/rechts), sonst „unavailable“.

    Jeder Fehler im Teamcode kommt als TaskError mit Datei und Zeile heraus. Aufrufer stoppen dann
    die Fahrt und laufen weiter, statt das ganze Programm zu beenden.
    """
    try:
        if step.discarded:
            return step
        front = status.tof_mm if status_fresh and status is not None else None
        state = "unavailable" if front is None else "no_target" if front == TOF_NO_TARGET else "distance"
        left, right = (sides or {}).get("links", {}), (sides or {}).get("rechts", {})
        side_ok = status_fresh  # ohne frische Verbindung keine Seitenwerte als gültig weitergeben
        obs = Observation(step.captured_s, step.chosen, tuple(markers), step.state,
                          front if state == "distance" else None, state, step.v_mps, step.omega_rad_s,
                          left.get("distance_mm") if side_ok else None,
                          left.get("state", "unavailable") if side_ok else "unavailable",
                          right.get("distance_mm") if side_ok else None,
                          right.get("state", "unavailable") if side_ok else "unavailable")
        command = task.decide(obs)
    except KeyboardInterrupt:  # Strg-C im Terminal gehört dem Fahrprogramm, nicht der Aufgabe.
        raise
    except BaseException as exc:  # Fehler im Teamcode als Meldung, nicht als Absturz.
        raise TaskError(describe_error(exc)) from exc
    if not isinstance(command, (tuple, list)) or len(command) != 2:
        raise TaskError(f"decide() muss (v_mps, omega_rad_s) liefern, nicht {command!r}")
    return replace(step, v_mps=_speed(command[0]), omega_rad_s=_speed(command[1]))
