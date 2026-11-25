"""Python-Start für die Fahrzeug-Simulation, ohne App-Installer."""
from __future__ import annotations

import sys

MINDEST_PYTHON = (3, 10)


def python_pruefen(version=sys.version_info) -> str | None:
    """Verständliche Meldung, wenn das Python zu alt ist (z. B. Apples 3.9 am Mac)."""
    if tuple(version[:2]) >= MINDEST_PYTHON:
        return None
    return (f"Python {version[0]}.{version[1]} ist zu alt, nötig ist mindestens 3.10 (empfohlen 3.11). "
            "Am Mac ist `python3` ohne weitere Installation oft Apples Python 3.9. Installiert Python 3.11 "
            "(python.org oder Homebrew), legt `.venv` damit neu an und installiert requirements.txt erneut.")


def main():
    meldung = python_pruefen()
    if meldung:
        print(meldung, file=sys.stderr)
        return 2
    extra = sys.argv[1:]
    if extra and extra[0] == "fahrzeug":  # alter Aufruf aus dem gemeinsamen Repository
        extra.pop(0)
    from followme.sim_lab import main as vehicle_main

    return vehicle_main(extra)


if __name__ == "__main__":
    raise SystemExit(main())
