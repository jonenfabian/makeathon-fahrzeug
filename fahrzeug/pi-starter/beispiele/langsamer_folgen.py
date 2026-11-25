"""Übung: der vorhandenen Folgeregelung mit verringerter Vorwärtsgeschwindigkeit folgen.

Laptop, im Projektordner mit start.py (Windows: .\\.venv\\Scripts\\python.exe statt .venv/bin/python):
    .venv/bin/python start.py --aufgabe fahrzeug/pi-starter/beispiele/langsamer_folgen.py
Fahrzeug: Datei nach der Startkarte Fahrzeug als ~/makeathon_aufgabe.py übertragen und dort starten.
"""
from followme.task import Observation, Task


class SlowerFollow(Task):
    def decide(self, observation: Observation):
        v_mps, omega_rad_s = super().decide(observation)
        return .6 * v_mps, omega_rad_s


def create_task():
    return SlowerFollow()
