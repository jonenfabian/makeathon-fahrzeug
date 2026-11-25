"""Übung: einmal beim sichtbaren Schild 100 stoppen, bis der Versuch neu startet.

Laptop, im Projektordner mit start.py (Windows: .\\.venv\\Scripts\\python.exe statt .venv/bin/python):
    .venv/bin/python start.py --aufgabe fahrzeug/pi-starter/beispiele/stoppschild.py --szenario schilder
Fahrzeug: Datei nach der Startkarte Fahrzeug als ~/makeathon_aufgabe.py übertragen und dort starten.
Im Dashboard bewusst starten, alternativ r + Enter im Pi-Terminal.
"""
from followme.task import Observation, Task


class StopOnSign(Task):
    def __init__(self):
        self.stopped = False

    def reset(self):
        # Läuft einmal je Start. Ein einzelnes verworfenes Bild setzt den Halt nicht zurück.
        self.stopped = False

    def decide(self, observation: Observation):
        # Ändert die ID oder ergänzt eine eigene Zustandsfolge.
        if any(marker.marker_id == 100 for marker in observation.markers):
            self.stopped = True
        if self.stopped:
            return 0., 0.
        return super().decide(observation)


def create_task():
    return StopOnSign()
