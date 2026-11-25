"""Lesender Zugriff auf das laufende Lernlabor (vorher in einem anderen Terminal start.py starten).

Im Projektordner mit start.py: .venv/bin/python fahrzeug/pi-starter/beispiele/status_lesen.py
"""
import json
import urllib.request

with urllib.request.urlopen("http://127.0.0.1:8000/api/state", timeout=3) as response:
    state = json.load(response)
print("Betrieb:", state["mode"], "| Regler:", state["perception"]["state"])
print("Erkannte Marker:", [marker["marker_id"] for marker in state["perception"]["markers"]])
print("Letzte Rückmeldung:", state["reason"])
