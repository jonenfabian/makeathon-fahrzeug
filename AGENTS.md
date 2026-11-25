# Hinweise für Coding Agents: Fahrzeug

Dieses Repository ist der Team-Einstieg für das Fahrzeug beim Makeathon. Die Teams schreiben
eine eigene **Aufgabendatei** in Python. Sie läuft in der lokalen Simulation und unverändert auf dem
Raspberry Pi des echten Fahrzeugs.

## Ordner

| Ort | Inhalt | Ändern? |
| --- | --- | --- |
| `fahrzeug/pi-starter/beispiele/` | Beispielaufgaben (`langsamer_folgen.py`, `stoppschild.py`, `status_lesen.py`) | Kopieren und anpassen |
| `fahrzeug/pi-starter/followme/` | Fahrprogramm, Erkennung, Simulation, Aufgabenvertrag (`task.py`) | Nein, nur lesen |
| `fahrzeug/firmware/` | C++-Kern der Pico-Firmware für den Simulator | Nein |
| `vor-ort/` | Startkarte, Aufgabenblatt, Ideen, Bildanleitung | Nein |
| `start.py` | Startet die Simulation unter http://127.0.0.1:8000 | Nein |

## Aufgabenvertrag

Die Aufgabendatei enthält `create_task()`. Sie liefert ein Objekt mit:

- `reset()`: eigenen Zustand zurücksetzen. Wird einmal je Start aufgerufen, nicht zwischen zwei Bildern.
- `decide(observation)`: gibt `(v_mps, omega_rad_s)` zurück. Vorwärtsgeschwindigkeit in m/s, Drehrate in
  rad/s, positiv = links. Wird für jedes neue Kamerabild aufgerufen und muss schnell antworten:
  keine Wartezeiten, keine langen Schleifen, keine Netzwerkzugriffe.

Felder von `observation` (`followme/task.py`, Klasse `Observation`):

| Feld | Bedeutung |
| --- | --- |
| `captured_s` | Aufnahmezeit des Bildes |
| `target` | verfolgtes Ziel (Personenmarker ID 300) oder `None`: `bearing_rad` (positiv = links), `distance_m` |
| `markers` | alle erkannten Marker: `marker_id`, `role` (`person`, `schild`, `fahrzeug`), `bearing_rad`, `distance_m` |
| `controller_state` | Zustand der vorhandenen Folgeregelung |
| `front_mm`, `front_state` | Abstandssensor vorn. `None` heißt unbekannt, nicht frei |
| `suggested_v_mps`, `suggested_omega_rad_s` | Vorschlag der vorhandenen Folgeregelung |
| `left_mm`/`left_state`, `right_mm`/`right_state` | nur mit der freiwilligen Ultraschall-Zusatzoption. Sonst `unavailable` |

Marker: Person ID 300 (168 mm), Schilder 100 bis 108 (150 mm), Fahrzeuge 2, 4, 6, 8 (50 mm),
alle AprilTag 36h11.

## Grenzen

- Der Pico auf dem Fahrzeug begrenzt Geschwindigkeit und Beschleunigung, sperrt die Vorwärtsfahrt unter
  etwa 40 cm Frontabstand, überwacht die Verbindung und hält bei Not-Aus an. Die Aufgabe kann das nicht
  abschalten und soll es nicht versuchen.
- Kein Code darf den Motortreiber direkt ansprechen. Bewegung nur über die Rückgabe von `decide()`.
- Keine Zugangsdaten (Pi-Passwort, WLAN) in Dateien oder ins Git schreiben.

## Prüfen

```sh
.venv/bin/python start.py --aufgabe fahrzeug/pi-starter/beispiele/EURE_DATEI.py --szenario schilder
```

Szenarien und Bedienung stehen in der README. Danach überträgt das Team die Datei selbst auf den Pi,
der Ablauf steht in `vor-ort/startkarte-fahrzeug.md`.
