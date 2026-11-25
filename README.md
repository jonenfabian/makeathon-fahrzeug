# Makeathon Fahrzeug

Das Fahrzeug auf dem eigenen Rechner ausprobieren, Python-Code ändern und das Ergebnis
in einer 3D-Simulation ansehen. Danach läuft derselbe Code auf dem echten Fahrzeug. Ihr braucht
**Git, Python ab 3.10 und einen Browser**. Wir empfehlen Python 3.11. Die Bedienoberfläche läuft lokal.

## Loslegen: macOS und Linux

Prüft zuerst die Python-Version. Am Mac ist `python3` ohne weitere Installation oft Apples Python 3.9,
das ist zu alt. Dann Python 3.11 von python.org oder über Homebrew installieren.

```sh
python3 --version
git clone https://github.com/jonenfabian/makeathon-fahrzeug.git
cd makeathon-fahrzeug
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python start.py
```

## Loslegen: Windows

PowerShell oder Eingabeaufforderung:

```text
py -3 --version
git clone https://github.com/jonenfabian/makeathon-fahrzeug.git
cd makeathon-fahrzeug
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe start.py
```

Öffnet anschließend **http://127.0.0.1:8000**. Später genügt der letzte Befehl.
Die virtuelle Python-Umgebung muss nicht extra aktiviert werden. Strg-C beendet den Server.
Der passende vorbereitete Firmware-Simulator wird automatisch ausgewählt. Ist eure Plattform
unten aufgeführt, braucht ihr keinen eigenen Compiler und keinen zusätzlichen App-Installer.

## Simulation

Im Dashboard drei Szenen ausprobieren: Person folgen, vor einer Kiste stoppen, Personenmarker
von einem Schild unterscheiden. Nach Stopp könnt ihr Kisten ergänzen und die Szene zurücksetzen.
Das Kamerabild wird tatsächlich vom AprilTag-Detektor ausgewertet. Der simulierte Pico verwendet
denselben C++-Firmwarekern wie das echte Fahrzeug.

Eigene Aufgabe starten (Windows: Python-Pfad wie oben ersetzen):

```sh
.venv/bin/python start.py --aufgabe fahrzeug/pi-starter/beispiele/stoppschild.py --szenario schilder
```

Ändert die Beispieldatei im Editor. `create_task()` liefert eine Aufgabe mit `reset()` und
`decide(beobachtung)`. Sie gibt `(v_mps, omega_rad_s)` zurück. Derselbe Aufgabenvertrag gilt im
Fahrprogramm auf dem echten Pi. Ihr übertragt die Datei selbst auf euren Pi. Das Fahrzeug-Dashboard
unter `http://PI_ADRESSE:8000` zeigt Kamera und Messwerte und bietet Fahrt starten / Stopp.
Start lädt die Aufgabendatei neu.

## Vor Ort und eigene Ideen

- [Leitseite vor Ort](vor-ort/README.md) mit Startkarte und Aufgabenblatt. Die Startkarte erklärt den
  Übergang von der Simulation zum echten Fahrzeug.
- [Oberdeck-Bildanleitung](vor-ort/Oberdeck_Bildanleitung.pdf): die Bauteile, die ihr selbst montiert,
  und der erste Fahrtest.
- [Ideen für eigene Erweiterungen](vor-ort/ideen.md), auch zum Umsetzen mit einem Coding Agent.
- [`AGENTS.md`](AGENTS.md): Kurzbeschreibung des Repositorys und der Schnittstellen für Coding Agents.
- [Personenmarker ID 300](vor-ort/personenmarker_tag36h11_id300_A4.pdf) zum Nachdrucken (A4, 168 mm).

Ihr habt vollen Zugang zum Fahrzeug und testet selbstständig. Not-Aus und die Schutzfunktionen im Pico
bleiben immer aktiv.

| Einstieg | Nächste eigene Aufgabe |
|---|---|
| Fahrzeug folgt einem Marker | Geschwindigkeit in `langsamer_folgen.py` ändern und Abstände vergleichen |
| Schild 100 erkennen | Einmal stoppen, andere IDs mit eigenen Aktionen verbinden |
| Kiste im Weg | Frontabstand und Abbremsen beobachten. Umfahren als spätere Erweiterung |

## Plattformen

Vorbereitete Simulatoren und ihr jeweiliger Quell-/Binärnachweis liegen unter
`fahrzeug/firmware/host_sim/prebuilt/`. Der automatische Lauf in GitHub Actions prüft
Windows x64, macOS Intel/Apple Silicon sowie Linux x64/ARM64. Maßgeblich ist das grüne
Prüfergebnis des jeweiligen Quellstands. Neue Firmwarequellen erfordern neue Simulatoren.
Die [Plattformtabelle](PLATTFORMEN.md) enthält den Nachweis für die mitgelieferten Dateien.

Die Simulation vereinfacht Kamera, Radbewegung und Kontakte. Reale Abnahmen finden am
Gerät statt. Das Repository enthält keine Zugangsdaten zu echten Geräten.

## Lizenzen

Die Lizenztexte fremder Dateien (three.js, Modelle) liegen direkt neben diesen Dateien.
Für den eigenen Fahrzeug-Code wird hier keine zusätzliche Lizenz erteilt. Die bisherigen
Rechte bleiben erhalten. Die Veröffentlichung ändert bestehende Lizenzbedingungen nicht.
