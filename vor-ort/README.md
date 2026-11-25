<!-- Wird als vor-ort/README.md in das Team-Repository makeathon-fahrzeug exportiert (tools/team_repository.py). Relative Links gelten dort. -->
# Start und Aufgaben vor Ort: Fahrzeug

Stand: 24. November 2026 · Makeathon Bremerhaven, 3.–5. Dezember 2026

Ihr habt die Beispiele auf eurem Laptop in der Simulation ausprobiert. Vor Ort baut ihr das Oberdeck
eures Fahrzeugs fertig, übertragt einen bekannten Ablauf auf das echte Fahrzeug und entwickelt danach
eine eigene Aufgabe.

| Schritt | Unterlage |
| --- | --- |
| 1 Oberdeck aufbauen und ersten Fahrtest machen | [Oberdeck-Bildanleitung](Oberdeck_Bildanleitung.pdf) |
| 2 Von der Simulation zum echten Fahrzeug | [Startkarte Fahrzeug](startkarte-fahrzeug.md) |
| 3 Grundaufgaben und eine Erweiterung | [Aufgabenblatt Fahrzeug](aufgabenblatt-fahrzeug.md) |
| 4 Eigene Ideen | [Ideen für Erweiterungen](ideen.md) |

## So arbeitet ihr

1. Übernehmt euer Fahrzeug und die vollständigen Geräteangaben. Wechselt im Team
   zwischen Programmieren, Versuchsbedienung und Protokollieren.
2. Führt die Startkarte selbstständig durch und tragt das Ergebnis eurer ersten Hardwareprobe ein.
3. Bearbeitet die Grundaufgabe und wählt anschließend eine Erweiterung. Testet jede Änderung
   zunächst in der Simulation und danach am Fahrzeug.
4. Speichert euren Python-Code und einen kurzen Bericht. Das Aufgabenblatt enthält
   Prüffälle und eine Vorlage für eure Ergebnisse.

Eine erfolgreiche Simulation ist ein eigener Nachweis. Die Hardwareprobe bekommt einen getrennten Eintrag.
Ihr habt vollen Zugriff auf euer Fahrzeug. Für Hardwaretests ist keine Anwesenheit oder einzelne Freigabe
eines Betreuers erforderlich. Not-Aus und die Schutzfunktionen im Pico bleiben immer aktiv.

## Gemeinsame Regeln

### Fahrfläche und Personenmarker

Alle Fahrzeuge folgen dem Personenmarker ID 300. Auf jeder Fahrfläche ist deshalb nur
ein Marker ID 300 sichtbar. Sonst folgt ein Fahrzeug womöglich der Person eines anderen
Teams. Andere Teams auf derselben Fläche drehen ihren Marker so lange weg.

### Akku und Laden

Das Fahrzeug fährt mit einem Lithium-Ionen-Akku (36 Wh).

- In Pausen fahrt ihr den Pi im SSH-Terminal mit `sudo poweroff` herunter. Wartet einige
  Sekunden und schaltet dann den Hauptschalter aus.
- Zeigt das Dashboard unter „Rückmeldung“ oder `~/fm-status` bei den Flags `AKKU-NIEDRIG`,
  hört ihr auf zu fahren und ladet den Akku.
- Laden nur mit dem beiliegenden Ladegerät: Hauptschalter aus, XT60-Kupplung trennen,
  Ladeadapter an die Akku-Buchse stecken, Ladegerät an den Adapter. Der Akku darf dabei
  im Fahrzeug bleiben. Volles Laden dauert etwa 3,5 Stunden.
- Beim Laden bleibt jemand in der Nähe. Akku und Ladegerät liegen nicht auf einer
  brennbaren Unterlage. **Ladeplatz:** ____________________
- Wird der Akku beim Laden warm oder ist er aufgebläht oder beschädigt, benutzt ihr ihn
  nicht weiter. Trennt das Ladegerät und zeigt den Akku der Orga.

## Arbeitsordner und Befehle

Die Laptop-Befehle beziehen sich auf euren Klon
[`makeathon-fahrzeug`](https://github.com/jonenfabian/makeathon-fahrzeug) aus der Vorab-Übung.
Öffnet das Terminal in diesem Projektordner. Dort liegen `start.py`, `.venv` und `fahrzeug/`.
Neu klonen müsst ihr nicht. Holt zu Beginn den aktuellen Stand mit Schritt 0 der Startkarte.
Die [README](../README.md) enthält die Installation für Windows, macOS und Linux.

Die Startkarte zeigt Python-Befehle mit `.venv/bin/python` für macOS/Linux.
Unter Windows ersetzt ihr diesen Teil durch `.\.venv\Scripts\python.exe`.
`ssh` und `scp` werden auf dem Laptop ausgeführt. Nach einer SSH-Anmeldung laufen die
Befehle auf dem Pi unter Linux, auch wenn euer Laptop Windows verwendet.

Großgeschriebene Angaben wie `PI_ADRESSE`, `PI_BENUTZER` und `FAHRZEUG` sind Platzhalter.
Ersetzt sie durch eure Geräteangaben. Passwörter gehören weder in euren Bericht noch ins Git.

## Was ihr zum Schluss zeigt

Zeigt einen vollständigen Ablauf, wiederholt ihn mit veränderter Ausgangslage und
erklärt einen behandelten Fehlerfall. Gebt euren Code und `bericht.md` ab. Nennt darin
die getestete Dateiversion, Gerät, Bedingungen, Ergebnis und eine verbleibende Grenze.
Legt vor der Abschlussvorführung gemeinsam fest, wer den Versuch startet und stoppt.
