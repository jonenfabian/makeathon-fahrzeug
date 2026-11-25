# Startkarte Fahrzeug

Stand: 24. November 2026 · Ziel: eure Python-Aufgabe auf den Pi übertragen und das echte
Fahrzeug im Browser starten, stoppen und beobachten. Ihr arbeitet selbstständig mit vollem Zugang.

**Team:** ____________________ **Fahrzeug:** __________ **Datum:** ____________________

**Eure Geräteangaben:** `PI_BENUTZER` __________ `PI_ADRESSE` ____________________

`FAHRZEUG` für die Konfiguration, etwa `fm01`: __________

## 0 Repository aktualisieren

Öffnet ein Terminal im Projektordner `makeathon-fahrzeug`, also im Ordner mit `start.py`.
Holt den aktuellen Stand und installiert die Abhängigkeiten neu.

macOS und Linux:

```sh
git pull
.venv/bin/python -m pip install -r requirements.txt
```

Windows (PowerShell oder Eingabeaufforderung):

```text
git pull
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Bricht `git pull` wegen geänderter Beispieldateien ab, sichert eure Datei unter einem neuen
Namen. Setzt das Original mit `git restore DATEI` zurück und wiederholt `git pull`.
Fehlt der Ordner `.venv`, installiert ihr nach der README des Team-Repositorys.

## 1 Auf dem Laptop prüfen

Die Befehle gelten für macOS/Linux. Unter Windows ersetzt ihr `.venv/bin/python` durch
`.\.venv\Scripts\python.exe`. Beendet einen vorherigen Simulationsserver mit Strg+C.

```sh
.venv/bin/python start.py --aufgabe fahrzeug/pi-starter/beispiele/langsamer_folgen.py
```

Öffnet `http://127.0.0.1:8000`, wählt „Der Person folgen“ und startet den Versuch.
Die Datei liefert die vorhandene Folgeregelung mit verringerter Vorwärtsgeschwindigkeit.
Prüft Fahrt, Kamerabild und Geschwindigkeit. Danach den Versuch stoppen.

## 2 Datei übertragen und Fahrprogramm starten

Stellt Montage und Verdrahtung anhand der Anleitung am Arbeitsplatz fertig und prüft den
Aufbau. Die vorbereitete SD-Karte enthält Laufzeit, Grundsoftware und Fahrzeugkonfiguration.
Verwendet eure Geräteangaben anstelle der großgeschriebenen Platzhalter.
Hinweise zu Akku und Laden stehen in [Start und Aufgaben vor Ort](README.md#akku-und-laden).

**Auf dem Laptop:**

```sh
scp fahrzeug/pi-starter/beispiele/langsamer_folgen.py PI_BENUTZER@PI_ADRESSE:makeathon_aufgabe.py
ssh PI_BENUTZER@PI_ADRESSE
```

`scp` und `ssh` nutzen den OpenSSH-Client eures Laptops. Gleicht einen erstmals angezeigten
Hostschlüssel mit den Geräteangaben ab. Gebt das Passwort in der Passwortabfrage ein.

**Ab jetzt im SSH-Terminal auf dem Pi.**

> **Vor der ersten Fahrt** (einmal je Fahrzeug)
>
> 1. Fahrzeug aufbocken, beide Antriebsräder frei. Not-Aus entriegelt, Frontsensor mit freier Sicht.
> 2. `~/fm-status --sekunden 5`: Zustand `STANDBY`, Flags `-`, beide Räder `+0 mm/s`.
> 3. `~/fm-motorimpuls`: Beide Räder drehen kurz vorwärts und stehen wieder. Die Ausgabe zeigt `"complete": true`.
>    Steht dort `false` oder dreht ein Rad rückwärts, fahrt nicht weiter und sprecht die Orga an.
> 4. `~/fm-fahrtest`: `r` drücken, dann `w` gedrückt halten. Not-Aus drücken: Die Räder stoppen. Not-Aus
>    entriegeln: Die Räder bleiben stehen, bis ihr erneut `r` drückt. `q` beendet den Test.

Prüft vor jedem Start des Fahrprogramms kurz mit `~/fm-status --sekunden 5` dieselben Werte.
`ToF kein Ziel` (Rohwert `65535`) bedeutet nur „kein Messwert“, nicht freie Fahrbahn.
Lasst die Anzeige enden. Nur ein Programm darf den Pico gleichzeitig öffnen.
Startet dann das Fahrprogramm mit der Konfiguration eures Fahrzeugs:

```sh
~/fm/bin/python -m followme.follow --konfiguration ~/followme-bausatz/pi/config/FAHRZEUG.json --aufgabe ~/makeathon_aufgabe.py --stream 8000
```

Lasst das SSH-Terminal geöffnet. Der Start dieses Programms löst noch keine Fahrt aus.
Die vorbereiteten Pfade sind `~/fm` und `~/followme-bausatz`. Weichen sie bei eurem Gerät
ab, verwendet ihr die Pfade aus euren Geräteangaben.

## 3 Im Browser fahren und beobachten

Öffnet auf dem Laptop **`http://PI_ADRESSE:8000`**. Das Dashboard trägt die Kennzeichnung
„Echtes Fahrzeug“. `127.0.0.1:8000` auf eurem Laptop gehört weiterhin zur lokalen Simulation.

Eine Person zeigt den Personenmarker **ID 300, schwarze Kante 168 mm** bei etwa 1,5–2 m
Abstand. Prüft im Live-Bild die Erkennung, dann etwas links und rechts. Auf eurer Fahrfläche
ist nur dieser eine Marker ID 300 sichtbar. Sonst folgt das Fahrzeug womöglich der Person
eines anderen Teams. Andere Teams drehen ihren Marker so lange weg. Wählt eine freie Fahrfläche.

| Im Dashboard | Wirkung |
| --- | --- |
| Fahrt starten | Aufgabendatei neu laden und bewusst ARM an den Pico senden |
| Stopp | Sollwerte verwerfen und STOP an den Pico senden. Wirkt von jeder geöffneten Seite aus |
| Kamera und Messwerte | Zielabstand, Frontabstand, Radgeschwindigkeiten, Encoder, Motorspannung und Pico-Flags verfolgen |

Der Pico bestätigt den Zustand unabhängig vom Browser. Nach Stopp prüft ihr **STANDBY
und stehende Räder**. Seitenwechsel, Schließen der bedienenden Seite oder Verlust ihrer
Verbindung beenden die Browserfahrt. Bleibt die Rückmeldung aus, gilt der Stillstand als
unbekannt. Bei unerwarteter Bewegung drückt ihr den Not-Aus am Fahrzeug. Eine wieder
geöffnete Seite startet nie automatisch. Zum Weiterfahren erneut bewusst starten.

Im Pi-Terminal bleiben `r` + Enter für Start, `s` + Enter für Stopp und `q` + Enter für
Programmende verfügbar. Eine im Terminal gestartete Fahrt hat keine Browser-Bindung.
Die Browser-Stopp-Taste wirkt trotzdem.

## 4 Im Editor weiterarbeiten

Stoppt die Fahrt, ändert eure Datei auf dem Laptop, testet in der Simulation und kopiert
sie erneut per `scp` auf `makeathon_aufgabe.py`. Beim nächsten Klick auf **Fahrt starten**
wird die Datei neu geladen. Reine Änderungen dieser Aufgabendatei brauchen keinen Neustart
des Pi-Servers. Bei Änderungen an anderen Modulen beendet ihr das Fahrprogramm und startet es neu.

- [ ] Prüfungen vor der ersten Fahrt erledigt, Datei übertragen, richtiges Gerät geprüft,
  Marker erkannt, erste Fahrt und Stopp beobachtet.

**Ergebnis / Grenze:** ______________________________________________________________

Weiter mit dem [Aufgabenblatt Fahrzeug](aufgabenblatt-fahrzeug.md).
