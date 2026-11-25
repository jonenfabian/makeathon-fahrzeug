# Aufgabenblatt Fahrzeug

Stand: 24. November 2026 · **Team:** ____________________ **Fahrzeug:** __________

Euer Fahrzeug soll einer Person folgen und eine eigene Erweiterung ausführen. Beginnt
mit der [Startkarte Fahrzeug](startkarte-fahrzeug.md). Montage, Verdrahtung und erster
Fahrtest gehören zum Einstieg. Ihr testet selbstständig und wählt dafür eine freie
Fahrfläche sowie eine nachvollziehbare Ausgangslage. Die Schutzfunktionen im Pico bleiben erhalten.

## F1 Folgen beobachten

Startet mit `fahrzeug/pi-starter/beispiele/langsamer_folgen.py`. Beobachtet in der
Simulation zunächst Losgehen, Stehenbleiben und Wiederanfahren. Vergleicht anschließend
am echten Fahrzeug die Reaktion auf Abstand und seitliche Position.
Der Personenmarker hat ID 300 und eine schwarze Kante von 168 mm. Der voreingestellte
Wunschabstand beträgt 1,5 m. Die Kameraentfernung ist ein Schätzwert.

| Prüffall | Erwartete Beobachtung |
| --- | --- |
| Person zeigt den Marker mittig und vergrößert langsam den Abstand | Ziel bleibt ID 300. Das Fahrzeug reagiert auf den größeren Abstand |
| Person bleibt stehen | Vorwärtsgeschwindigkeit sinkt, Fahrzeug kommt zum Stillstand |
| Person geht erneut los | Läuft die Fahrt noch, fährt das Fahrzeug wieder an |
| Versuch wird im Dashboard oder mit `s` + Enter gestoppt | Räder stehen. Die Fahrt bleibt bis zum bewussten neuen Start gesperrt |

**Geschafft, wenn:** Ihr könnt Kamerabild, Zielabstand, Sollgeschwindigkeit und tatsächliche
Fahrt zusammen erklären. Haltet mindestens einen Unterschied zwischen Simulation und
Hardware fest. Bestätigt Anhalten und Wiederanfahren jeweils in einer eigenen Beobachtung.
Beginnt hardwareseitig mit einem kurzen Ablauf und verlängert ihn schrittweise.

## F2 Eine Änderung messen

Ändert den Geschwindigkeitsfaktor in `langsamer_folgen.py` von `0.6` auf einen kleineren
positiven Wert. Ändert dabei nur diese eine Größe. Vergleicht beide Versionen unter
möglichst gleichen Bedingungen, zunächst in der Simulation und danach am Pi.

**Geschafft, wenn:** Ihr nennt alten und neuen Faktor und beschreibt eine beobachtete
Auswirkung, etwa Anfahrdauer oder Abstandsverlauf. Ihr übertragt dieselbe getestete Datei
wie auf der Startkarte auf den Pi. Stoppt die Fahrt vor dem Austausch. Der nächste
Dashboard-Start lädt die Datei neu. Erhöht keine Firmwaregrenzen und verändert keine Schutzfunktion.

## F3 Ein Stoppschild auswerten

Verwendet `fahrzeug/pi-starter/beispiele/stoppschild.py` als Grundlage. Startet auf dem
Laptop im Projektordner. Beendet einen vorherigen Server zuerst mit Strg+C:

```sh
.venv/bin/python start.py --aufgabe fahrzeug/pi-starter/beispiele/stoppschild.py --szenario schilder
```

Unter Windows ersetzt ihr `.venv/bin/python` durch `.\.venv\Scripts\python.exe`.
Die Schilder-Szene zeigt Schild 100 schon beim Start. Deshalb kann die Aufgabe sofort
Stillstand vorgeben. Erklärt zuerst dieses Verhalten. Für eine spätere Begegnung verwendet
ihr am echten Fahrzeug das Schild an einer dokumentierten Stelle.

Das Beispiel merkt sich ein erkanntes Schild 100 und liefert anschließend `(0.0, 0.0)`.
Bei fortlaufend gültigen Beobachtungen bleibt der Halt auch nach Entfernen des Schilds
bestehen. Der nächste bewusste Start auf dem Pi lädt die Aufgabe neu. Programmiert deshalb
keinen automatischen Wechsel aus Stopp und Neustart. `reset()` läuft genau einmal je Start, ein
verworfenes Bild setzt euren Zustand nicht zurück. Der gemerkte Schildzustand ist keine Stoppverriegelung im Pico.

| Prüffall | Erwartetes Ergebnis |
| --- | --- |
| Nur Personenmarker 300 sichtbar | Vorhandene Folgeregelung liefert den Vorschlag |
| Schild 100 wird erkannt | Aufgabe liefert Null-Sollwerte. Tatsächlichen Stillstand prüfen |
| Schild 100 verschwindet bei weiter frischen Bildern | Ohne Zurücksetzen bleibt die Aufgabe angehalten |
| Eine andere Schild-ID erscheint, 100 wurde noch nicht erkannt | Diese ID löst den Stopp von Schild 100 nicht aus |

**Geschafft, wenn:** Die Prüffälle sind nachvollziehbar protokolliert. Schild und Fahrziel
werden unterschieden. Übertragt für den Hardwaretest diesmal `stoppschild.py` mit `scp`
auf `makeathon_aufgabe.py`. Der Pi-Startbefehl der Startkarte bleibt gleich.

## F4 Eine Erweiterung wählen

Wählt nach F1–F3 eine Erweiterung und legt vor dem Programmieren ein Erfolgskriterium fest:

| Erweiterung | Euer Ergebnis |
| --- | --- |
| Langsam-Schild | Eine vereinbarte Schild-ID reduziert die Vorwärtsgeschwindigkeit. Aktivierung und Rücknahme sind eindeutig. In der Simulation mit `--szenario schilderweg` (Schilder 101, 102 und 100 am Weg) vorab testbar |
| Eigene Anzeige | Ziel-ID, Bildalter, Geschwindigkeit und Frontwert anzeigen. Unbekannte Werte verständlich kennzeichnen |
| Verdeckung untersuchen | Verlauf bei sichtbarem, verdecktem und wieder sichtbarem Ziel dokumentieren, mit Dauer und Ausgangslage |

Die Aufgabe liefert über `decide(beobachtung)` zwei Sollwerte zurück: Vorwärtsgeschwindigkeit
in **m/s** und Drehrate in **rad/s**. `reset()` setzt euren Zustand zurück. `create_task()`
erzeugt eure Aufgabe. Verwendet die Beobachtungen und den vorhandenen Regler-Vorschlag.
`front_mm = None` ist keine Freifahrt-Meldung. Lange Schleifen oder Wartezeiten gehören
nicht in `decide()`, weil für jede neue Beobachtung zeitnah eine Entscheidung nötig ist.

**Freiwillige Zusatzoption Ultraschall:** Wer F1–F4 geschafft hat und noch Zeit hat, kann bei der Orga
zwei Ultraschallsensoren HC-SR04 bekommen und schräg vorn anbauen. An den Fahrzeugen sind sie nicht
montiert. Die Firmware misst sie bereits. In der Beobachtung stehen dann `left_mm`/`left_state` und
`right_mm`/`right_state`. Ohne Sensor ist der Zustand `unavailable` oder `fault`, das ist keine
Freifahrt-Meldung. Der Pico sperrt mit diesen Werten nichts, eure Aufgabe muss selbst bremsen.
Ultraschall übersieht weiche Kleidung und schräge glatte Flächen.

## Ergebnisse und Abgabe

Speichert euren Aufgabencode und `bericht.md`. Wiederholt die gewählte Abschlussaufgabe
dreimal mit dokumentierter Ausgangslage. Stimmt Wiederholungen am gemeinsam genutzten Fahrzeug im Team ab.

| Aufgabe / Dateiversion | Simulation oder Fahrzeug | Ausgangslage | Erwartung | Beobachtung | Bestanden / offen |
| --- | --- | --- | --- | --- | --- |
| | | | | | |
| | | | | | |
| | | | | | |

Gebt Gerät, Datum und die geänderten Werte an. Beschreibt einen behandelten Fehlerfall,
eine Grenze und eure nächste Verbesserung. Zugangsdaten und Bilder von Personen gehören
nicht in den Bericht. Zeigt zum Abschluss den Ablauf und erklärt, was euer eigener Code entscheidet.
