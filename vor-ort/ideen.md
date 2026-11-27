<!-- Wird als vor-ort/ideen.md in das Team-Repository makeathon-fahrzeug exportiert (tools/team_repository.py). Relative Links gelten dort. -->
# Ideen für eigene Erweiterungen: Fahrzeug

Stand: 24. November 2026. Diese Liste ist eine Sammlung von Anregungen, keine Pflicht. Wählt nach
dem [Aufgabenblatt](aufgabenblatt-fahrzeug.md) eine Idee, legt vor dem Programmieren ein Erfolgskriterium
fest und testet jede Änderung zuerst in der Simulation.

Ausstattung jedes Fahrzeugs: Differentialantrieb mit Encodern, Raspberry Pi 3B+, Pico 2 mit allen
Schutzfunktionen, Weitwinkelkamera nach vorn, ein Abstandssensor vorn. Eure Aufgabe liefert über
`decide(beobachtung)` eine Vorwärtsgeschwindigkeit in m/s und eine Drehrate in rad/s. Der Pico begrenzt
Geschwindigkeit, Beschleunigung und stoppt vor Hindernissen vorn, egal was eure Aufgabe vorschlägt.

## Mit dem Vorhandenen

| Idee | Ergebnis | Worauf achten |
| --- | --- | --- |
| Ruhiger folgen | Weniger Ruckeln, gleichmäßiger Abstand, ruhige Kurven | Abstand und Drehrate mit Werten aus der Beobachtung vergleichen, nicht nach Gefühl |
| Ziel kurz verlieren und wiederfinden | Abbremsen, stoppen, begrenzt drehen, Marker wieder aufnehmen | Begrenzt suchen, nicht endlos drehen. Ohne frisches Bild nicht weiterfahren |
| Verkehrsschilder | Schilder 100 bis 108 lösen Stopp, Langsam, Links oder Rechts aus | Jede Aktion nur einmal pro Begegnung auslösen. Schild und Person nie verwechseln |
| Parkplatz anfahren | Ein aufrechtes Schild ansteuern und davor anhalten | Der Abstandssensor stoppt bei etwa 40 cm. Plant euren Haltepunkt davor |
| Eigene Anzeige | Ziel-ID, Bildalter, Geschwindigkeit und Frontwert anzeigen | Unbekannte Werte verständlich kennzeichnen |
| Fahrgenauigkeit messen | Ein Quadrat fahren und den Endfehler messen | Die Odometrie driftet. Mit Maßband vergleichen |
| Lieferfahrt | Eine Station auswählen, etwa Schild 103, und auf freier Fläche bis davor fahren | Station als Schild-ID in der Aufgabendatei festlegen. Die Station muss sichtbar sein, es gibt keine Karte und keine Wegplanung |

## Mit mehreren Fahrzeugen

| Idee | Ergebnis | Worauf achten |
| --- | --- | --- |
| Kolonnenfahrt | Das erste Fahrzeug folgt der Person, die weiteren dem Fahrzeug-Marker davor (IDs 2, 4, 6, 8) | Erst Erkennungsabstand und Bremsweg messen. Eigener Stopp bei Zielverlust |
| Choreografie | Mehrere Fahrzeuge starten gemeinsam kurze Bewegungsfolgen | Ein Takt über WLAN ist nie ganz gleich. Kurze, langsame Folgen |
| Dashboard erweitern | Eine eigene Übersicht auf dem Laptop zeigt die Messwerte mehrerer Fahrzeuge nebeneinander | Euer Programm liest `http://PI_ADRESSE:8000/status` (Messwerte als JSON) und `/stream` (Kamerabild). Start und Stopp bleiben im Fahrzeug-Dashboard |

Auf jeder Fahrfläche ist nur ein Personenmarker ID 300 sichtbar. Sonst folgt ein Fahrzeug der Person
eines anderen Teams.

## Mit Zusatzteilen

| Idee | Ergebnis | Voraussetzung |
| --- | --- | --- |
| LED-Anzeige | Zustand, Richtung oder Abstand mit den LED-Streifen zeigen | LED-Bonus in der [Oberdeck-Bildanleitung](Oberdeck_Bildanleitung.pdf) |
| Schwenkrad statt Kugelrolle | Ruhigeres Kamerabild beim Fahren, wenn der Hallenboden uneben ist | Schwenkräder und Druckteile bei der Orga. Erkennung beim Fahren vorher und nachher vergleichen |
| Hindernis umfahren oder an einer Wand entlangfahren | Seitlich messen und ausweichen | Freiwillige Zusatzoption Ultraschall, siehe Aufgabenblatt. Sensoren bei der Orga |

## Mit einem Coding Agent arbeiten

- Öffnet den ganzen Klon im Agenten. [`AGENTS.md`](../AGENTS.md) beschreibt Ordner, Aufgabenvertrag und Grenzen.
- Beschreibt das Ziel mit einem prüfbaren Erfolgskriterium, etwa: „Bei Schild 104 fährt das Fahrzeug
  halb so schnell, bis Schild 105 erscheint.“
- Lasst den Agenten nur eure Aufgabendatei ändern, nicht `followme/` und nicht die Firmware.
- Prüft jede Version in der Simulation, bevor sie auf den Pi kommt. Lest den Code, den ihr übertragt.
- `decide()` muss schnell antworten. Keine Wartezeiten und keine langen Schleifen darin.
