# Projekt Rules – Sunray Arduino Refactoring

> **Ziel:** Bestehenden Sunray **Arduino-C++**-Code **schrittweise und sicher** verbessern, ohne das Laufzeitverhalten unbeabsichtigt zu ändern. Jeder Schritt ist klein, nachvollziehbar, kompilierbar und rückrollbar.

---

## 1) Leitbild

* **Stabilität vor Eleganz.** Verbesserungen nur, wenn der Roboter danach **mindestens so zuverlässig** funktioniert wie vorher.
* **Ein Schritt = ein Mehrwert.** Jede Änderung hat ein klares Ziel (Lesbarkeit, Modularität, Testbarkeit, Bugfix) – keine „Umbauten ins Blaue“.
* **Arduino Due ist Zielplattform.** Alle Builds und Tests erfolgen für den **Arduino Due**.
* **Alfred-Code läuft auf Raspberry Pi.** Refactorings am Sunray-Arduino-Teil dürfen die Kompatibilität mit dem Raspberry-Pi-Code (Alfred) **nicht beeinträchtigen**.
* **Kommunikation Alfred ↔ STM:** Die Kommunikation über **AT+ Befehle** muss vollständig kompatibel bleiben. Erweiterungen sind erlaubt, aber **bestehende Befehle dürfen nicht entfernt oder verändert** werden.

---

## 2) Refactor-Ziele & Prioritäten

**Ziel:** Den Code in kleinere, wartbare Funktionen zerlegen, Performance und Fahrverhalten verbessern, gleichzeitig bestehende Funktionalität sicher erhalten.

### Prioritäten

1. **Code-Struktur verbessern**

   * Große Funktionen in kleine, testbare Module aufteilen.
   * Klar getrennte Verantwortlichkeiten (z. B. Map Handling, Motorsteuerung, Sensorlogik).

2. **Fahrverhalten optimieren**

   * **PID-Controller**: sanftere Motorregelung.
   * **Stanley-Controller**: präzisere Spurführung.
   * **Odometrie**: Drift minimieren.
   * **Obstacle Avoidance**: intelligentes Ausweichen.

3. **Pathfinder optimieren**

   * Effiziente Routenplanung, weniger Lücken.
   * Trennung in Kartenlogik, Zielpunkt-Auswahl und Fahrstrategie.

4. **Kommunikation erhalten**

   * AT+ Befehle unverändert lassen.
   * Erweiterungen nur zusätzlich, kein Entfernen oder Umbauen existierender Befehle.

### Vorgehensweise

* **Nur einen Block pro Schritt** anfassen.
* Nach jeder Änderung **kompilieren und testen**.
* Verhalten darf sich nicht verschlechtern.
* Erst nach stabilen Refactors → Parameter/Tuning der Algorithmen anpassen.

---

## 3) Begriffe & Change-Typen

* **Refactor (verhaltensgleich):** Nur Struktur, Namen, Aufteilung, Entkopplung. *Kein* Verhalten ändern.
* **Bugfix (verhaltensändernd, begründet):** Fehlerbehebung mit minimaler Scope. Muss messbare Evidenz/Begründung enthalten.
* **Cleanup:** Kommentare, Formatierung, tote/auskommentierte Codeblöcke entfernen – **ohne** Logikänderung.
* **Feature (verhaltensändernd):** **Außerhalb** dieses Dokuments. Nur via separatem Prozess/Branch.

---

## 4) Small-Step-Policy (harte Regeln)

1. **Atomar:** Pro Commit/PR genau **eine** abgeschlossene Mini-Änderung.
2. **Kompilieren:** Nach jeder Änderung muss das Projekt **für Arduino Due** fehlerfrei bauen.
3. **Tests:** Minimaltests (siehe Abschnitt 7) laufen **vor** dem Commit lokal.
4. **Kein Kaskadieren:** Keine Änderung, die weitere unplanmäßige Kettenänderungen erzwingt.
5. **Dokumentation:** „Was & Warum“ **im Commit-Text** + ggf. kurze Code-Kommentare.
6. **Rückrollbar:** Jede Änderung ist allein revertierbar, ohne Folgebrüche.
7. **AT+-Kompatibilität:** Jeder Commit muss sicherstellen, dass **alle bestehenden AT+ Befehle** unverändert funktionieren.

---

## 5) Modulgrenzen (Sunray-spezifisch)

Umbauten erfolgen **modulweise**, niemals modulübergreifend in einem Schritt:

* `Map` / Kartierung & Navigation
* `Motor` / Antrieb & Motorsteuerung
* `Perimeter` / Begrenzungserkennung
* `GPS` / Positionsbestimmung
* `IMU` / Lage- und Richtungssensorik
* `Battery` / Energie & Laden
* `Console/UI` / Serielle Konsole, Logging, Konfiguration
* `AT+ Interface` / Kommunikation Raspberry Pi ↔ STM

> **Regel:** In einem Schritt **nur ein Modul** berühren. Gemeinsame Utilitys erst anlegen, **nachdem** zwei Stellen den Bedarf belegt haben.

---

## 6) Benennung & Stil

* **Klassen:** `PascalCase`
* **Funktionen/Variablen:** `camelCase`
* **Konstanten/Defines/Enums:** `UPPER_SNAKE_CASE`
* **Dateien:** `ModuleName.cpp/.h` oder `module_name.h` – Projektweit **eine** Konvention.
* **Formatierung:** 4 Leerzeichen, Klammern immer setzen, keine Misch-Tabs.
* **Kommentare:** Kurz, präzise, erklären **warum**, nicht **was**.

---

## 7) Minimal-Tests pro Schritt (Pflicht)

**Build:**

* Arduino Due Target baut ohne Fehler/Warnungen.

**Smoke-Tests:**

* Startsequenz bis „bereit“.
* Sensor-Init meldet ok oder erwartete Werte.
* Motor-Enable/Disable ohne Bewegung funktioniert.
* **AT+ Befehle:** Alle bisherigen Befehle reagieren unverändert korrekt.

**Feld-Mini-Test:**

* 2–3 Min Mähen im sicheren Bereich. Keine Kreisbewegungen, Perimeter ok.

**Telemetrie/Logs:**

* Relevante Logs unverändert oder verbessert.

> Wenn einer dieser Punkte scheitert → **Revert**.

---

## 8) Sicherheitsnetze

* Feature-Flags optional, default aus.
* Kill-Switch bleibt unberührt.
* Watchdogs nicht deaktivieren.
* Logging-Level erweiterbar, Standard moderat.

---

## 9) Git-Strategie

* `main` stabil; Feature-Branch pro Änderung.
* PR ≤ \~200 Zeilen Diff.
* Mindestens 1 Review.
* Funktionsfähige Meilensteine taggen.

---

## 10) Commit-/PR-Vorlage

```
refactor(map): split load() into parseHeader() and readCells() (no behavior change)

WHY
- load() war >150 Zeilen; klare Verantwortlichkeiten fehlten.

WHAT
- Header-Parsing in parseHeader() extrahiert
- Zell-Lesen in readCells() extrahiert
- Unit-ähnliche Stub-Tests für parseHeader()

TEST
- Arduino Due Build ok
- Smoke: Start/Init ok; Map laden unverändert
- AT+ Befehle: alle alten unverändert, keine Regressionen
```

---

## 11) Review-Checkliste (PR)

* [ ] Atomar & ein Modul.
* [ ] Commit erklärt Warum & Was.
* [ ] Kein unbeabsichtigter Verhaltenswechsel.
* [ ] Build sauber; Warnungen geprüft.
* [ ] Smoke-/Feldtests dokumentiert.
* [ ] Stilregeln eingehalten.
* [ ] Revert möglich.
* [ ] **AT+ Befehle kompatibel.**

---

## 12) Definition of Done (pro Schritt)

* [ ] Kompiliert für Arduino Due.
* [ ] Minimaltests grün.
* [ ] Codequalität verbessert.
* [ ] Diff überschaubar.
* [ ] Dokumentation vorhanden.
* [ ] **AT+ Schnittstelle unverändert funktionsfähig.**
