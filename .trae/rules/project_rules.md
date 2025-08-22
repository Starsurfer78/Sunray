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

## 2) Begriffe & Change-Typen

* **Refactor (verhaltensgleich):** Nur Struktur, Namen, Aufteilung, Entkopplung. *Kein* Verhalten ändern.
* **Bugfix (verhaltensändernd, begründet):** Fehlerbehebung mit minimaler Scope. Muss messbare Evidenz/Begründung enthalten.
* **Cleanup:** Kommentare, Formatierung, tote/auskommentierte Codeblöcke entfernen – **ohne** Logikänderung.
* **Feature (verhaltensändernd):** **Außerhalb** dieses Dokuments. Nur via separatem Prozess/Branch.

---

## 3) Small-Step-Policy (harte Regeln)

1. **Atomar:** Pro Commit/PR genau **eine** abgeschlossene Mini-Änderung.
2. **Kompilieren:** Nach jeder Änderung muss das Projekt **für Arduino Due** fehlerfrei bauen.
3. **Tests:** Minimaltests (siehe Abschnitt 7) laufen **vor** dem Commit lokal.
4. **Kein Kaskadieren:** Keine Änderung, die weitere unplanmäßige Kettenänderungen erzwingt.
5. **Dokumentation:** „Was & Warum“ **im Commit-Text** (Vorlage unten) + ggf. kurze Code-Kommentare.
6. **Rückrollbar:** Jede Änderung ist allein revertierbar, ohne Folgebrüche.
7. **AT+-Kompatibilität:** Jeder Commit muss sicherstellen, dass **alle bestehenden AT+ Befehle** unverändert funktionieren.

---

## 4) Erlaubt pro Schritt (Beispiele)

* **Benennung:** Eine Funktion/Klasse/Variable sprechender benennen (projektweit konsistent).
* **Extraktion:** Langer Block → kleine Hilfsfunktion **im selben Modul**.
* **Duplikat entfernen:** Doppelte Logik in **eine** zentrale Funktion zusammenführen (ohne Semantikänderung).
* **Datei-Aufteilung:** Eine Klasse in eigene .h/.cpp-Dateien verschieben.
* **Kommentare/Docstrings:** Präzisieren, Missverständnisse entfernen, deutsch oder englisch – aber konsistent.
* **Formatierung:** `clang-format`/Einrückung/Leerzeilen vereinheitlichen.
* **AT+ Erweiterung:** Neue AT+ Befehle hinzufügen, solange bestehende Befehle unverändert bleiben.

### Nicht erlaubt (prohibitiv)

* Quer über **mehrere Module** gleichzeitig umbauen.
* **Signaturen** ändern, wenn Call-Sites zahlreich sind (erst Adapter/Semantic-Equivalent bereitstellen).
* **Verhalten ändern** ohne Bugfix-Nachweis und Test.
* Gleichzeitiger Mix aus Benennung **und** Logikänderung.
* **AT+ Befehle löschen oder ändern.**

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

* **Klassen:** `PascalCase` (z. B. `MotorDriver`)
* **Funktionen/Variablen:** `camelCase` (z. B. `calculateDistance`, `currentSpeed`)
* **Konstanten/Defines/Enums:** `UPPER_SNAKE_CASE` (`MAX_SPEED`)
* **Dateien:** `ModuleName.cpp/.h` oder `module_name.h` – Projektweit **eine** Konvention wählen.
* **Formatierung:** 4 Leerzeichen, Klammern immer setzen, keine Misch-Tabs.
* **Kommentare:** Kurz, präzise, erklären **warum**, nicht **was** (der Code zeigt das Was).

---

## 7) Minimal-Tests pro Schritt (Pflicht)

**Build:**

* Arduino Due Target baut ohne Fehler/Warnungen (möglichst `-Wall -Wextra`).

**Smoke-Tests (ohne Wiese):**

* Startsequenz bis „bereit“.
* Sensor-Init (GPS/IMU/Perimeter) meldet ok oder erwartete Mock-Werte.
* Motor-Enable/Disable ohne Bewegung (Trockentest) funktioniert.
* **AT+ Befehle:** Alle bisherigen Befehle reagieren unverändert korrekt. Neue Befehle zusätzlich dokumentiert und getestet.

**Feld-Mini-Test (wenn relevant):**

* 2–3 Min Mähen im sicheren Bereich. Beobachtung: keine „Autoscooter“-Kollisionen, kein Kreiseln, Perimeter-Erkennung ok.

**Telemetrie/Logs:**

* Relevante Logs bleiben gleich oder besser (kein Spam, keine neuen Fehler).

> Wenn einer dieser Punkte scheitert → **Revert** und Änderung nachbessern.

---

## 8) Sicherheitsnetze

* **Feature-Flags/Schalter:** Neues Verhalten optional machen (`#ifdef FEATURE_X`) – default **aus**.
* **Kill-Switch:** Sofort-Stop bleibt unberührt und wird nach jedem Build getestet.
* **Watchdogs:** Keine Deaktivierung ohne Ersatz.
* **Logging-Level:** Erhöhbar für Diagnose, aber in Normalbetrieb moderat.

---

## 9) Git-Strategie

* **Branching:** `main` stabil; pro Änderung ein **Feature-Branch** (`refactor/map/split-load()` etc.).
* **PR-Größe:** ≤ \~200 Zeilen Diff, klar fokussiert.
* **Reviews:** Mindestens 1 Review, Checkliste unten.
* **Tags:** Funktionsfähige Meilensteine taggen (`vX.Y-safe`).

---

## 10) Commit-/PR-Vorlage

**Commit-Subject:**

```
refactor(map): split load() into parseHeader() and readCells() (no behavior change)
```

**Body:**

```
WHY
- load() war >150 Zeilen; klare Verantwortlichkeiten fehlten.
- Lesbarkeit/Testbarkeit verbessern, ohne Semantik zu ändern.

WHAT
- Header-Parsing in parseHeader() extrahiert
- Zell-Lesen in readCells() extrahiert
- Unit-ähnliche Stub-Tests für parseHeader() mit Dummy-Stream

TEST
- Arduino Due Build ok (-Wall -Wextra)
- Smoke: Start/Init ok; Map laden von SD wie zuvor (Log identisch)
- 3-Min Feldtest: keine Abweichungen beobachtet
- AT+ Befehle: alle alten unverändert, keine Regressionen
```

---

## 11) Review-Checkliste (PR)

* [ ] Änderung ist **atomar** und auf **ein Modul** beschränkt.
* [ ] Commit-Nachricht erklärt **Warum & Was**.
* [ ] **Kein** unbeabsichtigter Verhaltenswechsel.
* [ ] Build sauber; Warnungen geprüft.
* [ ] Smoke-/Feldtests dokumentiert.
* [ ] Namens-/Stilregeln eingehalten.
* [ ] Revert wäre ohne Nebenwirkungen möglich.
* [ ] **AT+ Befehle vollständig kompatibel**.

---

## 12) Definition of Done (pro Schritt)

* [ ] Kompiliert für Arduino Due Ziel.
* [ ] Minimaltests grün (lokal protokolliert).
* [ ] Codequalität **verbessert** (konkreter Mehrwert benannt).
* [ ] Änderungsumfang klein (Diff ≈ überschaubar).
* [ ] Dokumentation (Commit/Kommentar) vorhanden.
* [ ] **AT+ Schnittstelle unverändert funktionsfähig.**

---

## 13) Beispiel-Fahrplan (erste Iterationen)

1. **Cleanup:** Entferne auskommentierte Blöcke in `Map` (keine Logikänderung).
2. **Refactor:** `Map::load()` in zwei private Helper aufteilen (Header/Cells).
3. **Benennung:** `foo()/bar()` zu `readCells()/calculateBoundary()` (nur wenn eindeutig).
4. **Perimeter:** Doppelte Schwellenwerte zentralisieren (Konstante + Kommentar).
5. **Motor:** Einfache Guard-Funktion `ensureSafeSpeed()` einführen (nur Aufruf, ohne neue Regeln).
6. **AT+ Tests:** Sicherstellen, dass nach Refactor alle bekannten Befehle unverändert antworten.

> Jede dieser Iterationen ist **allein** review- und testbar.

---

## 14) Tooling (Empfehlung)

* **Formatter:** `clang-format` (Projektweite `.clang-format`, z. B. LLVM-Basis, 4 Spaces).
* **Linter:** `clang-tidy` mit vorsichtiger Regelmenge (nur Lesbarkeit/Modernize, keine Semantik).
* **Build:** Reproduzierbares Arduino-CLI-Kommando im Repo dokumentieren (für **Arduino Due**). Zusätzlich Raspberry-Pi-Build des Alfred-Codes dokumentieren, um Interaktion sicherzustellen.

Beispiel `.clang-format`-Startpunkt:

```
BasedOnStyle: LLVM
IndentWidth: 4
TabWidth: 4
UseTab: Never
AllowShortIfStatementsOnASingleLine: false
BreakBeforeBraces: Allman
ColumnLimit: 120
SortIncludes: true
```

---

## 15) Notfall-Plan (Rollback)

* **Symptom** (z. B. Kreiseln/Autoscooter, AT+ Befehle reagieren nicht) erkannt → sofort Stop, Logs sichern, Branch **reverten**.
* Ticket mit: Commit-SHA, Symptome, Logs, reproduzierende Schritte, Hypothese.

---

## 16) Anhang: Risiko-Matrix (für Reviewer)

* **Low:** Kommentare/Format/Benennung, lokale Extraktion ohne neue Abhängigkeiten.
* **Medium:** Signaturänderung mit Adapter, Modul-interne Umverdrahtung.
* **High (vermeiden):** Zeitkritische Pfade, ISR, Motorsteuerung/Perimeter-Schwellen in einem Schritt, Multimodul-Umbauten, Änderungen am AT+ Kernprotokoll.

---

**Kurzfassung:** Klein anfangen, sauber dokumentieren, sofort testen, nie mehrere Dinge zugleich ändern – und immer eine sichere Rückkehrspur behalten. Zielplattform ist **Arduino Due**, die Interaktion mit dem **Raspberry-Pi Alfred Code** bleibt gewährleistet, und die **AT+ Schnittstelle bleibt vollständig kompatibel**.
