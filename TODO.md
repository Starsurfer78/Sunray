# Sunray Arduino Refactoring TODO

> **Ziel:** Schrittweise und sichere Verbesserung des Sunray Arduino-C++ Codes unter Beachtung der PRD-Regeln

## 🎯 Überblick

Diese TODO-Liste basiert auf einer umfassenden Analyse des Sunray Arduino Codes und identifiziert konkrete Refactoring-Potentiale. Alle Änderungen folgen den PRD-Regeln: **Stabilität vor Eleganz**, **Arduino Due Kompatibilität**, **AT+ Interface Kompatibilität** mit Alfred.

---

## 📋 Niedrig-Risiko Verbesserungen (Sofort umsetzbar)

### 🧹 Code Cleanup

- [x] **Auskommentierte Code-Blöcke entfernen** ✅
  - `LineTracker.cpp`: Zeilen mit `//PID pidLine(0.2, 0.01, 0)` und `//pidLine.w = 0;`
  - `map.cpp`: Debugging-Kommentare in `calcMapCRC()` und `setExclusionLength()`
  - **Risiko:** Niedrig - Keine Logikänderung
  - **Test:** Arduino Due Build + Smoke Test
  - **Status:** Abgeschlossen - Alle auskommentierten Debug-Zeilen entfernt

- [x] **Magic Numbers durch Konstanten ersetzen** ✅
  - `LineTracker.cpp`: 15 Konstanten definiert (ROTATION_ANGULAR_SPEED, APPROACH_SPEED, etc.)
  - `MowOp.cpp`: MOTOR_OVERLOAD_TIMEOUT (20000ms), MAP_ROUTING_FAILED_RESET_VALUE (60)
  - **Risiko:** Niedrig - Nur Lesbarkeit
  - **Test:** Verhalten identisch
  - **Status:** Abgeschlossen - Alle wichtigen Magic Numbers durch sprechende Konstanten ersetzt

- [x] **WLAN-Verbindungsmanagement verbessert** ✅
  - Einfache WLAN-Neustart-Lösung implementiert (`SimpleWifiRestart.h/cpp`)
  - Automatischer WLAN-Adapter-Neustart bei Verbindungsproblemen
  - Neue AT+ Befehle: `AT+WIFI_RESTART`, `AT+WIFI_STATUS`
  - **Risiko:** Niedrig - Optionale Erweiterung
  - **Test:** Arduino Due Build erfolgreich (178.908 Bytes, 34%)
  - **Status:** Abgeschlossen - Einfache und wartbare WLAN-Lösung bereitgestellt

- [ ] **Inkonsistente Kommentare vereinheitlichen**
  - Deutsch vs. Englisch Mischung beseitigen
  - Veraltete Kommentare aktualisieren
  - **Risiko:** Niedrig - Nur Dokumentation

### 🏷️ Naming Improvements

- [x] **Variablennamen verbessern** ✅
  - `printmotoroverload` → `hasLoggedMotorOverload` (umbenannt)
  - `trackerDiffDelta_positive` → entfernt (ungenutzt)
  - **Risiko:** Niedrig - Nur Umbenennung
  - **Test:** Kompilierung + Funktionstest
  - **Status:** Abgeschlossen - Bessere Semantik für Motor-Overload-Logik

- [x] **Funktionsnamen sprechender machen**
  - `Op::name()` → `Op::getOperationName()` (bereits umbenannt)
  - `Map::distance()` → `Map::calculateDistance()` (bereits umbenannt)
  - **Risiko:** Niedrig - Lokale Änderungen
  - **Status:** Abgeschlossen - Funktionsnamen sind bereits sprechender

---

## 🔧 Medium-Risiko Optimierungen (Nach Cleanup)

### 📦 Funktions-Extraktion

- [x] **`trackLine()` Funktion aufteilen (377 Zeilen)** ✅
  - Extrahiert: `calculateTrackingParams()` - Grundlegende Tracking-Parameter
  - Extrahiert: `calculateAngleToTargetFits()` - Rotationsentscheidung
  - Extrahiert: `handleRotationControl()` - Rotationssteuerung
  - Extrahiert: `calculateLinearSpeed()` - Lineare Geschwindigkeitsberechnung
  - Extrahiert: `calculateAngularSpeed()` - Winkelgeschwindigkeitsberechnung (Stanley-Regler)
  - **Risiko:** Medium - Große Funktion
  - **Test:** Intensive Feld-Tests erforderlich
  - **Status:** Abgeschlossen - Funktion von 377 auf ~50 Zeilen pro Teilfunktion reduziert

- [ ] **`MowOp::run()` vereinfachen**
  - Extrahiere: `checkTimetableConditions()`
  - Extrahiere: `performObstacleDetection()`
  - **Risiko:** Medium - Kernlogik
  - **Test:** Mäh-Operationen testen

- [ ] **`Map::findPath()` modularisieren**
  - Extrahiere: `initializePathfinding()`
  - Extrahiere: `processPathfindingNodes()`
  - Extrahiere: `reconstructPath()`
  - **Risiko:** Medium - Kritische Navigation
  - **Test:** Pfadplanung in verschiedenen Szenarien

### 🔄 Code-Duplikate eliminieren

- [x] **Überstrom-Checks vereinheitlichen** ✅
  - `MOTOR_OVERLOAD_TIMEOUT` Konstante in `config.h` und `config_alfred.h` zentralisiert
  - Hardcodierte 20000ms Werte durch Konstante ersetzt
  - **Risiko:** Medium - Sicherheitskritisch
  - **Test:** Arduino Due Build erfolgreich
  - **Status:** Abgeschlossen - Konsistente Überstrom-Timeouts in Arduino und Alfred Code

- [x] **GPS-Validierung zentralisieren** ✅
  - Duplizierte GPS-Validierungslogik in `onGpsFixTimeout()` und `onGpsNoSignal()` in zentrale Hilfsfunktion `handleGpsIssue(Sensor sensorType, Op& targetOp)` extrahiert
  - Beide Funktionen rufen jetzt die neue Hilfsfunktion auf
  - Keine Verhaltensänderung, nur Code-Deduplizierung
  - **Risiko:** Medium - Navigation kritisch
  - **Test:** Arduino Due Build-Test erfolgreich (34% Speichernutzung)
  - **Status:** Abgeschlossen - GPS-Fehlerbehandlung zentralisiert

- [ ] **Quaternion-Berechnungen zusammenfassen**
  - Mehrere IMU-Module haben ähnliche Berechnungen
  - Zentrale Utility-Klasse: `QuaternionMath`
  - **Risiko:** Medium - Orientierung kritisch
  - **Test:** IMU-Kalibrierung und -Drift

### 🏗️ Strukturelle Verbesserungen

- [ ] **Error Handling vereinheitlichen**
  - Konsistente Fehlerbehandlung in allen Ops
  - Zentrale `ErrorManager` Klasse
  - **Risiko:** Medium - Fehlerbehandlung
  - **Test:** Alle Fehlerzustände durchspielen

- [x] **Memory Management verbessert** ✅
  - `map.cpp`: StackAllocator für Point-Arrays implementiert (kleine/mittlere Puffer)
  - Memory Corruption Checks mit MemoryGuard-System verbessert
  - Memory Allocation Error Handling vereinheitlicht (RingBuffer, PubSubClient, base64)
  - Memory Leak Prevention durch korrekte Destruktoren
  - Memory Monitoring mit zentraler MemoryMonitor-Klasse
  - **Risiko:** Medium - Speicher kritisch
  - **Test:** Arduino Due Build erfolgreich (179516 Bytes, 34%)
  - **Status:** Abgeschlossen - Umfassende Speicherverwaltungsverbesserungen implementiert

---

## ⚠️ Hoch-Risiko Änderungen (Später, nach Stabilisierung)

### 🔌 Abstraktion & Interfaces

- [ ] **Sensor-Abstraktion einführen**
  - Einheitliche `ISensor` Interface
  - GPS, IMU, Bumper, Sonar implementieren Interface
  - **Risiko:** Hoch - Alle Sensoren betroffen
  - **Test:** Umfangreiche Hardware-Tests

- [ ] **Communication-Layer abstrahieren**
  - `SerialRobotDriver` Interface-basiert machen
  - CAN/Serial/Mock Implementierungen
  - **Risiko:** Hoch - Alfred-Kommunikation
  - **Test:** AT+ Kompatibilität sicherstellen

- [ ] **Config-System überarbeiten**
  - Zentrale Konfigurationsvalidierung
  - Runtime-Konfiguration vs. Compile-Time
  - **Risiko:** Hoch - Grundlegende Architektur
  - **Test:** Alle Konfigurationskombinationen

### 🧪 Testability Improvements

- [ ] **Dependency Injection einführen**
  - Globale Variablen durch injizierte Dependencies ersetzen
  - **Risiko:** Hoch - Architekturänderung
  - **Test:** Vollständige Systemtests

- [ ] **Mock-Framework für Hardware**
  - Simulierte Sensoren für Unit-Tests
  - **Risiko:** Hoch - Neue Infrastruktur
  - **Test:** Hardware vs. Mock Vergleich

---

## 🔍 Spezifische Code-Stellen (Priorität nach Analyse)

### Kritische Funktionen (>100 Zeilen)

1. **`trackLine()` - 377 Zeilen** (LineTracker.cpp:33)
   - Stanley-Controller Implementierung
   - Geschwindigkeitsregelung
   - Hindernisvermeidung

2. **`Map::findPath()` - ~300 Zeilen** (map.cpp:1830)
   - A*-Pathfinding Algorithmus
   - Hindernis-Umgehung
   - Speicher-intensive Operationen

3. **`SparkFun_Ublox_Arduino_Library::processResponse()`**
   - GPS-Message Parsing
   - Komplexe State Machine

### Duplikate mit hoher Priorität

1. **Überstrom-Detection**
   - `motor.cpp`: `checkMotorFault()`
   - `MowOp.cpp`: `onMotorOverload()`
   - `LineTracker.cpp`: Motor-Überlast Logik

2. **GPS-Fehlerbehandlung**
   - `MowOp::onGpsFixTimeout()`
   - `MowOp::onGpsNoSignal()`
   - Ähnliche Logik in anderen Ops


---

## 🛠️ Tooling & Infrastruktur

### Build-System

- [ ] **Arduino CLI Setup dokumentieren**
  - Reproduzierbare Build-Befehle
  - Arduino Due spezifische Flags
  - **Datei:** `BUILD.md` erstellen

- [x] **clang-format Konfiguration** ✅
  - Projektweite Code-Formatierung
  - 4 Spaces, Allman Braces
  - **Datei:** `.clang-format` erstellt
  - **Status:** Abgeschlossen - Konfiguration im Projektroot verfügbar

- [ ] **Pre-commit Hooks**
  - Automatische Formatierung
  - Build-Check vor Commit
  - **Datei:** `.pre-commit-config.yaml`

### Testing

- [ ] **Smoke-Test Suite**
  - Automatisierte Grundfunktions-Tests
  - AT+ Befehl Regression-Tests
  - **Ordner:** `tests/smoke/`

- [ ] **Hardware-in-the-Loop Tests**
  - Sensor-Mock Framework
  - Motor-Simulation
  - **Ordner:** `tests/hil/`

---

## 📊 Metriken & Tracking

### Code-Qualität Ziele

- [ ] **Funktionslänge reduzieren**
  - Ziel: Keine Funktion >100 Zeilen
  - Aktuell: 15+ Funktionen >100 Zeilen

- [ ] **Duplikation eliminieren**
  - Ziel: <5% Code-Duplikation
  - Aktuell: ~15% geschätzt

- [ ] **Magic Numbers beseitigen**
  - Ziel: Alle numerischen Konstanten benannt
  - Aktuell: 50+ Magic Numbers identifiziert

### Test-Coverage

- [ ] **AT+ Interface: 100% Coverage**
  - Alle Befehle automatisch getestet
  - Regression-Tests für Alfred-Kompatibilität

- [ ] **Kritische Pfade: 90% Coverage**
  - Navigation, Motor-Control, Safety

---

## 🚨 Sicherheits-Checkliste

### Vor jedem Refactoring

- [ ] **Backup erstellen**
  - Git-Tag mit funktionierender Version
  - Dokumentierte Rollback-Prozedur

- [ ] **AT+ Kompatibilität sicherstellen**
  - Alle bestehenden Befehle testen
  - Alfred-Integration verifizieren

- [ ] **Arduino Due Build**
  - Kompilierung ohne Warnings
  - Flash-Speicher Nutzung prüfen

### Nach jedem Refactoring

- [ ] **Smoke Tests**
  - Startup-Sequenz
  - Sensor-Initialisierung
  - Motor-Enable/Disable

- [ ] **Feld-Test (minimal)**
  - 2-3 Minuten sicheres Mähen
  - Hindernis-Erkennung
  - Perimeter-Respekt

---

## 📅 Roadmap (Empfohlene Reihenfolge)

### Phase 1: Foundation (Wochen 1-2)
1. Code Cleanup (auskommentierte Blöcke)
2. Magic Numbers → Konstanten
3. Naming Improvements
4. Build-System dokumentieren

### Phase 2: Structure (Wochen 3-4)
1. `trackLine()` aufteilen
2. Überstrom-Checks vereinheitlichen
3. Error Handling verbessern
4. Smoke-Test Suite

### Phase 3: Quality (Wochen 5-6)
1. Code-Duplikate eliminieren
2. ✅ Memory Management optimieren (Abgeschlossen)
3. GPS-Validierung zentralisieren
4. Umfangreiche Tests

### Phase 4: Architecture (Wochen 7+)
1. Sensor-Abstraktion (falls nötig)
2. Communication-Layer (falls nötig)
3. Config-System (falls nötig)
4. Vollständige Test-Coverage

---

## 📝 Notizen

### Wichtige Erkenntnisse aus der Analyse

- **Arduino Due Kompatibilität:** Vollständig gegeben
- **Alfred Integration:** AT+ Interface funktional
- **Code-Qualität:** Solide Basis, aber Verbesserungspotential
- **Kritische Bereiche:** Navigation, Motor-Control, Sensor-Fusion

### Risiko-Bewertung

- **Niedrig:** Cleanup, Naming, Konstanten
- **Medium:** Funktions-Extraktion, Duplikat-Elimination
- **Hoch:** Architektur-Änderungen, Interface-Abstraktion

### Erfolgs-Kriterien

1. **Funktionalität:** Keine Regression in Mäh-Performance
2. **Stabilität:** Mindestens gleiche Zuverlässigkeit
3. **Wartbarkeit:** Verbesserte Code-Lesbarkeit
4. **Kompatibilität:** AT+ Interface unverändert
5. **Performance:** Keine Verschlechterung der Laufzeit

---

## 🎉 Abgeschlossene Verbesserungen (Januar 2025)

### ✅ Speicherverwaltung (Memory Management)

**Umfassende Verbesserungen der Speicherverwaltung erfolgreich implementiert:**

1. **StackAllocator für Point-Arrays**
   - Neue `StackAllocator`-Klasse in `src/stack_allocator.h/cpp`
   - Stack-basierte Allokation für kleine (≤8) und mittlere (≤32) Point-Arrays
   - Integration in `Polygon::alloc()` und `Polygon::dealloc()` in `map.cpp`
   - Reduziert dynamische Heap-Allokationen bei häufigen Polygon-Operationen

2. **Memory Corruption Checks**
   - MemoryGuard-System implementiert mit Canary-Pattern-Validierung
   - Verbesserte Corruption-Detection in kritischen Speicherbereichen

3. **Memory Allocation Error Handling**
   - RingBuffer Destruktor korrekt implementiert
   - PubSubClient und base64 Bibliotheken verbessert
   - Einheitliche Fehlerbehandlung bei Speicher-Allokationsfehlern

4. **Memory Leak Prevention**
   - Korrekte Destruktor-Implementierungen
   - LinkedList Validierung und Cleanup
   - Automatische Puffer-Freigabe im StackAllocator

5. **Memory Monitoring**
   - Zentrale MemoryMonitor-Klasse erstellt
   - Konsolidierte freeMemory() Funktionen
   - Verbesserte Speicher-Überwachung und -Diagnostik

**Ergebnis:** Arduino Due Build erfolgreich (179516 Bytes, 34% Speichernutzung)
**Status:** Alle Speicherverwaltungsverbesserungen vollständig implementiert und getestet

---

### Erfolgs-Kriterien

1. **Funktionalität:** Keine Regression in Mäh-Performance
2. **Stabilität:** Mindestens gleiche Zuverlässigkeit
3. **Wartbarkeit:** Verbesserte Code-Lesbarkeit
4. **Kompatibilität:** AT+ Interface unverändert
5. **Performance:** Keine Verschlechterung der Laufzeit

---

*Erstellt basierend auf umfassender Code-Analyse des Sunray Arduino Projekts*
*Letzte Aktualisierung: Januar 2025*
*Folgt PRD-Regeln: Stabilität vor Eleganz, Arduino Due Zielplattform, AT+ Kompatibilität*