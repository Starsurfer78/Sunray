# TODO_MAP.md - Refactoring Plan für map.cpp

## Übersicht
Dieses Dokument beschreibt den schrittweisen Refactoring-Plan für `map.cpp` gemäß den Projekt-Rules. Jeder Schritt ist atomar, kompilierbar und rückrollbar.

## Aktuelle Problembereiche

### 1. Monolithische Struktur
- `map.cpp` enthält 4 verschiedene Klassen (Point, Polygon, PolygonList, Map)
- Über 1000 Zeilen Code in einer Datei
- Vermischte Verantwortlichkeiten

### 2. Unzureichende Kommentierung
- Fehlende Dokumentation für komplexe Algorithmen
- Magic Numbers ohne Erklärung
- Unklare Funktionszwecke

### 3. Memory Management
- Manuelle Speicherverwaltung ohne RAII
- Potentielle Memory Leaks
- Komplexe Pointer-Arithmetik

## Refactoring-Phasen

### Phase 1: Cleanup & Kommentierung (Wochen 1-2)

#### Schritt 1.1: Auskommentierten Code entfernen
- **Datei:** `map.cpp`
- **Ziel:** Entfernung toter Code-Blöcke
- **Test:** Kompilierung + Smoke-Test
- **Risiko:** Low

#### Schritt 1.2: Magic Numbers durch Konstanten ersetzen
- **Beispiel:** `if (distance < 0.3)` → `if (distance < MIN_POINT_DISTANCE)`
- **Datei:** `map.h` (Konstanten), `map.cpp` (Verwendung)
- **Test:** Funktionstest Navigation
- **Risiko:** Low

#### Schritt 1.3: Basis-Kommentierung hinzufügen
- **Ziel:** Jede öffentliche Methode dokumentieren
- **Format:** Doxygen-Style Kommentare
- **Sprache:** Deutsch (gemäß Projekt-Konvention)
- **Risiko:** Low

### Phase 2: Klassen-Trennung (Wochen 3-4)

#### Schritt 2.1: Point-Klasse extrahieren
- **Neue Dateien:** `Point.h`, `Point.cpp`
- **Abhängigkeiten:** Keine
- **Test:** Geometrie-Berechnungen
- **Risiko:** Low

#### Schritt 2.2: Polygon-Klassen extrahieren
- **Neue Dateien:** `Polygon.h`, `Polygon.cpp`
- **Abhängigkeiten:** `Point.h`
- **Test:** Polygon-Operationen
- **Risiko:** Medium

#### Schritt 2.3: GeometryUtils extrahieren
- **Neue Dateien:** `GeometryUtils.h`, `GeometryUtils.cpp`
- **Inhalt:** Statische Hilfsfunktionen für geometrische Berechnungen
- **Test:** Distanz- und Winkelberechnungen
- **Risiko:** Low

### Phase 3: Map-Klasse aufteilen (Wochen 5-6)

#### Schritt 3.1: MemoryManager extrahieren
- **Neue Dateien:** `MemoryManager.h`, `MemoryManager.cpp`
- **Verantwortung:** Speicherallokation, Corruption-Checks
- **Test:** Memory-Leak-Tests
- **Risiko:** Medium

#### Schritt 3.2: MapPersistence extrahieren
- **Neue Dateien:** `MapPersistence.h`, `MapPersistence.cpp`
- **Verantwortung:** Laden/Speichern von Karten
- **Test:** SD-Karten-Operationen
- **Risiko:** Medium

#### Schritt 3.3: WaypointManager extrahieren
- **Neue Dateien:** `WaypointManager.h`, `WaypointManager.cpp`
- **Verantwortung:** Wegpunkt-Verwaltung und Navigation
- **Test:** Mäh-Sequenzen
- **Risiko:** High

### Phase 4: Navigation & Pathfinding (Wochen 7-8)

#### Schritt 4.1: PathFinder extrahieren
- **Neue Dateien:** `PathFinder.h`, `PathFinder.cpp`
- **Verantwortung:** A*-Algorithmus und Pfadplanung
- **Test:** Hindernis-Umfahrung
- **Risiko:** High

#### Schritt 4.2: ObstacleManager extrahieren
- **Neue Dateien:** `ObstacleManager.h`, `ObstacleManager.cpp`
- **Verantwortung:** Dynamische Hindernisse
- **Test:** Kollisionsvermeidung
- **Risiko:** Medium

## Detaillierte Schritte

### Schritt 1.1: Cleanup auskommentierter Code

**Betroffene Bereiche:**
- Zeilen mit `//` auskommentiertem Code
- `#if 0` Blöcke
- Ungenutzte Debug-Ausgaben

**Vorgehen:**
1. Suche nach auskommentierten Blöcken
2. Prüfung auf historische Relevanz
3. Entfernung nach Bestätigung
4. Kompilierung und Test

**Commit-Message:**
```
cleanup(map): remove commented-out code blocks (no behavior change)

WHY
- Verbessert Lesbarkeit
- Reduziert Dateigröße
- Entfernt verwirrende alte Code-Fragmente

WHAT
- Entfernung von 15 auskommentierten Blöcken
- Keine aktive Logik betroffen

TEST
- Arduino Due Build ok
- Smoke-Test: Initialisierung erfolgreich
- AT+ Befehle: alle unverändert
```

### Schritt 1.2: Konstanten einführen

**Neue Konstanten in map.h:**
```cpp
// Geometrische Konstanten
static const float MIN_POINT_DISTANCE = 0.3f;  // Minimaler Abstand zwischen Punkten (m)
static const float MAX_POLYGON_AREA = 10000.0f; // Maximale Polygonfläche (m²)
static const int MAX_WAYPOINTS = 1000;          // Maximale Anzahl Wegpunkte

// Memory Management
static const uint32_t MEMORY_CORRUPTION_MARKER = 0xDEADBEEF;
static const int ALLOCATION_ALIGNMENT = 4;       // Byte-Alignment für Allokationen

// CRC Berechnung
static const uint16_t CRC_POLYNOMIAL = 0x1021;  // CRC-16-CCITT Polynom
```

## Test-Strategie

### Build-Tests
- Arduino Due Kompilierung mit `-Wall -Wextra`
- Keine neuen Warnungen
- Speicherverbrauch überwachen

### Funktions-Tests
- **Geometrie:** Point-Distanzen, Polygon-Containment
- **Navigation:** Wegpunkt-Sequenzen, Pfadplanung
- **Persistierung:** Karten laden/speichern
- **Memory:** Leak-Detection, Corruption-Checks

### AT+ Kompatibilität
- Alle bestehenden Befehle unverändert
- Antwortzeiten gleich oder besser
- Keine neuen Fehler-Codes

### Feld-Tests
- 5-Minuten Mäh-Test nach jedem Schritt
- Perimeter-Erkennung
- Docking-Sequenz
- Notfall-Stop

## Rollback-Plan

### Automatische Trigger
- Kompilierungsfehler
- Fehlgeschlagene Smoke-Tests
- AT+ Befehl-Regression
- Memory-Corruption-Fehler

### Manuelle Trigger
- Unerwartetes Roboter-Verhalten
- Performance-Verschlechterung
- Instabile Navigation

### Rollback-Prozess
1. Sofortiger Stop aller Tests
2. Git-Revert des letzten Commits
3. Kompilierung und Basis-Test
4. Dokumentation des Problems
5. Analyse vor nächstem Versuch

## Metriken & Erfolgs-Kriterien

### Code-Qualität
- **Zeilen pro Datei:** < 500 (aktuell: >1000)
- **Funktionen pro Klasse:** < 20 (aktuell: >30)
- **Zyklomatische Komplexität:** < 10 pro Funktion
- **Kommentarabdeckung:** > 80% öffentliche Methoden

### Performance
- **Kompilierzeit:** Nicht schlechter als vorher
- **RAM-Verbrauch:** ±5% des aktuellen Wertes
- **Ausführungszeit kritischer Pfade:** Keine Verschlechterung

### Wartbarkeit
- **Modulare Tests:** Jede Klasse einzeln testbar
- **Klare Abhängigkeiten:** Keine zirkulären Includes
- **Dokumentation:** Jede öffentliche API dokumentiert

## Zeitplan

| Woche | Phase | Schritte | Risiko |
|-------|-------|----------|--------|
| 1 | Cleanup | 1.1-1.2 | Low |
| 2 | Kommentierung | 1.3 | Low |
| 3 | Klassen-Trennung | 2.1-2.2 | Low-Medium |
| 4 | Geometrie-Utils | 2.3 | Low |
| 5 | Map-Aufspaltung | 3.1-3.2 | Medium |
| 6 | Waypoint-Manager | 3.3 | High |
| 7 | PathFinder | 4.1 | High |
| 8 | ObstacleManager | 4.2 | Medium |

## Nächste Schritte

1. **Sofort:** Schritt 1.1 - Cleanup auskommentierter Code
2. **Diese Woche:** Schritt 1.2 - Konstanten einführen
3. **Nächste Woche:** Schritt 1.3 - Basis-Kommentierung

---

*Erstellt: $(date)*  
*Letzte Aktualisierung: $(date)*  
*Verantwortlich: Refactoring Team*  
*Status: In Bearbeitung*