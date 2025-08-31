# Sunray Refactoring TODO

> **Ziel:** Schrittweise und sichere Verbesserung des Sunray Arduino-C++ Codes gemäß den Projekt-Regeln.
> **Plattform:** Arduino Due (primär), Alfred Raspberry Pi (kompatibel)
> **Prinzip:** Ein Schritt = ein Commit = ein Test = ein Mehrwert

---

## 🔧 Phase 1: Sofortige Cleanup-Maßnahmen (Low Risk)

### 1.1 Auskommentierte Code-Blöcke entfernen

**Beschreibung:** Entfernung von `//` und `/* */` kommentierten Code-Blöcken zur Reduzierung von Verwirrung und Dateigröße.

**Priorität:** Hoch  
**Risiko:** Niedrig  
**Aufwand:** 1-2 Stunden  

**Vorgehen:**
```bash
# Suche nach auskommentierten Blöcken
grep -r "//.*TODO\|//.*FIXME\|//.*XXX" sunray/
grep -r "/\*.*\*/" sunray/
```

**Beispiel-Commit:**
```
cleanup(map): remove commented debug code blocks (no behavior change)

WHY
- Commented code blocks create confusion and increase file size
- Dead code should be removed to improve readability

WHAT
- Removed 15 commented debug print statements in map.cpp
- Removed old algorithm implementation in findPath()
- Kept only essential comments explaining complex logic

TEST
- Arduino Due build: OK
- Smoke test: Map loading works as before
- No functional changes
```

---

### 1.2 Formatierung vereinheitlichen

**Beschreibung:** Anwendung einer konsistenten Code-Formatierung mit clang-format.

**Priorität:** Mittel  
**Risiko:** Niedrig  
**Aufwand:** 2-3 Stunden  

**Code-Vorlage (.clang-format):**
```yaml
BasedOnStyle: LLVM
IndentWidth: 4
TabWidth: 4
UseTab: Never
AllowShortIfStatementsOnASingleLine: false
BreakBeforeBraces: Allman
ColumnLimit: 120
SortIncludes: true
SpaceAfterCStyleCast: true
SpaceBeforeParens: ControlStatements
AlignConsecutiveAssignments: true
AlignConsecutiveDeclarations: true
```

**Vorgehen:**
```bash
# Formatierung anwenden
clang-format -i sunray/*.cpp sunray/*.h

# Nur bestimmte Dateien
clang-format -i sunray/map.cpp sunray/map.h
```

---

### 1.3 Magic Numbers durch Konstanten ersetzen

**Beschreibung:** Ersetzen von hartkodierten Zahlen durch benannte Konstanten.

**Priorität:** Hoch  
**Risiko:** Niedrig  
**Aufwand:** 3-4 Stunden  

**Code-Vorlage:**
```cpp
// Vorher in map.cpp
if (timeout > 1000) {
    return false;
}
float factor = 0.02;

// Nachher
const unsigned long PATHFINDER_TIMEOUT_MS = 1000;
const float GRID_RESOLUTION_FACTOR = 0.02;

if (timeout > PATHFINDER_TIMEOUT_MS) {
    return false;
}
float factor = GRID_RESOLUTION_FACTOR;
```

**Häufige Magic Numbers:**
- `1000`, `4000`, `5000` (Timeouts)
- `0.02`, `0.04`, `-0.04` (Faktoren)
- `20`, `50`, `100` (Schwellenwerte)
- Pin-Nummern
- Sensor-Kalibrierungswerte

---

## 🏗️ Phase 2: Strukturelle Verbesserungen (Medium Risk)

### 2.1 Große Funktionen aufteilen

**Beschreibung:** Aufteilung von Funktionen >100 Zeilen in kleinere, fokussierte Funktionen.

**Priorität:** Hoch  
**Risiko:** Mittel  
**Aufwand:** 4-6 Stunden pro Modul  

**Beispiel: map.cpp findPath() aufteilen**

**Vorher:**
```cpp
bool findPath(float startX, float startY, float targetX, float targetY) {
    // 257 Zeilen Code...
    // Initialisierung
    // A* Hauptschleife
    // Pfad-Rekonstruktion
    // Cleanup
}
```

**Nachher:**
```cpp
bool findPath(float startX, float startY, float targetX, float targetY) {
    if (!initializePathfinding(startX, startY, targetX, targetY)) {
        return false;
    }
    
    if (!executeAStarLoop()) {
        return false;
    }
    
    return reconstructPath();
}

private:
bool initializePathfinding(float startX, float startY, float targetX, float targetY);
bool executeAStarLoop();
bool reconstructPath();
```

---

### 2.2 Header-Dependencies optimieren

**Beschreibung:** Reduzierung von zirkulären Includes und unnötigen Dependencies.

**Priorität:** Mittel  
**Risiko:** Mittel  
**Aufwand:** 2-3 Stunden  

**Code-Vorlage:**
```cpp
// Vorher in map.h
#include "robot.h"
#include "motor.h"
#include "gps.h"

// Nachher in map.h
class Robot;  // Forward declaration
class Motor;  // Forward declaration
struct GPSData;  // Forward declaration

// In map.cpp
#include "robot.h"
#include "motor.h"
#include "gps.h"
```

---

### 2.3 Globale Variablen kapseln

**Beschreibung:** Verschiebung globaler Zustandsvariablen in Klassen für bessere Datenkapselung.

**Priorität:** Mittel  
**Risiko:** Mittel  
**Aufwand:** 3-4 Stunden pro Modul  

**Code-Vorlage:**
```cpp
// Vorher - Globale Variablen
float stateX = 0;
float stateY = 0;
float stateDelta = 0;
bool gpsJump = false;

// Nachher - Gekapselt in Klasse
class StateEstimator {
public:
    float getX() const { return stateX_; }
    float getY() const { return stateY_; }
    float getDelta() const { return stateDelta_; }
    bool hasGpsJump() const { return gpsJump_; }
    
    void updatePosition(float x, float y, float delta);
    
private:
    float stateX_ = 0;
    float stateY_ = 0;
    float stateDelta_ = 0;
    bool gpsJump_ = false;
};
```

---

## 🔧 Phase 3: Architektur-Verbesserungen (Medium Risk)

### 3.1 Error Handling vereinheitlichen

**Beschreibung:** Konsistente Fehlerbehandlung über alle Module.

**Priorität:** Hoch  
**Risiko:** Mittel  
**Aufwand:** 4-5 Stunden  

**Code-Vorlage:**
```cpp
// error_codes.h
enum class ErrorCode {
    SUCCESS = 0,
    GPS_TIMEOUT = 1,
    IMU_TIMEOUT = 2,
    MOTOR_OVERLOAD = 3,
    PERIMETER_LOST = 4,
    PATHFINDING_FAILED = 5
};

class ErrorHandler {
public:
    static void reportError(ErrorCode code, const char* message);
    static void clearError(ErrorCode code);
    static bool hasError(ErrorCode code);
    
private:
    static uint32_t errorFlags_;
};

// Verwendung
if (!gps.isAvailable()) {
    ErrorHandler::reportError(ErrorCode::GPS_TIMEOUT, "GPS signal lost");
    return false;
}
```

---

### 3.2 Logging-System verbessern

**Beschreibung:** Strukturiertes Logging mit konfigurierbaren Debug-Leveln.

**Priorität:** Mittel  
**Risiko:** Niedrig  
**Aufwand:** 3-4 Stunden  

**Code-Vorlage:**
```cpp
// logger.h
enum class LogLevel {
    ERROR = 0,
    WARN = 1,
    INFO = 2,
    DEBUG = 3,
    TRACE = 4
};

class Logger {
public:
    static void setLevel(LogLevel level);
    static void error(const char* module, const char* message);
    static void warn(const char* module, const char* message);
    static void info(const char* module, const char* message);
    static void debug(const char* module, const char* message);
    
private:
    static LogLevel currentLevel_;
};

// Makros für einfache Verwendung
#define LOG_ERROR(msg) Logger::error(__FILE__, msg)
#define LOG_WARN(msg) Logger::warn(__FILE__, msg)
#define LOG_INFO(msg) Logger::info(__FILE__, msg)
#define LOG_DEBUG(msg) Logger::debug(__FILE__, msg)

// Verwendung
LOG_INFO("GPS fix acquired");
LOG_DEBUG("Position updated: x=%.2f, y=%.2f", stateX, stateY);
```

---

### 3.3 State Machine explizit machen

**Beschreibung:** Explizite Definition von Robot-States und Zustandsübergängen.

**Priorität:** Mittel  
**Risiko:** Mittel  
**Aufwand:** 5-6 Stunden  

**Code-Vorlage:**
```cpp
// robot_state.h
enum class RobotState {
    IDLE,
    MOWING,
    DOCKING,
    CHARGING,
    ERROR,
    MANUAL
};

class StateMachine {
public:
    bool transitionTo(RobotState newState);
    RobotState getCurrentState() const { return currentState_; }
    bool canTransition(RobotState from, RobotState to) const;
    
private:
    RobotState currentState_ = RobotState::IDLE;
    unsigned long stateStartTime_ = 0;
    
    bool isValidTransition(RobotState from, RobotState to) const;
};

// Zustandsübergänge definieren
bool StateMachine::isValidTransition(RobotState from, RobotState to) const {
    switch (from) {
        case RobotState::IDLE:
            return (to == RobotState::MOWING || to == RobotState::DOCKING);
        case RobotState::MOWING:
            return (to == RobotState::IDLE || to == RobotState::DOCKING || to == RobotState::ERROR);
        // ...
    }
    return false;
}
```

---

## 🧪 Phase 4: Testbarkeit verbessern (Medium Risk)

### 4.1 Hardware-Abstraktion

**Beschreibung:** Abstraktion von Hardware-Zugriffen für bessere Testbarkeit.

**Priorität:** Mittel  
**Risiko:** Mittel  
**Aufwand:** 6-8 Stunden  

**Code-Vorlage:**
```cpp
// hardware_interface.h
class IGPSInterface {
public:
    virtual ~IGPSInterface() = default;
    virtual bool isAvailable() const = 0;
    virtual float getLatitude() const = 0;
    virtual float getLongitude() const = 0;
    virtual void update() = 0;
};

class IMotorInterface {
public:
    virtual ~IMotorInterface() = default;
    virtual void setSpeed(float left, float right) = 0;
    virtual void stop() = 0;
    virtual bool isOverloaded() const = 0;
};

// Echte Hardware-Implementierung
class GPSHardware : public IGPSInterface {
public:
    bool isAvailable() const override;
    float getLatitude() const override;
    float getLongitude() const override;
    void update() override;
};

// Mock für Tests
class GPSMock : public IGPSInterface {
public:
    void setPosition(float lat, float lon) { lat_ = lat; lon_ = lon; }
    bool isAvailable() const override { return available_; }
    float getLatitude() const override { return lat_; }
    float getLongitude() const override { return lon_; }
    void update() override {}
    
private:
    bool available_ = true;
    float lat_ = 0;
    float lon_ = 0;
};
```

---

### 4.2 Konfiguration zentralisieren

**Beschreibung:** Alle Konfigurationsparameter in eine zentrale Struktur.

**Priorität:** Mittel  
**Risiko:** Niedrig  
**Aufwand:** 3-4 Stunden  

**Code-Vorlage:**
```cpp
// config_manager.h
struct PathfinderConfig {
    bool useEuclideanHeuristic = true;
    unsigned long timeoutMs = 1000;
    float gridResolution = 0.02f;
    int maxIterations = 5000;
};

struct MotorConfig {
    float maxSpeed = 1.0f;
    float acceleration = 0.5f;
    int overloadThreshold = 800;
    unsigned long timeoutMs = 5000;
};

struct RobotConfig {
    PathfinderConfig pathfinder;
    MotorConfig motor;
    // ... weitere Module
};

class ConfigManager {
public:
    static RobotConfig& getInstance();
    static bool loadFromFile(const char* filename);
    static bool saveToFile(const char* filename);
    static bool validate();
    
private:
    static RobotConfig config_;
};

// Verwendung
auto& config = ConfigManager::getInstance();
if (config.pathfinder.useEuclideanHeuristic) {
    // Euclidean distance
} else {
    // Manhattan distance
}
```

---

## 🚀 Phase 5: Performance-Optimierungen (Low-Medium Risk)

### 5.1 Memory Management

**Beschreibung:** Optimierung der Speichernutzung und Reduzierung von Allokationen.

**Priorität:** Niedrig  
**Risiko:** Mittel  
**Aufwand:** 4-5 Stunden  

**Code-Vorlage:**
```cpp
// memory_pool.h
template<typename T, size_t N>
class StaticPool {
public:
    T* allocate() {
        for (size_t i = 0; i < N; ++i) {
            if (!used_[i]) {
                used_[i] = true;
                return &pool_[i];
            }
        }
        return nullptr;
    }
    
    void deallocate(T* ptr) {
        if (ptr >= pool_ && ptr < pool_ + N) {
            size_t index = ptr - pool_;
            used_[index] = false;
        }
    }
    
private:
    T pool_[N];
    bool used_[N] = {false};
};

// Verwendung für A* Nodes
using NodePool = StaticPool<AStarNode, 1000>;
static NodePool nodePool;

// Statt new/delete
AStarNode* node = nodePool.allocate();
// ... verwenden
nodePool.deallocate(node);
```

---

### 5.2 Loop-Optimierungen

**Beschreibung:** Optimierung häufig ausgeführter Schleifen und Berechnungen.

**Priorität:** Niedrig  
**Risiko:** Niedrig  
**Aufwand:** 2-3 Stunden  

**Code-Vorlage:**
```cpp
// Vorher - Wiederholte Berechnungen
for (int i = 0; i < nodes.size(); ++i) {
    float distance = sqrt(pow(nodes[i].x - targetX, 2) + pow(nodes[i].y - targetY, 2));
    if (distance < minDistance) {
        minDistance = distance;
        bestNode = &nodes[i];
    }
}

// Nachher - Optimiert
float targetXSquared = targetX * targetX;
float targetYSquared = targetY * targetY;
float minDistanceSquared = minDistance * minDistance;

for (int i = 0; i < nodes.size(); ++i) {
    float dx = nodes[i].x - targetX;
    float dy = nodes[i].y - targetY;
    float distanceSquared = dx * dx + dy * dy;  // Keine sqrt() nötig für Vergleich
    
    if (distanceSquared < minDistanceSquared) {
        minDistanceSquared = distanceSquared;
        bestNode = &nodes[i];
    }
}
minDistance = sqrt(minDistanceSquared);  // Nur einmal am Ende
```

---

## 📚 Phase 6: Dokumentation & Sicherheit (Kontinuierlich)

### 6.1 Code-Kommentare verbessern

**Beschreibung:** "Warum" statt "Was" erklären, Algorithmus-Beschreibungen hinzufügen.

**Priorität:** Niedrig  
**Risiko:** Niedrig  
**Aufwand:** 1-2 Stunden pro Modul  

**Code-Vorlage:**
```cpp
// Schlecht - erklärt WAS
// Setze Motor-Geschwindigkeit auf 0.5
motor.setSpeed(0.5);

// Gut - erklärt WARUM
// Reduziere Geschwindigkeit beim Annähern an Hindernisse für präzisere Steuerung
motor.setSpeed(APPROACH_SPEED);

/**
 * A* Pathfinding Algorithmus
 * 
 * Verwendet eine Heuristik (Manhattan oder Euclidean) um den optimalen Pfad
 * zwischen Start- und Zielpunkt zu finden. Die Implementierung berücksichtigt
 * Hindernisse und Perimeter-Grenzen.
 * 
 * @param startX Start-Position X-Koordinate (Meter)
 * @param startY Start-Position Y-Koordinate (Meter) 
 * @param targetX Ziel-Position X-Koordinate (Meter)
 * @param targetY Ziel-Position Y-Koordinate (Meter)
 * @return true wenn Pfad gefunden, false bei Timeout oder unmöglichem Pfad
 */
bool findPath(float startX, float startY, float targetX, float targetY);
```

---

### 6.2 Bounds-Checking

**Beschreibung:** Absicherung von Array-Zugriffen und Input-Validation.

**Priorität:** Hoch  
**Risiko:** Niedrig  
**Aufwand:** 2-3 Stunden  

**Code-Vorlage:**
```cpp
// Vorher - Unsicher
float getValue(int index) {
    return values[index];  // Potentieller Buffer-Overflow
}

// Nachher - Sicher
float getValue(int index) {
    if (index < 0 || index >= MAX_VALUES) {
        LOG_ERROR("Index out of bounds: %d", index);
        return 0.0f;  // Sicherer Default-Wert
    }
    return values[index];
}

// Template für sichere Array-Zugriffe
template<typename T, size_t N>
class SafeArray {
public:
    T& operator[](size_t index) {
        if (index >= N) {
            LOG_ERROR("Array index %zu out of bounds (size: %zu)", index, N);
            static T defaultValue{};
            return defaultValue;
        }
        return data_[index];
    }
    
    const T& operator[](size_t index) const {
        if (index >= N) {
            LOG_ERROR("Array index %zu out of bounds (size: %zu)", index, N);
            static T defaultValue{};
            return defaultValue;
        }
        return data_[index];
    }
    
    size_t size() const { return N; }
    
private:
    T data_[N];
};
```

---

## 📋 Implementierungs-Reihenfolge

### Woche 1-2: Cleanup
1. ✅ Auskommentierte Code-Blöcke entfernen (map.cpp)
2. ✅ Magic Numbers ersetzen (map.cpp)
3. ✅ Formatierung vereinheitlichen (map.cpp, map.h)

### Woche 3-4: Strukturell  
4. ⏳ findPath() Funktion aufteilen
5. ⏳ Header-Dependencies optimieren
6. ⏳ Globale Variablen in StateEstimator kapseln

### Woche 5-6: Architektur
7. ⏳ Error Handling vereinheitlichen
8. ⏳ Logging-System implementieren
9. ⏳ State Machine explizit machen

### Woche 7-8: Testbarkeit
10. ⏳ Hardware-Abstraktion für GPS/Motor
11. ⏳ Konfiguration zentralisieren
12. ⏳ Unit-Test Framework einrichten

### Woche 9-10: Performance
13. ⏳ Memory Pool für A* Nodes
14. ⏳ Loop-Optimierungen
15. ⏳ Sensor-Reading-Frequenz optimieren

### Kontinuierlich: Dokumentation & Sicherheit
16. ⏳ Code-Kommentare verbessern
17. ⏳ Bounds-Checking hinzufügen
18. ⏳ Watchdog-Verbesserungen

---

## 🔍 Testing-Checkliste pro Schritt

### Build-Test
```bash
# Arduino Due Build
arduino-cli compile --fqbn arduino:sam:arduino_due_x sunray/

# Warnings prüfen
arduino-cli compile --fqbn arduino:sam:arduino_due_x --warnings all sunray/
```

### Smoke-Test
- [ ] Startsequenz bis "bereit"
- [ ] GPS-Init meldet OK oder Mock-Werte
- [ ] IMU-Init meldet OK oder Mock-Werte  
- [ ] Motor Enable/Disable ohne Bewegung
- [ ] AT+ Befehle reagieren korrekt
- [ ] Keine neuen Fehler in Logs

### Feld-Test (wenn relevant)
- [ ] 2-3 Min Mähen ohne Kollisionen
- [ ] Perimeter-Erkennung funktioniert
- [ ] Keine unerwarteten Stopps
- [ ] Telemetrie-Daten plausibel

---

## 🚨 Rollback-Plan

**Bei Problemen:**
1. Sofort stoppen und Logs sichern
2. Branch reverten: `git revert <commit-sha>`
3. Ticket erstellen mit:
   - Commit-SHA
   - Symptome
   - Logs
   - Reproduzierende Schritte
   - Hypothese

**Notfall-Kommandos:**
```bash
# Letzten Commit rückgängig machen
git revert HEAD

# Zu letztem funktionierenden Tag zurück
git checkout v1.0-safe

# Änderungen verwerfen
git reset --hard HEAD
```

---

## 📊 Fortschritts-Tracking

**Metriken:**
- [ ] Zeilen Code reduziert
- [ ] Anzahl Magic Numbers eliminiert
- [ ] Funktionen <100 Zeilen
- [ ] Test-Coverage erhöht
- [ ] Build-Warnings reduziert
- [ ] Memory-Usage optimiert

**Tools:**
```bash
# Code-Metriken
cloc sunray/

# Komplexität messen
lizard sunray/

# Memory-Usage analysieren
arduino-cli compile --fqbn arduino:sam:arduino_due_x --verbose sunray/
```

---

*Letzte Aktualisierung: $(date)*  
*Nächste Review: In 1 Woche*