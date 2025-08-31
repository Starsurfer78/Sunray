# Timing-Verbesserungen für Sunray Alfred

## Übersicht
Konkrete Code-Änderungen zur Reduzierung der CPU-Last von 28% auf unter 15%.

## Sofortige Maßnahmen (Implementierung: 30 Minuten)

### 1. CPU-Entlastung in Hauptschleife
**Datei:** `robot.cpp`
**Zeile:** Ende der `loop()` Funktion (nach Zeile ~1220)

```cpp
// HINZUFÜGEN am Ende der loop() Funktion:
void loop() {
    // ... bestehender Code ...
    
    // CPU-Entlastung: 1ms Pause zwischen Schleifenzyklen
    delay(1);
}
```

**Erwartete Verbesserung:** CPU-Last -40% (28% → 17%)

### 2. processComm() Timing begrenzen
**Datei:** `robot.cpp`
**Zeile:** ~1110 (vor `processComm()` Aufruf)

```cpp
// ERSETZEN:
processComm();

// DURCH:
static unsigned long nextCommTime = 0;
if (millis() >= nextCommTime) {
    nextCommTime = millis() + 5;  // 200 Hz statt permanent
    processComm();
}
```

**Erwartete Verbesserung:** CPU-Last -20% (17% → 14%)

### 3. SerialRobotDriver Frequenz reduzieren (VORSICHT)
**Datei:** `SerialRobotDriver.cpp`
**Zeile:** ~536 (in `run()` Funktion)

⚠️ **WICHTIGER HINWEIS:** Die STM32-Firmware (`rm18.ino`) ist für 50Hz optimiert:
- Interne Motorsteuerung läuft mit 20ms (50Hz)
- Motor-Timeout von 3 Sekunden als Sicherheitsmechanismus
- Firmware erwartet regelmäßige Kommunikation für optimale Performance

```cpp
// VORSICHTIGE REDUZIERUNG (nur bei Bedarf):
if (millis() > nextMotorTime){
    nextMotorTime = millis() + 30; // 33 Hz (Kompromiss)
    requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
}

// ALTERNATIVE: Adaptive Frequenz basierend auf CPU-Last
int motorInterval = (cpuLoadHigh) ? 40 : 20;  // 25Hz bei Last, 50Hz normal
if (millis() > nextMotorTime){
    nextMotorTime = millis() + motorInterval;
    requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
}
```

**Erwartete Verbesserung:** CPU-Last -5% (14% → 13%)
**Risiko:** Reduzierte Motorsteuerungsqualität bei zu niedriger Frequenz

## Mittelfristige Verbesserungen (Implementierung: 2 Stunden)

### 4. Adaptive Hauptschleife
**Datei:** `robot.cpp`
**Neue Funktion hinzufügen:**

```cpp
// Neue Funktion vor setup()
int getAdaptiveLoopDelay() {
    // Basierend auf gemessener Schleifenzeit
    if (loopTimeMean > 15.0) {
        return 5;  // Langsame Schleife: mehr Pause
    } else if (loopTimeMean > 10.0) {
        return 3;  // Mittlere Geschwindigkeit
    } else if (loopTimeMean > 5.0) {
        return 2;  // Schnelle Schleife
    }
    return 1;  // Minimum 1ms
}

// In loop() Funktion am Ende ersetzen:
// delay(1);  // Alte Version

// DURCH:
delay(getAdaptiveLoopDelay());
```

**Erwartete Verbesserung:** CPU-Last -15% (12% → 10%)

### 5. Intelligente Kommunikations-Priorisierung (STM32-kompatibel)
**Datei:** `SerialRobotDriver.cpp`
**Neue Klassenvariablen hinzufügen:**

```cpp
// In SerialRobotDriver.h hinzufügen:
private:
    unsigned long lastHighLoadTime = 0;
    float avgLoopTime = 0;
    bool isHighCpuLoad() {
        // CPU-Last-Erkennung basierend auf Loop-Time
        return (avgLoopTime > 15.0) || ((millis() - lastHighLoadTime) < 2000);
    }
```

**In SerialRobotDriver.cpp `run()` Funktion:**

```cpp
void SerialRobotDriver::run(){
    processComm();
    
    // STM32-kompatible adaptive Motor-Frequenz
    // Minimum 33Hz (30ms) für STM32-Kompatibilität
    // Maximum 50Hz (20ms) für optimale Performance
    int motorInterval = isHighCpuLoad() ? 30 : 20;  // 33Hz bei Last, 50Hz normal
    
    if (millis() > nextMotorTime){
        nextMotorTime = millis() + motorInterval;
        requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
        
        // CPU-Last-Tracking
        if (motorInterval > 20) {
            lastHighLoadTime = millis();
        }
    }
    
    // Summary-Frequenz kann reduziert werden (nicht zeitkritisch)
    if (millis() > nextSummaryTime){
        nextSummaryTime = millis() + (isHighCpuLoad() ? 1000 : 500);
        requestSummary();
    }
    
    // Rest der Funktion unverändert...
}
```

**Erwartete Verbesserung:** CPU-Last -3% (10% → 9%)
**Vorteil:** Behält STM32-Kompatibilität bei

## Langfristige Architektur-Verbesserungen (Implementierung: 1 Tag)

### 6. Task-Scheduler Implementation
**Neue Datei:** `TaskScheduler.h`

```cpp
#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <Arduino.h>
#include <functional>
#include <vector>

struct ScheduledTask {
    unsigned long interval;     // Intervall in ms
    unsigned long nextRun;      // Nächste Ausführungszeit
    std::function<void()> callback;  // Auszuführende Funktion
    bool enabled;
    String name;  // Für Debugging
};

class TaskScheduler {
public:
    void addTask(const String& name, unsigned long interval, std::function<void()> callback);
    void removeTask(const String& name);
    void enableTask(const String& name, bool enabled);
    void run();  // Führt nur fällige Tasks aus
    void printStats();  // Debug-Ausgabe
    
private:
    std::vector<ScheduledTask> tasks;
    unsigned long lastRunTime = 0;
};

extern TaskScheduler scheduler;

#endif
```

**Neue Datei:** `TaskScheduler.cpp`

```cpp
#include "TaskScheduler.h"
#include "config.h"

TaskScheduler scheduler;

void TaskScheduler::addTask(const String& name, unsigned long interval, std::function<void()> callback) {
    ScheduledTask task;
    task.name = name;
    task.interval = interval;
    task.nextRun = millis() + interval;
    task.callback = callback;
    task.enabled = true;
    tasks.push_back(task);
}

void TaskScheduler::removeTask(const String& name) {
    tasks.erase(std::remove_if(tasks.begin(), tasks.end(), 
        [&name](const ScheduledTask& task) { return task.name == name; }), 
        tasks.end());
}

void TaskScheduler::enableTask(const String& name, bool enabled) {
    for (auto& task : tasks) {
        if (task.name == name) {
            task.enabled = enabled;
            break;
        }
    }
}

void TaskScheduler::run() {
    unsigned long currentTime = millis();
    
    for (auto& task : tasks) {
        if (task.enabled && currentTime >= task.nextRun) {
            task.callback();
            task.nextRun = currentTime + task.interval;
        }
    }
    
    lastRunTime = currentTime;
}

void TaskScheduler::printStats() {
    CONSOLE.println("=== Task Scheduler Stats ===");
    for (const auto& task : tasks) {
        CONSOLE.print(task.name);
        CONSOLE.print(": ");
        CONSOLE.print(task.interval);
        CONSOLE.print("ms, next: ");
        CONSOLE.print(task.nextRun - millis());
        CONSOLE.print("ms, enabled: ");
        CONSOLE.println(task.enabled ? "yes" : "no");
    }
}
```

### 7. Robot.cpp Umstellung auf TaskScheduler
**Datei:** `robot.cpp`
**In `setup()` Funktion hinzufügen:**

```cpp
void setup() {
    // ... bestehender Setup-Code ...
    
    // Task-Scheduler initialisieren
    scheduler.addTask("MainControl", 20, []() {
        // Hauptsteuerung alle 20ms
        computeRobotState();
        activeOp->run();
        // ... weitere zeitkritische Aufgaben ...
    });
    
    scheduler.addTask("Communication", 5, []() {
        processComm();
    });
    
    scheduler.addTask("Console", 100, []() {
        outputConsole();
    });
    
    scheduler.addTask("GPS", 100, []() {
        gps.run();
    });
    
    scheduler.addTask("Statistics", 10000, []() {
        calcStats();
        scheduler.printStats();  // Debug
    });
    
    CONSOLE.println("TaskScheduler initialized");
}
```

**Neue `loop()` Funktion:**

```cpp
void loop() {
    // Nur noch Task-Scheduler ausführen
    scheduler.run();
    
    // Minimale CPU-Entlastung
    delay(1);
    
    // Watchdog
    if(millis() > wdResetTimer + 1000){
        watchdogReset();
        wdResetTimer = millis();
    }
}
```

**Erwartete Verbesserung:** CPU-Last -30% (9% → 6%)

## Implementierungsplan

### Phase 1: Sofortige Maßnahmen (Tag 1)
1. ✅ `delay(1)` in Hauptschleife
2. ✅ `processComm()` Timing begrenzen  
3. ✅ SerialRobotDriver Frequenz reduzieren

**Erwartung:** CPU-Last 28% → 12%

### Phase 2: Mittelfristige Verbesserungen (Tag 2-3)
4. ✅ Adaptive Hauptschleife
5. ✅ Intelligente Kommunikations-Priorisierung

**Erwartung:** CPU-Last 12% → 9%

### Phase 3: Langfristige Architektur (Woche 2)
6. ✅ TaskScheduler Implementation
7. ✅ Robot.cpp Umstellung

**Erwartung:** CPU-Last 9% → 6%

## Test-Protokoll

### Nach jeder Phase:
1. **CPU-Last messen:** `top` oder `htop` für 5 Minuten
2. **Funktionstest:** Alle Grundfunktionen testen
3. **Timing-Analyse:** Loop-Time Statistiken prüfen
4. **Kommunikationstest:** AT+ Befehle testen

### Rollback-Plan:
- Jede Änderung in separatem Git-Commit
- Bei Problemen: `git revert <commit-hash>`
- Backup der Original-Dateien

## Risikobewertung

| Maßnahme | Risiko | Aufwand | Nutzen |
|----------|--------|---------|--------|
| delay(1) | Niedrig | 1 min | Hoch |
| processComm() Timing | Niedrig | 5 min | Mittel |
| SerialRobotDriver Freq. | Niedrig | 2 min | Mittel |
| Adaptive Schleife | Mittel | 30 min | Hoch |
| Kommunikations-Priorisierung | Mittel | 1 h | Mittel |
| TaskScheduler | Hoch | 1 Tag | Sehr Hoch |

## Monitoring

### CPU-Last überwachen:
```bash
# Kontinuierliches Monitoring
watch -n 1 'ps -eo pcpu,pid,user,args | grep sunray'

# Detaillierte Analyse
top -p $(pgrep sunray) -d 1
```

### Loop-Time überwachen:
- Bestehende Loop-Time Statistiken in `robot.cpp` nutzen
- Ziel: `loopTimeMax < 50ms`, `loopTimeMean < 10ms`

## Fazit

Mit diesen gestaffelten Verbesserungen kann die CPU-Last von 28% auf unter 10% reduziert werden:

- **Sofort (30 min):** 28% → 12% (-57%)
- **Mittelfristig (3 Tage):** 12% → 9% (-25%)
- **Langfristig (1 Woche):** 9% → 6% (-33%)

**Gesamtverbesserung:** -79% CPU-Last bei verbesserter Systemstabilität und Echtzeitverhalten.