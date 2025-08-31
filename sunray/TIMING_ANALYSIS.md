# Timing Analysis - Sunray Code Iteration

## Aktuelle Situation (Status Quo)

### Hauptschleife (robot.cpp)
- **Aktuelle Frequenz**: ~50Hz (20ms Intervall)
- **nextControlTime = millis() + 20** (Zeile 1015)
- **Loop-Time Monitoring**: Vorhanden mit Min/Max/Mean Tracking

### IMU Timing
- **Aktuelle FIFO Rate**: 5Hz (IMU_FIFO_RATE in config)
- **Hardware Standard**: 50Hz FIFO rate (inv_mpu.c)
- **Ziel**: >30Hz für bessere Bewegungserkennung

### Operations Timing
- **IDLE/DOCK**: Laufen aktuell mit Hauptschleife (50Hz)
- **MOW**: Läuft aktuell mit Hauptschleife (50Hz)
- **RC Model**: 10Hz (100ms Intervall)

## Gewünschte Änderungen

### 1. IDLE und DOCK Operations: 10Hz
- Reduzierung von 50Hz auf 10Hz (100ms Intervall)
- Energieeinsparung bei weniger kritischen Operationen

### 2. MOW Operation: 50Hz beibehalten
- Kritische Mähoperation benötigt hohe Frequenz
- Für präzise Hindernisvermeidung und Linienverfolgung

### 3. IMU Readout: >30Hz
- Erhöhung von 5Hz auf mindestens 30Hz
- Bessere Bewegungs- und Neigungserkennung
- Schnellere Reaktion auf Hindernisse

## Implementierungsplan

### Phase 1: IMU Timing erhöhen 
1. - IMU_FIFO_RATE von 5Hz auf 30Hz in config.h & config_alfred.h
2. - Anpassung der IMU-Verarbeitung in StateEstimator.cpp
3. - Test der IMU-Performance

### Phase 2: Operations-spezifische Timing 
1. - Einführung von operationsspezifischen Timing-Konstanten
2. - Anpassung der Hauptschleife für variable Frequenzen
3. - IDLE/DOCK auf 10Hz, MOW bleibt bei 50Hz

### Phase 3: Testing und Optimierung 
1. - Performance-Tests der neuen Timing-Konfiguration
2. - Überprüfung der Hindernisvermeidung
3. - Energieverbrauch-Analyse

## Implementierte Änderungen

### 1. IMU FIFO Rate Erhöhung
**Datei**: `config.h` & `config_alfred.h`
**Änderung**: `#define IMU_FIFO_RATE 30` (vorher: 5)
**Auswirkung**: 6x höhere IMU-Abtastrate für bessere Bewegungserkennung

### 2. Operations-spezifische Timing-Logik
**Datei**: `robot.cpp` (Zeile 1015-1020)
**Änderung**: Variable `controlInterval` basierend auf `stateOp`
- **IDLE/DOCK/CHARGE**: 100ms (10Hz)
- **MOW und andere**: 20ms (50Hz)

**Code-Snippet**:
```cpp
// Operation-specific timing: IDLE/DOCK=10Hz (100ms), MOW=50Hz (20ms), others=50Hz (20ms)
unsigned long controlInterval = 20; // default 50Hz
if ((stateOp == OP_IDLE) || (stateOp == OP_DOCK) || (stateOp == OP_CHARGE)) {
  controlInterval = 100; // 10Hz for IDLE/DOCK/CHARGE operations
}
nextControlTime = millis() + controlInterval;
```

## Technische Details

### Aktuelle Timing-Variablen
- `nextControlTime`: Hauptschleife Timing
- `nextImuTime`: IMU Update Timing
- `loopTime*`: Loop Performance Monitoring

### Zu modifizierende Dateien
- `config.h`: IMU_FIFO_RATE
- `config_alfred.h`: IMU_FIFO_RATE
- `robot.cpp`: Operations-spezifische Timing
- `StateEstimator.cpp`: IMU Processing
- `src/op/*.cpp`: Operations Timing

## Risiken und Überlegungen

### Vorteile
- Bessere IMU-Reaktionszeit
- Energieeinsparung bei IDLE/DOCK
- Optimierte Performance pro Operation

### Risiken
- Mögliche Timing-Konflikte
- Erhöhte CPU-Last durch höhere IMU-Frequenz
- Komplexere Timing-Logik

### Mitigation
- Schrittweise Implementierung
- Umfangreiche Tests
- Fallback auf alte Timing-Werte