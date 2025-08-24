# Erweiterte State Machine - Dokumentation

## Überblick

Die erweiterte State Machine bietet granulare Zustandsüberwachung und -steuerung für den Sunray Mähroboter. Anstatt nur 5 High-Level-Zustände zu verwenden, stehen nun 13 detaillierte Zustände zur Verfügung, die eine präzise Diagnose und Steuerung ermöglichen.

## Neue OperationType Enum

### Basis-Zustände (Rückwärtskompatibilität)
```cpp
OP_IDLE = 0,        // Ruhezustand
OP_MOW = 1,         // Normales Mähen
OP_CHARGE = 2,      // Ladevorgang
OP_ERROR = 3,       // Fehlerzustand
OP_DOCK = 4,        // Fahrt zur Ladestation
```

### Erweiterte granulare Zustände
```cpp
OP_ESCAPE_REVERSE = 5,    // Rückwärts-Hindernisvermeidung
OP_ESCAPE_FORWARD = 6,    // Vorwärts-Hindernisvermeidung
OP_GPS_WAIT_FIX = 7,      // Warten auf GPS-Fix
OP_GPS_WAIT_FLOAT = 8,    // Warten auf GPS-Float
OP_GPS_RECOVERY = 9,      // GPS-Recovery nach Neustart
OP_IMU_CALIBRATION = 10,  // IMU-Kalibrierung
OP_RELOCALIZATION = 11,   // Neupositionierung
OP_KIDNAP_WAIT = 12,      // Warten nach Kidnapping
```

## AT+ Befehle

### Bestehende Befehle (Rückwärtskompatibilität)
```
AT+C,0    // OP_IDLE - Stoppe Roboter
AT+C,1    // OP_MOW - Starte Mähen
AT+C,2    // OP_CHARGE - Lade Akku
AT+C,3    // OP_ERROR - Fehlerzustand
AT+C,4    // OP_DOCK - Fahre zur Ladestation
```

### Neue erweiterte Befehle
```
AT+C,5    // OP_ESCAPE_REVERSE - Rückwärts-Hindernisvermeidung
AT+C,6    // OP_ESCAPE_FORWARD - Vorwärts-Hindernisvermeidung
AT+C,7    // OP_GPS_WAIT_FIX - GPS-Fix warten
AT+C,8    // OP_GPS_WAIT_FLOAT - GPS-Float warten
AT+C,9    // OP_GPS_RECOVERY - GPS-Recovery
AT+C,10   // OP_IMU_CALIBRATION - IMU-Kalibrierung
AT+C,11   // OP_RELOCALIZATION - Neupositionierung
AT+C,12   // OP_KIDNAP_WAIT - Kidnapping-Wartemodus
```

### Zustandsabfrage
```
AT+S      // Gibt aktuellen Zustand zurück
```

**Antwortformat:**
```
stateOp,stateOpText,stateSensor,gpsSol,...
```

**Beispiele:**
```
1,mow,0,2,...                    // Normales Mähen
5,escape_reverse,1,2,...         // Hindernisvermeidung rückwärts
7,gps_wait_fix,0,0,...          // Warten auf GPS-Fix
10,imu_calibration,0,2,...       // IMU-Kalibrierung
```

## State Text Mapping

| OperationType | stateOpText | Beschreibung |
|---------------|-------------|-------------|
| OP_IDLE | "idle" | Ruhezustand |
| OP_MOW | "mow" | Normales Mähen |
| OP_CHARGE | "charge" | Ladevorgang |
| OP_ERROR | "error (...)" | Fehlerzustand mit Details |
| OP_DOCK | "dock" | Fahrt zur Ladestation |
| OP_ESCAPE_REVERSE | "escape_reverse" | Rückwärts-Hindernisvermeidung |
| OP_ESCAPE_FORWARD | "escape_forward" | Vorwärts-Hindernisvermeidung |
| OP_GPS_WAIT_FIX | "gps_wait_fix" | Warten auf GPS-Fix |
| OP_GPS_WAIT_FLOAT | "gps_wait_float" | Warten auf GPS-Float |
| OP_GPS_RECOVERY | "gps_recovery" | GPS-Recovery |
| OP_IMU_CALIBRATION | "imu_calibration" | IMU-Kalibrierung |
| OP_RELOCALIZATION | "relocalization" | Neupositionierung |
| OP_KIDNAP_WAIT | "kidnap_wait" | Kidnapping-Wartemodus |

## Typische State Transitions

### Normaler Mähvorgang
```
OP_IDLE → OP_MOW → OP_DOCK → OP_CHARGE → OP_IDLE
```

### Hindernisvermeidung während Mähen
```
OP_MOW → OP_ESCAPE_REVERSE → OP_MOW
OP_MOW → OP_ESCAPE_FORWARD → OP_MOW
```

### GPS-Problem-Recovery
```
OP_MOW → OP_GPS_WAIT_FIX → OP_GPS_RECOVERY → OP_MOW
OP_MOW → OP_GPS_WAIT_FLOAT → OP_GPS_WAIT_FIX → OP_MOW
```

### IMU-Kalibrierung
```
OP_IDLE → OP_IMU_CALIBRATION → OP_RELOCALIZATION → OP_MOW
```

### Kidnapping-Detection
```
OP_MOW → OP_KIDNAP_WAIT → OP_RELOCALIZATION → OP_MOW
```

## Kompatibilität

### Rückwärtskompatibilität
- Alle bestehenden AT+ Befehle (0-4) funktionieren unverändert
- Alte Alfred-Versionen können weiterhin die bekannten Zustände verwenden
- Neue Zustände (5-12) sind optional und erweitern die Funktionalität

### Alfred-Integration
- Raspberry Pi Code kann erweiterte Zustände abfragen
- Fallback auf bekannte Zustände für alte Alfred-Versionen
- MQTT-Topics enthalten die neuen stateOpText-Werte

## Vorteile der erweiterten State Machine

1. **Bessere Transparenz**: Präzise Anzeige des aktuellen Roboterzustands
2. **Granulare Kontrolle**: Direkte Steuerung spezifischer Operationen
3. **Verbesserte Diagnostik**: Detaillierte Zustandsinformationen für Debugging
4. **Erweiterte Funktionalität**: Neue Möglichkeiten für Alfred und externe Tools
5. **Rückwärtskompatibilität**: Bestehende Systeme funktionieren weiterhin

## Implementierungsdetails

### Geänderte Dateien
- `robot.h`: Erweiterte OperationType Enum
- `Op.cpp`: Erweiterte getGoalOperationType() und changeOperationTypeByOperator()
- `Storage.cpp`: Erweiterte updateStateOpText()

### Build-Kompatibilität
- Arduino Due: ✅ Erfolgreich kompiliert
- Speicherverbrauch: 180220 bytes (34%) - keine signifikante Erhöhung

## Verwendungsbeispiele

### Diagnose eines GPS-Problems
```bash
# Zustand abfragen
AT+S
# Antwort: 7,gps_wait_fix,0,0,...
# → Roboter wartet auf GPS-Fix

# GPS-Recovery manuell starten
AT+C,9
# → Startet GPS-Recovery-Prozess
```

### IMU-Kalibrierung starten
```bash
# IMU-Kalibrierung starten
AT+C,10
# → Startet IMU-Kalibrierung

# Status überwachen
AT+S
# Antwort: 10,imu_calibration,0,2,...
```

### Hindernisvermeidung überwachen
```bash
# Status während Mähen
AT+S
# Antwort: 5,escape_reverse,1,2,...
# → Roboter weicht Hindernis rückwärts aus
```

Diese erweiterte State Machine bietet eine solide Grundlage für erweiterte Roboterdiagnose und -steuerung, während die vollständige Rückwärtskompatibilität gewährleistet bleibt.