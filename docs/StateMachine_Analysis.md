# Sunray State Machine Analyse und Verbesserungsvorschlag

## Aktuelle State Machine (Status Quo)

### OperationType Enum (5 High-Level Zustände)
```cpp
enum OperationType {
    OP_IDLE,      // idle
    OP_MOW,       // mowing
    OP_CHARGE,    // charging
    OP_ERROR,     // serious error
    OP_DOCK,      // go to docking
};
```

### Aktuelle Operation-Klassen (14 detaillierte Zustände)
1. **IdleOp** - Roboter im Ruhezustand
2. **MowOp** - Mähvorgang
3. **ChargeOp** - Ladevorgang
4. **DockOp** - Fahrt zur Ladestation
5. **ErrorOp** - Fehlerzustand
6. **EscapeReverseOp** - Rückwärts-Hindernisvermeidung
7. **EscapeForwardOp** - Vorwärts-Hindernisvermeidung
8. **ImuCalibrationOp** - IMU-Kalibrierung
9. **RelocalizationOp** - Neupositionierung
10. **KidnapWaitOp** - Warten nach Kidnapping-Erkennung
11. **GpsWaitFixOp** - Warten auf GPS-Fix
12. **GpsWaitFloatOp** - Warten auf GPS-Float
13. **GpsRebootRecoveryOp** - GPS-Neustart-Recovery

### Problem der aktuellen Implementierung

**Diskrepanz zwischen OperationType und tatsächlichen Zuständen:**
- Nur 5 OperationType Enums, aber 14 verschiedene Operation-Klassen
- Viele wichtige Zwischenzustände (Escape, GPS-Wait, IMU-Calibration) werden nicht in der OperationType reflektiert
- `stateOp` zeigt nur den "Ziel-Zustand" an, nicht den aktuellen detaillierten Zustand
- Debugging und Monitoring erschwert, da wichtige Zwischenzustände unsichtbar sind

**Beispiel-Problem:**
```cpp
// Aktuell: Während EscapeReverseOp zeigt stateOp = OP_MOW
// Aber der Roboter ist eigentlich im Escape-Modus!
stateOp = activeOp->getGoalOperationType(); // Gibt OP_MOW zurück, obwohl EscapeReverseOp aktiv ist
```

## Vorschlag: Erweiterte State Machine

### Neue OperationType Enum (15 granulare Zustände)
```cpp
enum OperationType {
    // Basis-Zustände
    OP_IDLE,                    // Ruhezustand
    OP_ERROR,                   // Fehlerzustand
    
    // Mäh-Zustände
    OP_MOW,                     // Normales Mähen
    OP_MOW_UNDOCK,              // Ausfahren aus Ladestation zum Mähen
    
    // Dock-Zustände
    OP_DOCK,                    // Fahrt zur Ladestation
    OP_DOCK_APPROACH,           // Annäherung an Ladestation
    OP_CHARGE,                  // Ladevorgang
    
    // Escape/Recovery-Zustände
    OP_ESCAPE_REVERSE,          // Rückwärts-Hindernisvermeidung
    OP_ESCAPE_FORWARD,          // Vorwärts-Hindernisvermeidung
    OP_ESCAPE_ROTATION,         // Rotations-Hindernisvermeidung
    
    // GPS-Zustände
    OP_GPS_WAIT_FIX,           // Warten auf GPS-Fix
    OP_GPS_WAIT_FLOAT,         // Warten auf GPS-Float
    OP_GPS_RECOVERY,           // GPS-Recovery nach Neustart
    
    // Kalibrierungs-Zustände
    OP_IMU_CALIBRATION,        // IMU-Kalibrierung
    OP_RELOCALIZATION,         // Neupositionierung
    
    // Spezial-Zustände
    OP_KIDNAP_WAIT,            // Warten nach Kidnapping
};
```

### Vorteile der erweiterten State Machine

1. **Bessere Transparenz:**
   - Jeder tatsächliche Zustand ist in der OperationType sichtbar
   - `stateOp` reflektiert den aktuellen, nicht den Ziel-Zustand
   - Einfacheres Debugging und Monitoring

2. **Granulare Kontrolle:**
   - Unterscheidung zwischen verschiedenen Escape-Modi
   - Separate GPS-Zustände für verschiedene Recovery-Szenarien
   - Explizite Undock/Dock-Phasen

3. **Verbesserte Diagnostik:**
   - AT+ Befehle können spezifische Zustände abfragen
   - Logs zeigen präzise Zustandsinformationen
   - Fehleranalyse wird vereinfacht

4. **Erweiterte Funktionalität:**
   - Zustandsspezifische Timeouts und Parameter
   - Conditional State Transitions
   - State-spezifische Sensor-Behandlung

### Implementierungsplan

#### Phase 1: Enum-Erweiterung
```cpp
// robot.h - Erweiterte OperationType
enum OperationType {
    // ... neue Zustände wie oben
};
```

#### Phase 2: Mapping-Funktionen anpassen
```cpp
// Op.cpp - Erweiterte getGoalOperationType()
OperationType Op::getGoalOperationType(){
    Op *goalOp = getGoalOp();
    if (goalOp == &idleOp) return OP_IDLE;
    if (goalOp == &mowOp) return OP_MOW;
    if (goalOp == &chargeOp) return OP_CHARGE;
    if (goalOp == &dockOp) return OP_DOCK;
    if (goalOp == &errorOp) return OP_ERROR;
    if (goalOp == &escapeReverseOp) return OP_ESCAPE_REVERSE;
    if (goalOp == &escapeForwardOp) return OP_ESCAPE_FORWARD;
    if (goalOp == &gpsWaitFixOp) return OP_GPS_WAIT_FIX;
    if (goalOp == &gpsWaitFloatOp) return OP_GPS_WAIT_FLOAT;
    if (goalOp == &gpsRebootRecoveryOp) return OP_GPS_RECOVERY;
    if (goalOp == &imuCalibrationOp) return OP_IMU_CALIBRATION;
    if (goalOp == &relocalizationOp) return OP_RELOCALIZATION;
    if (goalOp == &kidnapWaitOp) return OP_KIDNAP_WAIT;
    // ...
    return OP_ERROR;
}
```

#### Phase 3: State Text Updates
```cpp
// Storage.cpp - Erweiterte updateStateOpText()
void updateStateOpText(){
    switch (stateOp){
        case OP_IDLE: stateOpText = "idle"; break;
        case OP_MOW: stateOpText = "mow"; break;
        case OP_MOW_UNDOCK: stateOpText = "mow_undock"; break;
        case OP_CHARGE: stateOpText = "charge"; break;
        case OP_DOCK: stateOpText = "dock"; break;
        case OP_DOCK_APPROACH: stateOpText = "dock_approach"; break;
        case OP_ESCAPE_REVERSE: stateOpText = "escape_reverse"; break;
        case OP_ESCAPE_FORWARD: stateOpText = "escape_forward"; break;
        case OP_GPS_WAIT_FIX: stateOpText = "gps_wait_fix"; break;
        case OP_GPS_WAIT_FLOAT: stateOpText = "gps_wait_float"; break;
        case OP_GPS_RECOVERY: stateOpText = "gps_recovery"; break;
        case OP_IMU_CALIBRATION: stateOpText = "imu_calibration"; break;
        case OP_RELOCALIZATION: stateOpText = "relocalization"; break;
        case OP_KIDNAP_WAIT: stateOpText = "kidnap_wait"; break;
        case OP_ERROR: 
            stateOpText = "error (";
            // ... error details
            break;
        default: stateOpText = "unknown"; break;
    }
}
```

#### Phase 4: AT+ Command Updates
```cpp
// comm.cpp - Erweiterte changeOperationTypeByOperator()
void Op::changeOperationTypeByOperator(OperationType op){
    switch (op){
        case OP_IDLE:
            activeOp->changeOp(idleOp, false);
            break;
        case OP_MOW:
            activeOp->changeOp(mowOp, false);
            break;
        case OP_DOCK:
            activeOp->changeOp(dockOp, false);
            break;
        case OP_CHARGE:
            activeOp->changeOp(chargeOp, false);
            break;
        case OP_ESCAPE_REVERSE:
            activeOp->changeOp(escapeReverseOp, false);
            break;
        // ... weitere Zustände
    }
}
```

### State Transition Beispiele

#### Normaler Mähvorgang:
```
OP_IDLE → OP_MOW_UNDOCK → OP_MOW → OP_DOCK → OP_DOCK_APPROACH → OP_CHARGE → OP_IDLE
```

#### Hindernisvermeidung während Mähen:
```
OP_MOW → OP_ESCAPE_REVERSE → OP_ESCAPE_ROTATION → OP_MOW
```

#### GPS-Problem-Recovery:
```
OP_MOW → OP_GPS_WAIT_FIX → OP_GPS_RECOVERY → OP_MOW
```

#### IMU-Kalibrierung:
```
OP_IDLE → OP_IMU_CALIBRATION → OP_RELOCALIZATION → OP_MOW
```

### Kompatibilität

**Rückwärtskompatibilität:**
- Bestehende AT+ Befehle funktionieren weiterhin
- Alte OperationType Werte (0-4) bleiben gültig
- Neue Zustände erhalten Werte ab 5

**Alfred-Kompatibilität:**
- Raspberry Pi Code kann erweiterte Zustände abfragen
- Fallback auf bekannte Zustände für alte Alfred-Versionen
- Neue AT+ Befehle für detaillierte Zustandsabfrage

### Fazit

Die erweiterte State Machine bietet:
- **Bessere Transparenz** des aktuellen Roboterzustands
- **Granulare Kontrolle** über verschiedene Betriebsmodi
- **Verbesserte Debugging-Möglichkeiten**
- **Erweiterte Monitoring-Funktionen**
- **Rückwärtskompatibilität** mit bestehender Software

Die Implementierung kann schrittweise erfolgen, ohne die bestehende Funktionalität zu beeinträchtigen.