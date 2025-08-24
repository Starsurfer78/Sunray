# TODO: GPS/IMU-Zustandsschätzung Verbesserungen

## Übersicht
Diese TODO-Liste beschreibt konkrete Verbesserungen für die GPS/IMU-Zustandsschätzung im Sunray-Projekt, basierend auf der Analyse der aktuellen Implementierung.

## 1. Erweiterte Orientierungswinkel-Integration

### Status: Geplant
### Priorität: Hoch
### Dateien: `StateEstimator.h`, `StateEstimator.cpp`

#### Aktuelle Situation:
- Roll (`stateRoll`) und Pitch (`statePitch`) werden bereits aus IMU-Daten gelesen
- `computeRobotState()` nutzt hauptsächlich Yaw-Winkel für Navigation
- IMU-Daten werden über `readIMU()` eingelesen

#### Aufgaben:
- [ ] 3D-Orientierungsmatrix aus Roll, Pitch und Yaw implementieren
- [ ] Neigungskompensierte Navigation für Hanglagen entwickeln
- [ ] Erweiterte Komplementärfilter für alle drei Orientierungswinkel
- [ ] Integration in bestehende `computeRobotState()` Funktion

#### Implementierungsdetails:
```cpp
// Neue Funktionen in StateEstimator.cpp:
void compute3DOrientationMatrix(float roll, float pitch, float yaw, float matrix[3][3]);
void compensateForSlope(float &posX, float &posY, float roll, float pitch);
void enhancedComplementaryFilter(float &roll, float &pitch, float &yaw);
```

## 2. Antennen-Offset-Implementierung

### Status: Geplant
### Priorität: Mittel
### Dateien: `config_alfred.h`, `StateEstimator.cpp`, `ublox.cpp`


#### Aktuelle GPS-Verarbeitung:
- **UBLOX**: Verarbeitet `UBX-NAV-RELPOSNED` und `UBX-NAV-HPPOSLLH` Nachrichten
- Koordinaten werden direkt ohne Offset-Korrektur verwendet

#### Aufgaben:
- [ ] Konfigurationsparameter in `config_alfred.h` hinzufügen

- [x] Antennen-Offset-Korrektur-Funktion implementieren ✅
- [x] Integration in GPS-Datenverarbeitung (UBLOX) ✅
- [x] 3D-Rotation der Offsets basierend auf aktueller Orientierung ✅
- [x] AT+ Befehle für Konfiguration implementiert ✅

#### ✅ IMPLEMENTIERT - Konfiguration:
```cpp
// In config_example.h hinzugefügt:

#define GPS_ANTENNA_OFFSET_X  0.15  // Meter, vorwärts vom Roboterzentrum
#define GPS_ANTENNA_OFFSET_Y  0.05  // Meter, seitlich vom Roboterzentrum  
#define GPS_ANTENNA_OFFSET_Z  0.20  // Meter, vertikal vom Roboterzentrum
#define ENABLE_ANTENNA_OFFSET_CORRECTION  // Aktiviert die Antennen-Offset-Korrektur
```

#### ✅ IMPLEMENTIERT - Klasse:
```cpp
// Neue Klasse: src/driver/GpsAntennaOffset.h/.cpp
class GpsAntennaOffset {
public:
  void setOffset(float x, float y, float z);
  void getOffset(float &x, float &y, float &z);
  void setEnabled(bool enabled);
  bool isEnabled();
  void correctPosition(float &posN, float &posE, float &posD, float heading);
};
```

#### ✅ IMPLEMENTIERT - Integration:
```cpp
// In StateEstimator.cpp integriert:
#ifdef ENABLE_ANTENNA_OFFSET_CORRECTION
  gpsAntennaOffset.correctPosition(posN, posE, posD, stateDelta);
#endif
```

#### ✅ IMPLEMENTIERT - AT+ Befehle:
```
AT+GPS_OFFSET,X,Y,Z,enabled    // Setzt GPS-Antennen-Offset
AT+GPS_OFFSET                  // Zeigt aktuelle Offset-Werte
AT+GPS_OFFSET_STATUS           // Zeigt Status und Werte
```
  float offsetZ_rotated = GPS_ANTENNA_OFFSET_Z + 
                         GPS_ANTENNA_OFFSET_X * sin(pitch);
  
  posX -= offsetX_rotated;
  posY -= offsetY_rotated; 
  posZ -= offsetZ_rotated;
}
```

## 3. Virtuelle Höhenkompensation

### Status: Geplant
### Priorität: Mittel
### Dateien: `StateEstimator.cpp`, `config_alfred.h`


#### Aktuelle Höhenverarbeitung:
- GPS-Höhe wird in `height` Variable gespeichert (HAE - Height Above Ellipsoid)
- Keine Kompensation für Roboterneigung

#### Aufgaben:
- [ ] Virtuelle Bodenhöhen-Berechnung implementieren
- [ ] Neigungseffekte auf gemessene Höhe kompensieren
- [ ] Integration in `computeRobotState()` Funktion
- [ ] Konfigurierbare Parameter für Höhenkompensation

#### Implementierung:
```cpp
// Neue Funktion in StateEstimator.cpp:
float calculateVirtualGroundHeight(float gpsHeight, float roll, float pitch) {
  // Kompensation der Neigungseffekte auf die gemessene Höhe
  float heightCorrection = GPS_ANTENNA_OFFSET_Z * (1.0 - cos(pitch) * cos(roll));
  return gpsHeight - heightCorrection;
}

// In computeRobotState() integrieren:
float virtualHeight = calculateVirtualGroundHeight(height, stateRoll, statePitch);
```

## 4. Erweiterte Zustandsschätzung

### Status: Geplant
### Priorität: Hoch
### Dateien: `StateEstimator.h`, `StateEstimator.cpp`

#### Integration in bestehende Architektur:
- [ ] Modulare Erweiterung ohne Änderung der AT+-Schnittstelle
- [ ] Konfigurierbare Feature-Flags für schrittweise Aktivierung
- [ ] Erhaltung der Kompatibilität mit Alfred (Raspberry Pi)
- [ ] Erweiterte Zustandsvariablen für 3D-Orientierung

#### Neue Zustandsvariablen:
```cpp
// In StateEstimator.h hinzufügen:
extern float stateOrientationMatrix[3][3];  // 3D-Orientierungsmatrix
extern float stateVirtualHeight;            // Virtuelle Bodenhöhe
extern float stateAntennaOffsetX;           // Aktuelle Antennen-Offset X
extern float stateAntennaOffsetY;           // Aktuelle Antennen-Offset Y
extern float stateAntennaOffsetZ;           // Aktuelle Antennen-Offset Z
```

## 5. Implementierungsreihenfolge

### Phase 1: Antennen-Offset-Korrektur (Woche 1-2)

#### Schritt 1.1: Konfigurationsparameter hinzufügen (Tag 1)
**Datei:** `sunray/config_.h`
**Commit:** `config: add GPS antenna offset parameters (no behavior change)`

```cpp
// GPS Antenna Offset Configuration
// Offset from robot center to GPS antenna position
#define GPS_ANTENNA_OFFSET_X_CM     15    // cm forward from robot center
#define GPS_ANTENNA_OFFSET_Y_CM     5     // cm right from robot center  
#define GPS_ANTENNA_OFFSET_Z_CM     20    // cm up from robot center
#define ENABLE_ANTENNA_OFFSET       false // Feature flag - default OFF
```

**Test:** Arduino Due Build erfolgreich, keine Verhaltensänderung

#### Schritt 1.2: Basis-Offset-Funktion implementieren (Tag 2)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `gps: add basic antenna offset calculation function (no behavior change)`

```cpp
// Add to StateEstimator.cpp after existing functions
void applyAntennaOffsetBasic(float &posX, float &posY, float yaw) {
  #if ENABLE_ANTENNA_OFFSET
    // Convert cm to meters
    float offsetX_m = GPS_ANTENNA_OFFSET_X_CM / 100.0;
    float offsetY_m = GPS_ANTENNA_OFFSET_Y_CM / 100.0;
    
    // Simple 2D rotation (yaw only)
    float offsetX_rotated = offsetX_m * cos(yaw) - offsetY_m * sin(yaw);
    float offsetY_rotated = offsetX_m * sin(yaw) + offsetY_m * cos(yaw);
    
    posX -= offsetX_rotated;
    posY -= offsetY_rotated;
  #endif
}
```

**Test:** Funktion kompiliert, wird noch nicht aufgerufen

#### Schritt 1.3: Integration in GPS-Verarbeitung (Tag 3)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `gps: integrate antenna offset in computeRobotState() (feature flag OFF)`

```cpp
// In computeRobotState() function, nach GPS-Position update:
void StateEstimator::computeRobotState(){
  // ... existing code ...
  
  if (gps.solution == SOL_FIXED || gps.solution == SOL_FLOAT) {
    // ... existing GPS processing ...
    
    // Apply antenna offset correction (only if enabled)
    applyAntennaOffsetBasic(posN, posE, stateDelta);
    
    // ... rest of existing code ...
  }
}
```

**Test:** Feature flag OFF - keine Verhaltensänderung, Build erfolgreich

#### Schritt 1.4: Tests mit statischen Offsets (Tag 4-5)
**Datei:** `sunray/config.h` (temporär für Tests)
**Commit:** `test: enable antenna offset for validation (temporary)`

```cpp
#define ENABLE_ANTENNA_OFFSET       true  // Enable for testing
#define GPS_ANTENNA_OFFSET_X_CM     10    // Test with 10cm forward
#define GPS_ANTENNA_OFFSET_Y_CM     0     // No side offset for first test
```

**Testprotokoll:**
- Statischer Test: Robot an bekannter Position, GPS-Koordinaten vor/nach Offset vergleichen
- Bewegungstest: 2m geradeaus fahren, Offset-Korrektur validieren
- Drehtest: 360° Drehung, Offset-Rotation prüfen

**Nach Tests:** Feature flag wieder auf `false` setzen

### Phase 2: Virtuelle Höhenkompensation (Woche 3)

#### Schritt 2.1: Höhenkompensations-Parameter hinzufügen (Tag 6)
**Datei:** `sunray/config.h`
**Commit:** `config: add virtual height compensation parameters (no behavior change)`

```cpp
// Virtual Height Compensation Configuration
#define ENABLE_VIRTUAL_HEIGHT_COMP  false // Feature flag - default OFF
#define ROBOT_HEIGHT_CM             25    // Robot height from ground to GPS antenna
#define MAX_SLOPE_COMPENSATION_DEG  20    // Maximum slope for compensation
#define HEIGHT_FILTER_ALPHA         0.1   // Low-pass filter for height smoothing
```

**Test:** Arduino Due Build erfolgreich, keine Verhaltensänderung

#### Schritt 2.2: Basis-Höhenkompensation implementieren (Tag 7)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `gps: add virtual ground height calculation (no behavior change)`

```cpp
// Add to StateEstimator.cpp
float calculateVirtualGroundHeight(float gpsHeight, float roll, float pitch) {
  #if ENABLE_VIRTUAL_HEIGHT_COMP
    // Convert robot height to meters
    float robotHeight_m = ROBOT_HEIGHT_CM / 100.0;
    
    // Calculate slope compensation
    float slopeEffect = robotHeight_m * (1.0 - cos(pitch) * cos(roll));
    
    // Limit compensation to reasonable slopes
    float maxCompensation = robotHeight_m * sin(MAX_SLOPE_COMPENSATION_DEG * PI / 180.0);
    slopeEffect = constrain(slopeEffect, -maxCompensation, maxCompensation);
    
    return gpsHeight - slopeEffect;
  #else
    return gpsHeight; // No compensation when disabled
  #endif
}

// Add height filtering
float filteredVirtualHeight = 0.0;
void updateVirtualHeight(float newHeight) {
  #if ENABLE_VIRTUAL_HEIGHT_COMP
    // Simple low-pass filter
    filteredVirtualHeight = (1.0 - HEIGHT_FILTER_ALPHA) * filteredVirtualHeight + 
                           HEIGHT_FILTER_ALPHA * newHeight;
  #else
    filteredVirtualHeight = newHeight;
  #endif
}
```

**Test:** Funktionen kompilieren, werden noch nicht aufgerufen

#### Schritt 2.3: Integration in Zustandsschätzung (Tag 8)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `gps: integrate virtual height in computeRobotState() (feature flag OFF)`

```cpp
// In computeRobotState() function, nach GPS-Höhen update:
void StateEstimator::computeRobotState(){
  // ... existing code ...
  
  if (gps.solution == SOL_FIXED || gps.solution == SOL_FLOAT) {
    // ... existing GPS processing ...
    
    // Calculate virtual ground height
    float virtualHeight = calculateVirtualGroundHeight(gps.height, stateRoll, statePitch);
    updateVirtualHeight(virtualHeight);
    
    // Use filtered virtual height for navigation (when enabled)
    #if ENABLE_VIRTUAL_HEIGHT_COMP
      // Store virtual height for use in navigation algorithms
      stateGroundHeight = filteredVirtualHeight;
    #else
      stateGroundHeight = gps.height; // Use raw GPS height
    #endif
    
    // ... rest of existing code ...
  }
}
```

**Test:** Feature flag OFF - keine Verhaltensänderung, Build erfolgreich

#### Schritt 2.4: Tests auf geneigten Flächen (Tag 9-10)
**Datei:** `sunray/config.h` (temporär für Tests)
**Commit:** `test: enable virtual height compensation for validation (temporary)`

```cpp
#define ENABLE_VIRTUAL_HEIGHT_COMP  true  // Enable for testing
#define ROBOT_HEIGHT_CM             20    // Test with actual robot height
#define HEIGHT_FILTER_ALPHA         0.2   // Faster response for testing
```

**Testprotokoll:**
- **Ebener Test:** Robot auf ebener Fläche, virtuelle vs. GPS-Höhe vergleichen
- **Neigungstest 5°:** Robot auf 5° Neigung, Höhenkompensation messen
- **Neigungstest 10°:** Robot auf 10° Neigung, Stabilität prüfen
- **Bewegungstest:** Fahrt über Neigung, kontinuierliche Höhenanpassung

**Erwartete Ergebnisse:**
- Virtuelle Höhe sollte stabiler sein als GPS-Rohhöhe
- Kompensation sollte Neigungseffekte reduzieren
- Filter sollte Höhensprünge glätten

**Nach Tests:** Feature flag wieder auf `false` setzen

### Phase 3: Erweiterte 3D-Orientierung (Woche 4-5)

#### Schritt 3.1: 3D-Orientierungsmatrix-Parameter (Tag 11)
**Datei:** `sunray/config.h`
**Commit:** `config: add 3D orientation matrix parameters (no behavior change)`

```cpp
// 3D Orientation Configuration
#define ENABLE_3D_ORIENTATION       false // Feature flag - default OFF
#define USE_ROTATION_MATRIX         false // Use full rotation matrix vs. simple angles
#define ORIENTATION_FILTER_GAIN     0.05  // Complementary filter gain for orientation
#define MAX_TILT_ANGLE_DEG          30    // Maximum allowed tilt angle
#define GYRO_DRIFT_COMPENSATION     true  // Enable gyroscope drift compensation
```

**Test:** Arduino Due Build erfolgreich, keine Verhaltensänderung

#### Schritt 3.2: Basis-Rotationsmatrix implementieren (Tag 12-13)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `imu: add 3D rotation matrix calculation (no behavior change)`

```cpp
// Add to StateEstimator.cpp
struct RotationMatrix {
  float m[3][3];
};

// Calculate rotation matrix from roll, pitch, yaw
RotationMatrix calculateRotationMatrix(float roll, float pitch, float yaw) {
  #if ENABLE_3D_ORIENTATION
    RotationMatrix R;
    float cr = cos(roll), sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);
    float cy = cos(yaw), sy = sin(yaw);
    
    // ZYX Euler angle rotation matrix
    R.m[0][0] = cy * cp;
    R.m[0][1] = cy * sp * sr - sy * cr;
    R.m[0][2] = cy * sp * cr + sy * sr;
    
    R.m[1][0] = sy * cp;
    R.m[1][1] = sy * sp * sr + cy * cr;
    R.m[1][2] = sy * sp * cr - cy * sr;
    
    R.m[2][0] = -sp;
    R.m[2][1] = cp * sr;
    R.m[2][2] = cp * cr;
    
    return R;
  #else
    // Return identity matrix when disabled
    RotationMatrix R = {{{1,0,0},{0,1,0},{0,0,1}}};
    return R;
  #endif
}

// Transform vector using rotation matrix
void transformVector(const RotationMatrix& R, float in[3], float out[3]) {
  #if ENABLE_3D_ORIENTATION
    for(int i = 0; i < 3; i++) {
      out[i] = 0;
      for(int j = 0; j < 3; j++) {
        out[i] += R.m[i][j] * in[j];
      }
    }
  #else
    // No transformation when disabled
    out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
  #endif
}
```

**Test:** Funktionen kompilieren, werden noch nicht verwendet

#### Schritt 3.3: Erweiterte Komplementärfilter (Tag 14-15)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `imu: add enhanced complementary filter for 3D orientation (feature flag OFF)`

```cpp
// Enhanced complementary filter for 3D orientation
void updateOrientation3D(float dt) {
  #if ENABLE_3D_ORIENTATION
    // Get accelerometer angles (gravity vector)
    float accelRoll = atan2(imu.accelY, sqrt(imu.accelX*imu.accelX + imu.accelZ*imu.accelZ));
    float accelPitch = atan2(-imu.accelX, sqrt(imu.accelY*imu.accelY + imu.accelZ*imu.accelZ));
    
    // Integrate gyroscope (with drift compensation)
    static float gyroDriftRoll = 0, gyroDriftPitch = 0, gyroDriftYaw = 0;
    
    #if GYRO_DRIFT_COMPENSATION
      // Simple drift estimation (when robot is stationary)
      if(abs(imu.gyroX) < 0.01 && abs(imu.gyroY) < 0.01 && abs(imu.gyroZ) < 0.01) {
        gyroDriftRoll = gyroDriftRoll * 0.999 + imu.gyroX * 0.001;
        gyroDriftPitch = gyroDriftPitch * 0.999 + imu.gyroY * 0.001;
        gyroDriftYaw = gyroDriftYaw * 0.999 + imu.gyroZ * 0.001;
      }
    #endif
    
    // Gyro integration with drift compensation
    float gyroRoll = (imu.gyroX - gyroDriftRoll) * dt;
    float gyroPitch = (imu.gyroY - gyroDriftPitch) * dt;
    float gyroYaw = (imu.gyroZ - gyroDriftYaw) * dt;
    
    // Complementary filter
    float alpha = ORIENTATION_FILTER_GAIN;
    stateRoll = (1.0 - alpha) * (stateRoll + gyroRoll) + alpha * accelRoll;
    statePitch = (1.0 - alpha) * (statePitch + gyroPitch) + alpha * accelPitch;
    stateYaw = stateYaw + gyroYaw; // Yaw from gyro only (no magnetometer yet)
    
    // Limit tilt angles for safety
    float maxTilt = MAX_TILT_ANGLE_DEG * PI / 180.0;
    stateRoll = constrain(stateRoll, -maxTilt, maxTilt);
    statePitch = constrain(statePitch, -maxTilt, maxTilt);
    
  #else
    // Use existing simple orientation update
    // ... existing code ...
  #endif
}
```

**Test:** Feature flag OFF - keine Verhaltensänderung, Build erfolgreich

#### Schritt 3.4: Integration in computeRobotState() (Tag 16-17)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `imu: integrate 3D orientation in state estimation (feature flag OFF)`

```cpp
// In computeRobotState() function:
void StateEstimator::computeRobotState(){
  // ... existing code ...
  
  // Update orientation with enhanced filter
  float dt = (millis() - lastIMUTime) / 1000.0;
  if(dt > 0.001 && dt < 0.1) { // Reasonable dt range
    updateOrientation3D(dt);
    
    #if ENABLE_3D_ORIENTATION && USE_ROTATION_MATRIX
      // Calculate current rotation matrix
      RotationMatrix currentRotation = calculateRotationMatrix(stateRoll, statePitch, stateYaw);
      
      // Apply rotation to antenna offset (if enabled)
      #if ENABLE_ANTENNA_OFFSET
        float antennaOffset[3] = {GPS_ANTENNA_OFFSET_X_CM/100.0, GPS_ANTENNA_OFFSET_Y_CM/100.0, GPS_ANTENNA_OFFSET_Z_CM/100.0};
        float rotatedOffset[3];
        transformVector(currentRotation, antennaOffset, rotatedOffset);
        
        // Apply rotated offset to GPS position
        stateX += rotatedOffset[0];
        stateY += rotatedOffset[1];
        // Z offset handled by virtual height compensation
      #endif
    #endif
  }
  lastIMUTime = millis();
  
  // ... rest of existing code ...
}
```

**Test:** Feature flags OFF - keine Verhaltensänderung, Build erfolgreich

#### Schritt 3.5: Validierungstests (Tag 18-20)
**Datei:** `sunray/config.h` (temporär für Tests)
**Commit:** `test: enable 3D orientation for validation (temporary)`

```cpp
#define ENABLE_3D_ORIENTATION       true  // Enable for testing
#define USE_ROTATION_MATRIX         true  // Test full matrix
#define ORIENTATION_FILTER_GAIN     0.02  // Conservative gain for testing
#define MAX_TILT_ANGLE_DEG          15    // Conservative limit for testing
```

**Testprotokoll:**
- **Statischer Test:** Robot in verschiedenen Neigungen, Orientierungsgenauigkeit messen
- **Bewegungstest:** Fahrt mit Richtungsänderungen, Orientierungsstabilität prüfen
- **Neigungstest:** Robot auf geneigter Fläche, 3D-Orientierung vs. 2D vergleichen
- **Kombinationstest:** Alle Features zusammen (Antenna Offset + Virtual Height + 3D Orientation)

**Erwartete Ergebnisse:**
- Stabilere Orientierungsschätzung bei Bewegung
- Bessere Positionsgenauigkeit durch 3D-Rotation der Antenna Offsets
- Reduzierte Drift durch erweiterten Komplementärfilter

**Nach Tests:** Feature flags wieder auf `false` setzen Umfassende Tests

### Phase 4: Validierung und Optimierung (Woche 6)

#### Schritt 4.1: Umfassende Systemtests (Tag 21-22)
**Datei:** `sunray/config.h` (finale Konfiguration)
**Commit:** `config: enable all GPS/IMU improvements for production testing`

```cpp
// Production Configuration - All Features Enabled
#define ENABLE_ANTENNA_OFFSET       true  // Enable antenna offset correction
#define ENABLE_VIRTUAL_HEIGHT_COMP  true  // Enable virtual height compensation
#define ENABLE_3D_ORIENTATION       true  // Enable 3D orientation
#define USE_ROTATION_MATRIX         true  // Use full rotation matrix

// Optimized Parameters (from testing)
#define GPS_ANTENNA_OFFSET_X_CM     15    // Final measured offset
#define GPS_ANTENNA_OFFSET_Y_CM     0     // Final measured offset
#define GPS_ANTENNA_OFFSET_Z_CM     25    // Final measured offset
#define ROBOT_HEIGHT_CM             20    // Final measured height
#define ORIENTATION_FILTER_GAIN     0.03  // Optimized filter gain
#define HEIGHT_FILTER_ALPHA         0.15  // Optimized height filter
```

**Systemtest-Protokoll:**
- **Präzisionstest:** 10x10m Quadrat abfahren, Abweichung messen
- **Hangtest:** 15° Neigung, Positionsgenauigkeit und Stabilität
- **Langzeittest:** 2h kontinuierlicher Betrieb, Drift-Analyse
- **Störungstest:** RTK-Ausfall simulation, Float-Mode Verhalten

#### Schritt 4.2: Performance-Optimierung (Tag 23)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `perf: optimize GPS/IMU calculations for better performance`

```cpp
// Optimized rotation matrix calculation (cache trigonometric values)
static float lastRoll = 0, lastPitch = 0, lastYaw = 0;
static RotationMatrix cachedRotation;
static bool rotationCacheValid = false;

RotationMatrix getOptimizedRotationMatrix(float roll, float pitch, float yaw) {
  #if ENABLE_3D_ORIENTATION
    // Check if we can use cached rotation matrix
    float rollDiff = abs(roll - lastRoll);
    float pitchDiff = abs(pitch - lastPitch);
    float yawDiff = abs(yaw - lastYaw);
    
    // Recalculate only if significant change (>1 degree)
    if(!rotationCacheValid || rollDiff > 0.017 || pitchDiff > 0.017 || yawDiff > 0.017) {
      cachedRotation = calculateRotationMatrix(roll, pitch, yaw);
      lastRoll = roll; lastPitch = pitch; lastYaw = yaw;
      rotationCacheValid = true;
    }
    
    return cachedRotation;
  #else
    RotationMatrix R = {{{1,0,0},{0,1,0},{0,0,1}}};
    return R;
  #endif
}

// Optimized state update with reduced calculations
void optimizedStateUpdate() {
  #if ENABLE_ANTENNA_OFFSET || ENABLE_VIRTUAL_HEIGHT_COMP || ENABLE_3D_ORIENTATION
    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();
    
    // Update at 50Hz maximum to reduce CPU load
    if(currentTime - lastUpdateTime < 20) return;
    lastUpdateTime = currentTime;
    
    // Batch all calculations
    if(gps.solution == SOL_FIXED || gps.solution == SOL_FLOAT) {
      // Apply all corrections in one pass
      float correctedX = gps.relPosN;
      float correctedY = gps.relPosE;
      float correctedHeight = gps.height;
      
      #if ENABLE_3D_ORIENTATION && ENABLE_ANTENNA_OFFSET
        RotationMatrix R = getOptimizedRotationMatrix(stateRoll, statePitch, stateYaw);
        float offset[3] = {GPS_ANTENNA_OFFSET_X_CM/100.0, GPS_ANTENNA_OFFSET_Y_CM/100.0, GPS_ANTENNA_OFFSET_Z_CM/100.0};
        float rotatedOffset[3];
        transformVector(R, offset, rotatedOffset);
        correctedX += rotatedOffset[0];
        correctedY += rotatedOffset[1];
      #endif
      
      #if ENABLE_VIRTUAL_HEIGHT_COMP
        correctedHeight = calculateVirtualGroundHeight(gps.height, stateRoll, statePitch);
        updateVirtualHeight(correctedHeight);
      #endif
      
      // Apply all corrections at once
      stateX = correctedX;
      stateY = correctedY;
      stateGroundHeight = correctedHeight;
    }
  #endif
}
```

**Performance-Ziele:**
- CPU-Last <5% zusätzlich
- Update-Rate 50Hz beibehalten
- Speicherverbrauch <1KB zusätzlich

#### Schritt 4.3: Diagnostik und Monitoring (Tag 24)
**Datei:** `sunray/src/driver/StateEstimator.cpp`
**Commit:** `diag: add GPS/IMU diagnostics and monitoring`

```cpp
// Diagnostic data structure
struct GPSIMUDiagnostics {
  float positionAccuracy;     // Current position accuracy estimate
  float orientationStability; // Orientation change rate
  float heightVariation;      // Height variation over time
  unsigned long lastGPSFix;   // Time of last GPS fix
  bool antennaOffsetActive;   // Antenna offset correction active
  bool heightCompActive;      // Height compensation active
  bool orientation3DActive;   // 3D orientation active
};

GPSIMUDiagnostics diagnostics;

void updateDiagnostics() {
  static float lastX = 0, lastY = 0, lastHeight = 0;
  static float lastRoll = 0, lastPitch = 0, lastYaw = 0;
  static unsigned long lastDiagTime = 0;
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastDiagTime) / 1000.0;
  
  if(dt > 1.0) { // Update diagnostics every second
    // Calculate position accuracy (based on GPS and movement)
    float posChange = sqrt(pow(stateX - lastX, 2) + pow(stateY - lastY, 2));
    diagnostics.positionAccuracy = gps.hAccuracy; // From GPS
    
    // Calculate orientation stability
    float orientChange = abs(stateRoll - lastRoll) + abs(statePitch - lastPitch) + abs(stateYaw - lastYaw);
    diagnostics.orientationStability = orientChange / dt;
    
    // Calculate height variation
    diagnostics.heightVariation = abs(stateGroundHeight - lastHeight);
    
    // Update status flags
    diagnostics.antennaOffsetActive = ENABLE_ANTENNA_OFFSET;
    diagnostics.heightCompActive = ENABLE_VIRTUAL_HEIGHT_COMP;
    diagnostics.orientation3DActive = ENABLE_3D_ORIENTATION;
    
    if(gps.solution == SOL_FIXED || gps.solution == SOL_FLOAT) {
      diagnostics.lastGPSFix = currentTime;
    }
    
    // Store for next iteration
    lastX = stateX; lastY = stateY; lastHeight = stateGroundHeight;
    lastRoll = stateRoll; lastPitch = statePitch; lastYaw = stateYaw;
    lastDiagTime = currentTime;
  }
}

// AT+ command for diagnostics
void printGPSIMUDiagnostics() {
  CONSOLE.print("GPS_IMU_DIAG,");
  CONSOLE.print(diagnostics.positionAccuracy, 3);
  CONSOLE.print(",");
  CONSOLE.print(diagnostics.orientationStability, 3);
  CONSOLE.print(",");
  CONSOLE.print(diagnostics.heightVariation, 3);
  CONSOLE.print(",");
  CONSOLE.print(millis() - diagnostics.lastGPSFix);
  CONSOLE.print(",");
  CONSOLE.print(diagnostics.antennaOffsetActive ? 1 : 0);
  CONSOLE.print(",");
  CONSOLE.print(diagnostics.heightCompActive ? 1 : 0);
  CONSOLE.print(",");
  CONSOLE.println(diagnostics.orientation3DActive ? 1 : 0);
}
```

#### Schritt 4.4: Dokumentation und Freigabe (Tag 25)
**Datei:** `sunray/docs/GPS_IMU_Improvements.md` (neu)
**Commit:** `docs: add comprehensive GPS/IMU improvements documentation`

```markdown
# GPS/IMU Verbesserungen - Benutzerhandbuch

## Übersicht
Diese Implementierung erweitert die Sunray GPS/IMU-Zustandsschätzung um:
- Antennen-Offset-Korrektur für präzise Positionierung
- Virtuelle Höhenkompensation für Hangnavigation
- 3D-Orientierungsschätzung für verbesserte Stabilität

## Konfiguration

### Antennen-Offsets messen
1. Robot auf ebener Fläche positionieren
2. GPS-Antenne und Roboter-Mittelpunkt vermessen
3. Offsets in config.h eintragen:
   ```cpp
   #define GPS_ANTENNA_OFFSET_X_CM  15  // Vorwärts/Rückwärts
   #define GPS_ANTENNA_OFFSET_Y_CM   0  // Links/Rechts
   #define GPS_ANTENNA_OFFSET_Z_CM  25  // Hoch/Runter
   ```

### Roboter-Höhe kalibrieren
1. Robot auf ebener Fläche
2. Abstand GPS-Antenne zu Boden messen
3. In config.h eintragen:
   ```cpp
   #define ROBOT_HEIGHT_CM  20
   ```

### Filter-Parameter anpassen
- `ORIENTATION_FILTER_GAIN`: 0.01-0.05 (niedriger = stabiler)
- `HEIGHT_FILTER_ALPHA`: 0.1-0.3 (niedriger = glatter)

## Diagnose
AT+ Befehl `AT+GPSDIAG` zeigt:
- Positionsgenauigkeit
- Orientierungsstabilität
- Höhenvariation
- Feature-Status

## Fehlerbehebung
- Hohe Orientierungsstabilität: Filter-Gain reduzieren
- Sprunghafte Höhe: Height-Filter-Alpha reduzieren
- Positionsabweichung: Antenna-Offsets überprüfen
```

**Finale Tests:**
- Alle Features aktiviert
- 24h Dauerlauf ohne Probleme
- Positionsgenauigkeit <3cm (RTK Fixed)
- Orientierungsstabilität <0.1°/s
- CPU-Last <5% zusätzlich

**Produktionsfreigabe-Kriterien:**
- [ ] Alle Tests bestanden
- [ ] Dokumentation vollständig
- [ ] Performance-Ziele erreicht
- [ ] Rückwärtskompatibilität gewährleistet
- [ ] Feature-Flags standardmäßig aktiviert

## 6. Test-Strategie

### Einheitentests (Unit Tests)

#### Test 1: Antennen-Offset-Berechnungen
**Datei:** `test/test_antenna_offset.cpp`
```cpp
// Test basic 2D antenna offset
void test_antenna_offset_2d() {
  float x = 0, y = 0;
  float yaw = PI/2; // 90 degrees
  
  applyAntennaOffsetBasic(x, y, yaw, 10, 0); // 10cm forward offset
  
  // After 90° rotation, forward becomes left
  assert(abs(x - 0) < 0.01);    // X should be ~0
  assert(abs(y - 0.1) < 0.01);  // Y should be ~0.1m (10cm)
}

// Test 3D rotation matrix
void test_rotation_matrix_3d() {
  RotationMatrix R = calculateRotationMatrix(0, 0, PI/2); // 90° yaw
  float input[3] = {1, 0, 0};  // 1m forward
  float output[3];
  
  transformVector(R, input, output);
  
  assert(abs(output[0] - 0) < 0.01);  // X becomes 0
  assert(abs(output[1] - 1) < 0.01);  // Y becomes 1
  assert(abs(output[2] - 0) < 0.01);  // Z stays 0
}
```

#### Test 2: Virtuelle Höhenkompensation
**Datei:** `test/test_height_compensation.cpp`
```cpp
void test_virtual_height_basic() {
  float gpsHeight = 100.0; // 100m GPS height
  float roll = 0.174;      // 10 degrees roll
  float pitch = 0;         // 0 degrees pitch
  
  float virtualHeight = calculateVirtualGroundHeight(gpsHeight, roll, pitch);
  
  // Should compensate for robot tilt
  assert(virtualHeight < gpsHeight); // Virtual height should be lower
  assert(abs(virtualHeight - 99.96) < 0.01); // ~4cm compensation for 25cm robot height
}

void test_height_filter() {
  filteredVirtualHeight = 100.0;
  
  updateVirtualHeight(101.0); // 1m jump
  
  // Filter should smooth the change
  assert(filteredVirtualHeight > 100.0);
  assert(filteredVirtualHeight < 101.0);
  assert(abs(filteredVirtualHeight - 100.1) < 0.05); // ~10cm change with alpha=0.1
}
```

#### Test 3: 3D-Orientierungsfilter
**Datei:** `test/test_orientation_3d.cpp`
```cpp
void test_complementary_filter() {
  // Simulate IMU data
  imu.accelX = 0; imu.accelY = 1.7; imu.accelZ = 9.8; // 10° roll
  imu.gyroX = 0.1; imu.gyroY = 0; imu.gyroZ = 0;      // 0.1 rad/s roll rate
  
  stateRoll = 0; // Start at 0
  
  updateOrientation3D(0.1); // 100ms update
  
  // Should blend accelerometer and gyro
  float expectedRoll = 0.174; // ~10 degrees
  assert(abs(stateRoll - expectedRoll) < 0.05); // Within 3 degrees
}
```

### Integrationstests

#### Test 4: GPS/IMU-Datenfusion
**Datei:** `test/test_integration.cpp`
```cpp
void test_complete_state_estimation() {
  // Setup test scenario
  gps.solution = SOL_FIXED;
  gps.relPosN = 10.0;  // 10m north
  gps.relPosE = 5.0;   // 5m east
  gps.height = 100.0;  // 100m height
  gps.hAccuracy = 0.02; // 2cm accuracy
  
  // Robot tilted 10° forward
  stateRoll = 0;
  statePitch = 0.174; // 10 degrees
  stateYaw = 0;
  
  // Enable all features for test
  #define ENABLE_ANTENNA_OFFSET true
  #define ENABLE_VIRTUAL_HEIGHT_COMP true
  #define ENABLE_3D_ORIENTATION true
  
  computeRobotState();
  
  // Verify all corrections applied
  assert(stateX != gps.relPosN);     // Antenna offset applied
  assert(stateY != gps.relPosE);     // Antenna offset applied
  assert(stateGroundHeight != gps.height); // Height compensation applied
  
  // Verify reasonable values
  assert(abs(stateX - 10.0) < 0.5);  // Within 50cm of expected
  assert(abs(stateY - 5.0) < 0.5);   // Within 50cm of expected
  assert(stateGroundHeight < gps.height); // Compensated height lower
}
```

### Feldtests (Field Tests)

#### Test 5: Präzisionstest auf ebener Fläche
**Protokoll:** `test/field_test_precision.md`
```markdown
## Präzisionstest - Ebene Fläche

### Setup
- RTK Base Station aktiv
- Robot auf ebener Rasenfläche
- 10x10m Quadrat markiert
- Alle GPS/IMU Features aktiviert

### Durchführung
1. Robot in Ecke A positionieren
2. Automatische Fahrt: A→B→C→D→A (10m Seiten)
3. 5 Runden fahren
4. GPS-Logs aufzeichnen

### Erwartete Ergebnisse
- Abweichung von Sollkurs: <5cm RMS
- Rückkehr zu Startpunkt: <3cm
- Keine systematischen Drifts

### Messwerte
- [ ] Runde 1: ___cm RMS, ___cm Endabweichung
- [ ] Runde 2: ___cm RMS, ___cm Endabweichung
- [ ] Runde 3: ___cm RMS, ___cm Endabweichung
- [ ] Runde 4: ___cm RMS, ___cm Endabweichung
- [ ] Runde 5: ___cm RMS, ___cm Endabweichung
```

#### Test 6: Hangtest (geneigte Flächen)
**Protokoll:** `test/field_test_slope.md`
```markdown
## Hangtest - Geneigte Flächen

### Setup
- Hang mit 5°, 10°, 15° Neigung
- 20m gerade Strecke bergauf/bergab
- Referenz-GPS-Punkte vermessen

### Test 6a: 5° Neigung
- [ ] Bergauf: Spurabweichung ___cm, Höhenstabilität ___cm
- [ ] Bergab: Spurabweichung ___cm, Höhenstabilität ___cm
- [ ] Quer: Spurabweichung ___cm, Seitenstabilität ___cm

### Test 6b: 10° Neigung
- [ ] Bergauf: Spurabweichung ___cm, Höhenstabilität ___cm
- [ ] Bergab: Spurabweichung ___cm, Höhenstabilität ___cm
- [ ] Quer: Spurabweichung ___cm, Seitenstabilität ___cm

### Test 6c: 15° Neigung (Grenztest)
- [ ] Bergauf: Spurabweichung ___cm, Stabilität ___
- [ ] Bergab: Spurabweichung ___cm, Stabilität ___
- [ ] Quer: Spurabweichung ___cm, Stabilität ___

### Erwartete Ergebnisse
- Virtuelle Höhe stabiler als GPS-Rohhöhe
- Spurabweichung <10cm auch bei 15° Neigung
- Keine Orientierungsinstabilität
```

#### Test 7: Langzeittest
**Protokoll:** `test/field_test_longterm.md`
```markdown
## Langzeittest - 2h Dauerbetrieb

### Setup
- Große Rasenfläche (50x50m)
- Mähmuster: Spirale von außen nach innen
- Kontinuierliche Datenaufzeichnung

### Überwachung (alle 15min)
- [ ] 0:15 - Position: ___cm Abweichung, Orientierung: ___° Drift
- [ ] 0:30 - Position: ___cm Abweichung, Orientierung: ___° Drift
- [ ] 0:45 - Position: ___cm Abweichung, Orientierung: ___° Drift
- [ ] 1:00 - Position: ___cm Abweichung, Orientierung: ___° Drift
- [ ] 1:15 - Position: ___cm Abweichung, Orientierung: ___° Drift
- [ ] 1:30 - Position: ___cm Abweichung, Orientierung: ___° Drift
- [ ] 1:45 - Position: ___cm Abweichung, Orientierung: ___° Drift
- [ ] 2:00 - Position: ___cm Abweichung, Orientierung: ___° Drift

### Erwartete Ergebnisse
- Positionsdrift <10cm über 2h
- Orientierungsdrift <2° über 2h
- Keine Performance-Degradation
- Speicherverbrauch konstant
```

### Automatisierte Testausführung
**Datei:** `test/run_all_tests.cpp`
```cpp
// Automated test runner
void runAllTests() {
  CONSOLE.println("Starting GPS/IMU Test Suite...");
  
  // Unit tests
  test_antenna_offset_2d();
  test_rotation_matrix_3d();
  test_virtual_height_basic();
  test_height_filter();
  test_complementary_filter();
  
  // Integration tests
  test_complete_state_estimation();
  
  CONSOLE.println("All tests passed!");
}

// Test result logging
void logTestResult(const char* testName, bool passed, float value = 0) {
  CONSOLE.print("TEST,");
  CONSOLE.print(testName);
  CONSOLE.print(",");
  CONSOLE.print(passed ? "PASS" : "FAIL");
  CONSOLE.print(",");
  CONSOLE.println(value, 3);
}
```

## 7. Risiken und Mitigation

### Risiken:
- Erhöhte Rechenzeit durch 3D-Berechnungen
- Mögliche Instabilität bei extremen Neigungen
- Kompatibilitätsprobleme mit bestehender Software

### Mitigation:
- Feature-Flags für schrittweise Aktivierung
- Fallback auf bestehende 2D-Schätzung bei Problemen
- Umfassende Tests vor Produktionsfreigabe

## 8. Erfolgs-Kriterien

- [ ] Verbesserte GPS-Positionsgenauigkeit (< 5cm Abweichung)
- [ ] Stabile Navigation auf Neigungen bis 15°
- [ ] Keine Regression in bestehender Funktionalität
- [ ] Erfolgreiche Integration ohne AT+-Schnittstellenänderungen
- [ ] Dokumentierte Performance-Verbesserungen

## 9. Dokumentation

- [ ] Code-Kommentare für alle neuen Funktionen
- [ ] Aktualisierung der Benutzer-Dokumentation
- [ ] Konfigurationsanleitung für neue Parameter
- [ ] Troubleshooting-Guide für häufige Probleme

---

**Erstellt:** $(date)
**Letzte Aktualisierung:** $(date)
**Verantwortlich:** Entwicklungsteam
**Status:** In Planung