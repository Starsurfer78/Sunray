# TODO: Motor Control & PID Optimization

## Übersicht
Diese TODO-Liste enthält konkrete Code-Verbesserungen für die Motorsteuerung und PID-Regler basierend auf der Analyse von `motor.cpp`, `pid.cpp`, `SerialRobotDriver.cpp` und `config_alfred.h`.

---

## 1. PID-Controller Optimierungen

### 1.1 Adaptive TaMax Implementation
**Priorität:** Hoch  
**Datei:** `motor.cpp` - Funktion `control()`  
**Problem:** Feste TaMax-Werte (0.1) unabhängig von Systemlast

```cpp
// VORHER (Zeile ~524):
motorLeftPID.TaMax = 0.1;

// NACHHER:
float systemLoad = getSystemLoad(); // Neue Hilfsfunktion
float adaptiveTaMax = constrain(systemLoad * 0.05f, 0.02f, 0.1f);
motorLeftPID.TaMax = adaptiveTaMax;
motorRightPID.TaMax = adaptiveTaMax;
```

### 1.2 PID Parameter Caching
**Priorität:** Mittel  
**Datei:** `motor.cpp` - Funktion `control()`  
**Problem:** Redundante Parameter-Setzung bei jedem Aufruf

```cpp
// Neue Klassenvariablen in motor.h:
bool pidParametersChanged = true;
float lastKp = -1, lastKi = -1, lastKd = -1;

// In control() Funktion:
if (pidParametersChanged || 
    lastKp != MOTOR_PID_KP || 
    lastKi != MOTOR_PID_KI || 
    lastKd != MOTOR_PID_KD) {
    
    motorLeftPID.Kp = motorRightPID.Kp = MOTOR_PID_KP;
    motorLeftPID.Ki = motorRightPID.Ki = MOTOR_PID_KI;
    motorLeftPID.Kd = motorRightPID.Kd = MOTOR_PID_KD;
    
    lastKp = MOTOR_PID_KP;
    lastKi = MOTOR_PID_KI;
    lastKd = MOTOR_PID_KD;
    pidParametersChanged = false;
}
```

### 1.3 PID Anti-Windup Verbesserung
**Priorität:** Mittel  
**Datei:** `pid.cpp` - Funktion `compute()`  
**Problem:** Einfache Begrenzung ohne Berücksichtigung der Ausgangssättigung

```cpp
// In pid.cpp nach Zeile ~56:
// Verbesserte Anti-Windup Logik
if (y > y_max || y < y_min) {
    // Nur integrieren wenn Ausgabe nicht gesättigt ist
    // oder Integration die Sättigung reduziert
    float unsaturatedY = Kp * e + Ki * Ta * esum + Kd/Ta * (e - eold);
    if ((y > y_max && unsaturatedY < y) || 
        (y < y_min && unsaturatedY > y)) {
        // Integration hilft - behalten
    } else {
        // Integration verschlechtert - zurücksetzen
        esum -= e;
    }
}
```

---

## 2. Motorstromüberwachung Verbesserungen

### 2.1 Dynamische Stromschwellenwerte mit Hysterese
**Priorität:** Hoch  
**Datei:** `config_alfred.h` und `motor.cpp`  
**Problem:** Zu aggressive statische Schwellenwerte führen zu Fehlalarmen

```cpp
// In config_alfred.h ergänzen:
#define MOTOR_CURRENT_HYSTERESIS 0.1f
#define MOTOR_TOO_LOW_CURRENT_MIN 0.01f  // Erhöht von 0.005
#define MOTOR_TOO_LOW_CURRENT_MAX 0.02f
#define MOTOR_FAULT_CURRENT_HYSTERESIS 0.5f

// Neue Funktion in motor.cpp:
float Motor::getAdaptiveLowCurrentThreshold(int pwm) {
    float pwmRatio = abs(pwm) / 255.0f;
    return MOTOR_TOO_LOW_CURRENT_MIN + 
           (MOTOR_TOO_LOW_CURRENT_MAX - MOTOR_TOO_LOW_CURRENT_MIN) * pwmRatio;
}

// In checkCurrentTooLowError() modifizieren:
bool Motor::checkCurrentTooLowError() {
    static bool leftLowCurrentState = false;
    static bool rightLowCurrentState = false;
    
    float leftThreshold = getAdaptiveLowCurrentThreshold(motorLeftPWMCurr);
    float rightThreshold = getAdaptiveLowCurrentThreshold(motorRightPWMCurr);
    
    // Hysterese anwenden
    if (!leftLowCurrentState) {
        leftLowCurrentState = (motorLeftCurr < leftThreshold) && (abs(motorLeftPWMCurr) > 100);
    } else {
        leftLowCurrentState = (motorLeftCurr < leftThreshold + MOTOR_CURRENT_HYSTERESIS) && (abs(motorLeftPWMCurr) > 100);
    }
    
    // Analog für rechten Motor...
    return leftLowCurrentState || rightLowCurrentState;
}
```

### 2.2 Stromtrend-Analyse
**Priorität:** Mittel  
**Datei:** `motor.h` und `motor.cpp`  
**Problem:** Keine Früherkennung von Blockierungen durch Stromtrend

```cpp
// In motor.h neue Klassenvariablen:
static const int CURRENT_HISTORY_SIZE = 10;
float motorLeftCurrentHistory[CURRENT_HISTORY_SIZE];
float motorRightCurrentHistory[CURRENT_HISTORY_SIZE];
int currentHistoryIndex = 0;
unsigned long lastCurrentTrendCheck = 0;

// Neue Funktion in motor.cpp:
bool Motor::checkCurrentTrend() {
    if (millis() - lastCurrentTrendCheck < 100) return false; // 10Hz
    
    // Aktualisiere Historie
    motorLeftCurrentHistory[currentHistoryIndex] = motorLeftCurr;
    motorRightCurrentHistory[currentHistoryIndex] = motorRightCurr;
    currentHistoryIndex = (currentHistoryIndex + 1) % CURRENT_HISTORY_SIZE;
    
    // Berechne Trend (einfache lineare Regression)
    float leftTrend = calculateCurrentTrend(motorLeftCurrentHistory);
    float rightTrend = calculateCurrentTrend(motorRightCurrentHistory);
    
    // Warnung bei steigendem Strom > 0.1A/s
    bool trendWarning = (leftTrend > 0.1f) || (rightTrend > 0.1f);
    
    lastCurrentTrendCheck = millis();
    return trendWarning;
}
```

---

## 3. SerialRobotDriver Kommunikationsoptimierung

### 3.1 Adaptive Kommunikationsfrequenz
**Priorität:** Mittel  
**Datei:** `SerialRobotDriver.cpp` - Funktion `run()`  
**Problem:** Feste 50Hz können bei hoher Systemlast problematisch sein

```cpp
// In SerialRobotDriver.cpp, run() Funktion (Zeile ~536):
// VORHER:
if (millis() > nextMotorTime){
    nextMotorTime = millis() + 20; // 50 hz
    requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
}

// NACHHER:
float systemLoad = getCpuLoad(); // Neue Funktion basierend auf CPU-Temperatur
int motorFrequency = (systemLoad > 0.8f) ? 40 : 50; // Hz
int motorInterval = 1000 / motorFrequency;

if (millis() > nextMotorTime){
    nextMotorTime = millis() + motorInterval;
    requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);
}
```

### 3.2 Verbesserte Mow-Motor PWM-Kontrolle
**Priorität:** Niedrig  
**Datei:** `SerialRobotDriver.cpp` - Funktion `setMotorPwm()`  
**Problem:** Harte 255/-255 Werte für Mähmotor sind suboptimal

```cpp
// In setMotorPwm() (Zeile ~610):
// VORHER:
if (mowPwm > 0) mowPwm = 255;
else if (mowPwm < 0) mowPwm = -255;

// NACHHER:
if (mowPwm > 0) {
    // Sanfterer Start: 180-255 statt sofort 255
    mowPwm = map(mowPwm, 1, 255, 180, 255);
} else if (mowPwm < 0) {
    mowPwm = map(mowPwm, -255, -1, -255, -180);
}
```

### 3.3 Kommunikations-Timeout Verbesserung
**Priorität:** Hoch  
**Datei:** `SerialRobotDriver.cpp` - Funktion `run()`  
**Problem:** Unzureichende Fehlerbehandlung bei Kommunikationsausfällen

```cpp
// Neue Klassenvariablen in SerialRobotDriver.h:
unsigned long lastSuccessfulMotorResponse = 0;
static const unsigned long MOTOR_COMM_TIMEOUT = 200; // ms

// In motorResponse() ergänzen:
void SerialRobotDriver::motorResponse() {
    // ... bestehender Code ...
    lastSuccessfulMotorResponse = millis();
    mcuCommunicationLost = false;
}

// In run() ergänzen:
if (millis() - lastSuccessfulMotorResponse > MOTOR_COMM_TIMEOUT) {
    if (!mcuCommunicationLost) {
        CONSOLE.println("WARN: Motor communication timeout - entering safe mode");
        mcuCommunicationLost = true;
        // Optional: Motoren stoppen
        requestLeftPwm = requestRightPwm = requestMowPwm = 0;
    }
}
```

---

## 4. Performance-Optimierungen

### 4.1 Adaptive Low-Pass Filter
**Priorität:** Niedrig  
**Datei:** `motor.cpp` und `lowpass_filter.cpp`  
**Problem:** Statische Filterkoeffizienten unabhängig von Geschwindigkeit

```cpp
// Neue Funktion in motor.cpp:
float Motor::getAdaptiveFilterAlpha(float speed) {
    // Bei höherer Geschwindigkeit weniger filtern (schnellere Reaktion)
    float maxSpeed = 1.0f; // m/s
    float minAlpha = 0.1f;
    float maxAlpha = 0.3f;
    
    float speedRatio = constrain(abs(speed) / maxSpeed, 0.0f, 1.0f);
    return minAlpha + (maxAlpha - minAlpha) * speedRatio;
}

// In control() Funktion:
float leftAlpha = getAdaptiveFilterAlpha(motorLeftRpmSet * wheelDiameter * PI / 60.0f);
float rightAlpha = getAdaptiveFilterAlpha(motorRightRpmSet * wheelDiameter * PI / 60.0f);

motorLeftPID.x = motorLeftLpf.filter(motorLeftRpmCurr, leftAlpha);
motorRightPID.x = motorRightLpf.filter(motorRightRpmCurr, rightAlpha);
```

### 4.2 Encoder-Tick Glitch-Filterung
**Priorität:** Mittel  
**Datei:** `SerialRobotDriver.cpp` - Funktion `getMotorEncoderTicks()`  
**Problem:** Unrealistische Tick-Sprünge werden nicht gefiltert

```cpp
// In getMotorEncoderTicks() nach Zeile ~674:
// Glitch-Filterung für unrealistische Werte
static const int MAX_REASONABLE_TICKS = 100; // pro Zyklus

if (abs(leftTicks) > MAX_REASONABLE_TICKS) {
    CONSOLE.print("WARN: Unrealistic left encoder ticks: ");
    CONSOLE.println(leftTicks);
    leftTicks = 0;
}

if (abs(rightTicks) > MAX_REASONABLE_TICKS) {
    CONSOLE.print("WARN: Unrealistic right encoder ticks: ");
    CONSOLE.println(rightTicks);
    rightTicks = 0;
}
```

---

## 5. Konfigurationsoptimierungen für Alfred

### 5.1 Optimierte PID-Parameter
**Priorität:** Hoch  
**Datei:** `config_alfred.h`  
**Problem:** Suboptimale PID-Werte für 320 Ticks/Revolution

```cpp
// VORHER (Zeile ~153-157):
#define MOTOR_PID_KP     0.5
#define MOTOR_PID_KI     0.01
#define MOTOR_PID_KD     0.01
#define MOTOR_PID_RAMP   0

// NACHHER:
#define MOTOR_PID_KP     0.6    // Erhöht für bessere Reaktion
#define MOTOR_PID_KI     0.008  // Reduziert für Stabilität
#define MOTOR_PID_KD     0.015  // Leicht erhöht für Dämpfung
#define MOTOR_PID_RAMP   50     // Aktiviert für sanftere Übergänge
```

### 5.2 Verbesserte Stromschwellenwerte
**Priorität:** Hoch  
**Datei:** `config_alfred.h`  
**Problem:** Zu aggressive Schwellenwerte für Alfred-Hardware

```cpp
// VORHER (Zeile ~144-146):
#define MOTOR_FAULT_CURRENT 3.0
#define MOTOR_TOO_LOW_CURRENT 0.005
#define MOTOR_OVERLOAD_CURRENT 1.2

// NACHHER:
#define MOTOR_FAULT_CURRENT 4.0      // Erhöht von 3.0
#define MOTOR_TOO_LOW_CURRENT 0.01   // Erhöht von 0.005
#define MOTOR_OVERLOAD_CURRENT 1.5   // Erhöht von 1.2
```

---

## 6. Monitoring und Diagnostik

### 6.1 PID Performance Tracking
**Priorität:** Niedrig  
**Datei:** Neue Datei `pid_monitor.h/.cpp`  
**Problem:** Keine Überwachung der PID-Performance

```cpp
// pid_monitor.h
class PIDMonitor {
public:
    void update(float setpoint, float actual, float output);
    float getAverageError();
    float getMaxError();
    void reset();
    
private:
    static const int HISTORY_SIZE = 100;
    float errorHistory[HISTORY_SIZE];
    int historyIndex = 0;
    float sumError = 0;
    float maxError = 0;
};

// In motor.cpp integrieren:
PIDMonitor leftMotorMonitor, rightMotorMonitor;

// Nach PID compute():
leftMotorMonitor.update(motorLeftRpmSet, motorLeftRpmCurr, motorLeftPID.y);
rightMotorMonitor.update(motorRightRpmSet, motorRightRpmCurr, motorRightPID.y);
```

### 6.2 AT+ Befehle für Motordiagnostik
**Priorität:** Niedrig  
**Datei:** `comm.cpp` oder entsprechende AT+ Handler  
**Problem:** Fehlende Diagnosebefehle für Motorstrom und PID-Status

```cpp
// Neue AT+ Befehle:
// AT+MOTOR_CURRENT -> Gibt aktuelle Motorströme zurück
// AT+MOTOR_PID_STATUS -> Gibt PID-Fehler und Performance zurück
// AT+MOTOR_THRESHOLDS -> Zeigt/setzt Stromschwellenwerte

// Beispiel Implementation:
void handleMotorCurrentCommand() {
    CONSOLE.print("AT+MOTOR_CURRENT,");
    CONSOLE.print(motorLeftCurr, 3);
    CONSOLE.print(",");
    CONSOLE.print(motorRightCurr, 3);
    CONSOLE.print(",");
    CONSOLE.println(mowCurr, 3);
}
```

---

## Implementierungsreihenfolge

1. **Phase 1 (Kritisch):**
   - 2.1 Dynamische Stromschwellenwerte mit Hysterese
   - 5.1 Optimierte PID-Parameter für Alfred
   - 5.2 Verbesserte Stromschwellenwerte
   - 3.3 Kommunikations-Timeout Verbesserung

2. **Phase 2 (Wichtig):**
   - 1.1 Adaptive TaMax Implementation
   - 1.2 PID Parameter Caching
   - 4.2 Encoder-Tick Glitch-Filterung
   - 2.2 Stromtrend-Analyse

3. **Phase 3 (Optimierung):**
   - 3.1 Adaptive Kommunikationsfrequenz
   - 1.3 PID Anti-Windup Verbesserung
   - 4.1 Adaptive Low-Pass Filter

4. **Phase 4 (Monitoring):**
   - 6.1 PID Performance Tracking
   - 6.2 AT+ Befehle für Motordiagnostik
   - 3.2 Verbesserte Mow-Motor PWM-Kontrolle

---

## Testprotokoll

Für jede Implementierung:

1. **Build-Test:** Arduino Due kompiliert ohne Fehler
2. **Smoke-Test:** Startsequenz bis "bereit" funktioniert
3. **Motor-Test:** Enable/Disable ohne Bewegung
4. **AT+ Test:** Alle bestehenden Befehle funktionieren unverändert
5. **Feld-Test:** 2-3 Min Mähen im sicheren Bereich
6. **Regression-Test:** Keine neuen Fehler in Logs

---

## Notizen

- Alle Änderungen müssen rückwärtskompatibel zur Alfred-Kommunikation sein
- AT+ Befehle dürfen nicht verändert oder entfernt werden
- Jede Änderung einzeln implementieren und testen
- Bei Problemen sofort zurückrollen
- Logs und Metriken vor/nach Änderungen vergleichen