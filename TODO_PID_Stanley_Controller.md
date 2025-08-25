# TODO: PID und Stanley Controller Implementation

## Übersicht
Implementierung eines robusten PID Controllers für Motorsteuerung und eines Stanley Controllers für präzise Pfadverfolgung als bessere Alternative zur aktuellen Motor-Vereinfachung.

## Ziele
- Stabilere und präzisere Motorsteuerung
- Bessere Pfadverfolgung durch Stanley Controller
- Modularer und wartbarer Code
- Rückwärtskompatibilität mit bestehenden AT+ Befehlen

## Phase 1: PID Controller Verbesserung (1-2 Tage)

### 1.1 PID Parameter Optimierung
```cpp
// In config.h - Neue PID Konstanten
#define MOTOR_PID_KP_DEFAULT 2.0f
#define MOTOR_PID_KI_DEFAULT 0.5f
#define MOTOR_PID_KD_DEFAULT 0.1f
#define MOTOR_PID_INTEGRAL_LIMIT 100.0f
#define MOTOR_PID_DERIVATIVE_FILTER 0.1f
#define MOTOR_PID_OUTPUT_LIMIT 255.0f
```

### 1.2 Erweiterte PID Klasse
```cpp
// In pid.h - Erweiterte PID Implementierung
class EnhancedPID {
private:
    float kp, ki, kd;
    float integral, lastError;
    float integralLimit;
    float derivativeFilter;
    float outputLimit;
    unsigned long lastTime;
    
public:
    EnhancedPID(float kp, float ki, float kd);
    void setLimits(float integralLimit, float outputLimit);
    void setDerivativeFilter(float alpha);
    float compute(float setpoint, float input, float deltaTime);
    void reset();
    void tune(float kp, float ki, float kd);
    float getIntegral() const { return integral; }
};
```

### 1.3 Motor Control Verbesserung
```cpp
// In motor.cpp - Verbesserte control() Funktion
void Motor::control() {
    unsigned long now = millis();
    float deltaTime = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;
    
    // Anti-Windup für PID
    if (deltaTime > MOTOR_MAX_DELTA_TIME) {
        motorLeftPID.reset();
        motorRightPID.reset();
        return;
    }
    
    // Feedforward Komponente
    float leftFeedforward = motorLeftRpmSet * MOTOR_RPM_TO_PWM_FEEDFORWARD;
    float rightFeedforward = motorRightRpmSet * MOTOR_RPM_TO_PWM_FEEDFORWARD;
    
    // PID Berechnung
    float leftPidOutput = motorLeftPID.compute(motorLeftRpmSet, motorLeftRpmCurr, deltaTime);
    float rightPidOutput = motorRightPID.compute(motorRightRpmSet, motorRightRpmCurr, deltaTime);
    
    // Kombinierte Ausgabe
    motorLeftPWMCurr = constrain(leftFeedforward + leftPidOutput, -255, 255);
    motorRightPWMCurr = constrain(rightFeedforward + rightPidOutput, -255, 255);
    
    speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
}
```

## Phase 2: Stanley Controller Implementation (3-5 Tage)

### 2.1 Stanley Controller Klasse
```cpp
// Neue Datei: stanley_controller.h
class StanleyController {
private:
    float k;  // Stanley gain parameter
    float ks; // Softening constant
    float maxSteerAngle;
    float wheelbase;
    
public:
    StanleyController(float k = 1.0f, float ks = 0.1f);
    void setParameters(float k, float ks, float maxSteerAngle, float wheelbase);
    float computeSteeringAngle(const Point& robotPos, float robotHeading, 
                              const Path& targetPath, float velocity);
    float computeCrossTrackError(const Point& robotPos, const Path& path);
    float computeHeadingError(float robotHeading, const Path& path);
};
```

### 2.2 Path Representation
```cpp
// In map.h - Erweiterte Pfad-Struktur
struct PathPoint {
    float x, y;
    float heading;  // Gewünschte Richtung an diesem Punkt
    float curvature; // Krümmung für Vorausschau
};

class Path {
private:
    std::vector<PathPoint> points;
    int currentIndex;
    
public:
    void addPoint(float x, float y, float heading = 0);
    PathPoint getClosestPoint(const Point& robotPos);
    PathPoint getLookaheadPoint(const Point& robotPos, float lookaheadDistance);
    float getHeadingAt(const Point& pos);
    bool isComplete() const;
};
```

### 2.3 Integration in Robot Control
```cpp
// In robot.cpp - Stanley Controller Integration
void Robot::stateForward() {
    if (!gps.solution == SOL_FIXED) return;
    
    Point currentPos = {gps.relPosN, gps.relPosE};
    float currentHeading = imu.yaw;
    float currentVelocity = sqrt(motorLeftRpmCurr * motorLeftRpmCurr + 
                                motorRightRpmCurr * motorRightRpmCurr) * WHEEL_CIRCUMFERENCE / 60.0f;
    
    // Stanley Controller für Lenkwinkel
    float steeringAngle = stanleyController.computeSteeringAngle(
        currentPos, currentHeading, targetPath, currentVelocity);
    
    // Umwandlung Lenkwinkel zu Differential Drive
    float angularVelocity = currentVelocity * tan(steeringAngle) / ROBOT_WHEELBASE;
    
    // Geschwindigkeitsregelung
    float targetLinearVel = min(ROBOT_MAX_SPEED, 
                               ROBOT_BASE_SPEED * (1.0f - abs(steeringAngle) / ROBOT_MAX_STEER_ANGLE));
    
    // Differential Drive Berechnung
    float leftVel = targetLinearVel - (angularVelocity * ROBOT_WHEELBASE / 2.0f);
    float rightVel = targetLinearVel + (angularVelocity * ROBOT_WHEELBASE / 2.0f);
    
    // RPM Umwandlung
    motorLeftRpmSet = leftVel * 60.0f / WHEEL_CIRCUMFERENCE;
    motorRightRpmSet = rightVel * 60.0f / WHEEL_CIRCUMFERENCE;
}
```

## Phase 3: Kalibrierung und Tuning (2-3 Tage)

### 3.1 Auto-Tuning AT+ Befehle
```cpp
// Neue AT+ Befehle für Tuning
// AT+PID_TUNE,LEFT,KP,KI,KD - PID Parameter setzen
// AT+PID_STATUS - Aktuelle PID Werte anzeigen
// AT+STANLEY_TUNE,K,KS - Stanley Parameter setzen
// AT+STANLEY_STATUS - Stanley Controller Status
// AT+PATH_TEST,DISTANCE - Testpfad fahren
```

### 3.2 Kalibrierungs-Funktionen
```cpp
// In motor.cpp - Auto-Tuning Funktionen
void Motor::autoTunePID() {
    // Ziegler-Nichols Methode Implementation
    float ku = findCriticalGain();
    float tu = findOscillationPeriod();
    
    float kp = 0.6f * ku;
    float ki = 2.0f * kp / tu;
    float kd = kp * tu / 8.0f;
    
    motorLeftPID.tune(kp, ki, kd);
    motorRightPID.tune(kp, ki, kd);
}

void Motor::testStepResponse() {
    // Sprungantwortest für PID Analyse
    unsigned long startTime = millis();
    motorLeftRpmSet = 50; // Test RPM
    
    while (millis() - startTime < 5000) {
        control();
        sense();
        
        // Log für Analyse
        CONSOLE.print(millis() - startTime);
        CONSOLE.print(",");
        CONSOLE.print(motorLeftRpmSet);
        CONSOLE.print(",");
        CONSOLE.println(motorLeftRpmCurr);
        
        delay(10);
    }
}
```

## Phase 4: Testing und Validation

### 4.1 Unit Tests
```cpp
// test_pid_stanley.cpp
void testPIDResponse() {
    EnhancedPID testPID(1.0, 0.1, 0.05);
    
    // Test Sprungantwortest
    float output = testPID.compute(100, 0, 0.01); // Setpoint 100, Input 0
    assert(output > 0); // Sollte positive Ausgabe haben
    
    // Test Steady State
    for (int i = 0; i < 1000; i++) {
        output = testPID.compute(100, 100, 0.01); // Setpoint erreicht
    }
    assert(abs(output) < 1.0); // Sollte nahe Null sein
}

void testStanleyController() {
    StanleyController stanley(1.0, 0.1);
    
    Point robotPos = {0, 0};
    Path straightPath;
    straightPath.addPoint(0, 0, 0);
    straightPath.addPoint(10, 0, 0);
    
    // Test gerader Pfad
    float steerAngle = stanley.computeSteeringAngle(robotPos, 0, straightPath, 1.0);
    assert(abs(steerAngle) < 0.1); // Sollte geradeaus fahren
    
    // Test Querversatz
    robotPos = {0, 1}; // 1m seitlicher Versatz
    steerAngle = stanley.computeSteeringAngle(robotPos, 0, straightPath, 1.0);
    assert(steerAngle < 0); // Sollte nach links lenken
}
```

### 4.2 Feldtests
```cpp
// Testsequenzen für Feldvalidierung
void runFieldTests() {
    // Test 1: Gerade Linie 10m
    testStraightLine(10.0);
    
    // Test 2: 90° Kurve
    testRightAngleTurn();
    
    // Test 3: S-Kurve
    testSCurve();
    
    // Test 4: Kreisfahrt
    testCircularPath(2.0); // 2m Radius
    
    // Test 5: Störungsunterdrückung
    testDisturbanceRejection();
}
```

## Implementierungsreihenfolge

1. **Tag 1-2**: Enhanced PID Klasse und Motor Control Verbesserung
2. **Tag 3-4**: Stanley Controller Grundimplementierung
3. **Tag 5-6**: Path Management und Integration
4. **Tag 7-8**: AT+ Befehle und Kalibrierung
5. **Tag 9-10**: Testing, Tuning und Dokumentation

## Vorteile gegenüber aktueller Lösung

- **Stabilität**: Robuste PID Implementation mit Anti-Windup
- **Präzision**: Stanley Controller für exakte Pfadverfolgung
- **Wartbarkeit**: Modularer, testbarer Code
- **Skalierbarkeit**: Erweiterbar für komplexere Pfade
- **Debugging**: Umfangreiche Logging und Test-Tools

## Risiken und Mitigation

- **Komplexität**: Schrittweise Implementation mit Fallback
- **Performance**: Profiling und Optimierung in kritischen Pfaden
- **Kalibrierung**: Auto-Tuning Tools und dokumentierte Verfahren
- **Kompatibilität**: Alle bestehenden AT+ Befehle bleiben funktional

## Erfolgsmetriken

- Pfadabweichung < 10cm bei 0.5m/s
- Überschwingen < 5% bei Geschwindigkeitsänderungen
- Stabile Kurvenfahrt ohne Oszillation
- Erfolgreiche Integration aller Testsequenzen

## Konfiguration

```cpp
// config.h - Neue Konstanten
#define ENABLE_ENHANCED_PID 1
#define ENABLE_STANLEY_CONTROLLER 1
#define MOTOR_PID_SAMPLE_TIME_MS 20
#define STANLEY_LOOKAHEAD_DISTANCE 1.0f
#define ROBOT_WHEELBASE 0.4f  // Radstand in Metern
#define ROBOT_MAX_STEER_ANGLE 0.5f  // Max Lenkwinkel in Radiant
```

Diese Implementation bietet eine professionelle, skalierbare Lösung für präzise Robotersteuerung mit bewährten Algorithmen aus der Fahrzeugtechnik.