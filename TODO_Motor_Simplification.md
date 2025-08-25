# TODO: Motor Control Simplification

## Ziel
Vereinfachung der Motorsteuerung durch direkten RPM-zu-PWM-Faktor anstatt komplexer PID-Steuerung mit instabilen Tick-basierten Berechnungen.

## Problem mit aktueller Implementierung
- Tick-basierte RPM-Berechnung ist instabil wegen schwankender Iterationszeiten
- Komplexe PID-Steuerung mit vielen Parametern
- Schwer zu kalibrieren und zu debuggen

## Vorgeschlagene Lösung
1. **Einfacher RPM-zu-PWM-Faktor**: Direkte Umrechnung von gewünschter RPM zu PWM-Wert
2. **Messung der tatsächlichen Geschwindigkeit**: Über Radumdrehungen
3. **GPS-Verifikation**: Geschwindigkeitsmessung auf gerader Strecke

## Code-Änderungen

### 1. Neue Konstanten in config.h
```cpp
// Motor RPM-zu-PWM Kalibrierung
#define MOTOR_RPM_TO_PWM_FACTOR 2.55f    // PWM pro RPM (255 PWM / 100 RPM)
#define MOTOR_BASE_RPM 50.0f             // Basis-RPM für normale Fahrt
#define MOTOR_MAX_RPM 100.0f             // Maximum RPM
#define MOTOR_MIN_RPM 10.0f              // Minimum RPM

// Kalibrierung und Test
#define MOTOR_CALIBRATION_STEPS 10       // Anzahl Kalibrierungsschritte
#define MOTOR_CALIBRATION_DURATION 5000  // ms pro Schritt
#define MOTOR_WHEEL_CIRCUMFERENCE_CM 31.4f // Radumfang in cm (d=10cm)
```

### 2. Vereinfachte control() Funktion
```cpp
void Motor::control() {
  if (!tractionMotorsEnabled) {
    motorLeftPWMCurr = 0;
    motorRightPWMCurr = 0;
    return;
  }

  // Direkte RPM-zu-PWM Umrechnung
  float leftPWM = motorLeftRpmSet * MOTOR_RPM_TO_PWM_FACTOR;
  float rightPWM = motorRightRpmSet * MOTOR_RPM_TO_PWM_FACTOR;

  // PWM-Limits anwenden
  leftPWM = constrain(leftPWM, -pwmMax, pwmMax);
  rightPWM = constrain(rightPWM, -pwmMax, pwmMax);

  // Sanfte Änderungen (optional)
  motorLeftPWMCurr = motorLeftPWMCurr * 0.9f + leftPWM * 0.1f;
  motorRightPWMCurr = motorRightPWMCurr * 0.9f + rightPWM * 0.1f;

  // PWM setzen
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
}
```

### 3. Neue Kalibrierungsfunktion
```cpp
void Motor::calibrateRpmToPwm() {
  CONSOLE.println("Starting RPM-to-PWM calibration...");
  
  for (int step = 1; step <= MOTOR_CALIBRATION_STEPS; step++) {
    float testRpm = MOTOR_MIN_RPM + (step * (MOTOR_MAX_RPM - MOTOR_MIN_RPM) / MOTOR_CALIBRATION_STEPS);
    float testPwm = testRpm * MOTOR_RPM_TO_PWM_FACTOR;
    
    CONSOLE.print("Step "); CONSOLE.print(step);
    CONSOLE.print(": RPM="); CONSOLE.print(testRpm);
    CONSOLE.print(", PWM="); CONSOLE.println(testPwm);
    
    // Motor mit Test-PWM laufen lassen
    speedPWM(testPwm, testPwm, 0);
    
    unsigned long startTime = millis();
    unsigned long startTicks = ticksLeft;
    
    // Warten und Ticks messen
    delay(MOTOR_CALIBRATION_DURATION);
    
    unsigned long endTicks = ticksLeft;
    unsigned long tickDiff = endTicks - startTicks;
    float actualRpm = (tickDiff * 60000.0f) / (MOTOR_CALIBRATION_DURATION * ticksPerRevolution);
    
    CONSOLE.print("  -> Actual RPM: "); CONSOLE.println(actualRpm);
    CONSOLE.print("  -> Factor should be: "); CONSOLE.println(testPwm / actualRpm);
    
    delay(1000); // Pause zwischen Tests
  }
  
  speedPWM(0, 0, 0); // Stoppen
  CONSOLE.println("Calibration complete. Adjust MOTOR_RPM_TO_PWM_FACTOR in config.h");
}
```

### 4. Geschwindigkeitsmessung über Radumdrehungen
```cpp
void Motor::measureActualSpeed() {
  static unsigned long lastMeasureTime = 0;
  static unsigned long lastTicksLeft = 0;
  static unsigned long lastTicksRight = 0;
  
  if (millis() - lastMeasureTime >= 1000) { // Jede Sekunde
    unsigned long tickDiffLeft = ticksLeft - lastTicksLeft;
    unsigned long tickDiffRight = ticksRight - lastTicksRight;
    
    // RPM berechnen
    float rpmLeft = (tickDiffLeft * 60.0f) / ticksPerRevolution;
    float rpmRight = (tickDiffRight * 60.0f) / ticksPerRevolution;
    
    // Geschwindigkeit in cm/s
    float speedLeft = (rpmLeft * MOTOR_WHEEL_CIRCUMFERENCE_CM) / 60.0f;
    float speedRight = (rpmRight * MOTOR_WHEEL_CIRCUMFERENCE_CM) / 60.0f;
    
    motorLeftSpeedMeasured = speedLeft;
    motorRightSpeedMeasured = speedRight;
    
    lastTicksLeft = ticksLeft;
    lastTicksRight = ticksRight;
    lastMeasureTime = millis();
  }
}
```

### 5. Neue AT+ Befehle für Kalibrierung
```cpp
// In comm.cpp hinzufügen:
case "AT+MOTORCAL":
  motor.calibrateRpmToPwm();
  break;
  
case "AT+MOTORTEST":
  // Einfacher Test mit verschiedenen RPM-Werten
  motor.testRpmToPwm();
  break;
```

## Implementierungsreihenfolge
1. ✅ Konstanten zu config.h hinzufügen
2. ✅ Vereinfachte control() Funktion implementieren
3. ✅ Kalibrierungsfunktion hinzufügen
4. ✅ Geschwindigkeitsmessung implementieren
5. ✅ AT+ Befehle für Tests hinzufügen
6. ✅ Ausgiebige Tests durchführen

## Vorteile
- **Einfacher**: Direkte RPM-zu-PWM Umrechnung
- **Stabiler**: Keine instabilen Tick-Zeitberechnungen
- **Kalibrierbar**: Einfache Anpassung des Faktors
- **Debuggbar**: Klare Zusammenhänge zwischen RPM und PWM
- **Testbar**: Systematische Kalibrierung möglich

## Risiken
- Weniger präzise Regelung als PID
- Benötigt manuelle Kalibrierung
- Keine automatische Anpassung an Laständerungen

## Fallback
Die alte PID-Implementierung bleibt als Kommentar erhalten und kann über #define wieder aktiviert werden.