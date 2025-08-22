# TODO: IMU Navigation Verbesserungen

## Übersicht
Diese TODO-Liste enthält Verbesserungsvorschläge für die Navigation mit MPU6050-Werten im Sunray-System.

## Aktuelle Situation
- **Yaw (Gierwinkel)**: Wird hauptsächlich für Navigation (Richtungsbestimmung, GPS-Fusion) und Hinderniserkennung verwendet
- **Pitch (Nickwinkel)** und **Roll (Rollwinkel)**: Werden nur begrenzt für Neigungserkennung und Sicherheitsfunktionen genutzt
- IMU ist über I2C mit Raspberry Pi verbunden (keine AT-Befehle erforderlich)

## Verbesserungsvorschläge

### 1. Erweiterte Pitch/Roll-Nutzung für Geländeadaption
- [ ] **Geschwindigkeitsanpassung bei Steigungen**
  - Geschwindigkeit bei steilen Anstiegen (Pitch > Schwellenwert) reduzieren
  - Implementierung in `StateEstimator.cpp` oder `motor.cpp`
  
- [ ] **Seitenhang-Kompensation**
  - Roll-Werte für Kompensation bei Seitenhängen nutzen
  - Motorgeschwindigkeiten entsprechend anpassen
  
- [ ] **Adaptive Schwellenwerte**
  - Dynamische Anpassung der Neigungsschwellenwerte basierend auf Gelände
  - Wetterabhängige Kalibrierung (Rutschgefahr)

### 2. Verbesserte Yaw-Fusion und Kalibrierung
- [ ] **Kalman-Filter Implementation**
  - Erweiterte Fusion von GPS- und IMU-Yaw-Daten
  - Bessere Rauschunterdrückung und Genauigkeit
  
- [ ] **Dynamische IMU-Kalibrierung**
  - Automatische Drift-Korrektur während des Betriebs
  - Temperaturkompensation für MPU6050
  
- [ ] **Magnetometer-Integration** (falls verfügbar)
  - Absolute Nordrichtung für bessere Orientierung
  - Kompensation magnetischer Störungen

### 3. Prädiktive Hinderniserkennung
- [ ] **Bewegungsvorhersage**
  - IMU-Daten für Vorhersage der Roboterbewegung nutzen
  - Frühzeitige Erkennung von Problemen
  
- [ ] **Vibrationserkennung**
  - Hochfrequente IMU-Daten für Vibrationsmuster analysieren
  - Erkennung von Messerproblemen oder Blockierungen
  
- [ ] **Rutsch-Erkennung**
  - Vergleich von erwarteter und tatsächlicher Bewegung
  - Kombination aus Rad-Odometrie und IMU-Daten

### 4. Adaptive Pfadplanung
- [ ] **Neigungsbasierte Routenwahl**
  - Bevorzugung flacherer Routen bei schwierigen Bedingungen
  - Integration in `map.cpp` Pfadplanungsalgorithmus
  
- [ ] **Griffigkeits-Mapping**
  - Speicherung von Geländeeigenschaften in der Karte
  - Lernende Algorithmen für optimale Routen
  
- [ ] **Wetteradaption**
  - Anpassung der Navigation bei verschiedenen Wetterbedingungen
  - Sicherheitsmargen bei Regen/Nässe

## Implementierungsreihenfolge (Empfehlung)

### Phase 1: Grundlagen (Niedrige Komplexität)
1. Geschwindigkeitsanpassung bei Steigungen
2. Erweiterte Neigungsschwellenwerte
3. Verbesserte Vibrationserkennung

### Phase 2: Erweiterte Fusion (Mittlere Komplexität)
1. Kalman-Filter für Yaw-Fusion
2. Dynamische IMU-Kalibrierung
3. Rutsch-Erkennung

### Phase 3: Intelligente Navigation (Hohe Komplexität)
1. Neigungsbasierte Pfadplanung
2. Griffigkeits-Mapping
3. Magnetometer-Integration

## Technische Umsetzungsdetails

### Raspberry Pi Verarbeitung
- Komplexere Algorithmen auf Raspberry Pi implementieren
- Arduino Due für Echtzeitsteuerung nutzen
- Optimierte Datenübertragung zwischen beiden Systemen

### Modulare Architektur
- Neue IMU-Verarbeitungsmodule erstellen
- Bestehende Navigation erweitern, nicht ersetzen
- Rückwärtskompatibilität gewährleisten

### Konfiguration
- Neue Parameter in `config.h` hinzufügen
- Feature-Flags für schrittweise Aktivierung
- Benutzerfreundliche Kalibrierungsroutinen

## Erwartete Vorteile
- **Robustere Navigation** in schwierigem Gelände
- **Effizientere Pfadplanung** durch Geländeberücksichtigung
- **Bessere Sicherheit** durch prädiktive Hinderniserkennung
- **Geringerer Verschleiß** durch angepasste Fahrweise
- **Höhere Mähqualität** durch stabilere Bewegungen

## Risiken und Überlegungen
- Erhöhte Systemkomplexität
- Mögliche Performance-Auswirkungen
- Notwendigkeit ausgiebiger Tests in verschiedenen Geländetypen
- Kalibrierungsaufwand für verschiedene Roboterkonfigurationen

---

**Hinweis**: Diese Verbesserungen sollten schrittweise und mit ausgiebigen Tests implementiert werden, um die Stabilität des Systems zu gewährleisten.