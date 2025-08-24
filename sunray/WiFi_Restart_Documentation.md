# WiFi Restart Funktionalität

## Übersicht

Die WiFi Restart Funktionalität bietet automatische Wiederverbindung bei WiFi-Verbindungsproblemen und manuelle Kontrolle über AT+ Befehle.

## Konfiguration

### config.h Einstellungen

```cpp
// WiFi Restart Configuration
// #define ENABLE_SIMPLE_WIFI_RESTART    // Uncomment to enable WiFi restart functionality
#define WIFI_CHECK_INTERVAL_MS 7000     // WiFi status check interval in milliseconds
#define WIFI_MAX_FAILURES 3             // Maximum consecutive failures before restart
#define WIFI_RESTART_DELAY_MS 5000      // Delay after WiFi restart in milliseconds
```

### Aktivierung

Um die WiFi Restart Funktionalität zu aktivieren, entkommentieren Sie die Zeile:
```cpp
#define ENABLE_SIMPLE_WIFI_RESTART
```

## Funktionsweise

### Automatische Überwachung

- **Intervall**: WiFi-Status wird alle `WIFI_CHECK_INTERVAL_MS` Millisekunden überprüft
- **Fehlerzählung**: Bei Verbindungsfehlern wird ein Zähler erhöht
- **Automatischer Restart**: Nach `WIFI_MAX_FAILURES` aufeinanderfolgenden Fehlern wird WiFi neu gestartet
- **Verzögerung**: Nach einem Restart wartet das System `WIFI_RESTART_DELAY_MS` Millisekunden

### WiFi Restart Prozess

1. WiFi Interface deaktivieren (`sudo ifdown wlan0`)
2. 2 Sekunden warten
3. WiFi Interface aktivieren (`sudo ifup wlan0`)
4. Konfigurierte Verzögerung abwarten

## AT+ Befehle

### AT+WIFI_RESTART

**Beschreibung**: Erzwingt einen sofortigen WiFi-Neustart

**Syntax**: `AT+WIFI_RESTART`

**Antwort**: `WIFI_RESTART,OK`

**Beispiel**:
```
> AT+WIFI_RESTART
< WIFI_RESTART,OK
```

### AT+WIFI_STATUS

**Beschreibung**: Gibt den aktuellen WiFi-Status und die Anzahl der Fehler zurück

**Syntax**: `AT+WIFI_STATUS`

**Antwort**: `WIFI_STATUS,<STATUS>,<FAILURE_COUNT>`

- `<STATUS>`: `CONNECTED` oder `DISCONNECTED`
- `<FAILURE_COUNT>`: Anzahl der aufeinanderfolgenden Verbindungsfehler

**Beispiel**:
```
> AT+WIFI_STATUS
< WIFI_STATUS,CONNECTED,0
```

### AT+WIFI_CONFIG

**Beschreibung**: Konfiguriert die WiFi Restart Parameter zur Laufzeit

**Syntax**: `AT+WIFI_CONFIG,<INTERVAL>,<MAX_FAILURES>,<RESTART_DELAY>`

- `<INTERVAL>`: Überprüfungsintervall in Millisekunden
- `<MAX_FAILURES>`: Maximale Anzahl von Fehlern vor Restart
- `<RESTART_DELAY>`: Verzögerung nach Restart in Millisekunden

**Antwort**: `WIFI_CONFIG,OK` oder `WIFI_CONFIG,ERROR`

**Beispiel**:
```
> AT+WIFI_CONFIG,5000,5,3000
< WIFI_CONFIG,OK
```

## Integration

### SerialRobotDriver

Die WiFi Restart Funktionalität ist in `SerialRobotDriver` integriert:

- **Initialisierung**: In `begin()` Methode
- **Überwachung**: In `updateWifiConnectionState()` Methode
- **AT+ Befehle**: In `processResponse()` Methode

### Abhängigkeiten

- **Linux**: Funktionalität ist nur unter Linux verfügbar
- **wpa_cli**: Für WiFi-Status-Abfrage
- **ifdown/ifup**: Für WiFi-Neustart (benötigt sudo-Rechte)

## Sicherheitshinweise

- Die WiFi Restart Funktionalität benötigt sudo-Rechte für `ifdown` und `ifup` Befehle
- Bei häufigen WiFi-Problemen sollten die Netzwerkeinstellungen überprüft werden
- Die Funktionalität ist standardmäßig deaktiviert und muss explizit aktiviert werden

## Kompatibilität

- **Arduino Due**: Vollständig kompatibel (Build erfolgreich)
- **Bestehende AT+ Befehle**: Keine Beeinträchtigung der vorhandenen Funktionalität
- **Rückwärtskompatibilität**: Vollständig gewährleistet

## Troubleshooting

### WiFi Restart funktioniert nicht

1. Überprüfen Sie, ob `ENABLE_SIMPLE_WIFI_RESTART` definiert ist
2. Stellen Sie sicher, dass sudo-Rechte für ifdown/ifup verfügbar sind
3. Überprüfen Sie die Netzwerkschnittstelle (Standard: wlan0)

### AT+ Befehle antworten nicht

1. Überprüfen Sie die Befehlssyntax
2. Stellen Sie sicher, dass die WiFi Restart Funktionalität aktiviert ist
3. Überprüfen Sie die serielle Verbindung

## Logs

Bei aktivierter WiFi Restart Funktionalität werden folgende Meldungen ausgegeben:

- `"WiFi restart functionality enabled"` - Bei der Initialisierung
- WiFi-Status-Updates über die normale Logging-Funktionalität