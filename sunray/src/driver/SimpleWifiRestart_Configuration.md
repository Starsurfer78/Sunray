# SimpleWifiRestart Konfiguration

Die SimpleWifiRestart-Funktionalität bietet jetzt konfigurierbare Parameter für eine flexible WiFi-Überwachung und -Wiederherstellung.

## Konfigurierbare Parameter in config.h

### WIFI_RESTART_CHECK_INTERVAL
- **Standard:** 30000 (30 Sekunden)
- **Beschreibung:** Intervall in Millisekunden zwischen WiFi-Statusprüfungen
- **Empfehlung:** 30-60 Sekunden für normale Nutzung

### WIFI_RESTART_MAX_FAILURES
- **Standard:** 3
- **Beschreibung:** Maximale Anzahl aufeinanderfolgender Verbindungsfehler vor einem Neustart
- **Empfehlung:** 2-5 Versuche je nach Netzwerkstabilität

### WIFI_RESTART_DELAY
- **Standard:** 10000 (10 Sekunden)
- **Beschreibung:** Wartezeit in Millisekunden nach einem WiFi-Neustart
- **Empfehlung:** 5-15 Sekunden je nach Hardware

## Kompatibilität mit Alfred

Die Parameter sind kompatibel mit den entsprechenden Einstellungen in `alfred/config_alfred.h`:
- `WIFI_RESTART_DELAY` entspricht dem Alfred-Parameter
- `WIFI_RESTART_MAX_FAILURES` entspricht `WIFI_RESTART_MAX_RETRIES`

## AT+ Befehle

### AT+WIFI_STATUS
Zeigt den aktuellen WiFi-Status an:
- `CONNECTED` - WiFi ist verbunden
- `DISCONNECTED` - WiFi ist getrennt
- `RESTARTING` - WiFi wird neu gestartet
- `FAILED` - WiFi-Neustart fehlgeschlagen

### AT+WIFI_RESTART
Erzwingt einen sofortigen WiFi-Neustart

### AT+WIFI_CONFIG
Zeigt die aktuellen Konfigurationsparameter an:
```
WIFI_CONFIG:timeout=3,interval=30000
```

## Aktivierung

Um die SimpleWifiRestart-Funktionalität zu aktivieren, entkommentieren Sie in `config.h`:
```cpp
#define ENABLE_SIMPLE_WIFI_RESTART true
```

## Anpassung der Parameter

Beispiel für eine konservative Konfiguration:
```cpp
#define WIFI_RESTART_CHECK_INTERVAL 60000  // 60 Sekunden
#define WIFI_RESTART_MAX_FAILURES 5       // 5 Versuche
#define WIFI_RESTART_DELAY 15000          // 15 Sekunden Wartezeit
```

Beispiel für eine aggressive Konfiguration:
```cpp
#define WIFI_RESTART_CHECK_INTERVAL 15000  // 15 Sekunden
#define WIFI_RESTART_MAX_FAILURES 2       // 2 Versuche
#define WIFI_RESTART_DELAY 5000           // 5 Sekunden Wartezeit
```