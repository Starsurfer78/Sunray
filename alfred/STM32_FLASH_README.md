# STM32 Flash-Scripts für Alfred Raspberry Pi

Diese Scripts ermöglichen das Kompilieren und Flashen der STM32-Firmware (rm18.ino) über den Raspberry Pi via /dev/ttyS0 und OpenOCD.

## 📋 Übersicht

> **Hinweis:** Alle Scripts verwenden automatisch den aktuellen Benutzer (`$HOME`) anstatt hardcodierte Pfade. Funktioniert mit jedem Benutzernamen (pi, sascha, etc.).

### Scripts

1. **`stm32_manager.sh`** - Hauptscript mit Menü-Interface (empfohlen)
2. **`compile_stm32.sh`** - Kompiliert rm18.ino zu .bin-Datei
3. **`flash_stm32.sh`** - Flasht .bin-Datei via OpenOCD/SWD

### Funktionen

- ✅ Automatische Kompilierung der STM32-Firmware
- ✅ Flash-Vorgang über OpenOCD und SWD-Interface
- ✅ AT-Kommunikationstest (19200 Baud)
- ✅ System-Status-Überwachung
- ✅ Sunray-Service-Management
- ✅ Umfassende Fehlerbehandlung und Logging

## 🚀 Schnellstart

### 1. Einfache Verwendung (empfohlen)

```bash
# Hauptscript mit Menü starten
sudo ./stm32_manager.sh
```

### 2. Einzelne Scripts

```bash
# Nur kompilieren
./compile_stm32.sh

# Nur flashen (benötigt Root-Rechte)
sudo ./flash_stm32.sh

# Kompilieren + Flashen
./compile_stm32.sh && sudo ./flash_stm32.sh
```

## 📦 Voraussetzungen

### Hardware

- Alfred Raspberry Pi mit STM32-Verbindung
- SWD-Verbindung (SWDIO, SWCLK, GND)
- Serielle Verbindung über /dev/ttyS0 @ 19200 Baud

### Software

```bash
# Arduino CLI installieren
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

# OpenOCD installieren
sudo apt-get update
sudo apt-get install openocd

# STM32-Board-Package (wird automatisch installiert)
arduino-cli core install STMicroelectronics:stm32
```

## 🔧 Konfiguration

### Wichtige Pfade

```bash
# Firmware-Verzeichnis
FIRMWARE_DIR="$HOME/sunray/firmware"

# Source-Code
FIRMWARE_SOURCE="rm18.ino"

# Kompilierte Binary
FIRMWARE_BIN="rm18.ino.bin"

# Serieller Port
SERIAL_PORT="/dev/ttyS0"
BAUDRATE="19200"
```

### OpenOCD-Konfiguration

Die Scripts verwenden die vorhandenen OpenOCD-Konfigurationsdateien:

- `config_files/openocd/swd-pi.ocd` - SWD-Interface-Konfiguration
- `config_files/openocd/flash-sunray-compat.ocd` - Flash-Konfiguration

## 📊 System-Status

Der `stm32_manager.sh` zeigt automatisch den System-Status an:

```
=== System-Status ===
✓ Sunray-Service: Aktiv
✓ Serieller Port: /dev/ttyS0 verfügbar
✓ Firmware-Binary: Vorhanden
  Größe: 15234 Bytes
  Datum: 2024-01-15
✓ Firmware-Source: Vorhanden
```

## 🧪 AT-Kommunikationstest

Teste die Verbindung zum STM32:

```bash
# Über Manager-Script (Option 4)
sudo ./stm32_manager.sh

# Manuell
echo "AT+V" > /dev/ttyS0
cat /dev/ttyS0
```

### Erwartete AT-Kommandos

- `AT+V` - Version abfragen
- `AT+S` - Status abfragen  
- `AT+M` - Motor-Status abfragen

## 🔍 Troubleshooting

### Problem: CRC-Fehler in der Kommunikation

**Ursachen:**
- Falsche Baudrate (sollte 19200 sein)
- Timing-Probleme
- Defekte Hardware-Verbindung

**Lösungen:**
```bash
# Baudrate prüfen
stty -F /dev/ttyS0

# Baudrate setzen
stty -F /dev/ttyS0 19200 cs8 -cstopb -parenb raw

# AT-Test durchführen
sudo ./stm32_manager.sh  # Option 4
```

### Problem: STM32 antwortet nicht

**Ursachen:**
- STM32 nicht geflasht
- Hardware-Verbindungsproblem
- Stromversorgung

**Lösungen:**
```bash
# Hardware prüfen
# - SWDIO (GPIO24)
# - SWCLK (GPIO25) 
# - GND
# - 3.3V Stromversorgung

# Firmware neu flashen
sudo ./stm32_manager.sh  # Option 2
```

### Problem: Kompilierung schlägt fehl

**Ursachen:**
- Arduino CLI nicht installiert
- STM32-Board-Package fehlt
- Syntax-Fehler in rm18.ino

**Lösungen:**
```bash
# Arduino CLI installieren
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

# Board-Package installieren
arduino-cli core update-index
arduino-cli core install STMicroelectronics:stm32

# Syntax prüfen
arduino-cli compile --verify-only $HOME/sunray/firmware/rm18.ino
```

### Problem: Flash-Vorgang schlägt fehl

**Ursachen:**
- OpenOCD nicht installiert
- SWD-Verbindung defekt
- Keine Root-Rechte

**Lösungen:**
```bash
# OpenOCD installieren
sudo apt-get install openocd

# Als Root ausführen
sudo ./flash_stm32.sh

# SWD-Verbindung testen
openocd -f config_files/openocd/swd-pi.ocd -c "init; targets; exit"
```

## 📝 Logging

Alle Aktivitäten werden geloggt:

```bash
# Log-Datei anzeigen
tail -f /tmp/stm32_manager.log

# Über Manager-Script (Option 7)
sudo ./stm32_manager.sh
```

## 🔄 Workflow-Beispiele

### Kompletter Neuaufbau

```bash
# 1. Sunray-Service stoppen
sudo systemctl stop sunray.service

# 2. Firmware kompilieren und flashen
sudo ./stm32_manager.sh  # Option 3

# 3. AT-Kommunikation testen
sudo ./stm32_manager.sh  # Option 4

# 4. Sunray-Service starten
sudo systemctl start sunray.service
```

### Nur Firmware-Update

```bash
# 1. Neue rm18.ino bereitstellen
cp neue_rm18.ino $HOME/sunray/firmware/rm18.ino

# 2. Kompilieren und flashen
sudo ./stm32_manager.sh  # Option 3

# 3. Service neu starten
sudo systemctl restart sunray.service
```

## ⚙️ Erweiterte Konfiguration

### Baudrate ändern

Falls eine andere Baudrate benötigt wird:

1. **In rm18.ino:**
   ```cpp
   #define CONSOLE_BAUDRATE 19200  // Neue Baudrate
   ```

2. **In config_alfred.h:**
   ```cpp
   #define ROBOT_BAUDRATE 19200    // Gleiche Baudrate
   ```

3. **In den Scripts:**
   ```bash
   # flash_stm32.sh und stm32_manager.sh
   stty -F /dev/ttyS0 19200  # Neue Baudrate
   ```

### OpenOCD-Konfiguration anpassen

Für andere GPIO-Pins oder Hardware:

```bash
# config_files/openocd/swd-pi.ocd bearbeiten
sysfsgpio_swdio_num 24   # GPIO für SWDIO
sysfsgpio_swclk_num 25   # GPIO für SWCLK
sysfsgpio_srst_num 23    # GPIO für Reset
```

## 📞 Support

Bei Problemen:

1. **Log-Datei prüfen:** `/tmp/stm32_manager.log`
2. **System-Status prüfen:** `sudo ./stm32_manager.sh` (Option 5)
3. **AT-Test durchführen:** `sudo ./stm32_manager.sh` (Option 4)
4. **Hilfe anzeigen:** `sudo ./stm32_manager.sh` (Option 8)

## 📄 Lizenz

Diese Scripts sind Teil des Sunray-Projekts und unterliegen der gleichen Lizenz.

---

**Erstellt für Alfred Mähroboter - STM32 Firmware Management**

*Letzte Aktualisierung: Januar 2024*