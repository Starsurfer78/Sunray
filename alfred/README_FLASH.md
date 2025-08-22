# STM32 Flash Setup für Raspberry Pi

Dieses Dokument beschreibt die Einrichtung und Verwendung des Flash-Scripts für den STM32 über UART.

## Hardware-Verbindungen

### Raspberry Pi zu STM32 Verbindungen:

| Raspberry Pi | STM32 (Blue Pill) | Funktion |
|--------------|-------------------|----------|
| GPIO 14 (TXD)| PA10 (RX)        | UART TX  |
| GPIO 15 (RXD)| PA9 (TX)         | UART RX  |
| GPIO 18      | BOOT0            | Boot Mode|
| GPIO 23      | RESET            | Reset    |
| 3.3V         | 3.3V             | Power    |
| GND          | GND              | Ground   |

### Wichtige Hinweise:
- **3.3V verwenden!** STM32 ist nicht 5V tolerant
- UART Kreuzverbindung: Pi TX → STM32 RX, Pi RX → STM32 TX
- Pull-up Widerstände (10kΩ) für BOOT0 und RESET empfohlen

## Software-Installation

### 1. UART aktivieren

Bearbeite `/boot/config.txt`:
```bash
sudo nano /boot/config.txt
```

Füge hinzu:
```
# UART aktivieren
enable_uart=1
dtoverlay=disable-bt
```

Bearbeite `/boot/cmdline.txt` und entferne:
```
console=serial0,115200
```

### 2. stm32flash installieren

```bash
sudo apt-get update
sudo apt-get install stm32flash
```

### 3. Arduino CLI installieren (optional)

```bash
# Download Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# STM32 Board Package installieren
arduino-cli core update-index
arduino-cli core install STMicroelectronics:stm32
```

### 4. Flash Script ausführbar machen

```bash
chmod +x /home/pi/sunray/alfred/flash_stm32.sh
```

## Verwendung

### Firmware flashen:

```bash
sudo ./flash_stm32.sh
```

### Script-Parameter anpassen:

Bearbeite die Konfiguration im Script:
```bash
# GPIO Pins (BCM Nummerierung)
BOOT0_PIN="18"              # GPIO Pin für BOOT0
RESET_PIN="23"              # GPIO Pin für RESET

# UART Einstellungen
UART_DEVICE="/dev/ttyAMA0"  # Raspberry Pi UART
BAUD_RATE="115200"

# Pfade
FIRMWARE_DIR="/home/pi/sunray/alfred/firmware"
FIRMWARE_FILE="rm18.ino"
```

## Troubleshooting

### Häufige Probleme

1. **Permission denied auf UART**
   ```bash
   sudo usermod -a -G dialout $USER
   # Neuanmeldung erforderlich
   ```

2. **GPIO bereits in Verwendung**
   ```bash
   # GPIO Pins freigeben
   echo 18 > /sys/class/gpio/unexport
   echo 23 > /sys/class/gpio/unexport
   ```

3. **STM32 antwortet nicht**
   - Verkabelung prüfen
   - BOOT0 auf 3.3V beim Flash-Vorgang
   - Reset-Signal korrekt angeschlossen
   - Spannungsversorgung prüfen (3.3V)

4. **Flash schlägt fehl**
   - Baudrate reduzieren (9600)
   - Längere Delays zwischen Reset/Boot
   - Andere UART-Schnittstelle testen
   - STM32 Chip-ID prüfen: `stm32flash /dev/ttyAMA0`

5. **Kompilierung schlägt fehl**
   - Arduino-CLI korrekt installiert?
   - STM32 Board-Package installiert?
   - Alle Bibliotheken verfügbar?

6. **Backup schlägt fehl**
   - Ausreichend Speicherplatz?
   - Schreibrechte im Backup-Verzeichnis?
   - STM32 im Bootloader-Modus?

### Debug-Modus

Für detaillierte Diagnose:
```bash
./flash_stm32.sh -v  # Verbose-Modus
```

### Log-Dateien

Logs werden gespeichert in:
- Flash-Log: `/tmp/stm32_flash.log`
- Backup-Log: `$BACKUP_DIR/backup.log`

### Manuelle Diagnose

```bash
# GPIO Status prüfen
ls -la /sys/class/gpio/

# UART testen
echo "AT" > /dev/ttyAMA0
cat /dev/ttyAMA0

# STM32 Chip-Info
stm32flash /dev/ttyAMA0

# Konfiguration validieren
./test_flash.sh
```

## Manueller Flash-Prozess

Falls das Script nicht funktioniert:

### 1. STM32 in Bootloader Modus:
```bash
# BOOT0 = HIGH
echo 18 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio18/direction
echo 1 > /sys/class/gpio/gpio18/value

# Reset
echo 23 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio23/direction
echo 0 > /sys/class/gpio/gpio23/value
sleep 0.1
echo 1 > /sys/class/gpio/gpio23/value
```

### 2. Firmware flashen:
```bash
stm32flash -w rm18.ino.hex -v -g 0x0 /dev/ttyAMA0 -b 115200
```

### 3. STM32 in Normal Modus:
```bash
# BOOT0 = LOW
echo 0 > /sys/class/gpio/gpio18/value

# Reset
echo 0 > /sys/class/gpio/gpio23/value
sleep 0.1
echo 1 > /sys/class/gpio/gpio23/value
```

## Automatisierung

### Systemd Service für automatisches Flashen:

Erstelle `/etc/systemd/system/stm32-flash.service`:
```ini
[Unit]
Description=STM32 Flash Service
After=network.target

[Service]
Type=oneshot
User=root
WorkingDirectory=/home/pi/sunray/alfred
ExecStart=/home/pi/sunray/alfred/flash_stm32.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Aktivieren:
```bash
sudo systemctl enable stm32-flash.service
sudo systemctl start stm32-flash.service
```

### Cron Job für regelmäßiges Update:

```bash
sudo crontab -e
```

Füge hinzu:
```
# Täglich um 3:00 Uhr STM32 neu flashen
0 3 * * * /home/pi/sunray/alfred/flash_stm32.sh > /var/log/stm32-flash.log 2>&1
```

## Sicherheitshinweise

- **Niemals 5V an STM32 anlegen!**
- Script nur mit sudo ausführen
- Vor Flash-Vorgang sicherstellen, dass Roboter gestoppt ist
- Bei Problemen sofort Stromversorgung trennen
- Backup der funktionierenden Firmware aufbewahren

## Log-Dateien

- Script Output: `/var/log/stm32-flash.log`
- System Log: `journalctl -u stm32-flash.service`
- UART Debug: `sudo minicom -D /dev/ttyAMA0 -b 115200`