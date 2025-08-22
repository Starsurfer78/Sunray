#!/bin/bash

# Test-Skript für flash_stm32.sh
# Führt verschiedene Validierungstests durch

set -e

# Farben für Ausgabe
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FLASH_SCRIPT="$SCRIPT_DIR/flash_stm32.sh"
CONFIG_FILE="$SCRIPT_DIR/flash_config.conf"

echo -e "${BLUE}=== Flash-Skript Validierung ===${NC}"
echo

# Test 1: Skript-Existenz
echo -e "${YELLOW}Test 1: Skript-Dateien prüfen...${NC}"
if [ ! -f "$FLASH_SCRIPT" ]; then
    echo -e "${RED}FEHLER: flash_stm32.sh nicht gefunden!${NC}"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}FEHLER: flash_config.conf nicht gefunden!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Alle Dateien vorhanden${NC}"

# Test 2: Syntax-Check
echo -e "${YELLOW}Test 2: Bash-Syntax prüfen...${NC}"
if bash -n "$FLASH_SCRIPT"; then
    echo -e "${GREEN}✓ Syntax korrekt${NC}"
else
    echo -e "${RED}FEHLER: Syntax-Fehler im Skript!${NC}"
    exit 1
fi

# Test 3: Konfiguration laden
echo -e "${YELLOW}Test 3: Konfiguration testen...${NC}"
source "$CONFIG_FILE"

# Wichtige Variablen prüfen
if [ -z "$UART_DEVICE" ]; then
    echo -e "${RED}FEHLER: UART_DEVICE nicht definiert!${NC}"
    exit 1
fi

if [ -z "$BOOT0_PIN" ]; then
    echo -e "${RED}FEHLER: BOOT0_PIN nicht definiert!${NC}"
    exit 1
fi

if [ -z "$RESET_PIN" ]; then
    echo -e "${RED}FEHLER: RESET_PIN nicht definiert!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Konfiguration gültig${NC}"
echo -e "  UART: $UART_DEVICE"
echo -e "  BOOT0: GPIO $BOOT0_PIN"
echo -e "  RESET: GPIO $RESET_PIN"

# Test 4: Abhängigkeiten prüfen (optional)
echo -e "${YELLOW}Test 4: Abhängigkeiten prüfen...${NC}"

# GPIO-System
if [ ! -d "/sys/class/gpio" ]; then
    echo -e "${YELLOW}⚠ GPIO-System nicht verfügbar (normal auf Desktop)${NC}"
else
    echo -e "${GREEN}✓ GPIO-System verfügbar${NC}"
fi

# stm32flash
if command -v stm32flash &> /dev/null; then
    echo -e "${GREEN}✓ stm32flash verfügbar${NC}"
else
    echo -e "${YELLOW}⚠ stm32flash nicht installiert${NC}"
    echo -e "  Installiere mit: sudo apt-get install stm32flash"
fi

# arduino-cli (optional)
if command -v arduino-cli &> /dev/null; then
    echo -e "${GREEN}✓ arduino-cli verfügbar${NC}"
else
    echo -e "${YELLOW}⚠ arduino-cli nicht installiert (optional)${NC}"
fi

# Test 5: Firmware-Datei prüfen
echo -e "${YELLOW}Test 5: Firmware-Datei prüfen...${NC}"
FIRMWARE_PATH="$SCRIPT_DIR/../firmware/rm18.ino"
if [ -f "$FIRMWARE_PATH" ]; then
    echo -e "${GREEN}✓ Firmware-Datei gefunden: $FIRMWARE_PATH${NC}"
    
    # Dateigröße
    SIZE=$(stat -c%s "$FIRMWARE_PATH" 2>/dev/null || echo "0")
    echo -e "  Größe: $SIZE Bytes"
else
    echo -e "${YELLOW}⚠ Firmware-Datei nicht gefunden: $FIRMWARE_PATH${NC}"
fi

# Test 6: Backup-Verzeichnis
echo -e "${YELLOW}Test 6: Backup-Verzeichnis prüfen...${NC}"
if [ "$CREATE_BACKUP" = "1" ]; then
    if [ ! -d "$BACKUP_DIR" ]; then
        echo -e "${YELLOW}⚠ Backup-Verzeichnis wird erstellt: $BACKUP_DIR${NC}"
        mkdir -p "$BACKUP_DIR" || echo -e "${RED}FEHLER: Kann Backup-Verzeichnis nicht erstellen${NC}"
    else
        echo -e "${GREEN}✓ Backup-Verzeichnis vorhanden: $BACKUP_DIR${NC}"
    fi
else
    echo -e "${BLUE}ℹ Backup deaktiviert${NC}"
fi

echo
echo -e "${GREEN}=== Validierung abgeschlossen ===${NC}"
echo -e "${BLUE}Das Flash-Skript ist bereit zur Verwendung!${NC}"
echo
echo -e "${YELLOW}Verwendung:${NC}"
echo -e "  ./flash_stm32.sh                    # Standard-Flash"
echo -e "  ./flash_stm32.sh -v                 # Verbose-Modus"
echo -e "  ./flash_stm32.sh -c custom.conf     # Eigene Konfiguration"
echo -e "  ./flash_stm32.sh -h                 # Hilfe anzeigen"