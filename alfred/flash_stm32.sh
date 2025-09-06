#!/bin/bash

# STM32 Flash Script für Alfred Raspberry Pi
# Flasht die STM32-Firmware über /dev/ttyS0 und OpenOCD

set -e  # Exit bei Fehlern

# Konfiguration
FIRMWARE_DIR="$HOME/sunray/alfred/firmware"
FIRMWARE_BIN="rm18.ino.bin"
OPENOCD_CONFIG_DIR="$HOME/sunray/alfred/config_files/openocd"
OPENOCD_BIN="/usr/local/bin/openocd"

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== STM32 Flash Script für Alfred ===${NC}"
echo -e "${YELLOW}Dieses Script flasht die STM32-Firmware über OpenOCD/SWD${NC}"
echo

# Prüfe ob Script als root läuft
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Fehler: Dieses Script muss als root ausgeführt werden${NC}"
    echo "Verwendung: sudo $0"
    exit 1
fi

# Prüfe ob OpenOCD installiert ist
if [ ! -f "$OPENOCD_BIN" ]; then
    echo -e "${RED}Fehler: OpenOCD nicht gefunden unter $OPENOCD_BIN${NC}"
    echo "Installiere OpenOCD mit: sudo apt-get install openocd"
    exit 1
fi

# Prüfe ob Firmware-Datei existiert
if [ ! -f "$FIRMWARE_DIR/$FIRMWARE_BIN" ]; then
    echo -e "${RED}Fehler: Firmware-Datei nicht gefunden: $FIRMWARE_DIR/$FIRMWARE_BIN${NC}"
    echo "Kompiliere zuerst die STM32-Firmware mit Arduino IDE"
    exit 1
fi

# Prüfe ob OpenOCD-Konfiguration existiert
if [ ! -f "$OPENOCD_CONFIG_DIR/swd-pi.ocd" ]; then
    echo -e "${RED}Fehler: OpenOCD-Konfiguration nicht gefunden: $OPENOCD_CONFIG_DIR/swd-pi.ocd${NC}"
    exit 1
fi

echo -e "${YELLOW}Stoppe Sunray-Service falls aktiv...${NC}"
systemctl stop sunray.service 2>/dev/null || true

echo -e "${YELLOW}Warte 2 Sekunden...${NC}"
sleep 2

echo -e "${YELLOW}Starte Flash-Vorgang...${NC}"
echo "Firmware: $FIRMWARE_DIR/$FIRMWARE_BIN"
echo "OpenOCD Config: $OPENOCD_CONFIG_DIR"
echo

# Erstelle temporäre Flash-Konfiguration
TEMP_FLASH_CONFIG="/tmp/flash_stm32.ocd"
cat > "$TEMP_FLASH_CONFIG" << EOF
# Temporäre Flash-Konfiguration für STM32
source $OPENOCD_CONFIG_DIR/swd-pi.ocd

init
targets
reset halt

echo "----Flashing STM32----"
program $FIRMWARE_DIR/$FIRMWARE_BIN 0x08000000 verify reset

echo "----Flash erfolgreich----"
exit
EOF

# Flash-Vorgang starten
echo -e "${GREEN}Starte OpenOCD Flash-Vorgang...${NC}"
if $OPENOCD_BIN -f "$TEMP_FLASH_CONFIG"; then
    echo -e "${GREEN}✓ STM32-Firmware erfolgreich geflasht!${NC}"
    
    # Aufräumen
    rm -f "$TEMP_FLASH_CONFIG"
    
    echo -e "${YELLOW}Warte 3 Sekunden für STM32-Neustart...${NC}"
    sleep 3
    
    echo -e "${YELLOW}Teste serielle Verbindung zu /dev/ttyS0...${NC}"
    if [ -e "/dev/ttyS0" ]; then
        echo -e "${GREEN}✓ /dev/ttyS0 verfügbar${NC}"
        
        # Teste AT-Kommando
        echo -e "${YELLOW}Teste AT-Kommunikation...${NC}"
        echo "AT+V" > /dev/ttyS0
        sleep 1
        
        # Lese Antwort (timeout nach 2 Sekunden)
        if timeout 2 cat /dev/ttyS0 | head -1; then
            echo -e "${GREEN}✓ STM32 antwortet auf AT-Kommandos${NC}"
        else
            echo -e "${YELLOW}⚠ Keine Antwort auf AT-Kommando (normal nach Flash)${NC}"
        fi
    else
        echo -e "${RED}✗ /dev/ttyS0 nicht verfügbar${NC}"
    fi
    
    echo
    echo -e "${GREEN}=== Flash-Vorgang abgeschlossen ===${NC}"
    echo -e "${YELLOW}Du kannst jetzt den Sunray-Service neu starten:${NC}"
    echo "sudo systemctl start sunray.service"
    
else
    echo -e "${RED}✗ Flash-Vorgang fehlgeschlagen!${NC}"
    rm -f "$TEMP_FLASH_CONFIG"
    exit 1
fi

echo
echo -e "${GREEN}Fertig!${NC}"