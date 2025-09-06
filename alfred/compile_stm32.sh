#!/bin/bash

# STM32 Compile Script für Alfred Raspberry Pi
# Kompiliert die STM32-Firmware (rm18.ino) mit Arduino CLI

set -e  # Exit bei Fehlern

# Konfiguration
FIRMWARE_DIR="$HOME/sunray/firmware"
FIRMWARE_SOURCE="rm18.ino"
ARDUINO_CLI="/usr/local/bin/arduino-cli"
BOARD_FQBN="STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8"
BUILD_DIR="/tmp/arduino_build"

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== STM32 Compile Script für Alfred ===${NC}"
echo -e "${YELLOW}Kompiliert rm18.ino für STM32F103 (Blue Pill)${NC}"
echo

# Prüfe ob Arduino CLI installiert ist
if [ ! -f "$ARDUINO_CLI" ]; then
    echo -e "${RED}Fehler: Arduino CLI nicht gefunden unter $ARDUINO_CLI${NC}"
    echo -e "${YELLOW}Installiere Arduino CLI:${NC}"
    echo "curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo "sudo mv bin/arduino-cli /usr/local/bin/"
    exit 1
fi

# Prüfe ob Firmware-Source existiert
if [ ! -f "$FIRMWARE_DIR/$FIRMWARE_SOURCE" ]; then
    echo -e "${RED}Fehler: Firmware-Source nicht gefunden: $FIRMWARE_DIR/$FIRMWARE_SOURCE${NC}"
    exit 1
fi

# Erstelle Build-Verzeichnis
mkdir -p "$BUILD_DIR"

echo -e "${BLUE}Arduino CLI Version:${NC}"
$ARDUINO_CLI version
echo

# Prüfe ob STM32-Board-Package installiert ist
echo -e "${YELLOW}Prüfe STM32-Board-Package...${NC}"
if ! $ARDUINO_CLI core list | grep -q "STMicroelectronics:stm32"; then
    echo -e "${YELLOW}Installiere STM32-Board-Package...${NC}"
    $ARDUINO_CLI core update-index
    $ARDUINO_CLI core install STMicroelectronics:stm32
else
    echo -e "${GREEN}✓ STM32-Board-Package bereits installiert${NC}"
fi

# Prüfe erforderliche Libraries
echo -e "${YELLOW}Prüfe erforderliche Libraries...${NC}"
REQUIRED_LIBS=("SoftwareSerial")

for lib in "${REQUIRED_LIBS[@]}"; do
    if ! $ARDUINO_CLI lib list | grep -q "$lib"; then
        echo -e "${YELLOW}Installiere Library: $lib${NC}"
        $ARDUINO_CLI lib install "$lib"
    else
        echo -e "${GREEN}✓ Library $lib bereits installiert${NC}"
    fi
done

echo
echo -e "${YELLOW}Starte Kompilierung...${NC}"
echo "Source: $FIRMWARE_DIR/$FIRMWARE_SOURCE"
echo "Board: $BOARD_FQBN"
echo "Build Dir: $BUILD_DIR"
echo

# Kompiliere die Firmware
if $ARDUINO_CLI compile \
    --fqbn "$BOARD_FQBN" \
    --build-path "$BUILD_DIR" \
    --output-dir "$FIRMWARE_DIR" \
    "$FIRMWARE_DIR/$FIRMWARE_SOURCE"; then
    
    echo -e "${GREEN}✓ Kompilierung erfolgreich!${NC}"
    
    # Prüfe ob .bin Datei erstellt wurde
    BIN_FILE="$FIRMWARE_DIR/rm18.ino.bin"
    if [ -f "$BIN_FILE" ]; then
        echo -e "${GREEN}✓ Binary-Datei erstellt: $BIN_FILE${NC}"
        
        # Zeige Datei-Informationen
        echo -e "${BLUE}Datei-Informationen:${NC}"
        ls -lh "$BIN_FILE"
        echo "Größe: $(stat -c%s "$BIN_FILE") Bytes"
        echo "MD5: $(md5sum "$BIN_FILE" | cut -d' ' -f1)"
        
        echo
        echo -e "${GREEN}=== Kompilierung abgeschlossen ===${NC}"
        echo -e "${YELLOW}Du kannst jetzt die Firmware flashen mit:${NC}"
        echo "sudo ./flash_stm32.sh"
        
    else
        echo -e "${RED}✗ Binary-Datei nicht gefunden!${NC}"
        exit 1
    fi
    
else
    echo -e "${RED}✗ Kompilierung fehlgeschlagen!${NC}"
    exit 1
fi

# Aufräumen
echo -e "${YELLOW}Räume temporäre Dateien auf...${NC}"
rm -rf "$BUILD_DIR"

echo
echo -e "${GREEN}Fertig!${NC}"