#!/bin/bash

# STM32 Flash Script für Raspberry Pi
# Flasht rm18.ino über UART auf STM32
# Autor: Sunray Project
# Datum: $(date +%Y-%m-%d)

set -e  # Exit bei Fehlern

# Standard Konfiguration (wird von flash_config.conf überschrieben)
UART_DEVICE="/dev/ttyAMA0"
BAUD_RATE="115200"
BOOT0_PIN="18"
RESET_PIN="23"
FIRMWARE_DIR="/home/pi/sunray/alfred/firmware"
FIRMWARE_FILE="rm18.ino"
BUILD_DIR="$FIRMWARE_DIR/build"
STM32_TARGET="STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8"
FLASH_ADDRESS="0x08000000"
VERBOSE=1
CREATE_BACKUP=1
BACKUP_DIR="$FIRMWARE_DIR/backups"
MAX_BACKUPS=5
VERIFY_FLASH=1
AUTO_COMPILE=1
AUTO_RESET=1
WAIT_FOR_BOOT=2
RESET_DELAY=0.1
BOOT_DELAY=0.5

# Konfigurationsdatei laden
CONFIG_FILE="$(dirname "$0")/flash_config.conf"
if [ -f "$CONFIG_FILE" ]; then
    echo -e "${BLUE}Lade Konfiguration: $CONFIG_FILE${NC}"
    source "$CONFIG_FILE"
fi

# Abgeleitete Variablen
HEX_FILE="$BUILD_DIR/$FIRMWARE_FILE.hex"

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== STM32 Flash Script ===${NC}"
echo -e "${BLUE}Firmware: $FIRMWARE_FILE${NC}"
echo -e "${BLUE}UART: $UART_DEVICE @ $BAUD_RATE${NC}"
echo

# Funktion: GPIO Setup
setup_gpio() {
    echo -e "${YELLOW}GPIO Setup...${NC}"
    
    # GPIO Pins exportieren falls nicht vorhanden
    if [ ! -d "/sys/class/gpio/gpio$BOOT0_PIN" ]; then
        echo $BOOT0_PIN > /sys/class/gpio/export
        sleep 0.1
    fi
    
    if [ ! -d "/sys/class/gpio/gpio$RESET_PIN" ]; then
        echo $RESET_PIN > /sys/class/gpio/export
        sleep 0.1
    fi
    
    # GPIO Pins als Output konfigurieren
    echo out > /sys/class/gpio/gpio$BOOT0_PIN/direction
    echo out > /sys/class/gpio/gpio$RESET_PIN/direction
    
    echo -e "${GREEN}GPIO Setup abgeschlossen${NC}"
}

# Funktion: STM32 in Bootloader Modus versetzen
enter_bootloader() {
    echo -e "${YELLOW}STM32 in Bootloader Modus versetzen...${NC}"
    
    # BOOT0 = HIGH (Bootloader Modus)
    echo 1 > /sys/class/gpio/gpio$BOOT0_PIN/value
    sleep 0.1
    
    # RESET = LOW (Reset aktiv)
    echo 0 > /sys/class/gpio/gpio$RESET_PIN/value
    sleep 0.1
    
    # RESET = HIGH (Reset freigeben)
    echo 1 > /sys/class/gpio/gpio$RESET_PIN/value
    sleep 0.5
    
    echo -e "${GREEN}STM32 im Bootloader Modus${NC}"
}

# Funktion: STM32 in Normal Modus versetzen
exit_bootloader() {
    echo -e "${YELLOW}STM32 in Normal Modus versetzen...${NC}"
    
    # BOOT0 = LOW (Normal Modus)
    echo 0 > /sys/class/gpio/gpio$BOOT0_PIN/value
    sleep 0.1
    
    # RESET = LOW (Reset aktiv)
    echo 0 > /sys/class/gpio/gpio$RESET_PIN/value
    sleep 0.1
    
    # RESET = HIGH (Reset freigeben)
    echo 1 > /sys/class/gpio/gpio$RESET_PIN/value
    sleep 0.5
    
    echo -e "${GREEN}STM32 im Normal Modus${NC}"
}

# Funktion: Backup erstellen
create_backup() {
    if [ "$CREATE_BACKUP" = "1" ]; then
        echo -e "${YELLOW}Erstelle Backup...${NC}"
        
        mkdir -p "$BACKUP_DIR"
        
        # Aktuelles Datum für Backup-Namen
        BACKUP_NAME="rm18_backup_$(date +%Y%m%d_%H%M%S).hex"
        BACKUP_PATH="$BACKUP_DIR/$BACKUP_NAME"
        
        # Versuche aktuellen Flash-Inhalt zu lesen
        if stm32flash -r "$BACKUP_PATH" "$UART_DEVICE" -b $BAUD_RATE 2>/dev/null; then
            echo -e "${GREEN}Backup erstellt: $BACKUP_NAME${NC}"
            
            # Alte Backups löschen (nur die neuesten behalten)
            if [ "$MAX_BACKUPS" -gt 0 ]; then
                cd "$BACKUP_DIR"
                ls -t rm18_backup_*.hex 2>/dev/null | tail -n +$((MAX_BACKUPS + 1)) | xargs rm -f 2>/dev/null || true
            fi
        else
            echo -e "${YELLOW}Backup konnte nicht erstellt werden (STM32 nicht im Bootloader?)${NC}"
        fi
    fi
}

# Funktion: Firmware kompilieren
compile_firmware() {
    if [ "$AUTO_COMPILE" = "1" ]; then
        echo -e "${YELLOW}Firmware kompilieren...${NC}"
        
        cd "$FIRMWARE_DIR"
        
        # Build Verzeichnis erstellen
        mkdir -p "$BUILD_DIR"
        
        # Arduino CLI verwenden (falls installiert)
        if command -v arduino-cli &> /dev/null; then
            echo -e "${BLUE}Verwende Arduino CLI...${NC}"
            
            # Verbose Output falls aktiviert
            COMPILE_ARGS="--fqbn $STM32_TARGET --output-dir $BUILD_DIR"
            if [ "$VERBOSE" = "1" ]; then
                COMPILE_ARGS="$COMPILE_ARGS --verbose"
            fi
            
            arduino-cli compile $COMPILE_ARGS "$FIRMWARE_FILE"
        else
            echo -e "${RED}Arduino CLI nicht gefunden!${NC}"
            echo -e "${YELLOW}Installiere Arduino CLI oder kompiliere manuell${NC}"
            exit 1
        fi
        
        # Prüfen ob HEX Datei existiert
        if [ ! -f "$HEX_FILE" ]; then
            echo -e "${RED}HEX Datei nicht gefunden: $HEX_FILE${NC}"
            exit 1
        fi
        
        # Dateigröße anzeigen
        HEX_SIZE=$(stat -c%s "$HEX_FILE")
        echo -e "${GREEN}Kompilierung erfolgreich (${HEX_SIZE} Bytes)${NC}"
    else
        echo -e "${YELLOW}Auto-Kompilierung deaktiviert, verwende vorhandene HEX Datei${NC}"
        
        if [ ! -f "$HEX_FILE" ]; then
            echo -e "${RED}HEX Datei nicht gefunden: $HEX_FILE${NC}"
            echo -e "${YELLOW}Aktiviere AUTO_COMPILE oder kompiliere manuell${NC}"
            exit 1
        fi
    fi
}

# Funktion: Firmware flashen
flash_firmware() {
    echo -e "${YELLOW}Firmware flashen...${NC}"
    
    # stm32flash prüfen
    if ! command -v stm32flash &> /dev/null; then
        echo -e "${RED}stm32flash nicht gefunden!${NC}"
        echo -e "${YELLOW}Installiere mit: sudo apt-get install stm32flash${NC}"
        exit 1
    fi
    
    # STM32 in Bootloader Modus
    enter_bootloader
    
    # Backup erstellen (falls aktiviert)
    create_backup
    
    # Flash Vorgang
    echo -e "${BLUE}Starte Flash Vorgang...${NC}"
    
    # Flash-Argumente zusammenstellen
    FLASH_ARGS="-w $HEX_FILE -g $FLASH_ADDRESS $UART_DEVICE -b $BAUD_RATE"
    
    if [ "$VERIFY_FLASH" = "1" ]; then
        FLASH_ARGS="-v $FLASH_ARGS"
        echo -e "${BLUE}Verifikation aktiviert${NC}"
    fi
    
    if [ "$VERBOSE" = "1" ]; then
        echo -e "${BLUE}Flash Kommando: stm32flash $FLASH_ARGS${NC}"
    fi
    
    # Chip löschen und flashen
    if stm32flash $FLASH_ARGS; then
        echo -e "${GREEN}Flash erfolgreich!${NC}"
        
        # Flash-Größe anzeigen
        if [ "$VERBOSE" = "1" ]; then
            FLASH_SIZE=$(stat -c%s "$HEX_FILE")
            echo -e "${BLUE}Geflashte Größe: ${FLASH_SIZE} Bytes${NC}"
        fi
    else
        echo -e "${RED}Flash fehlgeschlagen!${NC}"
        exit_bootloader
        exit 1
    fi
    
    # STM32 in Normal Modus (falls AUTO_RESET aktiviert)
    if [ "$AUTO_RESET" = "1" ]; then
        exit_bootloader
        
        # Warten auf Boot
        if [ "$WAIT_FOR_BOOT" -gt 0 ]; then
            echo -e "${YELLOW}Warte ${WAIT_FOR_BOOT}s auf STM32 Boot...${NC}"
            sleep "$WAIT_FOR_BOOT"
        fi
    else
        echo -e "${YELLOW}Auto-Reset deaktiviert. STM32 bleibt im Bootloader Modus.${NC}"
    fi
}

# Funktion: Kommunikationstest
test_communication() {
    echo -e "${YELLOW}Teste Kommunikation mit STM32...${NC}"
    
    # Kurz warten bis STM32 bereit ist
    sleep 1
    
    # Versuche AT-Kommando zu senden
    if command -v timeout &> /dev/null; then
        # Mit timeout (falls verfügbar)
        RESPONSE=$(timeout 5 bash -c "echo 'AT' > $UART_DEVICE && head -n 1 < $UART_DEVICE" 2>/dev/null || echo "")
    else
        # Ohne timeout (einfacher Test)
        echo "AT" > "$UART_DEVICE" 2>/dev/null || true
        sleep 0.5
        RESPONSE=$(head -c 10 < "$UART_DEVICE" 2>/dev/null || echo "")
    fi
    
    if [ -n "$RESPONSE" ]; then
        echo -e "${GREEN}Kommunikation erfolgreich: $RESPONSE${NC}"
    else
        echo -e "${YELLOW}Keine Antwort vom STM32 (normal bei erstem Boot)${NC}"
    fi
}

# Funktion: Cleanup
cleanup() {
    echo -e "${YELLOW}Cleanup...${NC}"
    
    # GPIO Pins freigeben
    if [ -d "/sys/class/gpio/gpio$BOOT0_PIN" ]; then
        echo $BOOT0_PIN > /sys/class/gpio/unexport 2>/dev/null || true
    fi
    
    if [ -d "/sys/class/gpio/gpio$RESET_PIN" ]; then
        echo $RESET_PIN > /sys/class/gpio/unexport 2>/dev/null || true
    fi
}

# Trap für Cleanup bei Exit
trap cleanup EXIT

# Hauptprogramm
main() {
    # Root Rechte prüfen
    if [ "$EUID" -ne 0 ]; then
        echo -e "${RED}Dieses Script benötigt Root Rechte!${NC}"
        echo -e "${YELLOW}Starte mit: sudo $0${NC}"
        exit 1
    fi
    
    # Firmware Verzeichnis prüfen
    if [ ! -d "$FIRMWARE_DIR" ]; then
        echo -e "${RED}Firmware Verzeichnis nicht gefunden: $FIRMWARE_DIR${NC}"
        exit 1
    fi
    
    # Firmware Datei prüfen
    if [ ! -f "$FIRMWARE_DIR/$FIRMWARE_FILE" ]; then
        echo -e "${RED}Firmware Datei nicht gefunden: $FIRMWARE_DIR/$FIRMWARE_FILE${NC}"
        exit 1
    fi
    
    # UART Device prüfen
    if [ ! -e "$UART_DEVICE" ]; then
        echo -e "${RED}UART Device nicht gefunden: $UART_DEVICE${NC}"
        echo -e "${YELLOW}Prüfe UART Konfiguration in /boot/config.txt${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}Alle Voraussetzungen erfüllt${NC}"
    echo
    
    # GPIO Setup
    setup_gpio
    
    # Firmware kompilieren
    compile_firmware
    
    # Firmware flashen
    flash_firmware
    
    # Kommunikationstest (falls aktiviert)
    if [ "$TEST_COMMUNICATION" = "1" ] && [ "$AUTO_RESET" = "1" ]; then
        test_communication
    fi
    
    echo
    echo -e "${GREEN}=== Flash Vorgang abgeschlossen ===${NC}"
    echo -e "${BLUE}STM32 wurde erfolgreich geflasht und läuft jetzt${NC}"
    
    # Zusammenfassung anzeigen
    if [ "$VERBOSE" = "1" ]; then
        echo
        echo -e "${BLUE}=== Zusammenfassung ===${NC}"
        echo -e "Firmware: $FIRMWARE_FILE"
        echo -e "HEX Datei: $HEX_FILE"
        echo -e "UART: $UART_DEVICE @ $BAUD_RATE"
        echo -e "GPIO BOOT0: $BOOT0_PIN"
        echo -e "GPIO RESET: $RESET_PIN"
        if [ "$CREATE_BACKUP" = "1" ]; then
            echo -e "Backup: $BACKUP_DIR"
        fi
    fi
}

# Script starten
main "$@"