#!/bin/bash

# STM32 Manager Script für Alfred Raspberry Pi
# Vereinfachtes Management für STM32-Firmware: Kompilieren, Flashen, Testen

set -e  # Exit bei Fehlern

# Konfiguration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$HOME/sunray/firmware"
LOG_FILE="/tmp/stm32_manager.log"

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging-Funktion
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG_FILE"
}

# Banner anzeigen
show_banner() {
    clear
    echo -e "${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║                    STM32 Manager für Alfred                 ║"
    echo "║                                                              ║"
    echo "║  Verwaltet STM32-Firmware für Alfred Mähroboter             ║"
    echo "║  Kommunikation über /dev/ttyS0 @ 19200 Baud                 ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
    echo
}

# Status-Informationen anzeigen
show_status() {
    echo -e "${BLUE}=== System-Status ===${NC}"
    
    # Prüfe Sunray-Service
    if systemctl is-active --quiet sunray.service; then
        echo -e "${GREEN}✓ Sunray-Service: Aktiv${NC}"
    else
        echo -e "${YELLOW}⚠ Sunray-Service: Inaktiv${NC}"
    fi
    
    # Prüfe serielle Verbindung
    if [ -e "/dev/ttyS0" ]; then
        echo -e "${GREEN}✓ Serieller Port: /dev/ttyS0 verfügbar${NC}"
    else
        echo -e "${RED}✗ Serieller Port: /dev/ttyS0 nicht verfügbar${NC}"
    fi
    
    # Prüfe Firmware-Datei
    if [ -f "$FIRMWARE_DIR/rm18.ino.bin" ]; then
        echo -e "${GREEN}✓ Firmware-Binary: Vorhanden${NC}"
        echo -e "  Größe: $(stat -c%s "$FIRMWARE_DIR/rm18.ino.bin") Bytes"
        echo -e "  Datum: $(stat -c%y "$FIRMWARE_DIR/rm18.ino.bin" | cut -d' ' -f1)"
    else
        echo -e "${YELLOW}⚠ Firmware-Binary: Nicht gefunden${NC}"
    fi
    
    # Prüfe Source-Code
    if [ -f "$FIRMWARE_DIR/rm18.ino" ]; then
        echo -e "${GREEN}✓ Firmware-Source: Vorhanden${NC}"
    else
        echo -e "${RED}✗ Firmware-Source: Nicht gefunden${NC}"
    fi
    
    echo
}

# AT-Kommando testen
test_at_communication() {
    echo -e "${YELLOW}Teste AT-Kommunikation mit STM32...${NC}"
    
    if [ ! -e "/dev/ttyS0" ]; then
        echo -e "${RED}✗ /dev/ttyS0 nicht verfügbar${NC}"
        return 1
    fi
    
    # Konfiguriere serielle Verbindung
    stty -F /dev/ttyS0 19200 cs8 -cstopb -parenb raw
    
    echo -e "${BLUE}Sende AT+V (Version abfragen)...${NC}"
    echo "AT+V" > /dev/ttyS0
    
    # Warte auf Antwort (timeout 3 Sekunden)
    if timeout 3 cat /dev/ttyS0 | head -1 > /tmp/at_response.txt 2>/dev/null; then
        response=$(cat /tmp/at_response.txt)
        if [ -n "$response" ]; then
            echo -e "${GREEN}✓ STM32 Antwort: $response${NC}"
            log "AT-Test erfolgreich: $response"
            rm -f /tmp/at_response.txt
            return 0
        fi
    fi
    
    echo -e "${RED}✗ Keine Antwort von STM32${NC}"
    echo -e "${YELLOW}Mögliche Ursachen:${NC}"
    echo "  - STM32 nicht geflasht oder defekt"
    echo "  - Falsche Baudrate (erwartet: 19200)"
    echo "  - Hardware-Verbindungsproblem"
    log "AT-Test fehlgeschlagen"
    rm -f /tmp/at_response.txt
    return 1
}

# Hauptmenü anzeigen
show_menu() {
    echo -e "${MAGENTA}=== Hauptmenü ===${NC}"
    echo "1) Firmware kompilieren (rm18.ino → .bin)"
    echo "2) Firmware flashen (OpenOCD/SWD)"
    echo "3) Kompilieren + Flashen (Komplett-Workflow)"
    echo "4) AT-Kommunikation testen"
    echo "5) System-Status anzeigen"
    echo "6) Sunray-Service starten/stoppen"
    echo "7) Log-Datei anzeigen"
    echo "8) Hilfe & Troubleshooting"
    echo "9) Beenden"
    echo
    echo -e "${CYAN}Wähle eine Option (1-9): ${NC}"
}

# Hilfe anzeigen
show_help() {
    echo -e "${BLUE}=== Hilfe & Troubleshooting ===${NC}"
    echo
    echo -e "${YELLOW}Typische Probleme und Lösungen:${NC}"
    echo
    echo -e "${GREEN}1. CRC-Fehler in der Kommunikation:${NC}"
    echo "   - Prüfe Baudrate (sollte 19200 sein)"
    echo "   - Teste AT-Kommunikation (Option 4)"
    echo "   - Flashe neueste Firmware (Option 2)"
    echo
    echo -e "${GREEN}2. STM32 antwortet nicht:${NC}"
    echo "   - Prüfe Hardware-Verbindungen (SWD: SWDIO, SWCLK, GND)"
    echo "   - Prüfe Stromversorgung des STM32"
    echo "   - Flashe Firmware neu (Option 2)"
    echo
    echo -e "${GREEN}3. Kompilierung schlägt fehl:${NC}"
    echo "   - Installiere Arduino CLI: curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo "   - Installiere STM32-Board-Package"
    echo "   - Prüfe rm18.ino Syntax"
    echo
    echo -e "${GREEN}4. Flash-Vorgang schlägt fehl:${NC}"
    echo "   - Installiere OpenOCD: sudo apt-get install openocd"
    echo "   - Prüfe SWD-Verbindung (GPIO-Pins)"
    echo "   - Führe als root aus: sudo ./stm32_manager.sh"
    echo
    echo -e "${YELLOW}Wichtige Dateien:${NC}"
    echo "   - Firmware-Source: $FIRMWARE_DIR/rm18.ino"
    echo "   - Firmware-Binary: $FIRMWARE_DIR/rm18.ino.bin"
    echo "   - Serieller Port: /dev/ttyS0 @ 19200 Baud"
    echo "   - Log-Datei: $LOG_FILE"
    echo
    echo -e "${YELLOW}AT-Kommandos zum Testen:${NC}"
    echo "   - AT+V    : Version abfragen"
    echo "   - AT+S    : Status abfragen"
    echo "   - AT+M    : Motor-Status abfragen"
    echo
}

# Service-Management
manage_service() {
    echo -e "${BLUE}=== Sunray-Service Management ===${NC}"
    
    if systemctl is-active --quiet sunray.service; then
        echo -e "${GREEN}Sunray-Service ist aktiv${NC}"
        echo "1) Service stoppen"
        echo "2) Service neu starten"
        echo "3) Zurück zum Hauptmenü"
        echo
        read -p "Wähle eine Option (1-3): " choice
        
        case $choice in
            1)
                echo -e "${YELLOW}Stoppe Sunray-Service...${NC}"
                systemctl stop sunray.service
                echo -e "${GREEN}✓ Service gestoppt${NC}"
                log "Sunray-Service gestoppt"
                ;;
            2)
                echo -e "${YELLOW}Starte Sunray-Service neu...${NC}"
                systemctl restart sunray.service
                echo -e "${GREEN}✓ Service neu gestartet${NC}"
                log "Sunray-Service neu gestartet"
                ;;
            3)
                return
                ;;
        esac
    else
        echo -e "${YELLOW}Sunray-Service ist inaktiv${NC}"
        echo "1) Service starten"
        echo "2) Zurück zum Hauptmenü"
        echo
        read -p "Wähle eine Option (1-2): " choice
        
        case $choice in
            1)
                echo -e "${YELLOW}Starte Sunray-Service...${NC}"
                systemctl start sunray.service
                echo -e "${GREEN}✓ Service gestartet${NC}"
                log "Sunray-Service gestartet"
                ;;
            2)
                return
                ;;
        esac
    fi
    
    echo
    read -p "Drücke Enter um fortzufahren..."
}

# Log-Datei anzeigen
show_log() {
    echo -e "${BLUE}=== Log-Datei (letzte 20 Zeilen) ===${NC}"
    if [ -f "$LOG_FILE" ]; then
        tail -20 "$LOG_FILE"
    else
        echo -e "${YELLOW}Keine Log-Datei gefunden${NC}"
    fi
    echo
    read -p "Drücke Enter um fortzufahren..."
}

# Hauptprogramm
main() {
    # Prüfe Root-Rechte für kritische Operationen
    if [ "$EUID" -ne 0 ] && [[ "$1" =~ ^[23]$ ]]; then
        echo -e "${RED}Fehler: Für Flash-Operationen sind Root-Rechte erforderlich${NC}"
        echo "Verwendung: sudo $0"
        exit 1
    fi
    
    log "STM32 Manager gestartet"
    
    while true; do
        show_banner
        show_status
        show_menu
        
        read -p "" choice
        echo
        
        case $choice in
            1)
                echo -e "${YELLOW}Starte Firmware-Kompilierung...${NC}"
                log "Kompilierung gestartet"
                if bash "$SCRIPT_DIR/compile_stm32.sh"; then
                    log "Kompilierung erfolgreich"
                else
                    log "Kompilierung fehlgeschlagen"
                fi
                echo
                read -p "Drücke Enter um fortzufahren..."
                ;;
            2)
                echo -e "${YELLOW}Starte Flash-Vorgang...${NC}"
                log "Flash-Vorgang gestartet"
                if bash "$SCRIPT_DIR/flash_stm32.sh"; then
                    log "Flash-Vorgang erfolgreich"
                else
                    log "Flash-Vorgang fehlgeschlagen"
                fi
                echo
                read -p "Drücke Enter um fortzufahren..."
                ;;
            3)
                echo -e "${YELLOW}Starte Komplett-Workflow (Kompilieren + Flashen)...${NC}"
                log "Komplett-Workflow gestartet"
                
                if bash "$SCRIPT_DIR/compile_stm32.sh"; then
                    echo -e "${GREEN}✓ Kompilierung erfolgreich${NC}"
                    echo -e "${YELLOW}Starte Flash-Vorgang...${NC}"
                    
                    if bash "$SCRIPT_DIR/flash_stm32.sh"; then
                        echo -e "${GREEN}✓ Komplett-Workflow erfolgreich abgeschlossen!${NC}"
                        log "Komplett-Workflow erfolgreich"
                    else
                        echo -e "${RED}✗ Flash-Vorgang fehlgeschlagen${NC}"
                        log "Flash-Vorgang im Komplett-Workflow fehlgeschlagen"
                    fi
                else
                    echo -e "${RED}✗ Kompilierung fehlgeschlagen${NC}"
                    log "Kompilierung im Komplett-Workflow fehlgeschlagen"
                fi
                echo
                read -p "Drücke Enter um fortzufahren..."
                ;;
            4)
                test_at_communication
                echo
                read -p "Drücke Enter um fortzufahren..."
                ;;
            5)
                # Status wird bereits oben angezeigt
                read -p "Drücke Enter um fortzufahren..."
                ;;
            6)
                manage_service
                ;;
            7)
                show_log
                ;;
            8)
                show_help
                read -p "Drücke Enter um fortzufahren..."
                ;;
            9)
                echo -e "${GREEN}Auf Wiedersehen!${NC}"
                log "STM32 Manager beendet"
                exit 0
                ;;
            *)
                echo -e "${RED}Ungültige Auswahl. Bitte wähle 1-9.${NC}"
                sleep 2
                ;;
        esac
    done
}

# Script starten
main "$@"