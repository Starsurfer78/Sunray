#!/bin/bash

# Installationsskript für STM32 Flash-Tools auf Raspberry Pi
# Installiert alle benötigten Abhängigkeiten für das Flash-System

set -e

# Farben für Ausgabe
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== STM32 Flash-Tools Installation ===${NC}"
echo -e "${YELLOW}Installiert alle benötigten Tools für STM32 Firmware-Flash${NC}"
echo

# System-Update
echo -e "${YELLOW}1. System aktualisieren...${NC}"
sudo apt-get update

# Basis-Tools installieren
echo -e "${YELLOW}2. Basis-Tools installieren...${NC}"
sudo apt-get install -y \
    curl \
    wget \
    git \
    build-essential \
    python3 \
    python3-pip

echo -e "${GREEN}✓ Basis-Tools installiert${NC}"

# stm32flash installieren
echo -e "${YELLOW}3. stm32flash installieren...${NC}"
if command -v stm32flash &> /dev/null; then
    echo -e "${GREEN}✓ stm32flash bereits installiert${NC}"
else
    sudo apt-get install -y stm32flash
    echo -e "${GREEN}✓ stm32flash installiert${NC}"
fi

# Arduino CLI installieren
echo -e "${YELLOW}4. Arduino CLI installieren...${NC}"
if command -v arduino-cli &> /dev/null; then
    echo -e "${GREEN}✓ Arduino CLI bereits installiert${NC}"
else
    # Arduino CLI herunterladen und installieren
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    
    # In PATH hinzufügen
    if ! grep -q "$HOME/bin" ~/.bashrc; then
        echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
    fi
    
    # Für aktuelle Session
    export PATH="$HOME/bin:$PATH"
    
    echo -e "${GREEN}✓ Arduino CLI installiert${NC}"
fi

# Arduino CLI konfigurieren
echo -e "${YELLOW}5. Arduino CLI konfigurieren...${NC}"

# Config-Datei erstellen
arduino-cli config init

# STM32 Board-Package hinzufügen
echo -e "${BLUE}  STM32 Board-Package installieren...${NC}"
arduino-cli core update-index
arduino-cli core install STMicroelectronics:stm32

echo -e "${GREEN}✓ Arduino CLI konfiguriert${NC}"

# UART aktivieren
echo -e "${YELLOW}6. UART konfigurieren...${NC}"

# Benutzer zur dialout-Gruppe hinzufügen
sudo usermod -a -G dialout $USER

# UART in /boot/config.txt aktivieren
if ! grep -q "enable_uart=1" /boot/config.txt; then
    echo "enable_uart=1" | sudo tee -a /boot/config.txt
    echo -e "${YELLOW}⚠ UART aktiviert - Neustart erforderlich${NC}"
    REBOOT_REQUIRED=1
else
    echo -e "${GREEN}✓ UART bereits aktiviert${NC}"
fi

# Serial Console deaktivieren (optional)
if grep -q "console=serial0" /boot/cmdline.txt; then
    echo -e "${YELLOW}⚠ Serial Console aktiv - kann Flash stören${NC}"
    echo -e "${BLUE}  Zum Deaktivieren: sudo raspi-config -> Advanced -> Serial -> No${NC}"
fi

echo -e "${GREEN}✓ UART konfiguriert${NC}"

# GPIO-Tools installieren (optional)
echo -e "${YELLOW}7. GPIO-Tools installieren...${NC}"
if command -v gpio &> /dev/null; then
    echo -e "${GREEN}✓ WiringPi bereits installiert${NC}"
else
    # WiringPi für GPIO-Kontrolle
    sudo apt-get install -y wiringpi
    echo -e "${GREEN}✓ WiringPi installiert${NC}"
fi

# Flash-Skripte ausführbar machen
echo -e "${YELLOW}8. Flash-Skripte konfigurieren...${NC}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

chmod +x "$SCRIPT_DIR/flash_stm32.sh" 2>/dev/null || true
chmod +x "$SCRIPT_DIR/test_flash.sh" 2>/dev/null || true

echo -e "${GREEN}✓ Skripte ausführbar${NC}"

# Installation testen
echo -e "${YELLOW}9. Installation testen...${NC}"

# Versionen anzeigen
echo -e "${BLUE}Installierte Versionen:${NC}"
echo -n "  stm32flash: "
stm32flash -h 2>&1 | head -n 1 | grep -o "v[0-9.]*" || echo "installiert"

echo -n "  arduino-cli: "
arduino-cli version 2>/dev/null | grep -o "[0-9.]*" || echo "installiert"

echo -n "  gpio: "
gpio -v 2>/dev/null | head -n 1 || echo "installiert"

# Test-Skript ausführen (falls vorhanden)
if [ -f "$SCRIPT_DIR/test_flash.sh" ]; then
    echo -e "${BLUE}  Führe Validierungstest aus...${NC}"
    bash "$SCRIPT_DIR/test_flash.sh"
fi

echo
echo -e "${GREEN}=== Installation abgeschlossen ===${NC}"

if [ "$REBOOT_REQUIRED" = "1" ]; then
    echo -e "${YELLOW}⚠ NEUSTART ERFORDERLICH für UART-Aktivierung${NC}"
    echo -e "${BLUE}Nach dem Neustart:${NC}"
else
    echo -e "${BLUE}Nächste Schritte:${NC}"
fi

echo -e "  1. Flash-Konfiguration anpassen: nano flash_config.conf"
echo -e "  2. Hardware anschließen (siehe README_FLASH.md)"
echo -e "  3. Firmware flashen: ./flash_stm32.sh"
echo
echo -e "${BLUE}Dokumentation: README_FLASH.md${NC}"
echo -e "${BLUE}Test: ./test_flash.sh${NC}"

if [ "$REBOOT_REQUIRED" = "1" ]; then
    echo
    read -p "Jetzt neustarten? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo reboot
    fi
fi