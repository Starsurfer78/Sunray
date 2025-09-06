# STM32 Flash-Scripts Installation

## 🚀 Schnelle Installation

### 1. Scripts ausführbar machen

```bash
cd $HOME/sunray/alfred
chmod +x *.sh
```

### 2. Abhängigkeiten installieren

```bash
# Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

# OpenOCD
sudo apt-get update
sudo apt-get install openocd

# STM32 Board Package
arduino-cli core update-index
arduino-cli core install STMicroelectronics:stm32
```

### 3. Erste Verwendung

```bash
# Manager-Script starten
sudo ./stm32_manager.sh

# Option 1: System-Status prüfen
# Option 3: Kompilieren + Flashen
# Option 4: AT-Test
```

## ✅ Verifikation

```bash
# AT-Kommando testen
echo "AT+V" > /dev/ttyS0
cat /dev/ttyS0

# Erwartete Antwort: Version-String
```

## 📖 Vollständige Dokumentation

Siehe `STM32_FLASH_README.md` für detaillierte Anweisungen.

---

**Bereit für STM32-Firmware-Management!**