// Ardumower Sunray - Alfred Platform
// Configuration Manager
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "config_structures.h"
#include "config_validator.h"
#include "config_migration.h"
#include "../../config.h"
#include <memory>

class ConfigManager {
public:
    enum class ConfigSource {
        LEGACY_DEFINES,
        STRUCTURED_CONFIG,
        FILE_IMPORT,
        DEFAULT_VALUES
    };
    
    enum class SaveFormat {
        BINARY,
        JSON,
        INI,
        HEADER_FILE
    };
    
    struct ConfigStatus {
        bool isLoaded;
        bool isValid;
        ConfigSource source;
        String version;
        String lastError;
        unsigned long lastModified;
        
        ConfigStatus() : isLoaded(false), isValid(false), source(ConfigSource::DEFAULT_VALUES), lastModified(0) {}
    };
    
private:
    static std::unique_ptr<SystemConfig> currentConfig;
    static ConfigStatus status;
    static bool initialized;
    
public:
    // Initialization and lifecycle
    static bool initialize();
    static bool shutdown();
    static bool isInitialized() { return initialized; }
    
    // Configuration loading
    static bool loadFromLegacy();
    static bool loadFromFile(const String& filename);
    static bool loadFromDefaults();
    static bool reload();
    
    // Configuration saving
    static bool saveToFile(const String& filename, SaveFormat format = SaveFormat::JSON);
    static bool saveAsHeader(const String& filename);
    static bool backup(const String& filename);
    
    // Configuration access
    static const SystemConfig& getConfig();
    static SystemConfig& getMutableConfig();
    static ConfigStatus getStatus() { return status; }
    
    // Component-specific access
    static const MotorConfig& getMotorConfig();
    static const ATProtocolConfig& getATProtocolConfig();
    static const SensorConfig& getSensorConfig();
    static const BatteryConfig& getBatteryConfig();
    
    // Configuration modification
    static bool updateMotorConfig(const MotorConfig& config);
    static bool updateATProtocolConfig(const ATProtocolConfig& config);
    static bool updateSensorConfig(const SensorConfig& config);
    static bool updateBatteryConfig(const BatteryConfig& config);
    
    // Validation and migration
    static ConfigValidator::ValidationResult validate();
    static ConfigMigration::MigrationResult migrate();
    static bool isConfigurationValid();
    
    // Runtime parameter access (for backward compatibility)
    static float getMotorPIDKp();
    static float getMotorPIDKi();
    static float getMotorPIDKd();
    static float getMotorFaultCurrent();
    static float getMotorOverloadCurrent();
    static float getBatteryUnderVoltage();
    static float getBatteryGoHomeVoltage();
    static float getBatteryFullVoltage();
    static unsigned long getSerialBaudRate();
    static unsigned long getMotorTimeout();
    
    // Runtime parameter modification
    static bool setMotorPIDKp(float value);
    static bool setMotorPIDKi(float value);
    static bool setMotorPIDKd(float value);
    static bool setMotorFaultCurrent(float value);
    static bool setBatteryUnderVoltage(float value);
    
    // Configuration monitoring
    static void enableAutoSave(bool enable, unsigned long intervalMs = 300000); // 5 minutes
    static void enableValidationChecks(bool enable);
    static void setConfigChangeCallback(void (*callback)(const String& component, const String& parameter));
    
    // Debugging and diagnostics
    static void printCurrentConfig();
    static void printConfigDifferences(const SystemConfig& other);
    static String getConfigSummary();
    static std::vector<String> getConfigHistory();
    
    // Factory reset
    static bool factoryReset();
    static bool resetToDefaults();
    
    // Configuration versioning
    static String getCurrentVersion();
    static bool isVersionCompatible(const String& version);
    static std::vector<String> getSupportedVersions();
    
private:
    // Internal helpers
    static bool validateAndApplyConfig(const SystemConfig& config);
    static void notifyConfigChange(const String& component, const String& parameter);
    static bool ensureConfigLoaded();
    static void updateStatus(bool loaded, bool valid, ConfigSource source, const String& error = "");
    
    // Auto-save functionality
    static bool autoSaveEnabled;
    static unsigned long autoSaveInterval;
    static unsigned long lastAutoSave;
    static void checkAutoSave();
    
    // Validation functionality
    static bool validationEnabled;
    static void (*configChangeCallback)(const String&, const String&);
    
    // Configuration history
    static std::vector<String> configHistory;
    static void addToHistory(const String& action);
    
    // File I/O helpers
    static bool saveAsJSON(const String& filename);
    static bool saveAsINI(const String& filename);
    static bool saveAsBinary(const String& filename);
    static bool loadFromJSON(const String& filename);
    static bool loadFromINI(const String& filename);
    static bool loadFromBinary(const String& filename);
};

// Static member initialization
std::unique_ptr<SystemConfig> ConfigManager::currentConfig = nullptr;
ConfigManager::ConfigStatus ConfigManager::status;
bool ConfigManager::initialized = false;
bool ConfigManager::autoSaveEnabled = false;
unsigned long ConfigManager::autoSaveInterval = 300000;
unsigned long ConfigManager::lastAutoSave = 0;
bool ConfigManager::validationEnabled = true;
void (*ConfigManager::configChangeCallback)(const String&, const String&) = nullptr;
std::vector<String> ConfigManager::configHistory;

// Implementation of key methods
inline bool ConfigManager::initialize() {
    if (initialized) {
        return true;
    }
    
    try {
        currentConfig = std::make_unique<SystemConfig>();
        
        // Try to load configuration in priority order
        bool loaded = false;
        
        // 1. Try to load from structured config file
        if (loadFromFile("alfred_config.json")) {
            loaded = true;
            updateStatus(true, true, ConfigSource::STRUCTURED_CONFIG);
            addToHistory("Loaded from structured config file");
        }
        // 2. Try to migrate from legacy defines
        else if (loadFromLegacy()) {
            loaded = true;
            updateStatus(true, true, ConfigSource::LEGACY_DEFINES);
            addToHistory("Migrated from legacy defines");
            
            // Save migrated config for future use
            saveToFile("alfred_config.json");
        }
        // 3. Fall back to defaults
        else {
            loadFromDefaults();
            updateStatus(true, true, ConfigSource::DEFAULT_VALUES);
            addToHistory("Loaded default configuration");
        }
        
        // Validate the loaded configuration
        auto validation = validate();
        if (!validation.isValid) {
            status.isValid = false;
            status.lastError = "Configuration validation failed";
            CONSOLE.println("WARNING: Configuration validation failed!");
            ConfigValidator::printValidationReport(validation);
        }
        
        initialized = true;
        CONSOLE.println("ConfigManager initialized successfully");
        
        return true;
        
    } catch (...) {
        updateStatus(false, false, ConfigSource::DEFAULT_VALUES, "Initialization failed");
        return false;
    }
}

inline bool ConfigManager::loadFromLegacy() {
    if (!ensureConfigLoaded()) {
        return false;
    }
    
    try {
        auto migration = ConfigMigration::migrateFromLegacyConfig();
        if (migration.success) {
            *currentConfig = migration.migratedConfig;
            addToHistory("Migrated from legacy configuration");
            CONSOLE.println("Successfully migrated legacy configuration");
            ConfigMigration::printMigrationReport(migration);
            return true;
        } else {
            status.lastError = "Legacy migration failed";
            return false;
        }
    } catch (...) {
        status.lastError = "Exception during legacy migration";
        return false;
    }
}

inline bool ConfigManager::loadFromDefaults() {
    if (!ensureConfigLoaded()) {
        return false;
    }
    
    try {
        // Load default configurations
        currentConfig->motor = ConfigMigration::getDefaultMotorConfig();
        currentConfig->atProtocol = ConfigMigration::getDefaultATProtocolConfig();
        currentConfig->sensor = ConfigMigration::getDefaultSensorConfig();
        currentConfig->battery = ConfigMigration::getDefaultBatteryConfig();
        
        // Set metadata
        currentConfig->configVersion = "2.0.0";
        currentConfig->migrationDate = "2024-01-01";
        currentConfig->legacyCompatible = true;
        currentConfig->isValid = true;
        
        addToHistory("Loaded default configuration");
        return true;
        
    } catch (...) {
        status.lastError = "Failed to load default configuration";
        return false;
    }
}

inline const SystemConfig& ConfigManager::getConfig() {
    if (!ensureConfigLoaded()) {
        // Return a static default config if loading fails
        static SystemConfig defaultConfig;
        return defaultConfig;
    }
    
    checkAutoSave();
    return *currentConfig;
}

inline SystemConfig& ConfigManager::getMutableConfig() {
    ensureConfigLoaded();
    return *currentConfig;
}

// Component-specific access methods
inline const MotorConfig& ConfigManager::getMotorConfig() {
    return getConfig().motor;
}

inline const ATProtocolConfig& ConfigManager::getATProtocolConfig() {
    return getConfig().atProtocol;
}

inline const SensorConfig& ConfigManager::getSensorConfig() {
    return getConfig().sensor;
}

inline const BatteryConfig& ConfigManager::getBatteryConfig() {
    return getConfig().battery;
}

// Runtime parameter access (backward compatibility)
inline float ConfigManager::getMotorPIDKp() {
    return getMotorConfig().pidKp;
}

inline float ConfigManager::getMotorPIDKi() {
    return getMotorConfig().pidKi;
}

inline float ConfigManager::getMotorPIDKd() {
    return getMotorConfig().pidKd;
}

inline float ConfigManager::getMotorFaultCurrent() {
    return getMotorConfig().faultCurrent;
}

inline float ConfigManager::getMotorOverloadCurrent() {
    return getMotorConfig().overloadCurrent;
}

inline float ConfigManager::getBatteryUnderVoltage() {
    return getBatteryConfig().underVoltage;
}

inline float ConfigManager::getBatteryGoHomeVoltage() {
    return getBatteryConfig().goHomeVoltage;
}

inline float ConfigManager::getBatteryFullVoltage() {
    return getBatteryConfig().fullVoltage;
}

inline unsigned long ConfigManager::getSerialBaudRate() {
    return getATProtocolConfig().baudRate;
}

inline unsigned long ConfigManager::getMotorTimeout() {
    return getATProtocolConfig().motorTimeout;
}

// Runtime parameter modification
inline bool ConfigManager::setMotorPIDKp(float value) {
    if (!ensureConfigLoaded()) return false;
    
    if (value <= 0 || value > 100) {
        status.lastError = "Invalid PID Kp value";
        return false;
    }
    
    currentConfig->motor.pidKp = value;
    notifyConfigChange("motor", "pidKp");
    addToHistory("Updated motor PID Kp to " + String(value));
    return true;
}

inline bool ConfigManager::setMotorPIDKi(float value) {
    if (!ensureConfigLoaded()) return false;
    
    if (value < 0 || value > 10) {
        status.lastError = "Invalid PID Ki value";
        return false;
    }
    
    currentConfig->motor.pidKi = value;
    notifyConfigChange("motor", "pidKi");
    addToHistory("Updated motor PID Ki to " + String(value));
    return true;
}

inline bool ConfigManager::setMotorPIDKd(float value) {
    if (!ensureConfigLoaded()) return false;
    
    if (value < 0 || value > 10) {
        status.lastError = "Invalid PID Kd value";
        return false;
    }
    
    currentConfig->motor.pidKd = value;
    notifyConfigChange("motor", "pidKd");
    addToHistory("Updated motor PID Kd to " + String(value));
    return true;
}

// Validation and utility methods
inline ConfigValidator::ValidationResult ConfigManager::validate() {
    if (!ensureConfigLoaded()) {
        ConfigValidator::ValidationResult result;
        result.addError(RobotErrorCode::CONFIG_INVALID, "Configuration not loaded");
        return result;
    }
    
    return ConfigValidator::validateSystemConfig(*currentConfig);
}

inline bool ConfigManager::isConfigurationValid() {
    auto validation = validate();
    return validation.isValid;
}

inline void ConfigManager::printCurrentConfig() {
    if (!ensureConfigLoaded()) {
        CONSOLE.println("Configuration not loaded");
        return;
    }
    
    ConfigValidator::printConfigSummary(*currentConfig);
}

inline String ConfigManager::getConfigSummary() {
    if (!ensureConfigLoaded()) {
        return "Configuration not loaded";
    }
    
    String summary = "Alfred Configuration Summary\n";
    summary += "Version: " + currentConfig->configVersion + "\n";
    summary += "Source: ";
    
    switch (status.source) {
        case ConfigSource::LEGACY_DEFINES: summary += "Legacy Defines"; break;
        case ConfigSource::STRUCTURED_CONFIG: summary += "Structured Config"; break;
        case ConfigSource::FILE_IMPORT: summary += "File Import"; break;
        case ConfigSource::DEFAULT_VALUES: summary += "Default Values"; break;
    }
    
    summary += "\nValid: " + String(status.isValid ? "Yes" : "No") + "\n";
    summary += "Last Modified: " + String(status.lastModified) + "\n";
    
    if (!status.lastError.isEmpty()) {
        summary += "Last Error: " + status.lastError + "\n";
    }
    
    return summary;
}

// Private helper methods
inline bool ConfigManager::ensureConfigLoaded() {
    if (!initialized) {
        return initialize();
    }
    
    if (!currentConfig) {
        currentConfig = std::make_unique<SystemConfig>();
        return loadFromDefaults();
    }
    
    return true;
}

inline void ConfigManager::updateStatus(bool loaded, bool valid, ConfigSource source, const String& error) {
    status.isLoaded = loaded;
    status.isValid = valid;
    status.source = source;
    status.lastError = error;
    status.lastModified = millis();
}

inline void ConfigManager::notifyConfigChange(const String& component, const String& parameter) {
    if (configChangeCallback) {
        configChangeCallback(component, parameter);
    }
    
    status.lastModified = millis();
}

inline void ConfigManager::addToHistory(const String& action) {
    String entry = String(millis()) + ": " + action;
    configHistory.push_back(entry);
    
    // Keep only last 50 entries
    if (configHistory.size() > 50) {
        configHistory.erase(configHistory.begin());
    }
}

inline void ConfigManager::checkAutoSave() {
    if (!autoSaveEnabled) return;
    
    unsigned long now = millis();
    if (now - lastAutoSave > autoSaveInterval) {
        saveToFile("alfred_config_auto.json");
        lastAutoSave = now;
        addToHistory("Auto-saved configuration");
    }
}

inline std::vector<String> ConfigManager::getConfigHistory() {
    return configHistory;
}

inline String ConfigManager::getCurrentVersion() {
    if (ensureConfigLoaded()) {
        return currentConfig->configVersion;
    }
    return "unknown";
}

#endif // CONFIG_MANAGER_H