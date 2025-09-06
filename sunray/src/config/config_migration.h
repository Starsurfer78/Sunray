// Ardumower Sunray - Alfred Platform
// Configuration Migration Helper
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH

#ifndef CONFIG_MIGRATION_H
#define CONFIG_MIGRATION_H

#include "config_structures.h"
#include "config_validator.h"
#include "../../config.h"

class ConfigMigration {
public:
    struct MigrationResult {
        bool success;
        SystemConfig migratedConfig;
        std::vector<String> changes;
        std::vector<String> warnings;
        String summary;
        
        MigrationResult() : success(false) {}
        
        void addChange(const String& change) {
            changes.push_back(change);
        }
        
        void addWarning(const String& warning) {
            warnings.push_back(warning);
        }
        
        String getReport() const {
            String report = "Configuration Migration Report\n";
            report += "=====================================\n";
            report += "Status: " + String(success ? "SUCCESS" : "FAILED") + "\n";
            report += "Changes: " + String(changes.size()) + "\n";
            report += "Warnings: " + String(warnings.size()) + "\n\n";
            
            if (!changes.empty()) {
                report += "CHANGES MADE:\n";
                for (const auto& change : changes) {
                    report += "  + " + change + "\n";
                }
                report += "\n";
            }
            
            if (!warnings.empty()) {
                report += "WARNINGS:\n";
                for (const auto& warning : warnings) {
                    report += "  ! " + warning + "\n";
                }
                report += "\n";
            }
            
            if (!summary.isEmpty()) {
                report += "SUMMARY:\n" + summary + "\n";
            }
            
            return report;
        }
    };
    
    // Main migration methods
    static MigrationResult migrateFromLegacyConfig();
    static MigrationResult migrateMotorConfig();
    static MigrationResult migrateATProtocolConfig();
    static MigrationResult migrateSensorConfig();
    static MigrationResult migrateBatteryConfig();
    
    // Backup and restore
    static bool backupCurrentConfig(const String& filename);
    static bool restoreConfig(const String& filename, SystemConfig& config);
    
    // Version management
    static String getCurrentConfigVersion();
    static bool isConfigVersionSupported(const String& version);
    static std::vector<String> getRequiredMigrationSteps(const String& fromVersion, const String& toVersion);
    
    // Validation after migration
    static ConfigValidator::ValidationResult validateMigratedConfig(const SystemConfig& config);
    
    // Export/Import
    static bool exportConfigToFile(const SystemConfig& config, const String& filename);
    static MigrationResult importConfigFromFile(const String& filename);
    
    // Utility methods
    static void printMigrationReport(const MigrationResult& result);
    static bool saveMigrationReport(const MigrationResult& result, const String& filename);
    
private:
    // Legacy config extraction helpers
    static MotorConfig extractMotorConfigFromLegacy();
    static ATProtocolConfig extractATProtocolConfigFromLegacy();
    static SensorConfig extractSensorConfigFromLegacy();
    static BatteryConfig extractBatteryConfigFromLegacy();
    
    // Value mapping helpers
    static float mapLegacyPIDValue(float legacyValue, const String& paramName);
    static unsigned long mapLegacyBaudRate(unsigned long legacyRate);
    static float mapLegacyCurrentValue(float legacyValue, const String& context);
    static float mapLegacyVoltageValue(float legacyValue, const String& context);
    
    // Compatibility checks
    static bool isLegacyConfigComplete();
    static std::vector<String> getMissingLegacyValues();
    static bool hasConflictingLegacyValues();
    
    // Default value providers
    static MotorConfig getDefaultMotorConfig();
    static ATProtocolConfig getDefaultATProtocolConfig();
    static SensorConfig getDefaultSensorConfig();
    static BatteryConfig getDefaultBatteryConfig();
};

// Implementation of migration methods
inline ConfigMigration::MigrationResult ConfigMigration::migrateFromLegacyConfig() {
    MigrationResult result;
    
    try {
        // Check if legacy config is complete
        if (!isLegacyConfigComplete()) {
            auto missing = getMissingLegacyValues();
            for (const auto& value : missing) {
                result.addWarning("Missing legacy value: " + value + ", using default");
            }
        }
        
        // Check for conflicts
        if (hasConflictingLegacyValues()) {
            result.addWarning("Conflicting legacy values detected, using priority order");
        }
        
        // Migrate individual components
        auto motorResult = migrateMotorConfig();
        auto atResult = migrateATProtocolConfig();
        auto sensorResult = migrateSensorConfig();
        auto batteryResult = migrateBatteryConfig();
        
        // Build system config
        result.migratedConfig.motor = motorResult.migratedConfig.motor;
        result.migratedConfig.atProtocol = atResult.migratedConfig.atProtocol;
        result.migratedConfig.sensor = sensorResult.migratedConfig.sensor;
        result.migratedConfig.battery = batteryResult.migratedConfig.battery;
        
        // Set metadata
        result.migratedConfig.configVersion = "2.0.0";
        result.migratedConfig.migrationDate = "2024-01-01"; // Should be current date
        result.migratedConfig.legacyCompatible = true;
        
        // Merge changes and warnings
        result.changes.insert(result.changes.end(), motorResult.changes.begin(), motorResult.changes.end());
        result.changes.insert(result.changes.end(), atResult.changes.begin(), atResult.changes.end());
        result.changes.insert(result.changes.end(), sensorResult.changes.begin(), sensorResult.changes.end());
        result.changes.insert(result.changes.end(), batteryResult.changes.begin(), batteryResult.changes.end());
        
        result.warnings.insert(result.warnings.end(), motorResult.warnings.begin(), motorResult.warnings.end());
        result.warnings.insert(result.warnings.end(), atResult.warnings.begin(), atResult.warnings.end());
        result.warnings.insert(result.warnings.end(), sensorResult.warnings.begin(), sensorResult.warnings.end());
        result.warnings.insert(result.warnings.end(), batteryResult.warnings.begin(), batteryResult.warnings.end());
        
        // Validate migrated config
        auto validation = validateMigratedConfig(result.migratedConfig);
        if (!validation.isValid) {
            result.addWarning("Migrated configuration has validation errors");
            result.migratedConfig.isValid = false;
        } else {
            result.migratedConfig.isValid = true;
        }
        
        result.success = true;
        result.summary = "Successfully migrated legacy configuration to structured format with " + 
                        String(result.changes.size()) + " changes and " + String(result.warnings.size()) + " warnings.";
        
    } catch (...) {
        result.success = false;
        result.summary = "Migration failed due to unexpected error";
    }
    
    return result;
}

inline ConfigMigration::MigrationResult ConfigMigration::migrateMotorConfig() {
    MigrationResult result;
    
    // Extract motor config from legacy defines
    result.migratedConfig.motor = extractMotorConfigFromLegacy();
    
    // Document changes
    result.addChange("Migrated PID parameters from legacy defines");
    result.addChange("Migrated current limits from legacy defines");
    result.addChange("Migrated mechanical parameters from legacy defines");
    
    // Check for potential issues
#ifdef MOTOR_PID_KP
    if (MOTOR_PID_KP > 5.0) {
        result.addWarning("High PID Kp value detected: " + String(MOTOR_PID_KP));
    }
#endif

#ifdef MOTOR_FAULT_CURRENT
    if (MOTOR_FAULT_CURRENT > 10.0) {
        result.addWarning("High fault current detected: " + String(MOTOR_FAULT_CURRENT));
    }
#endif
    
    result.success = true;
    return result;
}

inline ConfigMigration::MigrationResult ConfigMigration::migrateATProtocolConfig() {
    MigrationResult result;
    
    // Extract AT protocol config from legacy defines
    result.migratedConfig.atProtocol = extractATProtocolConfigFromLegacy();
    
    // Document changes
    result.addChange("Migrated baud rate from legacy defines");
    result.addChange("Migrated timeout values from legacy defines");
    result.addChange("Set default retry and buffer values");
    
    // Check for potential issues
#ifdef SERIAL_BAUD
    if (SERIAL_BAUD < 9600) {
        result.addWarning("Low baud rate may cause communication delays: " + String(SERIAL_BAUD));
    }
#endif
    
    result.success = true;
    return result;
}

inline ConfigMigration::MigrationResult ConfigMigration::migrateSensorConfig() {
    MigrationResult result;
    
    // Extract sensor config from legacy defines
    result.migratedConfig.sensor = extractSensorConfigFromLegacy();
    
    // Document changes
    result.addChange("Migrated bumper settings from legacy defines");
    result.addChange("Migrated temperature thresholds from legacy defines");
    result.addChange("Set default sensor timing values");
    
    result.success = true;
    return result;
}

inline ConfigMigration::MigrationResult ConfigMigration::migrateBatteryConfig() {
    MigrationResult result;
    
    // Extract battery config from legacy defines
    result.migratedConfig.battery = extractBatteryConfigFromLegacy();
    
    // Document changes
    result.addChange("Migrated battery voltage thresholds from legacy defines");
    result.addChange("Migrated charging current settings from legacy defines");
    
    // Check for potential issues
#ifdef BATTERY_UNDER_VOLTAGE
    if (BATTERY_UNDER_VOLTAGE < 10.0) {
        result.addWarning("Very low undervoltage threshold: " + String(BATTERY_UNDER_VOLTAGE));
    }
#endif
    
    result.success = true;
    return result;
}

// Legacy config extraction implementations
inline MotorConfig ConfigMigration::extractMotorConfigFromLegacy() {
    MotorConfig config = getDefaultMotorConfig();
    
    // Extract PID values
#ifdef MOTOR_PID_KP
    config.pidKp = MOTOR_PID_KP;
#endif
#ifdef MOTOR_PID_KI
    config.pidKi = MOTOR_PID_KI;
#endif
#ifdef MOTOR_PID_KD
    config.pidKd = MOTOR_PID_KD;
#endif
    
    // Extract current limits
#ifdef MOTOR_FAULT_CURRENT
    config.faultCurrent = MOTOR_FAULT_CURRENT;
#endif
#ifdef MOTOR_OVERLOAD_CURRENT
    config.overloadCurrent = MOTOR_OVERLOAD_CURRENT;
#endif
#ifdef MOTOR_TOO_LOW_CURRENT
    config.tooLowCurrent = MOTOR_TOO_LOW_CURRENT;
#endif
    
    // Extract mow motor settings
#ifdef MOW_FAULT_CURRENT
    config.mowFaultCurrent = MOW_FAULT_CURRENT;
#endif
#ifdef MOW_OVERLOAD_CURRENT
    config.mowOverloadCurrent = MOW_OVERLOAD_CURRENT;
#endif
#ifdef MOW_TOO_LOW_CURRENT
    config.mowTooLowCurrent = MOW_TOO_LOW_CURRENT;
#endif
    
    // Extract mechanical parameters
#ifdef WHEEL_DIAMETER
    config.wheelDiameter = WHEEL_DIAMETER;
#endif
#ifdef WHEEL_BASE
    config.wheelBase = WHEEL_BASE;
#endif
#ifdef TICKS_PER_REVOLUTION
    config.ticksPerRevolution = TICKS_PER_REVOLUTION;
#endif
    
    return config;
}

inline ATProtocolConfig ConfigMigration::extractATProtocolConfigFromLegacy() {
    ATProtocolConfig config = getDefaultATProtocolConfig();
    
    // Extract baud rate
#ifdef SERIAL_BAUD
    config.baudRate = SERIAL_BAUD;
#endif
    
    // Extract timeouts
#ifdef MOTOR_TIMEOUT
    config.motorTimeout = MOTOR_TIMEOUT;
#endif
#ifdef SUMMARY_TIMEOUT
    config.summaryTimeout = SUMMARY_TIMEOUT;
#endif
#ifdef RESPONSE_TIMEOUT
    config.responseTimeout = RESPONSE_TIMEOUT;
#endif
    
    return config;
}

inline SensorConfig ConfigMigration::extractSensorConfigFromLegacy() {
    SensorConfig config = getDefaultSensorConfig();
    
    // Extract bumper settings
#ifdef BUMPER_DEAD_TIME
    config.bumperDeadTime = BUMPER_DEAD_TIME;
#endif
#ifdef BUMPER_MAX_TRIGGER_TIME
    config.bumperMaxTriggerTime = BUMPER_MAX_TRIGGER_TIME;
#endif
    
    // Extract temperature settings
#ifdef DOCK_OVERHEAT_TEMP
    config.dockOverheatTemp = DOCK_OVERHEAT_TEMP;
#endif
#ifdef DOCK_TOO_COLD_TEMP
    config.dockTooColdTemp = DOCK_TOO_COLD_TEMP;
#endif
    
    return config;
}

inline BatteryConfig ConfigMigration::extractBatteryConfigFromLegacy() {
    BatteryConfig config = getDefaultBatteryConfig();
    
    // Extract voltage thresholds
#ifdef BATTERY_UNDER_VOLTAGE
    config.underVoltage = BATTERY_UNDER_VOLTAGE;
#endif
#ifdef BATTERY_GO_HOME_VOLTAGE
    config.goHomeVoltage = BATTERY_GO_HOME_VOLTAGE;
#endif
#ifdef BATTERY_FULL_VOLTAGE
    config.fullVoltage = BATTERY_FULL_VOLTAGE;
#endif
    
    // Extract current settings
#ifdef BATTERY_FULL_CURRENT
    config.fullCurrent = BATTERY_FULL_CURRENT;
#endif
    
    return config;
}

// Default config providers
inline MotorConfig ConfigMigration::getDefaultMotorConfig() {
    MotorConfig config;
    config.pidKp = 2.0;
    config.pidKi = 0.03;
    config.pidKd = 0.03;
    config.faultCurrent = 8.0;
    config.overloadCurrent = 2.0;
    config.tooLowCurrent = 0.05;
    config.mowFaultCurrent = 8.0;
    config.mowOverloadCurrent = 2.0;
    config.mowTooLowCurrent = 0.05;
    config.wheelDiameter = 250.0;
    config.wheelBase = 36.0;
    config.ticksPerRevolution = 1060;
    config.overloadSpeed = 0.1;
    return config;
}

inline ATProtocolConfig ConfigMigration::getDefaultATProtocolConfig() {
    ATProtocolConfig config;
    config.baudRate = 115200;
    config.motorTimeout = 8000;
    config.summaryTimeout = 4000;
    config.responseTimeout = 2000;
    config.maxRetries = 3;
    config.bufferSize = 512;
    config.enableCRC = true;
    return config;
}

inline SensorConfig ConfigMigration::getDefaultSensorConfig() {
    SensorConfig config;
    config.bumperDeadTime = 1000;
    config.bumperMaxTriggerTime = 30000;
    config.dockOverheatTemp = 60.0;
    config.dockTooColdTemp = -10.0;
    return config;
}

inline BatteryConfig ConfigMigration::getDefaultBatteryConfig() {
    BatteryConfig config;
    config.underVoltage = 21.7;
    config.goHomeVoltage = 23.7;
    config.fullVoltage = 29.4;
    config.fullCurrent = -0.3;
    return config;
}

inline bool ConfigMigration::isLegacyConfigComplete() {
    // Check if essential legacy defines exist
    bool hasMotorPID = false;
    bool hasCurrentLimits = false;
    bool hasBatteryVoltages = false;
    bool hasBaudRate = false;
    
#if defined(MOTOR_PID_KP) && defined(MOTOR_PID_KI) && defined(MOTOR_PID_KD)
    hasMotorPID = true;
#endif
    
#if defined(MOTOR_FAULT_CURRENT) && defined(MOTOR_OVERLOAD_CURRENT)
    hasCurrentLimits = true;
#endif
    
#if defined(BATTERY_UNDER_VOLTAGE) && defined(BATTERY_FULL_VOLTAGE)
    hasBatteryVoltages = true;
#endif
    
#ifdef SERIAL_BAUD
    hasBaudRate = true;
#endif
    
    return hasMotorPID && hasCurrentLimits && hasBatteryVoltages && hasBaudRate;
}

inline void ConfigMigration::printMigrationReport(const MigrationResult& result) {
    CONSOLE.println(result.getReport());
}

#endif // CONFIG_MIGRATION_H