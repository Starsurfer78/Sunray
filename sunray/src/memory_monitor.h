// Memory Monitoring System for Sunray
// Consolidates and improves memory monitoring capabilities

#ifndef MEMORY_MONITOR_H
#define MEMORY_MONITOR_H

#include <Arduino.h>

class MemoryMonitor {
public:
    struct MemoryInfo {
        int freeBytes;
        int totalBytes;
        float usagePercent;
        bool isLowMemory;
        unsigned long timestamp;
    };
    
    // Get current memory information
    static MemoryInfo getMemoryInfo();
    
    // Get free memory in bytes (consolidated from freeMemory() and freeRam())
    static int getFreeMemory();
    
    // Check if memory is critically low
    static bool isMemoryLow(int threshold = 1024); // Default 1KB threshold
    
    // Log memory status to console
    static void logMemoryStatus();
    
    // Track memory usage over time
    static void recordMemoryUsage();
    
    // Get memory statistics
    static void getMemoryStats(int& minFree, int& maxFree, float& avgFree);
    
    // Reset memory statistics
    static void resetStats();
    
private:
    static const int MAX_SAMPLES = 10;
    static int memorySamples[MAX_SAMPLES];
    static int sampleIndex;
    static int sampleCount;
    static unsigned long lastSampleTime;
    
    // Platform-specific implementations
    static int getAvailableMemory();
};

#endif // MEMORY_MONITOR_H