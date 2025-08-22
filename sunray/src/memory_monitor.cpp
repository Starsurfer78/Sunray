// Memory Monitoring System for Sunray
// Consolidates and improves memory monitoring capabilities

#include "memory_monitor.h"
#include "../config.h"

#ifndef __linux__
  extern "C" char* sbrk(int incr);
#endif

// Static member initialization
int MemoryMonitor::memorySamples[MAX_SAMPLES] = {0};
int MemoryMonitor::sampleIndex = 0;
int MemoryMonitor::sampleCount = 0;
unsigned long MemoryMonitor::lastSampleTime = 0;

MemoryMonitor::MemoryInfo MemoryMonitor::getMemoryInfo() {
    MemoryInfo info;
    info.freeBytes = getFreeMemory();
    info.timestamp = millis();
    
    // Estimate total memory based on platform
#if defined(__SAM3X8E__) // Arduino Due
    info.totalBytes = 96 * 1024; // 96KB SRAM
#elif defined(__AVR__)
    info.totalBytes = 8 * 1024;  // Typical AVR (e.g., Mega 2560: 8KB)
#else
    info.totalBytes = 32 * 1024; // Conservative estimate
#endif
    
    info.usagePercent = ((float)(info.totalBytes - info.freeBytes) / info.totalBytes) * 100.0f;
    info.isLowMemory = isMemoryLow();
    
    return info;
}

int MemoryMonitor::getFreeMemory() {
    return getAvailableMemory();
}

int MemoryMonitor::getAvailableMemory() {
#ifdef __AVR__
    // AVR-specific implementation (from helper.cpp freeRam)
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
#elif defined(__linux__)
    // Linux simulation
    return 1000000;
#else
    // Default implementation (from reset.cpp freeMemory)
    char top;
    return &top - reinterpret_cast<char*>(sbrk(0));
#endif
}

bool MemoryMonitor::isMemoryLow(int threshold) {
    return getFreeMemory() < threshold;
}

void MemoryMonitor::logMemoryStatus() {
    MemoryInfo info = getMemoryInfo();
    
    Serial.print(F("Memory: "));
    Serial.print(info.freeBytes);
    Serial.print(F(" bytes free ("));
    Serial.print(info.usagePercent, 1);
    Serial.print(F("% used)"));
    
    if (info.isLowMemory) {
        Serial.print(F(" [LOW MEMORY WARNING]"));
    }
    
    Serial.println();
}

void MemoryMonitor::recordMemoryUsage() {
    unsigned long currentTime = millis();
    
    // Sample every 5 seconds to avoid overhead
    if (currentTime - lastSampleTime < 5000) {
        return;
    }
    
    memorySamples[sampleIndex] = getFreeMemory();
    sampleIndex = (sampleIndex + 1) % MAX_SAMPLES;
    
    if (sampleCount < MAX_SAMPLES) {
        sampleCount++;
    }
    
    lastSampleTime = currentTime;
}

void MemoryMonitor::getMemoryStats(int& minFree, int& maxFree, float& avgFree) {
    if (sampleCount == 0) {
        minFree = maxFree = getFreeMemory();
        avgFree = minFree;
        return;
    }
    
    minFree = memorySamples[0];
    maxFree = memorySamples[0];
    long sum = 0;
    
    for (int i = 0; i < sampleCount; i++) {
        int sample = memorySamples[i];
        if (sample < minFree) minFree = sample;
        if (sample > maxFree) maxFree = sample;
        sum += sample;
    }
    
    avgFree = (float)sum / sampleCount;
}

void MemoryMonitor::resetStats() {
    sampleIndex = 0;
    sampleCount = 0;
    lastSampleTime = 0;
    
    for (int i = 0; i < MAX_SAMPLES; i++) {
        memorySamples[i] = 0;
    }
}