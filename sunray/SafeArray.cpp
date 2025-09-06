// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Safe Array Management Implementation

#include "SafeArray.h"
#include "config.h"
#include "map.h" // For Point class definition

// CONSOLE definition is now handled in config.h

// Global memory error counters (moved from map.cpp)
unsigned long memoryCorruptions = 0;
unsigned long memoryAllocErrors = 0;

// Template specializations for corruption checking

// Point specialization
template<>
void SafeArray<Point>::setCorruptionMarker() {
    #if SAFE_ARRAY_ENABLE_CHECKS
        if (data_ != nullptr && size_ > 0) {
            data_[size_].px = SAFE_ARRAY_CHECK_ID;
            data_[size_].py = SAFE_ARRAY_CHECK_ID;
        }
    #endif
}

template<>
bool SafeArray<Point>::isCorruptionMarkerValid() {
    #if SAFE_ARRAY_ENABLE_CHECKS
        if (data_ != nullptr && size_ > 0) {
            return (data_[size_].px == SAFE_ARRAY_CHECK_ID && 
                    data_[size_].py == SAFE_ARRAY_CHECK_ID);
        }
    #endif
    return true;
}

// Polygon specialization
template<>
void SafeArray<Polygon>::setCorruptionMarker() {
    #if SAFE_ARRAY_ENABLE_CHECKS
        if (data_ != nullptr && size_ > 0) {
            data_[size_].points = (Point*)SAFE_ARRAY_POISON_PTR;
        }
    #endif
}

template<>
bool SafeArray<Polygon>::isCorruptionMarkerValid() {
    #if SAFE_ARRAY_ENABLE_CHECKS
        if (data_ != nullptr && size_ > 0) {
            return (data_[size_].points == (Point*)SAFE_ARRAY_POISON_PTR);
        }
    #endif
    return true;
}

// Helper functions for memory management

/**
 * Get current memory corruption count
 */
unsigned long getMemoryCorruptions() {
    return memoryCorruptions;
}

/**
 * Get current memory allocation error count
 */
unsigned long getMemoryAllocErrors() {
    return memoryAllocErrors;
}

/**
 * Reset memory error counters (for testing)
 */
void resetMemoryErrorCounters() {
    memoryCorruptions = 0;
    memoryAllocErrors = 0;
}

/**
 * Print memory statistics
 */
void printMemoryStats() {
    CONSOLE.print("Memory Stats - Corruptions: ");
    CONSOLE.print(memoryCorruptions);
    CONSOLE.print(", Alloc Errors: ");
    CONSOLE.println(memoryAllocErrors);
}