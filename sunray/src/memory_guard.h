#ifndef MEMORY_GUARD_H
#define MEMORY_GUARD_H

#include "Arduino.h"

/**
 * Memory Guard System for detecting memory corruption and allocation errors
 * Used to monitor dynamic memory allocations in critical data structures
 */
class MemoryGuard {
public:
    // Magic values for corruption detection
    static const uint16_t GUARD_MAGIC_ID = 0x4A4A;
    static const uintptr_t GUARD_MAGIC_PTR = 0x12345678;
    
    // Error counters
    static unsigned long corruptionCount;
    static unsigned long allocationErrorCount;
    
    // Reset error counters
    static void reset();
    
    // Check and report memory errors
    static void checkAndReport();
    
    // Check if memory errors occurred
    static bool hasErrors();
    
    // Record corruption detection
    static void recordCorruption();
    
    // Record allocation failure
    static void recordAllocationError();
    
    // Validate guard values
    static bool validateGuardInt(uint16_t value);
    static bool validateGuardPtr(void* ptr);
    
    // Set guard values
    static void setGuardInt(uint16_t& target);
    static void setGuardPtr(void*& target);
};

// Convenience macros for guard operations
#define MEMORY_GUARD_ENABLED 1

#if MEMORY_GUARD_ENABLED
    #define GUARD_ALLOC_SIZE(size) ((size) + 1)
    #define GUARD_CHECK_INT(value) if (!MemoryGuard::validateGuardInt(value)) MemoryGuard::recordCorruption()
    #define GUARD_CHECK_PTR(ptr) if (!MemoryGuard::validateGuardPtr(ptr)) MemoryGuard::recordCorruption()
    #define GUARD_SET_INT(target) MemoryGuard::setGuardInt(target)
    #define GUARD_SET_PTR(target) MemoryGuard::setGuardPtr(target)
    #define GUARD_RECORD_ALLOC_ERROR() MemoryGuard::recordAllocationError()
#else
    #define GUARD_ALLOC_SIZE(size) (size)
    #define GUARD_CHECK_INT(value) 
    #define GUARD_CHECK_PTR(ptr) 
    #define GUARD_SET_INT(target) 
    #define GUARD_SET_PTR(target) 
    #define GUARD_RECORD_ALLOC_ERROR() 
#endif

#endif // MEMORY_GUARD_H