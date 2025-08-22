#include "memory_guard.h"
#include "../config.h"

// Static member initialization
unsigned long MemoryGuard::corruptionCount = 0;
unsigned long MemoryGuard::allocationErrorCount = 0;

void MemoryGuard::reset() {
    corruptionCount = 0;
    allocationErrorCount = 0;
}

void MemoryGuard::checkAndReport() {
    if (corruptionCount != 0) {
        CONSOLE.print("********************* ERROR: memoryCorruptions=");
        CONSOLE.println(corruptionCount);
        CONSOLE.println(" *********************");
    }
    
    if (allocationErrorCount != 0) {
        CONSOLE.print("********************* ERROR: memoryAllocErrors=");
        CONSOLE.println(allocationErrorCount);
        CONSOLE.println(" *********************");
    }
}

bool MemoryGuard::hasErrors() {
    return (corruptionCount != 0) || (allocationErrorCount != 0);
}

void MemoryGuard::recordCorruption() {
    corruptionCount++;
}

void MemoryGuard::recordAllocationError() {
    allocationErrorCount++;
}

bool MemoryGuard::validateGuardInt(uint16_t value) {
    return value == GUARD_MAGIC_ID;
}

bool MemoryGuard::validateGuardPtr(void* ptr) {
    return (uintptr_t)ptr == GUARD_MAGIC_PTR;
}

void MemoryGuard::setGuardInt(uint16_t& target) {
    target = GUARD_MAGIC_ID;
}

void MemoryGuard::setGuardPtr(void*& target) {
    target = (void*)GUARD_MAGIC_PTR;
}