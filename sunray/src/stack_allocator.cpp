// Stack-based allocator implementation

#include "stack_allocator.h"
#include "../map.h" // For Point class

// Static buffer instances
static Point smallPointBuffer[StackAllocator::SMALL_POINT_BUFFER_SIZE];
static Point mediumPointBuffer[StackAllocator::MEDIUM_POINT_BUFFER_SIZE];
static bool smallBufferInUse = false;
static bool mediumBufferInUse = false;

Point* StackAllocator::getSmallPointBuffer() {
    if (!smallBufferInUse) {
        smallBufferInUse = true;
        return smallPointBuffer;
    }
    return nullptr;
}

Point* StackAllocator::getMediumPointBuffer() {
    if (!mediumBufferInUse) {
        mediumBufferInUse = true;
        return mediumPointBuffer;
    }
    return nullptr;
}

bool StackAllocator::canUseSmallBuffer(size_t count) {
    return !smallBufferInUse && count <= SMALL_POINT_BUFFER_SIZE;
}

bool StackAllocator::canUseMediumBuffer(size_t count) {
    return !mediumBufferInUse && count <= MEDIUM_POINT_BUFFER_SIZE;
}

void StackAllocator::releaseBuffer(Point* buffer) {
    if (buffer == smallPointBuffer) {
        smallBufferInUse = false;
    } else if (buffer == mediumPointBuffer) {
        mediumBufferInUse = false;
    }
    // For dynamic allocations, caller must use delete[]
}

Point* StackAllocator::allocatePoints(size_t count, bool& usedStack) {
    usedStack = false;
    
    // Try small buffer first
    if (canUseSmallBuffer(count)) {
        usedStack = true;
        return getSmallPointBuffer();
    }
    
    // Try medium buffer
    if (canUseMediumBuffer(count)) {
        usedStack = true;
        return getMediumPointBuffer();
    }
    
    // Fall back to dynamic allocation
    return new Point[count];
}

void StackAllocator::deallocatePoints(Point* ptr, bool wasStack) {
    if (!wasStack && ptr != nullptr) {
        delete[] ptr;
    }
    // Stack allocations are automatically freed when buffer is released
}