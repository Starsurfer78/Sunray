// Stack-based allocator for small objects to reduce dynamic allocations
// Provides fixed-size buffers for common small allocations

#ifndef STACK_ALLOCATOR_H
#define STACK_ALLOCATOR_H

#include <Arduino.h>

// Forward declaration
class Point;

// Stack-based buffer for small Point arrays
template<typename T, size_t MAX_SIZE>
class StackBuffer {
public:
    StackBuffer() : size_(0) {}
    
    // Try to allocate from stack buffer
    T* allocate(size_t count) {
        if (count <= MAX_SIZE) {
            size_ = count;
            return buffer_;
        }
        return nullptr; // Fall back to dynamic allocation
    }
    
    // Check if allocation would fit in stack buffer
    bool canAllocate(size_t count) const {
        return count <= MAX_SIZE;
    }
    
    // Get current size
    size_t size() const { return size_; }
    
    // Get maximum capacity
    size_t capacity() const { return MAX_SIZE; }
    
private:
    T buffer_[MAX_SIZE];
    size_t size_;
};

// Common stack buffers for Sunray
class StackAllocator {
public:
    // Small point buffer for typical polygon operations
    static const size_t SMALL_POINT_BUFFER_SIZE = 16;
    static const size_t MEDIUM_POINT_BUFFER_SIZE = 64;
    
    // Get stack buffers (simplified interface)
    static Point* getSmallPointBuffer();
    static Point* getMediumPointBuffer();
    static bool canUseSmallBuffer(size_t count);
    static bool canUseMediumBuffer(size_t count);
    static void releaseBuffer(Point* buffer);
    
    // Helper function to choose appropriate buffer
    static Point* allocatePoints(size_t count, bool& usedStack);
    static void deallocatePoints(Point* ptr, bool wasStack);
};

#endif // STACK_ALLOCATOR_H