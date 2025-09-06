// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Safe Array Management with Memory Corruption Detection

#ifndef SUNRAY_SAFE_ARRAY_H
#define SUNRAY_SAFE_ARRAY_H

#include <Arduino.h>
#include <new>
#include "config.h"

// CONSOLE definition is now handled in config.h

// Memory corruption detection constants
#define SAFE_ARRAY_CHECK_ID        0x4A4A
#define SAFE_ARRAY_POISON_PTR      ((void*)0x12345678)

// Enable memory corruption checks only on Arduino platforms
#ifdef _SAM3XA_  // Arduino Due
  #define SAFE_ARRAY_ENABLE_CHECKS 1
#else
  #define SAFE_ARRAY_ENABLE_CHECKS 0
#endif

// Global memory error counters
extern unsigned long memoryCorruptions;
extern unsigned long memoryAllocErrors;

/**
 * Safe array template class with optional memory corruption detection
 * Provides RAII-style memory management for dynamic arrays
 */
template<typename T>
class SafeArray {
public:
    SafeArray() : data_(nullptr), size_(0) {}
    
    explicit SafeArray(int size) : data_(nullptr), size_(0) {
        alloc(size);
    }
    
    ~SafeArray() {
        dealloc();
    }
    
    // Disable copy constructor and assignment (for now)
    SafeArray(const SafeArray&) = delete;
    SafeArray& operator=(const SafeArray&) = delete;
    
    /**
     * Allocate array with given size
     * @param newSize Number of elements to allocate
     * @return true if successful, false on error
     */
    bool alloc(int newSize) {
        if (newSize == size_) return true;
        
        if (newSize < 0 || newSize > getMaxSize()) {
            CONSOLE.println("ERROR SafeArray::alloc invalid size");
            return false;
        }
        
        // Calculate actual allocation size (with corruption check if enabled)
        int allocSize = newSize;
        #if SAFE_ARRAY_ENABLE_CHECKS
            allocSize += 1; // Extra element for corruption detection
        #endif
        
        T* newData = new(std::nothrow) T[allocSize];
        if (newData == nullptr) {
            CONSOLE.println("ERROR SafeArray::alloc out of memory");
            memoryAllocErrors++;
            return false;
        }
        
        // Copy existing data if any
        if (data_ != nullptr) {
            int copySize = (newSize < size_) ? newSize : size_;
            for (int i = 0; i < copySize; i++) {
                newData[i] = data_[i];
            }
            
            // Check for corruption before freeing
            checkCorruption();
            delete[] data_;
        }
        
        data_ = newData;
        size_ = newSize;
        
        // Initialize corruption check marker
        #if SAFE_ARRAY_ENABLE_CHECKS
            initCorruptionCheck();
        #endif
        
        return true;
    }
    
    /**
     * Deallocate array
     */
    void dealloc() {
        if (data_ == nullptr) return;
        
        checkCorruption();
        delete[] data_;
        data_ = nullptr;
        size_ = 0;
    }
    
    /**
     * Get array size
     */
    int size() const { return size_; }
    
    /**
     * Check if array is allocated
     */
    bool isAllocated() const { return data_ != nullptr; }
    
    /**
     * Array access operators
     */
    T& operator[](int index) {
        return data_[index];
    }
    
    const T& operator[](int index) const {
        return data_[index];
    }
    
    /**
     * Get raw pointer (use with caution)
     */
    T* data() { return data_; }
    const T* data() const { return data_; }
    
protected:
    /**
     * Get maximum allowed size for this array type
     * Override in derived classes for type-specific limits
     */
    virtual int getMaxSize() const {
        return 32767; // Default safe limit for short indices
    }
    
private:
    T* data_;
    int size_;
    
    /**
     * Initialize corruption check markers
     */
    void initCorruptionCheck() {
        #if SAFE_ARRAY_ENABLE_CHECKS
            if (data_ != nullptr && size_ > 0) {
                // Set corruption check values in the extra element
                // This is type-specific and needs specialization
                setCorruptionMarker();
            }
        #endif
    }
    
    /**
     * Check for memory corruption
     */
    void checkCorruption() {
        #if SAFE_ARRAY_ENABLE_CHECKS
            if (data_ != nullptr && size_ > 0) {
                if (!isCorruptionMarkerValid()) {
                    memoryCorruptions++;
                    CONSOLE.println("WARNING: Memory corruption detected!");
                }
            }
        #endif
    }
    
    /**
     * Set corruption marker - needs specialization for each type
     */
    void setCorruptionMarker();
    
    /**
     * Check corruption marker - needs specialization for each type
     */
    bool isCorruptionMarkerValid();
};

// Forward declarations for specializations
class Point;
class Polygon;

/**
 * Specialized safe array for Points
 */
class SafePointArray : public SafeArray<Point> {
public:
    SafePointArray() : SafeArray<Point>() {}
    explicit SafePointArray(int size) : SafeArray<Point>(size) {}
    
protected:
    int getMaxSize() const override {
        return 10000; // Limit from original Polygon class
    }
};

/**
 * Specialized safe array for Polygons
 */
class SafePolygonArray : public SafeArray<Polygon> {
public:
    SafePolygonArray() : SafeArray<Polygon>() {}
    explicit SafePolygonArray(int size) : SafeArray<Polygon>(size) {}
    
protected:
    int getMaxSize() const override {
        return 5000; // Limit from original PolygonList class
    }
};

#endif // SUNRAY_SAFE_ARRAY_H