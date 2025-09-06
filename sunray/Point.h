// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Point class for 2D coordinates

#ifndef SUNRAY_POINT_H
#define SUNRAY_POINT_H

#include <Arduino.h>
#include <SD.h>

/**
 * Represents a 2D point with centimeter precision
 * Coordinates are stored as short integers in centimeters
 * and converted to/from meters for external interface
 */
class Point {
public:
    short px; // x-coordinate in centimeters
    short py; // y-coordinate in centimeters
    
    /**
     * Default constructor - initializes point to (0,0)
     */
    Point();
    
    /**
     * Constructor with meter coordinates
     * @param ax x-coordinate in meters
     * @param ay y-coordinate in meters
     */
    Point(float ax, float ay);
    
    /**
     * Initialize point to (0,0)
     */
    void init();
    
    /**
     * Get x-coordinate in meters
     * @return x-coordinate in meters
     */
    float x();
    
    /**
     * Get y-coordinate in meters
     * @return y-coordinate in meters
     */
    float y();
    
    /**
     * Set coordinates in meters
     * @param ax x-coordinate in meters
     * @param ay y-coordinate in meters
     */
    void setXY(float ax, float ay);
    
    /**
     * Copy coordinates from another point
     * @param fromPoint source point to copy from
     */
    void assign(Point &fromPoint);
    
    /**
     * Calculate simple CRC for the point
     * @return CRC value
     */
    long crc();
    
    /**
     * Read point from file
     * @param file file to read from
     * @return true if successful, false on error
     */
    bool read(File &file);
    
    /**
     * Write point to file
     * @param file file to write to
     * @return true if successful, false on error
     */
    bool write(File &file);
};

#endif // SUNRAY_POINT_H