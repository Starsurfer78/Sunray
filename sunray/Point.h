// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
/*
  Point class - represents a 2D coordinate point on the map
*/

#ifndef SUNRAY_POINT_H
#define SUNRAY_POINT_H

#include <Arduino.h>
#include <SD.h>

// Point class constants
const float METERS_TO_CM_FACTOR = 100.0;     // Conversion factor from meters to centimeters
const byte POINT_FILE_MARKER = 0xAA;         // File integrity marker for point data

// A point on the map with coordinates stored in centimeters for precision
class Point
{
  public:
    short px; // X coordinate in centimeters
    short py; // Y coordinate in centimeters
    
    // Constructors
    Point();                        // Default constructor - initializes point at origin (0,0)
    Point(float ax, float ay);      // Constructor with coordinates in meters
    
    // Coordinate access methods
    float x();                      // Get X coordinate in meters
    float y();                      // Get Y coordinate in meters
    
    // Coordinate manipulation
    void init();                    // Initialize point coordinates to origin
    void setXY(float ax, float ay); // Set coordinates in meters
    void assign(Point &fromPoint);  // Copy coordinates from another point
    
    // Data integrity and persistence
    long crc();                     // Calculate simple checksum for data integrity
    bool read(File &file);          // Read point data from file with integrity check
    bool write(File &file);         // Write point data to file with integrity marker
};

#endif // SUNRAY_POINT_H