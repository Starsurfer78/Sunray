// Polygon.h - Polygon and PolygonList classes for geometric operations
// Part of Sunray Arduino robot mapping system

#ifndef POLYGON_H
#define POLYGON_H

#include "Point.h"
#include <Arduino.h>
#include <SD.h>

// Polygon class constants
const byte POLYGON_FILE_MARKER = 0xBB;        // File integrity marker for Polygon serialization
const short MAX_POLYGON_POINTS = 10000;       // Maximum allowed points per polygon
const float POLYGON_BOUNDS_INIT = 9999.0;     // Initial value for bounding box calculations

// PolygonList class constants
const byte POLYGONLIST_FILE_MARKER = 0xCC;    // File integrity marker for PolygonList serialization
const short MAX_POLYGONS_IN_LIST = 5000;      // Maximum allowed polygons per list

// A closed loop of points representing a geometric polygon
class Polygon
{
  public:
    Point *points;      // Dynamic array of polygon vertices
    short numPoints;    // Number of points in the polygon
    
    // Constructors and destructor
    Polygon();                    // Default constructor - creates empty polygon
    Polygon(short aNumPoints);    // Constructor with pre-allocated point capacity
    ~Polygon();                   // Destructor
    
    // Memory management
    void init();                  // Initialize polygon to empty state
    bool alloc(short aNumPoints); // Allocate memory for polygon points
    void dealloc();               // Deallocate polygon memory
    
    // Utility methods
    void dump();                  // Debug output - print polygon points to console
    long crc();                   // Calculate checksum for data integrity
    void getCenter(Point &pt);    // Calculate polygon center using bounding box
    
    // File I/O operations
    bool read(File &file);        // Read polygon data from file
    bool write(File &file);       // Write polygon data to file
};

// A collection of polygons for managing multiple geometric shapes
class PolygonList
{
   public:
     Polygon *polygons;     // Dynamic array of polygons
     short numPolygons;     // Number of polygons in the list
     
     // Constructors and destructor
     PolygonList();                      // Default constructor - creates empty list
     PolygonList(short aNumPolygons);    // Constructor with pre-allocated polygon capacity
     ~PolygonList();                     // Destructor
     
     // Memory management
     void init();                        // Initialize polygon list to empty state
     bool alloc(short aNumPolygons);     // Allocate memory for polygon list
     void dealloc();                     // Deallocate polygon list memory
     
     // Utility methods
     void dump();                        // Debug output - print all polygons to console
     int numPoints();                    // Calculate total points across all polygons
     long crc();                         // Calculate checksum for data integrity
     
     // File I/O operations
     bool read(File &file);              // Read polygon list from file
     bool write(File &file);             // Write polygon list to file
};

#endif // POLYGON_H