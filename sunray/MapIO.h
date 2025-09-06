// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Map Input/Output operations

#ifndef MAP_IO_H
#define MAP_IO_H

#include "Point.h"
#include "Polygon.h"
#include "PolygonList.h"
#include <SD.h>

// File format constants
#define MAP_FILE_MARKER 0x00001000

// Forward declaration
class Map;

class MapIO {
public:
    // Load map data from SD card
    static bool loadMap(Map* map);
    
    // Save map data to SD card
    static bool saveMap(Map* map);
    
private:
    // Helper functions for loading
    static bool readMapHeader(File &file, uint32_t &mapCRC, int &exclusionPointsCount);
    static bool readMapData(File &file, Map* map, int exclusionPointsCount);
    
    // Helper functions for saving
    static bool writeMapHeader(File &file, uint32_t mapCRC, int exclusionPointsCount);
    static bool writeMapData(File &file, Map* map);
    
    // Validation functions
    static bool validateMapFile(File &file);
    static bool validateMapCRC(Map* map, uint32_t expectedCRC);
};

#endif // MAP_IO_H