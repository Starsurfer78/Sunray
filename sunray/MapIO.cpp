// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Map Input/Output operations implementation

#include "MapIO.h"
#include "map.h" // For Map class definition
#include "robot.h" // For CONSOLE macro
#include "MapConstants.h"
#include <SD.h>

// Load map data from SD card
bool MapIO::loadMap(Map* map) {
    bool res = true;
    
#if defined(ENABLE_SD_RESUME)
    CONSOLE.print("map load... ");
    
    File mapFile = SD.open("map.bin", FILE_READ);
    if (!mapFile) {
        CONSOLE.println("ERROR opening file for reading");
        return false;
    }
    
    uint32_t mapCRC;
    int exclusionPointsCount;
    
    // Read and validate header
    if (!readMapHeader(mapFile, mapCRC, exclusionPointsCount)) {
        mapFile.close();
        return false;
    }
    
    // Read map data
    if (!readMapData(mapFile, map, exclusionPointsCount)) {
        mapFile.close();
        return false;
    }
    
    mapFile.close();
    
    // Validate CRC
    if (!validateMapCRC(map, mapCRC)) {
        CONSOLE.println("ERROR map CRC");
        map->clearMap();
        return false;
    }
    
    CONSOLE.println("ok");
    
#else
    res = false;
#endif
    
    return res;
}

// Save map data to SD card
bool MapIO::saveMap(Map* map) {
    bool res = true;
    
#if defined(ENABLE_SD_RESUME)
    CONSOLE.print("map save... ");
    
    File mapFile = SD.open("map.bin", FILE_CREATE);
    if (!mapFile) {
        CONSOLE.println("ERROR opening file for writing");
        return false;
    }
    
    // Write header
    if (!writeMapHeader(mapFile, map->mapCRC, map->exclusionPointsCount)) {
        mapFile.close();
        return false;
    }
    
    // Write map data
    if (!writeMapData(mapFile, map)) {
        mapFile.close();
        return false;
    }
    
    mapFile.flush();
    mapFile.close();
    
    if (res) {
        CONSOLE.println("ok");
    } else {
        CONSOLE.println("ERROR saving map");
    }
    
#else
    res = false;
#endif
    
    return res;
}

// Read map file header
bool MapIO::readMapHeader(File &file, uint32_t &mapCRC, int &exclusionPointsCount) {
    uint32_t marker;
    
    if (file.read((uint8_t*)&marker, sizeof(marker)) == 0) {
        CONSOLE.println("ERROR reading marker");
        return false;
    }
    
    if (marker != MAP_FILE_MARKER) {
        CONSOLE.println("ERROR invalid marker");
        return false;
    }
    
    if (file.read((uint8_t*)&mapCRC, sizeof(mapCRC)) == 0) {
        CONSOLE.println("ERROR reading CRC");
        return false;
    }
    
    if (file.read((uint8_t*)&exclusionPointsCount, sizeof(exclusionPointsCount)) == 0) {
        CONSOLE.println("ERROR reading exclusion count");
        return false;
    }
    
    return true;
}

// Read map data from file
bool MapIO::readMapData(File &file, Map* map, int exclusionPointsCount) {
    // Read perimeter points
    if (!map->perimeterPoints.read(file)) {
        CONSOLE.println("ERROR reading perimeter");
        return false;
    }
    
    // Read exclusions
    if (!map->exclusions.read(file)) {
        CONSOLE.println("ERROR reading exclusions");
        return false;
    }
    
    // Read dock points
    if (!map->dockPoints.read(file)) {
        CONSOLE.println("ERROR reading dock points");
        return false;
    }
    
    // Read mow points
    if (!map->mowPoints.read(file)) {
        CONSOLE.println("ERROR reading mow points");
        return false;
    }
    
    // Set exclusion points count
    map->exclusionPointsCount = exclusionPointsCount;
    
    return true;
}

// Write map file header
bool MapIO::writeMapHeader(File &file, uint32_t mapCRC, int exclusionPointsCount) {
    uint32_t marker = MAP_FILE_MARKER;
    
    if (file.write((uint8_t*)&marker, sizeof(marker)) == 0) {
        CONSOLE.println("ERROR writing marker");
        return false;
    }
    
    if (file.write((uint8_t*)&mapCRC, sizeof(mapCRC)) == 0) {
        CONSOLE.println("ERROR writing CRC");
        return false;
    }
    
    if (file.write((uint8_t*)&exclusionPointsCount, sizeof(exclusionPointsCount)) == 0) {
        CONSOLE.println("ERROR writing exclusion count");
        return false;
    }
    
    return true;
}

// Write map data to file
bool MapIO::writeMapData(File &file, Map* map) {
    // Write perimeter points
    if (!map->perimeterPoints.write(file)) {
        CONSOLE.println("ERROR writing perimeter");
        return false;
    }
    
    // Write exclusions
    if (!map->exclusions.write(file)) {
        CONSOLE.println("ERROR writing exclusions");
        return false;
    }
    
    // Write dock points
    if (!map->dockPoints.write(file)) {
        CONSOLE.println("ERROR writing dock points");
        return false;
    }
    
    // Write mow points
    if (!map->mowPoints.write(file)) {
        CONSOLE.println("ERROR writing mow points");
        return false;
    }
    
    return true;
}

// Validate map file format
bool MapIO::validateMapFile(File &file) {
    uint32_t marker;
    
    if (file.read((uint8_t*)&marker, sizeof(marker)) == 0) {
        return false;
    }
    
    return (marker == MAP_FILE_MARKER);
}

// Validate map CRC
bool MapIO::validateMapCRC(Map* map, uint32_t expectedCRC) {
    uint32_t calculatedCRC = map->calcMapCRC();
    return (calculatedCRC == expectedCRC);
}