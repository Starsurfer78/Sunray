// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// PolygonList class implementation

#include "PolygonList.h"
#include "robot.h" // For CONSOLE macro
#include "SafeArray.h" // For CHECK_CORRUPT and memory management
#include "MapConstants.h"
#include <algorithm> // For min/max functions

// External memory error counters (defined in SafeArray.cpp)
extern unsigned long memoryAllocErrors;
extern unsigned long memoryCorruptions;

// Memory corruption check point for PolygonList
#define CHECK_POINT ((Point*)0xDEADBEEF)

PolygonList::PolygonList() {
    init();
}

PolygonList::PolygonList(short aNumPolygons) {
    init();
    alloc(aNumPolygons);
}

void PolygonList::init() {
    numPolygons = 0;
    polygons = NULL;
}

PolygonList::~PolygonList() {
    // dealloc();
}

bool PolygonList::alloc(short aNumPolygons) {
    if (aNumPolygons == numPolygons) return true;
    
    if ((aNumPolygons < 0) || (aNumPolygons > MAX_POLYGON_LIST_SIZE)) {
        CONSOLE.println("ERROR PolygonList::alloc invalid number");
        return false;
    }
    
    Polygon* newPolygons = new Polygon[aNumPolygons + CHECK_CORRUPT];
    if (newPolygons == NULL) {
        CONSOLE.println("ERROR PolygonList::alloc out of memory");
        memoryAllocErrors++;
        return false;
    }
    
    if (polygons != NULL) {
        memcpy(newPolygons, polygons, sizeof(Polygon) * std::min(numPolygons, aNumPolygons));
        
        if (aNumPolygons < numPolygons) {
            for (int i = aNumPolygons; i < numPolygons; i++) {
                // polygons[i].dealloc();
            }
        }
        
        if (polygons[numPolygons].points != CHECK_POINT) memoryCorruptions++;
        delete[] polygons;
    }
    
    polygons = newPolygons;
    numPolygons = aNumPolygons;
    polygons[numPolygons].points = CHECK_POINT;
    
    return true;
}

void PolygonList::dealloc() {
    if (polygons == NULL) return;
    
    for (int i = 0; i < numPolygons; i++) {
        polygons[i].dealloc();
    }
    
    if (polygons[numPolygons].points != CHECK_POINT) memoryCorruptions++;
    
    delete[] polygons;
    polygons = NULL;
    numPolygons = 0;
}

int PolygonList::numPoints() {
    int num = 0;
    for (int i = 0; i < numPolygons; i++) {
        num += polygons[i].numPoints;
    }
    return num;
}

void PolygonList::dump() {
    for (int i = 0; i < numPolygons; i++) {
        CONSOLE.print(i);
        CONSOLE.print(":");
        polygons[i].dump();
    }
    CONSOLE.println();
}

long PolygonList::crc() {
    long crc = 0;
    for (int i = 0; i < numPolygons; i++) {
        crc += polygons[i].crc();
    }
    return crc;
}

bool PolygonList::read(File &file) {
    byte marker = file.read();
    if (marker != POLYGON_LIST_FILE_MARKER) {
        CONSOLE.println("ERROR reading polygon list: invalid marker");
        return false;
    }
    
    short num = 0;
    file.read((uint8_t*)&num, sizeof(num));
    
    if (!alloc(num)) return false;
    
    for (short i = 0; i < num; i++) {
        if (!polygons[i].read(file)) return false;
    }
    
    return true;
}

bool PolygonList::write(File &file) {
    if (file.write(POLYGON_LIST_FILE_MARKER) == 0) {
        CONSOLE.println("ERROR writing polygon list marker");
        return false;
    }
    
    if (file.write((uint8_t*)&numPolygons, sizeof(numPolygons)) == 0) {
        CONSOLE.println("ERROR writing polygon list");
        return false;
    }
    
    for (int i = 0; i < numPolygons; i++) {
        if (!polygons[i].write(file)) return false;
    }
    
    return true;
}