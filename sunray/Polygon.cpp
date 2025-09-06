// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Polygon class implementation

#include "Polygon.h"
#include "robot.h" // For CONSOLE macro
#include "SafeArray.h" // For CHECK_CORRUPT and memory management
#include "MapConstants.h"
#include <algorithm> // For min/max functions

// External memory error counters (defined in SafeArray.cpp)
extern unsigned long memoryAllocErrors;
extern unsigned long memoryCorruptions;

Polygon::Polygon() {
    init();
}

Polygon::Polygon(short aNumPoints) {
    init();
    alloc(aNumPoints);
}

void Polygon::init() {
    numPoints = 0;
    points = NULL;
}

Polygon::~Polygon() {
    // dealloc();
}

bool Polygon::alloc(short aNumPoints) {
    if (aNumPoints == numPoints) return true;
    
    if ((aNumPoints < 0) || (aNumPoints > 10000)) {
        CONSOLE.println("ERROR Polygon::alloc invalid number");
        return false;
    }
    
    Point* newPoints = new Point[aNumPoints + CHECK_CORRUPT];
    if (newPoints == NULL) {
        CONSOLE.println("ERROR Polygon::alloc out of memory");
        memoryAllocErrors++;
        return false;
    }
    
    if (points != NULL) {
        memcpy(newPoints, points, sizeof(Point) * std::min(numPoints, aNumPoints));
        if (points[numPoints].px != CHECK_ID) memoryCorruptions++;
        if (points[numPoints].py != CHECK_ID) memoryCorruptions++;
        delete[] points;
    }
    
    points = newPoints;
    numPoints = aNumPoints;
    points[numPoints].px = CHECK_ID;
    points[numPoints].py = CHECK_ID;
    
    return true;
}

void Polygon::dealloc() {
    if (points == NULL) return;
    
    if (points[numPoints].px != CHECK_ID) memoryCorruptions++;
    if (points[numPoints].py != CHECK_ID) memoryCorruptions++;
    
    delete[] points;
    points = NULL;
    numPoints = 0;
}

void Polygon::dump() {
    for (int i = 0; i < numPoints; i++) {
        CONSOLE.print("(");
        CONSOLE.print(points[i].x());
        CONSOLE.print(",");
        CONSOLE.print(points[i].y());
        CONSOLE.print(")");
        if (i < numPoints - 1) CONSOLE.print(",");
    }
    CONSOLE.println();
}

long Polygon::crc() {
    long crc = 0;
    for (int i = 0; i < numPoints; i++) {
        crc += points[i].crc();
    }
    return crc;
}

bool Polygon::read(File &file) {
    byte marker = file.read();
    if (marker != POLYGON_FILE_MARKER) {
        CONSOLE.println("ERROR reading polygon: invalid marker");
        return false;
    }
    
    short num = 0;
    file.read((uint8_t*)&num, sizeof(num));
    
    if (!alloc(num)) return false;
    
    for (short i = 0; i < num; i++) {
        if (!points[i].read(file)) return false;
    }
    
    return true;
}

bool Polygon::write(File &file) {
    if (file.write(POLYGON_FILE_MARKER) == 0) return false;
    
    if (file.write((uint8_t*)&numPoints, sizeof(numPoints)) == 0) {
        CONSOLE.println("ERROR writing polygon");
        return false;
    }
    
    for (int i = 0; i < numPoints; i++) {
        if (!points[i].write(file)) return false;
    }
    
    return true;
}

void Polygon::getCenter(Point &pt) {
    float minX = 9999;
    float maxX = -9999;
    float minY = 9999;
    float maxY = -9999;
    
    for (int i = 0; i < numPoints; i++) {
        minX = std::min(minX, points[i].x());
        maxX = std::max(maxX, points[i].x());
        minY = std::min(minY, points[i].y());
        maxY = std::max(maxY, points[i].y());
    }
    
    pt.setXY((maxX - minX) / 2, (maxY - minY) / 2);
}