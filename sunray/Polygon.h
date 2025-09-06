// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Polygon class definition

#ifndef POLYGON_H
#define POLYGON_H

#include "Point.h"
#include <SD.h>

// File format constants
#define POLYGON_FILE_MARKER 0xBB

class Polygon {
public:
    short numPoints;
    Point* points;
    
    Polygon();
    Polygon(short aNumPoints);
    ~Polygon();
    
    void init();
    bool alloc(short aNumPoints);
    void dealloc();
    void dump();
    long crc();
    bool read(File &file);
    bool write(File &file);
    void getCenter(Point &pt);
};

#endif // POLYGON_H