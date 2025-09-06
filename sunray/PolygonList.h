// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// PolygonList class definition

#ifndef POLYGON_LIST_H
#define POLYGON_LIST_H

#include "Polygon.h"
#include <SD.h>

// File format constants
#define POLYGON_LIST_FILE_MARKER 0xCC

class PolygonList {
public:
    short numPolygons;
    Polygon* polygons;
    
    PolygonList();
    PolygonList(short aNumPolygons);
    ~PolygonList();
    
    void init();
    bool alloc(short aNumPolygons);
    void dealloc();
    int numPoints();
    void dump();
    long crc();
    bool read(File &file);
    bool write(File &file);
};

#endif // POLYGON_LIST_H