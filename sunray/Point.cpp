// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Point class implementation

#include "Point.h"
#include "robot.h" // For CONSOLE macro

// File format constants
#define POINT_FILE_MARKER 0xAA

Point::Point() {
    init();
}

Point::Point(float ax, float ay) {
    px = ax * 100;
    py = ay * 100;
}

void Point::init() {
    px = 0;
    py = 0;
}

float Point::x() {
    return ((float)px) / 100.0;
}

float Point::y() {
    return ((float)py) / 100.0;
}

void Point::setXY(float ax, float ay) {
    px = ax * 100;
    py = ay * 100;
}

void Point::assign(Point &fromPoint) {
    px = fromPoint.px;
    py = fromPoint.py;
}

long Point::crc() {
    return (px + py);
}

bool Point::read(File &file) {
    byte marker = file.read();
    if (marker != POINT_FILE_MARKER) {
        CONSOLE.println("ERROR reading point: invalid marker");
        return false;
    }
    
    bool res = true;
    res &= (file.read((uint8_t*)&px, sizeof(px)) != 0);
    res &= (file.read((uint8_t*)&py, sizeof(py)) != 0);
    
    if (!res) {
        CONSOLE.println("ERROR reading point");
    }
    
    return res;
}

bool Point::write(File &file) {
    bool res = true;
    res &= (file.write(POINT_FILE_MARKER) != 0);
    res &= (file.write((uint8_t*)&px, sizeof(px)) != 0);
    res &= (file.write((uint8_t*)&py, sizeof(py)) != 0);
    
    if (!res) {
        CONSOLE.println("ERROR writing point");
    }
    
    return res;
}