// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
/*
  Point class implementation - 2D coordinate point on the map
*/

#include "Point.h"
#include "robot.h"

// Default constructor - initializes point at origin (0,0)
Point::Point(){
  init();
}

// Constructor with coordinates in meters (stored internally as cm for precision)
Point::Point(float ax, float ay){
  px = ax * METERS_TO_CM_FACTOR;
  py = ay * METERS_TO_CM_FACTOR;
}

// Initialize point coordinates to origin
void Point::init(){
  px = 0;
  py = 0;
}

// Get X coordinate in meters (converts from internal cm storage)
float Point::x(){
  return ((float)px) / METERS_TO_CM_FACTOR;
}

// Get Y coordinate in meters (converts from internal cm storage)
float Point::y(){
  return ((float)py) / METERS_TO_CM_FACTOR;
}

// Set coordinates in meters (converted to cm for internal storage)
void Point::setXY(float ax, float ay){
  px = ax * METERS_TO_CM_FACTOR;
  py = ay * METERS_TO_CM_FACTOR;
}

// Copy coordinates from another point
void Point::assign(Point &fromPoint){
  px = fromPoint.px;
  py = fromPoint.py;
}

// Calculate simple checksum for data integrity verification
long Point::crc(){
  return (px + py);  
}

// Read point data from file with integrity check (0xAA marker)
bool Point::read(File &file){
  byte marker = file.read();
  if (marker != POINT_FILE_MARKER){
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

// Write point data to file with integrity marker (0xAA)
bool Point::write(File &file){
  bool res = true;
  res &= (file.write(POINT_FILE_MARKER) != 0);
  res &= (file.write((uint8_t*)&px, sizeof(px)) != 0);
  res &= (file.write((uint8_t*)&py, sizeof(py)) != 0);
  if (!res) {
    CONSOLE.println("ERROR writing point");
  }
  return res;
}