// Polygon.cpp - Implementation of Polygon and PolygonList classes
// Part of Sunray Arduino robot mapping system

#include "Polygon.h"
#include "robot.h"
#include "helper.h"

// Memory corruption detection
#define CHECK_CORRUPT   1
#define CHECK_ID        0x4A4A
extern Point *CHECK_POINT;
extern unsigned long memoryCorruptions;
extern unsigned long memoryAllocErrors;

// -----------------------------------
// Polygon class implementation
// -----------------------------------

// Default constructor - creates empty polygon
Polygon::Polygon(){
  init();
}

// Constructor with pre-allocated point capacity
Polygon::Polygon(short aNumPoints){
  init();
  alloc(aNumPoints);
}

// Initialize polygon to empty state
void Polygon::init(){
  numPoints = 0;
  points = NULL;
}

// Destructor - memory cleanup handled by explicit dealloc() calls
Polygon::~Polygon(){
  // dealloc();
}

// Allocate memory for polygon points with corruption detection
bool Polygon::alloc(short aNumPoints){
  if (aNumPoints == numPoints) return true;
  if ((aNumPoints < 0) || (aNumPoints > MAX_POLYGON_POINTS)) {
    CONSOLE.println("ERROR Polygon::alloc invalid number");
    return false;
  }
  Point* newPoints = new Point[aNumPoints+CHECK_CORRUPT];
  if (newPoints == NULL) {
    CONSOLE.println("ERROR Polygon::alloc out of memory");
    memoryAllocErrors++;
    return false;
  }
  if (points != NULL){
    memcpy(newPoints, points, sizeof(Point)* min(numPoints, aNumPoints));
    if (aNumPoints < numPoints){
      for (int i=aNumPoints; i < numPoints; i++){
        // points[i].dealloc();
      }
    }
    if (points[numPoints].px != CHECK_POINT->px) memoryCorruptions++;
    delete[] points;
  }
  points = newPoints;
  numPoints = aNumPoints;
  points[numPoints].px = CHECK_POINT->px;
  points[numPoints].py = CHECK_POINT->py;
  return true;
}

// Deallocate polygon memory with corruption check
void Polygon::dealloc(){
  if (points == NULL) return;
  if (points[numPoints].px != CHECK_POINT->px) memoryCorruptions++;
  delete[] points;
  points = NULL;
  numPoints = 0;
}

// Debug output - print polygon points to console
void Polygon::dump(){
  for (int i=0; i < numPoints; i++){
    CONSOLE.print("(");
    CONSOLE.print(points[i].x());
    CONSOLE.print(",");
    CONSOLE.print(points[i].y());
    CONSOLE.print(") ");
  }
  CONSOLE.println();
}

// Calculate checksum for data integrity verification
long Polygon::crc(){
  long crc = 0;
  for (int i=0; i < numPoints; i++){
    crc += points[i].crc();
  }
  return crc;
}

// Read polygon data from file with integrity check (0xBB marker)
bool Polygon::read(File &file){
  byte marker = file.read();
  if (marker != POLYGON_FILE_MARKER){
    CONSOLE.println("ERROR reading polygon: invalid marker");
    return false;
  }
  short num = 0;
  file.read((uint8_t*)&num, sizeof(num));

  if (!alloc(num)) return false;
  for (short i=0; i < num; i++){
    if (!points[i].read(file)) return false;
  }
  return true;
}

// Write polygon data to file with integrity marker (0xBB)
bool Polygon::write(File &file){
  if (file.write(POLYGON_FILE_MARKER) == 0) return false;
  if (file.write((uint8_t*)&numPoints, sizeof(numPoints)) == 0) {
    CONSOLE.println("ERROR writing polygon");
    return false;
  }

  for (int i=0; i < numPoints; i++){
    if (!points[i].write(file)) return false;
  }
  return true;
}

// Calculate polygon center point using bounding box method
void Polygon::getCenter(Point &pt){
  float minX = POLYGON_BOUNDS_INIT;
  float maxX = -POLYGON_BOUNDS_INIT;
  float minY = POLYGON_BOUNDS_INIT;
  float maxY = -POLYGON_BOUNDS_INIT;
  for (int i=0; i < numPoints; i++){
    minX = min(minX, points[i].x());
    maxX = max(maxX, points[i].x());
    minY = min(minY, points[i].y());
    maxY = max(maxY, points[i].y());
  }
  pt.setXY( (maxX-minX)/2, (maxY-minY)/2 );
}

// -----------------------------------
// PolygonList class implementation
// -----------------------------------

// Default constructor - creates empty polygon list
PolygonList::PolygonList(){
  init();
}

// Constructor with pre-allocated polygon capacity
PolygonList::PolygonList(short aNumPolygons){
  init();
  alloc(aNumPolygons);
}

// Initialize polygon list to empty state
void PolygonList::init(){
  numPolygons = 0;
  polygons = NULL;
}

// Destructor - memory cleanup handled by explicit dealloc() calls
PolygonList::~PolygonList(){
}

// Allocate memory for polygon list with corruption detection
bool PolygonList::alloc(short aNumPolygons){
  if (aNumPolygons == numPolygons) return true;
  if ((aNumPolygons < 0) || (aNumPolygons > MAX_POLYGONS_IN_LIST)) {
    CONSOLE.println("ERROR PolygonList::alloc invalid number");
    return false;
  }
  Polygon* newPolygons = new Polygon[aNumPolygons+CHECK_CORRUPT];
  if (newPolygons == NULL){
    CONSOLE.println("ERROR PolygonList::alloc out of memory");
    memoryAllocErrors++;
    return false;
  }
  if (polygons != NULL){
    memcpy(newPolygons, polygons, sizeof(Polygon)* min(numPolygons, aNumPolygons));
    if (aNumPolygons < numPolygons){
      for (int i=aNumPolygons; i < numPolygons; i++){
        // Additional cleanup if needed
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

// Deallocate polygon list memory with corruption check
void PolygonList::dealloc(){
  if (polygons == NULL) return;
  for (int i=0; i < numPolygons; i++){
    polygons[i].dealloc();
  }
  if (polygons[numPolygons].points != CHECK_POINT) memoryCorruptions++;
  delete[] polygons;
  polygons = NULL;
  numPolygons = 0;
}

// Calculate total number of points across all polygons
int PolygonList::numPoints(){
  int num = 0;
  for (int i=0; i < numPolygons; i++){
     num += polygons[i].numPoints;
  }
  return num;
}

// Debug output - print all polygons with indices to console
void PolygonList::dump(){
  for (int i=0; i < numPolygons; i++){
    CONSOLE.print(i);
    CONSOLE.print(":");
    polygons[i].dump();
  }
  CONSOLE.println();
}

// Calculate checksum for data integrity verification
long PolygonList::crc(){
  long crc = 0;
  for (int i=0; i < numPolygons; i++){
    crc += polygons[i].crc();
  }
  return crc;
}

// Read polygon list from file with integrity check (0xCC marker)
bool PolygonList::read(File &file){
  byte marker = file.read();
  if (marker != POLYGONLIST_FILE_MARKER){
    CONSOLE.println("ERROR reading polygon list: invalid marker");
    return false;
  }
  short num = 0;
  file.read((uint8_t*)&num, sizeof(num));

  if (!alloc(num)) return false;
  for (short i=0; i < num; i++){
    if (!polygons[i].read(file)) return false;
  }
  return true;
}

// Write polygon list to file with integrity marker (0xCC)
bool PolygonList::write(File &file){
  if (file.write(POLYGONLIST_FILE_MARKER) == 0) {
    CONSOLE.println("ERROR writing polygon list marker");
    return false;
  }
  if (file.write((uint8_t*)&numPolygons, sizeof(numPolygons)) == 0) {
    CONSOLE.println("ERROR writing polygon list");
    return false;
  }

  for (int i=0; i < numPolygons; i++){
    if (!polygons[i].write(file)) return false;
  }
  return true;
}