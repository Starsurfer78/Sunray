// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH


#include "map.h"
#include "robot.h"
#include "config.h"
#include "StateEstimator.h"
#include "events.h"
#include "helper.h"
#include "src/memory_guard.h"
#include "src/stack_allocator.h"
#include <Arduino.h>
#include <cfloat>


#ifndef _SAM3XA_                 // not Arduino Due
  #define FLOAT_CALC    1    // comment out line for using integer calculations instead of float  
#endif

// Legacy memory corruption checking - migrating to MemoryGuard system
// we check for memory corruptions by storing one additional item in all dynamic arrays and 
// checking the value of the item during free operation
#define CHECK_CORRUPT   1
#define CHECK_ID        MemoryGuard::GUARD_MAGIC_ID

Point *CHECK_POINT = (Point*)MemoryGuard::GUARD_MAGIC_PTR;  // using MemoryGuard magic pointer

// Legacy counters - will be replaced by MemoryGuard in future versions
unsigned long memoryCorruptions = 0;        
unsigned long memoryAllocErrors = 0;


Point::Point(){
  init();
}

void Point::init(){
  px = 0;
  py = 0;
}

float Point::x(){
  return ((float)px) / 100.0;
}

float Point::y(){
  return ((float)py) / 100.0;
}


Point::Point(float ax, float ay){
  px = ax * 100;
  py = ay * 100;
}

void Point::assign(Point &fromPoint){
  px = fromPoint.px;
  py = fromPoint.py;
}

void Point::setXY(float ax, float ay){
  px = ax * 100;
  py = ay * 100;
}

long Point::crc(){
  return (px + py);  
}

bool Point::read(File &file){
  byte marker = file.read();
  if (marker != 0xAA){
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

bool Point::write(File &file){
  bool res = true;
  res &= (file.write(0xAA) != 0);
  res &= (file.write((uint8_t*)&px, sizeof(px)) != 0);
  res &= (file.write((uint8_t*)&py, sizeof(py)) != 0);
  if (!res) {
    CONSOLE.println("ERROR writing point");
  }
  return res;
}


// -----------------------------------
Polygon::Polygon(){  
  init();
}


Polygon::Polygon(short aNumPoints){ 
  init();
  alloc(aNumPoints);  
}

void Polygon::init(){  
  numPoints = 0;  
  points = NULL;
  stackAllocated = false;
}

Polygon::~Polygon(){
  // dealloc();
}

bool Polygon::alloc(short aNumPoints){
  if (aNumPoints == numPoints) return true;
  if ((aNumPoints < 0) || (aNumPoints > 10000)) {
    CONSOLE.println("ERROR Polygon::alloc invalid number");    
    return false;
  }
  // Try stack-based allocation for small arrays first
  bool usedStack = false;
  Point* newPoints = StackAllocator::allocatePoints(aNumPoints+CHECK_CORRUPT, usedStack);
  if (newPoints == NULL) {
    CONSOLE.println("ERROR Polygon::alloc out of memory");
    memoryAllocErrors++;
    return false;
  }
  
  if (points != NULL){
    memcpy(newPoints, points, sizeof(Point)* min(numPoints,aNumPoints) );        
    if (points[numPoints].px != CHECK_ID) memoryCorruptions++;
    if (points[numPoints].py != CHECK_ID) memoryCorruptions++;
    
    // Properly deallocate old points
    StackAllocator::deallocatePoints(points, stackAllocated);
    if (stackAllocated) {
      StackAllocator::releaseBuffer(points);
    }
  }
  
  // Store allocation method for proper deallocation
  stackAllocated = usedStack; 
  points = newPoints;              
  numPoints = aNumPoints;
  points[numPoints].px=CHECK_ID;
  points[numPoints].py=CHECK_ID;
  return true;
}

void Polygon::dealloc(){
  if (points == NULL) return;  
  if (points[numPoints].px != CHECK_ID) memoryCorruptions++;
  if (points[numPoints].py != CHECK_ID) memoryCorruptions++;
  
  // Use StackAllocator for proper deallocation
  StackAllocator::deallocatePoints(points, stackAllocated);
  if (stackAllocated) {
    StackAllocator::releaseBuffer(points);
  }
  
  points = NULL;
  numPoints = 0;
  stackAllocated = false;
}

void Polygon::dump(){
  for (int i=0; i < numPoints; i++){
    CONSOLE.print("(");
    CONSOLE.print(points[i].x());
    CONSOLE.print(",");
    CONSOLE.print(points[i].y());
    CONSOLE.print(")");   
    if (i < numPoints-1) CONSOLE.print(",");
  }
  CONSOLE.println();
}

long Polygon::crc(){
  long crc = 0;
  for (int i=0; i < numPoints; i++){
    crc += points[i].crc();
  }
  return crc;
}

bool Polygon::read(File &file){
  byte marker = file.read();
  if (marker != 0xBB){
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

bool Polygon::write(File &file){
  if (file.write(0xBB) == 0) return false;  
  if (file.write((uint8_t*)&numPoints, sizeof(numPoints)) == 0) {
    CONSOLE.println("ERROR writing polygon");
    return false; 
  }

  for (int i=0; i < numPoints; i++){    
    if (!points[i].write(file)) return false;
  }
  return true;  
}

void Polygon::getCenter(Point &pt){
  float minX = 9999;
  float maxX = -9999;
  float minY = 9999;
  float maxY = -9999;
  for (int i=0; i < numPoints; i++){
    minX = min(minX, points[i].x());
    maxX = max(maxX, points[i].x());
    minY = min(minY, points[i].y());
    maxY = max(maxY, points[i].y());
  }
  pt.setXY( (maxX-minX)/2, (maxY-minY)/2 ); 
}

// -----------------------------------

PolygonList::PolygonList(){
  init();
}
  
PolygonList::PolygonList(short aNumPolygons){
  init();
  alloc(aNumPolygons);  
}

void PolygonList::init(){
  numPolygons = 0;
  polygons = NULL;  
}

PolygonList::~PolygonList(){
  //dealloc();
}

bool PolygonList::alloc(short aNumPolygons){  
  if (aNumPolygons == numPolygons) return true;
  if ((aNumPolygons < 0) || (aNumPolygons > 5000)) {
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
        //polygons[i].dealloc();        
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

int PolygonList::numPoints(){
  int num = 0;
  for (int i=0; i < numPolygons; i++){
     num += polygons[i].numPoints;
  }
  return num;
}

void PolygonList::dump(){
  for (int i=0; i < numPolygons; i++){
    CONSOLE.print(i);
    CONSOLE.print(":");
    polygons[i].dump();
  }  
  CONSOLE.println();
}

long PolygonList::crc(){
  long crc = 0;
  for (int i=0; i < numPolygons; i++){
    crc += polygons[i].crc();
  }
  return crc;
}

bool PolygonList::read(File &file){
  byte marker = file.read();
  if (marker != 0xCC){
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

bool PolygonList::write(File &file){
  if (file.write(0xCC) == 0) {
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


// -----------------------------------

Node::Node(){
  init();
}

Node::Node(Point *aPoint, Node *aParentNode){
  init();
  point = aPoint;
  parent = aParentNode;  
};

void Node::init(){
  g = 0;
  h = 0;  
  f = 0;
  opened = false;
  closed = false;
  point = NULL;
  parent = NULL;
}

void Node::dealloc(){
}


// -----------------------------------



NodeList::NodeList(){
  init();
}
  
NodeList::NodeList(short aNumNodes){
  init();
  alloc(aNumNodes);  
}

void NodeList::init(){
  numNodes = 0;
  nodes = NULL;
  stackAllocated = false;
}

NodeList::~NodeList(){
  //dealloc();
}

bool NodeList::alloc(short aNumNodes){  
  if (aNumNodes == numNodes) return true;
  if ((aNumNodes < 0) || (aNumNodes > 20000)) {
    CONSOLE.println("ERROR NodeList::alloc invalid number");    
    return false;
  }
  
  // Try stack-based allocation for small arrays first
  bool usedStack = false;
  // Use regular malloc for Node allocation since StackAllocator only handles Points
  Node* newNodes = (Node*)malloc((aNumNodes+CHECK_CORRUPT) * sizeof(Node));
  usedStack = false;
  if (newNodes == NULL) {
    CONSOLE.println("ERROR NodeList::alloc out of memory");
    memoryAllocErrors++;
    return false;
  }
  
  if (nodes != NULL){
    memcpy(newNodes, nodes, sizeof(Node)* min(numNodes, aNumNodes));        
    if (aNumNodes < numNodes){
      for (int i=aNumNodes; i < numNodes; i++){
        //nodes[i].dealloc();        
      }  
    }
    if (nodes[numNodes].point != CHECK_POINT) memoryCorruptions++;
    
    // Properly deallocate old nodes
     if (stackAllocated) {
       // This should not happen with current implementation
     } else {
       free(nodes);
     }
  } 
  
  // Store allocation method for proper deallocation
  stackAllocated = usedStack;
  nodes = newNodes;              
  numNodes = aNumNodes;  
  nodes[numNodes].point=CHECK_POINT;
  return true;
}

void NodeList::dealloc(){
  if (nodes == NULL) return;
  for (int i=0; i < numNodes; i++){
    nodes[i].dealloc();        
  }  
  if (nodes[numNodes].point != CHECK_POINT) memoryCorruptions++;
  
  // Use StackAllocator for proper deallocation
  if (stackAllocated) {
    // This should not happen with current implementation
  } else {
    free(nodes);
  }
  
  nodes = NULL;
  numNodes = 0;
  stackAllocated = false;
}



// ---------------------------------------------------------------------

// rescale to -PI..+PI
float Map::scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) return (-2*PI+d);
  else if (d < -PI) return (2*PI+d);
  else return d;
}


// scale setangle, so that both PI angles have the same sign
float Map::scalePIangles(float setAngle, float currAngle){
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}


// compute course (angle in rad) between two points
float Map::pointsAngle(float x1, float y1, float x2, float y2){
  float dX = x2 - x1;
  float dY = y2 - y1;
  float angle = scalePI(atan2(dY, dX));           
  return angle;
}



// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float Map::distancePI(float x, float w)
{
  // cases:
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree
  float d = scalePI(w - x);
  if (d < -PI) d = d + 2*PI;
  else if (d > PI) d = d - 2*PI;
  return d; 
}

// This is the Manhattan distance
float Map::distanceManhattan(Point &pos0, Point &pos1){
  float d1 = abs (pos1.x() - pos0.x());
  float d2 = abs (pos1.y() - pos0.y());
  return d1 + d2;
}



void Map::begin(){
  CONSOLE.println("Map::begin");
  memoryCorruptions = 0;
  wayMode = WAY_MOW;
  trackReverse = false;
  trackSlow = false;
  useGPSfixForPosEstimation = true;
  useGPSfloatForPosEstimation = true;
  useGPSfloatForDeltaEstimation = true;
  useGPSfixForDeltaEstimation = true;
  useIMU = true;
  mowPointsIdx = 0;
  freePointsIdx = 0;
  dockPointsIdx = 0;
  shouldDock = false; 
  shouldRetryDock = false; 
  shouldMow = false;         
  mapCRC = 0;  
  CONSOLE.print("sizeof Point=");
  CONSOLE.println(sizeof(Point));  
  load();
  dump();
  //stressTestMapTransfer();
  //float distToLine = distanceLineInfinite(12.43, 6.18, 12.42, 6.18, 12.43, 6.18);  
}

long Map::calcMapCRC(){   
  long crc = perimeterPoints.crc() + exclusions.crc() + dockPoints.crc() + mowPoints.crc();       
  
  return crc;
}

void Map::dump(){ 
  CONSOLE.print("map dump - mapCRC=");
  CONSOLE.println(mapCRC);
  CONSOLE.print("points: ");
  points.dump();
  CONSOLE.print("perimeter pts: ");
  CONSOLE.println(perimeterPoints.numPoints);
  //perimeterPoints.dump();
  CONSOLE.print("exclusion pts: ");
  CONSOLE.println(exclusionPointsCount);  
  CONSOLE.print("exclusions: ");  
  CONSOLE.println(exclusions.numPolygons);  
  //exclusions.dump();  
  CONSOLE.print("dock pts: ");
  CONSOLE.println(dockPoints.numPoints);
  //dockPoints.dump();
  CONSOLE.print("mow pts: ");  
  CONSOLE.println(mowPoints.numPoints);  
  // Map dump information (debug output removed for performance)
  // First mow point, free points count, and indices available internally
  checkMemoryErrors();
}


void Map::checkMemoryErrors(){
  // Use new MemoryGuard system for better error reporting
  MemoryGuard::checkAndReport();
  
  // Legacy error checking for backward compatibility
  if (memoryCorruptions != 0){
    CONSOLE.print("********************* ERROR: legacy memoryCorruptions=");
    CONSOLE.println(memoryCorruptions);
    CONSOLE.println(" *********************");
  } 
  if (memoryAllocErrors != 0){
    CONSOLE.print("********************* ERROR: legacy memoryAllocErrors=");
    CONSOLE.println(memoryAllocErrors);
    CONSOLE.println(" *********************");
  } 
}


bool Map::load(){
  bool res = true;
#if defined(ENABLE_SD_RESUME)  
  CONSOLE.print("map load... ");
  if (!SD.exists("map.bin")) {
    CONSOLE.println("no map file!");
    return false;
  }
  mapFile = SD.open("map.bin", FILE_READ);
  if (!mapFile){        
    CONSOLE.println("ERROR opening file for reading");
    return false;
  }
  uint32_t marker = 0;
  mapFile.read((uint8_t*)&marker, sizeof(marker));
  if (marker != 0x00001000){
    CONSOLE.print("ERROR: invalid marker: ");
    CONSOLE.println(marker, HEX);
    return false;
  }
  res &= (mapFile.read((uint8_t*)&mapCRC, sizeof(mapCRC)) != 0); 
  res &= (mapFile.read((uint8_t*)&exclusionPointsCount, sizeof(exclusionPointsCount)) != 0);     
  res &= perimeterPoints.read(mapFile);
  res &= exclusions.read(mapFile);    
  res &= dockPoints.read(mapFile);
  res &= mowPoints.read(mapFile);        
  
  mapFile.close();  
  long expectedCRC = calcMapCRC();
  if (mapCRC != expectedCRC){
    CONSOLE.print("ERROR: invalid map CRC:");
    CONSOLE.print(mapCRC);
    CONSOLE.print(" expected:");
    CONSOLE.println(expectedCRC);
    res = false;
  }
  if (res){
    CONSOLE.println("ok");
  } else {
    CONSOLE.println("ERROR loading map");
    clearMap(); 
  }
#endif
  return res;
}


bool Map::save(){
  bool res = true;
#if defined(ENABLE_SD_RESUME)  
  CONSOLE.print("map save... ");
  mapFile = SD.open("map.bin", FILE_CREATE); // O_WRITE | O_CREAT);
  if (!mapFile){        
    CONSOLE.println("ERROR opening file for writing");
    return false;
  }
  uint32_t marker = 0x00001000;
  res &= (mapFile.write((uint8_t*)&marker, sizeof(marker)) != 0);
  res &= (mapFile.write((uint8_t*)&mapCRC, sizeof(mapCRC)) != 0);
  res &= (mapFile.write((uint8_t*)&exclusionPointsCount, sizeof(exclusionPointsCount)) != 0);
  if (res){
    res &= perimeterPoints.write(mapFile);
    res &= exclusions.write(mapFile);    
    res &= dockPoints.write(mapFile);
    res &= mowPoints.write(mapFile);        
  }      
  if (res){
    CONSOLE.println("ok");
  } else {
    CONSOLE.println("ERROR saving map");
  }
  mapFile.flush();
  mapFile.close();
#endif
  return res;    
}



void Map::finishedUploadingMap(){
  CONSOLE.println("finishedUploadingMap");
  #ifdef DRV_SIM_ROBOT
    float x;
    float y;
    float delta;
    if (getDockingPos(x, y, delta)){
      CONSOLE.println("SIM: setting robot pos to docking pos");      
      if (!DOCK_FRONT_SIDE) delta = scalePI(delta + 3.1415);
      robotDriver.setSimRobotPosState(x, y, delta);
    } else {
      CONSOLE.println("SIM: error getting docking pos");
      if (perimeterPoints.numPoints > 0){
        Point pt = perimeterPoints.points[0];
        //perimeterPoints.getCenter(pt);
        robotDriver.setSimRobotPosState(pt.x(), pt.y(), 0);
      }
    }
  #endif
  mapCRC = calcMapCRC();
  Logger.event(EVT_USER_UPLOAD_MAP);
  dump();
  save();
}
 
   
void Map::clearMap(){
  CONSOLE.println("clearMap");
  points.dealloc();    
  perimeterPoints.dealloc();    
  exclusions.dealloc();
  dockPoints.dealloc();      
  mowPoints.dealloc();    
  freePoints.dealloc();
  obstacles.dealloc();
  pathFinderObstacles.dealloc();
  pathFinderNodes.dealloc();
}

 
// set point
bool Map::setPoint(int idx, float x, float y){  
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR setPoint: memory errors");
    return false; 
  }  
  if (idx == 0){   
    clearMap();
  }    
  if (idx % 100 == 0){
    if (freeMemory () < 20000){
      CONSOLE.println("OUT OF MEMORY");
      return false;
    }
  }
  if (points.alloc(idx+1)){
    points.points[idx].setXY(x, y);      
    return true;
  }
  return false;
}


// set number points for point type
// transfers points from global point list to specific point list
bool Map::setWayCount(WayType type, int count){
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR setWayCount: memory errors");
    return false; 
  }  
  switch (type){
    case WAY_PERIMETER:            
      if (perimeterPoints.alloc(count)) {
        for (int i=0; i < count; i++){
          int sidx = i;
          if (sidx < points.numPoints) perimeterPoints.points[i].assign( points.points[sidx] );
        }
      }
      break;
    case WAY_EXCLUSION:      
      exclusionPointsCount = count;                  
      break;
    case WAY_DOCK:    
      if (dockPoints.alloc(count)){
        for (int i=0; i < count; i++){
          int sidx = perimeterPoints.numPoints + exclusionPointsCount + i;
          if (sidx < points.numPoints) dockPoints.points[i].assign( points.points[ sidx ] );
        }
      }
      break;
    case WAY_MOW:          
      if (mowPoints.alloc(count)){
        for (int i=0; i < count; i++){
          int sidx = perimeterPoints.numPoints + exclusionPointsCount + dockPoints.numPoints + i;
          if (sidx < points.numPoints) mowPoints.points[i].assign( points.points[ sidx ] );
        }
        if (exclusionPointsCount == 0){
          points.dealloc(); // free point list
          finishedUploadingMap();
        }
      }
      break;    
    case WAY_FREE:      
      break;
    default: 
      return false;       
  }
  mowPointsIdx = 0;
  dockPointsIdx = 0;
  freePointsIdx = 0;  
  return true;
}


// set number exclusion points for exclusion
bool Map::setExclusionLength(int idx, int len){  
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR setExclusionLength: memory errors");
    return false; 
  }  
  /*CONSOLE.print("setExclusionLength ");
  CONSOLE.print(idx);
  CONSOLE.print("=");
  CONSOLE.println(len);*/
    
  if (!exclusions.alloc(idx+1)) return false;
  if (!exclusions.polygons[idx].alloc(len)) return false;
  
  
  int ptIdx = 0;  
  for (int i=0; i < idx; i++){    
    ptIdx += exclusions.polygons[i].numPoints;    
  }    
  for (int j=0; j < len; j++){
    int sidx =  perimeterPoints.numPoints + ptIdx;
    if (sidx < points.numPoints) exclusions.polygons[idx].points[j].assign( points.points[ sidx ] );        
    ptIdx ++;
  }
  CONSOLE.print("ptIdx=");
  CONSOLE.print(ptIdx);
  CONSOLE.print(" exclusionPointsCount=");
  CONSOLE.print(exclusionPointsCount);
  CONSOLE.println();
  if (ptIdx == exclusionPointsCount){
    points.dealloc();
    finishedUploadingMap();
  }
  
   
  return true;
}


// visualize result with:  https://www.graphreader.com/plotter
void Map::generateRandomMap(){
  CONSOLE.println("Map::generateRandomMap");
  clearMap();    
  int idx = 0;
  float angle = 0;
  int steps = 30;
  for (int i=0; i < steps; i++){
    float maxd = 10;
    float d = 10 + ((float)random(maxd*10))/10.0;  // -d/2;
    float x = cos(angle) * d;
    float y = sin(angle) * d; 
    setPoint(idx, x, y);
    //CONSOLE.print(idx);
    //CONSOLE.print(",");
    CONSOLE.print(x);
    CONSOLE.print(",");    
    CONSOLE.println(y);
    angle += 2*PI / ((float)steps);
    idx++;    
  }
  setWayCount(WAY_PERIMETER, steps);
  setWayCount(WAY_EXCLUSION, 0);
  setWayCount(WAY_DOCK, 0);
  setWayCount(WAY_MOW, 0);
}


// set desired progress in mowing points list
// 1.0 = 100%
// TODO: use path finder for valid free points to target point
void Map::setMowingPointPercent(float perc){
  if (mowPoints.numPoints == 0) return;
  mowPointsIdx = (int)( ((float)mowPoints.numPoints) * perc);
  if (mowPointsIdx >= mowPoints.numPoints) {
    mowPointsIdx = mowPoints.numPoints-1;
  }
}

void Map::skipNextMowingPoint(){
  if (mowPoints.numPoints == 0) return;
  mowPointsIdx++;
  if (mowPointsIdx >= mowPoints.numPoints) {
    mowPointsIdx = mowPoints.numPoints-1;
  }  
}


void Map::repeatLastMowingPoint(){
  if (mowPoints.numPoints == 0) return;
  if (mowPointsIdx > 1) {
    mowPointsIdx--;
  }
}

void Map::run(){
  switch (wayMode){
    case WAY_DOCK:      
      if (dockPointsIdx < dockPoints.numPoints){
        targetPoint.assign( dockPoints.points[dockPointsIdx] );
      }
      break;
    case WAY_MOW:
      if (mowPointsIdx < mowPoints.numPoints){
        targetPoint.assign( mowPoints.points[mowPointsIdx] );
      }
      break;
    case WAY_FREE:      
      if (freePointsIdx < freePoints.numPoints){
        targetPoint.assign(freePoints.points[freePointsIdx]);
      }
      break;
  } 
  percentCompleted = (((float)mowPointsIdx) / ((float)mowPoints.numPoints) * 100.0);
}

float Map::distanceToTargetPoint(float stateX, float stateY){  
  float dX = targetPoint.x() - stateX;
  float dY = targetPoint.y() - stateY;
  float targetDist = sqrt( sq(dX) + sq(dY) );    
  return targetDist;
}

float Map::distanceToLastTargetPoint(float stateX, float stateY){  
  float dX = lastTargetPoint.x() - stateX;
  float dY = lastTargetPoint.y() - stateY;
  float targetDist = sqrt( sq(dX) + sq(dY) );    
  return targetDist;
}

// check if path from last target to target to next target is a curve
bool Map::nextPointIsStraight(){
  if (wayMode != WAY_MOW) return false;
  if (mowPointsIdx+1 >= mowPoints.numPoints) return false;     
  Point nextPt;
  nextPt.assign(mowPoints.points[mowPointsIdx+1]);  
  float angleCurr = pointsAngle(lastTargetPoint.x(), lastTargetPoint.y(), targetPoint.x(), targetPoint.y());
  float angleNext = pointsAngle(targetPoint.x(), targetPoint.y(), nextPt.x(), nextPt.y());
  angleNext = scalePIangles(angleNext, angleCurr);                    
  float diffDelta = distancePI(angleCurr, angleNext);                 
  //CONSOLE.println(fabs(diffDelta)/PI*180.0);
  return ((fabs(diffDelta)/PI*180.0) < 12); // reduced from 20° to 12° for more precise curve driving
}


// get docking position and orientation (x,y,delta)
bool Map::getDockingPos(float &x, float &y, float &delta, int idx){
  if (idx == -1) idx = dockPoints.numPoints-1; 
  if ((idx < 0) || (idx >= dockPoints.numPoints) || (dockPoints.numPoints < 2)) return false;
  Point dockFinalPt;
  Point dockPrevPt;
  dockFinalPt.assign(dockPoints.points[ idx]);  
  if (idx > 0) dockPrevPt.assign(dockPoints.points[ idx-1]);
    else dockPrevPt.assign(dockPoints.points[ 1 ]);
  x = dockFinalPt.x();
  y = dockFinalPt.y();
  delta = pointsAngle(dockPrevPt.x(), dockPrevPt.y(), dockFinalPt.x(), dockFinalPt.y());  
  return true;
}             

// mower has been docked
void Map::setIsDocked(bool flag){
  //CONSOLE.print("Map::setIsDocked ");
  //CONSOLE.println(flag);
  if (flag){
    if (dockPoints.numPoints < 2) return; // keep current wayMode (not enough docking points for docking wayMode)  
    wayMode = WAY_DOCK;
    dockPointsIdx = dockPoints.numPoints-2;
    //targetPointIdx = dockStartIdx + dockPointsIdx;                     
    trackReverse = (DOCK_FRONT_SIDE);             
    trackSlow = true;
    useGPSfixForPosEstimation = !DOCK_IGNORE_GPS;
    useGPSfixForDeltaEstimation = !DOCK_IGNORE_GPS;    
    useGPSfloatForPosEstimation = false;  
    useGPSfloatForDeltaEstimation = false;
    useIMU = true; // false
  } else {
    wayMode = WAY_FREE;
    dockPointsIdx = 0;    
    trackReverse = false;             
    trackSlow = false;
    useGPSfixForPosEstimation = true;
    useGPSfixForDeltaEstimation = true;
    useGPSfloatForPosEstimation = true;    
    useGPSfloatForDeltaEstimation = true;
    useIMU = true;
  }  
}

bool Map::isUndocking(){
  return ((maps.wayMode == WAY_DOCK) && (maps.shouldMow));
}

bool Map::isDocking(){
  return ((maps.wayMode == WAY_DOCK) && (maps.shouldDock));
}

bool Map::isBetweenLastAndNextToLastDockPoint(){
  //return true;
  return (
      ((isUndocking()) && ((isTargetingLastDockPoint()) || (isTargetingNextToLastDockPoint())))  || 
      ((isDocking())   && (isTargetingLastDockPoint()))   
  );
}

bool Map::isTargetingLastDockPoint(){
  // is on the way to the last docking point
  return (maps.dockPointsIdx == maps.dockPoints.numPoints-1);
}

bool Map::isTargetingNextToLastDockPoint(){
  // is on the way to the next-to-last docking point
  return (maps.dockPointsIdx == maps.dockPoints.numPoints-2);
}

bool Map::retryDocking(float stateX, float stateY){
  CONSOLE.println("Map::retryDocking");    
  if (!shouldDock) {
    CONSOLE.println("ERROR retryDocking: not docking!");
    return false;  
  }  
  if (shouldRetryDock) {
    CONSOLE.println("ERROR retryDocking: already retrying!");   
    return false;
  } 
  if (dockPointsIdx > 0) dockPointsIdx--;    
  shouldRetryDock = true;
  trackReverse = (DOCK_FRONT_SIDE) || ((!DOCK_FRONT_SIDE) && (dockPointsIdx < dockPoints.numPoints-3));
  return true;
}


bool Map::startDocking(float stateX, float stateY){  
  // Starting docking sequence
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR startDocking: memory errors");
    return false; 
  }  
  shouldDock = true;
  shouldRetryDock = false;
  shouldMow = false;    
  if (dockPoints.numPoints > 0){
    if (wayMode == WAY_DOCK) {
      // Already in docking mode, skip path planning
      return true;
    }
    // find valid path from robot to first docking point      
    //freePoints.alloc(0);
    Point src;
    Point dst;
    src.setXY(stateX, stateY);    
    dst.assign(dockPoints.points[0]);        
    //findPathFinderSafeStartPoint(src, dst);      
    wayMode = WAY_FREE;              
    if (findPath(src, dst)){      
      return true;
    } else {
      // No valid path found to docking point
      return false;
    }
  } else {
    // No docking points available
    Logger.event(EVT_ERROR_NO_MAP_POINTS);
    return false; 
  }
}

bool Map::startMowing(float stateX, float stateY){  
  // Starting mowing sequence
  //stressTest();
  //testIntegerCalcs();
  //return false;
  // ------
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    // Memory errors detected, cannot start mowing
    return false; 
  }  
  shouldDock = false;
  shouldRetryDock = false;
  shouldMow = true;    
  if (mowPoints.numPoints > 0){
    // find valid path from robot (or first docking point) to mowing point    
    //freePoints.alloc(0);
    Point src;
    Point dst;
    src.setXY(stateX, stateY);
    if ((wayMode == WAY_DOCK) && (dockPoints.numPoints > 0)) {
      src.assign(dockPoints.points[0]);
    } else {
      wayMode = WAY_FREE;      
      freePointsIdx = 0;    
    }        
    if (findObstacleSafeMowPoint(dst)){
      //dst.assign(mowPoints.points[mowPointsIdx]);      
      //findPathFinderSafeStartPoint(src, dst);      
      if (findPath(src, dst)){        
        return true;
      } else {
        CONSOLE.println("ERROR: no path");
        Logger.event(EVT_ERROR_NO_MAP_ROUTE);
        return false;      
      }
    } else {
      CONSOLE.println("ERROR: no safe start point");
      return false;
    }
  } else {
    Logger.event(EVT_ERROR_NO_MAP_POINTS);
    CONSOLE.println("ERROR: no points");
    return false; 
  }
}


void Map::clearObstacles(){  
  CONSOLE.println("clearObstacles");
  obstacles.dealloc();  
}

// add dynamic octagon obstacle in front of robot on line going from robot to target point
bool Map::addObstacle(float stateX, float stateY){     
  float d1 = OBSTACLE_DIAMETER / 6.0;   // distance from center to nearest octagon edges
  float d2 = OBSTACLE_DIAMETER / 2.0;  // distance from center to farest octagon edges
  
  float angleCurr = pointsAngle(stateX, stateY, targetPoint.x(), targetPoint.y());
  float r = d2 + 0.05;
  float x = stateX + cos(angleCurr) * r;
  float y = stateY + sin(angleCurr) * r;
  
  CONSOLE.print("addObstacle ");
  CONSOLE.print(x);
  CONSOLE.print(",");
  CONSOLE.println(y);
  if (obstacles.numPolygons > 50){
    CONSOLE.println("error: too many obstacles");
    return false;
  }
  int idx = obstacles.numPolygons;
  if (!obstacles.alloc(idx+1)) return false;
  if (!obstacles.polygons[idx].alloc(8)) return false;
  
  obstacles.polygons[idx].points[0].setXY(x-d2, y-d1);
  obstacles.polygons[idx].points[1].setXY(x-d1, y-d2);
  obstacles.polygons[idx].points[2].setXY(x+d1, y-d2);
  obstacles.polygons[idx].points[3].setXY(x+d2, y-d1);
  obstacles.polygons[idx].points[4].setXY(x+d2, y+d1);
  obstacles.polygons[idx].points[5].setXY(x+d1, y+d2);
  obstacles.polygons[idx].points[6].setXY(x-d1, y+d2);
  obstacles.polygons[idx].points[7].setXY(x-d2, y+d1);         
  return true;
}


// check if given point is inside perimeter (and outside exclusions) of current map 
bool Map::isInsidePerimeterOutsideExclusions(Point &pt){
  if (!maps.pointIsInsidePolygon( maps.perimeterPoints, pt)) return false;    

  for (int idx=0; idx < maps.obstacles.numPolygons; idx++){
    if (!maps.pointIsInsidePolygon( maps.obstacles.polygons[idx], pt)) return false;
  }

  for (int idx=0; idx < maps.exclusions.numPolygons; idx++){
    if (maps.pointIsInsidePolygon( maps.exclusions.polygons[idx], pt)) return false;
  }    
  return true;
}


// check if mowing point is inside any obstacle, and if so, find next mowing point (outside any obstacles)
// returns: valid path start point (outside any obstacle) going to the mowing point (which can be used as input for pathfinder)
bool Map::findObstacleSafeMowPoint(Point &findPathToPoint){  
  bool safe;  
  Point dst;  
  while (true){
    safe = true;  
    dst.assign(mowPoints.points[mowPointsIdx]);
    CONSOLE.print("findObstacleSafeMowPoint checking ");    
    CONSOLE.print(dst.x());
    CONSOLE.print(",");
    CONSOLE.println(dst.y());
    for (int idx=0; idx < obstacles.numPolygons; idx++){
      if (pointIsInsidePolygon( obstacles.polygons[idx], dst)){
        safe = false;
        break;
      }
    }
    if (safe) {
      // find valid start point on path to mowing point 
      if (mowPointsIdx == 0) {  // first mowing point has no path
        findPathToPoint.assign(dst);
        return true;
      }
      Point src;
      src.assign(mowPoints.points[mowPointsIdx-1]); // path source is last mowing point
      Point sect;
      Point minSect;
      float minDist = 9999;      
      for (int idx=0; idx < obstacles.numPolygons; idx++){
        if (linePolygonIntersectPoint( dst, src, obstacles.polygons[idx], sect)) {  // find shortest section point to dst    
          float dist = calculateDistance(sect, dst);
          if (dist < minDist ){
            minDist = dist;
            minSect.assign(sect); 
          }
        }
      }
      if (minDist < 9999){ // obstacle on path, use last section point on path for path source 
        findPathToPoint.assign(minSect);
        return true;
      }
      // no obstacle on path, just use next mowing point 
      findPathToPoint.assign(dst);
      return true;
    }    
    // try next mowing point
    if (mowPointsIdx >= mowPoints.numPoints-1){
      CONSOLE.println("findObstacleSafeMowPoint error: no more mowing points reachable due to obstacles");
      return false;
    } 
    mowPointsIdx++;
  }
}

bool Map::mowingCompleted(){
  return (mowPointsIdx >= mowPoints.numPoints-1);
} 

// check if point is inside perimeter and outside exclusions/obstacles
bool Map::checkpoint(float x, float y){
  Point src;
  src.setXY(x, y);
  if (!maps.pointIsInsidePolygon( maps.perimeterPoints, src)){
    return false;
  }
  for (int i=0; i < maps.exclusions.numPolygons; i++){
    if (maps.pointIsInsidePolygon( maps.exclusions.polygons[i], src)){
       return false;
    }
  } 
  for (int i=0; i < obstacles.numPolygons; i++){
    if (maps.pointIsInsidePolygon( maps.obstacles.polygons[i], src)){
       return false;
    }
  }  

  return true;
}

// Helper function to calculate minimum distance from point to polygon edge
float Map::distanceToPolygonEdge(float x, float y, Polygon &polygon) {
  float minDistance = FLT_MAX;
  
  for (int i = 0; i < polygon.numPoints; i++) {
    int nextIdx = (i + 1) % polygon.numPoints;
    Point lineStart = polygon.points[i];
    Point lineEnd = polygon.points[nextIdx];
    
    // Calculate distance from point to line segment
    float A = x - lineStart.x();
    float B = y - lineStart.y();
    float C = lineEnd.x() - lineStart.x();
    float D = lineEnd.y() - lineStart.y();
    
    float dot = A * C + B * D;
    float lenSq = C * C + D * D;
    
    if (lenSq == 0) {
      // Line segment is actually a point
      float distance = sqrt(A * A + B * B);
      minDistance = min(minDistance, distance);
      continue;
    }
    
    float param = dot / lenSq;
    float xx, yy;
    
    if (param < 0) {
      xx = lineStart.x();
      yy = lineStart.y();
    } else if (param > 1) {
      xx = lineEnd.x();
      yy = lineEnd.y();
    } else {
      xx = lineStart.x() + param * C;
      yy = lineStart.y() + param * D;
    }
    
    float dx = x - xx;
    float dy = y - yy;
    float distance = sqrt(dx * dx + dy * dy);
    minDistance = min(minDistance, distance);
  }
  
  return minDistance;
}

// Enhanced pathfinder with safety offsets
bool Map::isPointSafeForRobot(float x, float y) {
  #ifdef ROBOT_WIDTH  // Use new meter-based configuration if available
    float robotRadius = max(ROBOT_WIDTH, ROBOT_LENGTH) / 2.0; // Use actual robot dimensions
    float safetyOffset = PERIMETER_SAFETY_OFFSET;
    
    // Check perimeter with safety offset
    Point testPoint;
    testPoint.setXY(x, y);
    
    // Simple distance check to perimeter (basic implementation)
    // For now, use existing checkpoint and add buffer logic
    if (!checkpoint(x, y)) {
      return false; // Point is already outside safe area
    }
    
    // Check if point is too close to perimeter edges
    // This is a simplified implementation - in production, proper distance calculation would be needed
    float checkRadius = robotRadius + safetyOffset;
    for (float angle = 0; angle < 2 * PI; angle += PI/8) {
      float checkX = x + checkRadius * cos(angle);
      float checkY = y + checkRadius * sin(angle);
      if (!checkpoint(checkX, checkY)) {
        return false; // Robot would collide at this position
      }
    }
    
    // Check exclusion zones with safety offset
    testPoint.setXY(x, y);
    float exclusionCheckRadius = robotRadius + EXCLUSION_SAFETY_OFFSET;
    for (int i = 0; i < maps.exclusions.numPolygons; i++) {
      for (float angle = 0; angle < 2 * PI; angle += PI/4) {
        float checkX = x + exclusionCheckRadius * cos(angle);
        float checkY = y + exclusionCheckRadius * sin(angle);
        Point checkPoint;
        checkPoint.setXY(checkX, checkY);
        if (maps.pointIsInsidePolygon(maps.exclusions.polygons[i], checkPoint)) {
          return false; // Robot would enter exclusion zone
        }
      }
    }
    
    // Check obstacles with safety offset
    float obstacleCheckRadius = robotRadius + OBSTACLE_SAFETY_OFFSET;
    for (int i = 0; i < obstacles.numPolygons; i++) {
      for (float angle = 0; angle < 2 * PI; angle += PI/4) {
        float checkX = x + obstacleCheckRadius * cos(angle);
        float checkY = y + obstacleCheckRadius * sin(angle);
        Point checkPoint;
        checkPoint.setXY(checkX, checkY);
        if (maps.pointIsInsidePolygon(obstacles.polygons[i], checkPoint)) {
          return false; // Robot would collide with obstacle
        }
      }
    }
    
    return true;
  #else
    return checkpoint(x, y); // Original behavior
  #endif
}

// find start point for path finder on line from src to dst
// that is insider perimeter and outside exclusions
void Map::findPathFinderSafeStartPoint(Point &src, Point &dst){
  CONSOLE.print("findPathFinderSafePoint (");  
  CONSOLE.print(src.x());
  CONSOLE.print(",");
  CONSOLE.print(src.y());
  CONSOLE.print(") (");
  CONSOLE.print(dst.x());
  CONSOLE.print(",");
  CONSOLE.print(dst.y());
  CONSOLE.println(")");  
  Point sect;
  if (!pointIsInsidePolygon( perimeterPoints, src)){
    if (linePolygonIntersectPoint( src, dst, perimeterPoints, sect)){
      src.assign(sect);
      CONSOLE.println("found safe point inside perimeter");
      return;
    }    
  }
  for (int i=0; i < exclusions.numPolygons; i++){
    if (pointIsInsidePolygon( exclusions.polygons[i], src)){
      if (linePolygonIntersectionCount(src, dst, exclusions.polygons[i]) == 1){
        // source point is not reachable      
        if (linePolygonIntersectPoint( src, dst, exclusions.polygons[i], sect)){    
          src.assign(sect);
          CONSOLE.println("found safe point outside exclusion");
          return;
        }
      }
    }
  }  
  // point is inside perimeter and outside exclusions
  CONSOLE.println("point is inside perimeter and outside exclusions");
}


// go to next point
// sim=true: only simulate (do not change data)
bool Map::nextPoint(bool sim,float stateX, float stateY){
  //CONSOLE.print("nextPoint sim=");
  //CONSOLE.print(sim);
  //CONSOLE.print(" wayMode=");
  //CONSOLE.println(wayMode);
  if (wayMode == WAY_DOCK){
    return (nextDockPoint(sim));
  } 
  else if (wayMode == WAY_MOW) {
#ifndef __linux__
    return (nextMowPoint(sim));
#else
    Point src;
    Point dst;
    bool r = (nextMowPoint(sim));
    if (!r) {
      // no new mow point available - fast path exit
      return false;
    }

    src.setXY(stateX, stateY);
    // dst might be in an obstacle... check if we can move or may use a new point...
    if (!findObstacleSafeMowPoint(dst)) {
      // didn't find a safe dst fall back to old behaviour
      CONSOLE.println("Map::nextPoint: WARN: no safe mow point found - fall back to normal behaviour!");
      return true;
    }
    bool fr = findPath(src, dst);
    if (!fr) {
      // try again without obstacles
      clearObstacles();
      fr = findPath(src, dst);
    }
    if (!fr) {
      // still didn't find a path - fall back to old behaviour
      CONSOLE.println("Map::nextPoint: WARN: no path - fall back to normal behaviour!");
      return true;
    }

    // move to WAY_FREE list
    wayMode = WAY_FREE;

    return true;
#endif
  } 
  else if (wayMode == WAY_FREE) {
    return (nextFreePoint(sim));
  } else return false;
}


// get next mowing point
bool Map::nextMowPoint(bool sim){  
  if (shouldMow){
    if (mowPointsIdx+1 < mowPoints.numPoints){
      // next mowing point       
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) mowPointsIdx++;
      //if (!sim) targetPointIdx++;      
      return true;
    } else {
      // finished mowing;
      mowPointsIdx = 0;            
      return false;
    }         
  } else if ((shouldDock) && (dockPoints.numPoints > 0)) {      
      // go docking
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) freePointsIdx = 0; 
      if (!sim) wayMode = WAY_FREE;      
      return true;    
  } else return false;  
}

// get next docking point  
bool Map::nextDockPoint(bool sim){    
  /*CONSOLE.print("nextDockPoint: shouldDock=");
  CONSOLE.print(shouldDock);
  CONSOLE.print("  dockPointsIdx=");
  CONSOLE.print(dockPointsIdx);
  CONSOLE.print("  dockPoints.numPoints=");
  CONSOLE.print(dockPoints.numPoints);
  CONSOLE.println();*/
  if (shouldDock){
    // should dock  
    if (dockPointsIdx+1 < dockPoints.numPoints){
      if (!sim) { 
        lastTargetPoint.assign(targetPoint);
        if (dockPointsIdx == 0) {
          CONSOLE.println("nextDockPoint: shouldRetryDock=false");
          shouldRetryDock=false;
        }
        if (shouldRetryDock) {
          CONSOLE.println("nextDockPoint: shouldRetryDock=true");
          dockPointsIdx--;
          trackReverse = (DOCK_FRONT_SIDE) || ((!DOCK_FRONT_SIDE) && (dockPointsIdx < dockPoints.numPoints-3));                    
        } else {
          dockPointsIdx++; 
          trackReverse = (!DOCK_FRONT_SIDE) && (dockPointsIdx >= dockPoints.numPoints-3) ; // dock reverse only near dock
        }
      }              
      if (!sim) trackSlow = true;
      if (!sim) useGPSfixForPosEstimation = true;
      if (!sim) useGPSfixForDeltaEstimation = true;      
      if (!sim) useGPSfloatForPosEstimation = false;    
      if (!sim) useGPSfloatForDeltaEstimation = false;    
      if (!sim) useIMU = true;     // false      
      return true;
    } else {
      // finished docking
      return false;
    } 
  } else if (shouldMow){
    // should undock
    if (dockPointsIdx > 0){
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) dockPointsIdx--;              
      if (!sim) {
        trackReverse = (DOCK_FRONT_SIDE) && (dockPointsIdx >= dockPoints.numPoints-3) ; // undock reverse only in dock
      }              
      if (!sim) trackSlow = true;      
      return true;
    } else {
      // finished undocking
      if ((shouldMow) && (mowPoints.numPoints > 0 )){
        if (!sim) lastTargetPoint.assign(targetPoint);
        //if (!sim) targetPointIdx = freeStartIdx;
        if (!sim) wayMode = WAY_FREE;      
        if (!sim) trackReverse = false;              
        if (!sim) trackSlow = false;
        if (!sim) useGPSfixForPosEstimation = true;        
        if (!sim) useGPSfixForDeltaEstimation = true;
        if (!sim) useGPSfloatForPosEstimation = true;    
        if (!sim) useGPSfloatForDeltaEstimation = true;    
        if (!sim) useIMU = true;    
        return true;
      } else return false;        
    }  
  }
  return false;
}

// get next free point  
bool Map::nextFreePoint(bool sim){
  // free points
  if (freePointsIdx+1 < freePoints.numPoints){
    if (!sim) lastTargetPoint.assign(targetPoint);
    if (!sim) freePointsIdx++;                  
    return true;
  } else {
    // finished free points
    if ((shouldMow) && (mowPoints.numPoints > 0 )){
      // start mowing
      if (!sim) lastTargetPoint.assign(targetPoint);      
      if (!sim) wayMode = WAY_MOW;
      return true;  
    } else if ((shouldDock) && (dockPoints.numPoints > 0)){      
      // start docking
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) dockPointsIdx = 0;      
      if (!sim) wayMode = WAY_DOCK;
      if (!sim) trackReverse = (!DOCK_FRONT_SIDE) && (dockPointsIdx >= dockPoints.numPoints-3) ; // dock reverse only near dock    
      return true;
    } else return false;
  }  
}

void Map::setLastTargetPoint(float stateX, float stateY){
  lastTargetPoint.setXY(stateX, stateY);
}


// ------------------------------------------------------ 


float Map::calculateDistance(Point &src, Point &dst) {
  return sqrt(sq(src.x()-dst.x())+sq(src.y()-dst.y()));  
}


// checks if point is inside bounding box given by points A, B
bool Map::isPointInBoundingBox(Point &pt, Point &A, Point &B){
  float minX = min(A.x(), B.x());
  float minY = min(A.y(), B.y());
  float maxX = max(A.x(), B.x());
  float maxY = max(A.y(), B.y());    
  if (pt.x() < minX-0.02) return false;
  if (pt.y() < minY-0.02) return false;
  if (pt.x() > maxX+0.02) return false;
  if (pt.y() > maxY+0.02) return false;
  return true;
}

// calculates intersection point (or touch point) of two lines 
// (A,B) 1st line
// (C,D) 2nd line  
// https://www.geeksforgeeks.org/program-for-point-of-intersection-of-two-lines/
bool Map::lineLineIntersection(Point &A, Point &B, Point &C, Point &D, Point &pt)  { 
  //console.log('lineLineIntersection', A,B,C,D);
  if ((calculateDistance(A, C) < 0.02) || (calculateDistance(A, D) < 0.02)) { 
    pt.assign(A);   
    return true;
  } 
  if ((calculateDistance(B, C) < 0.02) || (calculateDistance(B, D) < 0.02)){
    pt.assign(B);
    return true;
  }   
  // Line AB represented as a1x + b1y = c1 
  float a1 = B.y() - A.y(); 
  float b1 = A.x() - B.x(); 
  float c1 = a1*(A.x()) + b1*(A.y()); 
  // Line CD represented as a2x + b2y = c2 
  float a2 = D.y() - C.y(); 
  float b2 = C.x() - D.x(); 
  float c2 = a2*(C.x())+ b2*(C.y());   
  float determinant = a1*b2 - a2*b1;   
  if (determinant == 0)  { 
      // The lines are parallel.         
      //console.log('lines are parallel');
      return false;
  } else { 
      float x = (b2*c1 - b1*c2)/determinant; 
      float y = (a1*c2 - a2*c1)/determinant;             
      Point cp;
      cp.setXY(x, y);    
      if (!isPointInBoundingBox(cp, A, B)) return false; // not in bounding box of 1st line
      if (!isPointInBoundingBox(cp, C, D)) return false; // not in bounding box of 2nd line
      pt.assign(cp);
      return true;
  } 
} 

    

// determines if a line intersects (or touches) a polygon and returns shortest intersection point from src
bool Map::linePolygonIntersectPoint( Point &src, Point &dst, Polygon &poly, Point &sect) {      
  //Poly testpoly = poly;
  //if (allowtouch) testpoly = this.polygonOffset(poly, -0.02);      
  Point p1;
  Point p2;
  Point cp;
  float minDist = 9999;
  //CONSOLE.print("linePolygonIntersectPoint (");
  //CONSOLE.print(src.x());  
  //CONSOLE.print(",");
  //CONSOLE.print(src.y());
  //CONSOLE.print(") (");
  //CONSOLE.print(dst.x());
  //CONSOLE.print(",");
  //CONSOLE.print(dst.y());
  //CONSOLE.println(")");
  for (int i = 0; i < poly.numPoints; i++) {      
    p1.assign( poly.points[i] );
    p2.assign( poly.points[ (i+1) % poly.numPoints] );                
    //CONSOLE.print("(");
    //CONSOLE.print(p1.x());
    //CONSOLE.print(",");
    //CONSOLE.print(p1.y());
    //CONSOLE.print(") ");
    //CONSOLE.print("(");
    //CONSOLE.print(p2.x());
    //CONSOLE.print(",");
    //CONSOLE.print(p2.y());
    //CONSOLE.print(") ");
    if (lineIntersects(p1, p2, src, dst)) {        
      //CONSOLE.print(" intersect  ");
      if (lineLineIntersection(p1, p2, src, dst, cp)){        
        //CONSOLE.print(cp.x());
        //CONSOLE.print(",");
        //CONSOLE.print(cp.y());
        float dist = calculateDistance(src, cp);
        //CONSOLE.print("  dist=");
        //CONSOLE.println(dist);
        if (dist < minDist){
          minDist = dist;
          sect.assign(cp);
        }        
      }
    } // else CONSOLE.println();    
    //if (this.doIntersect(p1, p2, src, dst)) return true;
    //if (this.lineLineIntersection(p1, p2, src, dst) != null) return true;      
  }     
  return (minDist < 9999);  
}



// checks if point is inside obstacle polygon (or touching polygon line or points)
// The algorithm is ray-casting to the right. Each iteration of the loop, the test point is checked against
// one of the polygon's edges. The first line of the if-test succeeds if the point's y-coord is within the
// edge's scope. The second line checks whether the test point is to the left of the line
// If that is true the line drawn rightwards from the test point crosses that edge.
// By repeatedly inverting the value of c, the algorithm counts how many times the rightward line crosses the
// polygon. If it crosses an odd number of times, then the point is inside; if an even number, the point is outside.
bool Map::pointIsInsidePolygon( Polygon &polygon, Point &pt)
{
  int i, j, c = 0;
  int nvert = polygon.numPoints;
  if (nvert == 0) return false;
  Point pti;
  Point ptj;  
  int x = pt.px;
  int y = pt.py;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    pti.assign(polygon.points[i]);
    ptj.assign(polygon.points[j]);    
    
    #ifdef FLOAT_CALC    
    if ( ((pti.y()>pt.y()) != (ptj.y()>pt.y())) &&
     (pt.x() < (ptj.x()-pti.x()) * (pt.y()-pti.y()) / (ptj.y()-pti.y()) + pti.x()) )
       c = !c;             
    #else           
    if ( ((pti.py>y) != (ptj.py>y)) &&
     (x < (ptj.px-pti.px) * (y-pti.py) * 10 / (ptj.py-pti.py) / 10 + pti.px) )
       c = !c;
    #endif       
    
  }
  //if (c != d){
  //  CONSOLE.println("pointIsInsidePolygon bogus");
  //}
  return (c % 2 != 0);
}      


// checks if two lines intersect (or if single line points touch with line or line points)
// (p0,p1) 1st line
// (p2,p3) 2nd line
bool Map::lineIntersects (Point &p0, Point &p1, Point &p2, Point &p3) {
  /*CONSOLE.print("lineIntersects ((");
  CONSOLE.print(p0.x);  
  CONSOLE.print(",");
  CONSOLE.print(p0.y);
  CONSOLE.print("),(");
  CONSOLE.print(p1.x);
  CONSOLE.print(",");
  CONSOLE.print(p1.y);
  CONSOLE.print("))   ((");   
  CONSOLE.print(p2.x);  
  CONSOLE.print(",");
  CONSOLE.print(p2.y);
  CONSOLE.print("),(");
  CONSOLE.print(p3.x);
  CONSOLE.print(",");
  CONSOLE.print(p3.y);
  CONSOLE.print(")) ");*/  
  int p0x = p0.px;
  int p0y = p0.py;
  int p1x = p1.px;
  int p1y = p1.py;
  int p2x = p2.px;
  int p2y = p2.py;
  int p3x = p3.px;
  int p3y = p3.py;  
  int s1x = p1x - p0x;
  int s1y = p1y - p0y;
  int s2x = p3x - p2x;
  int s2y = p3y - p2y;
  
  #ifdef FLOAT_CALC
  float s = ((float) (-s1y * (p0x - p2x) + s1x * (p0y - p2y))  ) /  ((float) (-s2x * s1y + s1x * s2y)  );
  float t = ((float) (s2x * (p0y - p2y) - s2y * (p0x - p2x))   ) /  ((float)  (-s2x * s1y + s1x * s2y) );
  return ((s >= 0) && (s <= 1) && (t >= 0) && (t <= 1));
  
  #else  
  int snom = (-s1y * (p0x - p2x) + s1x * (p0y - p2y));
  int sdenom = (-s2x * s1y + s1x * s2y);
  int tnom = (s2x * (p0y - p2y) - s2y * (p0x - p2x));
  int tdenom = (-s2x * s1y + s1x * s2y);  
  
  if ( (snom < 0) && ( (sdenom > 0) || (snom < sdenom) ) ) return false;
  if ( (snom > 0) && ( (sdenom < 0) || (snom > sdenom) ) ) return false;
      
  if ( (tnom < 0) && ( (tdenom > 0) || (tnom < tdenom) ) ) return false;
  if ( (tnom > 0) && ( (tdenom < 0) || (tnom > tdenom) ) ) return false;    
  
  return true;
  #endif
  //if (res1 != res2){
  //  CONSOLE.println("lineIntersects bogus");
  //}  
}


// determines how often a line intersects (or touches) a polygon
int Map::linePolygonIntersectionCount(Point &src, Point &dst, Polygon &poly){
  Point p1;
  Point p2;
  int count = 0;
  for (int i = 0; i < poly.numPoints; i++) {      
    p1.assign( poly.points[i] );
    p2.assign( poly.points[ (i+1) % poly.numPoints] );             
    if (lineIntersects(p1, p2, src, dst)) {        
      count++;
    }  
  }       
  return count;
}
    
  
// determines if a line intersects (or touches) a polygon
bool Map::linePolygonIntersection( Point &src, Point &dst, Polygon &poly) {      
  //Poly testpoly = poly;
  //if (allowtouch) testpoly = this.polygonOffset(poly, -0.02);      
  Point p1;
  Point p2;
  for (int i = 0; i < poly.numPoints; i++) {      
    p1.assign( poly.points[i] );
    p2.assign( poly.points[ (i+1) % poly.numPoints] );             
    if (lineIntersects(p1, p2, src, dst)) {        
      return true;
    }
    //if (this.doIntersect(p1, p2, src, dst)) return true;
    //if (this.lineLineIntersection(p1, p2, src, dst) != null) return true;      
  }       
  return false;
}
      
  
// Calculates the area of a polygon.
float Map::polygonArea(Polygon &poly){
  float a = 0;
  int i;
  int l;
  Point v0;
  Point v1;
  for (i = 0, l = poly.numPoints; i < l; i++) {
    v0.assign( poly.points[i] );
    v1.assign( poly.points[i == l - 1 ? 0 : i + 1] );
    a += v0.x() * v1.y();
    a -= v1.x() * v0.y();
  }
  return a / 2;
}
  

  
  
// offset polygon points by distance
// https://stackoverflow.com/questions/54033808/how-to-offset-polygon-edges
bool Map::polygonOffset(Polygon &srcPoly, Polygon &dstPoly, float dist){    
  bool orient = (polygonArea(srcPoly) >= 0);
  //console.log('orient',orient);
  //var orient2 = ClipperLib.Clipper.Orientation(poly);
  //if (orient != orient2){
  //  console.log('polygonOffset bogus!');   
  //}        
  if (!dstPoly.alloc(srcPoly.numPoints)) return false;
  Point p1;
  Point p2;
  Point p3;  
  for (int idx1 = 0; idx1 < srcPoly.numPoints; idx1++){
    int idx2 = idx1-1;
    if (idx2 < 0) idx2 = srcPoly.numPoints-1;
    int idx3 = idx1+1;
    if (idx3 > srcPoly.numPoints-1) idx3 = 0;      
    p2.assign(srcPoly.points[idx2]); // previous           
    p1.assign(srcPoly.points[idx1]); // center
    p3.assign(srcPoly.points[idx3]); // next                 
    float a3 = atan2(p3.y() - p1.y(), p3.x() - p1.x());            
    float a2 = atan2(p2.y() - p1.y(), p2.x() - p1.x());      
    float angle = a2 + (a3-a2)/2;      
    if (a3 < a2) angle-=PI;      
    if (!orient) angle+=PI;    
    dstPoly.points[idx1].setXY( p1.x() + dist * cos(angle), p1.y() + dist * sin(angle) );     
  }  
  return true;
}

      
float Map::calcHeuristic(Point &pos0, Point &pos1) {
  // Use Euclidean distance for better A* performance (admissible and consistent heuristic)
  return calculateDistance(pos0, pos1);
  //return distanceManhattan(pos0, pos1);  // Manhattan distance (less optimal for A*)
}
  

// given a start node, we check potential next node with all obstacle nodes:
// 1. if start node is outside perimeter, it must be within a certain distance to section point with perimeter (to next node), 
//  and must have section count of one
// 2. if start node is inside exclusion, it must be within a certain distance to section point with exclusion (to next node)  
// 3. otherwise: line between start node and next node node must not intersect any obstacle  
  
int Map::findNextNeighbor(NodeList &nodes, PolygonList &obstacles, Node &node, int startIdx) {
  for (int idx = startIdx+1; idx < nodes.numNodes; idx++){
    if (nodes.nodes[idx].opened) continue;
    if (nodes.nodes[idx].closed) continue;                
    if (nodes.nodes[idx].point == node.point) continue;     
    Point *pt = nodes.nodes[idx].point;            
    
    bool safe = true;            
    Point sectPt;  
  
    // check new path with all obstacle polygons (perimeter, exclusions, obstacles)     
    for (int idx3 = 0; idx3 < obstacles.numPolygons; idx3++){             
       bool isPeri = ((perimeterPoints.numPoints > 0) && (idx3 == 0));  // if first index, it's perimeter, otherwise exclusions                           
       if (isPeri){ // we check with the perimeter?         
         //CONSOLE.println("we check with perimeter");
         bool insidePeri = pointIsInsidePolygon(obstacles.polygons[idx3], *node.point);
         if (!insidePeri) { // start point outside perimeter?                                                                                      
             //CONSOLE.println("start point oustide perimeter");
             if (linePolygonIntersectPoint( *node.point, *pt, obstacles.polygons[idx3], sectPt)){               
               float dist = calculateDistance(*node.point, sectPt);
               if (dist > ALLOW_ROUTE_OUTSIDE_PERI_METER){ safe = false; break; } // entering perimeter with long distance is not safe                             
               if (linePolygonIntersectionCount( *node.point, *pt, obstacles.polygons[idx3]) != 1){ 
                 safe = false; break; 
               }
               continue;           
             } else { safe = false; break; }                                          
         }
       } else {
         //CONSOLE.println("we check with exclusion");
         bool insideObstacle = pointIsInsidePolygon(obstacles.polygons[idx3], *node.point);
         if (insideObstacle) { // start point inside obstacle?          
             if (linePolygonIntersectPoint( *node.point, *pt, obstacles.polygons[idx3], sectPt)){               
               float dist = calculateDistance(*node.point, sectPt);
               if (dist > ALLOW_ROUTE_OUTSIDE_PERI_METER){ 
                 safe = false; break; 
               } // exiting obstacle with long distance is not safe                             
               continue;           
             } else { safe = false; break; }                                          
         }
       }        
       if (linePolygonIntersection (*node.point, *pt, obstacles.polygons[idx3])){
         safe = false;
         break;
       }             
    }
    // Path safety check completed
    if (safe) {          
      //pt.visited = true;
      //var anode = {pos: pt, parent: node, f:0, g:0, h:0};          
      //ret.push(anode);
      return idx;
    }            
  }       
  return -1;
}  


// astar path finder 
// https://briangrinstead.com/blog/astar-search-algorithm-in-javascript/
bool Map::findPath(Point &src, Point &dst){
  if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)){
    CONSOLE.println("ERROR findPath: memory errors");
    return false; 
  }  
  
  unsigned long startTime = millis();
  CONSOLE.print("findPath (");
  CONSOLE.print(src.x());
  CONSOLE.print(",");
  CONSOLE.print(src.y());
  CONSOLE.print(") (");
  CONSOLE.print(dst.x());
  CONSOLE.print(",");
  CONSOLE.print(dst.y());
  CONSOLE.println(")");  
  
  if (ENABLE_PATH_FINDER){    
    CONSOLE.print("path finder is enabled");      
    #ifdef FLOAT_CALC
      CONSOLE.print(" (using FLOAT_CALC)");    
    #endif
    CONSOLE.println();
    
    // Initialize pathfinding data structures
    if (!initializePathfinding(src, dst)) {
      return false;
    }
    
    // Run A* pathfinding algorithm
    Node* resultNode = processPathfindingNodes(src, dst, startTime);
    
    // Reconstruct path from result
    if (!reconstructPath(resultNode, dst)) {
      return false;
    }
  } else {  // path finder not enabled (ENABLE_PATH_FINDER=false)    
    if (!freePoints.alloc(2)) return false;
    freePoints.points[0].assign(src);    
    freePoints.points[1].assign(dst);        
  }    
  freePointsIdx=0;  
  
  checkMemoryErrors();  
  resetImuTimeout();
  return true;  
}

// Initialize pathfinding data structures (obstacles and nodes)
bool Map::initializePathfinding(Point &src, Point &dst) {
  // create path-finder obstacles    
  int idx = 0;
  if (!pathFinderObstacles.alloc(1 + exclusions.numPolygons + obstacles.numPolygons)) return false;
  
  if (freeMemory () < 5000){
    CONSOLE.println("OUT OF MEMORY");
    return false;
  }

  // For validating a potential route, we will use  'linePolygonIntersectPoint' and check for intersections between route start point 
  // and end point. To have something to check intersection with, we offset the perimeter (make bigger) and exclusions
  //  (maker schmaller) and use them as 'obstacles'.
  
  if (!polygonOffset(perimeterPoints, pathFinderObstacles.polygons[idx], 0.04)) return false;
  idx++;
  
  for (int i=0; i < exclusions.numPolygons; i++){
    if (!polygonOffset(exclusions.polygons[i], pathFinderObstacles.polygons[idx], -0.04)) return false;
    idx++;
  }      
  for (int i=0; i < obstacles.numPolygons; i++){
    if (!polygonOffset(obstacles.polygons[i], pathFinderObstacles.polygons[idx], -0.04)) return false;
    idx++;
  }  
  
  // create nodes
  int allocNodeCount = exclusions.numPoints() + obstacles.numPoints() + perimeterPoints.numPoints + 2;
  CONSOLE.print ("freem=");
  CONSOLE.print(freeMemory ());    
  CONSOLE.print("  allocating nodes ");
  CONSOLE.print(allocNodeCount);
  CONSOLE.print(" (");
  CONSOLE.print(sizeof(Node) * allocNodeCount);
  CONSOLE.println(" bytes)");

  if (!pathFinderNodes.alloc(allocNodeCount)) return false;
  for (int i=0; i < pathFinderNodes.numNodes; i++){
    pathFinderNodes.nodes[i].init();
  }
  // exclusion nodes
  idx = 0;
  for (int i=0; i < exclusions.numPolygons; i++){
    for (int j=0; j < exclusions.polygons[i].numPoints; j++){    
      pathFinderNodes.nodes[idx].point = &exclusions.polygons[i].points[j];
      idx++;
    }
  }
  // obstacle nodes    
  for (int i=0; i < obstacles.numPolygons; i++){
    for (int j=0; j < obstacles.polygons[i].numPoints; j++){    
      pathFinderNodes.nodes[idx].point = &obstacles.polygons[i].points[j];
      idx++;
    }
  }
  // perimeter nodes
  for (int j=0; j < perimeterPoints.numPoints; j++){    
    pathFinderNodes.nodes[idx].point = &perimeterPoints.points[j];
    idx++;
  }      
  // start node
  Node *start = &pathFinderNodes.nodes[idx];
  start->point = &src;
  start->opened = true;
  idx++;
  // end node
  Node *end = &pathFinderNodes.nodes[idx];
  end->point = &dst;    
  idx++;
  
  CONSOLE.print ("freem=");
  CONSOLE.println (freeMemory ());
  
  return true;
}

// Process A* pathfinding algorithm
Node* Map::processPathfindingNodes(Point &src, Point &dst, unsigned long startTime) {
  unsigned long nextProgressTime = 0;
  // Adaptive timeout based on problem size: base timeout + factor based on number of nodes
  int baseTimeout = 500;
  int adaptiveTimeout = baseTimeout + (pathFinderNodes.numNodes / 10);
  int timeout = adaptiveTimeout;    
  Node *currentNode = NULL;
  Node *end = &pathFinderNodes.nodes[pathFinderNodes.numNodes-1]; // end node is last
  
  // Path-finder started
  
  // Early termination: Check if destination is directly reachable without obstacles
  bool directPathPossible = true;
  for (int i = 0; i < pathFinderObstacles.numPolygons; i++) {
    if (linePolygonIntersection(src, dst, pathFinderObstacles.polygons[i])) {
      directPathPossible = false;
      break;
    }
  }
  
  if (directPathPossible) {
    // Direct path possible - early termination
    // Create simple two-point path
    if (freePoints.alloc(2)) {
      freePoints.points[0].assign(src);
      freePoints.points[1].assign(dst);
    }
    return &pathFinderNodes.nodes[pathFinderNodes.numNodes-1]; // Return end node
  }
  
  while(true) {       
    if (millis() >= nextProgressTime){
      nextProgressTime = millis() + 4000;          
      // Progress indicator (removed for performance)
      watchdogReset();     
    }
    timeout--;            
    if (timeout == 0){
      // Pathfinding timeout reached
      break;
    }
    // Find the node with lowest f(x) value in open list
    int lowInd = -1;
    float lowestF = FLT_MAX;
    for(int i=0; i<pathFinderNodes.numNodes; i++) {
      if (pathFinderNodes.nodes[i].opened && pathFinderNodes.nodes[i].f < lowestF) { 
        lowInd = i;
        lowestF = pathFinderNodes.nodes[i].f;
      }              
    }
    if (lowInd == -1) break; // No more open nodes
    currentNode = &pathFinderNodes.nodes[lowInd]; 
    // End case -- result has been found, return the traced path
    if (calculateDistance(*currentNode->point, *end->point) < 0.02) break;        
    // Normal case -- move currentNode from open to closed, process each of its neighbors      
    currentNode->opened = false;
    currentNode->closed = true;
    
    int neighborIdx = -1;
    while (true) {        
      neighborIdx = findNextNeighbor(pathFinderNodes, pathFinderObstacles, *currentNode, neighborIdx); 
      if (neighborIdx == -1) break;
      Node* neighbor = &pathFinderNodes.nodes[neighborIdx];                
      
      if (millis() >= nextProgressTime){
        nextProgressTime = millis() + 4000;          
        // Neighbor processing indicator (removed for performance)
        watchdogReset();     
      }
      
      float gScore = currentNode->g + calculateDistance(*currentNode->point, *neighbor->point);
      bool gScoreIsBest = false;
      bool found = neighbor->opened;        
      if (!found){          
        // This the the first time we have arrived at this node, it must be the best
        // Also, we need to take the h (heuristic) score since we haven't done so yet 
        gScoreIsBest = true;
        neighbor->h = calcHeuristic(*neighbor->point, *end->point);
        neighbor->opened = true;
      }
      else if(gScore < neighbor->g) {
        // We have already seen the node, but last time it had a worse g (distance from start)
        gScoreIsBest = true;
      } 
      if(gScoreIsBest) {
        // Found an optimal (so far) path to this node.   Store info on how we got here and
        //  just how good it really is...
        neighbor->parent = currentNode;
        neighbor->g = gScore;
        neighbor->f = neighbor->g + neighbor->h;
      }
    }
  } 

  CONSOLE.print("finish nodes=");
  CONSOLE.print(pathFinderNodes.numNodes);
  CONSOLE.print(" duration=");
  CONSOLE.println(millis()-startTime);  

  resetImuTimeout();
  
  return currentNode;
}

// Reconstruct path from pathfinding result
bool Map::reconstructPath(Node* resultNode, Point &dst) {
  if ((resultNode != NULL) && (calculateDistance(*resultNode->point, dst) < 0.02)) {
    Node *curr = resultNode;
    int nodeCount = 0;
    while(curr) {                
      nodeCount++;
      curr = curr->parent;        
    }      
    if (!freePoints.alloc(nodeCount)) return false;
    curr = resultNode;
    int idx = nodeCount-1;
    while(curr) {                                
      freePoints.points[idx].assign( *curr->point );
      idx--;
      curr = curr->parent;                
    }            
    // Apply path smoothing for more natural robot movements
    smoothPath();
  } else {
    // No result was found
    CONSOLE.println("pathfinder: no path");      
    return false;
  }       
  return true;
}

// Smooth path by removing unnecessary waypoints that are in direct line of sight
void Map::smoothPath() {
  if (freePoints.numPoints < 3) return; // Need at least 3 points to smooth
  
  Polygon tempPoints;
  if (!tempPoints.alloc(freePoints.numPoints)) return;
  
  int writeIdx = 0;
  tempPoints.points[writeIdx++].assign(freePoints.points[0]); // Always keep start point
  
  for (int i = 1; i < freePoints.numPoints - 1; i++) {
    Point &prev = tempPoints.points[writeIdx - 1];
    Point &current = freePoints.points[i];
    Point &next = freePoints.points[i + 1];
    
    // Check if we can skip current point by going directly from prev to next
    bool canSkip = true;
    
    // Check if direct line from prev to next intersects with any obstacles
    for (int j = 0; j < pathFinderObstacles.numPolygons; j++) {
      if (linePolygonIntersection(prev, next, pathFinderObstacles.polygons[j])) {
        canSkip = false;
        break;
      }
    }
    
    // If we can't skip this point, add it to the smoothed path
    if (!canSkip) {
      tempPoints.points[writeIdx++].assign(current);
    }
  }
  
  tempPoints.points[writeIdx++].assign(freePoints.points[freePoints.numPoints - 1]); // Always keep end point
  
  // Copy smoothed path back to freePoints
  freePoints.dealloc();
  if (freePoints.alloc(writeIdx)) {
    for (int i = 0; i < writeIdx; i++) {
      freePoints.points[i].assign(tempPoints.points[i]);
    }
  }
  
  tempPoints.dealloc();
}


// path finder stress test
void Map::stressTest(){  
  Point src;
  Point dst;
  float d = 30.0;
  for (int i=0 ; i < 10; i++){
    for (int j=0 ; j < 20; j++){
      addObstacle( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    }
    src.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    dst.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    findPath(src, dst);    
    clearObstacles();
  }  
  checkMemoryErrors();
}
  
  

// integer calculation correctness test
void Map::testIntegerCalcs(){  
  Point pt;
  float d = 30.0;
  for (int i=0 ; i < 5000; i++){
    pt.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    pointIsInsidePolygon( perimeterPoints, pt);    
    CONSOLE.print(".");
  }
  
  Point p1;
  Point p2;
  Point p3;
  Point p4;
  for (int i=0 ; i < 5000; i++){
    p1.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    p2.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    p3.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    p4.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    lineIntersects (p1, p2, p3, p4);
    CONSOLE.print(",");
  }   
}
  

// map transfer stress test
void Map::stressTestMapTransfer(){
  CONSOLE.print("stressTestMapTransfer start");
  for (int counter=0 ; counter < 5000; counter++){
    int idx = counter;
    maps.setPoint(idx, 0, 0);
    setExclusionLength(idx, idx);
    maps.setWayCount(WAY_PERIMETER, idx);                
    maps.setWayCount(WAY_EXCLUSION, idx);                
    maps.setWayCount(WAY_DOCK, idx);                
    maps.setWayCount(WAY_MOW, idx);                
    maps.setWayCount(WAY_FREE, idx);                
  }
  CONSOLE.print("stressTestMapTransfer end");
}

