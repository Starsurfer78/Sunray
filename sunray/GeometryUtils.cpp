// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Geometry utility functions implementation

#include "GeometryUtils.h"
#include "robot.h" // For CONSOLE macro
#include <math.h>

// Line intersection test using integer arithmetic for precision
bool GeometryUtils::lineIntersects(Point &p0, Point &p1, Point &p2, Point &p3) {
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
    float s = ((float)(-s1y * (p0x - p2x) + s1x * (p0y - p2y))) / ((float)(-s2x * s1y + s1x * s2y));
    float t = ((float)(s2x * (p0y - p2y) - s2y * (p0x - p2x))) / ((float)(-s2x * s1y + s1x * s2y));
    return ((s >= 0) && (s <= 1) && (t >= 0) && (t <= 1));
#else
    int snom = (-s1y * (p0x - p2x) + s1x * (p0y - p2y));
    int sdenom = (-s2x * s1y + s1x * s2y);
    int tnom = (s2x * (p0y - p2y) - s2y * (p0x - p2x));
    int tdenom = (-s2x * s1y + s1x * s2y);
    
    if ((snom < 0) && ((sdenom > 0) || (snom < sdenom))) return false;
    if ((snom > 0) && ((sdenom < 0) || (snom > sdenom))) return false;
    
    if ((tnom < 0) && ((tdenom > 0) || (tnom < tdenom))) return false;
    if ((tnom > 0) && ((tdenom < 0) || (tnom > tdenom))) return false;
    
    return true;
#endif
}

// Count how often a line intersects a polygon
int GeometryUtils::linePolygonIntersectionCount(Point &src, Point &dst, Polygon &poly) {
    Point p1;
    Point p2;
    int count = 0;
    
    for (int i = 0; i < poly.numPoints; i++) {
        p1.assign(poly.points[i]);
        p2.assign(poly.points[(i + 1) % poly.numPoints]);
        
        if (lineIntersects(p1, p2, src, dst)) {
            count++;
        }
    }
    
    return count;
}

// Test if a line intersects a polygon
bool GeometryUtils::linePolygonIntersection(Point &src, Point &dst, Polygon &poly) {
    Point p1;
    Point p2;
    
    for (int i = 0; i < poly.numPoints; i++) {
        p1.assign(poly.points[i]);
        p2.assign(poly.points[(i + 1) % poly.numPoints]);
        
        if (lineIntersects(p1, p2, src, dst)) {
            return true;
        }
    }
    
    return false;
}

// Calculate the area of a polygon using the shoelace formula
float GeometryUtils::polygonArea(Polygon &poly) {
    float a = 0;
    Point v0;
    Point v1;
    
    for (int i = 0; i < poly.numPoints; i++) {
        v0.assign(poly.points[i]);
        v1.assign(poly.points[i == poly.numPoints - 1 ? 0 : i + 1]);
        
        a += v0.x() * v1.y();
        a -= v1.x() * v0.y();
    }
    
    return a / 2;
}

// Offset polygon points by distance
// https://stackoverflow.com/questions/54033808/how-to-offset-polygon-edges
bool GeometryUtils::polygonOffset(Polygon &srcPoly, Polygon &dstPoly, float dist) {
    bool orient = (polygonArea(srcPoly) >= 0);
    
    if (!dstPoly.alloc(srcPoly.numPoints)) return false;
    
    Point p1;
    Point p2;
    Point p3;
    
    for (int idx1 = 0; idx1 < srcPoly.numPoints; idx1++) {
        int idx2 = idx1 - 1;
        if (idx2 < 0) idx2 = srcPoly.numPoints - 1;
        
        int idx3 = idx1 + 1;
        if (idx3 > srcPoly.numPoints - 1) idx3 = 0;
        
        p2.assign(srcPoly.points[idx2]); // previous
        p1.assign(srcPoly.points[idx1]); // center
        p3.assign(srcPoly.points[idx3]); // next
        
        float a3 = atan2(p3.y() - p1.y(), p3.x() - p1.x());
        float a2 = atan2(p2.y() - p1.y(), p2.x() - p1.x());
        float angle = a2 + (a3 - a2) / 2;
        
        if (a3 < a2) angle -= PI;
        if (!orient) angle += PI;
        
        dstPoly.points[idx1].setXY(p1.x() + dist * cos(angle), p1.y() + dist * sin(angle));
    }
    
    return true;
}

// Calculate Euclidean distance between two points
float GeometryUtils::distance(Point &p1, Point &p2) {
    float dx = p1.x() - p2.x();
    float dy = p1.y() - p2.y();
    return sqrt(dx * dx + dy * dy);
}

// Calculate Manhattan distance between two points
float GeometryUtils::distanceManhattan(Point &p1, Point &p2) {
    return abs(p1.x() - p2.x()) + abs(p1.y() - p2.y());
}

// Placeholder implementations for functions that may be implemented elsewhere
bool GeometryUtils::pointIsInsidePolygon(Polygon &poly, Point &pt) {
    // This function needs to be implemented based on the actual algorithm used
    // in the original code. For now, return false as placeholder.
    return false;
}

bool GeometryUtils::linePolygonIntersectPoint(Point &src, Point &dst, Polygon &poly, Point &sectPt) {
    // This function needs to be implemented based on the actual algorithm used
    // in the original code. For now, return false as placeholder.
    return false;
}