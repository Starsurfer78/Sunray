// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Geometry utility functions

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include "Point.h"
#include "Polygon.h"

// Geometry utility class with static methods
class GeometryUtils {
public:
    // Line intersection functions
    static bool lineIntersects(Point &p0, Point &p1, Point &p2, Point &p3);
    static bool linePolygonIntersection(Point &src, Point &dst, Polygon &poly);
    static int linePolygonIntersectionCount(Point &src, Point &dst, Polygon &poly);
    
    // Polygon functions
    static float polygonArea(Polygon &poly);
    static bool polygonOffset(Polygon &srcPoly, Polygon &dstPoly, float dist);
    
    // Point-in-polygon test (if implemented elsewhere)
    static bool pointIsInsidePolygon(Polygon &poly, Point &pt);
    
    // Distance functions
    static float distance(Point &p1, Point &p2);
    static float distanceManhattan(Point &p1, Point &p2);
    
    // Line-polygon intersection with point result
    static bool linePolygonIntersectPoint(Point &src, Point &dst, Polygon &poly, Point &sectPt);
};

// Constants for geometry calculations are now handled in config.h

#endif // GEOMETRY_UTILS_H