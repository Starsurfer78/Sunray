#include "PathFinder.h"
#include "map.h"
#include "robot.h"
#include "SafeArray.h"
#include "GeometryUtils.h"
#include "MapConstants.h"
#include "StateEstimator.h"
#include <algorithm> // For min/max functions

// Define missing constants and functions
#define CHECK_POINT ((Point*)0xDEADBEEF)
// resetImuTimeout() is defined in StateEstimator.cpp

// Node implementation
Node::Node() {
    init();
}

Node::Node(Point *aPoint, Node *aParentNode) {
    point = aPoint;
    parent = aParentNode;
}

void Node::init() {
    point = NULL;
    parent = NULL;
    f = g = h = 0;
    opened = false;
    closed = false;
}

void Node::dealloc() {
    // Points are owned by other objects, don't delete them
    point = NULL;
    parent = NULL;
}

// NodeList implementation
NodeList::NodeList() {
    init();
}

NodeList::NodeList(short aNumNodes) {
    init();
    alloc(aNumNodes);
}

void NodeList::init() {
    numNodes = 0;
    nodes = NULL;
}

NodeList::~NodeList() {
    dealloc();
}

bool NodeList::alloc(short aNumNodes) {
    if (aNumNodes == numNodes) return true;
    if ((aNumNodes < 0) || (aNumNodes > 20000)) {
        CONSOLE.println("ERROR NodeList::alloc invalid number");
        return false;
    }
    Node* newNodes = new Node[aNumNodes + CHECK_CORRUPT];
    if (newNodes == NULL) {
        CONSOLE.println("ERROR NodeList::alloc");
        memoryAllocErrors++;
        return false;
    }
    if (nodes != NULL) {
        memcpy(newNodes, nodes, sizeof(Node) * std::min(numNodes, aNumNodes));
        if (aNumNodes < numNodes) {
            for (int i = aNumNodes; i < numNodes; i++) {
                // nodes[i].dealloc();
            }
        }
        if (nodes[numNodes].point != CHECK_POINT) memoryCorruptions++;
        delete[] nodes;
    }
    nodes = newNodes;
    numNodes = aNumNodes;
    nodes[numNodes].point = CHECK_POINT;
    return true;
}

void NodeList::dealloc() {
    if (nodes == NULL) return;
    for (int i = 0; i < numNodes; i++) {
        nodes[i].dealloc();
    }
    if (nodes[numNodes].point != CHECK_POINT) memoryCorruptions++;
    delete[] nodes;
    nodes = NULL;
    numNodes = 0;
}

// PathFinder implementation
bool PathFinder::findPath(Map &map, Point &src, Point &dst) {
    if ((memoryCorruptions != 0) || (memoryAllocErrors != 0)) {
        CONSOLE.println("ERROR findPath: memory errors");
        return false;
    }

    unsigned long nextProgressTime = 0;
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

    if (!ENABLE_PATH_FINDER) {
        // Path finder not enabled - create simple direct path
        if (!map.freePoints.alloc(2)) return false;
        map.freePoints.points[0].assign(src);
        map.freePoints.points[1].assign(dst);
        map.freePointsIdx = 0;
        return true;
    }

    CONSOLE.print("path finder is enabled");
#ifdef FLOAT_CALC
    CONSOLE.print(" (using FLOAT_CALC)");
#endif
    CONSOLE.println();

    // Create path-finder obstacles
    PolygonList pathFinderObstacles;
    if (!createPathFinderObstacles(map, pathFinderObstacles)) {
        return false;
    }

    // Create nodes
    NodeList pathFinderNodes;
    if (!createPathFinderNodes(map, pathFinderNodes, src, dst)) {
        return false;
    }

    // Find start and end nodes
    Node *start = &pathFinderNodes.nodes[pathFinderNodes.numNodes - 2];
    Node *end = &pathFinderNodes.nodes[pathFinderNodes.numNodes - 1];
    start->opened = true;

    int timeout = PATHFINDER_TIMEOUT;
    Node *currentNode = NULL;

    CONSOLE.print("freem=");
    CONSOLE.println(freeMemory());
    CONSOLE.println("starting path-finder");

    // A* main loop
    while (true) {
        if (millis() >= nextProgressTime) {
            nextProgressTime = millis() + PATHFINDER_PROGRESS_INTERVAL;
            CONSOLE.print(".");
            watchdogReset();
        }
        timeout--;
        if (timeout == 0) {
            CONSOLE.println("timeout");
            break;
        }

        // Find lowest f(x) node to process next
        int lowInd = -1;
        for (int i = 0; i < pathFinderNodes.numNodes; i++) {
            if ((pathFinderNodes.nodes[i].opened) && 
                ((lowInd == -1) || (pathFinderNodes.nodes[i].f < pathFinderNodes.nodes[lowInd].f))) {
                lowInd = i;
            }
        }

        if (lowInd == -1) break;
        currentNode = &pathFinderNodes.nodes[lowInd];

        // Check if we reached the destination
        if (GeometryUtils::distance(*currentNode->point, *end->point) < PATHFINDER_GOAL_TOLERANCE) break;

        // Move currentNode from open to closed
        currentNode->opened = false;
        currentNode->closed = true;

        // Process neighbors
        int neighborIdx = -1;
        while (true) {
            neighborIdx = findNextNeighbor(pathFinderNodes, pathFinderObstacles, *currentNode, neighborIdx, map);
            if (neighborIdx == -1) break;

            Node* neighbor = &pathFinderNodes.nodes[neighborIdx];

            if (millis() >= nextProgressTime) {
                nextProgressTime = millis() + 4000;
                CONSOLE.print("+");
                watchdogReset();
            }

            // Calculate g score
            float gScore = currentNode->g + GeometryUtils::distance(*currentNode->point, *neighbor->point);
            bool gScoreIsBest = false;
            bool found = neighbor->opened;

            if (!found) {
                // First time arriving at this node
                gScoreIsBest = true;
                neighbor->h = calcHeuristic(*neighbor->point, *end->point);
                neighbor->opened = true;
            } else if (gScore < neighbor->g) {
                // Found a better path to this node
                gScoreIsBest = true;
            }

            if (gScoreIsBest) {
                // Update optimal path to this node
                neighbor->parent = currentNode;
                neighbor->g = gScore;
                neighbor->f = neighbor->g + neighbor->h;
            }
        }
    }

    CONSOLE.print("finish nodes=");
    CONSOLE.print(pathFinderNodes.numNodes);
    CONSOLE.print(" duration=");
    CONSOLE.println(millis() - startTime);

    resetImuTimeout();

    // Extract path if found
    bool result = extractPath(currentNode, end, map);
    
    map.checkMemoryErrors();
    resetImuTimeout();
    return result;
}

float PathFinder::calcHeuristic(Point &pos0, Point &pos1) {
    // Manhattan distance heuristic
    return abs(pos0.x() - pos1.x()) + abs(pos0.y() - pos1.y());
}

int PathFinder::findNextNeighbor(NodeList &nodes, PolygonList &obstacles, Node &node, int startIdx, Map &map) {
    Point dbgSrcPt;
    dbgSrcPt.setXY(-999, -999);
    float dbgSrcDist = GeometryUtils::distance(*node.point, dbgSrcPt);
    bool debug = (dbgSrcDist < 0.5);

    if (debug) {
        CONSOLE.print("findNextNeighbor src=");
        CONSOLE.print((*node.point).x());
        CONSOLE.print(",");
        CONSOLE.println((*node.point).y());
    }

    for (int idx = startIdx + 1; idx < nodes.numNodes; idx++) {
        if (nodes.nodes[idx].opened) continue;
        if (nodes.nodes[idx].closed) continue;
        if (nodes.nodes[idx].point == node.point) continue;
        Point *pt = nodes.nodes[idx].point;

        if (debug) {
            CONSOLE.print("findNextNeighbor dst=");
            CONSOLE.print(pt->x());
            CONSOLE.print(",");
            CONSOLE.println(pt->y());
            CONSOLE.println("findNextNeighbor trigger debug");
        }

        bool validNeighbor = true;

        // Check intersection with obstacles
        for (int idx3 = 0; idx3 < obstacles.numPolygons; idx3++) {
            if (obstacles.polygons[idx3].numPoints < 3) continue;

            // Check perimeter (first polygon)
            if (idx3 == 0) {
                bool insidePeri = map.pointIsInsidePolygon(obstacles.polygons[idx3], *node.point);
                if (!insidePeri) {
                    Point sectPt;
                    if (GeometryUtils::linePolygonIntersectPoint(*node.point, *pt, obstacles.polygons[idx3], sectPt)) {
                        float dist = GeometryUtils::distance(*node.point, sectPt);
                        if (dist > 0.3) {
                            validNeighbor = false;
                            break;
                        }
                    }
                } else {
                    if (GeometryUtils::linePolygonIntersectionCount(*node.point, *pt, obstacles.polygons[idx3]) != 1) {
                        validNeighbor = false;
                        break;
                    }
                }
            } else {
                // Check exclusions and obstacles
                bool insideObstacle = map.pointIsInsidePolygon(obstacles.polygons[idx3], *node.point);
                if (insideObstacle) {
                    Point sectPt;
                    if (GeometryUtils::linePolygonIntersectPoint(*node.point, *pt, obstacles.polygons[idx3], sectPt)) {
                        float dist = GeometryUtils::distance(*node.point, sectPt);
                        if (dist > 0.3) {
                            validNeighbor = false;
                            break;
                        }
                    }
                } else {
                    if (GeometryUtils::linePolygonIntersection(*node.point, *pt, obstacles.polygons[idx3])) {
                        validNeighbor = false;
                        break;
                    }
                }
            }
        }

        if (validNeighbor) {
            return idx;
        }
    }
    return -1;
}

bool PathFinder::createPathFinderObstacles(Map &map, PolygonList &pathFinderObstacles) {
    int idx = 0;
    if (!pathFinderObstacles.alloc(1 + map.exclusions.numPolygons + map.obstacles.numPolygons)) return false;

    if (freeMemory() < MIN_FREE_MEMORY_BYTES) {
        CONSOLE.println("OUT OF MEMORY");
        return false;
    }

    // Offset perimeter (make bigger)
    if (!map.polygonOffset(map.perimeterPoints, pathFinderObstacles.polygons[idx], 0.04)) return false;
    idx++;

    // Offset exclusions (make smaller)
    for (int i = 0; i < map.exclusions.numPolygons; i++) {
        if (!map.polygonOffset(map.exclusions.polygons[i], pathFinderObstacles.polygons[idx], -0.04)) return false;
        idx++;
    }

    // Offset obstacles (make smaller)
    for (int i = 0; i < map.obstacles.numPolygons; i++) {
        if (!map.polygonOffset(map.obstacles.polygons[i], pathFinderObstacles.polygons[idx], -0.04)) return false;
        idx++;
    }

    return true;
}

bool PathFinder::createPathFinderNodes(Map &map, NodeList &pathFinderNodes, Point &src, Point &dst) {
    int allocNodeCount = map.exclusions.numPoints() + map.obstacles.numPoints() + map.perimeterPoints.numPoints + 2;
    CONSOLE.print("freem=");
    CONSOLE.print(freeMemory());
    CONSOLE.print("  allocating nodes ");
    CONSOLE.print(allocNodeCount);
    CONSOLE.print(" (");
    CONSOLE.print(sizeof(Node) * allocNodeCount);
    CONSOLE.println(" bytes)");

    if (!pathFinderNodes.alloc(allocNodeCount)) return false;
    for (int i = 0; i < pathFinderNodes.numNodes; i++) {
        pathFinderNodes.nodes[i].init();
    }

    int idx = 0;
    // Exclusion nodes
    for (int i = 0; i < map.exclusions.numPolygons; i++) {
        for (int j = 0; j < map.exclusions.polygons[i].numPoints; j++) {
            pathFinderNodes.nodes[idx].point = &map.exclusions.polygons[i].points[j];
            idx++;
        }
    }

    // Obstacle nodes
    for (int i = 0; i < map.obstacles.numPolygons; i++) {
        for (int j = 0; j < map.obstacles.polygons[i].numPoints; j++) {
            pathFinderNodes.nodes[idx].point = &map.obstacles.polygons[i].points[j];
            idx++;
        }
    }

    // Perimeter nodes
    for (int j = 0; j < map.perimeterPoints.numPoints; j++) {
        pathFinderNodes.nodes[idx].point = &map.perimeterPoints.points[j];
        idx++;
    }

    // Start node
    pathFinderNodes.nodes[idx].point = &src;
    idx++;

    // End node
    pathFinderNodes.nodes[idx].point = &dst;
    idx++;

    return true;
}

bool PathFinder::extractPath(Node *currentNode, Node *endNode, Map &map) {
    if ((currentNode != NULL) && (GeometryUtils::distance(*currentNode->point, *endNode->point) < 0.02)) {
        Node *curr = currentNode;
        int nodeCount = 0;
        while (curr) {
            nodeCount++;
            curr = curr->parent;
        }
        if (!map.freePoints.alloc(nodeCount)) return false;
        curr = currentNode;
        int idx = nodeCount - 1;
        while (curr) {
            map.freePoints.points[idx].assign(*curr->point);
            CONSOLE.print("node pt=");
            CONSOLE.print(curr->point->x());
            CONSOLE.print(",");
            CONSOLE.println(curr->point->y());
            idx--;
            curr = curr->parent;
        }
        map.freePointsIdx = 0;
        return true;
    } else {
        CONSOLE.println("pathfinder: no path");
        return false;
    }
}