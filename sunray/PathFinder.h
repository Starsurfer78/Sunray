#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "Point.h"
#include "Polygon.h"
#include "PolygonList.h"

// Forward declarations
class Map;

// Node class for A* pathfinding
class Node {
public:
    Point *point;
    Node *parent;
    float f, g, h;
    bool opened;
    bool closed;
    
    Node();
    Node(Point *aPoint, Node *aParentNode);
    void init();
    void dealloc();
};

// NodeList class for managing nodes
class NodeList {
public:
    Node *nodes;
    short numNodes;
    
    NodeList();
    NodeList(short aNumNodes);
    ~NodeList();
    void init();
    bool alloc(short aNumNodes);
    void dealloc();
};

// PathFinder class implementing A* algorithm
class PathFinder {
public:
    static bool findPath(Map &map, Point &src, Point &dst);
    
private:
    static float calcHeuristic(Point &pos0, Point &pos1);
    static int findNextNeighbor(NodeList &nodes, PolygonList &obstacles, Node &node, int startIdx, Map &map);
    static bool createPathFinderObstacles(Map &map, PolygonList &pathFinderObstacles);
    static bool createPathFinderNodes(Map &map, NodeList &pathFinderNodes, Point &src, Point &dst);
    static bool extractPath(Node *currentNode, Node *endNode, Map &map);
};

#endif // PATHFINDER_H