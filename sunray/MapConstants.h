#ifndef MAP_CONSTANTS_H
#define MAP_CONSTANTS_H

// File format markers for binary serialization
#define POINT_FILE_MARKER           0xAA    // Marker for Point data in binary files
#define POLYGON_FILE_MARKER         0xBB    // Marker for Polygon data in binary files
#define POLYGON_LIST_FILE_MARKER    0xCC    // Marker for PolygonList data in binary files

// Memory corruption check constants
#define CHECK_CORRUPT               1       // Enable memory corruption checks
#define CHECK_ID                    0x4A4A  // Magic number for corruption detection

// Coordinate system constants
#define COORDINATE_SCALE_FACTOR     100     // Scale factor for converting float to int coordinates (cm precision)

// Boundary values for safe allocation limits
#define MAX_POLYGON_POINTS          10000   // Maximum number of points per polygon
#define MAX_POLYGON_LIST_SIZE       5000    // Maximum number of polygons in a list
#define MAX_NODE_LIST_SIZE          20000   // Maximum number of nodes for pathfinding

// Extreme values for boundary calculations
#define EXTREME_COORDINATE_MAX      9999.0f // Large positive value for min/max calculations
#define EXTREME_COORDINATE_MIN      -9999.0f // Large negative value for min/max calculations

// Pathfinding constants
#define PATHFINDER_TIMEOUT          1000    // Maximum iterations for pathfinding algorithm
#define PATHFINDER_GOAL_TOLERANCE   0.02f   // Distance tolerance for reaching pathfinding goal
#define PATHFINDER_OBSTACLE_OFFSET  0.04f   // Offset distance for obstacle avoidance
#define PATHFINDER_SAFETY_DISTANCE  0.3f    // Safety distance from obstacles

// Memory management
#define MIN_FREE_MEMORY_BYTES       5000    // Minimum free memory required for pathfinding

// Progress reporting intervals
#define PATHFINDER_PROGRESS_INTERVAL 4000   // Milliseconds between progress reports

#endif // MAP_CONSTANTS_H