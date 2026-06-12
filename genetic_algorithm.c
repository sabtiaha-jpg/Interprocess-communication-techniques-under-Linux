#ifndef ASTAR_H
#define ASTAR_H

#include "grid.h"

typedef struct AStarPath {
    Coordinate* path;
    int length;
    double fitness;
    int survivors_rescued;
    double coverage;
    double risk;
} AStarPath;

// A* pathfinding functions
AStarPath* find_path_a_star(Grid3D* grid, Coordinate start, Coordinate goal);
void free_astar_path(AStarPath* path);

// Helper functions for comparison
double astar_path_get_fitness(AStarPath* path);
int astar_path_get_length(AStarPath* path);
int astar_path_get_survivors_rescued(AStarPath* path);
double astar_path_get_coverage(AStarPath* path);
double astar_path_get_risk(AStarPath* path);

#endif