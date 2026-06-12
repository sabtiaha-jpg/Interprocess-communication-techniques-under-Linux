#ifndef COMPARISON_H
#define COMPARISON_H

#include "genetic_algorithm.h"

// Forward declaration (لأن AStarPath غير معروف بعد هنا)
typedef struct AStarPath AStarPath;

// Comparison functions
void compare_with_astar(Population* pop, Grid3D* grid, int top_n);
void print_astar_comparison(AStarPath* astar_path, Chromosome* ga_chromosome);

#endif