#ifndef GRID_H
#define GRID_H

#include "config.h"
#include <stdbool.h>

typedef enum {
    CELL_FREE,
    CELL_OBSTACLE,
    CELL_SURVIVOR,
    CELL_HIGH_RISK,
    CELL_START,
    CELL_BASE
} CellType;

typedef struct {
    CellType type;
    int survivor_priority;  // 0-10 (0 = no survivor)
    int risk_level;         // 1-5
    bool visited;
    int robot_id;           // -1 if no robot
    float co2_level;        // CO2 percentage (0-100%)
    float temperature;      // Temperature in Celsius
    bool has_heat_signal;   // Heat signature detected
} GridCell;

typedef struct {
    int x, y, z;
} Coordinate;

typedef struct {
    GridCell**** cells;  // 3D array: cells[z][y][x]
    int size_x, size_y, size_z;
    Coordinate base_station;
    int total_survivors;
    int total_obstacles;
} Grid3D;

Grid3D* create_grid(Config cfg);
void initialize_grid(Grid3D* grid);
void free_grid(Grid3D* grid);
bool is_valid_coordinate(Grid3D* grid, Coordinate coord);
bool is_collision(Grid3D* grid, Coordinate coord);
bool is_survivor_cell(Grid3D* grid, Coordinate coord);
int get_survivor_priority(Grid3D* grid, Coordinate coord);
int get_risk_level(Grid3D* grid, Coordinate coord);
void reset_visited(Grid3D* grid);
void reset_grid_complete(Grid3D* grid);  // NEW FUNCTION - Complete grid reset
void print_grid_slice(Grid3D* grid, int z);

#endif