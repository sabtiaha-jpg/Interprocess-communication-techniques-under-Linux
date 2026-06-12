#ifndef ROBOT_H
#define ROBOT_H

#include "genetic_algorithm.h"
#include <stdbool.h>

typedef struct {
    int id;
    Chromosome assigned_mission;
    Coordinate current_position;
    double battery_level;
    bool active;
    int survivors_delivered;
    double distance_traveled;
    int carrying_capacity;   // Max survivors this robot can carry
    int supplies_left;       // Current supplies remaining
} Robot;

// Helper functions
bool is_adjacent_or_same(Coordinate c1, Coordinate c2);
double distance_between(Coordinate c1, Coordinate c2);

// Robot management
void initialize_robots(Robot* robots, int num_robots, Grid3D* grid);
void assign_missions_to_robots(Robot* robots, int num_robots, Population* pop, Grid3D* grid);
void simulate_robots(Robot* robots, int num_robots, Grid3D* grid);
void print_robot_status(Robot* robots, int num_robots);

// Path optimization for multiple robots
void optimize_robot_paths(Robot* robots, int num_robots, Grid3D* grid);
void avoid_collisions(Robot* robots, int num_robots, Grid3D* grid);
void save_robot_paths(Robot* robots, int num_robots, const char* filename);
void print_sensor_report(Robot* robot, Grid3D* grid);  // Sensor data report
void validate_and_adjust_paths(Robot* robots, int num_robots, Grid3D* grid);  // NEW

#endif