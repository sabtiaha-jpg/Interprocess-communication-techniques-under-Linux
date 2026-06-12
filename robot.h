#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include "grid.h"

// Action-based chromosome representation
typedef enum {
    ACTION_UP,          // Increase z
    ACTION_DOWN,        // Decrease z  
    ACTION_LEFT,        // Decrease x
    ACTION_RIGHT,       // Increase x
    ACTION_FORWARD,     // Increase y
    ACTION_BACKWARD,    // Decrease y
    ACTION_WAIT,        // Stay in place
    ACTION_SCAN,        // Scan area
    ACTION_RETURN       // Return to base
} Action;

typedef struct {
    Action* actions;           // Sequence of actions
    int num_actions;           // Length of action sequence
    Coordinate start_point;    // Starting position
    Coordinate* actual_path;   // Actual coordinates after execution
    int path_length;           // Length of actual path
    double fitness;            // Fitness score
    int survivors_rescued;     // Number of survivors reached
    double coverage;           // Coverage percentage
    double total_risk;         // Total risk accumulated
    double battery_used;       // Battery consumption
    bool* survivors_found;     // Which survivors were found
} Chromosome;

typedef struct {
    Chromosome* chromosomes;
    int size;
    int max_actions;
} Population;

// Chromosome creation and evaluation
Chromosome create_random_chromosome(Grid3D* grid, int max_actions);
float calculate_sensor_priority(GridCell* cell);  // NEW: Sensor-based priority
void evaluate_chromosome(Chromosome* chrom, Grid3D* grid, 
                         double w1, double w2, double w3, double w4);
void free_chromosome(Chromosome* chrom);
Chromosome deep_copy_chromosome(Chromosome* src);

// Population management
Population create_initial_population(Grid3D* grid, int pop_size, int max_actions);
void evaluate_population(Population* pop, Grid3D* grid, 
                         double w1, double w2, double w3, double w4);
void free_population(Population* pop);

// Genetic operators
Chromosome tournament_selection(Population* pop, int tournament_size);
void single_point_crossover(Chromosome* parent1, Chromosome* parent2, 
                           Chromosome* child1, Chromosome* child2);
void mutate_chromosome(Chromosome* chrom, double mutation_rate, Grid3D* grid);
void apply_elitism(Population* pop, Population* new_pop, int elitism_count);

// Evolution
void evolve_population(Population* pop, Grid3D* grid, double mutation_rate, 
                       int tournament_size, int elitism_percent,
                       double w1, double w2, double w3, double w4);

// Helper functions
void execute_actions(Chromosome* chrom, Grid3D* grid);
void print_chromosome_info(Chromosome* chrom);

#endif
