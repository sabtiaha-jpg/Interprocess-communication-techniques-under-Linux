#include "genetic_algorithm.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

// Helper function to get random action
Action get_random_action() {
    return rand() % 9;  // 9 possible actions
}

// Create random chromosome with action sequence
Chromosome create_random_chromosome(Grid3D* grid, int max_actions) {
    Chromosome chrom;
    
    chrom.num_actions = rand() % (max_actions / 2) + (max_actions / 2);
    chrom.actions = malloc(chrom.num_actions * sizeof(Action));
    
    // Random start point (not in obstacle)
    do {
        chrom.start_point.x = rand() % grid->size_x;
        chrom.start_point.y = rand() % grid->size_y;
        chrom.start_point.z = rand() % grid->size_z;
    } while (is_collision(grid, chrom.start_point));
    
    // Generate random action sequence
    for (int i = 0; i < chrom.num_actions; i++) {
        chrom.actions[i] = get_random_action();
    }
    
    // Initialize other fields
    chrom.actual_path = NULL;
    chrom.path_length = 0;
    chrom.fitness = 0.0;
    chrom.survivors_rescued = 0;
    chrom.coverage = 0.0;
    chrom.total_risk = 0.0;
    chrom.battery_used = 0.0;
    chrom.survivors_found = NULL;
    
    return chrom;
}

// Execute action sequence to generate actual path
void execute_actions(Chromosome* chrom, Grid3D* grid) {
    if (chrom->actual_path) free(chrom->actual_path);
    
    // Allocate path array
    chrom->actual_path = malloc((chrom->num_actions + 1) * sizeof(Coordinate));
    chrom->path_length = 1;
    chrom->actual_path[0] = chrom->start_point;
    
    Coordinate current = chrom->start_point;
    
    for (int i = 0; i < chrom->num_actions; i++) {
        Coordinate next = current;
        
        switch (chrom->actions[i]) {
            case ACTION_UP:
                if (next.z < grid->size_z - 1) next.z++;
                break;
            case ACTION_DOWN:
                if (next.z > 0) next.z--;
                break;
            case ACTION_LEFT:
                if (next.x > 0) next.x--;
                break;
            case ACTION_RIGHT:
                if (next.x < grid->size_x - 1) next.x++;
                break;
            case ACTION_FORWARD:
                if (next.y < grid->size_y - 1) next.y++;
                break;
            case ACTION_BACKWARD:
                if (next.y > 0) next.y--;
                break;
            case ACTION_WAIT:
                // Stay in place
                break;
            case ACTION_SCAN:
                // Scanning increases coverage without moving
                break;
            case ACTION_RETURN:
                // Try to move toward base
                if (next.x < grid->base_station.x) next.x++;
                else if (next.x > grid->base_station.x) next.x--;
                if (next.y < grid->base_station.y) next.y++;
                else if (next.y > grid->base_station.y) next.y--;
                if (next.z > grid->base_station.z) next.z--;
                break;
        }
        
        // Check if move is valid
        if (is_valid_coordinate(grid, next) && !is_collision(grid, next)) {
            current = next;
        }
        // If invalid, stay in place
        
        chrom->actual_path[chrom->path_length] = current;
        chrom->path_length++;
    }
}

// Calculate priority with sensor data (CO2, temperature)
float calculate_sensor_priority(GridCell* cell) {
    if (cell->type != CELL_SURVIVOR) {
        return 0.0;
    }
    
    float priority = (float)cell->survivor_priority;
    
    // CO2 factor: higher CO2 indicates danger, increase priority
    // CO2 range: 70-100%, so factor is 0.7-1.0
    float co2_factor = cell->co2_level / 100.0;
    
    // Temperature factor: if temp > 40°C, increase priority (sign of danger)
    float temp_factor = (cell->temperature > 40.0) ? 1.2 : 1.0;
    
    // Final priority with sensor modulation
    float adjusted_priority = priority * co2_factor * temp_factor;
    
    return adjusted_priority > 0 ? adjusted_priority : 1.0;
}

// Evaluate chromosome fitness
void evaluate_chromosome(Chromosome* chrom, Grid3D* grid, 
                         double w1, double w2, double w3, double w4) {
    // Execute actions to get actual path
    execute_actions(chrom, grid);
    
    // Reset grid visited flags
    reset_visited(grid);
    
    // Initialize metrics
    chrom->survivors_rescued = 0;
    chrom->coverage = 0.0;
    chrom->total_risk = 0.0;
    chrom->battery_used = 0.0;
    
    int unique_cells = 0;
    int total_survivors_found = 0;
    
    if (chrom->survivors_found) free(chrom->survivors_found);
    chrom->survivors_found = calloc(grid->total_survivors, sizeof(bool));
    
    // Analyze the actual path
    for (int i = 0; i < chrom->path_length; i++) {
        Coordinate coord = chrom->actual_path[i];
        
        if (!is_valid_coordinate(grid, coord)) continue;
        
        GridCell* cell = grid->cells[coord.z][coord.y][coord.x];
        
        // Check if cell is visited for the first time
        if (!cell->visited) {
            unique_cells++;
            cell->visited = true;
            
            // Check for survivor and use sensor-based priority
            if (cell->type == CELL_SURVIVOR) {
                float sensor_priority = calculate_sensor_priority(cell);
                chrom->survivors_rescued += (int)sensor_priority;
                total_survivors_found++;
                
                // Add penalty for high CO2 areas (>90%)
                if (cell->co2_level > 90.0) {
                    chrom->total_risk += 2.0;  // Additional risk penalty
                }
            }
        }
        
        // Accumulate risk
        chrom->total_risk += cell->risk_level;
        
        // Battery consumption: moving costs 1, waiting costs 0.1
        if (i > 0) {
            Coordinate prev = chrom->actual_path[i-1];
            if (coord.x != prev.x || coord.y != prev.y || coord.z != prev.z) {
                chrom->battery_used += 1.0;  // Movement cost
            } else {
                chrom->battery_used += 0.1;  // Stationary cost
            }
        }
    }
    
    // Calculate coverage percentage
    int total_cells = grid->size_x * grid->size_y * grid->size_z;
    chrom->coverage = (double)unique_cells / total_cells * 100.0;
    
    // Fitness function
    double length_penalty = chrom->path_length * 0.05;
    double risk_penalty = chrom->total_risk * 0.1;
    double battery_penalty = chrom->battery_used * 0.02;
    
    chrom->fitness = w1 * chrom->survivors_rescued + 
                    w2 * chrom->coverage *10.0-
                    w3 * length_penalty -
                    w4 * (risk_penalty + battery_penalty);
}

// Deep copy chromosome function
Chromosome deep_copy_chromosome(Chromosome* src) {
    Chromosome dest;
    
    // Copy basic fields
    dest.num_actions = src->num_actions;
    dest.start_point = src->start_point;
    dest.fitness = src->fitness;
    
    // Allocate and copy actions
    if (src->num_actions > 0 && src->actions) {
        dest.actions = malloc(src->num_actions * sizeof(Action));
        memcpy(dest.actions, src->actions, src->num_actions * sizeof(Action));
    } else {
        dest.actions = NULL;
    }
    
    // Initialize other fields
    dest.actual_path = NULL;
    dest.path_length = 0;
    dest.survivors_rescued = 0;
    dest.coverage = 0.0;
    dest.total_risk = 0.0;
    dest.battery_used = 0.0;
    dest.survivors_found = NULL;
    
    return dest;
}

// Create initial population
Population create_initial_population(Grid3D* grid, int pop_size, int max_actions) {
    Population pop;
    pop.size = pop_size;
    pop.max_actions = max_actions;
    pop.chromosomes = malloc(pop_size * sizeof(Chromosome));
    
    for (int i = 0; i < pop_size; i++) {
        pop.chromosomes[i] = create_random_chromosome(grid, max_actions);
    }
    
    return pop;
}

// Evaluate entire population
void evaluate_population(Population* pop, Grid3D* grid, 
                         double w1, double w2, double w3, double w4) {
    for (int i = 0; i < pop->size; i++) {
        evaluate_chromosome(&pop->chromosomes[i], grid, w1, w2, w3, w4);
    }
}

// Tournament selection
Chromosome tournament_selection(Population* pop, int tournament_size) {
    Chromosome best;
    best.fitness = -INFINITY;
    
    for (int i = 0; i < tournament_size; i++) {
        int idx = rand() % pop->size;
        if (pop->chromosomes[idx].fitness > best.fitness) {
            best = pop->chromosomes[idx];
        }
    }
    
    return best;
}

// Single-point crossover
void single_point_crossover(Chromosome* parent1, Chromosome* parent2, 
                           Chromosome* child1, Chromosome* child2) {
    int min_len = (parent1->num_actions < parent2->num_actions) ? 
                  parent1->num_actions : parent2->num_actions;
    
    if (min_len < 2) {
        *child1 = *parent1;
        *child2 = *parent2;
        return;
    }
    
    int crossover_point = rand() % (min_len - 1) + 1;
    
    // Create child 1
    child1->num_actions = parent1->num_actions;
    child1->actions = malloc(child1->num_actions * sizeof(Action));
    child1->start_point = parent1->start_point;
    
    for (int i = 0; i < crossover_point; i++) {
        child1->actions[i] = parent1->actions[i];
    }
    for (int i = crossover_point; i < child1->num_actions; i++) {
        child1->actions[i] = parent2->actions[i];
    }
    
    // Create child 2
    child2->num_actions = parent2->num_actions;
    child2->actions = malloc(child2->num_actions * sizeof(Action));
    child2->start_point = parent2->start_point;
    
    for (int i = 0; i < crossover_point; i++) {
        child2->actions[i] = parent2->actions[i];
    }
    for (int i = crossover_point; i < child2->num_actions; i++) {
        child2->actions[i] = parent1->actions[i];
    }
    
    // Initialize other fields
    child1->actual_path = NULL;
    child1->path_length = 0;
    child1->fitness = 0.0;
    child1->survivors_found = NULL;
    
    child2->actual_path = NULL;
    child2->path_length = 0;
    child2->fitness = 0.0;
    child2->survivors_found = NULL;
}

// Mutation operator
void mutate_chromosome(Chromosome* chrom, double mutation_rate, Grid3D* grid) {
    for (int i = 0; i < chrom->num_actions; i++) {
        if ((double)rand() / RAND_MAX < mutation_rate) {
            chrom->actions[i] = get_random_action();
        }
    }
    
    // Small chance to change start point
    if ((double)rand() / RAND_MAX < mutation_rate * 0.1) {
        do {
            chrom->start_point.x = rand() % grid->size_x;
            chrom->start_point.y = rand() % grid->size_y;
            chrom->start_point.z = rand() % grid->size_z;
        } while (is_collision(grid, chrom->start_point));
    }
}

// Apply elitism
void apply_elitism(Population* pop, Population* new_pop, int elitism_count) {
    // Sort population by fitness (descending)
    for (int i = 0; i < pop->size - 1; i++) {
        for (int j = i + 1; j < pop->size; j++) {
            if (pop->chromosomes[i].fitness < pop->chromosomes[j].fitness) {
                Chromosome temp = pop->chromosomes[i];
                pop->chromosomes[i] = pop->chromosomes[j];
                pop->chromosomes[j] = temp;
            }
        }
    }
    
    // Copy elites to new population
    for (int i = 0; i < elitism_count; i++) {
        new_pop->chromosomes[i] = pop->chromosomes[i];
    }
}

// Evolve population
void evolve_population(Population* pop, Grid3D* grid, double mutation_rate, 
                       int tournament_size, int elitism_percent,
                       double w1, double w2, double w3, double w4) {
    // Evaluate current population
    evaluate_population(pop, grid, w1, w2, w3, w4);
    
    // Calculate number of elites
    int elitism_count = pop->size * elitism_percent / 100;
    
    // Create new population
    Population new_pop;
    new_pop.size = pop->size;
    new_pop.max_actions = pop->max_actions;
    new_pop.chromosomes = malloc(pop->size * sizeof(Chromosome));
    
    // Apply elitism
    apply_elitism(pop, &new_pop, elitism_count);
    
    // Fill rest of population with selection, crossover, mutation
    for (int i = elitism_count; i < pop->size; i += 2) {
        Chromosome parent1 = tournament_selection(pop, tournament_size);
        Chromosome parent2 = tournament_selection(pop, tournament_size);
        
        Chromosome child1, child2;
        single_point_crossover(&parent1, &parent2, &child1, &child2);
        
        mutate_chromosome(&child1, mutation_rate, grid);
        mutate_chromosome(&child2, mutation_rate, grid);
        
        new_pop.chromosomes[i] = child1;
        if (i + 1 < pop->size) {
            new_pop.chromosomes[i + 1] = child2;
        }
    }
    
    // Free old population (except elites which were copied)
    for (int i = elitism_count; i < pop->size; i++) {
        free_chromosome(&pop->chromosomes[i]);
    }
    free(pop->chromosomes);
    
    // Replace with new population
    pop->chromosomes = new_pop.chromosomes;
}

// Free chromosome memory
void free_chromosome(Chromosome* chrom) {
    if (chrom->actions) free(chrom->actions);
    if (chrom->actual_path) free(chrom->actual_path);
    if (chrom->survivors_found) free(chrom->survivors_found);
}

// Free population memory
void free_population(Population* pop) {
    for (int i = 0; i < pop->size; i++) {
        free_chromosome(&pop->chromosomes[i]);
    }
    free(pop->chromosomes);
}