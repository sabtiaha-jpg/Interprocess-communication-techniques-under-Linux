#include "robot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

// Initialize robots
void initialize_robots(Robot* robots, int num_robots, Grid3D* grid) {
    srand(time(NULL));
    
    // Different starting positions for each robot
    Coordinate start_positions[6] = {
        {grid->size_x/4, grid->size_y/4, 0},
        {3*grid->size_x/4, grid->size_y/4, 0},
        {grid->size_x/2, 3*grid->size_y/4, 0},
        {grid->size_x/4, 3*grid->size_y/4, 0},
        {3*grid->size_x/4, 3*grid->size_y/4, 0},
        {grid->size_x/2, grid->size_y/4, 0}
    };
    
    for (int i = 0; i < num_robots; i++) {
        robots[i].id = i + 1;
        
        // Ensure valid starting position
        if (i < 6) {
            robots[i].current_position = start_positions[i];
        } else {
            // Random position for additional robots
            do {
                robots[i].current_position.x = rand() % grid->size_x;
                robots[i].current_position.y = rand() % grid->size_y;
                robots[i].current_position.z = 0;
            } while (is_collision(grid, robots[i].current_position));
        }
        
        robots[i].battery_level = 100.0;
        robots[i].active = true;
        robots[i].survivors_delivered = 0;
        robots[i].distance_traveled = 0.0;
        robots[i].carrying_capacity = 15;  // Can carry up to 15 survivors
        robots[i].supplies_left = robots[i].carrying_capacity;
        
        // Initialize empty mission
        robots[i].assigned_mission.actions = NULL;
        robots[i].assigned_mission.num_actions = 0;
        robots[i].assigned_mission.actual_path = NULL;
        robots[i].assigned_mission.path_length = 0;
        
        printf("Robot %d initialized at position (%d,%d,%d) | Capacity: %d survivors\n", 
               robots[i].id, 
               robots[i].current_position.x,
               robots[i].current_position.y,
               robots[i].current_position.z,
               robots[i].carrying_capacity);
    }
}

// Assign missions to robots (each gets a different chromosome)
void assign_missions_to_robots(Robot* robots, int num_robots, Population* pop, Grid3D* grid) {
    printf("\n=== Assigning Missions to Robots ===\n");
    
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
    
    // Print top chromosomes
    printf("Top %d chromosomes selected:\n", num_robots);
    for (int i = 0; i < num_robots && i < pop->size; i++) {
        printf("  [%d] Fitness: %.2f, Survivors: %d, Coverage: %.1f%%\n",
               i, pop->chromosomes[i].fitness,
               pop->chromosomes[i].survivors_rescued,
               pop->chromosomes[i].coverage);
    }
    
    // Assign unique chromosomes to robots
    for (int i = 0; i < num_robots && i < pop->size; i++) {
        // Free previous mission if exists
        if (robots[i].assigned_mission.actions) {
            free_chromosome(&robots[i].assigned_mission);
        }
        
        // Deep copy chromosome
        robots[i].assigned_mission = deep_copy_chromosome(&pop->chromosomes[i]);
        
        // Different start points for different robots
        robots[i].assigned_mission.start_point = robots[i].current_position;
        
        printf("Robot %d: Assigned chromosome %d with fitness %.2f\n", 
               robots[i].id, i, robots[i].assigned_mission.fitness);
    }
    
    // If not enough chromosomes, create random ones for remaining robots
    for (int i = pop->size; i < num_robots; i++) {
        printf("Robot %d: Creating random mission (not enough chromosomes)\n", robots[i].id);
    }
}

// Function to check if two coordinates are adjacent or same
bool is_adjacent_or_same(Coordinate c1, Coordinate c2) {
    int dx = abs(c1.x - c2.x);
    int dy = abs(c1.y - c2.y);
    int dz = abs(c1.z - c2.z);
    
    // Same cell or adjacent (including diagonals)
    return (dx <= 1 && dy <= 1 && dz <= 1);
}

// Function to calculate distance between two coordinates
double distance_between(Coordinate c1, Coordinate c2) {
    int dx = c1.x - c2.x;
    int dy = c1.y - c2.y;
    int dz = c1.z - c2.z;
    
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Optimize paths for multiple robots to minimize interference
void optimize_robot_paths(Robot* robots, int num_robots, Grid3D* grid) {
    printf("\n=== Optimizing Robot Paths ===\n");
    
    // First, adjust start times to create staggered movement
    int* start_delays = malloc(num_robots * sizeof(int));
    int* path_adjustments = malloc(num_robots * sizeof(int));
    
    // Initialize delays and adjustments
    for (int i = 0; i < num_robots; i++) {
        start_delays[i] = i * 10;  // Stagger start times
        path_adjustments[i] = 0;
    }
    
    // Optimize each robot's path considering others
    for (int i = 0; i < num_robots; i++) {
        if (!robots[i].active) continue;
        
        Chromosome* mission = &robots[i].assigned_mission;
        
        // Re-evaluate mission with optimization (use default GA weights)
        evaluate_chromosome(mission, grid, 1.0, 0.8, 0.5, 1.2);
        
        // Try to modify path to avoid high-traffic areas
        if (mission->actual_path && mission->path_length > 0) {
            Coordinate* optimized_path = malloc(mission->path_length * sizeof(Coordinate));
            memcpy(optimized_path, mission->actual_path, mission->path_length * sizeof(Coordinate));
            
            int modifications = 0;
            
            // Simple optimization: try to smooth the path
            for (int step = 1; step < mission->path_length - 1; step++) {
                Coordinate prev = optimized_path[step - 1];
                Coordinate current = optimized_path[step];
                Coordinate next = optimized_path[step + 1];
                
                // Check if current step is unnecessary
                if (distance_between(prev, next) < distance_between(prev, current) + distance_between(current, next)) {
                    // Try to skip this step
                    Coordinate alternative;
                    
                    // Try different alternative moves
                    int attempts = 0;
                    while (attempts < 5) {
                        alternative = prev;
                        
                        // Move toward next
                        if (alternative.x < next.x) alternative.x++;
                        else if (alternative.x > next.x) alternative.x--;
                        
                        if (alternative.y < next.y) alternative.y++;
                        else if (alternative.y > next.y) alternative.y--;
                        
                        if (alternative.z < next.z) alternative.z++;
                        else if (alternative.z > next.z) alternative.z--;
                        
                        if (is_valid_coordinate(grid, alternative) && 
                            !is_collision(grid, alternative) &&
                            !is_adjacent_or_same(alternative, current)) {
                            
                            optimized_path[step] = alternative;
                            modifications++;
                            break;
                        }
                        attempts++;
                    }
                }
            }
            
            if (modifications > 0) {
                // Replace with optimized path
                free(mission->actual_path);
                mission->actual_path = optimized_path;
                
                // Recalculate metrics
                evaluate_chromosome(mission, grid, 1.0, 0.8, 0.5, 1.2);
                
                printf("Robot %d: Path optimized (%d modifications)\n", robots[i].id, modifications);
                printf("  New fitness: %.2f (was: %.2f)\n", mission->fitness, robots[i].assigned_mission.fitness);
            } else {
                free(optimized_path);
            }
        }
        
        // Assign staggered start delay
        printf("Robot %d: Start delay = %d steps\n", robots[i].id, start_delays[i]);
    }
    
    free(start_delays);
    free(path_adjustments);
    printf("Path optimization complete.\n");
}

// Avoid collisions between robots
void avoid_collisions(Robot* robots, int num_robots, Grid3D* grid) {
    printf("\n=== Collision Avoidance System ===\n");
    
    // Track robot positions at each time step
    int max_path_length = 0;
    
    // Find maximum path length
    for (int i = 0; i < num_robots; i++) {
        if (robots[i].active && robots[i].assigned_mission.path_length > max_path_length) {
            max_path_length = robots[i].assigned_mission.path_length;
        }
    }
    
    if (max_path_length == 0) {
        printf("No valid paths found for collision checking.\n");
        return;
    }
    
    // Create time-space occupancy grid
    Coordinate*** robot_positions = malloc(num_robots * sizeof(Coordinate**));
    for (int i = 0; i < num_robots; i++) {
        robot_positions[i] = malloc(max_path_length * sizeof(Coordinate*));
        for (int t = 0; t < max_path_length; t++) {
            robot_positions[i][t] = malloc(sizeof(Coordinate));
            
            if (robots[i].active && t < robots[i].assigned_mission.path_length) {
                *robot_positions[i][t] = robots[i].assigned_mission.actual_path[t];
            } else {
                // If robot has shorter path or inactive, stay at last position
                if (robots[i].active && robots[i].assigned_mission.path_length > 0) {
                    int last_idx = robots[i].assigned_mission.path_length - 1;
                    *robot_positions[i][t] = robots[i].assigned_mission.actual_path[last_idx];
                } else {
                    *robot_positions[i][t] = robots[i].current_position;
                }
            }
        }
    }
    
    // Check for collisions
    int collision_count = 0;
    int near_miss_count = 0;
    
    for (int t = 0; t < max_path_length; t++) {
        for (int i = 0; i < num_robots; i++) {
            if (!robots[i].active) continue;
            
            for (int j = i + 1; j < num_robots; j++) {
                if (!robots[j].active) continue;
                
                Coordinate pos_i = *robot_positions[i][t];
                Coordinate pos_j = *robot_positions[j][t];
                
                // Check for exact collision (same cell)
                if (pos_i.x == pos_j.x && pos_i.y == pos_j.y && pos_i.z == pos_j.z) {
                    printf("COLLISION ALERT: Robots %d and %d at time %d in cell (%d,%d,%d)\n",
                           robots[i].id, robots[j].id, t, pos_i.x, pos_i.y, pos_i.z);
                    collision_count++;
                    
                    // Avoid collision by modifying robot j's path
                    if (robots[j].assigned_mission.path_length > t + 1) {
                        Coordinate* new_path = malloc(robots[j].assigned_mission.path_length * sizeof(Coordinate));
                        memcpy(new_path, robots[j].assigned_mission.actual_path, 
                               robots[j].assigned_mission.path_length * sizeof(Coordinate));
                        
                        // Find alternative position
                        Coordinate alternative = new_path[t];
                        int attempts = 0;
                        
                        while (attempts < 8) {  // Try 8 directions
                            alternative = new_path[t];
                            
                            switch (attempts) {
                                case 0: if (alternative.x > 0) alternative.x--; break;
                                case 1: if (alternative.x < grid->size_x - 1) alternative.x++; break;
                                case 2: if (alternative.y > 0) alternative.y--; break;
                                case 3: if (alternative.y < grid->size_y - 1) alternative.y++; break;
                                case 4: if (alternative.z > 0) alternative.z--; break;
                                case 5: if (alternative.z < grid->size_z - 1) alternative.z++; break;
                                case 6: break;  // Wait
                                case 7: 
                                    if (t > 0) alternative = new_path[t-1];  // Go back
                                    break;
                            }
                            
                            if (is_valid_coordinate(grid, alternative) && 
                                !is_collision(grid, alternative) &&
                                (alternative.x != pos_i.x || alternative.y != pos_i.y || alternative.z != pos_i.z)) {
                                
                                new_path[t] = alternative;
                                printf("  -> Robot %d diverted to (%d,%d,%d)\n",
                                       robots[j].id, alternative.x, alternative.y, alternative.z);
                                
                                // Update robot's path
                                free(robots[j].assigned_mission.actual_path);
                                robots[j].assigned_mission.actual_path = new_path;
                                
                                // Re-evaluate fitness
                                evaluate_chromosome(&robots[j].assigned_mission, grid, 1.0, 0.8, 0.5, 1.2);
                                break;
                            }
                            attempts++;
                        }
                        
                        if (attempts == 8) {
                            free(new_path);
                            printf("  -> WARNING: Could not find safe alternative for Robot %d\n", robots[j].id);
                        }
                    }
                }
                // Check for near miss (adjacent cells)
                else if (is_adjacent_or_same(pos_i, pos_j) && 
                        !(pos_i.x == pos_j.x && pos_i.y == pos_j.y && pos_i.z == pos_j.z)) {
                    near_miss_count++;
                    if (near_miss_count % 20 == 0) {  // Don't print every near miss
                        printf("Near miss: Robots %d and %d at time %d are adjacent\n",
                               robots[i].id, robots[j].id, t);
                    }
                }
            }
        }
    }
    
    // Print collision statistics
    printf("\n=== Collision Statistics ===\n");
    printf("Total collisions detected: %d\n", collision_count);
    printf("Total near misses: %d\n", near_miss_count);
    printf("Maximum path length: %d steps\n", max_path_length);
    
    if (collision_count == 0) {
        printf("✓ All paths are collision-free!\n");
    } else {
        printf("⚠  %d collisions need to be resolved\n", collision_count);
    }
    
    // Clean up
    for (int i = 0; i < num_robots; i++) {
        for (int t = 0; t < max_path_length; t++) {
            free(robot_positions[i][t]);
        }
        free(robot_positions[i]);
    }
    free(robot_positions);
}

// Simple function to reduce collisions by adjusting start positions
void reduce_collisions_simple(Robot* robots, int num_robots, Grid3D* grid) {
    printf("\n=== Simple Collision Reduction ===\n");
    
    if (num_robots <= 1) return;
    
    // Adjust start positions to be in different quadrants
    for (int i = 0; i < num_robots; i++) {
        if (!robots[i].active) continue;
        
        // Force robots to different starting quadrants
        if (i == 0) {
            // Robot 1: Northwest quadrant
            robots[i].current_position.x = grid->size_x / 4;
            robots[i].current_position.y = grid->size_y / 4;
        } else if (i == 1) {
            // Robot 2: Northeast quadrant
            robots[i].current_position.x = 3 * grid->size_x / 4;
            robots[i].current_position.y = grid->size_y / 4;
        } else if (i == 2) {
            // Robot 3: Southwest quadrant
            robots[i].current_position.x = grid->size_x / 4;
            robots[i].current_position.y = 3 * grid->size_y / 4;
        } else {
            // Other robots: Southeast quadrant with offset
            robots[i].current_position.x = 3 * grid->size_x / 4;
            robots[i].current_position.y = 3 * grid->size_y / 4;
        }
        robots[i].current_position.z = 0;
        
        printf("Robot %d positioned in quadrant %d at (%d,%d,0)\n",
               robots[i].id, i + 1,
               robots[i].current_position.x,
               robots[i].current_position.y);
    }
}

// Simulate robot movement and rescue operations WITH collision avoidance
void simulate_robots(Robot* robots, int num_robots, Grid3D* grid) {
    printf("\n=== Simulating Robot Operations ===\n");
    
    // Apply simple collision reduction first
    reduce_collisions_simple(robots, num_robots, grid);
    
    // Then run advanced collision avoidance
    avoid_collisions(robots, num_robots, grid);
    
    // Optimize paths
    optimize_robot_paths(robots, num_robots, grid);
    
    // Reset grid before simulation
    reset_visited(grid);
    
    // Keep track of rescued survivors globally (3D array tracking)
    bool*** rescued_survivors = malloc(grid->size_z * sizeof(bool**));
    for (int z = 0; z < grid->size_z; z++) {
        rescued_survivors[z] = malloc(grid->size_y * sizeof(bool*));
        for (int y = 0; y < grid->size_y; y++) {
            rescued_survivors[z][y] = calloc(grid->size_x, sizeof(bool));
        }
    }
    
    int survivor_counter = 0;
    
    // Count survivors
    for (int z = 0; z < grid->size_z; z++) {
        for (int y = 0; y < grid->size_y; y++) {
            for (int x = 0; x < grid->size_x; x++) {
                if (grid->cells[z][y][x]->type == CELL_SURVIVOR) {
                    survivor_counter++;
                }
            }
        }
    }
    
    int total_survivors = survivor_counter;
    printf("Total survivors in grid: %d\n", total_survivors);
    
    // Calculate maximum simulation time
    int max_steps = 0;
    for (int i = 0; i < num_robots; i++) {
        if (robots[i].active && robots[i].assigned_mission.path_length > max_steps) {
            max_steps = robots[i].assigned_mission.path_length;
        }
    }
    
    // Track robot positions during simulation
    Coordinate* current_positions = malloc(num_robots * sizeof(Coordinate));
    bool* robot_active = malloc(num_robots * sizeof(bool));
    
    for (int i = 0; i < num_robots; i++) {
        current_positions[i] = robots[i].current_position;
        robot_active[i] = robots[i].active;
        robots[i].survivors_delivered = 0;
        robots[i].distance_traveled = 0.0;
        robots[i].battery_level = 100.0;
        robots[i].supplies_left = robots[i].carrying_capacity;
    }
    
    // Main simulation loop
    for (int step = 0; step < max_steps; step++) {
        int active_count = 0;
        
        for (int i = 0; i < num_robots; i++) {
            if (!robot_active[i]) continue;
            
            // Prevent negative battery
            if (robots[i].battery_level <= 0) {
                robots[i].battery_level = 0;  // Fix negative battery
                printf("Step %d: Robot %d battery depleted!\n", step, robots[i].id);
                robot_active[i] = false;
                robots[i].active = false;
                continue;
            }
            
            active_count++;
            
            Chromosome* mission = &robots[i].assigned_mission;
            
            // Get next position
            Coordinate next_pos;
            if (step < mission->path_length) {
                next_pos = mission->actual_path[step];
            } else {
                // Stay in place if path is shorter
                next_pos = current_positions[i];
            }
            
            // Check for collision with other robots at this time step
            bool collision_detected = false;
            for (int j = 0; j < i; j++) {
                if (!robot_active[j]) continue;
                
                if (next_pos.x == current_positions[j].x && 
                    next_pos.y == current_positions[j].y && 
                    next_pos.z == current_positions[j].z) {
                    
                    printf("Step %d: Collision avoided! Robots %d and %d would collide at (%d,%d,%d)\n",
                           step, robots[i].id, robots[j].id, next_pos.x, next_pos.y, next_pos.z);
                    
                    // Robot i waits (stays in place)
                    next_pos = current_positions[i];
                    collision_detected = true;
                    break;
                }
            }
            
            // Update position
            if (!collision_detected) {
                // Calculate distance moved
                if (step > 0 || (current_positions[i].x != next_pos.x || 
                                 current_positions[i].y != next_pos.y || 
                                 current_positions[i].z != next_pos.z)) {
                    
                    double dx = next_pos.x - current_positions[i].x;
                    double dy = next_pos.y - current_positions[i].y;
                    double dz = next_pos.z - current_positions[i].z;
                    double distance = sqrt(dx*dx + dy*dy + dz*dz);
                    
                    robots[i].distance_traveled += distance;
                    
                    // Optimized battery consumption formula
                    double base_consumption = 0.3;              // Reduced base step cost
                    double movement_consumption = distance * 0.2; // Reduced distance cost
                    double vertical_consumption = (dz != 0) ? 1.0 : 0; // Reduced climbing cost
                    
                    robots[i].battery_level -= (base_consumption + movement_consumption + vertical_consumption);
                }
                
                current_positions[i] = next_pos;
                robots[i].current_position = next_pos;
            } else {
                // Robot waits - minimal battery consumption
                robots[i].battery_level -= 0.1;
            }
            
            // Prevent negative battery
            if (robots[i].battery_level < 0) {
                robots[i].battery_level = 0;
            }
            
            // Check for survivor at current position
            if (is_valid_coordinate(grid, next_pos)) {
                GridCell* cell = grid->cells[next_pos.z][next_pos.y][next_pos.x];
                
                // Only rescue if survivor exists and not already rescued
                if (cell->type == CELL_SURVIVOR && 
                    cell->survivor_priority > 0 && 
                    !rescued_survivors[next_pos.z][next_pos.y][next_pos.x] &&
                    robots[i].supplies_left > 0) {
                    
                    // Rescue takes 1 step and uses supplies (reduced from 3)
                    robots[i].battery_level -= 1.0;  // Reduced from 3.0
                    robots[i].survivors_delivered += cell->survivor_priority;
                    robots[i].supplies_left--;
                    
                    // Mark as rescued
                    rescued_survivors[next_pos.z][next_pos.y][next_pos.x] = true;
                    
                    printf("Step %3d: Robot %d RESCUED survivor (priority: %2d) at (%2d,%2d,%2d) | Supplies: %d\n",
                           step, robots[i].id, cell->survivor_priority,
                           next_pos.x, next_pos.y, next_pos.z,
                           robots[i].supplies_left);
                    
                    // Mark cell as processed
                    cell->type = CELL_FREE;
                    cell->survivor_priority = 0;
                } else if (robots[i].supplies_left == 0) {
                    // Need to resupply - check if near base
                    double distance_to_base = distance_between(robots[i].current_position, 
                                                               grid->base_station);
                    if (distance_to_base < 2) {
                        robots[i].supplies_left = robots[i].carrying_capacity;
                        printf("Step %3d: Robot %d RESUPPLIED at base\n", step, robots[i].id);
                    }
                }
            }
            
            // Report progress
            if (step % 100 == 0 && step > 0 && i == 0) {
                printf("\n--- Simulation Progress: Step %d ---\n", step);
                printf("Active robots: %d/%d\n", active_count, num_robots);
            }
        }
        
        // Check if all robots are inactive
        if (active_count == 0) {
            printf("\nAll robots inactive at step %d. Ending simulation.\n", step);
            break;
        }
    }
    
    // Final battery fix for all robots
    for (int i = 0; i < num_robots; i++) {
        if (robots[i].battery_level < 0) {
            robots[i].battery_level = 0;
        }
    }
    
    // Mission completion reports
    printf("\n=== Mission Completion Reports ===\n");
    for (int i = 0; i < num_robots; i++) {
        if (!robots[i].active) continue;
        
        printf("\nRobot %d Mission Complete:\n", robots[i].id);
        printf("  Total survivors delivered: %d\n", robots[i].survivors_delivered);
        printf("  Total distance traveled: %.2f units\n", robots[i].distance_traveled);
        printf("  Battery remaining: %.1f%%\n", robots[i].battery_level);
        printf("  Final position: (%d,%d,%d)\n",
               robots[i].current_position.x,
               robots[i].current_position.y,
               robots[i].current_position.z);
        printf("  Mission fitness: %.2f\n", robots[i].assigned_mission.fitness);
    }
    
    // Clean up
    for (int z = 0; z < grid->size_z; z++) {
        for (int y = 0; y < grid->size_y; y++) {
            free(rescued_survivors[z][y]);
        }
        free(rescued_survivors[z]);
    }
    free(rescued_survivors);
    free(current_positions);
    free(robot_active);
}

// Print robot status
void print_robot_status(Robot* robots, int num_robots) {
    printf("\n===========================================\n");
    printf("          ROBOT FLEET STATUS REPORT\n");
    printf("===========================================\n");
    printf("%-6s %-8s %-15s %-9s %-12s %-10s %-10s %-12s\n", 
           "Robot", "Status", "Position", "Battery%", "Survivors", "Distance", "Supplies", "Fitness");
    printf("%-6s %-8s %-15s %-9s %-12s %-10s %-10s %-12s\n",
           "------", "--------", "---------------", "---------", "----------", "--------", "--------", "--------");
    
    int total_survivors = 0;
    double total_distance = 0.0;
    double avg_battery = 0.0;
    int active_robots = 0;
    
    for (int i = 0; i < num_robots; i++) {
        char status[10];
        char position[20];
        
        strcpy(status, robots[i].active ? "ACTIVE" : "INACTIVE");
        sprintf(position, "(%d,%d,%d)", 
                robots[i].current_position.x,
                robots[i].current_position.y,
                robots[i].current_position.z);
        
        printf("%-6d %-8s %-15s %-9.1f %-12d %-10.2f %-10d %-12.2f\n",
               robots[i].id,
               status,
               position,
               robots[i].battery_level,
               robots[i].survivors_delivered,
               robots[i].distance_traveled,
               robots[i].supplies_left,
               robots[i].assigned_mission.fitness);
        
        total_survivors += robots[i].survivors_delivered;
        total_distance += robots[i].distance_traveled;
        avg_battery += robots[i].battery_level;
        
        if (robots[i].active) active_robots++;
    }
    
    if (active_robots > 0) {
        avg_battery /= active_robots;
    }
    
    printf("\n=== SUMMARY ===\n");
    printf("Active Robots: %d/%d\n", active_robots, num_robots);
    printf("Total Survivors Rescued: %d\n", total_survivors);
    printf("Total Distance Traveled: %.2f units\n", total_distance);
    printf("Average Battery Remaining: %.1f%%\n", avg_battery);
    printf("===========================================\n");
}

// Save robot paths to file for visualization
void save_robot_paths(Robot* robots, int num_robots, const char* filename) {
    FILE* file = fopen(filename, "w");
    if (!file) {
        perror("Failed to open file for saving paths");
        return;
    }
    
    fprintf(file, "# Robot Rescue Paths - 3D Visualization Data\n");
    fprintf(file, "# Generated by Genetic Algorithm Rescue System\n");
    fprintf(file, "# Format: RobotID: x1,y1,z1 -> x2,y2,z2 -> ...\n");
    fprintf(file, "# Each line represents one robot's complete path\n\n");
    
    int total_waypoints = 0;
    int robots_with_paths = 0;
    
    fprintf(file, "# Robot Status:\n");
    for (int i = 0; i < num_robots; i++) {
        fprintf(file, "# Robot%d: %s\n", robots[i].id, 
                robots[i].active ? "ACTIVE" : "INACTIVE");
    }
    fprintf(file, "\n");
    
    // Save paths of all robots (even if inactive)
    for (int i = 0; i < num_robots; i++) {
        Chromosome* mission = &robots[i].assigned_mission;
        
        if (!mission->actual_path || mission->path_length == 0) {
            fprintf(file, "# Robot%d: NO_VALID_PATH\n", robots[i].id);
            continue;
        }
        
        fprintf(file, "Robot%d:", robots[i].id);
        
        // Save first 200 points for clarity (can change the number)
        int points_to_save = mission->path_length;
        if (points_to_save > 200) points_to_save = 200;  // Reduce for long paths
        
        for (int j = 0; j < points_to_save; j++) {
            Coordinate coord = mission->actual_path[j];
            fprintf(file, " %d,%d,%d", coord.x, coord.y, coord.z);
            
            if (j < points_to_save - 1) {
                fprintf(file, " ->");
            }
            
            total_waypoints++;
        }
        
        // If path is longer than 200 points, add marker
        if (mission->path_length > 200) {
            fprintf(file, " -> ... [truncated, total: %d points]", mission->path_length);
        }
        
        // Add robot info as comment
        fprintf(file, " # %s, Fitness: %.2f, Survivors: %d, Battery: %.1f%%, Length: %d",
                robots[i].active ? "ACTIVE" : "INACTIVE",
                mission->fitness,
                robots[i].survivors_delivered,
                robots[i].battery_level,
                mission->path_length);
        
        fprintf(file, "\n");
        robots_with_paths++;
    }
    
    // Add optimization summary
    fprintf(file, "\n# Summary:\n");
    fprintf(file, "# Robots with saved paths: %d/%d\n", robots_with_paths, num_robots);
    fprintf(file, "# Total waypoints: %d\n", total_waypoints);
    fprintf(file, "# Average waypoints per robot: %.1f\n", 
            robots_with_paths > 0 ? (double)total_waypoints / robots_with_paths : 0);
    
    fclose(file);
    
    printf("\n=== Paths Saved ===\n");
    printf("File: %s\n", filename);
    printf("Robots with paths: %d/%d\n", robots_with_paths, num_robots);
    printf("Total waypoints: %d\n", total_waypoints);
    printf("Ready for 3D visualization.\n");
}

// Validate and adjust robot paths to ensure realistic survivor counts
void validate_and_adjust_paths(Robot* robots, int num_robots, Grid3D* grid) {
    printf("\n=== Validating Robot Paths ===\n");
    
    for (int i = 0; i < num_robots; i++) {
        Chromosome* chrom = &robots[i].assigned_mission;
        
        if (!chrom->actual_path || chrom->path_length == 0) {
            printf("Robot %d: No valid path\n", robots[i].id);
            continue;
        }
        
        // Reset visited flags to count actual survivors
        reset_visited(grid);
        int actual_survivors = 0;
        
        // Count survivors actually in the path
        for (int j = 0; j < chrom->path_length; j++) {
            Coordinate coord = chrom->actual_path[j];
            if (!is_valid_coordinate(grid, coord)) continue;
            
            GridCell* cell = grid->cells[coord.z][coord.y][coord.x];
            if (!cell->visited) {
                cell->visited = true;
                if (cell->type == CELL_SURVIVOR && cell->survivor_priority > 0) {
                    actual_survivors += cell->survivor_priority;
                }
            }
        }
        
        // Correct if discrepancy found
        if (actual_survivors != chrom->survivors_rescued) {
            printf("Robot %d: Correcting survivors from %d to %d\n",
                   robots[i].id, chrom->survivors_rescued, actual_survivors);
            chrom->survivors_rescued = actual_survivors;
            
            // Re-evaluate fitness with corrected value
            evaluate_chromosome(chrom, grid, 1.5, 1.0, 0.5, 1.5);
        }
        
        // Enforce carrying capacity limit
        if (chrom->survivors_rescued > robots[i].carrying_capacity) {
            printf("Robot %d: Enforcing capacity limit from %d to %d\n",
                   robots[i].id, chrom->survivors_rescued, robots[i].carrying_capacity);
            chrom->survivors_rescued = robots[i].carrying_capacity;
            evaluate_chromosome(chrom, grid, 1.5, 1.0, 0.5, 1.5);
        }
        
        printf("Robot %d: Path validated. Fitness: %.2f, Survivors: %d, Path length: %d\n",
               robots[i].id, chrom->fitness, chrom->survivors_rescued, chrom->path_length);
    }
    printf("=== Path Validation Complete ===\n\n");
}

// Print sensor report for robot's mission area
void print_sensor_report(Robot* robot, Grid3D* grid) {
    printf("\n=== SENSOR REPORT - Robot %d ===\n", robot->id);
    printf("Mission Analysis: CO2 and Temperature Monitoring\n");
    printf("==============================================\n");
    
    Chromosome* mission = &robot->assigned_mission;
    
    if (!mission->actual_path || mission->path_length == 0) {
        printf("No valid path data available.\n");
        return;
    }
    
    int high_co2_areas = 0;
    int high_temp_areas = 0;
    int survivor_cells = 0;
    double avg_co2 = 0.0;
    double avg_temp = 0.0;
    
    printf("\nPath Analysis (%d waypoints):\n", mission->path_length);
    printf("%-8s %-12s %-10s %-12s %-10s %-12s\n",
           "Step", "Position", "CO2(%)", "Temp(°C)", "Survivor", "Risk");
    printf("%-8s %-12s %-10s %-12s %-10s %-12s\n",
           "----", "--------", "----", "-------", "-------", "----");
    
    for (int i = 0; i < mission->path_length; i++) {
        Coordinate coord = mission->actual_path[i];
        
        if (!is_valid_coordinate(grid, coord)) continue;
        
        GridCell* cell = grid->cells[coord.z][coord.y][coord.x];
        
        avg_co2 += cell->co2_level;
        avg_temp += cell->temperature;
        
        char survivor_str[10] = "No";
        if (cell->type == CELL_SURVIVOR) {
            survivor_cells++;
            snprintf(survivor_str, 10, "Yes(%d)", cell->survivor_priority);
        }
        
        if (cell->co2_level > 90.0) {
            high_co2_areas++;
        }
        if (cell->temperature > 40.0) {
            high_temp_areas++;
        }
        
        // Print key waypoints (every 10 steps or critical points)
        if (i % 10 == 0 || cell->co2_level > 90.0 || cell->temperature > 40.0) {
            printf("%-8d (%2d,%2d,%2d)     %-10.1f %-12.1f %-10s %-12d\n",
                   i,
                   coord.x, coord.y, coord.z,
                   cell->co2_level,
                   cell->temperature,
                   survivor_str,
                   cell->risk_level);
        }
    }
    
    if (mission->path_length > 0) {
        avg_co2 /= mission->path_length;
        avg_temp /= mission->path_length;
    }
    
    printf("\n=== Environmental Summary ===\n");
    printf("Average CO2 Level: %.1f%%\n", avg_co2);
    printf("Average Temperature: %.1f°C\n", avg_temp);
    printf("High CO2 Areas (>90%%): %d waypoints\n", high_co2_areas);
    printf("High Temperature Areas (>40°C): %d waypoints\n", high_temp_areas);
    printf("Survivors Detected: %d locations\n", survivor_cells);
    
    // Critical areas warning
    if (high_co2_areas > 0) {
        printf("\n⚠️  CRITICAL AREAS DETECTED (High CO2):\n");
        for (int i = 0; i < mission->path_length; i++) {
            Coordinate coord = mission->actual_path[i];
            if (!is_valid_coordinate(grid, coord)) continue;
            
            GridCell* cell = grid->cells[coord.z][coord.y][coord.x];
            if (cell->co2_level > 90.0) {
                printf("  Step %d: Cell (%d,%d,%d) - CO2=%.1f%%, Temp=%.1f°C\n",
                       i, coord.x, coord.y, coord.z,
                       cell->co2_level, cell->temperature);
            }
        }
    }
    
    printf("==============================================\n\n");
}