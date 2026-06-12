#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/msg.h>
#include <pthread.h>
#include <math.h>

#include "config.h"
#include "grid.h"
#include "genetic_algorithm.h"
#include "ipc.h"
#include "robot.h"

// A* result structure
typedef struct {
    double fitness;
    int length;
    int survivors_rescued;
    double coverage;
    double risk;
} AStarResult;

// Simple A* function
AStarResult find_path_a_star_simple(Grid3D* grid, Coordinate start, Coordinate goal) {
    AStarResult result = {0};
    
    // Simple direct path
    int dx = abs(goal.x - start.x);
    int dy = abs(goal.y - start.y);
    int dz = abs(goal.z - start.z);
    
    result.length = dx + dy + dz + 1;
    if (result.length <= 0) result.length = 1;
    
    // Calculate basic metrics
    reset_visited(grid);
    
    // Simple path (straight line from start to goal)
    Coordinate current = start;
    int steps = 0;
    int max_steps = result.length;
    int total_cells = grid->size_x * grid->size_y * grid->size_z;
    int visited_cells = 0;
    
    while (steps < max_steps && 
           (current.x != goal.x || current.y != goal.y || current.z != goal.z)) {
        
        if (is_valid_coordinate(grid, current)) {
            if (!grid->cells[current.z][current.y][current.x]->visited) {
                grid->cells[current.z][current.y][current.x]->visited = true;
                visited_cells++;
                
                if (is_survivor_cell(grid, current)) {
                    result.survivors_rescued += get_survivor_priority(grid, current);
                }
            }
            
            result.risk += get_risk_level(grid, current);
        }
        
        // Move toward goal
        if (current.x < goal.x) current.x++;
        else if (current.x > goal.x) current.x--;
        else if (current.y < goal.y) current.y++;
        else if (current.y > goal.y) current.y--;
        else if (current.z < goal.z) current.z++;
        else if (current.z > goal.z) current.z--;
        
        steps++;
    }
    
    // Add final position
    if (is_valid_coordinate(grid, current)) {
        if (!grid->cells[current.z][current.y][current.x]->visited) {
            visited_cells++;
            if (is_survivor_cell(grid, current)) {
                result.survivors_rescued += get_survivor_priority(grid, current);
            }
        }
        result.risk += get_risk_level(grid, current);
    }
    
    // Calculate coverage percentage
    result.coverage = total_cells > 0 ? (double)visited_cells / total_cells * 100.0 : 0.0;
    
    // Calculate fitness (same formula as GA)
    double length_penalty = result.length * 0.05;
    double risk_penalty = result.risk * 0.1;
    
    result.fitness = 1.0 * result.survivors_rescued + 
                    0.8 * result.coverage - 
                    0.5 * length_penalty - 
                    1.2 * risk_penalty;
    
    return result;
}

// Comparison function
void compare_ga_with_astar(Population* pop, Grid3D* grid) {
    printf("\n═══════════════════════════════════════════════════════\n");
    printf("           GENETIC ALGORITHM vs A* COMPARISON\n");
    printf("═══════════════════════════════════════════════════════\n");
    
    if (pop->size == 0) {
        printf("No chromosomes to compare.\n");
        return;
    }
    
    // Sort population to get best chromosome
    for (int i = 0; i < pop->size - 1; i++) {
        for (int j = i + 1; j < pop->size; j++) {
            if (pop->chromosomes[i].fitness < pop->chromosomes[j].fitness) {
                Chromosome temp = pop->chromosomes[i];
                pop->chromosomes[i] = pop->chromosomes[j];
                pop->chromosomes[j] = temp;
            }
        }
    }
    
    Chromosome* best_ga = &pop->chromosomes[0];
    
    // Use A* to find path from start to end
    Coordinate start = best_ga->start_point;
    Coordinate end;
    
    if (best_ga->path_length > 0) {
        end = best_ga->actual_path[best_ga->path_length - 1];
    } else {
        end = start;  // If no path, use start as end
    }
    
    printf("\nBest GA Chromosome:\n");
    printf("  Start: (%d,%d,%d), End: (%d,%d,%d)\n",
           start.x, start.y, start.z, end.x, end.y, end.z);
    
    AStarResult astar = find_path_a_star_simple(grid, start, end);
    
    // Print comparison table
    printf("\n╔════════════════════════════════════════════════════╗\n");
    printf("║              COMPARISON RESULTS                    ║\n");
    printf("╠════════════════════════════════════════════════════╣\n");
    printf("║ Metric           │ Genetic Algorithm │ A* Algorithm║\n");
    printf("╠══════════════════╪═══════════════════╪═════════════╣\n");
    printf("║ Fitness Score    │ %16.2f │ %11.2f ║\n", 
           best_ga->fitness, astar.fitness);
    printf("║ Path Length      │ %16d │ %11d ║\n", 
           best_ga->path_length, astar.length);
    printf("║ Survivors Saved  │ %16d │ %11d ║\n", 
           best_ga->survivors_rescued, astar.survivors_rescued);
    printf("║ Coverage         │ %15.1f%% │ %10.1f%% ║\n", 
           best_ga->coverage, astar.coverage);
    printf("║ Total Risk       │ %16.1f │ %11.1f ║\n", 
           best_ga->total_risk, astar.risk);
    printf("╚══════════════════╧═══════════════════╧═════════════╝\n");
    
    // Calculate similarity percentage
    double ga_fitness = best_ga->fitness;
    double astar_fitness = astar.fitness;
    
    double similarity = 0.0;
    
    if (fabs(astar_fitness) > 0.0001) {
        // Normalize scores for comparison
        double normalized_ga = ga_fitness / fabs(astar_fitness);
        double normalized_astar = astar_fitness / fabs(astar_fitness);
        
        similarity = 1.0 - fabs(normalized_ga - normalized_astar) / 2.0;
        if (similarity < 0) similarity = 0;
        if (similarity > 1) similarity = 1;
    }
    
    printf("\nSimilarity Analysis:\n");
    printf("  Similarity Score: %.1f%%\n", similarity * 100);
    
    if (similarity >= 0.9) {
        printf("  ★★★★★ EXCELLENT - GA matches A* optimal solution!\n");
    } else if (similarity >= 0.75) {
        printf("  ★★★★☆ VERY GOOD - GA is close to optimal solution\n");
    } else if (similarity >= 0.6) {
        printf("  ★★★☆☆ GOOD - GA provides acceptable solution\n");
    } else if (similarity >= 0.4) {
        printf("  ★★☆☆☆ FAIR - GA differs from optimal solution\n");
    } else {
        printf("  ★☆☆☆☆ POOR - GA solution needs improvement\n");
    }
    
    // Additional insights
    printf("\nKey Insights:\n");
    if (best_ga->fitness > astar.fitness) {
        printf("  • GA outperformed A* by %.2f points!\n", best_ga->fitness - astar.fitness);
        printf("  • This suggests GA found better multi-objective balance\n");
    } else if (astar.fitness > best_ga->fitness) {
        printf("  • A* found better solution by %.2f points\n", astar.fitness - best_ga->fitness);
        printf("  • Consider adjusting GA parameters for better results\n");
    } else {
        printf("  • Both algorithms found equally good solutions\n");
    }
    
    if (best_ga->survivors_rescued > astar.survivors_rescued) {
        printf("  • GA saved %d more survivors\n", best_ga->survivors_rescued - astar.survivors_rescued);
    }
    
    if (best_ga->coverage > astar.coverage) {
        printf("  • GA explored %.1f%% more area\n", best_ga->coverage - astar.coverage);
    }
    
    printf("═══════════════════════════════════════════════════════\n");
}

// Global variables for clean shutdown
static int shm_id, msg_id, sem_id;
static pid_t* worker_pids = NULL;
static SharedData* shared_data = NULL;

// Simple semaphore operations
void sem_lock(int sem_id) {
    struct sembuf op = {0, -1, 0};
    semop(sem_id, &op, 1);
}

void sem_unlock(int sem_id) {
    struct sembuf op = {0, 1, 0};
    semop(sem_id, &op, 1);
}

// Cleanup resources
void cleanup_resources() {
    printf("\n\n=== Cleaning Up Resources ===\n");
    
    if (worker_pids && shared_data) {
        printf("Terminating worker processes...\n");
        free(worker_pids);
    }
    
    if (shared_data) {
        printf("Detaching shared memory...\n");
        shmdt(shared_data);
    }
    
    if (shm_id > 0) {
        printf("Removing shared memory...\n");
        shmctl(shm_id, IPC_RMID, NULL);
    }
    
    if (msg_id > 0) {
        printf("Removing message queue...\n");
        msgctl(msg_id, IPC_RMID, NULL);
    }
    
    if (sem_id > 0) {
        printf("Removing semaphore...\n");
        semctl(sem_id, 0, IPC_RMID);
    }
    
    printf("Cleanup complete.\n");
}

// Signal handler
void handle_signal(int sig) {
    printf("\nSignal %d received. Cleaning up...\n", sig);
    cleanup_resources();
    exit(0);
}

// Sort population by fitness (descending)
void sort_population(Population* pop) {
    for (int i = 0; i < pop->size - 1; i++) {
        for (int j = i + 1; j < pop->size; j++) {
            if (pop->chromosomes[i].fitness < pop->chromosomes[j].fitness) {
                Chromosome temp = pop->chromosomes[i];
                pop->chromosomes[i] = pop->chromosomes[j];
                pop->chromosomes[j] = temp;
            }
        }
    }
}

// Print population statistics
void print_population_stats(Population* pop, int generation) {
    if (pop->size == 0) return;
    
    double best = pop->chromosomes[0].fitness;
    double worst = pop->chromosomes[pop->size-1].fitness;
    double sum = 0.0;
    
    for (int i = 0; i < pop->size; i++) {
        sum += pop->chromosomes[i].fitness;
    }
    double average = sum / pop->size;
    
    printf("Gen %4d: Best=%.2f, Avg=%.2f, Worst=%.2f\n", 
           generation, best, average, worst);
}

int main(int argc, char* argv[]) {
    // Setup signal handling
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    atexit(cleanup_resources);
    
    printf("=== RESCUE ROBOT PATH OPTIMIZATION ===\n");
    printf("=== Genetic Algorithm with IPC ===\n\n");
    
    // Read configuration
    Config cfg;
    if (argc > 1) {
        cfg = read_config(argv[1]);
    } else {
        cfg = read_config("config.txt");
    }
    print_config(cfg);
    
    // Validate configuration
    if (cfg.num_workers <= 0) cfg.num_workers = 1;
    if (cfg.num_workers > cfg.population_size) {
        cfg.num_workers = cfg.population_size;
    }
    
    // Create grid environment
    printf("\n=== Creating 3D Rescue Environment ===\n");
    Grid3D* grid = create_grid(cfg);
    initialize_grid(grid);
    printf("Grid created: %dx%dx%d\n", grid->size_x, grid->size_y, grid->size_z);
    printf("Survivors: %d, Obstacles: %d\n", grid->total_survivors, grid->total_obstacles);
    
    // Create IPC resources
    printf("\n=== Initializing IPC System ===\n");
    shm_id = shmget(IPC_PRIVATE, sizeof(SharedData), IPC_CREAT | 0666);
    msg_id = msgget(IPC_PRIVATE, IPC_CREAT | 0666);
    sem_id = semget(IPC_PRIVATE, 1, IPC_CREAT | 0666);
    
    if (shm_id < 0 || msg_id < 0 || sem_id < 0) {
        perror("IPC creation failed");
        return 1;
    }
    
    // Initialize semaphore
    union semun {
        int val;
        struct semid_ds *buf;
        unsigned short *array;
    } arg;
    arg.val = 1;
    semctl(sem_id, 0, SETVAL, arg);
    
    // Setup shared data
    shared_data = (SharedData*)shmat(shm_id, NULL, 0);
    if (!shared_data) {
        perror("Shared memory attach failed");
        return 1;
    }
    
    shared_data->grid = grid;
    shared_data->cfg = cfg;
    
    // Create initial population
    printf("\n=== Creating Initial Population ===\n");
    shared_data->pop = create_initial_population(grid, cfg.population_size, cfg.max_actions);
    printf("Population size: %d chromosomes\n", cfg.population_size);
    printf("Chromosome length: up to %d actions\n", cfg.max_actions);
    
    // Evaluate initial population
    evaluate_population(&shared_data->pop, grid, cfg.w1, cfg.w2, cfg.w3, cfg.w4);
    sort_population(&shared_data->pop);
    print_population_stats(&shared_data->pop, 0);
    
    // Genetic Algorithm Main Loop
    printf("\n=== Starting Genetic Algorithm ===\n");
    printf("Evolution for %d generations...\n", cfg.max_generations);
    
    struct timespec start_time, end_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    
    int stagnation_count = 0;
    double previous_best = shared_data->pop.chromosomes[0].fitness;
    
    for (int gen = 1; gen <= cfg.max_generations; gen++) {
        // Evolve population
        evolve_population(&shared_data->pop, grid, cfg.mutation_rate,
                         cfg.tournament_size, cfg.elitism_percent,
                         cfg.w1, cfg.w2, cfg.w3, cfg.w4);
        
        // Sort and print stats
        sort_population(&shared_data->pop);
        
        // Check for stagnation
        double current_best = shared_data->pop.chromosomes[0].fitness;
        double improvement = current_best - previous_best;
        
        if (improvement < 0.1) {
            stagnation_count++;
        } else {
            stagnation_count = 0;
        }
        
        previous_best = current_best;
        
        // Print progress
        if (gen % 10 == 0 || gen == 1 || gen == cfg.max_generations) {
            print_population_stats(&shared_data->pop, gen);
        }
        
        // Early stopping if converged
        if (stagnation_count > 50 && gen > 100) {
            printf("Early stopping at generation %d (converged)\n", gen);
            break;
        }
    }
    
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    double ga_time = (end_time.tv_sec - start_time.tv_sec) + 
                    (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
    
    printf("\n=== GA Evolution Complete ===\n");
    printf("Total generations: %d\n", cfg.max_generations);
    printf("Total evolution time: %.2f seconds\n", ga_time);
    printf("Average time per generation: %.3f seconds\n", ga_time / cfg.max_generations);
    
    // Final population stats
    printf("\n=== Final Population Statistics ===\n");
    printf("Best chromosome fitness: %.2f\n", shared_data->pop.chromosomes[0].fitness);
    printf("Survivors rescued: %d\n", shared_data->pop.chromosomes[0].survivors_rescued);
    printf("Coverage: %.1f%%\n", shared_data->pop.chromosomes[0].coverage);
    printf("Risk accumulated: %.1f\n", shared_data->pop.chromosomes[0].total_risk);
    
    // Print top 5 chromosomes
    printf("\nTop 5 Chromosomes:\n");
    for (int i = 0; i < 5 && i < shared_data->pop.size; i++) {
        printf("  %d. Fitness: %6.2f | Survivors: %3d | Coverage: %5.1f%% | Length: %3d\n",
               i+1,
               shared_data->pop.chromosomes[i].fitness,
               shared_data->pop.chromosomes[i].survivors_rescued,
               shared_data->pop.chromosomes[i].coverage,
               shared_data->pop.chromosomes[i].path_length);
    }
    
    // ============================================
    // A* COMPARISON SECTION
    // ============================================
    
    compare_ga_with_astar(&shared_data->pop, grid);
    
    // ============================================
    // ROBOT DEPLOYMENT SECTION
    // ============================================
    
    // Deploy robots
    printf("\n=== Deploying Rescue Robots ===\n");
    Robot* robots = malloc(cfg.num_robots * sizeof(Robot));
    if (!robots) {
        perror("Failed to allocate robots");
        return 1;
    }
    
    initialize_robots(robots, cfg.num_robots, grid);
    assign_missions_to_robots(robots, cfg.num_robots, &shared_data->pop, grid);
    
    // **FIXED: Reset grid to original state before generating robot paths**
    printf("\n=== Generating Robot Paths ===\n");
    printf("Resetting grid to original state...\n");
    reset_grid_complete(grid);
    
    for (int i = 0; i < cfg.num_robots; i++) {
        if (robots[i].active) {
            printf("\nGenerating path for Robot %d... ", robots[i].id);
            
            // Reset visited flags for each robot
            reset_visited(grid);
            
            if (robots[i].assigned_mission.actions && 
                robots[i].assigned_mission.num_actions > 0) {
                
                // Execute the pre-designed mission path
                execute_actions(&robots[i].assigned_mission, grid);
                
                // Evaluate with GA weights
                evaluate_chromosome(&robots[i].assigned_mission, grid, 
                                  cfg.w1, cfg.w2, cfg.w3, cfg.w4);
                
                printf("Path length: %d, Fitness: %.2f, Survivors: %d\n", 
                       robots[i].assigned_mission.path_length,
                       robots[i].assigned_mission.fitness,
                       robots[i].assigned_mission.survivors_rescued);
            } else {
                printf("ERROR: No actions assigned to robot!\n");
                robots[i].active = false;
            }
        }
    }
    
    // **NEW: Validate and correct paths before simulation**
    printf("\n=== Validating Robot Paths ===\n");
    validate_and_adjust_paths(robots, cfg.num_robots, grid);
    
    // **FIXED: Reset grid again before simulation**
    printf("\n=== Resetting grid for simulation ===\n");
    reset_grid_complete(grid);
    
    simulate_robots(robots, cfg.num_robots, grid);
    print_robot_status(robots, cfg.num_robots);
    
    // Save paths for visualization
    save_robot_paths(robots, cfg.num_robots, "paths_3d.dat");
    printf("\n=== Starting OpenGL Visualization ===\n");
    printf("Opening 3D visualization window...\n");
    printf("Close the window to continue to Python visualization.\n\n");

    visualize_robots_opengl(robots, cfg.num_robots, grid);

    // Note: visualize_robots_opengl will run glutMainLoop() which blocks
    // until the window is closed. After that, continue with Python visualization.

    printf("\n=== OpenGL Visualization Closed ===\n");
    printf("Continuing with Python visualization...\n");
    // Free resources
    free(robots);
    
    printf("\n=== Mission Complete ===\n");
    printf("Optimized paths saved to: paths_3d.dat\n");
    printf("Use 'python3 src/visualization.py' for 3D visualization\n");
    printf("===========================================\n\n");
    
    return 0;
}