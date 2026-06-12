#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Config read_config(const char* filename) {
    Config cfg;
    
    // Default values
    cfg.grid_x = DEFAULT_GRID_X;
    cfg.grid_y = DEFAULT_GRID_Y;
    cfg.grid_z = DEFAULT_GRID_Z;
    cfg.population_size = DEFAULT_POP_SIZE;
    cfg.max_generations = DEFAULT_MAX_GEN;
    cfg.num_robots = DEFAULT_NUM_ROBOTS;
    cfg.num_workers = DEFAULT_NUM_WORKERS;
    cfg.tournament_size = DEFAULT_TOURNAMENT_SIZE;
    cfg.mutation_rate = DEFAULT_MUTATION_RATE;
    cfg.elitism_percent = DEFAULT_ELITISM_PERCENT;
    cfg.w1 = DEFAULT_W1;
    cfg.w2 = DEFAULT_W2;
    cfg.w3 = DEFAULT_W3;
    cfg.w4 = DEFAULT_W4;
    cfg.max_actions = DEFAULT_MAX_ACTIONS;
    
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("Config file '%s' not found. Using default values.\n", filename);
        return cfg;
    }
    
    char line[256];
    while (fgets(line, sizeof(line), file)) {
        line[strcspn(line, "\n")] = 0;
        
        if (line[0] == '#' || line[0] == '\0') continue;
        
        char key[100], value[100];
        if (sscanf(line, "%99[^=]=%99s", key, value) == 2) {
            if (strcmp(key, "grid_x") == 0) cfg.grid_x = atoi(value);
            else if (strcmp(key, "grid_y") == 0) cfg.grid_y = atoi(value);
            else if (strcmp(key, "grid_z") == 0) cfg.grid_z = atoi(value);
            else if (strcmp(key, "population_size") == 0) cfg.population_size = atoi(value);
            else if (strcmp(key, "max_generations") == 0) cfg.max_generations = atoi(value);
            else if (strcmp(key, "num_robots") == 0) cfg.num_robots = atoi(value);
            else if (strcmp(key, "num_workers") == 0) cfg.num_workers = atoi(value);
            else if (strcmp(key, "tournament_size") == 0) cfg.tournament_size = atoi(value);
            else if (strcmp(key, "mutation_rate") == 0) cfg.mutation_rate = atof(value);
            else if (strcmp(key, "elitism_percent") == 0) cfg.elitism_percent = atoi(value);
            else if (strcmp(key, "w1") == 0) cfg.w1 = atof(value);
            else if (strcmp(key, "w2") == 0) cfg.w2 = atof(value);
            else if (strcmp(key, "w3") == 0) cfg.w3 = atof(value);
            else if (strcmp(key, "w4") == 0) cfg.w4 = atof(value);
            else if (strcmp(key, "max_actions") == 0) cfg.max_actions = atoi(value);
        }
    }
    
    fclose(file);
    return cfg;
}

void print_config(Config cfg) {
    printf("\n=== CONFIGURATION ===\n");
    printf("Grid Size: %dx%dx%d\n", cfg.grid_x, cfg.grid_y, cfg.grid_z);
    printf("Population Size: %d\n", cfg.population_size);
    printf("Max Generations: %d\n", cfg.max_generations);
    printf("Number of Robots: %d\n", cfg.num_robots);
    printf("Number of Workers: %d\n", cfg.num_workers);
    printf("Tournament Size: %d\n", cfg.tournament_size);
    printf("Mutation Rate: %.3f\n", cfg.mutation_rate);
    printf("Elitism: %d%%\n", cfg.elitism_percent);
    printf("Weights: w1=%.2f, w2=%.2f, w3=%.2f, w4=%.2f\n", 
           cfg.w1, cfg.w2, cfg.w3, cfg.w4);
    printf("Max Actions per Chromosome: %d\n", cfg.max_actions);
}
