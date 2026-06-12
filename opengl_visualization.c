#ifndef CONFIG_H
#define CONFIG_H

#define DEFAULT_GRID_X 20
#define DEFAULT_GRID_Y 20
#define DEFAULT_GRID_Z 5
#define DEFAULT_POP_SIZE 50
#define DEFAULT_MAX_GEN 200
#define DEFAULT_NUM_ROBOTS 3
#define DEFAULT_NUM_WORKERS 4
#define DEFAULT_TOURNAMENT_SIZE 5
#define DEFAULT_MUTATION_RATE 0.05
#define DEFAULT_ELITISM_PERCENT 10
#define DEFAULT_W1 1.0
#define DEFAULT_W2 0.8
#define DEFAULT_W3 0.5
#define DEFAULT_W4 1.2
#define DEFAULT_MAX_ACTIONS 100

typedef struct {
    int grid_x, grid_y, grid_z;
    int population_size;
    int max_generations;
    int num_robots;
    int num_workers;
    int tournament_size;
    double mutation_rate;
    int elitism_percent;
    double w1, w2, w3, w4;
    int max_actions;
} Config;

Config read_config(const char* filename);
void print_config(Config cfg);

#endif
