# Interprocess-communication-techniques-under-Linux
# Rescue Robot Path Optimizer
**ENCS4330 – Real-Time Applications & Embedded Systems | Project #2
Birzeit University – Electrical & Computer Engineering Department

A multi-process Linux application that uses **Genetic Algorithms** and **IPC techniques** to optimize rescue robot paths inside a collapsed 3D building environment.
---
## Overview

The application models a collapsed building as a **3D grid** where:
- Obstacles represent debris
- Accessible cells represent potential survivor locations (detected via simulated heat/CO2 sensors)
- Multiple rescue robots are deployed to maximize survivor coverage while minimizing travel time, path length, and collision risk

---

## Architecture

The system uses a **persistent worker pool** (master + N worker processes) to parallelize fitness evaluation across the population:

```
Master Process
    ├── Creates shared memory, message queue, and semaphores
    ├── Spawns N worker processes (kept alive across generations)
    ├── Sends chromosomes via message queue → workers evaluate fitness
    ├── Workers return results via message queue → master collects
    └── Evolves population: selection → crossover → mutation → elitism
```

Workers are created **once** at startup and reused for all generations — avoiding the overhead of forking per generation.

---

## IPC Design

| Mechanism | Purpose |
|---|---|
| **Shared Memory** (`shmget/shmat`) | Shares grid, config, population state, and worker status between master and workers |
| **Message Queues** (`msgget/msgsnd/msgrcv`) | Master sends work tasks (`MSG_TYPE_WORK`) to workers; workers return fitness results (`MSG_TYPE_RESULT`); termination via `MSG_TYPE_TERMINATE` |
| **Semaphores** (`semget/semop`) | Synchronizes access to shared data across worker processes |
| **POSIX Mutex** (`pthread_mutex_t`) | Guards critical sections inside shared memory |

---

## Genetic Algorithm

### Chromosome Representation
Each chromosome encodes a robot's path as a **sequence of actions**:
`UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD, WAIT, SCAN, RETURN`

The actual 3D coordinates are derived by executing these actions on the grid.

### Fitness Function
```
f = w1 * survivors + w2 * coverage - w3 * path_length - w4 * risk
```
Weights `w1`–`w4` are user-configurable in `config.txt`.

### Genetic Operators
| Operator | Description |
|---|---|
| **Selection** | Tournament selection (configurable tournament size) |
| **Crossover** | Single-point crossover — exchanges action sequences between two parents |
| **Mutation** | Randomly replaces actions to explore alternative routes |
| **Elitism** | Top N% of chromosomes are preserved unchanged into the next generation |

### Termination
Evolution stops when the maximum number of generations is reached or when fitness stagnates.

---

## Project Structure

```
project2/
├── src/
│   ├── main.c                    # Entry point, master process logic
│   ├── grid.c / grid.h           # 3D grid map and cell definitions
│   ├── genetic_algorithm.c / .h  # GA operators, chromosome, population
│   ├── ipc.c / ipc.h             # Shared memory, message queues, semaphores, worker pool
│   ├── robot.c / robot.h         # Robot state and movement logic
│   ├── config.c / config.h       # Config file parsing with defaults
│   ├── opengl_visualization.c/.h # Real-time OpenGL 3D visualization
│   ├── visualization.py          # Python/Matplotlib 3D path viewer
│   └── visualizaion_comparison.py# Path comparison visualization
├── config.txt                    # Runtime parameters (see below)
├── Makefile                      # Build system
├── paths_3d.dat                  # Output: robot paths (generated at runtime)
└── README.md
```

---

## Dependencies

Install on Ubuntu/WSL with:
```bash
make install
```
Or manually:
```bash
sudo apt update
sudo apt install -y gcc freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev python3-matplotlib
```

**Required:** GCC, POSIX threads (`pthread`), OpenGL/GLUT, libm, librt, Python 3 + Matplotlib

---

## Build & Run

```bash
# Install dependencies (first time only)
make install

# Compile
make

# Run with config file
./project2 config.txt

# Run with make shortcut
make run

# Visualize output paths (after running)
python3 src/visualization.py

# Clean build artifacts
make clean
```

If `config.txt` is not provided, the application falls back to hardcoded defaults.

---

## Configuration

All parameters are set in `config.txt` (passed as a command-line argument):

```ini
grid_x=20           # Grid width
grid_y=20           # Grid height
grid_z=5            # Grid depth (floors)
num_robots=3        # Number of rescue robots
population_size=50  # GA population size
max_generations=200 # Maximum evolution iterations
num_workers=5       # Number of worker processes in the pool
tournament_size=5   # Tournament selection size
mutation_rate=0.1   # Mutation probability (0.0 – 1.0)
elitism_percent=20  # Top % of population preserved each generation
max_actions=100     # Maximum actions per chromosome
w1=1.0              # Fitness weight: survivors rescued
w2=1.07             # Fitness weight: coverage area
w3=0.8              # Fitness weight: path length (penalty)
w4=2.5              # Fitness weight: risk exposure (penalty)
```

---

## Output

| File | Description |
|---|---|
| `paths_3d.dat` | Final optimized robot paths (3D coordinates per robot) |
| Console | Per-generation stats: best fitness, survivors reached, coverage %, risk |

---

## Visualization

**Python (Matplotlib):**
```bash
python3 src/visualization.py
```
Reads `paths_3d.dat` and renders all robot paths in 3D with survivor locations marked.

**OpenGL (real-time):**
Built into the application — launches automatically when the GA finishes and renders the 3D grid with animated robot paths.
