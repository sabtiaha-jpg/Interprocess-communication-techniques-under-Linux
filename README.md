🚀 Rescue Robot Path Optimizer
📌 Quick Start
bash

make clean && make
./project2 config.txt

📂 Project Structure
text
├── src/           # C source files
├── config.txt     # Settings
├── Makefile      # Build script
└── visualization.py  # 3D viewer

🛠️ Build & Run
bash
# Compile
make

# Run
./project2 config.txt

# Visualize paths
python3  src/visualization.py

# Clean
make clean
⚙️ Config (config.txt)
ini
grid_x=20          # Grid width
grid_y=20          # Grid height  
grid_z=5           # Grid depth
num_robots=3       # Number of robots
population_size=50 # GA population
max_generations=200 # GA iterations

🎯 What It Does
Creates 3D disaster environment

Uses genetic algorithm to find optimal paths

Deploys multiple rescue robots

Avoids collisions and obstacles

Saves paths for 3D visualization

📊 Output
paths_3d.dat - Robot paths

Console statistics

3D visualization with Python

🔧 Dependencies
GCC compiler

Python 3 + Matplotlib

Linux/Unix system

