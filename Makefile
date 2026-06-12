# Makefile for Rescue Robot Path Optimizer
CC = gcc
CFLAGS = -g -Wall -O2 -pthread -Isrc
LIBS = -lm -lrt -lpthread -lGL -lGLU -lglut

# Source files (without astar_simple.c and comparison.c since they have issues)
SRC = src/main.c src/config.c src/grid.c src/genetic_algorithm.c \
      src/ipc.c src/robot.c src/opengl_visualization.c

# Object files
OBJ = $(SRC:.c=.o)

# Executable
TARGET = project2

# Default target
all: $(TARGET)

# Link
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(LIBS)

# Compile
src/%.o: src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Run
run: $(TARGET)
	./$(TARGET) config.txt

# Clean
clean:
	rm -f $(TARGET) $(OBJ) paths_3d.dat generation_stats.txt paths_astar.dat

# Install dependencies
install:
	sudo apt update
	sudo apt install -y freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev python3-matplotlib

# Help
help:
	@echo "Available commands:"
	@echo "  make all     - Build project"
	@echo "  make run     - Run project"
	@echo "  make clean   - Clean build files"
	@echo "  make install - Install dependencies (WSL/Ubuntu)"
	@echo "  make help    - Show this help"

.PHONY: all run clean install help
