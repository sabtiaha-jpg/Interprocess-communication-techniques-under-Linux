import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def read_robot_paths(filename):
    robots = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            if ':' in line:
                # Get robot ID and path
                robot_id, path_str = line.split(':', 1)
                robot_id = robot_id.strip()
                path_str = path_str.strip()
                
                # Remove Fitness info if exists
                if '#' in path_str:
                    path_str = path_str.split('#')[0].strip()
                
                # Split path into coordinates
                points = path_str.split('->')
                robot_path = []
                
                for point in points:
                    point = point.strip()
                    if point:
                        try:
                            x, y, z = map(int, point.split(','))
                            robot_path.append((x, y, z))
                        except ValueError:
                            continue
                
                if robot_path:
                    robots.append({
                        'id': robot_id,
                        'path': robot_path
                    })
    
    print(f"Number of robots loaded: {len(robots)}")
    for robot in robots:
        print(f"  {robot['id']}: {len(robot['path'])} waypoints")
    
    return robots

def visualize_3d_paths(robots):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Colors for different robots
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'orange', 'purple', 'brown', 'pink']
    
    for i, robot in enumerate(robots):
        robot_path = robot['path']
        color = colors[i % len(colors)]
        
        # Convert path to arrays for plotting
        x_vals = [p[0] for p in robot_path]
        y_vals = [p[1] for p in robot_path]
        z_vals = [p[2] for p in robot_path]
        
        # Plot the path
        ax.plot(x_vals, y_vals, z_vals, color=color, linewidth=2, 
                marker='o', markersize=4, label=robot['id'])
        
        # Mark start and end points
        ax.scatter(x_vals[0], y_vals[0], z_vals[0], 
                  color='green', s=100, marker='^', label='Start' if i == 0 else "")
        ax.scatter(x_vals[-1], y_vals[-1], z_vals[-1], 
                  color='red', s=100, marker='v', label='End' if i == 0 else "")
    
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    ax.set_title('3D Robot Rescue Paths Visualization')
    ax.legend()
    ax.grid(True)
    
    plt.savefig("visualization.png")


def main():
    print("Loading robot paths...")
    robots = read_robot_paths("paths_3d.dat")
    
    if not robots:
        print("No robots loaded. Check the data file.")
        return
    
    print(f"Loaded {len(robots)} robot paths")
    print("Creating visualization...")
    visualize_3d_paths(robots)
    print("Visualization complete.")

if __name__ == "__main__":
    main()