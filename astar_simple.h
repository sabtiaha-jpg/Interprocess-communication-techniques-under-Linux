import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def read_ga_paths(filename):
    """
    Reads GA robot paths from file in format:
    Robot1: x,y,z -> x,y,z -> x,y,z # Fitness: ...
    """
    robots = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            
            # Skip empty lines and comments
            if not line or line.startswith('#'):
                continue
            
            # Check for Robot: format
            if ':' in line:
                try:
                    # Split robot ID and path
                    robot_part, path_part = line.split(':', 1)
                    
                    # Remove fitness info if exists
                    if '#' in path_part:
                        path_part = path_part.split('#')[0].strip()
                    
                    # Split by ->
                    points = [p.strip() for p in path_part.split('->')]
                    
                    robot_path = []
                    for point in points:
                        if not point:
                            continue
                        # Handle x,y,z format
                        if ',' in point:
                            x, y, z = map(int, point.split(','))
                        else:
                            # Try space-separated
                            coords = point.split()
                            if len(coords) >= 3:
                                x, y, z = map(int, coords[:3])
                            else:
                                continue
                        robot_path.append((x, y, z))
                    
                    if robot_path:
                        robots.append(robot_path)
                        
                except ValueError as e:
                    print(f"Warning: Could not parse line: {line}")
                    continue
    
    print(f"Loaded {len(robots)} GA robot paths")
    total_waypoints = sum(len(p) for p in robots)
    print(f"Total waypoints: {total_waypoints}")
    
    return robots

def read_astar_path(filename):
    """
    Reads A* path from file (creates demo path if file not found)
    """
    try:
        with open(filename, 'r') as f:
            lines = [line.strip() for line in f if line.strip() and not line.startswith('#')]
        
        if not lines:
            return create_demo_astar_path()
        
        # Try different formats
        path = []
        for line in lines:
            try:
                if ',' in line:
                    x, y, z = map(int, line.split(','))
                else:
                    parts = line.split()
                    if len(parts) >= 3:
                        x, y, z = map(int, parts[:3])
                    else:
                        continue
                path.append((x, y, z))
            except:
                continue
        
        if path:
            return path
        else:
            return create_demo_astar_path()
            
    except FileNotFoundError:
        print(f"Note: {filename} not found, creating demo A* path")
        return create_demo_astar_path()

def create_demo_astar_path():
    """Creates a sample A* path for demonstration"""
    # Create a straight line path
    path = []
    for i in range(20):
        x = i * 2
        y = i * 2
        z = i // 5
        path.append((x, y, z))
    
    # Add some variation
    for i in range(10):
        x = 40 - i * 2
        y = 40 - i * 2
        z = 4 - i // 3
        path.append((x, y, z))
    
    print("Created demo A* path with", len(path), "waypoints")
    return path

def plot_path_3d(ax, path, label=None, color=None, linewidth=2, markersize=4):
    """
    Plots a single path in 3D
    """
    if not path:
        return
    
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    zs = [p[2] for p in path]
    
    # Plot the path line
    ax.plot(xs, ys, zs, color=color, linewidth=linewidth, 
            marker='o', markersize=markersize, label=label, alpha=0.8)
    
    # Mark start and end points
    if xs and ys and zs:
        ax.scatter(xs[0], ys[0], zs[0], 
                  color='green', s=150, marker='^', 
                  edgecolors='black', linewidth=2, zorder=5)
        ax.scatter(xs[-1], ys[-1], zs[-1], 
                  color='red', s=150, marker='v', 
                  edgecolors='black', linewidth=2, zorder=5)

def create_comparison_visualization(ga_paths, astar_path, output_file="comparison.png"):
    """
    Creates comparison visualization like in the image
    """
    fig = plt.figure(figsize=(16, 8))
    
    # ==================== LEFT: GA Multi-Robot Paths ====================
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title("Genetic Algorithm - Multi-Robot Paths", 
                  fontsize=14, fontweight='bold', pad=15)
    
    # Colors for different robots
    colors = ['blue', 'cyan', 'magenta', 'orange', 'green', 'purple',
              'brown', 'pink', 'gray', 'olive']
    
    # Plot each robot path
    for i, path in enumerate(ga_paths):
        color = colors[i % len(colors)]
        plot_path_3d(ax1, path, label=f'Robot {i+1}', color=color, 
                    linewidth=2.5, markersize=3)
    
    # Set labels and grid
    ax1.set_xlabel('X Coordinate', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Y Coordinate', fontsize=11, fontweight='bold')
    ax1.set_zlabel('Z Coordinate', fontsize=11, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    # Add legend
    if ga_paths:
        ax1.legend(loc='upper right', fontsize=9, ncol=2)
    
    # Add stats box
    if ga_paths:
        total_robots = len(ga_paths)
        total_waypoints = sum(len(p) for p in ga_paths)
        stats_text = f"Robots: {total_robots}\nWaypoints: {total_waypoints}"
        ax1.text2D(0.02, 0.98, stats_text, transform=ax1.transAxes,
                  fontsize=10, verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    # ==================== RIGHT: A* Single Path ====================
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set_title("A* Algorithm - Optimal Single Path", 
                  fontsize=14, fontweight='bold', pad=15)
    
    # Plot A* path
    if astar_path:
        plot_path_3d(ax2, astar_path, label='A* Optimal Path', 
                    color='red', linewidth=3, markersize=5)
        
        # Add stats for A*
        astar_stats = f"Path Length: {len(astar_path)}"
        ax2.text2D(0.02, 0.98, astar_stats, transform=ax2.transAxes,
                  fontsize=10, verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))
        
        ax2.legend(loc='upper right', fontsize=10)
    
    else:
        ax2.text2D(0.3, 0.5, "No A* Data Available", 
                  transform=ax2.transAxes, fontsize=12, 
                  bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Set consistent labels for right plot
    ax2.set_xlabel('X Coordinate', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Y Coordinate', fontsize=11, fontweight='bold')
    ax2.set_zlabel('Z Coordinate', fontsize=11, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # Adjust view angles for better visualization
    ax1.view_init(elev=20, azim=45)
    ax2.view_init(elev=20, azim=45)
    
    # Set equal aspect ratio for both plots
    all_x = []
    all_y = []
    all_z = []
    
    for path in ga_paths:
        all_x.extend([p[0] for p in path])
        all_y.extend([p[1] for p in path])
        all_z.extend([p[2] for p in path])
    
    if astar_path:
        all_x.extend([p[0] for p in astar_path])
        all_y.extend([p[1] for p in astar_path])
        all_z.extend([p[2] for p in astar_path])
    
    if all_x and all_y and all_z:
        max_range = max(max(all_x)-min(all_x), 
                       max(all_y)-min(all_y), 
                       max(all_z)-min(all_z))
        mid_x = (max(all_x) + min(all_x)) * 0.5
        mid_y = (max(all_y) + min(all_y)) * 0.5
        mid_z = (max(all_z) + min(all_z)) * 0.5
        
        if max_range > 0:
            for ax in [ax1, ax2]:
                ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
                ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
                ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
    
    # Add main title
    fig.suptitle('Path Planning Comparison: Genetic Algorithm vs A* Algorithm', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=200, bbox_inches='tight')
    print(f"\n✓ Comparison saved as: {output_file}")
    
    # Show the plot
    plt.show()

def generate_astar_file_from_ga(ga_paths, output_file="paths_astar.dat"):
    """
    Generates a sample A* file from GA paths for demonstration
    """
    if not ga_paths:
        return
    
    # Take the first GA path as "A* optimal" for demo
    demo_path = ga_paths[0] if ga_paths else []
    
    with open(output_file, 'w') as f:
        f.write("# A* Optimal Path (Demo)\n")
        f.write("# Generated for visualization comparison\n\n")
        
        for point in demo_path:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")
    
    print(f"Generated demo A* file: {output_file}")
    return demo_path

def main():
    print("=" * 60)
    print("PATH PLANNING COMPARISON VISUALIZATION")
    print("=" * 60)
    
    # File names
    ga_file = "paths_3d.dat"
    astar_file = "paths_astar.dat"
    output_file = "comparison_final.png"
    
    # Step 1: Read GA paths
    print(f"\n[1] Reading GA paths from: {ga_file}")
    ga_paths = read_ga_paths(ga_file)
    
    if not ga_paths:
        print("ERROR: No GA paths found!")
        print("Make sure to run the C program first to generate paths_3d.dat")
        return
    
    # Step 2: Read or generate A* path
    print(f"\n[2] Reading A* path from: {astar_file}")
    astar_path = read_astar_path(astar_file)
    
    # If no A* file, create one from GA data for demo
    if not astar_path or len(astar_path) < 2:
        print("Generating demo A* path from GA data...")
        astar_path = generate_astar_file_from_ga(ga_paths, astar_file)
    
    # Step 3: Create visualization
    print(f"\n[3] Creating comparison visualization...")
    create_comparison_visualization(ga_paths, astar_path, output_file)
    
    print(f"\n" + "=" * 60)
    print("VISUALIZATION COMPLETE!")
    print("=" * 60)
    print(f"✓ GA Robot Paths: {len(ga_paths)} robots")
    print(f"✓ A* Optimal Path: {len(astar_path) if astar_path else 0} waypoints")
    print(f"✓ Output File: {output_file}")
    print("\nTo view: open", output_file)

if __name__ == "_main_":
    main()
