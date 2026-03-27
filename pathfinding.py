import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from heapq import heappush, heappop
import os

class PathPlanner:
    def __init__(self, obstacles_file, safe_distance=1.0, grid_resolution=0.5, 
                 boundary_x=13.716, boundary_y=13.716):
        """
        Initialize the path planner
        
        Args:
            obstacles_file: Path to file containing obstacle coordinates
            safe_distance: Safe distance to maintain from obstacles (meters)
            grid_resolution: Resolution of the grid for A* (meters)
            boundary_x: X boundary distance from origin (positive or negative, meters)
            boundary_y: Y boundary distance from origin (positive or negative, meters)
        """
        self.obstacles = self.load_obstacles(obstacles_file)
        self.safe_distance = safe_distance
        self.original_safe_distance = safe_distance
        self.grid_resolution = grid_resolution
        self.boundary_x = boundary_x
        self.boundary_y = boundary_y
        self.original_boundary_x = boundary_x
        self.original_boundary_y = boundary_y
        
        # Calculate boundary limits based on signs
        # Robot is at (0,0) which is always at a corner
        if boundary_x > 0:
            self.x_min = 0
            self.x_max = boundary_x
        else:
            self.x_min = boundary_x
            self.x_max = 0
            
        if boundary_y > 0:
            self.y_min = 0
            self.y_max = boundary_y
        else:
            self.y_min = boundary_y
            self.y_max = 0
        
        self.start = (0, 0)
        self.goal = None
        self.path = None
        self.smoothed_path = None
        self.waypoints = None
        
    def update_boundaries(self):
        """Update boundary limits based on current boundary_x and boundary_y"""
        if self.boundary_x > 0:
            self.x_min = 0
            self.x_max = self.boundary_x
        else:
            self.x_min = self.boundary_x
            self.x_max = 0
            
        if self.boundary_y > 0:
            self.y_min = 0
            self.y_max = self.boundary_y
        else:
            self.y_min = self.boundary_y
            self.y_max = 0
        
    def load_obstacles(self, filename):
        """Load obstacles from text file"""
        obstacles = []
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        coords = line.split(',')
                        if len(coords) == 2:
                            x = float(coords[0].strip())
                            y = float(coords[1].strip())
                            obstacles.append((x, y))
            print(f"Loaded {len(obstacles)} obstacles")
            return obstacles
        except FileNotFoundError:
            print(f"Error: File '{filename}' not found")
            return []
        except Exception as e:
            print(f"Error loading obstacles: {e}")
            return []
    
    def is_within_boundary(self, point):
        """Check if a point is within the field boundaries"""
        return (self.x_min <= point[0] <= self.x_max and 
                self.y_min <= point[1] <= self.y_max)
    
    def is_safe_from_obstacles(self, point):
        """Check if a point is safe (far enough from obstacles)"""
        for obs in self.obstacles:
            distance = np.sqrt((point[0] - obs[0])**2 + (point[1] - obs[1])**2)
            if distance < self.safe_distance:
                return False
        return True
    
    def is_safe(self, point):
        """Check if a point is both within boundary and safe from obstacles"""
        return self.is_within_boundary(point) and self.is_safe_from_obstacles(point)
    
    def is_line_safe(self, point1, point2, num_checks=20):
        """Check if the line segment between two points is safe"""
        for i in range(num_checks + 1):
            t = i / num_checks
            x = point1[0] + t * (point2[0] - point1[0])
            y = point1[1] + t * (point2[1] - point1[1])
            if not self.is_safe((x, y)):
                return False
        return True
    
    def get_neighbors(self, node):
        """Get valid neighbors for A* algorithm"""
        x, y = node
        # 8-directional movement
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (-1, -1), (1, -1), (-1, 1)
        ]
        
        neighbors = []
        for dx, dy in directions:
            new_x = x + dx * self.grid_resolution
            new_y = y + dy * self.grid_resolution
            new_node = (new_x, new_y)
            
            if self.is_safe(new_node):
                neighbors.append(new_node)
        
        return neighbors
    
    def heuristic(self, node1, node2):
        """Euclidean distance heuristic"""
        return np.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)
    
    def a_star(self):
        """A* pathfinding algorithm"""
        if not self.is_safe(self.start):
            print("Error: Start position is not safe!")
            return None
        
        if not self.is_safe(self.goal):
            print("Error: Goal position is not safe!")
            return None
        
        print("Running A* algorithm...")
        
        # Round start and goal to grid
        start_rounded = (
            round(self.start[0] / self.grid_resolution) * self.grid_resolution,
            round(self.start[1] / self.grid_resolution) * self.grid_resolution
        )
        goal_rounded = (
            round(self.goal[0] / self.grid_resolution) * self.grid_resolution,
            round(self.goal[1] / self.grid_resolution) * self.grid_resolution
        )
        
        # Priority queue: (f_score, counter, node)
        counter = 0
        open_set = [(0, counter, start_rounded)]
        counter += 1
        
        came_from = {}
        g_score = {start_rounded: 0}
        f_score = {start_rounded: self.heuristic(start_rounded, goal_rounded)}
        
        closed_set = set()
        
        while open_set:
            current_f, _, current = heappop(open_set)
            
            # Check if we reached the goal
            if self.heuristic(current, goal_rounded) < self.grid_resolution * 1.5:
                # Reconstruct path
                path = [goal_rounded]  # End at exact goal
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(self.start)  # Start at exact start
                path.reverse()
                print(f"Path found with {len(path)} nodes")
                return path
            
            closed_set.add(current)
            
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # Distance between current and neighbor
                if abs(neighbor[0] - current[0]) > 0 and abs(neighbor[1] - current[1]) > 0:
                    tentative_g = g_score[current] + self.grid_resolution * np.sqrt(2)
                else:
                    tentative_g = g_score[current] + self.grid_resolution
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_rounded)
                    heappush(open_set, (f_score[neighbor], counter, neighbor))
                    counter += 1
        
        print("No path found!")
        return None
    
    def smooth_path(self, path):
        """
        Smooth the path by removing unnecessary waypoints (corner cutting)
        Only removes points if the direct line is safe
        """
        if path is None or len(path) <= 2:
            return path
        
        print("Smoothing path (removing unnecessary corners)...")
        
        smoothed = [path[0]]  # Start with the first point
        i = 0
        
        while i < len(path) - 1:
            # Try to connect current point to furthest visible point
            j = len(path) - 1
            found_connection = False
            
            while j > i + 1:
                # Check if we can go directly from path[i] to path[j]
                if self.is_line_safe(path[i], path[j], num_checks=30):
                    smoothed.append(path[j])
                    i = j
                    found_connection = True
                    break
                j -= 1
            
            if not found_connection:
                # Can't skip any points, add the next one
                i += 1
                if i < len(path):
                    smoothed.append(path[i])
        
        # Ensure goal is the last point
        if smoothed[-1] != path[-1]:
            smoothed.append(path[-1])
        
        print(f"Path smoothed from {len(path)} to {len(smoothed)} waypoints")
        
        # Verify all segments are safe
        for i in range(len(smoothed) - 1):
            if not self.is_line_safe(smoothed[i], smoothed[i+1], num_checks=30):
                print("Warning: Smoothed path has unsafe segment, reverting to original")
                return path
        
        return smoothed
    
    def interpolate_waypoints(self, path, num_waypoints=20):
        """Interpolate waypoints along the path with linear interpolation"""
        if path is None or len(path) < 2:
            return None
        
        print(f"Interpolating {num_waypoints} waypoints...")
        
        # Calculate cumulative distance along path
        distances = [0]
        for i in range(1, len(path)):
            dist = np.sqrt(
                (path[i][0] - path[i-1][0])**2 + 
                (path[i][1] - path[i-1][1])**2
            )
            distances.append(distances[-1] + dist)
        
        total_distance = distances[-1]
        
        # Create evenly spaced waypoints
        waypoints = []
        target_distances = np.linspace(0, total_distance, num_waypoints)
        
        for target_dist in target_distances:
            # Find which segment this distance falls on
            for i in range(len(distances) - 1):
                if distances[i] <= target_dist <= distances[i + 1]:
                    # Linear interpolation between path[i] and path[i+1]
                    segment_length = distances[i + 1] - distances[i]
                    if segment_length > 0:
                        t = (target_dist - distances[i]) / segment_length
                    else:
                        t = 0
                    
                    x = path[i][0] + t * (path[i + 1][0] - path[i][0])
                    y = path[i][1] + t * (path[i + 1][1] - path[i][1])
                    
                    waypoint = (x, y)
                    
                    # Verify waypoint is safe
                    if self.is_safe(waypoint):
                        waypoints.append(waypoint)
                    else:
                        print(f"Warning: Waypoint {len(waypoints)+1} is unsafe!")
                        # Use closest safe point from path
                        waypoints.append(path[i])
                    break
        
        # Ensure we end exactly at the goal
        if waypoints and waypoints[-1] != path[-1]:
            waypoints[-1] = path[-1]
        
        # Final safety check
        for i, wp in enumerate(waypoints):
            if not self.is_safe(wp):
                print(f"ERROR: Waypoint {i+1} at {wp} is not safe!")
        
        return waypoints
    
    def save_waypoints(self, filename="waypoints.txt"):
        """Save waypoints to text file"""
        if self.waypoints is None:
            print("No waypoints to save")
            return
        
        try:
            with open(filename, 'w') as f:
                for wp in self.waypoints:
                    f.write(f"{wp[0]:.1f}, {wp[1]:.1f}\n")
            print(f"Waypoints saved to '{filename}'")
        except Exception as e:
            print(f"Error saving waypoints: {e}")
    
    def plot_map(self, show_path=True, show_smoothed=True, show_waypoints=False):
        """Plot the map with obstacles and paths"""
        plt.figure(figsize=(12, 10))
        ax = plt.gca()
        
        # Draw boundary box (starting at origin)
        field_width = abs(self.boundary_x)
        field_height = abs(self.boundary_y)
        
        # Rectangle starts at the minimum corner
        boundary_rect = Rectangle(
            (self.x_min, self.y_min), 
            field_width, 
            field_height,
            linewidth=3, 
            edgecolor='black', 
            facecolor='lightgray',
            alpha=0.3,
            zorder=0,
            label='Field Boundary'
        )
        ax.add_patch(boundary_rect)
        
        # Plot obstacles
        if self.obstacles:
            obs_array = np.array(self.obstacles)
            plt.scatter(obs_array[:, 0], obs_array[:, 1], 
                       c='red', s=200, marker='o', label='Obstacles', zorder=3)
            
            # Draw safe distance circles
            for obs in self.obstacles:
                circle = plt.Circle(obs, self.safe_distance, 
                                   color='red', fill=True, 
                                   alpha=0.2, zorder=1)
                ax.add_patch(circle)
                circle_outline = plt.Circle(obs, self.safe_distance, 
                                   color='red', fill=False, 
                                   linestyle='--', alpha=0.5, linewidth=1.5, zorder=2)
                ax.add_patch(circle_outline)
        
        # Plot start and goal
        plt.scatter(*self.start, c='green', s=300, marker='*', 
                   label='Start (0,0)', zorder=5, edgecolors='black', linewidth=2)
        if self.goal:
            plt.scatter(*self.goal, c='blue', s=300, marker='*', 
                       label='Goal', zorder=5, edgecolors='black', linewidth=2)
        
        # Plot A* path
        if show_path and self.path:
            path_array = np.array(self.path)
            plt.plot(path_array[:, 0], path_array[:, 1], 
                    'gray', linewidth=2, alpha=0.6, label='A* Path', 
                    linestyle='--', marker='o', markersize=4)
        
        # Plot smoothed path
        if show_smoothed and self.smoothed_path:
            smoothed_array = np.array(self.smoothed_path)
            plt.plot(smoothed_array[:, 0], smoothed_array[:, 1], 
                    'orange', linewidth=3, alpha=0.8, label='Smoothed Path',
                    marker='s', markersize=6)
        
        # Plot waypoints
        if show_waypoints and self.waypoints:
            wp_array = np.array(self.waypoints)
            plt.scatter(wp_array[:, 0], wp_array[:, 1], 
                       c='purple', s=100, marker='o', 
                       label='Waypoints', zorder=4, edgecolors='black')
            
            # Number the waypoints
            for i, wp in enumerate(self.waypoints):
                plt.annotate(f'{i+1}', (wp[0], wp[1]), 
                           xytext=(5, 5), textcoords='offset points',
                           fontsize=8, color='purple', fontweight='bold')
        
        plt.xlabel('X (meters)', fontsize=12)
        plt.ylabel('Y (meters)', fontsize=12)
        plt.title('Robot Path Planning with Obstacle Avoidance', fontsize=14)
        plt.legend(loc='best')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Set axis limits with some padding
        padding = 1
        plt.xlim(self.x_min - padding, self.x_max + padding)
        plt.ylim(self.y_min - padding, self.y_max + padding)
        
        plt.tight_layout()
    
    def flash_waypoints(self, num_flashes=3, interval=500):
        """Animate flashing waypoints on the map"""
        if self.waypoints is None:
            print("No waypoints to display")
            return
        
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Draw boundary box
        field_width = abs(self.boundary_x)
        field_height = abs(self.boundary_y)
        
        boundary_rect = Rectangle(
            (self.x_min, self.y_min), 
            field_width, 
            field_height,
            linewidth=3, 
            edgecolor='black', 
            facecolor='lightgray',
            alpha=0.3,
            zorder=0
        )
        ax.add_patch(boundary_rect)
        
        # Plot static elements
        if self.obstacles:
            obs_array = np.array(self.obstacles)
            ax.scatter(obs_array[:, 0], obs_array[:, 1], 
                      c='red', s=200, marker='o', label='Obstacles', zorder=3)
            
            for obs in self.obstacles:
                circle = plt.Circle(obs, self.safe_distance, 
                                   color='red', fill=True, 
                                   alpha=0.2, zorder=1)
                ax.add_patch(circle)
                circle_outline = plt.Circle(obs, self.safe_distance, 
                                   color='red', fill=False, 
                                   linestyle='--', alpha=0.5, linewidth=1.5, zorder=2)
                ax.add_patch(circle_outline)
        
        ax.scatter(*self.start, c='green', s=300, marker='*', 
                  label='Start (0,0)', zorder=5, edgecolors='black', linewidth=2)
        ax.scatter(*self.goal, c='blue', s=300, marker='*', 
                  label='Goal', zorder=5, edgecolors='black', linewidth=2)
        
        if self.smoothed_path:
            smoothed_array = np.array(self.smoothed_path)
            ax.plot(smoothed_array[:, 0], smoothed_array[:, 1], 
                   'orange', linewidth=3, alpha=0.5, label='Path')
        
        # Waypoints that will flash
        wp_array = np.array(self.waypoints)
        waypoint_scatter = ax.scatter(wp_array[:, 0], wp_array[:, 1], 
                                     c='purple', s=150, marker='o', 
                                     label='Waypoints', zorder=4, 
                                     edgecolors='black', linewidth=2)
        
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title('Robot Path Planning - Waypoints', fontsize=14)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Set axis limits
        padding = 1
        ax.set_xlim(self.x_min - padding, self.x_max + padding)
        ax.set_ylim(self.y_min - padding, self.y_max + padding)
        
        def animate(frame):
            # Flash on/off
            if frame % 2 == 0:
                waypoint_scatter.set_sizes([150] * len(self.waypoints))
                waypoint_scatter.set_alpha(1.0)
            else:
                waypoint_scatter.set_sizes([50] * len(self.waypoints))
                waypoint_scatter.set_alpha(0.3)
            return waypoint_scatter,
        
        anim = FuncAnimation(fig, animate, frames=num_flashes*2, 
                           interval=interval, blit=True, repeat=False)
        
        plt.tight_layout()
        plt.show()
    
    def find_path_with_adjustments(self):
        """
        Try to find a path with automatic adjustments to boundaries and safe distance
        Returns True if path found, False otherwise
        """
        # First, check if goal is within current boundaries
        if not self.is_within_boundary(self.goal):
            print(f"\nGoal ({self.goal[0]:.2f}, {self.goal[1]:.2f}) is outside field boundaries!")
            print(f"Field boundaries: X=[{self.x_min:.2f}, {self.x_max:.2f}], Y=[{self.y_min:.2f}, {self.y_max:.2f}]")
            
            # Adjust boundaries in the direction of the goal
            print("\nAttempting to adjust field boundaries...")
            adjustment_increment = 0.25
            max_adjustments = 50
            adjustments = 0
            
            while not self.is_within_boundary(self.goal) and adjustments < max_adjustments:
                adjustments += 1
                
                # Extend boundary in the direction needed
                if self.goal[0] > self.x_max:
                    self.boundary_x += adjustment_increment
                elif self.goal[0] < self.x_min:
                    self.boundary_x -= adjustment_increment
                
                if self.goal[1] > self.y_max:
                    self.boundary_y += adjustment_increment
                elif self.goal[1] < self.y_min:
                    self.boundary_y -= adjustment_increment
                
                self.update_boundaries()
            
            if not self.is_within_boundary(self.goal):
                print("Cannot adjust boundaries enough to reach goal.")
                return False
            
            print(f"Boundaries adjusted to X: {self.boundary_x:.2f}m, Y: {self.boundary_y:.2f}m")
        
        # Try to find path with current safe distance
        max_attempts = 20
        attempt = 0
        
        while attempt < max_attempts:
            attempt += 1
            
            print(f"\nAttempt {attempt}: Safe distance = {self.safe_distance:.2f}m")
            
            # Check if goal is safe with current settings
            if not self.is_safe_from_obstacles(self.goal):
                print(f"Goal is within safe distance of obstacles. Reducing safe distance...")
                self.safe_distance = max(0.1, self.safe_distance - 0.25)
                continue
            
            # Try A* algorithm
            self.path = self.a_star()
            
            if self.path is not None:
                print(f"✓ Path found with safe distance = {self.safe_distance:.2f}m")
                if self.safe_distance != self.original_safe_distance:
                    print(f"  (Original safe distance was {self.original_safe_distance:.2f}m)")
                return True
            else:
                # No path found, reduce safe distance
                print(f"No path found with safe distance = {self.safe_distance:.2f}m")
                if self.safe_distance > 0.1:
                    self.safe_distance = max(0.1, self.safe_distance - 0.25)
                    print(f"Reducing safe distance to {self.safe_distance:.2f}m and retrying...")
                else:
                    print("Cannot reduce safe distance further.")
                    return False
        
        print("Maximum attempts reached. Cannot find path.")
        return False
    
    def run(self, num_waypoints=20):
        """Main execution function"""
        print("\n=== Robot Path Planning System ===\n")
        
        # Display current settings
        print(f"Safe distance from obstacles: {self.safe_distance} meters")
        print(f"Grid resolution: {self.grid_resolution} meters")
        print(f"Field boundary: X = {self.boundary_x:.2f}m, Y = {self.boundary_y:.2f}m")
        print(f"Field limits: X=[{self.x_min:.2f}, {self.x_max:.2f}], Y=[{self.y_min:.2f}, {self.y_max:.2f}]")
        print(f"Robot start position: (0, 0) - at corner of field")
        print(f"Number of obstacles: {len(self.obstacles)}")
        
        # Plot initial map with obstacles
        if self.obstacles:
            self.plot_map(show_path=False, show_smoothed=False)
            plt.title('Map with Obstacles and Boundary')
            plt.show()
        
        # Get goal coordinates from user
        while True:
            try:
                goal_input = input("\nEnter goal coordinates (x,y) in meters: ")
                goal_input = goal_input.strip().replace('(', '').replace(')', '')
                x, y = map(float, goal_input.split(','))
                self.goal = (x, y)
                print(f"Goal set to: ({x}, {y})")
                break
            except Exception as e:
                print(f"Invalid input. Please enter coordinates in format: x,y or (x,y)")
        
        # Try to find path with automatic adjustments
        if not self.find_path_with_adjustments():
            print("\n✗ Failed to find a path. Exiting.")
            return
        
        # Show A* path
        self.plot_map(show_path=True, show_smoothed=False)
        plt.title(f'Path Found with A* Algorithm (Safe Distance: {self.safe_distance:.2f}m)')
        plt.show()
        
        # Smooth the path (remove unnecessary corners)
        self.smoothed_path = self.smooth_path(self.path)
        
        # Show smoothed path
        self.plot_map(show_path=True, show_smoothed=True)
        plt.title('Smoothed Path (Corners Removed)')
        plt.show()
        
        # Interpolate waypoints
        self.waypoints = self.interpolate_waypoints(self.smoothed_path, num_waypoints)
        
        if self.waypoints is None:
            print("Failed to generate waypoints. Exiting.")
            return
        
        # Save waypoints
        self.save_waypoints()
        
        # Show map with waypoints
        self.plot_map(show_path=False, show_smoothed=True, show_waypoints=True)
        plt.title(f'Final Path with {len(self.waypoints)} Waypoints')
        plt.show()
        
        # Flash waypoints
        print("\nDisplaying flashing waypoints animation...")
        self.flash_waypoints()
        
        print("\n=== Path Planning Complete ===")
        print(f"Total path length: {self.calculate_path_length(self.waypoints):.2f} meters")
        print(f"Final safe distance: {self.safe_distance:.2f} meters")
        print(f"Final field boundary: X = {self.boundary_x:.2f}m, Y = {self.boundary_y:.2f}m")
    
    def calculate_path_length(self, path):
        """Calculate total length of path"""
        if path is None or len(path) < 2:
            return 0
        
        total = 0
        for i in range(len(path) - 1):
            total += np.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
        return total


def main():
    """Main function to run the path planner"""
    
    # Configuration
    obstacles_file = "obstacles.txt"
    safe_distance = 1.5  # meters
    grid_resolution = 0.3  # meters
    num_waypoints = 20
    
    # Convert yards to meters (1 yard = 0.9144 meters)
    field_size_yards = 15
    field_size_meters = field_size_yards * 0.9144
    
    print("Robot Path Planning System")
    print("-" * 50)
    print("\nNOTE: Robot starts at (0,0) which is at the corner of the field.")
    print("The field extends in the direction specified by boundary X and Y values.")
    print("Positive X: field extends to the right")
    print("Negative X: field extends to the left")
    print("Positive Y: field extends upward")
    print("Negative Y: field extends downward")
    
    # Check if obstacles file exists, if not create a sample
    if not os.path.exists(obstacles_file):
        print(f"\nCreating sample obstacles file: {obstacles_file}")
        with open(obstacles_file, 'w') as f:
            f.write("2.0, 2.0\n")
            f.write("5.0, 4.0\n")
            f.write("3.0, 6.0\n")
            f.write("6.0, 3.0\n")
            f.write("8.0, 8.0\n")
    
    # Allow user to customize parameters
    use_default = input("\nUse default parameters? (y/n): ").strip().lower()
    
    if use_default != 'y':
        try:
            safe_distance = float(input(f"Enter safe distance from obstacles (default {safe_distance}m): ") or safe_distance)
            grid_resolution = float(input(f"Enter grid resolution (default {grid_resolution}m): ") or grid_resolution)
            
            # Get boundary dimensions
            print("\nEnter field boundary (robot is at corner 0,0):")
            boundary_x_input = input(f"  X boundary in yards (+ for right, - for left, default +{field_size_yards}): ").strip()
            boundary_y_input = input(f"  Y boundary in yards (+ for up, - for down, default +{field_size_yards}): ").strip()
            
            if boundary_x_input:
                boundary_x = float(boundary_x_input) * 0.9144
            else:
                boundary_x = field_size_meters
                
            if boundary_y_input:
                boundary_y = float(boundary_y_input) * 0.9144
            else:
                boundary_y = field_size_meters
            
            num_waypoints = int(input(f"Enter number of waypoints (default {num_waypoints}): ") or num_waypoints)
        except:
            print("Invalid input, using defaults")
            boundary_x = field_size_meters
            boundary_y = field_size_meters
    else:
        boundary_x = field_size_meters
        boundary_y = field_size_meters
    
    print(f"\nField boundary: X = {boundary_x:.2f}m ({boundary_x/0.9144:.1f} yards), Y = {boundary_y:.2f}m ({boundary_y/0.9144:.1f} yards)")
    
    # Create and run path planner
    planner = PathPlanner(obstacles_file, safe_distance, grid_resolution, boundary_x, boundary_y)
    planner.run(num_waypoints)


if __name__ == "__main__":
    main()

    