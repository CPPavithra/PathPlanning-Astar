import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from queue import PriorityQueue
import math
from matplotlib.animation import FuncAnimation
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import matplotlib

matplotlib.rcParams['animation.embed_limit'] = 2**128

class RoverSimulation:
    def __init__(self, size=50):
        self.size = size
        self.grid = np.random.rand(size, size) * 10  # Height map (0-10)
        self.obstacles = np.random.rand(size, size) > 0.85  # 15% obstacles
        self.visibility_range = 8
        self.known_terrain = np.full((size, size), np.nan)  # Unknown initially
        
        # Initialize rover at a clear starting position
        self.rover_pos = (0, 0)
        self.goal = (10, 27)
        
        # Ensure start and goal are not obstacles
        self.obstacles[0, 0] = False
        self.obstacles[size-1, size-1] = False
        
        self.path = []
        self.current_target = None  # Current target position (waypoint or exploration target)
        self.waypoints = []
        self.history = []
        self.current_waypoint_idx = 0
        
        self.stats = {
            'distance_traveled': 0,
            'obstacles_avoided': 0,
            'replans': 0,
            'waypoints_reached': 0,
            'exploration_percentage': 0
        }
        
        # Color scheme
        self.colors = {
            'rover': '#ff3333',
            'goal': '#33cc33',
            'path': '#3366ff',
            'obstacle': '#333333',
            'free': '#e6e6e6',
            'unknown': '#808080'
        }

    def update_visibility(self):
        """Update known terrain based on rover's current position and visibility range"""
        x, y = self.rover_pos
        for i in range(max(0, x-self.visibility_range), min(self.size, x+self.visibility_range+1)):
            for j in range(max(0, y-self.visibility_range), min(self.size, y+self.visibility_range+1)):
                # Use Euclidean distance for circular visibility
                distance = math.sqrt((i-x)**2 + (j-y)**2)
                if distance <= self.visibility_range:
                    if self.obstacles[j, i]:  # Note: obstacles uses [row, col] = [j, i]
                        self.known_terrain[j, i] = -1  # Mark as obstacle
                    else:
                        self.known_terrain[j, i] = self.grid[j, i]  # Store actual height
    
    def heuristic(self, a, b):
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_neighbors(self, pos):
        """Get valid neighboring positions"""
        x, y = pos
        neighbors = []
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < self.size and 0 <= ny < self.size):
                neighbors.append((nx, ny))
        return neighbors
    
    def is_traversable(self, pos):
        """Check if a position is traversable based on known terrain"""
        x, y = pos
        if not (0 <= x < self.size and 0 <= y < self.size):
            return False
        
        terrain_value = self.known_terrain[y, x]  # Note: terrain uses [row, col] = [y, x]
        
        # Unknown terrain is considered traversable for exploration
        if np.isnan(terrain_value):
            return True
        
        # Obstacles are not traversable
        if terrain_value < 0:
            return False
            
        return True
    
    def a_star_search(self, start, goal):
        """A* pathfinding algorithm"""
        if start == goal:
            return []
            
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while not frontier.empty():
            _, current = frontier.get()
            
            if current == goal:
                # Reconstruct path
                path = []
                while current != start:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
                
            for next_pos in self.get_neighbors(current):
                if not self.is_traversable(next_pos):
                    continue
                
                # Calculate movement cost (higher for diagonal moves)
                dx = abs(next_pos[0] - current[0])
                dy = abs(next_pos[1] - current[1])
                move_cost = 1.4 if (dx + dy) == 2 else 1.0
                
                new_cost = cost_so_far[current] + move_cost
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(next_pos, goal)
                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current
                    
        return []  # No path found
    
    def can_see_goal(self):
        """Check if the goal is within visible range and reachable"""
        distance = math.sqrt((self.goal[0] - self.rover_pos[0])**2 + 
                           (self.goal[1] - self.rover_pos[1])**2)
        return distance <= self.visibility_range and self.is_traversable(self.goal)
    
    def plan_path_to_target(self, target):
        """Plan path to a specific target and set current target"""
        path = self.a_star_search(self.rover_pos, target)
        if path:
            self.path = path
            self.current_target = target
            self.stats['replans'] += 1
            return True
        return False
    
    def plan_next_move(self):
        """Plan the next move - either to goal if visible, or exploration"""
        # First priority: If goal is visible and reachable, go there
        if self.can_see_goal():
            return self.plan_path_to_target(self.goal)
        
        # Second priority: Find nearest unknown area to explore
        unknown_cells = []
        for i in range(self.size):
            for j in range(self.size):
                if np.isnan(self.known_terrain[j, i]):
                    # Check if it's near the visibility boundary
                    dist_to_rover = math.sqrt((i - self.rover_pos[0])**2 + (j - self.rover_pos[1])**2)
                    if dist_to_rover > self.visibility_range:
                        unknown_cells.append((i, j))
        
        if not unknown_cells:
            # If no unknown areas, try direct path to goal
            return self.plan_path_to_target(self.goal)
        
        # Find closest unknown cell and plan path to explore it
        closest_unknown = min(unknown_cells, key=lambda pos: self.heuristic(self.rover_pos, pos))
        
        # Try to find a path to a position that can see the unknown cell
        for radius in range(1, self.visibility_range + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if dx*dx + dy*dy <= radius*radius:
                        target_x = closest_unknown[0] + dx
                        target_y = closest_unknown[1] + dy
                        target = (target_x, target_y)
                        
                        if (0 <= target_x < self.size and 0 <= target_y < self.size and 
                            self.is_traversable(target)):
                            if self.plan_path_to_target(target):
                                return True
        
        # If no exploration path found, try direct path to goal
        return self.plan_path_to_target(self.goal)
    
    def move_rover(self):
        """Move rover one step along the planned path"""
        if not self.path:
            return False
            
        next_pos = self.path[0]
        
        # Check if next position is still traversable
        if not self.is_traversable(next_pos):
            self.stats['obstacles_avoided'] += 1
            # Clear current path and replan
            self.path = []
            self.current_target = None
            if not self.plan_next_move():
                return False
            if not self.path:
                return False
            next_pos = self.path[0]
        
        # Move rover
        old_pos = self.rover_pos
        self.rover_pos = next_pos
        self.path.pop(0)
        
        # Calculate distance traveled
        dx = abs(self.rover_pos[0] - old_pos[0])
        dy = abs(self.rover_pos[1] - old_pos[1])
        distance = math.sqrt(dx*dx + dy*dy)
        self.stats['distance_traveled'] += distance
        
        # Check if current target reached
        if self.rover_pos == self.current_target:
            self.current_target = None
            self.path = []  # Clear path when target reached
            self.stats['waypoints_reached'] += 1
            
        return True
    
    def run_simulation(self, max_steps=1000):
        """Run the complete simulation"""
        print(f"Starting simulation with {max_steps} max steps...")
        self.history = []
        step = 0
        
        # Initial visibility update
        self.update_visibility()
        
        while step < max_steps:
            # Plan path if needed
            if not self.path:
                if self.rover_pos == self.goal:
                    print(f"Goal reached in {step} steps!")
                    break
                
                # Plan next move (either to goal if visible, or exploration)
                if not self.plan_next_move():
                    print("No valid path found, ending simulation")
                    break
            
            # Move rover
            if not self.move_rover():
                print("Cannot move rover, ending simulation")
                break
                
            # Update visibility
            self.update_visibility()
            
            # Calculate exploration percentage
            total_cells = self.size * self.size
            known_cells = np.sum(~np.isnan(self.known_terrain))
            self.stats['exploration_percentage'] = (known_cells / total_cells) * 100
            
            # Save snapshot for animation
            self.history.append({
                'step': step,
                'position': self.rover_pos,
                'path': self.path.copy() if self.path else [],
                'current_target': self.current_target,
                'known_terrain': self.known_terrain.copy(),
                'stats': self.stats.copy()
            })
            
            step += 1
            
            # Check if goal reached
            if self.rover_pos == self.goal:
                print(f"Goal reached in {step} steps!")
                break
            
            # Progress update
            if step % 100 == 0:
                print(f"Step {step}: Position {self.rover_pos}, Distance to goal: {self.heuristic(self.rover_pos, self.goal)}, Exploration: {self.stats['exploration_percentage']:.1f}%")
        
        print(f"Simulation completed in {step} steps")
        print(f"Final stats: {self.stats}")
    
    def create_animation(self):
        """Create matplotlib animation of the simulation"""
        if not self.history:
            print("No simulation history available for animation")
            return None
            
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
        
        # Setup colormaps
        terrain_cmap = plt.cm.viridis
        
        def update(frame):
            ax1.clear()
            ax2.clear()
            
            if frame >= len(self.history):
                return
                
            data = self.history[frame]
            
            # Left plot: Known terrain
            display_grid = data['known_terrain'].copy()
            
            # Create visualization grid
            vis_grid = np.full((self.size, self.size), 0.5)  # Unknown areas
            
            # Known free areas
            free_mask = (display_grid >= 0) & (~np.isnan(display_grid))
            vis_grid[free_mask] = 0.8
            
            # Obstacles
            obstacle_mask = display_grid < 0
            vis_grid[obstacle_mask] = 0.1
            
            ax1.imshow(vis_grid, cmap='gray', origin='lower')
            
            # Draw current planned path (only to current target, not final goal)
            if data['path']:
                path_x = [p[0] for p in data['path']]
                path_y = [p[1] for p in data['path']]
                ax1.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.7, label='Current Path')
            
            # Draw current target if exists
            if data.get('current_target'):
                target_x, target_y = data['current_target']
                ax1.plot(target_x, target_y, 'bo', markersize=8, label='Current Target')
            
            # Draw rover and goal
            ax1.plot(data['position'][0], data['position'][1], 'ro', markersize=12, label='Rover')
            ax1.plot(self.goal[0], self.goal[1], 'g^', markersize=12, label='Final Goal')
            
            # Draw visibility circle
            circle = plt.Circle(data['position'], self.visibility_range, 
                              fill=False, color='red', linestyle='--', alpha=0.5)
            ax1.add_patch(circle)
            
            ax1.set_xlim(-0.5, self.size-0.5)
            ax1.set_ylim(-0.5, self.size-0.5)
            ax1.set_title(f"Step {data['step']}: Rover Position {data['position']}")
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Right plot: True terrain (for reference)
            ax2.imshow(self.obstacles.astype(int), cmap='gray_r', origin='lower')
            ax2.plot(data['position'][0], data['position'][1], 'ro', markersize=12, label='Rover')
            ax2.plot(self.goal[0], self.goal[1], 'g^', markersize=12, label='Final Goal')
            
            # Draw visibility circle on true terrain too
            circle2 = plt.Circle(data['position'], self.visibility_range, 
                               fill=False, color='red', linestyle='--', alpha=0.5)
            ax2.add_patch(circle2)
            
            ax2.set_xlim(-0.5, self.size-0.5)
            ax2.set_ylim(-0.5, self.size-0.5)
            ax2.set_title("True Environment")
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            
        anim = FuncAnimation(fig, update, frames=len(self.history),
                           interval=200, repeat=True)
        plt.tight_layout()
        return anim
    
    def create_summary_dashboard(self):
        """Create Plotly dashboard with simulation results"""
        if not self.history:
            print("No simulation history available for dashboard")
            return None
            
        steps = [h['step'] for h in self.history]
        distances = [self.heuristic(h['position'], self.goal) for h in self.history]
        exploration = [h['stats']['exploration_percentage'] for h in self.history]
        
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=(
                "Distance to Goal Over Time",
                "Terrain Exploration Progress",
                "Rover Movement Heatmap", 
                "Performance Metrics"
            )
        )
        
        # Distance to goal
        fig.add_trace(
            go.Scatter(x=steps, y=distances, name="Distance to Goal", line=dict(color='red')),
            row=1, col=1
        )
        
        # Exploration progress
        fig.add_trace(
            go.Scatter(x=steps, y=exploration, name="Exploration %", line=dict(color='green')),
            row=1, col=2
        )
        
        # Movement heatmap
        activity = np.zeros((self.size, self.size))
        for h in self.history:
            x, y = h['position']
            activity[y, x] += 1
        
        fig.add_trace(
            go.Heatmap(z=activity, colorscale='YlOrRd', showscale=False),
            row=2, col=1
        )
        
        # Performance metrics
        metrics = ["distance_traveled", "obstacles_avoided", "replans", "waypoints_reached"]
        values = [round(self.stats[m], 2) for m in metrics]
        
        fig.add_trace(
            go.Bar(x=metrics, y=values, marker_color=['blue', 'orange', 'red', 'green']),
            row=2, col=2
        )
        
        fig.update_layout(
            title="Rover Path Planning Simulation Results",
            height=800,
            showlegend=True,
            template="plotly_white"
        )
        
        return fig

# Example usage
if __name__ == "__main__":
    print("Starting rover path planning simulation...")
    
    # Create and run simulation
    sim = RoverSimulation(size=30)  # Smaller size for faster demo
    sim.run_simulation(max_steps=500)
    
    # Create visualizations
    if sim.history:
        print("\nCreating animation...")
        anim = sim.create_animation()
        if anim:
            plt.show()
        
        print("Creating dashboard...")
        dashboard = sim.create_summary_dashboard()
        if dashboard:
            dashboard.show()
    else:
        print("No simulation data available for visualization")
