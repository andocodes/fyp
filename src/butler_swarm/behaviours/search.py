#!/usr/bin/env python3

import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import heapq # For A* priority queue

class SearchBehaviour:
    """Collaborative search Behaviour for exploring an environment."""

    def __init__(self, agent):
        """Initialize with reference to parent agent."""
        self.agent = agent

        # Environment parameters
        self.map_size = 20.0  # meters
        self.cell_size = 1.0  # meters
        self.grid_dims = int(self.map_size / self.cell_size)

        # Initialize grid (0: unknown, 1: visited, 2: obstacle)
        self.grid = np.zeros((self.grid_dims, self.grid_dims))

        # Search parameters
        self.search_radius = 2.0  # meters
        self.frontier_weight = 1.5
        self.exploration_rate = 0.2

        # Movement parameters
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0

        # Visualization
        self.vis_pub = rospy.Publisher('/visualization/search_grid', MarkerArray, queue_size=10)
        self.update_count = 0

        # Shared map
        self.shared_map = {}

        # Target position
        self.target_position = None
        self.current_path = None # Add state for storing the A* path
        self.current_path_index = 0 # Add state for tracking progress along the path

        # --- A* Pathfinding Control ---
        self.MAX_ASTAR_ATTEMPTS_PER_CYCLE = 5 # Limit computation

    def compute(self):
        """Compute the movement vector based on search strategy."""
        # --- Reactive Obstacle Avoidance ---
        OBSTACLE_DISTANCE_THRESHOLD = 0.7 # Increased detection distance
        OBSTACLE_WIDTH_THRESHOLD = 0.5  # Increased detection width
        TURN_SPEED = self.max_angular_speed
        SAFE_LINEAR_SPEED = 0.1 # Slow down when avoiding
        TURN_SCALING = 1.5 

        turn_force = 0.0
        obstacle_imminent = False
        closest_obs_dist = OBSTACLE_DISTANCE_THRESHOLD

        agent_obstacles = getattr(self.agent, 'obstacles', [])
        if not agent_obstacles:
             rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id}: No obstacle data available for avoidance check.")

        for obs_x, obs_y in agent_obstacles:
            if 0 < obs_x < OBSTACLE_DISTANCE_THRESHOLD and abs(obs_y) < OBSTACLE_WIDTH_THRESHOLD + (obs_x * 0.2):
                obstacle_imminent = True
                closest_obs_dist = min(closest_obs_dist, obs_x)
                turn_force -= np.sign(obs_y) * (1 + (OBSTACLE_DISTANCE_THRESHOLD - obs_x) / OBSTACLE_DISTANCE_THRESHOLD) 

        if obstacle_imminent:
            # If avoiding obstacle, clear any existing path plan
            self.current_path = None 
            self.current_path_index = 0
            self.target_position = None # Clear final target too
            rospy.loginfo_throttle(1.0, f"Agent {self.agent.agent_id}: Obstacle detected! Distance: {closest_obs_dist:.2f}, Turn Force: {turn_force:.2f}")
            if abs(turn_force) > 0:
                final_angular_z = np.clip(turn_force * TURN_SCALING, -TURN_SPEED, TURN_SPEED)
            else:
                final_angular_z = TURN_SPEED 
            
            final_linear_x = np.clip(SAFE_LINEAR_SPEED * (closest_obs_dist / OBSTACLE_DISTANCE_THRESHOLD), 0.0, SAFE_LINEAR_SPEED)
            
            rospy.loginfo_throttle(1.0, f"Agent {self.agent.agent_id}: Avoidance maneuver: l={final_linear_x:.2f}, a={final_angular_z:.2f}")
            return final_linear_x, final_angular_z

        # --- Frontier Exploration --- 
        self._update_grid()
        self._share_map()
        self._integrate_shared_maps()
        if self.update_count % 10 == 0:
            self._visualize_grid() # Also visualize path later
        self.update_count += 1

        # Target/Path Management
        replan_needed = False
        if self.current_path:
            # Check if we've reached the current waypoint
            current_waypoint_world = self._grid_to_world(*self.current_path[self.current_path_index])
            if np.linalg.norm(current_waypoint_world - self.agent.position[:2]) < self.cell_size * 0.6: # Threshold based on cell size
                self.current_path_index += 1
                if self.current_path_index >= len(self.current_path):
                    # Reached end of path (final target)
                    rospy.loginfo(f"Agent {self.agent.agent_id}: Reached end of path for target {self.target_position}.")
                    self.current_path = None
                    self.current_path_index = 0
                    self.target_position = None # Clear final target, force replan
                    replan_needed = True
                else:
                     rospy.loginfo(f"Agent {self.agent.agent_id}: Reached waypoint {self.current_path_index-1}, moving to waypoint {self.current_path_index} {self.current_path[self.current_path_index]}.")
        else:
            # No current path, needs planning or replanning
            replan_needed = True

        # Select new frontier target and plan path if needed
        if replan_needed:
            selected_target_world, planned_path = self._select_frontier_and_plan_path()
            if selected_target_world is not None and planned_path is not None:
                rospy.loginfo(f"Agent {self.agent.agent_id}: New target selected {selected_target_world} with path of length {len(planned_path)}.")
                self.target_position = selected_target_world # Store final world goal
                self.current_path = planned_path         # Store grid path
                self.current_path_index = 1               # Start moving towards the *second* node (index 1) as index 0 is the start
                # Handle path of length 1 (start=goal)? A* should prevent this if start!=goal.
                if len(self.current_path) <= 1:
                     rospy.logwarn(f"Agent {self.agent.agent_id}: Path generated has length <= 1. Clearing path.")
                     self.current_path = None
                     self.target_position = None
            else:
                 rospy.loginfo(f"Agent {self.agent.agent_id}: Failed to select a new reachable frontier target.")
                 self.target_position = None
                 self.current_path = None
                 self.current_path_index = 0

        # --- Movement --- 
        # No path / No target -> Stop
        if self.current_path is None or self.current_path_index >= len(self.current_path):
             rospy.loginfo_throttle(5.0, f"Agent {self.agent.agent_id}: No path or target, stopping.")
             # Visualize grid without path/target markers if desired
             self._visualize_grid(visualize_path=False, visualize_target=False)
             return 0.0, 0.0

        # Follow the current path
        next_waypoint_grid = self.current_path[self.current_path_index]
        linear_x, angular_z = self._move_to_waypoint(next_waypoint_grid)
        # Visualize grid, current path, and final target
        self._visualize_grid(visualize_path=True, visualize_target=True) 
        return linear_x, angular_z

    def _update_grid(self):
        """Update the grid based on current observations."""
        # Convert agent position to grid coordinates
        agent_grid_x, agent_grid_y = self._world_to_grid(self.agent.position[0], self.agent.position[1])

        # Mark current cell as visited
        if 0 <= agent_grid_x < self.grid_dims and 0 <= agent_grid_y < self.grid_dims:
            # Ensure current cell isn't marked as obstacle if agent is there
            if self.grid[agent_grid_x, agent_grid_y] == 2:
                 rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id}: Agent started or is in cell marked as obstacle ({agent_grid_x},{agent_grid_y}). Marking as visited.")
            self.grid[agent_grid_x, agent_grid_y] = 1

        # Mark cells within sensor range as visited or obstacles
        for obs in getattr(self.agent, 'obstacles', []):
            # Convert obstacle position (relative to agent) to world coordinates
            # Note: This assumes obs are in agent's forward-facing frame (X forward)
            # If laser is mounted differently, need transformation here.
            current_pos = self.agent.position[:2]
            # Assuming obs[0] is X (forward) and obs[1] is Y (left) in agent frame
            # We might need rotation based on agent heading if obs aren't pre-rotated
            # Let's assume for now that getattr returns obstacles in world-aligned frame for simplicity
            # BASED ON scan_callback, obs are RELATIVE (x,y) in laser frame, aligned with base_link
            # So we DO need to rotate them based on agent heading
            _, _, current_heading = self._get_euler_from_quaternion()
            obs_world_x = current_pos[0] + obs[0] * np.cos(current_heading) - obs[1] * np.sin(current_heading)
            obs_world_y = current_pos[1] + obs[0] * np.sin(current_heading) + obs[1] * np.cos(current_heading)
            
            # Convert to grid coordinates
            grid_x, grid_y = self._world_to_grid(obs_world_x, obs_world_y)

            if 0 <= grid_x < self.grid_dims and 0 <= grid_y < self.grid_dims:
                # ---- Add check: Don't mark agent's own cell as obstacle ----
                if (grid_x, grid_y) != (agent_grid_x, agent_grid_y):
                    self.grid[grid_x, grid_y] = 2 # Mark as obstacle
                # else: # Optional: log if self-scan is detected
                     # rospy.logdebug_throttle(10.0, f"Agent {self.agent.agent_id}: Ignored obstacle marking for own cell ({grid_x},{grid_y}) from scan.")

                # Mark cells between agent and obstacle as free space (visited)
                # Check bounds before calling mark_line
                if 0 <= agent_grid_x < self.grid_dims and 0 <= agent_grid_y < self.grid_dims:
                     self._mark_line(agent_grid_x, agent_grid_y, grid_x, grid_y, mark_end=False)

    def _world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        # Shift to positive quadrant and scale
        grid_x = int((x + self.map_size/2) / self.cell_size)
        grid_y = int((y + self.map_size/2) / self.cell_size)
        return grid_x, grid_y

    def _grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates."""
        world_x = grid_x * self.cell_size - self.map_size/2 + self.cell_size/2
        world_y = grid_y * self.cell_size - self.map_size/2 + self.cell_size/2
        return world_x, world_y

    def _mark_line(self, x0, y0, x1, y1, mark_end=True):
        """Mark all cells on a line between (x0,y0) and (x1,y1) as visited."""
        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if 0 <= x0 < self.grid_dims and 0 <= y0 < self.grid_dims:
                # Mark as visited if not obstacle
                if self.grid[x0, y0] != 2:
                    self.grid[x0, y0] = 1

            if (x0 == x1 and y0 == y1):
                if mark_end and 0 <= x0 < self.grid_dims and 0 <= y0 < self.grid_dims:
                    self.grid[x0, y0] = 2  # Mark endpoint as obstacle
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def _share_map(self):
        """Share our map with neighbors through communication."""
        # Compress the map to only include known cells
        known_cells = []
        for x in range(self.grid_dims):
            for y in range(self.grid_dims):
                if self.grid[x, y] > 0:  # If cell is visited or obstacle
                    known_cells.append((x, y, int(self.grid[x, y])))

        # Publish map information
        map_data = {
            'agent_id': self.agent.agent_id,
            'cells': known_cells,
            'timestamp': rospy.Time.now().to_sec()
        }

        # In a real implementation, this would be published on a topic
        # For simplicity, we'll just update a dictionary
        self.agent.share_data('map', map_data)

    def _integrate_shared_maps(self):
        """Integrate maps shared by neighbors."""
        # Get this agent's current grid coordinates first
        try:
             agent_grid_x, agent_grid_y = self._world_to_grid(self.agent.position[0], self.agent.position[1])
             current_agent_cell = (agent_grid_x, agent_grid_y)
        except AttributeError:
             rospy.logerr("Agent position not available for map integration self-check.")
             return # Cannot proceed without position
        
        # Get map data from the communication module
        shared_maps = self.agent.comm.get_shared_map_data()
        
        for agent_id, data in shared_maps.items():
            # Skip own data (should be handled by comms but double check)
            # if agent_id == self.agent.agent_id:
            #     continue
            
            if 'cells' not in data:
                rospy.logwarn_throttle(10, f"Agent {self.agent.agent_id}: Received map data from agent {agent_id} without 'cells' key.")
                continue

            for cell in data['cells']:
                if not isinstance(cell, (list, tuple)) or len(cell) != 3:
                    rospy.logwarn_throttle(10, f"Agent {self.agent.agent_id}: Received malformed cell data from agent {agent_id}: {cell}")
                    continue
                    
                x, y, state = cell
                cell_coords = (x, y)
                
                if 0 <= x < self.grid_dims and 0 <= y < self.grid_dims:
                    # --- Add Check: Don't let neighbours mark our current cell as an obstacle --- 
                    is_own_cell = (cell_coords == current_agent_cell)
                    current_state = self.grid[x, y]

                    # Conditions for updating:
                    # 1. Cell is currently unknown (0)
                    # 2. Cell is known (1 or 2), but incoming state is obstacle (2), *unless* it's our own cell.
                    # 3. Cell is known obstacle (2), and incoming state is visited (1) -> Downgrade? (Maybe not safe)
                    
                    should_update = False
                    if current_state == 0: # If unknown, accept any update
                        # But still don't mark own cell as obstacle based on neighbor
                        if not (is_own_cell and state == 2):
                            should_update = True
                    elif current_state == 1: # If visited, only accept obstacle update
                        if state == 2 and not is_own_cell: # Only accept obstacle if it's NOT our own cell
                            should_update = True
                    # else: current_state == 2 (Obstacle) - Generally, don't overwrite obstacle with visited from neighbour?
                    # Let's stick to only upgrading to obstacle or filling unknown.

                    if should_update:
                        self.grid[x, y] = state

    def _select_frontier_and_plan_path(self):
        """Select the best frontier (top N attempts) and find an A* path."""
        frontiers = []
        for x in range(self.grid_dims):
            for y in range(self.grid_dims):
                if self.grid[x, y] == 0:
                    for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                        nx, ny = x + dx, y + dy
                        if (0 <= nx < self.grid_dims and 0 <= ny < self.grid_dims and self.grid[nx, ny] == 1):
                            frontiers.append((x, y))
                            break
        if not frontiers:
             rospy.loginfo(f"Agent {self.agent.agent_id}: No frontiers found during selection.")
             return None, None

        # Calculate utility
        utilities = []
        agent_pos = self.agent.position[:2]
        agent_grid_x, agent_grid_y = self._world_to_grid(agent_pos[0], agent_pos[1])
        start_grid_node = (agent_grid_x, agent_grid_y)

        for x, y in frontiers:
            world_x, world_y = self._grid_to_world(x, y)
            distance = np.linalg.norm(np.array([world_x, world_y]) - agent_pos)
            potential = 0
            for dx in range(-2, 3): 
                for dy in range(-2, 3): 
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < self.grid_dims and 0 <= ny < self.grid_dims and self.grid[nx, ny] == 0):
                        potential += 1
            utility = potential * self.frontier_weight - distance
            utilities.append((utility, (x, y), (world_x, world_y)))

        # Sort frontiers by utility
        utilities.sort(key=lambda item: item[0], reverse=True)

        # Find the best frontier for which A* finds a path (LIMIT ATTEMPTS)
        attempts = 0
        for utility, target_grid_node, target_world_coords in utilities:
            # Check if we exceeded the attempt limit for this cycle
            if attempts >= self.MAX_ASTAR_ATTEMPTS_PER_CYCLE:
                rospy.loginfo(f"Agent {self.agent.agent_id}: Reached max A* attempts ({self.MAX_ASTAR_ATTEMPTS_PER_CYCLE}) for this cycle.")
                break # Stop trying for this cycle

            attempts += 1
            # rospy.loginfo(f"Agent {self.agent.agent_id}: Attempting A* ({attempts}/{self.MAX_ASTAR_ATTEMPTS_PER_CYCLE}) to grid {target_grid_node} (utility {utility:.2f})")
            path = self._find_path(start_grid_node, target_grid_node)
            
            if path is not None and len(path) > 1:
                # rospy.loginfo(f"Agent {self.agent.agent_id}: A* path found to {target_grid_node}.")
                # Log success only when returning
                rospy.loginfo(f"Agent {self.agent.agent_id}: Found reachable frontier {target_grid_node} (utility {utility:.2f}) on attempt {attempts}.")
                return np.array(target_world_coords), path
            # else: # Path failed, continue to next utility if attempts remain
            pass # Correct indentation for the pass statement

        # No path found within the allowed attempts or no frontiers left
        rospy.logwarn(f"Agent {self.agent.agent_id}: Could not find A* path to any of top {attempts} frontiers.")
        return None, None

    def _move_to_waypoint(self, target_grid_coords):
        """Generate movement commands to reach the next waypoint (grid coords)."""
        # Convert grid waypoint to world coordinates
        target_world_pos = self._grid_to_world(*target_grid_coords)

        # Get current position and orientation
        current_pos = self.agent.position[:2]
        _, _, current_heading = self._get_euler_from_quaternion()

        # Vector to target waypoint
        direction = target_world_pos - current_pos
        distance = np.linalg.norm(direction)

        # Reduce speed when close to waypoint to avoid overshooting?
        # Maybe needed later, for now use max speed.
        linear_vel_scale = 1.0 
        # K_p_linear = 1.0 # Proportional control for linear vel?
        # linear_x = np.clip(distance * K_p_linear, 0, self.max_linear_speed)

        if distance > 0.05: # Only calculate movement if not already at waypoint
            # Normalize direction
            unit_direction = direction / distance
            # Calculate desired heading
            desired_heading = np.arctan2(unit_direction[1], unit_direction[0])
            # Calculate angular difference
            angular_diff = self._get_angular_difference(current_heading, desired_heading)

            # Proportional control for angular velocity
            K_p_angular = 2.5 # Tuning parameter
            angular_z = np.clip(angular_diff * K_p_angular, -self.max_angular_speed, self.max_angular_speed)
            
            # --- Adjusted Turn Reduction --- 
            # Reduce linear speed less aggressively when turning.
            # Example: Reduce linearly down to 30% speed for a 180-degree turn.
            # Max reduction factor (0.7 means minimum speed is 30%)
            max_reduction_factor = 0.7 
            turn_reduction = 1.0 - max_reduction_factor * (abs(angular_diff) / np.pi)
            # Ensure turn_reduction doesn't go below a minimum threshold (e.g., 0.1)
            turn_reduction = max(0.1, turn_reduction)

            linear_x = self.max_linear_speed * turn_reduction * linear_vel_scale
            
            # If very close to the waypoint, prioritize alignment? 
            # if distance < self.cell_size * 0.5:
            #    linear_x = self.max_linear_speed * 0.2 # Creep speed

            # Log the intended movement towards the waypoint
            # rospy.loginfo_throttle(1.0, f"Agent {self.agent.agent_id}: Moving to waypoint {target_grid_coords} @ {target_world_pos}. Vel: l={linear_x:.2f}, a={angular_z:.2f}")
            return linear_x, angular_z
        else:
            # Already at waypoint, stop/wait for path index update
             # rospy.loginfo(f"Agent {self.agent.agent_id}: Reached vicinity of waypoint {target_grid_coords}.")
             return 0.0, 0.0

    def _visualize_grid(self, visualize_path=True, visualize_target=True):
        """Visualize the grid, path, and target as markers in RViz."""
        marker_array = MarkerArray()
        marker_id = 0
        # Use agent's odometry frame instead of world
        viz_frame_id = f"{self.agent.agent_name}/odom" 

        # --- Grid Visualization --- 
        for x in range(self.grid_dims):
            for y in range(self.grid_dims):
                marker = Marker()
                marker.header.frame_id = viz_frame_id # Use agent odom frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "search_grid"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                world_x, world_y = self._grid_to_world(x, y)
                marker.pose.position.x = world_x
                marker.pose.position.y = world_y
                marker.pose.position.z = 0.0 # Place grid slightly lower
                marker.pose.orientation.w = 1.0
                marker.scale.x = self.cell_size # Fill the cell
                marker.scale.y = self.cell_size
                marker.scale.z = 0.05 # Make them thin
                
                marker.color = ColorRGBA()
                cell_state = self.grid[x, y]

                if cell_state == 1: # Visited
                    marker.color.g = 0.7; marker.color.a = 0.5 # Green
                elif cell_state == 2: # Obstacle
                    marker.color.r = 0.7; marker.color.a = 0.7 # Red
                elif cell_state == 0: # Unknown
                    marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5; # Grey
                    marker.color.a = 0.2 # Make unknown faint
                else: # Should not happen
                     continue # Skip invalid states

                marker_array.markers.append(marker)

        # --- Path Visualization --- 
        if visualize_path and self.current_path is not None and self.current_path_index < len(self.current_path):
            path_marker = Marker()
            path_marker.header.frame_id = viz_frame_id # Use agent odom frame
            path_marker.header.stamp = rospy.Time.now()
            path_marker.ns = "planned_path"
            path_marker.id = marker_id 
            marker_id += 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.pose.orientation.w = 1.0
            path_marker.scale.x = 0.15 # Slightly thicker line
            path_marker.color.b = 1.0; path_marker.color.a = 0.9 # Bright Blue

            for i in range(self.current_path_index, len(self.current_path)):
                grid_x, grid_y = self.current_path[i]
                world_x, world_y = self._grid_to_world(grid_x, grid_y)
                p = Point(x=world_x, y=world_y, z=0.1) # Draw path slightly higher
                path_marker.points.append(p)
            
            if path_marker.points:
                 marker_array.markers.append(path_marker)

        # --- Target Visualization --- 
        if visualize_target and self.target_position is not None:
            target_marker = Marker()
            target_marker.header.frame_id = viz_frame_id # Use agent odom frame
            target_marker.header.stamp = rospy.Time.now()
            target_marker.ns = "final_target"
            target_marker.id = marker_id
            marker_id += 1
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = self.target_position[0]
            target_marker.pose.position.y = self.target_position[1]
            target_marker.pose.position.z = 0.2
            target_marker.pose.orientation.w = 1.0
            target_marker.scale.x = 0.3; target_marker.scale.y = 0.3; target_marker.scale.z = 0.3
            target_marker.color.r = 1.0; target_marker.color.g = 1.0; target_marker.color.a = 0.9 # Bright Yellow
            marker_array.markers.append(target_marker)

        # --- Clear Old Markers --- 
        if not any(m.ns == "planned_path" for m in marker_array.markers):
             # Pass the correct frame_id for delete markers too
             delete_header = rospy.Header(frame_id=viz_frame_id, stamp=rospy.Time.now())
             delete_marker = Marker(header=delete_header, ns="planned_path", id=0, action=Marker.DELETEALL)
             marker_array.markers.append(delete_marker)
        if not any(m.ns == "final_target" for m in marker_array.markers):
             delete_header = rospy.Header(frame_id=viz_frame_id, stamp=rospy.Time.now())
             delete_marker = Marker(header=delete_header, ns="final_target", id=0, action=Marker.DELETEALL)
             marker_array.markers.append(delete_marker)

        self.vis_pub.publish(marker_array)

    def _get_euler_from_quaternion(self):
        """Convert the agent's quaternion to Euler angles."""
        # Extract quaternion
        x = self.agent.orientation[0]
        y = self.agent.orientation[1]
        z = self.agent.orientation[2]
        w = self.agent.orientation[3]

        # Roll (around x-axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (around y-axis)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (around z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _get_angular_difference(self, current, desired):
        """Calculate the shortest angular difference between two angles."""
        diff = desired - current
        # Normalize to [-pi, pi]
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff

    def _find_path(self, start_grid, goal_grid):
        """Finds a path from start_grid to goal_grid using A* on self.grid."""
        
        rows, cols = self.grid.shape
        start_node = tuple(start_grid)
        goal_node = tuple(goal_grid)

        # Check if start or goal are obstacles
        if not (0 <= start_node[0] < rows and 0 <= start_node[1] < cols and self.grid[start_node] != 2):
            rospy.logwarn(f"A*: Start node {start_node} is invalid or obstacle.")
            return None
        if not (0 <= goal_node[0] < rows and 0 <= goal_node[1] < cols and self.grid[goal_node] != 2):
            # Allow goal to be unknown (0) or visited (1)
             if self.grid[goal_node] == 2:
                 rospy.logwarn(f"A*: Goal node {goal_node} is an obstacle.")
                 return None
            # If goal is outside grid, maybe still plan to nearest valid point? For now, fail.
             elif not (0 <= goal_node[0] < rows and 0 <= goal_node[1] < cols):
                  rospy.logwarn(f"A*: Goal node {goal_node} is outside grid bounds.")
                  return None

        # Define neighbors (4-connectivity for grid)
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)] # Right, Left, Down, Up
        # Optional: Add diagonals: [(1, 1), (1, -1), (-1, 1), (-1, -1)]

        # --- Define Costs --- 
        COST_VISITED = 1.0
        COST_UNKNOWN = 5.0 # Penalize planning through unknown space
        COST_OBSTACLE = 1000.0 # High cost, but not infinite, allows planning *through* if necessary

        close_set = set()
        came_from = {}
        gscore = {start_node: 0}
        fscore = {start_node: self._heuristic(start_node, goal_node)}
        
        oheap = []
        heapq.heappush(oheap, (fscore[start_node], start_node))

        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == goal_node:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_node) # Add start node itself
                return path[::-1] # Return reversed path

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                
                # Check bounds
                if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                    continue
                
                # --- Calculate Cost based on Cell State --- 
                neighbor_state = self.grid[neighbor]
                cost_to_neighbor = 0
                if neighbor_state == 1: # Visited
                    cost_to_neighbor = COST_VISITED
                elif neighbor_state == 0: # Unknown
                    cost_to_neighbor = COST_UNKNOWN
                elif neighbor_state == 2: # Obstacle
                    cost_to_neighbor = COST_OBSTACLE
                    # Add a check here? Maybe still block if cost is HUGE?
                    # For now, allow pathing through at high cost.
                else: # Should not happen
                     rospy.logwarn_throttle(10.0, f"A*: Encountered unexpected grid state {neighbor_state} at {neighbor}")
                     continue # Skip invalid state
                
                # Check if obstacle cost makes it effectively impassable (optional)
                # if cost_to_neighbor >= COST_OBSTACLE:
                #     continue
                    
                # Calculate tentative gscore
                tentative_g_score = gscore[current] + cost_to_neighbor

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue

                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self._heuristic(neighbor, goal_node)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        
        rospy.logwarn(f"A*: Failed to find a path from {start_node} to {goal_node}")
        return None # Path not found

    def _heuristic(self, node_a, node_b):
        """Heuristic function for A* (Manhattan distance)."""
        return abs(node_a[0] - node_b[0]) + abs(node_a[1] - node_b[1])
        # Optional: Euclidean distance: return np.sqrt((node_a[0] - node_b[0])**2 + (node_a[1] - node_b[1])**2)