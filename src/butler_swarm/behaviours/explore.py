#!/usr/bin/env python3

import rospy
import numpy as np
import json
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point
import tf.transformations
import heapq
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import collections
import math
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from .avoidance_base import ObstacleAvoidanceBehaviour

# Define grid state constants
GRID_UNKNOWN = 0
GRID_FREE = 1
GRID_OBSTACLE = 2
GRID_INFLATED = 3

class ExploreBehaviour(ObstacleAvoidanceBehaviour):
    """Behaviour for collaborative exploration using frontier detection and task allocation."""

    def __init__(self, agent):
        """Initialize the Explore behaviour."""
        super(ExploreBehaviour, self).__init__(agent)

        self.OBSTACLE_DISTANCE_THRESHOLD = 0.5  # Distance threshold to consider an obstacle (meters)
        
        self.max_linear_speed = rospy.get_param('~explore/max_linear_speed', 0.5)  # m/s
        self.max_angular_speed = rospy.get_param('~explore/max_angular_speed', 1.0)  # rad/s
        
        self.known_frontiers = {}  # Store frontiers: {frontier_id: frontier_data_dict}
        self.current_target_frontier_id = None

        # Grid Map Parameters
        self.map_size = rospy.get_param('~explore/map_size', 20.0)  # meters
        self.cell_size = rospy.get_param('~explore/cell_size', 0.5) # meters
        if self.cell_size <= 0:
             rospy.logwarn("Cell size must be positive, defaulting to 0.5m")
             self.cell_size = 0.5
        self.grid_dims = int(self.map_size / self.cell_size) 
        # Initialize grid (0: unknown, 1: free/visited, 2: obstacle, 3: inflated obstacle)
        # Center the initial grid around (0,0) in world coords
        # Grid index (0,0) corresponds to world (-map_size/2, -map_size/2)
        self.grid_origin_offset = np.array([-self.map_size/2.0, -self.map_size/2.0])
        self.grid = np.zeros((self.grid_dims, self.grid_dims), dtype=np.int8)
        # Mark initial cell around (0,0) as free maybe? Or rely on first scan.
        initial_grid_x, initial_grid_y = self._world_to_grid(0.0, 0.0)
        if 0 <= initial_grid_x < self.grid_dims and 0 <= initial_grid_y < self.grid_dims:
             self.grid[initial_grid_x, initial_grid_y] = GRID_FREE 

        # Inflation parameters (currently unused by logic, but keep params)
        self.inflation_radius_cells = rospy.get_param('~explore/inflation_radius_cells', 2) # cells 

        # --- Path Planning State --- 
        self.current_path = None # List of (grid_x, grid_y) tuples
        self.current_path_index = 0 # Index of the next waypoint in current_path
        
        # --- Movement parameters (Use agent's max speeds) ---
        # Using self.agent properties: self.agent.max_linear_speed, self.agent.max_angular_speed

        # --- Termination State --- #
        self.exploration_complete = False
        self.last_activity_time = rospy.Time.now() # Initialize for timeout check
        self.no_frontiers_timeout = rospy.Duration(rospy.get_param('~explore/no_frontiers_timeout', 15.0)) # Stop after X sec of no open frontiers
        self.path_planning_failures = {} # Track consecutive A* failures per frontier
        self.MAX_PATH_FAILURES = rospy.get_param('~explore/max_path_failures', 3) # Release claim after this many consecutive failures

        # --- Path Stuck Detection --- #
        self.last_path_progress_time = None
        self.last_path_position = None
        self.PATH_STUCK_TIMEOUT = rospy.Duration(rospy.get_param('~explore/path_stuck_timeout', 7.0)) # Timeout if no progress on path
        self.MIN_PROGRESS_DIST = self.cell_size * 0.5 # Min distance moved to count as progress

        # --- TF Listener --- #
        # Use agent's TF buffer and listener (initialized in base class, but use agent's instance)
        self.tf_buffer = self.agent.tf_buffer
        self.tf_listener = self.agent.tf_listener # Reuse agent's listener
        # Frame names
        self.odom_frame = f"{self.agent.agent_name}/odom"
        self.base_frame = self.agent.base_frame # Use agent's base frame
        self.laser_frame = self.agent.laser_frame # Use agent's laser frame

        # Publishers for visualization
        self.grid_pub = rospy.Publisher(f'/{self.agent.agent_name}/explore_grid', Marker, queue_size=2, latch=True)
        self.path_pub = rospy.Publisher(f'/{self.agent.agent_name}/explore_path', Marker, queue_size=2, latch=True)
        self.frontier_pub = rospy.Publisher(f'/{self.agent.agent_name}/explore_frontiers', MarkerArray, queue_size=5, latch=True)

        # Register callback for frontier updates from other agents
        self.agent.comm.register_callback('frontiers', self.frontier_update_callback)

        # Initialize timers to None
        self.update_grid_timer = None
        self.frontier_publish_timer = None
        self.visualization_timer = None
        self.bidding_timer = None

        rospy.loginfo(f"Explore behaviour for agent {self.agent.agent_id} initialized.")
        rospy.loginfo(f"Grid Dimensions: {self.grid_dims}x{self.grid_dims} ({self.map_size}m @ {self.cell_size}m/cell)")

    def start(self):
        """Start the timers for background processing when Explore behaviour becomes active."""
        rospy.loginfo(f"Agent {self.agent.agent_id}: Starting Explore behaviour timers.")
        # Shutdown existing timers first to prevent duplicates if start is called again
        self.stop()
        
        # Start timers
        self.update_grid_timer = rospy.Timer(rospy.Duration(0.1), self._update_grid_callback) # 10 Hz
        self.frontier_publish_timer = rospy.Timer(rospy.Duration(2.0), self._publish_frontiers_callback) # 0.5 Hz
        self.visualization_timer = rospy.Timer(rospy.Duration(0.5), self._visualization_callback) # 2 Hz
        self.bidding_timer = rospy.Timer(rospy.Duration(1.0), self._bidding_cycle_callback) # 1 Hz
        # Reset relevant state when starting?
        self.last_activity_time = rospy.Time.now() # Reset activity timer
        self.exploration_complete = False # Ensure not marked complete

    def stop(self):
        """Stop the timers and reset state when Explore behaviour becomes inactive."""
        rospy.loginfo(f"Agent {self.agent.agent_id}: Stopping Explore behaviour timers.")
        if self.update_grid_timer:
            self.update_grid_timer.shutdown()
            self.update_grid_timer = None
        if self.frontier_publish_timer:
            self.frontier_publish_timer.shutdown()
            self.frontier_publish_timer = None
        if self.visualization_timer:
            self.visualization_timer.shutdown()
            self.visualization_timer = None
        if self.bidding_timer:
            self.bidding_timer.shutdown()
            self.bidding_timer = None
        
        # Reset path and target when stopping explore
        self.current_path = None
        if self.current_target_frontier_id:
             # Optionally release claim if stopping? Or assume switch is temporary?
             # Let's not release claim automatically on stop for now.
             pass 
        self.current_target_frontier_id = None
        self._publish_current_path() # Publish empty path to clear RViz
        # Reset path progress state
        self.last_path_position = None
        self.last_path_progress_time = None
        # Let's keep avoidance state persistent for now.
        # self.avoidance_state = 'none'
        # self.stuck_start_time = None

    def compute(self):
        """Compute movement commands for exploration."""
        # === Initial State Debug Logging START ===
        try:
            agent_pos = self.agent.position[:2]
            agent_grid_x, agent_grid_y = self._world_to_grid(agent_pos[0], agent_pos[1])
            current_cell_state = "OOB" # Out Of Bounds
            if 0 <= agent_grid_x < self.grid_dims and 0 <= agent_grid_y < self.grid_dims:
                current_cell_state = self.grid[agent_grid_x, agent_grid_y]
            rospy.logdebug(f"Agent {self.agent.agent_id} Compute Start: Pos=({agent_pos[0]:.2f},{agent_pos[1]:.2f}), Grid=({agent_grid_x},{agent_grid_y}), GridState={current_cell_state}")
        except Exception as e:
            rospy.logwarn(f"Agent {self.agent.agent_id} Compute Start: Error getting initial grid state: {e}")
        # === Initial State Debug Logging END ===

        # --- Check for Termination Conditions --- #
        if self.exploration_complete:
            rospy.loginfo_once(f"Agent {self.agent.agent_id}: Exploration complete. Stopping.")
            return 0.0, 0.0
        # Check if timed out due to no frontiers
        if self.last_activity_time and (rospy.Time.now() - self.last_activity_time > self.no_frontiers_timeout):
             rospy.logwarn(f"Agent {self.agent.agent_id}: No activity/open frontiers for {self.no_frontiers_timeout.to_sec()}s. Exploration timed out.")
             self.exploration_complete = True
             return 0.0, 0.0

        # --- Compute Avoidance / Recovery --- #
        # Call the base class method to check state and compute avoidance velocities
        avoid_linear_x, avoid_angular_z, should_override = self.compute_avoidance()
        if should_override:
             # If compute_avoidance returns override=True, use its velocities
             return avoid_linear_x, avoid_angular_z
        # --- End Avoidance / Recovery --- #

        # --- Normal Exploration Logic (Path Following or Task Selection) --- #
        linear_x, angular_z = 0.0, 0.0
        
        # If we have a path, follow it
        if self.current_path:
            linear_x, angular_z = self._follow_path()
            # Check if path following resulted in being stuck (triggers avoidance/recovery)
            self._check_path_progress()
        else:
            # No path: Find and claim a new target frontier (handled by bidding cycle)
            rospy.logdebug_throttle(5.0, f"Agent {self.agent.agent_id}: No current path. Waiting for bidding cycle to select target.")
            # While waiting for a path, maybe just spin slowly?
            linear_x = 0.0
            angular_z = 0.1 # Slow rotation while idle?

        return linear_x, angular_z

    def _check_path_progress(self):
        """Checks if the agent is making progress along the current path."""
        if not self.current_path or self.avoidance_state != 'none':
            # Don't check progress if no path or currently avoiding/recovering
            return

        now = rospy.Time.now()
        current_pos = self.agent.position[:2]

        if self.last_path_position is None:
            # Initialize on first check
            self.last_path_position = current_pos
            self.last_path_progress_time = now
            return

        distance_moved = np.linalg.norm(current_pos - self.last_path_position)

        if distance_moved >= self.MIN_PROGRESS_DIST:
            # Made progress, reset timer and position
            self.last_path_position = current_pos
            self.last_path_progress_time = now
            rospy.logdebug(f"Agent {self.agent.agent_id}: Path progress detected ({distance_moved:.2f}m). Resetting stuck timer.")
        elif (now - self.last_path_progress_time) > self.PATH_STUCK_TIMEOUT:
            # No progress for too long, likely stuck!
            rospy.logwarn(f"Agent {self.agent.agent_id}: Detected path stuck! No progress for {self.PATH_STUCK_TIMEOUT.to_sec()}s. Distance moved: {distance_moved:.2f}m. Triggering recovery.")
            # Reset path (or should we try recovery first?)
            # For now, let's trigger recovery and keep the path
            # self.current_path = None
            # self._publish_current_path()
            # Start recovery jiggle
            self.avoidance_state = 'recovery_jiggle'
            self.recovery_state = None # Will be set to 'backward' by compute_avoidance
            self.stuck_start_time = now # Record when stuck state started
            # Reset progress tracking
            self.last_path_position = None
            self.last_path_progress_time = None

            # Also increment failure count for the current frontier?
            if self.current_target_frontier_id:
                 failures = self.path_planning_failures.get(self.current_target_frontier_id, 0) + 1
                 self.path_planning_failures[self.current_target_frontier_id] = failures
                 rospy.logwarn(f"Agent {self.agent.agent_id}: Path stuck increments failure count for {self.current_target_frontier_id} to {failures}")
                 if failures >= self.MAX_PATH_FAILURES:
                      rospy.logwarn(f"Agent {self.agent.agent_id}: Max path failures ({self.MAX_PATH_FAILURES}) reached for {self.current_target_frontier_id} due to being stuck. Releasing claim.")
                      self._release_claim(self.current_target_frontier_id)
                      # Reset avoidance state after releasing claim?
                      self.avoidance_state = 'none' 
                      self.stuck_start_time = None
            else:
                 rospy.logwarn(f"Agent {self.agent.agent_id}: Path stuck but no current target frontier ID set.")
                 # If no target, just reset avoidance state after jiggle completes.
                 pass 

    def _follow_path(self):
        """Follow the current A* path using a simple P-controller."""
        if not self.current_path or self.current_path_index >= len(self.current_path):
            rospy.loginfo(f"Agent {self.agent.agent_id}: Reached end of path or no path.")
            if self.current_target_frontier_id:
                 rospy.loginfo(f"Agent {self.agent.agent_id}: Marking frontier {self.current_target_frontier_id} as explored.")
                 self._mark_frontier_explored(self.current_target_frontier_id)
                 self.current_target_frontier_id = None # Clear target after reaching
                 # Reset path failure count on success
                 self.path_planning_failures.pop(self.current_target_frontier_id, None)
            self.current_path = None
            self.current_path_index = 0
            self._publish_current_path() # Publish empty path
            return 0.0, 0.0

        # Get current position and next waypoint in world coordinates
        current_pos_world = self.agent.position[:2]
        next_waypoint_grid = self.current_path[self.current_path_index]
        next_waypoint_world = self._grid_to_world(next_waypoint_grid[0], next_waypoint_grid[1])

        # Calculate vector and distance to the next waypoint
        direction_vector = next_waypoint_world - current_pos_world
        distance_to_waypoint = np.linalg.norm(direction_vector)

        # Check if waypoint is reached
        WAYPOINT_REACHED_THRESHOLD = self.cell_size # Adjust as needed
        if distance_to_waypoint < WAYPOINT_REACHED_THRESHOLD:
            rospy.logdebug(f"Agent {self.agent.agent_id}: Reached waypoint {self.current_path_index}/{len(self.current_path)-1} at grid {next_waypoint_grid}.")
            self.current_path_index += 1
            # Reset path progress check on reaching waypoint
            self.last_path_position = None 
            self.last_path_progress_time = None
            # Check if this was the last waypoint
            if self.current_path_index >= len(self.current_path):
                rospy.loginfo(f"Agent {self.agent.agent_id}: Reached final waypoint of the path.")
                # Final handling (marking frontier, etc.) done at start of next call
                return 0.0, 0.0
            else:
                 # Get the *new* next waypoint for heading calculation
                 next_waypoint_grid = self.current_path[self.current_path_index]
                 next_waypoint_world = self._grid_to_world(next_waypoint_grid[0], next_waypoint_grid[1])
                 direction_vector = next_waypoint_world - current_pos_world
                 distance_to_waypoint = np.linalg.norm(direction_vector)


        # Calculate desired heading
        desired_heading = math.atan2(direction_vector[1], direction_vector[0])
        current_heading = self.agent.get_yaw()
        angular_diff = self._normalize_angle(desired_heading - current_heading)

        # Simple P-controller for angular velocity
        ANGULAR_P_GAIN = 1.5 # Tune this
        angular_z = np.clip(angular_diff * ANGULAR_P_GAIN, -self.agent.max_angular_speed, self.max_angular_speed)

        # Simple P-controller for linear velocity (reduce speed when turning)
        LINEAR_P_GAIN = 0.8 # Tune this
        # Reduce linear speed significantly if turning sharply
        turn_reduction_factor = max(0.1, 1.0 - abs(angular_diff) / (math.pi / 2.0)) # Reduce more for >90deg turns
        linear_x = np.clip(distance_to_waypoint * LINEAR_P_GAIN * turn_reduction_factor, 0.0, self.agent.max_linear_speed)

        # Prevent oscillation: if angular error is large, prioritize turning
        ANGLE_THRESHOLD_FOR_TURN_ONLY = np.deg2rad(30) # Tune this
        if abs(angular_diff) > ANGLE_THRESHOLD_FOR_TURN_ONLY:
            linear_x = 0.0 # Stop moving forward to turn more effectively

        rospy.logdebug(f"Agent {self.agent.agent_id} PathFollow: Waypt={next_waypoint_grid}, Dist={distance_to_waypoint:.2f}, AngDiff={math.degrees(angular_diff):.1f}, LinX={linear_x:.2f}, AngZ={angular_z:.2f}")

        # --- Add Potential Field steering if very close to obstacles --- #
        # This provides a finer-grained avoidance than the state machine alone
        # Check obstacles directly in front
        min_obstacle_dist = float('inf')
        # Use raw scan for this check?
        ranges = self.agent.last_scan_ranges
        angle_min = self.agent.last_scan_angle_min
        angle_increment = self.agent.last_scan_angle_increment
        forward_check_angle = np.deg2rad(15) # Check +/- 15 degrees

        if ranges:
            for i, r in enumerate(ranges):
                if not np.isfinite(r) or r <= 0:
                    continue
                angle = angle_min + i * angle_increment
                if abs(angle) < forward_check_angle:
                     min_obstacle_dist = min(min_obstacle_dist, r)
        
        PF_CLOSE_THRESHOLD = 0.3 # Threshold to trigger PF override during path following
        if min_obstacle_dist < PF_CLOSE_THRESHOLD:
             rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id}: PathFollow - Obstacle very close ({min_obstacle_dist:.2f}m < {PF_CLOSE_THRESHOLD:.2f}m). Using PF for immediate avoidance.")
             # Calculate PF velocity, passing the direction to the next waypoint as the attraction vector
             attraction = direction_vector # Use the already calculated vector to the next waypoint
             pf_linear_x, pf_angular_z = self._calculate_potential_field_velocity(attraction_vector=attraction) 
             # Blend with path following? Or just use PF?
             # Let's just use PF when very close.
             linear_x = pf_linear_x
             angular_z = pf_angular_z
             # Check if this close obstacle should trigger the full avoidance state
             # Use a slightly larger threshold for state transition than the immediate PF override
             if min_obstacle_dist < (PF_CLOSE_THRESHOLD + 0.1): 
                 if self.avoidance_state == 'none': # Only transition if not already avoiding/recovering
                     rospy.logwarn(f"Agent {self.agent.agent_id}: Obstacle detected ({min_obstacle_dist:.2f}m) during path following. Entering 'avoiding' state.")
                     self.avoidance_state = 'avoiding'
                     self.pf_avoid_start_time = rospy.Time.now()
                     if self.stuck_start_time is None: # Start overall stuck timer
                         self.stuck_start_time = self.pf_avoid_start_time
                     # Stop motion immediately when entering state
                     linear_x = 0.0
                     angular_z = 0.0

        return linear_x, angular_z

    def _update_grid_callback(self, event):
        """Timer callback to update the occupancy grid based on laser scan."""
        if not self.agent.last_scan_ranges:
            return # No scan data yet

        # Get agent's current position in grid coordinates
        try:
            agent_pos_world = self.agent.position[:2]
            agent_grid_x, agent_grid_y = self._world_to_grid(agent_pos_world[0], agent_pos_world[1])
        except Exception as e:
             rospy.logerr(f"Agent {self.agent.agent_id} _update_grid: Failed to get agent grid position: {e}")
             return
             
        # Ensure agent position is within grid bounds
        if not (0 <= agent_grid_x < self.grid_dims and 0 <= agent_grid_y < self.grid_dims):
             rospy.logwarn_throttle(10, f"Agent {self.agent.agent_id} is outside the grid bounds! Cannot update grid.")
             return

        # Mark agent's current cell as free
        self.grid[agent_grid_x, agent_grid_y] = GRID_FREE

        # Get laser scan data (use stored data)
        ranges = self.agent.last_scan_ranges
        angle_min = self.agent.last_scan_angle_min
        angle_increment = self.agent.last_scan_angle_increment
        lookup_time = self.agent.last_scan_time # Use the timestamp from the scan

        # Try to get the transform from laser frame to odom frame at the scan time
        try:
            # --- Increased Timeout --- #
            transform = self.tf_buffer.lookup_transform(self.odom_frame, self.laser_frame, lookup_time, rospy.Duration(0.2)) # Increased from 0.1
            # -----------------------
            # --- DEBUG LOG --- #
            rospy.logdebug(f"Agent {self.agent.agent_id} TF OK: Laser->Odom @ {lookup_time.to_sec():.3f}")
            # --------------- #
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"Agent {self.agent.agent_id} TF Fail (Scan Time): {e}. Retrying with Time(0)...")
            # --- DEBUG LOG --- #
            rospy.logwarn(f"Agent {self.agent.agent_id} DBG ScanCB TF Fail ScanTime: {e}")
            # --------------- #
            try:
                # If first fails, wait briefly and try again with Time(0) as fallback
                rospy.sleep(0.05) # Wait 50ms
                lookup_time = rospy.Time(0) # Fallback to Time(0)
                # --- Increased Timeout --- #
                transform = self.tf_buffer.lookup_transform(self.odom_frame, self.laser_frame, lookup_time, rospy.Duration(0.2)) # Increased from 0.1
                # -----------------------
                # --- DEBUG LOG --- #
                rospy.logdebug(f"Agent {self.agent.agent_id} TF OK: Laser->Odom @ Time(0)")
                # --------------- #
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e2:
                rospy.logerr_throttle(5.0, f"Agent {self.agent.agent_id} TF Fail (Time(0)): {e2}. Cannot update grid.")
                # --- DEBUG LOG --- #
                rospy.logerr(f"Agent {self.agent.agent_id} DBG ScanCB TF Fail Time(0): {e2}")
                # --------------- #
                return # Cannot proceed without transform

        # Iterate through laser ranges
        num_ranges = len(ranges)
        for i in range(num_ranges):
            r = ranges[i]
            angle = angle_min + i * angle_increment

            # Check for max range (invalid reading, typically)
            is_max_range = (r >= self.agent.last_scan_range_max * 0.99) # Use slightly less than max
            is_valid_range = (r > self.agent.last_scan_range_min and not is_max_range)

            # Calculate the point in the laser frame
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)

            # Transform the point to the odom frame using the obtained transform
            point_laser = PointStamped()
            point_laser.header.stamp = lookup_time # Use the time corresponding to the transform
            point_laser.header.frame_id = self.laser_frame
            point_laser.point.x = x_laser
            point_laser.point.y = y_laser
            try:
                point_odom = tf2_geometry_msgs.do_transform_point(point_laser, transform)
                # ---> ADD CHECK FOR FINITE VALUES <--- #
                if not np.isfinite(point_odom.point.x) or not np.isfinite(point_odom.point.y):
                    rospy.logwarn_throttle(10.0, f"Agent {self.agent.agent_id} TF result is not finite ({point_odom.point.x}, {point_odom.point.y}). Skipping point {i}.")
                    continue
                # ---> END CHECK <--- #
            except Exception as e_tf_point:
                 rospy.logwarn(f"Agent {self.agent.agent_id} Point TF Fail: {e_tf_point}. Skipping point {i}")
                 continue

            # Convert world coordinates (odom) to grid coordinates
            hit_grid_x, hit_grid_y = self._world_to_grid(point_odom.point.x, point_odom.point.y)

            # Mark the line from agent to hit point using Bresenham's
            # Mark cells as free up to the hit point (or max range)
            # Ensure agent grid coords are valid before marking line
            if 0 <= agent_grid_x < self.grid_dims and 0 <= agent_grid_y < self.grid_dims:
                # Determine the end point for Bresenham's line
                # If it's a max range reading, we trace further out
                if is_max_range:
                    max_range_dist = self.agent.last_scan_range_max + self.cell_size * 2 # Extend beyond max range
                    x_laser_max = max_range_dist * math.cos(angle)
                    y_laser_max = max_range_dist * math.sin(angle)
                    point_laser_max = PointStamped()
                    point_laser_max.header.stamp = lookup_time
                    point_laser_max.header.frame_id = self.laser_frame
                    point_laser_max.point.x = x_laser_max
                    point_laser_max.point.y = y_laser_max
                    try:
                        point_odom_max = tf2_geometry_msgs.do_transform_point(point_laser_max, transform)
                        end_grid_x, end_grid_y = self._world_to_grid(point_odom_max.point.x, point_odom_max.point.y)
                    except Exception as e_tf_max:
                         rospy.logwarn(f"Agent {self.agent.agent_id} Max Range Point TF Fail: {e_tf_max}. Using hit grid coords.")
                         end_grid_x, end_grid_y = hit_grid_x, hit_grid_y
                    # Mark line up to this extended point as free
                    self._mark_line(agent_grid_x, agent_grid_y, end_grid_x, end_grid_y, mark_obstacle=False)
                
                elif is_valid_range:
                    # Mark line up to the hit point as free, and mark the hit point itself as an obstacle
                    self._mark_line(agent_grid_x, agent_grid_y, hit_grid_x, hit_grid_y, mark_obstacle=True)
            else:
                 # Should not happen based on check above, but log if it does
                 rospy.logerr("Agent grid coordinates became invalid during scan processing!")

        # --- Grid Inflation (Optional - Currently disabled by logic) --- #
        # self._inflate_obstacles()

    def _mark_line(self, x0, y0, x1, y1, mark_obstacle=False):
        """Mark cells along a line using Bresenham's algorithm.
           Marks cells as FREE (1). If mark_obstacle is True, the endpoint (x1, y1)
           is marked as OBSTACLE (2) if it's within bounds.
           Modified to clear obstacles back to free if ray passes through.
        """
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy  # error value e_xy
        count = 0 # Limit loop iterations for safety
        max_count = self.grid_dims * 2 # Heuristic limit

        x, y = x0, y0
        while count < max_count:
            # Check if current point (x, y) is the endpoint
            is_endpoint = (x == x1 and y == y1)

            # Check bounds before accessing grid
            if 0 <= x < self.grid_dims and 0 <= y < self.grid_dims:
                current_state = self.grid[x, y]
                
                # If this is the endpoint AND we should mark obstacles:
                if is_endpoint and mark_obstacle:
                    if current_state != GRID_OBSTACLE:
                        rospy.logdebug(f"Marking OBSTACLE at ({x},{y}). Prev state: {current_state}")
                        self.grid[x, y] = GRID_OBSTACLE
                    break # Stop after marking the obstacle endpoint
                else:
                    # Mark intermediate cells or non-obstacle endpoints as FREE
                    # Only mark if currently UNKNOWN (0) or OBSTACLE (2)
                    # Do NOT overwrite INFLATED (3) cells back to FREE
                    if current_state == GRID_UNKNOWN or current_state == GRID_OBSTACLE:
                         rospy.logdebug(f"Marking FREE at ({x},{y}). Prev state: {current_state}")
                         self.grid[x, y] = GRID_FREE

            # If we reached the endpoint (and didn't mark it as obstacle), break
            if is_endpoint:
                break

            # Bresenham's algorithm steps
            e2 = 2 * err
            if e2 >= dy: # e_xy+e_x > 0
                err += dy
                x += sx
            if e2 <= dx: # e_xy+e_y < 0
                err += dx
                y += sy
            count += 1
        
        if count >= max_count:
            rospy.logwarn("_mark_line loop limit reached! Potential infinite loop detected.")

    # --- Obstacle Inflation (Keep function, but not called by default) --- #
    def _inflate_obstacles(self):
        """Inflate obstacles in the grid map."""
        if self.inflation_radius_cells <= 0:
            return

        rows, cols = self.grid.shape
        inflated_grid = self.grid.copy()
        obstacle_indices = np.argwhere(self.grid == GRID_OBSTACLE)

        for r_obs, c_obs in obstacle_indices:
            for r_inflate in range(-self.inflation_radius_cells, self.inflation_radius_cells + 1):
                for c_inflate in range(-self.inflation_radius_cells, self.inflation_radius_cells + 1):
                    # Check Euclidean distance for circular inflation
                    if r_inflate**2 + c_inflate**2 <= self.inflation_radius_cells**2:
                        r, c = r_obs + r_inflate, c_obs + c_inflate
                        # Check bounds and if the cell isn't already an obstacle
                        if 0 <= r < rows and 0 <= c < cols and self.grid[r, c] != GRID_OBSTACLE:
                            inflated_grid[r, c] = GRID_INFLATED # Mark as inflated

        self.grid = inflated_grid

    # --- World <-> Grid Conversion --- #
    def _world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid indices."""
        grid_x = int((world_x - self.grid_origin_offset[0]) / self.cell_size)
        grid_y = int((world_y - self.grid_origin_offset[1]) / self.cell_size)
        return grid_x, grid_y

    def _grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates (center of the cell)."""
        world_x = self.grid_origin_offset[0] + (grid_x + 0.5) * self.cell_size
        world_y = self.grid_origin_offset[1] + (grid_y + 0.5) * self.cell_size
        return np.array([world_x, world_y])

    # --- Frontier Detection and Handling --- #
    def _detect_local_frontiers(self):
        """Detect frontiers in the local grid map."""
        frontier_clusters = []
        visited_frontier_cells = set()
        rows, cols = self.grid.shape

        for r in range(rows):
            for c in range(cols):
                # Check if current cell is free and not yet part of a cluster
                if self.grid[r, c] == GRID_FREE and (r, c) not in visited_frontier_cells:
                    is_frontier_cell = False
                    # Check 4-connectivity neighbors for unknown cells
                    for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols and self.grid[nr, nc] == GRID_UNKNOWN:
                            is_frontier_cell = True
                            break
                    
                    # If it's a frontier cell, start BFS to find the cluster
                    if is_frontier_cell:
                        current_cluster = []
                        q = collections.deque([(r, c)])
                        cluster_visited = {(r,c)}
                        visited_frontier_cells.add((r,c)) # Mark as visited for overall detection

                        while q:
                            curr_r, curr_c = q.popleft()
                            
                            # Check if this cell is indeed a frontier cell itself
                            is_curr_frontier = False
                            for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                                 nr, nc = curr_r + dr, curr_c + dc
                                 if 0 <= nr < rows and 0 <= nc < cols and self.grid[nr, nc] == GRID_UNKNOWN:
                                     is_curr_frontier = True
                                     break
                            
                            # Only add actual frontier cells to the cluster
                            if is_curr_frontier:
                                current_cluster.append((curr_r, curr_c))

                                # Find neighboring FREE cells that are also frontiers for clustering
                                # Use 8-connectivity for clustering frontiers
                                for dr_c, dc_c in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                                    nr_c, nc_c = curr_r + dr_c, curr_c + dc_c
                                    # Check bounds, ensure it's FREE, and not visited in this cluster search
                                    if (0 <= nr_c < rows and 0 <= nc_c < cols and 
                                        self.grid[nr_c, nc_c] == GRID_FREE and 
                                        (nr_c, nc_c) not in cluster_visited):
                                        
                                        # Check if this neighbor is ALSO a frontier cell (borders unknown)
                                        is_neighbor_frontier = False
                                        for dr_n, dc_n in [(0,1), (0,-1), (1,0), (-1,0)]:
                                            nr_n, nc_n = nr_c + dr_n, nc_c + dc_n
                                            if 0 <= nr_n < rows and 0 <= nc_n < cols and self.grid[nr_n, nc_n] == GRID_UNKNOWN:
                                                is_neighbor_frontier = True
                                                break
                                        
                                        if is_neighbor_frontier:
                                             q.append((nr_c, nc_c))
                                             cluster_visited.add((nr_c, nc_c))
                                             visited_frontier_cells.add((nr_c, nc_c)) # Mark globally visited

                        if current_cluster: # Only add non-empty clusters
                             frontier_clusters.append(current_cluster)
        
        # Process clusters into frontier data dictionaries
        new_frontiers = []
        MIN_FRONTIER_SIZE = 3 # Minimum number of cells to be considered a valid frontier
        for cluster in frontier_clusters:
            if len(cluster) >= MIN_FRONTIER_SIZE:
                centroid_grid = np.mean(cluster, axis=0).round().astype(int)
                # centroid_world = self._grid_to_world(centroid_grid[0], centroid_grid[1]) # Call grid_to_world
                try:
                    centroid_world_val = self._grid_to_world(centroid_grid[0], centroid_grid[1])
                except Exception as e_gw:
                     rospy.logerr(f"Agent {self.agent.agent_id}: Error in _grid_to_world({centroid_grid[0]}, {centroid_grid[1]}): {e_gw}")
                     centroid_world_val = None # Handle potential errors in conversion

                # Ensure centroid_world is stored as a list for JSON compatibility
                if isinstance(centroid_world_val, np.ndarray):
                    centroid_world_list = centroid_world_val.tolist()
                elif isinstance(centroid_world_val, (list, tuple)) and len(centroid_world_val) >= 2:
                    # If it's already a list or tuple (like the error suggested), ensure it's a list
                    centroid_world_list = list(centroid_world_val)[:2] # Take first 2 elements as list
                else:
                    # Log an error if it's None or an unexpected type/format
                    rospy.logerr(f"Agent {self.agent.agent_id} FrontierDetect: _grid_to_world returned invalid value {centroid_world_val} (type: {type(centroid_world_val)}) for grid {centroid_grid}. Using default [0,0].")
                    centroid_world_list = [0.0, 0.0] # Default value

                # Create a unique ID based on centroid world coordinates
                # Use the safe list version for ID generation
                frontier_id = f"fx_{centroid_world_list[0]:.1f}_{centroid_world_list[1]:.1f}"
                
                # Create frontier data dictionary
                frontier_data = {
                    'id': frontier_id,
                    'centroid_world': centroid_world_list, # Use the converted list
                    'centroid_grid': centroid_grid.tolist(), # centroid_grid is ndarray
                    'size': len(cluster),
                    'status': 'open', # 'open', 'claimed', 'explored'
                    'claimant_agent_id': -1, # ID of the agent claiming/exploring it
                    'discovered_by': self.agent.agent_id,
                    'last_updated': rospy.Time.now().to_sec()
                }
                new_frontiers.append(frontier_data)

        rospy.logdebug(f"Agent {self.agent.agent_id}: Detected {len(new_frontiers)} new local frontiers.")
        return new_frontiers

    def _publish_frontiers_callback(self, event):
        """Timer callback to detect and publish local frontiers."""
        local_frontiers = self._detect_local_frontiers()
        published_count = 0
        for frontier in local_frontiers:
            fid = frontier['id']
            # Publish only if it's new or significantly changed (e.g., size, maybe status if locally marked explored?)
            # For simplicity, let's republish if known status is different or size changed
            # Or just publish if not 'explored'?
            should_publish = False
            if fid not in self.known_frontiers:
                 should_publish = True
                 # Add to known frontiers immediately
                 self.known_frontiers[fid] = frontier 
            else:
                 # Update existing entry
                 # Only update if status is not 'explored' by someone else
                 # And if our local info seems more up-to-date (e.g., status changes)
                 current_known = self.known_frontiers[fid]
                 if current_known.get('status','open') != 'explored':
                    # Update size, maybe status if we locally determined it changed
                    # Keep claimant info unless we are releasing
                    current_known['size'] = frontier['size'] # Update size
                    current_known['last_updated'] = frontier['last_updated']
                    # Don't overwrite status/claimant unless we are the source of change
                    # (e.g., self._release_claim or self._mark_frontier_explored)
                    should_publish = True # Republish updated info
                 else:
                     # If already marked explored globally, don't republish unless we somehow reopened it
                     pass 
            
            if should_publish:
                self.agent.share_data('frontiers', frontier) # Publish via comm module
                published_count += 1
        
        if published_count > 0:
             rospy.loginfo(f"Agent {self.agent.agent_id}: Published {published_count} new/updated local frontiers.")
        else:
             rospy.logdebug(f"Agent {self.agent.agent_id}: No new/updated local frontiers to publish.")

    def frontier_update_callback(self, data):
        """Callback for receiving frontier updates from other agents."""
        frontier_id = data.get('id')
        sender_id = data.get('agent_id')
        if not frontier_id or sender_id == self.agent.agent_id:
            return

        # Movement parameters (could be tuned differently from Search)
        self.max_linear_speed = 1.2
        self.max_angular_speed = 1.5

        # --- Obstacle Avoidance Params ---
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.5 # Threshold to trigger PF avoidance
        # self.OBSTACLE_WIDTH_THRESHOLD = 0.18 # Not directly used by PF
        # self.AVOIDANCE_TURN_SPEED = 0.5 # Obsolete
        # self.AVOIDANCE_SAFE_LINEAR_SPEED = 0.1 # Obsolete
        # self.AVOIDANCE_TURN_SCALING = 2.5 # Obsolete

        # --- Potential Field Parameters ---
        self.PF_OBSTACLE_INFLUENCE_RADIUS = 1.0 # meters
        self.PF_REPULSION_GAIN = 0.5 # Gain for obstacle repulsion
        self.PF_ATTRACTION_GAIN = 0.2 # Gain for default forward attraction
        self.PF_DAMPING_FACTOR = 0.8 # Damping factor for velocity smoothing (0=no damping, 1=immediate change)
        self.PF_AVOID_DURATION = rospy.Duration(3.0) # Default duration to stay in avoiding state
        self.PF_K_LINEAR = 0.5 # Proportional gain for linear velocity from force
        self.PF_K_ANGULAR = 1.5 # Proportional gain for angular velocity from force angle

        # --- Recovery Jiggle Parameters ---
        self.RECOVERY_BACKWARD_SPEED = -0.1 # m/s
        self.RECOVERY_BACKWARD_DURATION = rospy.Duration(1.5) # s
        self.RECOVERY_TURN_SPEED = 0.5 # rad/s
        self.RECOVERY_TURN_DURATION = rospy.Duration(2.0) # s
        self.RECOVERY_FORWARD_SPEED = 0.1 # m/s
        self.RECOVERY_FORWARD_DURATION = rospy.Duration(1.0) # s

        # --- Avoidance/Recovery State ---
        self.avoidance_state = 'none' # 'none', 'avoiding', 'recovery_jiggle'
        # self.escape_target_angle = None # Obsolete
        # self.escape_move_start_time = None # Obsolete
        self.stuck_start_time = None # Timestamp when avoidance/recovery was first triggered (for timeout)
        self.pf_avoid_start_time = None # Timestamp when PF avoidance started
        self.recovery_state = None # Sub-state for jiggle: 'backward', 'turning', 'forward'
        self.recovery_state_start_time = None # Timestamp for current recovery sub-state
        self.recovery_target_angle = None # Target yaw for recovery turn
        self.prev_pf_linear_x = 0.0 # Previous PF linear velocity for damping
        self.prev_pf_angular_z = 0.0 # Previous PF angular velocity for damping


        # --- Stuck Timeout --- #
        self.stuck_timeout = rospy.Duration(15.0) # Max time allowed in avoidance/recovery states before retry

        # --- Old SAM Parameters (To be removed) --- #
        # self.escape_turn_speed = 0.6
        # self.escape_angle_tolerance = np.deg2rad(10)
        # self.escape_move_speed = 0.1
        # self.escape_move_duration = rospy.Duration(3.5)
        # self.avoidance_cooldown_speed = 0.1
        # self.avoidance_cooldown_duration = rospy.Duration(0.75)
        # self.escape_min_clearance = 0.6
        # self.escape_min_gap_width_rad = np.deg2rad(20)
        # self.escape_fallback_angle_bias_rad = np.deg2rad(15)

        # --- Termination State --- #
        self.exploration_complete = False
        self.last_activity_time = None 
        self.no_frontiers_timeout = rospy.Duration(15.0) # Stop after 15s of no open frontiers
        self.path_planning_failures = {} # Track consecutive A* failures per frontier
        self.MAX_PATH_FAILURES = 3 # Release claim after this many consecutive failures

        # --- Path Stuck Detection --- #
        self.last_path_progress_time = None
        self.last_path_position = None
        self.PATH_STUCK_TIMEOUT = rospy.Duration(7.0) # Timeout if no progress on path
        self.MIN_PROGRESS_DIST = self.cell_size * 0.5 # Min distance moved to count as progress

        # TF listener for transforming points
        # --- Increased Cache Time --- #
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0)) # Increased from 30.0
        # -------------------------- #
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.odom_frame = f"{self.agent.agent_name}/odom" # Correct odom frame name
        self.laser_frame = f"{self.agent.agent_name}/laser" # Correct laser frame name

        # Register callback for frontier updates from other agents
        # This relies on the modified SwarmCommunication class
        self.agent.comm.register_callback('frontiers', self.frontier_update_callback)

        # --- Task Allocator --- #
        self.allocator = TaskAllocator(self.agent, self)
        self.agent.comm.register_bid_callback(self.allocator.record_bid)
        # ---------------------- #

        # --- Grid Map Publisher --- #
        self.grid_map_pub = rospy.Publisher(f'/agent_{self.agent.agent_id}/explore_grid', OccupancyGrid, queue_size=1, latch=True)
        # --- Path Publisher --- #
        self.path_pub = rospy.Publisher(f'/agent_{self.agent.agent_id}/explore_path', Path, queue_size=1)
        # ------------------------ #
        rospy.loginfo(f"Agent {self.agent.agent_id}: ExploreBehaviour initialized and registered frontier callback.")

        self.last_pos_update_time = rospy.Time.now()

        # Avoidance state machine parameters
        self.escape_target_yaw = None
        self.avoidance_timer = rospy.Time.now()
        self.stuck_timer_start = None

    def frontier_update_callback(self, data):
        """Callback triggered when frontier information is received."""
        try:
            # Data is already a dictionary thanks to SwarmCommunication
            frontier_id = data.get('frontier_id')
            sender_id = data.get('agent_id') # ID of agent sending the update

            if not frontier_id:
                rospy.logwarn_throttle(10, f"Agent {self.agent.agent_id}: Received frontier update without ID from {sender_id}.")
                return

            # TODO: Add logic to handle stale data (e.g., based on timestamps if added)

            # Update or add the frontier information
            # Ensure position is stored as list/tuple, not numpy array if it comes from detection
            if isinstance(data.get('position'), np.ndarray):
                 data['position'] = data['position'].tolist()
            self.known_frontiers[frontier_id] = data
            rospy.logdebug(f"Agent {self.agent.agent_id}: Updated/Added frontier {frontier_id} based on message from {sender_id}. New status: {data.get('status')}")

            # If this update invalidates our path to the current target, clear the path
            if self.current_target_frontier_id == frontier_id and data.get('status') not in ['open', 'claimed']:
                 rospy.loginfo(f"Agent {self.agent.agent_id}: Current target frontier {frontier_id} became invalid (status: {data.get('status')}). Clearing path.")
                 self.current_target_frontier_id = None
                 self.current_path = None
                 # Target ID is cleared later in compute if needed

            # If this update affects a cell ON our current path, maybe replan?
            # TODO: More sophisticated path invalidation based on grid changes

            # If a new open frontier is received, reset inactivity timer
            if data.get('status') == 'open':
                self.last_activity_time = rospy.Time.now()
                self.exploration_complete = False # Potentially new work

        except Exception as e:
            rospy.logerr(f"Agent {self.agent.agent_id}: Error processing frontier update: {e}")

    def compute(self):
        """Compute movement commands for exploration."""
        # === Initial State Debug Logging START ===
        try:
            agent_grid_x, agent_grid_y = self._world_to_grid(self.agent.position[0], self.agent.position[1])
            current_cell_state = self.grid[agent_grid_x, agent_grid_y]
            rospy.logdebug(f"Agent {self.agent.agent_id} Compute Start: Pos=({self.agent.position[0]:.2f},{self.agent.position[1]:.2f}), Grid=({agent_grid_x},{agent_grid_y}), GridState={current_cell_state}")
        except Exception as e:
            rospy.logwarn(f"Agent {self.agent.agent_id} Compute Start: Error getting initial grid state: {e}")
        # === Initial State Debug Logging END ===
            
        # If already marked as complete, just stop
        # Use a flag to signal immediate stop requested by timeout logic
        stop_exploration_requested = False

        # --- Check for Exploration Completion Timeout (Only if activity has started) ---
        if self.last_activity_time is not None and (rospy.Time.now() - self.last_activity_time > self.no_frontiers_timeout):
            # Check if there are *actually* any known 'open' frontiers left
            has_open_frontiers = any(f_data.get('status') == 'open' for f_data in self.known_frontiers.values())
            if not has_open_frontiers:
                rospy.loginfo(f"Agent {self.agent.agent_id}: No activity/open frontiers for {self.no_frontiers_timeout.to_sec():.1f}s. Marking exploration complete.")
                self.exploration_complete = True
                self._publish_current_path() # Publish empty path
                stop_exploration_requested = True # Signal to stop this cycle
            else:
                # Timeout exceeded, but we still know of open frontiers (maybe unreachable/unwinnable bids)
                # Reset timer to prevent immediate re-triggering if we just received an update
                rospy.logdebug_throttle(5.0, f"Agent {self.agent.agent_id}: Inactivity timeout exceeded, but open frontiers still known. Resetting activity timer.")
                self.last_activity_time = rospy.Time.now()
        # --- End Exploration Completion Timeout Check ---

        # Check completion flags
        if self.exploration_complete or stop_exploration_requested:
            rospy.loginfo_throttle(10.0, f"Agent {self.agent.agent_id}: Exploration previously marked complete. Stopping.")
            return 0.0, 0.0
            
        final_linear_x, final_angular_z = 0.0, 0.0 # Default velocities

        # --- Avoidance State Machine ---
        if self.avoidance_state == 'none':
            # --- Normal Operation Logic --- #
            # (Obstacle detection happens first within this block)

            # --- Reactive Obstacle Avoidance (Direct Laser Scan Check) --- #
            obstacle_imminent = False
            triggering_range = float('inf')
            triggering_angle = 0.0
            # Define the forward arc to check (e.g., +/- 45 degrees)
            forward_arc_rad = np.deg2rad(45.0)
            ranges = self.agent.last_scan_ranges
            angle_min = self.agent.last_scan_angle_min
            angle_increment = self.agent.last_scan_angle_increment
            num_ranges = len(ranges) if ranges else 0

            if num_ranges > 0:
                # --- DEBUG LOG ---
                rospy.logdebug(f"Agent {self.agent.agent_id} DBG AvoidCheck: Entering direct laser check. Ranges={num_ranges}")
                # ---------------
                for i, r in enumerate(ranges):
                    angle = angle_min + i * angle_increment
                    # Check if the angle is within the forward arc
                    if abs(angle) < forward_arc_rad:
                        # Check if the range is below the distance threshold
                        if r < self.OBSTACLE_DISTANCE_THRESHOLD:
                            obstacle_imminent = True
                            if r < triggering_range: # Find the closest obstacle within the arc
                                triggering_range = r
                                triggering_angle = angle
                                # --- DEBUG LOG ---
                                rospy.logdebug(f"Agent {self.agent.agent_id} DBG AvoidCheck: Imminent obstacle candidate: range {r:.2f} < {self.OBSTACLE_DISTANCE_THRESHOLD:.2f} at angle {np.rad2deg(angle):.1f} deg (Idx {i})")
                                # ---------------
                            # Optimization: Can break early if needed, but checking all might be safer
                            # break # Optional: break on first detection

                if obstacle_imminent:
                    # --- DEBUG LOG ---
                    rospy.loginfo(f"Agent {self.agent.agent_id} DBG AvoidCheck: IMMINENT OBSTACLE DETECTED! Closest: {triggering_range:.2f}m @ {np.rad2deg(triggering_angle):.1f} deg. Transitioning to 'avoiding'.")
                    # ---------------
                    # --- Transition to Potential Field Avoidance ---
                    self.avoidance_state = 'avoiding'
                    self.stuck_start_time = rospy.Time.now() # Start overall stuck timer
                    self.pf_avoid_start_time = rospy.Time.now() # Start PF duration timer
                    final_linear_x, final_angular_z = 0.0, 0.0 # STOP immediately for this cycle
                    # --------------------------------------------
                    
                    # Clear path and target
                    rospy.loginfo(f"Agent {self.agent.agent_id}: Obstacle detected! Clearing current path and target.")
                    self.current_target_frontier_id = None
                    self.current_path = None
                    self._publish_current_path() # Update RViz
                else:
                    # Log if scan was processed but no imminent obstacle found
                    rospy.logdebug_throttle(10.0, f"Agent {self.agent.agent_id} AvoidCheck: Scan checked, no imminent obstacles in forward {np.rad2deg(forward_arc_rad):.0f} deg arc.")
                    # --- DEBUG LOG ---
                    if not obstacle_imminent:
                         rospy.logdebug_throttle(10.0, f"Agent {self.agent.agent_id} DBG AvoidCheck: No imminent obstacle detected in forward arc check.")
                    # ---------------
            else:
                 rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id} AvoidCheck: No laser scan ranges available for check.")
            # --- End of Direct Laser Scan Check ---

            # --- Normal Operation (Grid Update, Planning, Following) --- #
            # Only proceed if no obstacle was detected *this cycle*
            if not obstacle_imminent:
                self._update_grid()
                # *** First meaningful activity can start the timeout clock ***
                if self.last_activity_time is None:
                    self.last_activity_time = rospy.Time.now() 

                self._publish_local_frontiers(self._detect_local_frontiers_from_grid())

                replan_needed = self.current_path is None
                # Path completion / advance index
                if self.current_path:
                    # Reset activity timer ONLY if progress is made
                    current_pos = self.agent.position[:2]
                    if self.last_path_position is None: # Initialize first time we have a path
                        self.last_path_position = current_pos
                        self.last_path_progress_time = rospy.Time.now()
                    else:
                        dist_moved = np.linalg.norm(current_pos - self.last_path_position)
                        if dist_moved >= self.MIN_PROGRESS_DIST:
                            rospy.logdebug(f"Agent {self.agent.agent_id}: Path progress detected ({dist_moved:.2f}m). Resetting timers.")
                            self.last_activity_time = rospy.Time.now() # Update overall activity timer
                            self.last_path_progress_time = rospy.Time.now() # Update path stuck timer
                            self.last_path_position = current_pos
                        # else: Keep timers running if no significant progress

                    # TODO: Path validation check against grid map?
                    if self.current_path_index < len(self.current_path):
                        current_waypoint_world = self._grid_to_world(*self.current_path[self.current_path_index])
                        # Reduced waypoint arrival threshold
                        if np.linalg.norm(current_waypoint_world - self.agent.position[:2]) < self.cell_size * 0.4: # Reduced from 0.7
                            self.current_path_index += 1
                            if self.current_path_index >= len(self.current_path):
                                rospy.loginfo(f"Agent {self.agent.agent_id}: Reached end of path for frontier {self.current_target_frontier_id}.")
                                # --- Update grid BEFORE marking explored ---
                                self._update_grid() 
                                # ------------------------------------------
                                self._mark_frontier_explored(self.current_target_frontier_id)
                                self.current_path = None
                                self.current_target_frontier_id = None
                                replan_needed = True
                                self._publish_current_path() # Publish empty path
                                # Reset path stuck detection state on path completion
                                self.last_path_progress_time = None 
                                self.last_path_position = None 
                    else:
                        # This case means index is out of bounds, path is invalid
                        rospy.logwarn(f"Agent {self.agent.agent_id}: Path index became invalid. Clearing path.")
                        self.current_path = None; replan_needed = True
                        self._publish_current_path() # Publish empty path
                        # Reset path stuck detection state
                        self.last_path_progress_time = None 
                        self.last_path_position = None 

                # Select new target and plan path if needed
                if replan_needed:
                    # Filter known frontiers to find candidates for allocation
                    candidate_frontiers = {
                        f_id: f_data for f_id, f_data in self.known_frontiers.items()
                        if f_data.get('status') == 'open'
                    }

                    rospy.logdebug(f"Agent {self.agent.agent_id}: Found {len(candidate_frontiers)} open frontiers for allocation.")

                    selected_frontier_id, planned_path = self.allocator.allocate_task(candidate_frontiers)

                    if selected_frontier_id:
                        # Task successfully allocated by the TaskAllocator (claim was published internally)
                        rospy.loginfo(f"Agent {self.agent.agent_id}: Task {selected_frontier_id} allocated via bidding.")
                        self.last_activity_time = rospy.Time.now() # Found a target
                        self.current_target_frontier_id = selected_frontier_id
                        self.path_planning_failures.pop(selected_frontier_id, None) # Reset failure count
                        
                        if planned_path:
                            self.current_path = planned_path
                            self.current_path_index = 1 # Start from the second point
                            self._publish_current_path() # Publish new path
                        else:
                            # This case should ideally not happen if allocator returns ID only if path exists
                            # But handle defensively
                            rospy.logerr(f"Agent {self.agent.agent_id}: Allocator returned target {selected_frontier_id} but no path! Releasing claim.")
                            self._release_claim(selected_frontier_id)
                            self.current_target_frontier_id = None
                            self.current_path = None
                            self._publish_current_path()

                    else:
                        # No task allocated this round (either no candidates, or lost all bids)
                        rospy.logdebug_throttle(5.0, f"Agent {self.agent.agent_id}: No task allocated this cycle.")
                        # --- REMOVED INCORRECT COMPLETION LOGIC --- #
                        # Check if exploration is complete.
                        # rospy.logdebug_throttle(5.0, f"Agent {self.agent.agent_id}: No open frontiers found for {self.no_frontiers_timeout.to_sec()}s. Marking exploration complete.")
                        # self.exploration_complete = True
                        # final_linear_x, final_angular_z = 0.0, 0.0 # Stop
                        # self._publish_current_path() # Publish empty path
                        # -------------------------------------------- #

                # Movement Execution
                if self.current_path and self.current_path_index < len(self.current_path):
                    next_waypoint_grid = self.current_path[self.current_path_index]
                    final_linear_x, final_angular_z = self._move_to_waypoint(next_waypoint_grid)
                    # Initialize path stuck timers if we just got a new path
                    if self.last_path_progress_time is None:
                        self.last_path_position = self.agent.position[:2]
                        self.last_path_progress_time = rospy.Time.now()
                else:
                    # Stop if no path (either completed, failed planning, or waiting for timeout)
                    final_linear_x, final_angular_z = 0.0, 0.0 
        elif self.avoidance_state == 'avoiding':
            rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id}: Avoidance state: Avoiding (PF)")
            
            # Check stuck timeout (overall timeout for being in avoidance/recovery)
            if self.stuck_start_time and (rospy.Time.now() - self.stuck_start_time > self.stuck_timeout):
                rospy.logwarn(f"Agent {self.agent.agent_id}: Stuck in PF avoidance for > {self.stuck_timeout.to_sec()}s. Triggering recovery jiggle.")
                self._handle_stuck_recovery()
                final_linear_x, final_angular_z = 0.0, 0.0 # Stop this cycle
            # Check PF duration timeout
            elif self.pf_avoid_start_time and (rospy.Time.now() - self.pf_avoid_start_time > self.PF_AVOID_DURATION):
                rospy.loginfo(f"Agent {self.agent.agent_id}: PF avoidance duration ({self.PF_AVOID_DURATION.to_sec()}s) complete. Transitioning back to 'none'.")
                self.avoidance_state = 'none'
                self.pf_avoid_start_time = None
                self.stuck_start_time = None # Reset overall stuck timer as well
                self.prev_pf_linear_x = 0.0 # Reset damping state
                self.prev_pf_angular_z = 0.0
                final_linear_x, final_angular_z = 0.0, 0.0 # Stop this cycle
            else:
                # Calculate and apply potential field velocity
                pf_linear_x, pf_angular_z = self._calculate_potential_field_velocity()
                
                # Apply damping
                final_linear_x = (1 - self.PF_DAMPING_FACTOR) * self.prev_pf_linear_x + self.PF_DAMPING_FACTOR * pf_linear_x
                final_angular_z = (1 - self.PF_DAMPING_FACTOR) * self.prev_pf_angular_z + self.PF_DAMPING_FACTOR * pf_angular_z
                
                # Update previous velocities for next damping calculation
                self.prev_pf_linear_x = final_linear_x
                self.prev_pf_angular_z = final_angular_z
                
                rospy.logdebug_throttle(0.5, f"Agent {self.agent.agent_id}: PF Cmd: Raw=({pf_linear_x:.2f}, {pf_angular_z:.2f}), Damped=({final_linear_x:.2f}, {final_angular_z:.2f})")
        
        # --- Recovery Jiggle State ---
        elif self.avoidance_state == 'recovery_jiggle':
             rospy.logdebug_throttle(0.5, f"Agent {self.agent.agent_id}: Avoidance state: Recovery Jiggle (Sub-state: {self.recovery_state})")
             # The recovery jiggle logic handles its own state transitions and velocity commands
             final_linear_x, final_angular_z = self._execute_recovery_jiggle()
        # --- End of New States ---

        # --- Moved Stuck Check Here (to apply regardless of state if timeout reached) --- #
        # Check stuck timeout (this applies even during normal operation if inactive)
        if self.last_activity_time and (rospy.Time.now() - self.last_activity_time > self.stuck_timeout):
             rospy.logwarn(f"Agent {self.agent.agent_id}: Stuck timeout due to inactivity! Initiating recovery.")
             if self.current_target_frontier_id:
                 self._release_claim(self.current_target_frontier_id)
             self._handle_stuck_recovery()
             final_linear_x, final_angular_z = 0.0, 0.0
        # --------------------------------------------------------------------------------- #

        # --- Path Planning Failure Handling (Check moved outside normal state logic) --- #
        # Only check for repeated path failures if we are NOT currently avoiding obstacles
        if self.avoidance_state == 'none' and self.current_target_frontier_id and self.current_path is None:
            failure_count = self.path_planning_failures.get(self.current_target_frontier_id, 0)
            if failure_count >= self.MAX_PATH_FAILURES:
                 rospy.logwarn(f"Agent {self.agent.agent_id}: Path planning failed {failure_count} times for {self.current_target_frontier_id}. Releasing claim.")
                 self._release_claim(self.current_target_frontier_id)
                 # No need to immediately replan here, next compute cycle will trigger allocation
            # Increment failure count handled where planning is attempted (inside allocator? or if allocator returns None?)
            # Let's assume allocator returns None if path fails, increment here:
            elif self.current_target_frontier_id: # Increment only if we had a target but failed to get path/allocation
                 self.path_planning_failures[self.current_target_frontier_id] = failure_count + 1
                 rospy.logdebug(f"Agent {self.agent.agent_id}: Path planning/allocation failed for {self.current_target_frontier_id} (Attempt {failure_count + 1}/{self.MAX_PATH_FAILURES})")
        # --- End Path Planning Failure --- #

        # --- Return final velocities ---
        return final_linear_x, final_angular_z # Ensure final values are returned

    # --- Potential Field Calculation ---
    def _calculate_potential_field_velocity(self, attraction_vector=None):
        """Calculates desired velocity based on repulsive forces from laser scan and a default attractive force."""
        ranges = self.agent.last_scan_ranges
        if not ranges:
            rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id}: No laser scan data available for potential field calculation.")
            return 0.0, 0.0

        angle_min = self.agent.last_scan_angle_min
        angle_increment = self.agent.last_scan_angle_increment
        num_ranges = len(ranges)

        # Initialize forces (attractive force pulling straight forward)
        total_force_x = self.PF_ATTRACTION_GAIN
        total_force_y = 0.0
        
        processed_points = 0
        # Calculate repulsive forces from obstacles
        for i, r in enumerate(ranges):
            # Ignore invalid ranges (inf, nan, or <=0)
            if not np.isfinite(r) or r <= 0:
                continue 
                
            # Consider only points within the influence radius
            if r < self.PF_OBSTACLE_INFLUENCE_RADIUS:
                processed_points += 1
                angle = angle_min + i * angle_increment
                
                # Calculate repulsive force (inverse square law)
                # Add a small epsilon to avoid division by zero if r is extremely small
                magnitude = self.PF_REPULSION_GAIN / (r**2 + 1e-6)
                
                # Add force components (repulsion pushes away from the obstacle)
                total_force_x -= magnitude * np.cos(angle)
                total_force_y -= magnitude * np.sin(angle)

        rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id} PF Calc: Processed {processed_points} scan points. Total Force=({total_force_x:.2f}, {total_force_y:.2f})")

        # Calculate desired angle from the resultant force vector
        # Angle is relative to the robot's current forward direction (X-axis)
        desired_angle = np.arctan2(total_force_y, total_force_x)
        angle_error = self._normalize_angle(desired_angle) # Error is simply the desired angle relative to forward

        # Calculate target angular velocity based on angle error
        target_angular_z = np.clip(angle_error * self.PF_K_ANGULAR, -self.max_angular_speed, self.max_angular_speed)

        # Calculate target linear velocity based on the magnitude of the force in the forward direction
        # We use total_force_x as a proxy for the net forward push/pull
        # Clamp linear velocity to be non-negative (or slightly negative if needed? No, PF should push away)
        target_linear_x = np.clip(total_force_x * self.PF_K_LINEAR, 0, self.max_linear_speed)

        # --- Optional: Reduce linear speed when turning sharply --- # 
        # max_turn_reduction = 0.7 # e.g., reduce speed by up to 70% for 180 deg turn error
        # turn_reduction_factor = max(0.1, 1.0 - max_turn_reduction * (abs(angle_error) / np.pi))
        # target_linear_x *= turn_reduction_factor
        # --------------------------------------------------------- #

        rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id} PF Calc: Desired Angle={np.rad2deg(desired_angle):.1f} deg, TargetVel=({target_linear_x:.2f}, {target_angular_z:.2f})")

        # Return raw target velocities (damping is applied in compute method)
        return target_linear_x, target_angular_z

    # --- Grid Map Methods (Copied/Adapted from SearchBehaviour) --- #
    def _world_to_grid(self, wx, wy):
        """Convert world coordinates (odom frame) to grid indices."""
        grid_x = int((wx + self.map_size / 2) / self.cell_size)
        grid_y = int((wy + self.map_size / 2) / self.cell_size)
        # Clip to grid bounds
        grid_x = np.clip(grid_x, 0, self.grid_dims - 1)
        grid_y = np.clip(grid_y, 0, self.grid_dims - 1)
        return grid_x, grid_y

    def _grid_to_world(self, grid_x, grid_y):
        """Convert grid indices to world coordinates (center of cell)."""
        world_x = grid_x * self.cell_size - self.map_size / 2 + self.cell_size / 2
        world_y = grid_y * self.cell_size - self.map_size / 2 + self.cell_size / 2
        return np.array([world_x, world_y])

    def _get_euler_from_quaternion(self):
        """Helper to get yaw from agent's orientation."""
        return tf.transformations.euler_from_quaternion(self.agent.orientation)

    def _get_angular_difference(self, current, desired):
        """Calculate the shortest angular difference between two angles in radians."""
        diff = desired - current
        while diff > np.pi: diff -= 2 * np.pi
        while diff < -np.pi: diff += 2 * np.pi
        return diff

    def _update_grid(self):
        """Update the grid based on current observations."""
        agent_grid_x, agent_grid_y = self._world_to_grid(self.agent.position[0], self.agent.position[1])
        self.grid[agent_grid_x, agent_grid_y] = 1
        ranges = self.agent.last_scan_ranges; scan_time = self.agent.last_scan_time
        if not ranges: return
        max_range = self.agent.last_scan_range_max; min_range = self.agent.last_scan_range_min
        angle_min = self.agent.last_scan_angle_min; angle_increment = self.agent.last_scan_angle_increment
        
        # --- Use the actual scan time for TF lookups --- #
        lookup_time = scan_time if scan_time else rospy.Time.now() # Use scan time if available
        if not scan_time:
            rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id}: Scan message missing timestamp! Using current time for TF lookup.")
        # ------------------------------------------------- #
        
        transform = None
        try:
            # First attempt to lookup transform using scan time
            # --- Increased Timeout --- #
            transform = self.tf_buffer.lookup_transform(self.odom_frame, self.laser_frame, lookup_time, rospy.Duration(0.2)) # Increased from 0.1
            # -----------------------
            # --- DEBUG LOG --- #
            # Log transform details if successful
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            rz = tf.transformations.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])[2]
            rospy.logdebug(f"Agent {self.agent.agent_id} DBG GridUpdate TF: Using transform Odom<-Laser @ time {lookup_time.to_sec():.3f}. Trans=({tx:.2f},{ty:.2f}), RotZ={np.rad2deg(rz):.1f}deg")
            # --- End DEBUG --- #
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"Agent {self.agent.agent_id}: TF Lookup failed (1st try - time: {lookup_time.to_sec()}): {e}. Retrying after short delay with Time(0)...")
            try:
                # If first fails, wait briefly and try again with Time(0) as fallback
                rospy.sleep(0.05) # Wait 50ms
                lookup_time = rospy.Time(0) # Fallback to Time(0)
                # --- Increased Timeout --- #
                transform = self.tf_buffer.lookup_transform(self.odom_frame, self.laser_frame, lookup_time, rospy.Duration(0.2)) # Increased from 0.1
                # -----------------------
                # --- DEBUG LOG --- #
                rospy.logdebug(f"Agent {self.agent.agent_id} TF OK: Laser->Odom @ Time(0)")
                # --------------- #
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e2:
                rospy.logerr_throttle(5.0, f"Agent {self.agent.agent_id} TF Fail (Time(0)): {e2}. Cannot update grid.")
                # --- DEBUG LOG --- #
                rospy.logerr(f"Agent {self.agent.agent_id} DBG ScanCB TF Fail Time(0): {e2}")
                # --------------- #
                return # Cannot proceed without transform

        # Iterate through laser ranges
        num_ranges = len(ranges)
        for i in range(num_ranges):
            r = ranges[i]
            angle = angle_min + i * angle_increment

            # Check for max range (invalid reading, typically)
            is_max_range = (r >= self.agent.last_scan_range_max * 0.99) # Use slightly less than max
            is_valid_range = (r > self.agent.last_scan_range_min and not is_max_range)

            # Calculate the point in the laser frame
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)

            # Transform the point to the odom frame using the obtained transform
            point_laser = PointStamped()
            point_laser.header.stamp = lookup_time # Use the time corresponding to the transform
            point_laser.header.frame_id = self.laser_frame
            point_laser.point.x = x_laser
            point_laser.point.y = y_laser
            try:
                point_odom = tf2_geometry_msgs.do_transform_point(point_laser, transform)
                # ---> ADD CHECK FOR FINITE VALUES <--- #
                if not np.isfinite(point_odom.point.x) or not np.isfinite(point_odom.point.y):
                    rospy.logwarn_throttle(10.0, f"Agent {self.agent.agent_id} TF result is not finite ({point_odom.point.x}, {point_odom.point.y}). Skipping point {i}.")
                    continue
                # ---> END CHECK <--- #
            except Exception as e_tf_point:
                 rospy.logwarn(f"Agent {self.agent.agent_id} Point TF Fail: {e_tf_point}. Skipping point {i}")
                 continue

            # Convert world coordinates (odom) to grid coordinates
            hit_grid_x, hit_grid_y = self._world_to_grid(point_odom.point.x, point_odom.point.y)

            # Mark the line from agent to hit point using Bresenham's
            # Mark cells as free up to the hit point (or max range)
            # Ensure agent grid coords are valid before marking line
            if 0 <= agent_grid_x < self.grid_dims and 0 <= agent_grid_y < self.grid_dims:
                # Determine the end point for Bresenham's line
                # If it's a max range reading, we trace further out
                if is_max_range:
                    max_range_dist = self.agent.last_scan_range_max + self.cell_size * 2 # Extend beyond max range
                    x_laser_max = max_range_dist * math.cos(angle)
                    y_laser_max = max_range_dist * math.sin(angle)
                    point_laser_max = PointStamped()
                    point_laser_max.header.stamp = lookup_time
                    point_laser_max.header.frame_id = self.laser_frame
                    point_laser_max.point.x = x_laser_max
                    point_laser_max.point.y = y_laser_max
                    try:
                        point_odom_max = tf2_geometry_msgs.do_transform_point(point_laser_max, transform)
                        end_grid_x, end_grid_y = self._world_to_grid(point_odom_max.point.x, point_odom_max.point.y)
                    except Exception as e_tf_max:
                         rospy.logwarn(f"Agent {self.agent.agent_id} Max Range Point TF Fail: {e_tf_max}. Using hit grid coords.")
                         end_grid_x, end_grid_y = hit_grid_x, hit_grid_y
                    # Mark line up to this extended point as free
                    self._mark_line(agent_grid_x, agent_grid_y, end_grid_x, end_grid_y, mark_obstacle=False)
                
                elif is_valid_range:
                    # Mark line up to the hit point as free, and mark the hit point itself as an obstacle
                    self._mark_line(agent_grid_x, agent_grid_y, hit_grid_x, hit_grid_y, mark_obstacle=True)
            else:
                 # Should not happen based on check above, but log if it does
                 rospy.logerr("Agent grid coordinates became invalid during scan processing!")

        # --- Grid Inflation (Optional - Currently disabled by logic) --- #
        # self._inflate_obstacles()

    def _inflate_obstacles(self, inflation_radius_cells=2):
        """Inflate obstacles on the grid by marking nearby cells with a higher cost."""
        # --- Return early if inflation radius is zero --- #
        if inflation_radius_cells <= 0:
            # rospy.logdebug(f"Agent {self.agent.agent_id}: Obstacle inflation disabled (radius <= 0).")
            return
        # --------------------------------------------- #
        new_inflated_cells = []
        obstacle_value = 2
        inflated_value = 3 # New value for inflated cells

        # Find all current obstacle cells
        obstacle_indices = np.argwhere(self.grid == obstacle_value)

        if obstacle_indices.size == 0: return # No obstacles to inflate

        # Iterate through each obstacle cell
        for r_obs, c_obs in obstacle_indices:
            # Check neighbors within the inflation radius
            for dr in range(-inflation_radius_cells, inflation_radius_cells + 1):
                for dc in range(-inflation_radius_cells, inflation_radius_cells + 1):
                    # Skip self
                    if dr == 0 and dc == 0: continue
                    
                    # Calculate neighbor coordinates
                    nr, nc = r_obs + dr, c_obs + dc

                    # Check bounds
                    if 0 <= nr < self.grid_dims and 0 <= nc < self.grid_dims:
                        # Check if neighbor is NOT already an obstacle
                        if self.grid[nr, nc] != obstacle_value:
                             # Check if it's not already planned to be inflated (avoid duplicates)
                             if self.grid[nr, nc] != inflated_value:
                                 # Add to list to be marked later (avoid modifying grid during iteration)
                                 new_inflated_cells.append((nr, nc))

        # Mark the identified cells as inflated
        inflated_count = 0
        for r_inf, c_inf in new_inflated_cells:
             # Final check before marking (in case grid changed)
             if self.grid[r_inf, c_inf] != obstacle_value:
                self.grid[r_inf, c_inf] = inflated_value
                inflated_count += 1
        
        if inflated_count > 0:
             rospy.logdebug(f"Agent {self.agent.agent_id}: Inflated {inflated_count} cells around obstacles.")

    def _mark_line(self, x0, y0, x1, y1, mark_obstacle=False):
        """Mark cells on line as free (1), optionally handle endpoint."""
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy; x, y = x0, y0
        while True:
            # === DEBUG LOGGING START ===
            log_this_cell = False
            if abs(x0 - x) <= 2 and abs(y0 - y) <= 2: # Log cells near the agent start point of the line
                log_this_cell = True
            # === DEBUG LOGGING END ===
            is_endpoint = (x == x1 and y == y1); mark_this_cell = True
            if is_endpoint and not endpoint_inclusive: mark_this_cell = False
            if mark_this_cell and (0 <= x < self.grid_dims and 0 <= y < self.grid_dims):
                 # --- Allow overwriting Unknown (0) OR Obstacle (2) with Free (1) ---
                 # This corrects for cases where a previous obstacle detection was incorrect/transient
                 if self.grid[x, y] in [0, 2]: 
                    if log_this_cell:
                        rospy.logdebug(f"Agent {self.agent.agent_id} DBG MarkLine Free: Marking FREE at grid ({x},{y}) along line from ({x0},{y0}) to ({x1},{y1}). Prev state={self.grid[x, y]}")
                    self.grid[x, y] = 1
                 # --- Keep Inflated (3) as Inflated --- 
                 # elif self.grid[x, y] == 3: 
                 #    pass # Don't overwrite inflated cells with free space from ray tracing
                 # -----------------------------------------------------------------
            if is_endpoint: break
            e2 = 2 * err; 
            if e2 > -dy: err -= dy; x += sx
            if e2 < dx: err += dx; y += sy

    # --- Frontier Detection & Publishing --- #

    def _detect_local_frontiers_from_grid(self):
        """Detect frontier cells where known free space (1) meets unknown (0)."""
        frontier_cells = []
        for r in range(self.grid_dims):
            for c in range(self.grid_dims):
                # Check if current cell is free space
                if self.grid[r, c] == 1:
                    # Check neighbors (4-connectivity)
                    is_frontier = False
                    for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                        nr, nc = r + dr, c + dc
                        # Check bounds and if neighbor is unknown
                        if 0 <= nr < self.grid_dims and 0 <= nc < self.grid_dims:
                            if self.grid[nr, nc] == 0:
                                is_frontier = True
                                break # Found an unknown neighbor, this is a frontier cell
                        else:
                             # Cell on the edge of the grid map is also a frontier if free
                             is_frontier = True
                             break
                             
                    if is_frontier:
                        frontier_cells.append((r, c))

        # Convert grid cells to world coordinates and create dictionaries
        # TODO: Implement clustering of adjacent frontier_cells for better targets
        frontiers = []
        processed_ids = set() # Use simple ID to avoid duplicates from clustering later
        for r, c in frontier_cells:
            world_x, world_y = self._grid_to_world(r, c)
            frontier_pos = [world_x, world_y]
            # Use same ID generation as before
            frontier_id = f"fx_{frontier_pos[0]:.1f}_{frontier_pos[1]:.1f}" 
            
            if frontier_id not in processed_ids:
                frontiers.append({
                    'frontier_id': frontier_id,
                    'position': frontier_pos,
                    'size': 1.0, # Placeholder size, could be cluster size later
                })
                processed_ids.add(frontier_id)

        rospy.logdebug(f"Agent {self.agent.agent_id}: Detected {len(frontiers)} unique frontier points from grid.")
        return frontiers

    def _publish_local_frontiers(self, local_frontiers):
        """Publish newly detected local frontiers if not already known, and add to own knowledge."""
        published_count = 0
        for frontier in local_frontiers:
            f_id = frontier['frontier_id']
            if f_id not in self.known_frontiers:
                frontier_data = frontier.copy()
                frontier_data['status'] = 'open'
                frontier_data['claimant_agent_id'] = -1
                self.agent.share_data('frontiers', frontier_data)
                published_count += 1
                # --- Immediately add to own known_frontiers --- #
                self.known_frontiers[f_id] = frontier_data 
                rospy.logdebug(f"Agent {self.agent.agent_id}: Published and locally added new frontier {f_id}.")

        if published_count > 0:
            rospy.loginfo(f"Agent {self.agent.agent_id}: Published {published_count} new local frontiers.")

    # --- Frontier Selection & Claiming --- #

    def _select_target_frontier(self):
        """Select the best frontier to target based on status and distance."""
        neighbour_states = self.agent.comm.get_neighbour_states()
        my_pos = self.agent.position[:2] # Use only x, y
        min_dist_sq = float('inf')
        best_frontier_id = None
        min_agent_dist_sq = 0.5**2 # Don't pick frontiers too close to other agents

        # Get positions of other agents
        agent_positions = []
        for agent_id, state_data in neighbour_states.items():
            pos = state_data.get('position')
            if pos:
                agent_positions.append(np.array(pos[:2]))

        # Iterate through known frontiers
        utilities = []
        for f_id, f_data in list(self.known_frontiers.items()):
            if f_data.get('status') == 'open':
                f_pos = np.array(f_data.get('position', [0, 0]))
                
                # Check proximity to other agents
                too_close_to_agent = False
                for agent_pos in agent_positions:
                    dist_to_agent_sq = np.sum((f_pos - agent_pos)**2)
                    if dist_to_agent_sq < min_agent_dist_sq:
                        too_close_to_agent = True
                        break
                
                if too_close_to_agent:
                    continue # Skip this frontier

                # --- Add Check: Ensure grid cell for frontier is not an obstacle or inflated --- #
                f_grid_x, f_grid_y = self._world_to_grid(f_pos[0], f_pos[1])
                # Check grid bounds first
                if 0 <= f_grid_x < self.grid_dims and 0 <= f_grid_y < self.grid_dims:
                    grid_state = self.grid[f_grid_x, f_grid_y]
                    # Skip if the target cell itself is an obstacle (2) or inflated (3)
                    if grid_state == 2 or grid_state == 3:
                        rospy.logdebug(f"Agent {self.agent.agent_id}: Skipping frontier {f_id} because target cell ({f_grid_x},{f_grid_y}) is obstacle/inflated (state: {grid_state}).")
                        continue
                else:
                    # Skip if frontier is outside the current grid map bounds
                    rospy.logdebug(f"Agent {self.agent.agent_id}: Skipping frontier {f_id} because target cell ({f_grid_x},{f_grid_y}) is outside grid bounds.")
                    continue
                # -------------------------------------------------------------------------- #

                # Calculate distance from self to frontier
                dist_sq = np.sum((f_pos - my_pos)**2)
                distance = np.sqrt(dist_sq)

                # --- Calculate Utility: Distance + Distance Bonus --- #
                # Simple utility: prioritize distance (closer is better), but add a bonus for being further away
                # This seems counter-intuitive. Let's redefine utility.
                # Higher utility should be better.
                # Option 1: Inverse distance (closer = higher utility) + Small distance bonus (further = slight boost)
                # Option 2: Negative distance (closer = higher utility) + Small distance bonus
                # Option 3: Information Gain - Distance Cost (More complex, needs unknown cell counting)

                # Let's try Option 2: Utility = -Distance + Distance_Bonus_Factor * Distance
                # This prioritizes closeness but gives a slight edge to farther targets.
                # distance_bonus_factor = 0.1 # Tunable parameter (0.1 means 10% bonus for distance)
                # utility = -distance + distance_bonus_factor * distance # Closer is still better overall

                # --- Use Utility: Information Gain - Distance Cost --- #
                potential = 0
                # Simplified check: count unknown neighbors around the frontier cell
                for dx in range(-1, 2): # Check 3x3 area
                    for dy in range(-1, 2):
                        if dx == 0 and dy == 0: continue
                        nx, ny = f_grid_x + dx, f_grid_y + dy
                        if 0 <= nx < self.grid_dims and 0 <= ny < self.grid_dims and self.grid[nx, ny] == 0:
                            potential += 1
                # Weight potential (information gain) positively, distance negatively
                utility = potential * 1.0 - distance * 0.5

                # Store utility, distance, and ID
                # We want to select the MAX utility later
                utilities.append({'id': f_id, 'utility': utility, 'distance': distance})

        if not utilities:
             rospy.logdebug_throttle(5, f"Agent {self.agent.agent_id}: No valid 'open' frontiers found after filtering.")
             return None

        # Select the frontier with the HIGHEST utility
        best_frontier = max(utilities, key=lambda x: x['utility'])
        best_frontier_id = best_frontier['id']
        best_utility = best_frontier['utility']
        best_distance = best_frontier['distance']

        if best_frontier_id:
            rospy.loginfo(f"Agent {self.agent.agent_id}: Selected target frontier {best_frontier_id} with utility {best_utility:.2f} at distance {best_distance:.2f}")
        else:
            # This case should ideally not be reached if utilities list was not empty
            rospy.logwarn_throttle(5, f"Agent {self.agent.agent_id}: Failed to select a best frontier from utility list.")

        return best_frontier_id

    def _claim_target_frontier(self, frontier_id):
        """Publish an update for frontier_id with status='claimed' and claimant_agent_id=self.agent.agent_id"""
        # Check if frontier still exists
        if frontier_id not in self.known_frontiers:
            rospy.logwarn(f"Agent {self.agent.agent_id}: Tried to claim non-existent frontier {frontier_id}")
            return
            
        claim_data = self.known_frontiers[frontier_id].copy() # Get current data
        # Only claim if it's currently open
        if claim_data.get('status') == 'open':
            claim_data['status'] = 'claimed'
            claim_data['claimant_agent_id'] = self.agent.agent_id
            self.agent.share_data('frontiers', claim_data)
            # Update local knowledge immediately
            self.known_frontiers[frontier_id] = claim_data
            rospy.loginfo(f"Agent {self.agent.agent_id}: Published claim for frontier {frontier_id}")
        else:
            rospy.logdebug(f"Agent {self.agent.agent_id}: Frontier {frontier_id} was not 'open' when trying to claim (Status: {claim_data.get('status')}). Claim aborted.")
            # If it was claimed by us previously, maybe just proceed?
            # If claimed by other, select new target (handled in compute)

    def _mark_frontier_explored(self, frontier_id):
        """Mark a frontier as explored and publish the update."""
        if frontier_id in self.known_frontiers:
            self.known_frontiers[frontier_id]['status'] = 'explored'
            self.known_frontiers[frontier_id]['claimant_agent_id'] = -1
            self.agent.share_data('frontiers', self.known_frontiers[frontier_id])
            rospy.loginfo(f"Agent {self.agent.agent_id}: Marked frontier {frontier_id} as explored.")
        else:
            rospy.logwarn(f"Agent {self.agent.agent_id}: Tried to mark non-existent frontier {frontier_id} as explored.")

    # --- Path Planning Methods (Copied/Adapted from SearchBehaviour) --- #
    def _find_path(self, start_grid, goal_grid):
        """Finds a path from start_grid to goal_grid using A* on self.grid.
        
        Returns:
             tuple: (list_of_coords, total_cost) if path found, else (None, float('inf'))
        """
        
        rows, cols = self.grid.shape
        start_node = tuple(start_grid)
        original_goal_node = tuple(goal_grid) # Store the original goal

        # Check if start is valid
        if not (0 <= start_node[0] < rows and 0 <= start_node[1] < cols):
            rospy.logwarn(f"Agent {self.agent.agent_id} DBG A* Fail: Start node {start_node} is outside grid bounds ({rows}x{cols}).")
            return None, float('inf')
        if self.grid[start_node] in [2, 3]: # Cannot start in obstacle/inflated
            rospy.logwarn(f"Agent {self.agent.agent_id} DBG A* Fail: Start node {start_node} is obstacle/inflated (state={self.grid[start_node]}). Pathfinding failed.")
            return None, float('inf')
        
        # Check if original goal is outside grid
        if not (0 <= original_goal_node[0] < rows and 0 <= original_goal_node[1] < cols):
             # --- DEBUG LOG ---
             rospy.logwarn(f"Agent {self.agent.agent_id} DBG A* Fail: Original goal node {original_goal_node} outside grid ({rows}x{cols}).")
             # ---------------
             return None, float('inf')

        # --- Handle Obstacle/Inflated Goal --- #
        goal_node = original_goal_node
        if self.grid[goal_node] in [2, 3]: # If original goal is obstacle or inflated
            rospy.logwarn(f"Agent {self.agent.agent_id} A*: Original goal {goal_node} (state={self.grid[goal_node]}) is obstacle/inflated. Searching for nearest valid & reachable neighbor via BFS.")
            
            # --- Start BFS from original goal to find nearest valid cell --- #
            q = collections.deque([(original_goal_node, 0)]) # (node, distance)
            visited = {original_goal_node}
            nearest_valid_neighbor = None
            min_dist_found = float('inf')
            # --- Increased BFS limit for finding neighbors --- #
            BFS_LIMIT = 500 # Increased limit from 250
            # ------------------------------------------------- #
            bfs_count = 0
 
            # --- Log start of neighbor BFS --- #
            rospy.logdebug(f"Agent {self.agent.agent_id} A* NeighborBFS: Starting search for valid neighbor near obstacle goal {original_goal_node} with limit {BFS_LIMIT}")
            # --------------------------------- #

            while q and bfs_count < BFS_LIMIT:
                current_node, dist = q.popleft()
                bfs_count += 1

                # Check if this node is valid (not obstacle/inflated)
                if self.grid[current_node] not in [2, 3]:
                    # Found the first valid cell during BFS - this is the closest
                    nearest_valid_neighbor = current_node
                    min_dist_found = dist
                    rospy.loginfo(f"Agent {self.agent.agent_id} A*: BFS found nearest valid neighbor {nearest_valid_neighbor} at distance {dist} from original {original_goal_node}.")
                    break # Found the closest one

                # Explore neighbors (using 8-connectivity for broader search)
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        if dx == 0 and dy == 0: continue
                        nx, ny = current_node[0] + dx, current_node[1] + dy
                        neighbor_node = (nx, ny)

                        # Check bounds and if not visited
                        if (0 <= nx < rows and 0 <= ny < cols and neighbor_node not in visited):
                            visited.add(neighbor_node)
                            q.append((neighbor_node, dist + 1))
            # --- End BFS --- #
                            
            if nearest_valid_neighbor:
                # Found a valid neighbor via BFS
                rospy.loginfo(f"Agent {self.agent.agent_id} A*: Redirecting goal from {original_goal_node} to nearest valid neighbor {nearest_valid_neighbor} found via BFS.")
                goal_node = nearest_valid_neighbor # Update the goal for A*
            else:
                # --- Log BFS failure reason --- #
                rospy.logerr(f"Agent {self.agent.agent_id} A* NeighborBFS Fail: Original goal {original_goal_node} is obstacle/inflated, and BFS within limit {BFS_LIMIT} explored {bfs_count} nodes without finding valid neighbors. Pathfinding failed.")
                # ------------------------------ #
                return None, float('inf') # Cannot find any valid target nearby
        # --- End Obstacle Goal Handling --- #

        # Final check: Ensure the determined goal_node is not obstacle/inflated (should be guaranteed by logic above)
        if self.grid[goal_node] in [2, 3]:
             rospy.logerr(f"Agent {self.agent.agent_id} A*: CRITICAL ERROR: Final goal node {goal_node} (state={self.grid[goal_node]}) is obstacle/inflated after check! Pathfinding failed.")
             return None, float('inf')

        # --- Add Reachability Check --- #
        # --- Increased BFS Limit --- #
        if not self._is_reachable(start_node, goal_node, limit=2000): # Increased limit from 1000
        # -------------------------- #
            rospy.logwarn(f"Agent {self.agent.agent_id} A*: Goal {goal_node} determined unreachable from {start_node} via pre-check BFS. Pathfinding aborted.")
            return None, float('inf')
        # --- End Reachability Check --- #

        # A* algorithm implementation follows...
        # --- Use 8-Connectivity --- #
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)] # 8-connectivity

        COST_VISITED = 1.0 # Cost for already free/visited cells (state 1)
        COST_UNKNOWN = 1.5 # Cost for unknown cells (state 0) - encourage exploration slightly
        COST_DIAGONAL_VISITED = 1.414 # Approx sqrt(2)
        COST_DIAGONAL_UNKNOWN = 1.5 * 1.414 # Approx 2.121
        # Obstacle (2) and Inflated (3) cells are now treated as impassable (infinite cost) by skipping them below.

        close_set = set()
        came_from = {}
        gscore = {start_node: 0}
        fscore = {start_node: self._heuristic(start_node, goal_node)}
        
        oheap = []
        heapq.heappush(oheap, (fscore[start_node], start_node))

        path_found = False
        final_cost = float('inf')
        final_path = None

        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == goal_node:
                # Path found
                final_path = self._reconstruct_path(came_from, current, start_node)
                final_cost = gscore[current]
                path_found = True
                rospy.logdebug(f"Agent {self.agent.agent_id} DBG A* Success: Path found from {start_node} to {goal_node} (orig: {original_goal_node}). Cost={final_cost:.2f}, Length={len(final_path)}. Path snippet: {final_path[:3]}...{final_path[-3:]}")
                break # Exit while loop

            close_set.add(current)

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                
                # Check grid bounds
                if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                    continue
                
                # --- Check if neighbor is impassable (obstacle/inflated) ---
                neighbor_state = self.grid[neighbor]
                if neighbor_state in [2, 3]: 
                    continue # Skip obstacle/inflated cells entirely

                # Determine cost to move to this valid neighbor
                # --- Adjust cost for diagonal movement --- #
                is_diagonal = (i != 0 and j != 0)
                cost_to_neighbor = 0
                if neighbor_state == 1: # Free/Visited
                    cost_to_neighbor = COST_DIAGONAL_VISITED if is_diagonal else COST_VISITED
                elif neighbor_state == 0: # Unknown
                    cost_to_neighbor = COST_DIAGONAL_UNKNOWN if is_diagonal else COST_UNKNOWN
                # ---------------------------------------- #
                else:
                    # This case should not be reached due to the check above, but log defensively
                    rospy.logwarn(f"Agent {self.agent.agent_id} A*: Neighbor {neighbor} has unexpected state {neighbor_state}. Skipping.")
                    continue
                
                tentative_g_score = gscore[current] + cost_to_neighbor

                # Check if already processed or if this path is worse
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue

                # This path is better or neighbor not visited yet
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self._heuristic(neighbor, goal_node)
                    # Add neighbor to the open list (priority queue)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        
        # End of while loop
        if not path_found:
            # --- DEBUG LOG ---
            rospy.logwarn(f"Agent {self.agent.agent_id} DBG A* Fail: Pathfinding failed from {start_node} to goal {goal_node} (original: {original_goal_node}). Open heap empty after exploring {len(close_set)} nodes.")
            # ---------------
            return None, float('inf')
        else:
            return final_path, final_cost # Return the found path and its cost

    def _reconstruct_path(self, came_from, current, start_node):
        """Helper to reconstruct path from A* results."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start_node)
        return path[::-1]

    def _heuristic(self, node_a, node_b):
        """Heuristic function for A* (Manhattan distance)."""
        return abs(node_a[0] - node_b[0]) + abs(node_a[1] - node_b[1])

    def _move_to_waypoint(self, target_grid_coords):
        """Generate movement commands to reach the next waypoint (grid coords)."""
        target_world_pos = np.array(self._grid_to_world(*target_grid_coords))
        current_pos = self.agent.position[:2]
        current_heading_rad = self._get_euler_from_quaternion()[2]
        direction = target_world_pos - current_pos
        distance = np.linalg.norm(direction)

        linear_x = 0.0
        angular_z = 0.0

        if distance > 0.05:
            desired_heading_rad = np.arctan2(direction[1], direction[0])
            angular_diff_rad = self._get_angular_difference(current_heading_rad, desired_heading_rad)
            
            K_p_angular = 2.5
            angular_z = np.clip(angular_diff_rad * K_p_angular, -self.max_angular_speed, self.max_angular_speed)
            
            # Reduce linear speed based on how much turning is needed
            # Lower reduction factor means faster turning is allowed
            max_reduction_factor = 0.5 # Speed can drop to 50% for 180 deg turn (Reduced from 0.7)
            turn_reduction = max(0.1, 1.0 - max_reduction_factor * (abs(angular_diff_rad) / np.pi))
            linear_x = self.max_linear_speed * turn_reduction
            
            # Add debug logging
            rospy.logdebug(
                f"Agent {self.agent.agent_id} _move_to_waypoint: "
                f"TargetGrid={target_grid_coords}, TargetWorld={target_world_pos}, CurrWorld={current_pos}, Dist={distance:.2f}, "
                f"CurrHead={np.rad2deg(current_heading_rad):.1f}, DesHead={np.rad2deg(desired_heading_rad):.1f}, AngDiff={np.rad2deg(angular_diff_rad):.1f}, "
                f"TurnRed={turn_reduction:.2f}, FinalLin={linear_x:.2f}, FinalAng={angular_z:.2f}"
            )
        # else: # Already close to waypoint, command zero velocity (will be handled by path index update)
        #    rospy.logdebug(f"Agent {self.agent.agent_id}: Reached vicinity of waypoint {target_grid_coords}.")

        return linear_x, angular_z

    def _validate_path(self, path):
        """Implement the logic to validate the path."""
        # This is a placeholder and should be replaced with actual path validation logic
        rospy.loginfo(f"Agent {self.agent.agent_id}: Validating path of length {len(path)}")
        return True # Placeholder return, actual implementation needed

    def _normalize_angle(self, angle):
        """Normalize an angle to be within [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle 

    def _publish_grid_map(self):
        """Publishes the internal grid map as a nav_msgs/OccupancyGrid."""
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.odom_frame

        msg.info.map_load_time = rospy.Time.now() # Can use current time
        msg.info.resolution = self.cell_size
        msg.info.width = self.grid_dims
        msg.info.height = self.grid_dims

        # Origin is bottom-left corner in world frame
        msg.info.origin = Pose()
        msg.info.origin.position.x = -self.map_size / 2.0
        msg.info.origin.position.y = -self.map_size / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0 # Default orientation

        # Convert grid data (0-unknown, 1-free, 2-obstacle, 3-inflated) to OccupancyGrid format (-1 to 100)
        grid_data = self.grid.flatten()
        occupancy_data = np.zeros_like(grid_data, dtype=np.int8)
        occupancy_data[grid_data == 0] = -1 # Unknown
        occupancy_data[grid_data == 1] = 0   # Free
        occupancy_data[grid_data == 2] = 100 # Obstacle
        occupancy_data[grid_data == 3] = 90  # Inflated
        msg.data = occupancy_data.tolist()

        self.grid_map_pub.publish(msg)
        rospy.logdebug_throttle(5.0, f"Agent {self.agent.agent_id}: Published grid map.") 

    def _publish_current_path(self):
        """Publishes the current path as a nav_msgs/Path for RViz."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.odom_frame

        if self.current_path:
            for grid_point in self.current_path:
                if not isinstance(grid_point, (list, tuple)) or len(grid_point) != 2:
                    rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id}: Invalid point in current_path: {grid_point}")
                    continue
                wx, wy = self._grid_to_world(grid_point[0], grid_point[1])
                pose = PoseStamped()
                pose.header.stamp = path_msg.header.stamp
                pose.header.frame_id = self.odom_frame
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.position.z = 0.0 # Assuming planar
                pose.pose.orientation.w = 1.0 # Default orientation
                path_msg.poses.append(pose)

        self.path_pub.publish(path_msg) 

    def _handle_stuck_recovery(self):
        """Initiates the recovery jiggle sequence when the agent gets stuck."""
        rospy.logwarn(f"Agent {self.agent.agent_id}: Stuck timeout ({self.stuck_timeout.to_sec()}s) reached! Initiating recovery jiggle.")

        # --- Reset relevant states --- #
        self.stuck_start_time = None # Reset timer for the *next* stuck event
        self.pf_avoid_start_time = None # Ensure PF timer is also clear
        self.prev_pf_linear_x = 0.0 # Reset PF damping state
        self.prev_pf_angular_z = 0.0
        self.last_activity_time = rospy.Time.now() # Reset general inactivity timer

        # --- Clear current goal and release claim --- #
        if self.current_target_frontier_id:
            self._release_claim(self.current_target_frontier_id) # Releases claim and clears target/path
        else:
            # Explicitly clear path if no target was set (or release didn't clear it)
            self.current_path = None 
            self._publish_current_path() # Publish empty path
        
        # --- Transition to Recovery Jiggle State --- #
        self.avoidance_state = 'recovery_jiggle'
        self.recovery_state = 'backward' # Start with moving backward
        self.recovery_state_start_time = rospy.Time.now()
        self.recovery_target_angle = None # Not needed for backward state
        rospy.loginfo(f"Agent {self.agent.agent_id}: Recovery Action: Transitioning to 'recovery_jiggle', starting with 'backward' state.")

    # --- Recovery Jiggle Execution ---
    def _execute_recovery_jiggle(self):
        """Executes the multi-step recovery maneuver (backward, turn, forward)."""
        linear_x, angular_z = 0.0, 0.0
        now = rospy.Time.now()

        if self.recovery_state is None or self.recovery_state_start_time is None:
            rospy.logerr(f"Agent {self.agent.agent_id}: Entered _execute_recovery_jiggle with invalid state ({self.recovery_state}) or start time ({self.recovery_state_start_time}). Aborting recovery.")
            self.avoidance_state = 'none' # Go back to normal state
            self.stuck_start_time = None # Ensure stuck timer is reset
            return 0.0, 0.0
            
        elapsed_time = now - self.recovery_state_start_time

        # --- Backward State ---
        if self.recovery_state == 'backward':
            if elapsed_time < self.RECOVERY_BACKWARD_DURATION:
                linear_x = self.RECOVERY_BACKWARD_SPEED
                angular_z = 0.0
                rospy.logdebug_throttle(0.5, f"Agent {self.agent.agent_id}: Recovery Jiggle: Backward ({elapsed_time.to_sec():.1f}/{self.RECOVERY_BACKWARD_DURATION.to_sec():.1f}s)")
            else:
                # Transition to Turning state
                rospy.loginfo(f"Agent {self.agent.agent_id}: Recovery Jiggle: Backward complete. Transitioning to Turning.")
                self.recovery_state = 'turning'
                self.recovery_state_start_time = now
                # Calculate random target angle relative to current yaw (+/- 60 to 120 deg)
                turn_magnitude = np.random.uniform(np.deg2rad(60), np.deg2rad(120))
                turn_direction = np.random.choice([-1, 1])
                self.recovery_target_angle = self._normalize_angle(self.agent.get_yaw() + turn_direction * turn_magnitude)
                rospy.loginfo(f"Agent {self.agent.agent_id}: Recovery Jiggle: Calculated random turn target: {np.rad2deg(self.recovery_target_angle):.1f} deg (World Yaw)")
                linear_x, angular_z = 0.0, 0.0 # Stop briefly before turning
        
        # --- Turning State ---
        elif self.recovery_state == 'turning':
            if elapsed_time < self.RECOVERY_TURN_DURATION:
                current_yaw = self.agent.get_yaw()
                yaw_error = self._normalize_angle(self.recovery_target_angle - current_yaw)
                # Simple proportional control for turning, capped by speed
                # Use direction of error to determine turn direction
                turn_speed_cmd = np.sign(yaw_error) * self.RECOVERY_TURN_SPEED
                angular_z = turn_speed_cmd
                linear_x = 0.0
                # Optional: Could add check for angle tolerance to finish turn early
                rospy.logdebug_throttle(0.5, f"Agent {self.agent.agent_id}: Recovery Jiggle: Turning ({elapsed_time.to_sec():.1f}/{self.RECOVERY_TURN_DURATION.to_sec():.1f}s). Target: {np.rad2deg(self.recovery_target_angle):.1f}, Curr: {np.rad2deg(current_yaw):.1f}, Cmd: {angular_z:.2f}")
            else:
                # Transition to Forward state
                rospy.loginfo(f"Agent {self.agent.agent_id}: Recovery Jiggle: Turning complete. Transitioning to Forward.")
                self.recovery_state = 'forward'
                self.recovery_state_start_time = now
                self.recovery_target_angle = None
                linear_x, angular_z = 0.0, 0.0 # Stop briefly before moving forward

        # --- Forward State ---
        elif self.recovery_state == 'forward':
            if elapsed_time < self.RECOVERY_FORWARD_DURATION:
                linear_x = self.RECOVERY_FORWARD_SPEED
                angular_z = 0.0
                rospy.logdebug_throttle(0.5, f"Agent {self.agent.agent_id}: Recovery Jiggle: Forward ({elapsed_time.to_sec():.1f}/{self.RECOVERY_FORWARD_DURATION.to_sec():.1f}s)")
            else:
                # Recovery sequence complete, transition back to 'none'
                rospy.loginfo(f"Agent {self.agent.agent_id}: Recovery Jiggle: Forward complete. Transitioning back to 'none'.")
                self.avoidance_state = 'none'
                self.recovery_state = None
                self.recovery_state_start_time = None
                self.stuck_start_time = None # Ensure overall stuck timer is reset after successful recovery
                linear_x, angular_z = 0.0, 0.0 # Stop
        
        # --- Unknown State ---
        else:
            rospy.logerr(f"Agent {self.agent.agent_id}: Unknown recovery state: {self.recovery_state}. Aborting recovery.")
            self.avoidance_state = 'none'
            self.recovery_state = None
            self.recovery_state_start_time = None
            self.stuck_start_time = None
            linear_x, angular_z = 0.0, 0.0
            
        return linear_x, angular_z

    # --- Add method to release claim --- #
    def _release_claim(self, frontier_id):
        """Release the claim on a frontier and publish the update."""
        if frontier_id and frontier_id in self.known_frontiers:
            # Check if we are actually the claimant
            if self.known_frontiers[frontier_id].get('claimant_agent_id') == self.agent.agent_id:
                rospy.loginfo(f"Agent {self.agent.agent_id}: Releasing claim on frontier {frontier_id}.")
                update_data = self.known_frontiers[frontier_id].copy()
                update_data['status'] = 'open'
                update_data['claimant_agent_id'] = -1
                # Publish the update
                self.agent.share_data('frontiers', update_data)
                # Update local knowledge
                self.known_frontiers[frontier_id] = update_data
                # Clear target and path if this was the current target
                if self.current_target_frontier_id == frontier_id:
                    self.current_target_frontier_id = None
                    self.current_path = None
                    self._publish_current_path() # Publish empty path
                # Reset failure count for this frontier
                self.path_planning_failures.pop(frontier_id, None)
            else:
                 rospy.logdebug(f"Agent {self.agent.agent_id}: Tried to release claim on {frontier_id}, but not the claimant.")
        else:
             rospy.logwarn(f"Agent {self.agent.agent_id}: Tried to release claim on non-existent or None frontier: {frontier_id}")

    # --- Add Reachability Check Method --- #
    def _is_reachable(self, start_grid, goal_grid, limit=2000): # Changed default limit
        """Check reachability using a limited BFS on the current grid."""
        rows, cols = self.grid.shape
        start_node = tuple(start_grid)
        goal_node = tuple(goal_grid)
        
        # --- Log BFS parameters --- #
        rospy.logdebug(f"Agent {self.agent.agent_id} Reachability Check: Start={start_node}, Goal={goal_node}, Limit={limit}")
        # -------------------------- #

        # Quick check if start or goal are invalid themselves
        if not (0 <= start_node[0] < rows and 0 <= start_node[1] < cols and self.grid[start_node] not in [2, 3]):
            rospy.logdebug(f"Agent {self.agent.agent_id} Reachability: Start node {start_node} invalid for reachability check.")
            return False
        if not (0 <= goal_node[0] < rows and 0 <= goal_node[1] < cols and self.grid[goal_node] not in [2, 3]):
            rospy.logdebug(f"Agent {self.agent.agent_id} Reachability: Goal node {goal_node} invalid for reachability check.")
            return False

        if start_node == goal_node:
            return True # Already there

        q = collections.deque([start_node])
        visited = {start_node}
        count = 0

        # Use 8-connectivity for reachability check as well
        bfs_neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while q and count < limit:
            current = q.popleft()
            count += 1

            if current == goal_node:
                rospy.logdebug(f"Agent {self.agent.agent_id} Reachability: Goal {goal_node} found reachable from {start_node} within {count} steps.")
                return True

            for i, j in bfs_neighbors:
                neighbor = current[0] + i, current[1] + j

                if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and
                        neighbor not in visited and
                        self.grid[neighbor] not in [2, 3]): # Check traversability (free or unknown)
                    visited.add(neighbor)
                    q.append(neighbor)

        # --- Log BFS failure details --- #
        rospy.logdebug(f"Agent {self.agent.agent_id} Reachability Fail: Goal {goal_node} NOT reachable from {start_node} within BFS limit {limit}. Explored {count} nodes.") # Use limit parameter in log
        # ------------------------------- #
        return False
    # --- End Reachability Check Method --- #

    # --- Visualization --- #
    def _visualization_callback(self, event):
        """Timer callback to publish visualization markers."""
        self._publish_grid()
        self._publish_frontiers()
        # Path is published when planned or cleared

    def _publish_grid(self):
        """Publish the current grid map as visualization markers."""
        marker = Marker()
        marker.header.frame_id = self.odom_frame # Publish grid in odom frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"agent_{self.agent.agent_id}_grid"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cell_size
        marker.scale.y = self.cell_size
        marker.scale.z = 0.01 # Flat cubes

        marker.points = []
        marker.colors = []
        # Import ColorRGBA if not already imported at top level
        from std_msgs.msg import ColorRGBA # Add this import inside if needed

        rows, cols = self.grid.shape
        for r in range(rows):
            for c in range(cols):
                state = self.grid[r, c]
                if state != GRID_UNKNOWN: # Don't visualize unknown cells
                    point = Point()
                    world_coords = self._grid_to_world(r, c)
                    point.x = world_coords[0]
                    point.y = world_coords[1]
                    point.z = -0.1 # Slightly below agent
                    marker.points.append(point)

                    color = ColorRGBA()
                    color.a = 0.5 # Semi-transparent
                    if state == GRID_FREE:
                        color.r = 0.0; color.g = 1.0; color.b = 0.0 # Green for free
                    elif state == GRID_OBSTACLE:
                        color.r = 1.0; color.g = 0.0; color.b = 0.0 # Red for obstacle
                    elif state == GRID_INFLATED:
                        color.r = 1.0; color.g = 0.5; color.b = 0.0 # Orange for inflated
                    marker.colors.append(color)
        
        if marker.points:
             self.grid_pub.publish(marker)

    def _publish_current_path(self):
        """Publish the current path as a line strip marker."""
        marker = Marker()
        marker.header.frame_id = self.odom_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"agent_{self.agent.agent_id}_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05 # Line width
        from std_msgs.msg import ColorRGBA # Import ColorRGBA if not already imported
        from geometry_msgs.msg import Point # Import Point if not already imported
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0) # Blue path

        marker.points = []
        if self.current_path:
            # Add agent's current position as the start
            current_world_pos = self.agent.position
            start_point = Point()
            start_point.x = current_world_pos[0]
            start_point.y = current_world_pos[1]
            start_point.z = 0.0 # Assume path is planar
            marker.points.append(start_point)
            # Add waypoints
            # Iterate from current index if following, otherwise show full path
            start_idx = self.current_path_index if self.current_path_index < len(self.current_path) else 0
            for grid_node in self.current_path[start_idx:]:
                if not isinstance(grid_node, (list, tuple)) or len(grid_node) != 2:
                    rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id} PathViz: Invalid point {grid_node} in path.")
                    continue # Skip invalid points
                try:
                    waypoint_world = self._grid_to_world(grid_node[0], grid_node[1])
                    p = Point()
                    p.x = waypoint_world[0]
                    p.y = waypoint_world[1]
                    p.z = 0.0
                    marker.points.append(p)
                except IndexError:
                    rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id} PathViz: Grid node {grid_node} out of bounds.")
        else:
            # If no path, publish an empty marker to clear previous path in RViz
             marker.action = Marker.DELETE # Or publish empty point list

        self.path_pub.publish(marker)

    def _publish_frontiers(self):
        """Publish known frontiers as markers."""
        marker_array = MarkerArray()
        now = rospy.Time.now()
        marker_id_counter = 0
        # Import ColorRGBA if not already imported
        from std_msgs.msg import ColorRGBA
        from geometry_msgs.msg import Point # Import Point if needed

        for fid, fdata in self.known_frontiers.items():
            # Check if centroid_world exists and is valid
            centroid = fdata.get('centroid_world')
            if not isinstance(centroid, (list, tuple)) or len(centroid) < 2:
                 rospy.logwarn_throttle(10.0, f"Agent {self.agent.agent_id} FrontierViz: Invalid centroid {centroid} for frontier {fid}. Skipping.")
                 continue
                 
            marker = Marker()
            marker.header.frame_id = self.odom_frame
            marker.header.stamp = now
            marker.ns = f"agent_{self.agent.agent_id}_frontiers"
            marker.id = marker_id_counter
            marker_id_counter += 1
            marker.type = Marker.SPHERE # Represent frontier centroid as sphere
            marker.action = Marker.ADD
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = 0.1 # Slightly above ground
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.lifetime = rospy.Duration(2.5) # Match publish rate + buffer

            status = fdata.get('status', 'open')
            color = ColorRGBA(a=0.8)
            if status == 'open':
                color.r = 1.0; color.g = 1.0; color.b = 0.0 # Yellow
            elif status == 'claimed':
                color.r = 0.0; color.g = 1.0; color.b = 1.0 # Cyan
                # Optionally color based on claimant ID?  # Correct indentation
                claimant = fdata.get('claimant_agent_id', -1) # Correct indentation
                if claimant == self.agent.agent_id:
                     # Maybe make own claimed frontiers slightly different?
                     color.b = 0.7 # Correct indentation
            elif status == 'explored':
                color.r = 0.5; color.g = 0.5; color.b = 0.5 # Grey
            marker.color = color
            marker_array.markers.append(marker)
            
            # Add text marker for Frontier ID
            text_marker = Marker()
            text_marker.header.frame_id = self.odom_frame
            text_marker.header.stamp = now
            text_marker.ns = f"agent_{self.agent.agent_id}_frontier_ids"
            text_marker.id = marker_id_counter # Use different ID
            marker_id_counter += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = centroid[0]
            text_marker.pose.position.y = centroid[1]
            text_marker.pose.position.z = 0.3 # Above sphere
            text_marker.scale.z = 0.15 # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = fid # Display the frontier ID
            text_marker.lifetime = rospy.Duration(2.5)
            marker_array.markers.append(text_marker)

        if marker_array.markers:
            self.frontier_pub.publish(marker_array)

    # --- Bidding and Task Allocation --- #
    def _bidding_cycle_callback(self, event):
        """Timer callback to evaluate frontiers and potentially bid or select task."""
        if self.current_path or self.avoidance_state != 'none':
            # Don't bid or select new task if already following path or avoiding/recovering
            return
            
        open_frontiers = []
        for fid, fdata in self.known_frontiers.items():
            if fdata.get('status', 'open') == 'open':
                # Check reachability before considering it truly open for bidding
                start_grid = self._world_to_grid(self.agent.position[0], self.agent.position[1])
                goal_grid = fdata.get('centroid_grid')
                if goal_grid and self._is_reachable(start_grid, tuple(goal_grid)): # Use quick check
                     open_frontiers.append(fdata)
                else:
                     rospy.logdebug(f"Agent {self.agent.agent_id}: Frontier {fid} at {goal_grid} is open but unreachable from {start_grid}, skipping bid.")

        if not open_frontiers:
            rospy.logdebug_throttle(5.0, f"Agent {self.agent.agent_id}: No reachable open frontiers found.")
            # Check if we should terminate due to inactivity
            self._check_termination()
            return
        else:
             # Activity detected (found open frontiers), reset inactivity timer
             self.last_activity_time = rospy.Time.now()

        # --- Simple Selection (Highest Utility) - Replace with proper bidding later --- #
        best_frontier = None
        max_utility = -float('inf')
        best_cost = float('inf') # Store cost for logging

        for frontier in open_frontiers:
            utility, cost = self._calculate_utility(frontier)
            rospy.logdebug(f"Agent {self.agent.agent_id}: Evaluating Frontier {frontier['id']}. Cost: {cost:.2f}, Util: {utility:.4f}")
            if utility > max_utility:
                max_utility = utility
                best_cost = cost
                best_frontier = frontier
        # --------------------------------------------------------------------------- #

        if best_frontier:
            # --- Replace with Bidding Logic --- #
            # For now, directly claim the best one we found
            rospy.loginfo(f"Agent {self.agent.agent_id}: Selected task {best_frontier['id']} (Cost: {best_cost:.2f}, Util: {max_utility:.4f}) for claim.") # Add closing quote
            self._claim_frontier(best_frontier['id'])
            # --- End Bidding Logic Placeholder --- #
        else:
            rospy.logdebug(f"Agent {self.agent.agent_id}: Could not select a best frontier from available open ones.")

    def _calculate_utility(self, frontier_data):
        """Calculate the utility of exploring a given frontier.
        
           Higher utility is better.

           Returns:
               tuple: A tuple containing (utility, cost). utility is higher for better frontiers,
                      cost is an estimate (e.g., A* path cost).
        """
        try:
            start_pos = self.agent.position[:2]
            goal_pos_world = np.array(frontier_data['centroid_world'])
            start_grid = self._world_to_grid(start_pos[0], start_pos[1])
            goal_grid = tuple(frontier_data['centroid_grid'])

            # Check bounds for start/goal grid coords
            if not (0 <= start_grid[0] < self.grid_dims and 0 <= start_grid[1] < self.grid_dims):
                rospy.logwarn(f"Agent {self.agent.agent_id} UtilCalc: Start position {start_pos} is outside grid! Cannot calculate utility.")
                return -float('inf'), float('inf')
            if not (0 <= goal_grid[0] < self.grid_dims and 0 <= goal_grid[1] < self.grid_dims):
                 rospy.logwarn(f"Agent {self.agent.agent_id} UtilCalc: Goal grid {goal_grid} for frontier {frontier_data['id']} is outside grid! Cannot calculate utility.")
                 return -float('inf'), float('inf')

            # Estimate cost using A* path length
            path, cost = self._find_path(start_grid, goal_grid)

            if path is None:
                 # If unreachable by A* (should have been caught by BFS, but double-check)
                 rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id} UtilCalc: Frontier {frontier_data['id']} at {goal_grid} determined unreachable by A* from {start_grid}.")
                 return -float('inf'), float('inf')
            
            # Use the A* score directly for now, lower is better.
            cost_value = cost 

            # Information Gain (simple: size of frontier?)
            info_gain = frontier_data.get('size', 1) # Default to 1 if size missing

            # Utility = Information Gain / Cost (higher is better)
            # Avoid division by zero
            if cost_value <= 1e-6:
                utility = float('inf') # Very close, high utility
            else:
                 # Combine info gain and cost - using inverse cost for simplicity
                 utility = 1.0 / cost_value # Lower cost => Higher utility
                 # Optionally add info_gain influence: utility = info_gain / cost_value
            
            return utility, cost_value

        except Exception as e:
            rospy.logerr(f"Agent {self.agent.agent_id}: Error calculating utility for frontier {frontier_data.get('id', 'N/A')}: {e}")
            import traceback
            traceback.print_exc() # Print stack trace for debugging
            return -float('inf'), float('inf')

    def _check_termination(self):
        # Implement any additional logic you want to execute when checking for termination
        # This is just a placeholder and should be replaced with actual logic
        pass

    # --- ADD BACK MISSING METHODS --- #

    def _claim_frontier(self, frontier_id):
        """Claim a frontier, publish the claim, and plan path."""
        if frontier_id not in self.known_frontiers:
            rospy.logerr(f"Agent {self.agent.agent_id}: Cannot claim non-existent frontier {frontier_id}")
            return

        frontier_data = self.known_frontiers[frontier_id]
        if frontier_data.get('status', 'open') != 'open':
            rospy.logwarn(f"Agent {self.agent.agent_id}: Cannot claim frontier {frontier_id}, status is {frontier_data.get('status')}.")
            return

        # Update local status and publish claim
        rospy.loginfo(f"Agent {self.agent.agent_id}: Publishing claim for frontier {frontier_id}")
        frontier_data['status'] = 'claimed'
        frontier_data['claimant_agent_id'] = self.agent.agent_id
        frontier_data['last_updated'] = rospy.Time.now().to_sec()
        self.known_frontiers[frontier_id] = frontier_data # Update local knowledge
        self.agent.share_data('frontiers', frontier_data) # Publish claim
        self.current_target_frontier_id = frontier_id
        # Reset path failure count when claiming
        self.path_planning_failures.pop(frontier_id, None)

        # Plan path
        start_grid = self._world_to_grid(self.agent.position[0], self.agent.position[1])
        goal_grid = tuple(frontier_data['centroid_grid'])
        
        # Ensure start_grid is valid before planning
        if not (0 <= start_grid[0] < self.grid_dims and 0 <= start_grid[1] < self.grid_dims):
             rospy.logerr(f"Agent {self.agent.agent_id} PathPlan: Start grid {start_grid} invalid. Cannot plan path to {frontier_id}.")
             self._release_claim(frontier_id) # Release if we can't even start planning
             return

        rospy.loginfo(f"Agent {self.agent.agent_id}: Planning path from {start_grid} to {goal_grid} for frontier {frontier_id}")
        path, cost = self._find_path(start_grid, goal_grid)

        if path:
            rospy.loginfo(f"Agent {self.agent.agent_id}: Path found for {frontier_id} (Cost: {cost:.2f}, Length: {len(path)} steps). Starting navigation.")
            self.current_path = path
            # Start path index at 1 to target the first waypoint, not the start cell itself
            self.current_path_index = 1 
            self._publish_current_path() # Visualize path
            # Reset path progress tracking
            self.last_path_position = None
            self.last_path_progress_time = None
        else:
            rospy.logwarn(f"Agent {self.agent.agent_id}: Path planning failed for claimed frontier {frontier_id}. Releasing claim.")
            self._release_claim(frontier_id)
            # Increment failure count after failed planning attempt
            failures = self.path_planning_failures.get(frontier_id, 0) + 1
            self.path_planning_failures[frontier_id] = failures
            rospy.logwarn(f"Agent {self.agent.agent_id}: Path planning failure increments count for {frontier_id} to {failures}")

    def _mark_frontier_explored(self, frontier_id):
        """Mark a frontier as explored and publish the update."""
        if frontier_id and frontier_id in self.known_frontiers:
            fdata = self.known_frontiers[frontier_id]
            if fdata.get('status') != 'explored': # Only update if not already marked
                 rospy.loginfo(f"Agent {self.agent.agent_id}: Marking frontier {frontier_id} as explored.")
                 fdata['status'] = 'explored'
                 fdata['last_updated'] = rospy.Time.now().to_sec()
                 # Keep claimant ID for record? Optional.
                 # fdata['claimant_agent_id'] = -1 # Or keep the ID of who explored it?
                 self.known_frontiers[frontier_id] = fdata
                 self.agent.share_data('frontiers', fdata)
        else:
            rospy.logwarn(f"Agent {self.agent.agent_id}: Tried to mark non-existent or None frontier as explored: {frontier_id}")

    def _release_claim(self, frontier_id):
        """Release the claim on a frontier and publish the update."""
        if frontier_id and frontier_id in self.known_frontiers:
            # Check if we are actually the claimant
            if self.known_frontiers[frontier_id].get('claimant_agent_id') == self.agent.agent_id:
                rospy.loginfo(f"Agent {self.agent.agent_id}: Releasing claim on frontier {frontier_id}.")
                update_data = self.known_frontiers[frontier_id].copy()
                update_data['status'] = 'open'
                update_data['claimant_agent_id'] = -1
                update_data['last_updated'] = rospy.Time.now().to_sec()
                # Publish the update
                self.agent.share_data('frontiers', update_data)
                # Update local knowledge
                self.known_frontiers[frontier_id] = update_data
                # Clear target and path if this was the current target
                if self.current_target_frontier_id == frontier_id:
                    self.current_target_frontier_id = None
                    self.current_path = None
                    self._publish_current_path() # Publish empty path
                    # Reset path progress tracking
                    self.last_path_position = None
                    self.last_path_progress_time = None
                # Reset failure count for this frontier
                self.path_planning_failures.pop(frontier_id, None)
            else:
                 rospy.logdebug(f"Agent {self.agent.agent_id}: Tried to release claim on {frontier_id}, but not the claimant ({self.known_frontiers[frontier_id].get('claimant_agent_id')}).")
        else:
             rospy.logwarn(f"Agent {self.agent.agent_id}: Tried to release claim on non-existent or None frontier: {frontier_id}")

    # --- Path Planning (A*) --- #
    # ... (rest of the file, including A* methods, _is_reachable, etc.) ...