#!/usr/bin/env python3

import rospy
import numpy as np
import json
import tf2_ros # For TF lookups
import tf2_geometry_msgs # For transforming geometry_msgs
from geometry_msgs.msg import PointStamped
import tf.transformations # Added for euler_from_quaternion
import heapq # Added for A*
from nav_msgs.msg import OccupancyGrid, MapMetaData # For grid visualization
from geometry_msgs.msg import Pose, PoseStamped # For OccupancyGrid origin and Path poses
from nav_msgs.msg import Path # For path visualization
import collections # Added for BFS

# --- Import Task Allocator --- #
from ..utils.task_allocator import TaskAllocator
# --------------------------- #

# Assuming navigation/frontier detection utilities will be needed later
# from ..utils.navigation import navigate_to_point, detect_frontiers
# from ..utils.costmap import Costmap # Or however costmap is accessed

class ExploreBehaviour:
    """Behaviour for coordinated multi-agent exploration using frontier sharing."""

    def __init__(self, agent):
        """Initialize the Explore behaviour."""
        self.agent = agent
        self.known_frontiers = {}  # Store frontiers: {frontier_id: frontier_data_dict}
        self.current_target_frontier_id = None

        # --- Grid Map Parameters (from SearchBehaviour) ---
        self.map_size = 20.0  # meters
        self.cell_size = 0.5  # meters - *** REDUCED from 1.0 for finer A* ***
        self.grid_dims = int(self.map_size / self.cell_size) # Recalculate dimensions
        # Initialize grid (0: unknown, 1: free/visited, 2: obstacle, 3: inflated obstacle)
        self.grid = np.zeros((self.grid_dims, self.grid_dims), dtype=np.int8)

        # --- Path Planning State (from SearchBehaviour) ---
        self.current_path = None # List of (grid_x, grid_y) tuples
        self.current_path_index = 0 # Index of the next waypoint in current_path
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
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
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
    def _calculate_potential_field_velocity(self):
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
        return world_x, world_y

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
            transform = self.tf_buffer.lookup_transform(self.odom_frame, self.laser_frame, lookup_time, rospy.Duration(0.1))
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
                transform = self.tf_buffer.lookup_transform(self.odom_frame, self.laser_frame, lookup_time, rospy.Duration(0.1))
                # --- DEBUG LOG --- #
                # Log transform details if successful
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                rz = tf.transformations.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])[2]
                rospy.logdebug(f"Agent {self.agent.agent_id} DBG GridUpdate TF: Using transform Odom<-Laser @ time {lookup_time.to_sec():.3f}. Trans=({tx:.2f},{ty:.2f}), RotZ={np.rad2deg(rz):.1f}deg")
                # --- End DEBUG --- #
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e2:
                 # If retry also fails, log final warning and return
                 rospy.logwarn_throttle(5, f"Agent {self.agent.agent_id}: TF Lookup failed (2nd try - Time(0)): {e2}. Skipping grid update this cycle.")
                 return # Give up for this cycle
            
        # If transform lookup succeeded (either first or second try)
        agent_pos_grid = (agent_grid_x, agent_grid_y)
        # Process scan ranges using the obtained transform
        for i, r in enumerate(ranges):
            # === DEBUG LOGGING START ===
            log_this_point = False # Flag to log details for specific points if needed (e.g., near agent)
            angle = angle_min + i * angle_increment; x_laser = r * np.cos(angle); y_laser = r * np.sin(angle)
            # --- Use scan_time (if valid) for PointStamped header --- #
            point_laser = PointStamped()
            point_laser.header.stamp = scan_time if scan_time else rospy.Time.now()
            point_laser.header.frame_id = self.laser_frame
            # ------------------------------------------------------ #
            try:
                if min_range < r < max_range:
                    point_laser.point.x = x_laser; point_laser.point.y = y_laser
                    point_odom = self.tf_buffer.transform(point_laser, self.odom_frame, timeout=rospy.Duration(0.1)) 
                    obs_grid_x, obs_grid_y = self._world_to_grid(point_odom.point.x, point_odom.point.y)
                    if (obs_grid_x, obs_grid_y) != agent_pos_grid: 
                        # === DEBUG LOGGING ===
                        # Log all obstacle markings
                        log_this_point = True
                        rospy.logdebug(f"Agent {self.agent.agent_id} DBG GridUpdate Obstacle: Marking obstacle at grid ({obs_grid_x},{obs_grid_y}) from world ({point_odom.point.x:.2f},{point_odom.point.y:.2f}). Laser index {i}.")
                        # ====================
                        self.grid[obs_grid_x, obs_grid_y] = 2
                    self._mark_line(agent_grid_x, agent_grid_y, obs_grid_x, obs_grid_y, False, False)
                elif r >= max_range:
                     point_laser.point.x = max_range * np.cos(angle); point_laser.point.y = max_range * np.sin(angle)
                     point_odom = self.tf_buffer.transform(point_laser, self.odom_frame, timeout=rospy.Duration(0.1)) 
                     max_grid_x, max_grid_y = self._world_to_grid(point_odom.point.x, point_odom.point.y)
                     self._mark_line(agent_grid_x, agent_grid_y, max_grid_x, max_grid_y, False, True)
            # Catch errors during point transformation or grid marking
            except Exception as e: rospy.logwarn_throttle(10, f"Agent {self.agent.agent_id}: Error processing scan point {i}: {e}")

        # --- After processing scan, inflate obstacles --- #
        # --- Disabled inflation by setting radius to 0 --- #
        self._inflate_obstacles(inflation_radius_cells=0) # Was 1
        # --------------------------------------------- #

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

    def _mark_line(self, x0, y0, x1, y1, mark_endpoint_obstacle=False, endpoint_inclusive=True):
        """Mark cells on line as free (1), optionally handle endpoint."""
        dx = abs(x1 - x0); dy = abs(y1 - y0); sx = 1 if x0 < x1 else -1; sy = 1 if y0 < y1 else -1
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
                 # --- Ensure only Unknown (0) is marked Free (1) ---
                 if self.grid[x, y] == 0: 
                    if log_this_cell:
                        rospy.logdebug(f"Agent {self.agent.agent_id} DBG MarkLine Free: Marking FREE at grid ({x},{y}) along line from ({x0},{y0}) to ({x1},{y1})")
                    self.grid[x, y] = 1
                 # ----------------------------------------------------
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
            BFS_LIMIT = 100 # Limit search depth/iterations to prevent excessive search
            bfs_count = 0

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
                rospy.logerr(f"Agent {self.agent.agent_id} A*: Original goal {original_goal_node} is obstacle/inflated, and BFS within limit {BFS_LIMIT} found no valid neighbors. Pathfinding failed.")
                return None, float('inf') # Cannot find any valid target nearby
        # --- End Obstacle Goal Handling --- #

        # Final check: Ensure the determined goal_node is not obstacle/inflated (should be guaranteed by logic above)
        if self.grid[goal_node] in [2, 3]:
             rospy.logerr(f"Agent {self.agent.agent_id} A*: CRITICAL ERROR: Final goal node {goal_node} (state={self.grid[goal_node]}) is obstacle/inflated after check! Pathfinding failed.")
             return None, float('inf')

        # --- Add Reachability Check --- #
        if not self._is_reachable(start_node, goal_node):
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
    def _is_reachable(self, start_grid, goal_grid, limit=500):
        """Check reachability using a limited BFS on the current grid."""
        rows, cols = self.grid.shape
        start_node = tuple(start_grid)
        goal_node = tuple(goal_grid)

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

        rospy.logdebug(f"Agent {self.agent.agent_id} Reachability: Goal {goal_node} NOT reachable from {start_node} within BFS limit {limit}. Explored {count} nodes.")
        return False
    # --- End Reachability Check Method --- #