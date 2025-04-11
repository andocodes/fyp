#!/usr/bin/env python3

import rospy
import numpy as np
import math

class ObstacleAvoidanceBehaviour:
    """Base class providing potential field and recovery jiggle obstacle avoidance."""

    def __init__(self, agent):
        self.agent = agent
        
        # Default speed limits if agent doesn't provide them
        self.default_max_linear_speed = rospy.get_param('~avoidance/max_linear_speed', 0.5)  # m/s
        self.default_max_angular_speed = rospy.get_param('~avoidance/max_angular_speed', 1.0)  # rad/s

        # Potential Field Parameters
        self.PF_OBSTACLE_INFLUENCE_RADIUS = 1.0 # meters
        self.PF_REPULSION_GAIN = 0.5 # Gain for obstacle repulsion
        self.PF_ATTRACTION_GAIN = 0.2 # Gain for default forward attraction
        self.PF_DAMPING_FACTOR = 0.8 # Damping factor for velocity smoothing (0=no damping, 1=immediate change)
        self.PF_AVOID_DURATION = rospy.Duration(3.0) # Default duration to stay in avoiding state
        self.PF_K_LINEAR = 0.5 # Proportional gain for linear velocity from force
        self.PF_K_ANGULAR = 1.5 # Proportional gain for angular velocity from force angle

        # Recovery Jiggle Parameters
        self.RECOVERY_BACKWARD_SPEED = -0.1 # m/s
        self.RECOVERY_BACKWARD_DURATION = rospy.Duration(1.5) # s
        self.RECOVERY_TURN_SPEED = 0.5 # rad/s
        self.RECOVERY_TURN_DURATION = rospy.Duration(2.0) # s
        self.RECOVERY_FORWARD_SPEED = 0.1 # m/s
        self.RECOVERY_FORWARD_DURATION = rospy.Duration(1.0) # s

        # Avoidance/Recovery State
        self.avoidance_state = 'none' # 'none', 'avoiding', 'recovery_jiggle'
        self.stuck_start_time = None # Timestamp when avoidance/recovery was first triggered (for timeout)
        self.pf_avoid_start_time = None # Timestamp when PF avoidance started
        self.recovery_state = None # Sub-state for jiggle: 'backward', 'turning', 'forward'
        self.recovery_state_start_time = None # Timestamp for current recovery sub-state
        self.recovery_target_angle = None # Target yaw for recovery turn
        self.prev_pf_linear_x = 0.0 # Previous PF linear velocity for damping
        self.prev_pf_angular_z = 0.0 # Previous PF angular velocity for damping

        # Stuck Timeout
        self.stuck_timeout = rospy.Duration(15.0) # Max time allowed in avoidance/recovery states before retry

        rospy.loginfo(f"Agent {self.agent.agent_id}: ObstacleAvoidanceBehaviour initialized.")

    def _normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def compute_avoidance(self):
        """
        Checks the current avoidance state and computes avoidance/recovery velocities.
        Returns: (linear_x, angular_z, should_override)
        should_override is True if avoidance/recovery is active.
        """
        linear_x, angular_z = 0.0, 0.0
        should_override = False
        now = rospy.Time.now()

        # Check for Stuck Timeout
        if self.avoidance_state != 'none' and self.stuck_start_time is not None:
            if now - self.stuck_start_time > self.stuck_timeout:
                rospy.logwarn(f"Agent {self.agent.agent_id}: Stuck timeout exceeded ({self.stuck_timeout.to_sec()}s) in state '{self.avoidance_state}'. Resetting state.")
                self.avoidance_state = 'none'
                self.recovery_state = None
                self.stuck_start_time = None
                return 0.0, 0.0, True # Override with stop command

        # State Machine Logic
        if self.avoidance_state == 'avoiding':
            should_override = True
            # Check if potential field avoidance duration has expired
            if self.pf_avoid_start_time and (now - self.pf_avoid_start_time) > self.PF_AVOID_DURATION:
                rospy.loginfo(f"Agent {self.agent.agent_id}: PF avoidance duration expired. Transitioning back to 'none'.")
                self.avoidance_state = 'none'
                self.pf_avoid_start_time = None
                self.stuck_start_time = None # Reset overall stuck timer
                linear_x, angular_z = 0.0, 0.0 # Stop briefly
            else:
                # Execute potential field avoidance
                linear_x, angular_z = self._calculate_potential_field_velocity()
                rospy.logdebug_throttle(0.5, f"Agent {self.agent.agent_id}: Executing Potential Field Avoidance. Vel=({linear_x:.2f}, {angular_z:.2f})")

        elif self.avoidance_state == 'recovery_jiggle':
            should_override = True
            # Initialize recovery if just entered
            if self.recovery_state is None:
                rospy.loginfo(f"Agent {self.agent.agent_id}: Entering Recovery Jiggle. Starting backward move.")
                self.recovery_state = 'backward'
                self.recovery_state_start_time = now
                if self.stuck_start_time is None: # Ensure stuck timer is running
                    self.stuck_start_time = now 
            # Execute the current recovery step
            linear_x, angular_z = self._execute_recovery_jiggle()
            rospy.logdebug_throttle(0.5, f"Agent {self.agent.agent_id}: Executing Recovery Jiggle ({self.recovery_state}). Vel=({linear_x:.2f}, {angular_z:.2f})")

        # Reset damped velocities if not using PF
        if self.avoidance_state != 'avoiding':
             self.prev_pf_linear_x = 0.0
             self.prev_pf_angular_z = 0.0

        return linear_x, angular_z, should_override

    def _calculate_potential_field_velocity(self, attraction_vector=None):
        """Calculates desired velocity based on repulsive forces from laser scan and an attractive force.
        
        Args:
            attraction_vector (np.array, optional): A 2D numpy array representing the desired direction
                                                 of attraction (e.g., towards a goal). If None,
                                                 defaults to pulling straight forward.
                                                 Defaults to None.
        """
        ranges = self.agent.last_scan_ranges
        if not ranges:
            rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id}: No laser scan data available for potential field calculation.")
            return 0.0, 0.0

        angle_min = self.agent.last_scan_angle_min
        angle_increment = self.agent.last_scan_angle_increment
        num_ranges = len(ranges)

        # Initialize forces based on attraction vector
        # Default: pull straight forward
        attraction_force_x = self.PF_ATTRACTION_GAIN
        attraction_force_y = 0.0
        
        # If a valid attraction vector is provided, use it
        if attraction_vector is not None and isinstance(attraction_vector, np.ndarray) and attraction_vector.shape == (2,):
            norm = np.linalg.norm(attraction_vector)
            if norm > 1e-5: # Avoid division by zero
                normalized_attraction = attraction_vector / norm
                attraction_force_x = normalized_attraction[0] * self.PF_ATTRACTION_GAIN
                attraction_force_y = normalized_attraction[1] * self.PF_ATTRACTION_GAIN
                rospy.logdebug_throttle(2.0, f"Agent {self.agent.agent_id} PF: Using attraction vector {normalized_attraction} scaled by {self.PF_ATTRACTION_GAIN}")
            else:
                 rospy.logdebug_throttle(2.0, f"Agent {self.agent.agent_id} PF: Attraction vector norm too small, defaulting to forward pull.")
        else:
             rospy.logdebug_throttle(2.0, f"Agent {self.agent.agent_id} PF: No valid attraction vector provided, defaulting to forward pull.")

        total_force_x = attraction_force_x
        total_force_y = attraction_force_y
        processed_points = 0

        # Use raw ranges for PF calculation
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
        
        if processed_points == 0:
             rospy.logdebug_throttle(5, f"Agent {self.agent.agent_id} PF: No obstacles within influence radius {self.PF_OBSTACLE_INFLUENCE_RADIUS}m.")

        # Calculate desired angle from the resultant force vector
        # Note: This angle is relative to the robot's current forward direction (base_link frame)
        desired_angle = np.arctan2(total_force_y, total_force_x)
        angle_error = self._normalize_angle(desired_angle) # Error is simply the desired angle

        # Get max speeds with fallbacks to defaults if the agent doesn't provide them
        max_linear_speed = getattr(self.agent, 'max_linear_speed', self.default_max_linear_speed)
        max_angular_speed = getattr(self.agent, 'max_angular_speed', self.default_max_angular_speed)

        # Calculate target angular velocity based on angle error
        target_angular_z = np.clip(angle_error * self.PF_K_ANGULAR, -max_angular_speed, max_angular_speed)

        # Calculate target linear velocity based on the magnitude of the force 
        target_linear_x = np.clip(total_force_x * self.PF_K_LINEAR, 0, max_linear_speed) # Allow only forward

        # Apply Damping
        # Smooth velocity changes to prevent jerky movements
        final_linear_x = (target_linear_x * (1.0 - self.PF_DAMPING_FACTOR) + 
                          self.prev_pf_linear_x * self.PF_DAMPING_FACTOR)
        final_angular_z = (target_angular_z * (1.0 - self.PF_DAMPING_FACTOR) + 
                           self.prev_pf_angular_z * self.PF_DAMPING_FACTOR)

        self.prev_pf_linear_x = final_linear_x
        self.prev_pf_angular_z = final_angular_z

        return final_linear_x, final_angular_z

    def _execute_recovery_jiggle(self):
        """Executes the multi-step recovery maneuver (backward, turn, forward)."""
        linear_x, angular_z = 0.0, 0.0
        now = rospy.Time.now()
        
        if not self.recovery_state_start_time:
            rospy.logerr(f"Agent {self.agent.agent_id}: Recovery jiggle called without start time!")
            self.avoidance_state = 'none' # Reset state to be safe
            self.recovery_state = None
            return 0.0, 0.0

        elapsed_time = now - self.recovery_state_start_time

        # Backward State
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
                # Calculate random turn angle
                turn_magnitude = np.random.uniform(np.deg2rad(60), np.deg2rad(120))
                turn_direction = np.random.choice([-1, 1])
                self.recovery_target_angle = self._normalize_angle(self.agent.get_yaw() + turn_direction * turn_magnitude)
                rospy.loginfo(f"Agent {self.agent.agent_id}: Recovery Jiggle: Target turn angle: {math.degrees(self.recovery_target_angle):.1f} deg")
                linear_x, angular_z = 0.0, 0.0 # Stop briefly before turning
        
        # Turning State
        elif self.recovery_state == 'turning':
            if elapsed_time < self.RECOVERY_TURN_DURATION:
                current_yaw = self.agent.get_yaw()
                yaw_error = self._normalize_angle(self.recovery_target_angle - current_yaw)
                
                # Stop turning if close enough to target angle
                if abs(yaw_error) < np.deg2rad(5): # Angle tolerance
                    rospy.loginfo(f"Agent {self.agent.agent_id}: Recovery Jiggle: Reached target angle. Transitioning to Forward.")
                    self.recovery_state = 'forward'
                    self.recovery_state_start_time = now
                    self.recovery_target_angle = None
                    linear_x, angular_z = 0.0, 0.0 # Stop
                else:
                    # Apply turn speed based on error sign
                    turn_speed_cmd = np.sign(yaw_error) * self.RECOVERY_TURN_SPEED
                    angular_z = turn_speed_cmd
                    linear_x = 0.0
                    rospy.logdebug_throttle(0.5, f"Agent {self.agent.agent_id}: Recovery Jiggle: Turning ({elapsed_time.to_sec():.1f}/{self.RECOVERY_TURN_DURATION.to_sec():.1f}s). Current: {math.degrees(current_yaw):.1f}, Target: {math.degrees(self.recovery_target_angle):.1f}, Error: {math.degrees(yaw_error):.1f}")
            else:
                # Duration expired, force transition to Forward state
                rospy.loginfo(f"Agent {self.agent.agent_id}: Recovery Jiggle: Turning duration expired. Transitioning to Forward.")
                self.recovery_state = 'forward'
                self.recovery_state_start_time = now
                self.recovery_target_angle = None
                linear_x, angular_z = 0.0, 0.0 # Stop briefly before moving forward

        # Forward State
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