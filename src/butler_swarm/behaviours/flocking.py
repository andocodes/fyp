#!/usr/bin/env python3

import numpy as np
import rospy

# Import the base class
from .avoidance_base import ObstacleAvoidanceBehaviour

class FlockingBehaviour(ObstacleAvoidanceBehaviour): # Inherit from base
    """Implementation of Reynolds' Boids algorithm for flocking Behaviour."""

    def __init__(self, agent):
        """Initialize with reference to parent agent."""
        # Call the base class initializer FIRST
        super(FlockingBehaviour, self).__init__(agent)
        # self.agent = agent # Already set by base class

        # Behaviour weights
        self.separation_weight = rospy.get_param('~flock/separation_weight', 1.5)
        self.alignment_weight = rospy.get_param('~flock/alignment_weight', 1.0)
        self.cohesion_weight = rospy.get_param('~flock/cohesion_weight', 1.0)

        # Behaviour ranges
        self.separation_range = rospy.get_param('~flock/separation_range', 1.0)
        self.perception_range = rospy.get_param('~flock/perception_range', 3.0)

        # Maximum speeds - Define defaults in case agent doesn't have these attributes
        self.default_max_linear_speed = rospy.get_param('~flock/max_linear_speed', 0.5)  # m/s
        self.default_max_angular_speed = rospy.get_param('~flock/max_angular_speed', 1.0)  # rad/s
        
        # --- Obstacle Avoidance parameters & state are now in the base class --- #
        
        rospy.loginfo(f"Flocking behaviour for agent {self.agent.agent_id} initialized.")

    def compute(self):
        """Compute the movement vector based on flocking rules or avoidance."""
        
        # --- Compute Avoidance / Recovery --- #
        # Call the base class method to check state and compute avoidance velocities
        avoid_linear_x, avoid_angular_z, should_override = self.compute_avoidance()
        if should_override:
             # If compute_avoidance returns override=True, use its velocities
             return avoid_linear_x, avoid_angular_z
        # --- End Avoidance / Recovery --- #

        # --- Normal Flocking Logic --- #
        neighbour_states = self.agent.comm.get_neighbour_states()
        # Log neighbour states less frequently
        rospy.logdebug_throttle(5, f"Agent {self.agent.agent_id}: Flocking - Received {len(neighbour_states)} neighbour states: {list(neighbour_states.keys())}")
        
        neighbors_in_range = self._get_neighbors_in_range(neighbour_states)
        if not neighbors_in_range:
             rospy.logdebug_throttle(5, f"Agent {self.agent.agent_id}: Flocking - No neighbours in range {self.perception_range}m. Moving forward slowly.")
             # Default behavior: move forward slowly if no neighbors are detected
             max_linear_speed = getattr(self.agent, 'max_linear_speed', self.default_max_linear_speed)
             return max_linear_speed * 0.2, 0.0 

        # Calculate flocking components
        sep_lin, sep_ang = self._separation(neighbors_in_range)
        ali_lin, ali_ang = self._alignment(neighbors_in_range)
        coh_lin, coh_ang = self._cohesion(neighbors_in_range)
        rospy.logdebug_throttle(1, f"Agent {self.agent.agent_id}: Flocking - Sep:({sep_lin:.2f},{sep_ang:.2f}) Align:({ali_lin:.2f},{ali_ang:.2f}) Coh:({coh_lin:.2f},{coh_ang:.2f})")

        # Calculate steering vectors
        steer_sep = self._separation_vector(neighbors_in_range)
        steer_ali = self._alignment_vector(neighbors_in_range)
        steer_coh = self._cohesion_vector(neighbors_in_range)

        # Weighted sum of steering vectors
        final_steer = (steer_sep * self.separation_weight + 
                       steer_ali * self.alignment_weight + 
                       steer_coh * self.cohesion_weight)

        # Get max speeds with fallback to defaults
        max_linear_speed = getattr(self.agent, 'max_linear_speed', self.default_max_linear_speed)
        max_angular_speed = getattr(self.agent, 'max_angular_speed', self.default_max_angular_speed)

        # Calculate desired angle from the final steering vector
        if np.linalg.norm(final_steer) > 1e-4:
            desired_heading = np.arctan2(final_steer[1], final_steer[0])
            # Calculate angular velocity command
            current_heading = self.agent.get_yaw()
            angular_diff = self._normalize_angle(desired_heading - current_heading)
            # Use a P-controller for angular velocity
            angular_z = np.clip(angular_diff * 1.5, -max_angular_speed, max_angular_speed)
            
            # Calculate linear velocity command
            linear_x = max_linear_speed * 0.8
        else:
            # No significant steering force
            linear_x = 0.1
            angular_z = 0.0

        # Limit speeds
        linear_x = np.clip(linear_x, 0, max_linear_speed)

        # --- Obstacle Check (Fine-grained check before applying final velocity) --- #
        # Check obstacles directly in front using raw scan
        min_obstacle_dist = float('inf')
        ranges = self.agent.last_scan_ranges
        if ranges:
            angle_min = self.agent.last_scan_angle_min
            angle_increment = self.agent.last_scan_angle_increment
            forward_check_angle = np.deg2rad(15) # Check +/- 15 degrees
            for i, r in enumerate(ranges):
                if not np.isfinite(r) or r <= 0:
                    continue
                angle = angle_min + i * angle_increment
                if abs(angle) < forward_check_angle:
                     min_obstacle_dist = min(min_obstacle_dist, r)
        
        PF_CLOSE_THRESHOLD = 0.3 # Threshold to trigger PF override
        if min_obstacle_dist < PF_CLOSE_THRESHOLD:
             rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id}: Flocking - Obstacle very close ({min_obstacle_dist:.2f}m). Switching to PF.")
             # Use potential field velocity directly, passing the combined flocking steer vector as attraction
             attraction = final_steer # Use the combined steering vector from flocking rules
             pf_linear_x, pf_angular_z = self._calculate_potential_field_velocity(attraction_vector=attraction)
             linear_x, angular_z = pf_linear_x, pf_angular_z
             # Trigger full avoidance state if needed
             if min_obstacle_dist < (PF_CLOSE_THRESHOLD + 0.1): 
                 if self.avoidance_state == 'none': # Only transition if not already avoiding/recovering
                     rospy.logwarn(f"Agent {self.agent.agent_id}: Obstacle detected ({min_obstacle_dist:.2f}m) during flocking. Entering 'avoiding' state.")
                     self.avoidance_state = 'avoiding'
                     self.pf_avoid_start_time = rospy.Time.now()
                     if self.stuck_start_time is None: self.stuck_start_time = self.pf_avoid_start_time
                     # Stop motion immediately when entering state
                     return 0.0, 0.0
        # --- End Obstacle Check --- #

        rospy.logdebug_throttle(1, f"Agent {self.agent.agent_id}: Flocking - Final Vel: l={linear_x:.2f}, a={angular_z:.2f}")
        return linear_x, angular_z

    def _get_neighbors_in_range(self, all_neighbour_states):
        """Get list of neighbors within perception range from the provided state dictionary."""
        in_range = {}
        current_pos_2d = self.agent.position[:2]
        for agent_id, data in all_neighbour_states.items():
            # Ensure data has position before processing
            if 'position' in data:
                neighbour_pos = np.array(data['position'])
                distance = np.linalg.norm(neighbour_pos[:2] - current_pos_2d)
                if 0 < distance < self.perception_range: # Ensure distance > 0
                    # Store the data dictionary for this neighbour
                    in_range[agent_id] = data 
        return in_range

    # --- Refactor Flocking Rules to return STEERING VECTORS --- #

    def _separation_vector(self, neighbors_in_range):
        """Rule 1: Calculate separation steering vector."""
        steering = np.zeros(2)
        count = 0
        current_pos_2d = self.agent.position[:2]

        for agent_id, data in neighbors_in_range.items():
            if 'position' in data:
                neighbour_pos_2d = np.array(data['position'])[:2]
                diff = current_pos_2d - neighbour_pos_2d
                distance = np.linalg.norm(diff)

                if 0 < distance < self.separation_range:
                    # Repulsion force vector (points away from neighbor)
                    repulsion = diff / (distance * distance) # Scale by inverse square distance
                    steering += repulsion
                    count += 1
        # Average the steering vector
        if count > 0:
            steering /= count
        return steering

    def _alignment_vector(self, neighbors_in_range):
        """Rule 2: Calculate alignment steering vector."""
        avg_velocity = np.zeros(2)
        count = 0
        for agent_id, data in neighbors_in_range.items():
            if 'velocity' in data:
                # Consider only linear velocity for alignment direction
                avg_velocity += np.array(data['velocity'])[:2]
                count += 1
        
        if count > 0:
            avg_velocity /= count
            # Normalize the average velocity vector to get the desired direction
            norm = np.linalg.norm(avg_velocity)
            if norm > 0:
                 desired_direction = avg_velocity / norm
                 # Steering vector points towards the desired direction 
                 # Magnitude could be scaled by agent's max speed or constant?
                 # Let's return the desired direction vector for now.
                 return desired_direction 
        return np.zeros(2)

    def _cohesion_vector(self, neighbors_in_range):
        """Rule 3: Calculate cohesion steering vector."""
        center_of_mass = np.zeros(2)
        count = 0
        for agent_id, data in neighbors_in_range.items():
             if 'position' in data:
                center_of_mass += np.array(data['position'])[:2]
                count += 1
        
        if count > 0:
            center_of_mass /= count
            # Vector pointing from agent to center of mass
            direction_to_center = center_of_mass - self.agent.position[:2]
            # Normalize the direction vector 
            norm = np.linalg.norm(direction_to_center)
            if norm > 0:
                 # Steering vector points towards the center 
                 # Magnitude could be scaled by distance or constant?
                 # Return the normalized direction vector for now.
                 return direction_to_center / norm
        return np.zeros(2)

    def _separation(self, neighbors_in_range):
        """Rule 1: Separation - avoid crowding neighbors."""
        steer_vector = self._separation_vector(neighbors_in_range)
        
        # Get max speeds with fallback to defaults
        max_linear_speed = getattr(self.agent, 'max_linear_speed', self.default_max_linear_speed)
        max_angular_speed = getattr(self.agent, 'max_angular_speed', self.default_max_angular_speed)
        
        # Convert steering vector to linear and angular components
        lin_vel, ang_vel = 0.0, 0.0
        
        # If we have a meaningful steering vector
        if np.linalg.norm(steer_vector) > 1e-5:
            # Linear speed proportional to vector magnitude
            lin_vel = min(np.linalg.norm(steer_vector) * self.separation_weight, max_linear_speed)
            
            # Calculate angular component from vector direction
            desired_heading = np.arctan2(steer_vector[1], steer_vector[0])
            current_heading = self.agent.get_yaw()
            angular_diff = self._normalize_angle(desired_heading - current_heading)
            ang_vel = np.clip(angular_diff * 1.5, -max_angular_speed, max_angular_speed)
        
        return lin_vel, ang_vel

    def _alignment(self, neighbors_in_range):
        """Rule 2: Alignment - steer towards average heading of neighbors."""
        steer_vector = self._alignment_vector(neighbors_in_range)
        
        # Get max speeds with fallback to defaults
        max_linear_speed = getattr(self.agent, 'max_linear_speed', self.default_max_linear_speed)
        max_angular_speed = getattr(self.agent, 'max_angular_speed', self.default_max_angular_speed)
        
        # Convert steering vector to linear and angular components
        lin_vel, ang_vel = 0.0, 0.0
        
        # If we have a meaningful steering vector
        if np.linalg.norm(steer_vector) > 1e-5:
            # Linear speed proportional to vector magnitude
            lin_vel = min(np.linalg.norm(steer_vector) * self.alignment_weight, max_linear_speed)
            
            # Calculate angular component from vector direction
            desired_heading = np.arctan2(steer_vector[1], steer_vector[0])
            current_heading = self.agent.get_yaw()
            angular_diff = self._normalize_angle(desired_heading - current_heading)
            ang_vel = np.clip(angular_diff * 1.5, -max_angular_speed, max_angular_speed)
        
        return lin_vel, ang_vel

    def _cohesion(self, neighbors_in_range):
        """Rule 3: Cohesion - steer towards center of mass of neighbors."""
        steer_vector = self._cohesion_vector(neighbors_in_range)
        
        # Get max speeds with fallback to defaults
        max_linear_speed = getattr(self.agent, 'max_linear_speed', self.default_max_linear_speed)
        max_angular_speed = getattr(self.agent, 'max_angular_speed', self.default_max_angular_speed)
        
        # Convert steering vector to linear and angular components
        lin_vel, ang_vel = 0.0, 0.0
        
        # If we have a meaningful steering vector
        if np.linalg.norm(steer_vector) > 1e-5:
            # Linear speed proportional to vector magnitude
            lin_vel = min(np.linalg.norm(steer_vector) * self.cohesion_weight, max_linear_speed)
            
            # Calculate angular component from vector direction
            desired_heading = np.arctan2(steer_vector[1], steer_vector[0])
            current_heading = self.agent.get_yaw()
            angular_diff = self._normalize_angle(desired_heading - current_heading)
            ang_vel = np.clip(angular_diff * 1.5, -max_angular_speed, max_angular_speed)
        
        return lin_vel, ang_vel

    # --- Remove duplicated helper methods now in base class --- #
    # def _get_euler_from_quaternion(self):
    # def _normalize_angle(self, angle):
    # def _calculate_potential_field_velocity(self):
    # def _execute_recovery_jiggle(self):