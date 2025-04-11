#!/usr/bin/env python3

import numpy as np
import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

# Import the base class
from .avoidance_base import ObstacleAvoidanceBehaviour

class FormationBehaviour(ObstacleAvoidanceBehaviour):
    """Behaviour for arranging agents in a specified formation."""

    def __init__(self, agent):
        """Initialize the Formation behaviour."""
        # Call the base class initializer FIRST
        super(FormationBehaviour, self).__init__(agent)
        # self.agent = agent # Already set by base class

        self.current_formation = None
        self.reference_point = np.array([0.0, 0.0])
        self.formation_scale = rospy.get_param('~formation/scale', 1.0)
        self.k_linear = rospy.get_param('~formation/k_linear', 0.8) # P-gain for linear velocity
        self.k_angular = rospy.get_param('~formation/k_angular', 1.5) # P-gain for angular velocity
        self.target_tolerance = rospy.get_param('~formation/target_tolerance', 0.2) # meters

        # --- Obstacle Avoidance parameters & state are now in the base class --- #

        rospy.loginfo(f"Formation behaviour for agent {self.agent.agent_id} initialized.")

    def set_formation(self, data):
        """Set the formation parameters from data dictionary."""
        formation_type = data.get('formation')
        if not formation_type:
            rospy.logwarn(f"Agent {self.agent.agent_id}: Invalid formation data, missing 'formation' field")
            return
            
        rospy.loginfo(f"Agent {self.agent.agent_id}: Setting formation to {formation_type}")
        self.current_formation = formation_type
        
        # Also handle reference point if provided
        center = data.get('center', [0, 0])
        self.set_reference_point(center[0], center[1])

    def set_reference_point(self, x, y):
        """Set the reference point for the formation center."""
        rospy.loginfo(f"Agent {self.agent.agent_id}: Setting formation reference point to ({x:.2f}, {y:.2f})")
        self.reference_point = np.array([x, y])

    def compute(self):
        """Compute movement commands to achieve and maintain the formation."""
        
        # --- Compute Avoidance / Recovery --- #
        # Call the base class method to check state and compute avoidance velocities
        avoid_linear_x, avoid_angular_z, should_override = self.compute_avoidance()
        if should_override:
             # If compute_avoidance returns override=True, use its velocities
             return avoid_linear_x, avoid_angular_z
        # --- End Avoidance / Recovery --- #

        # --- Normal Formation Logic --- #
        if not self.current_formation:
            rospy.logdebug_throttle(5.0, f"Agent {self.agent.agent_id}: No formation set. Remaining stationary.")
            return 0.0, 0.0

        neighbour_states = self.agent.comm.get_neighbour_states()
        all_agent_ids = sorted([self.agent.agent_id] + list(neighbour_states.keys())) # Ensure consistent ordering
        total_agents = len(all_agent_ids)
        try:
             agent_index = all_agent_ids.index(self.agent.agent_id)
        except ValueError:
             rospy.logerr(f"Agent {self.agent.agent_id}: Could not find self in agent list {all_agent_ids}!")
             return 0.0, 0.0 # Cannot calculate position without index

        # Calculate target position based on formation type
        target_pos = self.reference_point # Default if no valid formation or only 1 agent
        if total_agents > 0:
            if self.current_formation == 'line':
                target_pos = self._line_formation(agent_index, total_agents)
            elif self.current_formation == 'circle':
                target_pos = self._circle_formation(agent_index, total_agents)
            elif self.current_formation == 'grid':
                target_pos = self._grid_formation(agent_index, total_agents)
            elif self.current_formation == 'arrow':
                target_pos = self._arrow_formation(agent_index, total_agents)
            else:
                rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id}: Unknown formation type '{self.current_formation}'. Holding position.")
                # Use reference point as target if formation is unknown
                pass # target_pos is already reference_point
        
        # Calculate movement commands towards target position
        current_pos = self.agent.position[:2]
        direction_vector = target_pos - current_pos
        distance_to_target = np.linalg.norm(direction_vector)

        rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id}: Formation '{self.current_formation}'. Target={target_pos}, Current={current_pos}, Dist={distance_to_target:.2f}")

        # If close enough to target, stop or slow down
        if distance_to_target < self.target_tolerance:
            rospy.logdebug_throttle(5.0, f"Agent {self.agent.agent_id}: Reached target position within tolerance.")
            linear_x = 0.0
            angular_z = 0.0 # Stop turning as well
        else:
            # Calculate desired heading
            desired_heading = math.atan2(direction_vector[1], direction_vector[0])
            current_heading = self.agent.get_yaw()
            angular_diff = self._normalize_angle(desired_heading - current_heading)

            # P-controller for angular velocity
            angular_z = np.clip(angular_diff * self.k_angular, -self.agent.max_angular_speed, self.agent.max_angular_speed)

            # P-controller for linear velocity (reduce speed when turning)
            turn_reduction_factor = max(0.1, 1.0 - abs(angular_diff) / (math.pi / 2.0))
            linear_x = np.clip(distance_to_target * self.k_linear * turn_reduction_factor, 0.0, self.agent.max_linear_speed)

            # Prioritize turning if angular error is large
            ANGLE_THRESHOLD_FOR_TURN_ONLY = np.deg2rad(30)
            if abs(angular_diff) > ANGLE_THRESHOLD_FOR_TURN_ONLY:
                linear_x = 0.0

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
             rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id}: Formation - Obstacle very close ({min_obstacle_dist:.2f}m). Switching to PF.")
             # Use potential field velocity directly, passing the direction to target pos as attraction
             attraction = direction_vector # Use the vector towards the target formation slot
             pf_linear_x, pf_angular_z = self._calculate_potential_field_velocity(attraction_vector=attraction)
             linear_x, angular_z = pf_linear_x, pf_angular_z
             # Trigger full avoidance state if needed
             if min_obstacle_dist < (PF_CLOSE_THRESHOLD + 0.1): 
                 if self.avoidance_state == 'none': # Only transition if not already avoiding/recovering
                     rospy.logwarn(f"Agent {self.agent.agent_id}: Obstacle detected ({min_obstacle_dist:.2f}m) during formation. Entering 'avoiding' state.")
                     self.avoidance_state = 'avoiding'
                     self.pf_avoid_start_time = rospy.Time.now()
                     if self.stuck_start_time is None: self.stuck_start_time = self.pf_avoid_start_time
                     # Stop motion immediately when entering state
                     return 0.0, 0.0
        # --- End Obstacle Check --- #

        rospy.logdebug_throttle(1, f"Agent {self.agent.agent_id}: Formation - Final Vel: l={linear_x:.2f}, a={angular_z:.2f}")
        return linear_x, angular_z

    # --- Formation Calculation Methods --- #
    def _line_formation(self, agent_index, total_agents):
        """Calculate target position for line formation."""
        # Arrange agents along the x-axis relative to the reference point
        offset_x = (agent_index - (total_agents - 1) / 2.0) * self.formation_scale
        target_pos = self.reference_point + np.array([offset_x, 0.0])
        return target_pos

    def _circle_formation(self, agent_index, total_agents):
        """Calculate target position for circle formation."""
        radius = self.formation_scale * (total_agents / (2 * np.pi)) # Adjust radius based on scale and agent count
        angle = 2 * np.pi * agent_index / total_agents
        offset_x = radius * np.cos(angle)
        offset_y = radius * np.sin(angle)
        target_pos = self.reference_point + np.array([offset_x, offset_y])
        return target_pos

    def _grid_formation(self, agent_index, total_agents):
        """Calculate target position for grid formation."""
        # Arrange in a square-like grid
        grid_size = math.ceil(math.sqrt(total_agents))
        row = agent_index // grid_size
        col = agent_index % grid_size
        # Center the grid around the reference point
        offset_x = (col - (grid_size - 1) / 2.0) * self.formation_scale
        offset_y = (row - (grid_size - 1) / 2.0) * self.formation_scale
        target_pos = self.reference_point + np.array([offset_x, offset_y])
        return target_pos

    def _arrow_formation(self, agent_index, total_agents):
        """Calculate target position for arrow formation (V-shape)."""
        if agent_index == 0: # Leader at the tip
            offset_x = 0.0
            offset_y = 0.0
        else:
            # Alternate placing agents on left/right arms
            arm_index = (agent_index + 1) // 2
            side = 1 if agent_index % 2 != 0 else -1 # Right side = 1, Left side = -1
            offset_x = -arm_index * self.formation_scale * 0.707 # Move back along arrow axis
            offset_y = side * arm_index * self.formation_scale * 0.707 # Move out perpendicular
        
        target_pos = self.reference_point + np.array([offset_x, offset_y])
        return target_pos
        
    # --- Remove duplicated helper methods now in base class --- #
    # def _normalize_angle(self, angle):
    # def _calculate_potential_field_velocity(self):
    # def _execute_recovery_jiggle(self):