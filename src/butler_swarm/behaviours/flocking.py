#!/usr/bin/env python3

import numpy as np
import rospy

class FlockingBehaviour:
    """Implementation of Reynolds' Boids algorithm for flocking Behaviour."""

    def __init__(self, agent):
        """Initialize with reference to parent agent."""
        self.agent = agent

        # Behaviour weights
        self.separation_weight = 1.5
        self.alignment_weight = 1.0
        self.cohesion_weight = 1.0

        # Behaviour ranges
        self.separation_range = 1.0
        self.perception_range = 3.0

        # Maximum speeds
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0

    def compute(self):
        """Compute the movement vector based on flocking rules."""
        # --- Reactive Obstacle Avoidance ---
        OBSTACLE_DISTANCE_THRESHOLD = 0.5 # meters in front
        OBSTACLE_WIDTH_THRESHOLD = 0.3  # meters to the side
        TURN_SPEED = self.max_angular_speed
        SAFE_LINEAR_SPEED = 0.1 # Slow down when avoiding

        turn_direction = 0.0
        obstacle_imminent = False
        obstacles_in_front = []

        for obs_x, obs_y in getattr(self.agent, 'obstacles', []):
            if 0 < obs_x < OBSTACLE_DISTANCE_THRESHOLD and abs(obs_y) < OBSTACLE_WIDTH_THRESHOLD:
                obstacle_imminent = True
                obstacles_in_front.append((obs_x, obs_y))
                turn_direction -= np.sign(obs_y) 

        if obstacle_imminent:
            rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id}: Flocking - Obstacle detected, turning!")
            if abs(turn_direction) > 0:
                final_angular_z = np.sign(turn_direction) * TURN_SPEED
            else:
                final_angular_z = TURN_SPEED # Default turn if obstacle dead ahead
            
            final_linear_x = SAFE_LINEAR_SPEED
            return final_linear_x, final_angular_z

        # --- Flocking Logic (Original/Modified) ---
        neighbour_states = self.agent.comm.get_neighbour_states()
        if not neighbour_states:
            rospy.logdebug_throttle(5, f"Agent {self.agent.agent_id}: Flocking - No neighbour states received.")
        else:
            rospy.logdebug_throttle(5, f"Agent {self.agent.agent_id}: Flocking - Received {len(neighbour_states)} neighbour states: {list(neighbour_states.keys())}")
        
        neighbors_in_range = self._get_neighbors_in_range(neighbour_states)
        if not neighbors_in_range:
             rospy.logdebug_throttle(5, f"Agent {self.agent.agent_id}: Flocking - No neighbours in range {self.perception_range}m.")

        if not neighbors_in_range:
            return 0.0, 0.0 

        separation = self._separation(neighbors_in_range)
        alignment = self._alignment(neighbors_in_range)
        cohesion = self._cohesion(neighbors_in_range)
        rospy.logdebug_throttle(1, f"Agent {self.agent.agent_id}: Flocking - Sep:{separation} Align:{alignment} Coh:{cohesion}")

        linear_x = (separation[0] * self.separation_weight +
                    alignment[0] * self.alignment_weight +
                    cohesion[0] * self.cohesion_weight)

        angular_z = (separation[1] * self.separation_weight +
                     alignment[1] * self.alignment_weight +
                     cohesion[1] * self.cohesion_weight)

        # Limit speeds
        linear_x = np.clip(linear_x, -self.max_linear_speed, self.max_linear_speed)
        angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            rospy.logdebug_throttle(1, f"Agent {self.agent.agent_id}: Flocking - Computed Vel: l={linear_x:.2f}, a={angular_z:.2f}")

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
                if distance < self.perception_range:
                    # Store the data dictionary for this neighbour
                    in_range[agent_id] = data 
        return in_range

    def _separation(self, neighbors_in_range):
        """Rule 1: Separation - avoid crowding neighbors."""
        steering = np.zeros(2)
        count = 0
        current_pos_2d = self.agent.position[:2]

        for agent_id, data in neighbors_in_range.items():
            if 'position' in data:
                neighbour_pos_2d = np.array(data['position'])[:2]
                diff = current_pos_2d - neighbour_pos_2d
                distance = np.linalg.norm(diff)

                if distance < self.separation_range and distance > 0:
                    repulsion = diff / (distance * distance)
                    steering += repulsion
                    count += 1
        if count > 0:
            steering /= count

        linear_x = steering[0]
        angular_z = np.arctan2(steering[1], steering[0]) if np.linalg.norm(steering) > 0 else 0
        return linear_x, angular_z

    def _alignment(self, neighbors_in_range):
        """Rule 2: Alignment - steer towards average heading of neighbors."""
        avg_velocity = np.zeros(2)
        count = 0

        for agent_id, data in neighbors_in_range.items():
            if 'velocity' in data:
                avg_velocity += np.array(data['velocity'])[:2]
                count += 1
        if count > 0:
            avg_velocity /= count

        linear_x = avg_velocity[0]
        desired_heading = np.arctan2(avg_velocity[1], avg_velocity[0]) if np.linalg.norm(avg_velocity) > 0 else 0
        _, _, current_heading = self._get_euler_from_quaternion()
        angular_z = self._get_angular_difference(current_heading, desired_heading)
        return linear_x, angular_z

    def _cohesion(self, neighbors_in_range):
        """Rule 3: Cohesion - steer towards center of mass of neighbors."""
        center_of_mass = np.zeros(2)
        count = 0

        for agent_id, data in neighbors_in_range.items():
             if 'position' in data:
                center_of_mass += np.array(data['position'])[:2]
                count += 1
        if count > 0:
            center_of_mass /= count
            direction = center_of_mass - self.agent.position[:2]
            distance = np.linalg.norm(direction)

            if distance > 0:
                direction /= distance  
                linear_x = direction[0] * distance
                desired_heading = np.arctan2(direction[1], direction[0])
                _, _, current_heading = self._get_euler_from_quaternion()
                angular_z = self._get_angular_difference(current_heading, desired_heading)
                return linear_x, angular_z

        return 0.0, 0.0

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