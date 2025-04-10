#!/usr/bin/env python3

import numpy as np
import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class FormationBehaviour:
    """Dynamic pattern formation for swarm agents."""

    def __init__(self, agent):
        """Initialize with reference to parent agent."""
        self.agent = agent

        # Available formations
        self.formations = {
            'line': self._line_formation,
            'roundtable': self._circle_formation,
            'grid': self._grid_formation,
            'arrow': self._arrow_formation
        }

        # Current formation
        self.current_formation = 'roundtable'

        # Formation parameters
        self.formation_scale = 2.0  # Size of formation
        self.spacing = 1.0  # Spacing between agents

        # Reference point (center of formation)
        self.reference_point = np.zeros(2)

        # Movement parameters
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        self.position_tolerance = 0.1

        # Visualization
        self.vis_pub = rospy.Publisher('/visualization/formation', MarkerArray, queue_size=10)

    def set_formation(self, formation_name):
        """Set the current formation pattern."""
        if formation_name in self.formations:
            self.current_formation = formation_name
            rospy.loginfo(f"Agent {self.agent.agent_id} switching to {formation_name} formation")
        else:
            rospy.logerr(f"Formation {formation_name} not available")

    def set_reference_point(self, point):
        """Set the reference point for the formation."""
        self.reference_point = np.array(point)

    def compute(self):
        """Compute the movement vector based on formation position."""
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
            rospy.logdebug_throttle(1.0, f"Agent {self.agent.agent_id}: Formation - Obstacle detected, turning!")
            if abs(turn_direction) > 0:
                final_angular_z = np.sign(turn_direction) * TURN_SPEED
            else:
                final_angular_z = TURN_SPEED # Default turn if obstacle dead ahead
            
            final_linear_x = SAFE_LINEAR_SPEED
            return final_linear_x, final_angular_z

        # --- Formation Logic (Original) ---
        # Get target position based on current formation
        target_position = self._get_formation_position()

        # Visualize formation
        self._visualize_formation()

        # Move towards target position
        return self._move_to_target(target_position)

    def _get_formation_position(self):
        """Get the target position for this agent in the current formation."""
        # Call the appropriate formation function
        formation_func = self.formations[self.current_formation]
        return formation_func()

    def _line_formation(self):
        """Calculate position in a line formation."""
        # Get neighbour states and include self
        neighbour_states = self.agent.comm.get_neighbour_states()
        agent_ids = sorted(list(neighbour_states.keys()) + [self.agent.agent_id])
        position = agent_ids.index(self.agent.agent_id)
        total = len(agent_ids)

        # Calculate offset from center
        offset = position - (total - 1) / 2

        # Calculate position
        x = self.reference_point[0]
        y = self.reference_point[1] + offset * self.spacing

        return np.array([x, y])

    def _circle_formation(self):
        """Calculate position in a circle formation."""
        # Get neighbour states and include self
        neighbour_states = self.agent.comm.get_neighbour_states()
        agent_ids = sorted(list(neighbour_states.keys()) + [self.agent.agent_id])
        position = agent_ids.index(self.agent.agent_id)
        total = len(agent_ids)

        # Calculate angle
        angle = 2 * np.pi * position / total

        # Calculate position
        radius = self.formation_scale
        x = self.reference_point[0] + radius * np.cos(angle)
        y = self.reference_point[1] + radius * np.sin(angle)

        return np.array([x, y])

    def _grid_formation(self):
        """Calculate position in a grid formation."""
        # Get neighbour states and include self
        neighbour_states = self.agent.comm.get_neighbour_states()
        agent_ids = sorted(list(neighbour_states.keys()) + [self.agent.agent_id])
        position = agent_ids.index(self.agent.agent_id)
        total = len(agent_ids)

        # Calculate grid dimensions
        grid_size = int(np.ceil(np.sqrt(total)))

        # Calculate position in grid
        row = position // grid_size
        col = position % grid_size

        # Calculate offset from center
        offset_x = col - (grid_size - 1) / 2
        offset_y = row - (grid_size - 1) / 2

        # Calculate position
        x = self.reference_point[0] + offset_x * self.spacing
        y = self.reference_point[1] + offset_y * self.spacing

        return np.array([x, y])

    def _arrow_formation(self):
        """Calculate position in an arrow formation."""
        # Get neighbour states and include self
        neighbour_states = self.agent.comm.get_neighbour_states()
        agent_ids = sorted(list(neighbour_states.keys()) + [self.agent.agent_id])
        position = agent_ids.index(self.agent.agent_id)
        total = len(agent_ids)

        # Define arrow shape
        arrow_positions = []

        # Add the arrow head
        arrow_positions.append([0, 0])

        # Add the arrow shaft
        shaft_length = min(5, total - 1)  # Use at most 5 agents for the shaft
        for i in range(shaft_length):
            arrow_positions.append([-(i+1) * self.spacing, 0])

        # Add the arrow fins
        if total > shaft_length + 1:
            fins = min(4, total - shaft_length - 1)  # Use at most 4 agents for fins
            for i in range(fins):
                if i % 2 == 0:  # Left fin
                    arrow_positions.append([-(i//2 + 1) * self.spacing, (i//2 + 1) * self.spacing])
                else:  # Right fin
                    arrow_positions.append([-(i//2 + 1) * self.spacing, -(i//2 + 1) * self.spacing])

        # Add any remaining agents behind the arrow
        for i in range(len(arrow_positions), total):
            row = (i - len(arrow_positions)) // 3 + shaft_length + 1
            col = (i - len(arrow_positions)) % 3 - 1
            arrow_positions.append([-row * self.spacing, col * self.spacing])

        # If we don't have enough positions defined, default to a circle
        if position >= len(arrow_positions):
            return self._circle_formation()

        # Get position for this agent
        pos = arrow_positions[position]

        # Rotate to face forward (arrow points along x-axis)
        angle = 0  # Could make this dynamic
        rotated_x = pos[0] * np.cos(angle) - pos[1] * np.sin(angle)
        rotated_y = pos[0] * np.sin(angle) + pos[1] * np.cos(angle)

        # Add reference point
        x = self.reference_point[0] + rotated_x
        y = self.reference_point[1] + rotated_y

        return np.array([x, y])

    def _move_to_target(self, target_position):
        """Generate movement commands to reach the target position."""
        # Get current position and orientation
        current_pos = self.agent.position[:2]
        _, _, current_heading = self._get_euler_from_quaternion()

        # Vector to target
        direction = target_position - current_pos
        distance = np.linalg.norm(direction)

        # If we're already at the target position
        if distance < self.position_tolerance:
            return 0.0, 0.0

        # Normalize direction
        if distance > 0:
            direction /= distance

        # Calculate desired heading
        desired_heading = np.arctan2(direction[1], direction[0])

        # Calculate angular difference (shortest path)
        angular_diff = self._get_angular_difference(current_heading, desired_heading)

        # Set linear and angular velocity
        linear_x = min(self.max_linear_speed, distance)
        angular_z = np.clip(angular_diff, -self.max_angular_speed, self.max_angular_speed)

        return linear_x, angular_z

    def _visualize_formation(self):
        """Visualize the formation patterns."""
        marker_array = MarkerArray()

        # Get neighbour states and include self
        neighbour_states = self.agent.comm.get_neighbour_states()
        agent_ids = sorted(list(neighbour_states.keys()) + [self.agent.agent_id])

        for i, agent_id in enumerate(agent_ids):
            # Get position for this agent in current formation
            temp_agent_id = self.agent.agent_id
            self.agent.agent_id = agent_id
            position = self._get_formation_position()
            self.agent.agent_id = temp_agent_id

            # Create marker for target position
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "formation_targets"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.05

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            # Color based on whether it's for this agent
            marker.color = ColorRGBA()
            if agent_id == self.agent.agent_id:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
            marker.color.a = 0.5

            marker_array.markers.append(marker)

        # Create lines connecting formation points
        line_marker = Marker()
        line_marker.header.frame_id = "world"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "formation_lines"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.scale.x = 0.05  # Line width

        line_marker.color.r = 0.0
        line_marker.color.g = 0.5
        line_marker.color.b = 1.0
        line_marker.color.a = 0.5

        # Add points for each agent's target position
        for i, agent_id in enumerate(agent_ids):
            temp_agent_id = self.agent.agent_id
            self.agent.agent_id = agent_id
            position = self._get_formation_position()
            self.agent.agent_id = temp_agent_id

            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = 0.05

            line_marker.points.append(point)

        # Close the loop for circle formation
        if self.current_formation == 'roundtable' and len(agent_ids) > 2:
            temp_agent_id = self.agent.agent_id
            self.agent.agent_id = agent_ids[0]
            position = self._get_formation_position()
            self.agent.agent_id = temp_agent_id

            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = 0.05

            line_marker.points.append(point)

        marker_array.markers.append(line_marker)

        # Publish marker array
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