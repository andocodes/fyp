#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

class SwarmVisualization:
    """Handles visualization of swarm agents and their Behaviours."""

    def __init__(self):
        """Initialize the visualization module."""
        # Publishers
        # Use latch=True so RViz gets the last message even if it starts later
        self.marker_pub = rospy.Publisher('/visualization/markers', MarkerArray, queue_size=10, latch=True)
        # Path visualization can be intensive, maybe use a separate publisher or topic if needed
        # self.agent_path_pub = rospy.Publisher('/visualization/agent_paths', MarkerArray, queue_size=10, latch=True)

        # Tracking agent paths (if desired, keep limited length)
        # self.agent_paths = {}
        # self.max_path_length = 100

    def visualize_single_agent(self, agent_id, agent_data):
        """Visualize a single agent by publishing markers."""
        marker_array = MarkerArray()
        now = rospy.Time.now()
        agent_id_int = int(agent_id)

        # --- Agent Body Marker ---
        marker = Marker()
        marker.header.frame_id = "world" # Assuming world frame
        marker.header.stamp = now
        marker.ns = "agent_bodies" # Namespace for bodies
        marker.id = agent_id_int
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD # Add or modify marker

        # Position and orientation from agent_data
        marker.pose.position.x = agent_data['position'][0]
        marker.pose.position.y = agent_data['position'][1]
        marker.pose.position.z = agent_data['position'][2]
        marker.pose.orientation.x = agent_data['orientation'][0]
        marker.pose.orientation.y = agent_data['orientation'][1]
        marker.pose.orientation.z = agent_data['orientation'][2]
        marker.pose.orientation.w = agent_data['orientation'][3]

        # Size (match URDF)
        marker.scale.x = 0.4  # Diameter
        marker.scale.y = 0.4  # Diameter
        marker.scale.z = 0.1  # Height

        # Color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0) # Auto-delete if not updated for 1 sec

        marker_array.markers.append(marker)

        # --- Direction Indicator Marker ---
        direction = Marker()
        direction.header.frame_id = "world"
        direction.header.stamp = now
        direction.ns = "agent_directions" # Namespace for directions
        direction.id = agent_id_int # Use same ID, different namespace
        direction.type = Marker.ARROW
        direction.action = Marker.ADD

        # Position (slightly above agent body)
        direction.pose.position.x = agent_data['position'][0]
        direction.pose.position.y = agent_data['position'][1]
        direction.pose.position.z = agent_data['position'][2] + 0.05
        # Orientation (same as agent body)
        direction.pose.orientation.x = agent_data['orientation'][0]
        direction.pose.orientation.y = agent_data['orientation'][1]
        direction.pose.orientation.z = agent_data['orientation'][2]
        direction.pose.orientation.w = agent_data['orientation'][3]

        # Size
        direction.scale.x = 0.3  # Length
        direction.scale.y = 0.05 # Width
        direction.scale.z = 0.05 # Height

        # Color
        direction.color.r = 1.0
        direction.color.g = 0.0
        direction.color.b = 0.0
        direction.color.a = 1.0
        direction.lifetime = rospy.Duration(1.0)

        marker_array.markers.append(direction)

        # --- (Optional) Path Marker Update ---
        # self._update_agent_path(agent_id, agent_data['position'])
        # path_marker = self._create_path_marker(agent_id)
        # if path_marker:
        #     marker_array.markers.append(path_marker)

        # Publish the markers for this agent
        self.marker_pub.publish(marker_array)


    # --- Centralized visualization method (Keep for potential future use) ---
    def visualize_agents(self, agents):
        """Visualize all agents in the swarm (centralized approach)."""
        all_markers = MarkerArray()
        for agent_id, agent_data in agents.items():
            # You could reuse parts of visualize_single_agent logic here
            # to generate markers for each agent and append to all_markers
            pass # Placeholder
        self.marker_pub.publish(all_markers)

    # --- Path Visualization (Optional) ---
    # def _update_agent_path(self, agent_id, position):
    #     # ... (implementation as before or improved)

    # def _create_path_marker(self, agent_id):
    #     # ... (create a LINE_STRIP marker from self.agent_paths[agent_id])
    #     return path_marker

    # def _visualize_agent_paths(self): # Likely call this from visualize_agents
    #     # ... (publish all path markers)