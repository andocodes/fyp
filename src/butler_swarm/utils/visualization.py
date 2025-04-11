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
        self.marker_pub = rospy.Publisher('/visualization/markers', MarkerArray, queue_size=10, latch=True)
        self.obstacle_pub = rospy.Publisher('/visualization/obstacles', MarkerArray, queue_size=10, latch=True)

    def visualize_position(self, agent_id, position, orientation):
        """Visualize a single agent's position and orientation."""
        marker_array = MarkerArray()
        now = rospy.Time.now()
        agent_id_int = int(agent_id)

        # Agent Body Marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = now
        marker.ns = "agent_bodies"
        marker.id = agent_id_int
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position and orientation
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        # Size (match URDF)
        marker.scale.x = 0.4  # Diameter
        marker.scale.y = 0.4  # Diameter
        marker.scale.z = 0.1  # Height

        # Color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0)

        marker_array.markers.append(marker)

        # Direction Indicator Marker
        direction = Marker()
        direction.header.frame_id = "world"
        direction.header.stamp = now
        direction.ns = "agent_directions"
        direction.id = agent_id_int
        direction.type = Marker.ARROW
        direction.action = Marker.ADD

        # Position (slightly above agent body)
        direction.pose.position.x = position[0]
        direction.pose.position.y = position[1]
        direction.pose.position.z = position[2] + 0.05
        # Orientation
        direction.pose.orientation.x = orientation[0]
        direction.pose.orientation.y = orientation[1]
        direction.pose.orientation.z = orientation[2]
        direction.pose.orientation.w = orientation[3]

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

        # Publish the markers for this agent
        self.marker_pub.publish(marker_array)

    def visualize_obstacles(self, agent_id, obstacles):
        """Visualize obstacles detected by an agent."""
        if not obstacles:
            return
            
        marker_array = MarkerArray()
        now = rospy.Time.now()
        agent_id_int = int(agent_id)
        
        # Create a single marker with all obstacle points
        marker = Marker()
        marker.header.frame_id = f"agent_{agent_id}/base_link"
        marker.header.stamp = now
        marker.ns = "agent_obstacles"
        marker.id = agent_id_int
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Size
        marker.scale.x = 0.1  # Point size
        marker.scale.y = 0.1  # Point size
        
        # Color
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.7
        marker.lifetime = rospy.Duration(0.5)
        
        # Add all obstacle points
        for obstacle in obstacles:
            point = Point()
            point.x = obstacle[0]
            point.y = obstacle[1]
            point.z = 0.0
            marker.points.append(point)
            
        marker_array.markers.append(marker)
        
        # Publish the obstacles
        self.obstacle_pub.publish(marker_array)

    def visualize_agents(self, agents):
        """Visualize all agents in the swarm (centralized approach)."""
        all_markers = MarkerArray()
        for agent_id, agent_data in agents.items():
            # Create markers for each agent
            pass # Not implemented
        self.marker_pub.publish(all_markers)