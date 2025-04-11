#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf.transformations
import json
import tf2_ros
import tf2_geometry_msgs

# Import behaviours (use relative imports for ROS package structure)
from .behaviours.flocking import FlockingBehaviour
from .behaviours.search import SearchBehaviour
from .behaviours.formation import FormationBehaviour
from .behaviours.explore import ExploreBehaviour
from .behaviours.idle import IdleBehaviour

# Import utilities (use relative imports)
from .utils.communication import SwarmCommunication
from .utils.visualization import SwarmVisualization

class SwarmAgent:
    """Base class for swarm agents in Project Butler."""

    def __init__(self, agent_id):
        """Initialize the agent with a unique ID."""
        self.agent_id = agent_id
        self.agent_name = f"agent_{agent_id}"

        # Set maximum speeds
        self.max_linear_speed = rospy.get_param(f'~agent_{agent_id}/max_linear_speed', 0.5)  # m/s
        self.max_angular_speed = rospy.get_param(f'~agent_{agent_id}/max_angular_speed', 1.0)  # rad/s

        # TF Buffer and Listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.base_frame = f"{self.agent_name}/base_link"
        self.laser_frame = f"{self.agent_name}/laser"

        # Position and orientation
        self.position = np.zeros(3)  # x, y, z in odom frame
        self.orientation = np.zeros(4)  # quaternion x, y, z, w in odom frame
        self.velocity = np.zeros(3)

        # Latest LaserScan data storage
        self.last_scan_time = rospy.Time(0)
        self.last_scan_ranges = []
        self.last_scan_angle_min = 0.0
        self.last_scan_angle_increment = 0.0
        self.last_scan_range_min = 0.0
        self.last_scan_range_max = 0.0
        self.obstacles = [] 

        # Communication system
        self.comm = SwarmCommunication(agent_id)

        # Visualization system
        self.vis = SwarmVisualization()

        # Publishers
        self.cmd_vel_pub = rospy.Publisher(f'/{self.agent_name}/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber(f'/{self.agent_name}/odom', Odometry, self.odom_callback)
        rospy.Subscriber(f'/{self.agent_name}/scan', LaserScan, self.scan_callback)

        # Subscribe to swarm commands
        self.comm.register_callback('behaviour', self.behaviour_callback)
        self.comm.register_callback('formation', self.formation_callback)

        # Initialize behaviours
        self.behaviours = {
            'idle': IdleBehaviour(self),
            'flocking': FlockingBehaviour(self),
            'search': SearchBehaviour(self),
            'formation': FormationBehaviour(self),
            'explore': ExploreBehaviour(self)
        }

        # Current behaviour state
        self.current_behaviour = 'idle'

        rospy.loginfo(f"Agent {self.agent_id} ({self.agent_name}) initialized. Default behaviour: {self.current_behaviour}.")


    def odom_callback(self, msg):
        """Process odometry data to update position and orientation."""
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w

        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.linear.y
        self.velocity[2] = msg.twist.twist.linear.z

    def get_yaw(self):
        """Calculate the current yaw angle (rotation around Z) from the orientation quaternion."""
        quaternion = (
            self.orientation[0],
            self.orientation[1],
            self.orientation[2],
            self.orientation[3]
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def scan_callback(self, msg):
        """Process laser scan data, storing raw data and calculating obstacles."""
        # Store raw scan data for behaviours like Explore
        self.last_scan_time = msg.header.stamp
        self.last_scan_ranges = list(msg.ranges)
        self.last_scan_angle_min = msg.angle_min
        self.last_scan_angle_increment = msg.angle_increment
        self.last_scan_range_min = msg.range_min
        self.last_scan_range_max = msg.range_max

        # Get current positions of neighbours in the odom frame
        neighbour_states = self.comm.get_neighbour_states()
        neighbour_positions_odom = []
        for agent_id, state_data in neighbour_states.items():
            pos = state_data.get('position')
            if pos:
                neighbour_positions_odom.append(np.array(pos[:2]))

        agent_filter_radius_sq = 0.4**2
        odom_frame_name = f"{self.agent_name}/odom"

        # Recalculate obstacles in the agent's base_frame
        self.obstacles = []
        forward_arc_limit_rad = np.deg2rad(60)

        for i, distance in enumerate(msg.ranges):
            # Filter out invalid range readings
            if self.last_scan_range_min < distance < self.last_scan_range_max:                
                current_angle = msg.angle_min + i * msg.angle_increment

                # Point in laser frame
                x_laser = distance * np.cos(current_angle)
                y_laser = distance * np.sin(current_angle)

                # Create PointStamped message in laser frame
                point_laser = PointStamped()
                point_laser.header.stamp = msg.header.stamp
                point_laser.header.frame_id = self.laser_frame
                point_laser.point.x = x_laser
                point_laser.point.y = y_laser
                point_laser.point.z = 0.0

                is_other_agent = False
                try:
                    # Transform point to ODOM frame for comparison with neighbors
                    point_odom = self.tf_buffer.transform(point_laser, odom_frame_name, timeout=rospy.Duration(0.05))
                    point_world_pos = np.array([point_odom.point.x, point_odom.point.y])
                    
                    # Check distance to each neighbor
                    for neighbor_pos in neighbour_positions_odom:
                        dist_sq = np.sum((point_world_pos - neighbor_pos)**2)
                        if dist_sq < agent_filter_radius_sq:
                            is_other_agent = True
                            rospy.logdebug(f"Agent {self.agent_id} DBG ScanCB FilterAgent: Point {i} (World: {point_world_pos[0]:.2f},{point_world_pos[1]:.2f}) filtered as agent near {neighbor_pos}. DistSq={dist_sq:.2f}")
                            break
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e_odom:
                     rospy.logwarn_throttle(10.0, f"Agent {self.agent_id} ScanCB: TF transform failed from {self.laser_frame} to {odom_frame_name}: {e_odom}")
                     rospy.logwarn(f"Agent {self.agent_id} DBG ScanCB TF Fail Odom: Laser->Odom lookup failed for agent filtering. PointIdx={i}. Error: {e_odom}")
                     pass 

                if not is_other_agent:
                    try:
                        # Transform point to BASE frame for obstacle list
                        point_base = self.tf_buffer.transform(point_laser, self.base_frame, timeout=rospy.Duration(0.1))
                        tx = point_base.point.x
                        ty = point_base.point.y
                        # Store relative coordinates in base_frame
                        self.obstacles.append((tx, ty))

                        rospy.logdebug(
                             f"Agent {self.agent_id} DBG ScanCB AddObstacle: PointIdx={i}, Laser=({x_laser:.2f},{y_laser:.2f}), Base=({tx:.2f},{ty:.2f})"
                        )

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e_base:
                         rospy.logwarn_throttle(5.0, f"Agent {self.agent_id} ScanCB: TF transform failed from {self.laser_frame} to {self.base_frame}: {e_base}")
                         rospy.logwarn(f"Agent {self.agent_id} DBG ScanCB TF Fail Base: Laser->Base lookup failed for obstacle list. PointIdx={i}. Error: {e_base}")

    def behaviour_callback(self, data):
        """Process behaviour commands received from swarm controller."""
        try:
            new_behaviour = data.get('behaviour')
            if new_behaviour and new_behaviour in self.behaviours:
                if self.current_behaviour != new_behaviour:
                    rospy.loginfo(f"Agent {self.agent_id} switching from {self.current_behaviour} to {new_behaviour} behaviour")
                    
                    if self.current_behaviour == 'explore':
                        if hasattr(self.behaviours['explore'], 'stop'):
                            self.behaviours['explore'].stop()
                    
                    # Set the new behaviour
                    self.current_behaviour = new_behaviour
                    
                    if self.current_behaviour == 'explore':
                        if hasattr(self.behaviours['explore'], 'start'):
                            self.behaviours['explore'].start()
        except Exception as e:
            rospy.logerr(f"Agent {self.agent_id} Error in behaviour_callback: {e}")

    def formation_callback(self, data):
        """Process formation commands received from swarm controller."""
        try:
            if self.current_behaviour == 'formation':
                formation_type = data.get('formation')
                if formation_type and hasattr(self.behaviours['formation'], 'set_formation'):
                    rospy.loginfo(f"Agent {self.agent_id} switching to {formation_type} formation")
                    # Pass all data to formation.set_formation for handling there
                    self.behaviours['formation'].set_formation(data)
        except Exception as e:
            rospy.logerr(f"Agent {self.agent_id} Error in formation_callback: {e}")

    def publish_state(self):
        """Publish agent state information for other agents to use."""
        state = {
            'position': self.position.tolist(),
            'orientation': self.orientation.tolist(),
            'velocity': self.velocity.tolist(),
            'behaviour': self.current_behaviour
        }
        self.comm.publish_state(state)

    def share_data(self, topic, data):
        """Share generic data with other agents."""
        self.comm.publish(topic, data)

    def move(self, linear_x, angular_z):
        """Send movement commands to the robot."""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)

    def update(self):
        """Main update loop for agent behavior."""
        
        # Get velocities from current behaviour
        linear_x, angular_z = 0.0, 0.0
        try:
            if self.current_behaviour in self.behaviours:
                linear_x, angular_z = self.behaviours[self.current_behaviour].compute()
            else:
                rospy.logwarn_throttle(5.0, f"Agent {self.agent_id}: Unknown behaviour '{self.current_behaviour}', using idle")
                self.current_behaviour = 'idle'
                linear_x, angular_z = self.behaviours['idle'].compute()
        except Exception as e:
            rospy.logerr(f"Agent {self.agent_id} Error in behaviour compute: {e}")
            # Fall back to idle on error
            linear_x, angular_z = 0.0, 0.0

        # Send movement commands
        self.move(linear_x, angular_z)
        
        # Publish state for other agents
        self.publish_state()
        
        # Visualize agent state
        self._visualize()

    def _visualize(self):
        """Visualize agent state in RViz."""
        try:
            # Visualize agent pose
            self.vis.visualize_position(self.agent_id, self.position, self.orientation)
            # Visualize obstacles (from scan)
            self.vis.visualize_obstacles(self.agent_id, self.obstacles)
        except Exception as e:
            rospy.logerr(f"Agent {self.agent_id} Error in visualization: {e}")

    def stop(self):
        """Clean shutdown."""
        # Stop the robot
        self.move(0, 0)
        rospy.loginfo(f"Agent {self.agent_id} stopped.")

# Note: The old if __name__ == '__main__': block is removed.
# This file is now intended to be imported as a module, not run directly.
# The executable logic is in nodes/agent_node.py. 