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
from .behaviours.flocking import FlockingBehaviour # Check spelling here if needed
from .behaviours.search import SearchBehaviour     # Check spelling here if needed
from .behaviours.formation import FormationBehaviour # Check spelling here if needed
from .behaviours.explore import ExploreBehaviour # <-- Import ExploreBehaviour
from .behaviours.idle import IdleBehaviour # <-- Import IdleBehaviour

# Import utilities (use relative imports)
from .utils.communication import SwarmCommunication
from .utils.visualization import SwarmVisualization

class SwarmAgent:
    """Base class for swarm agents in Project Butler."""

    def __init__(self, agent_id):
        """Initialize the agent with a unique ID."""
        self.agent_id = agent_id
        self.agent_name = f"agent_{agent_id}"

        # TF Buffer and Listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.base_frame = f"{self.agent_name}/base_link" # Define base frame
        self.laser_frame = f"{self.agent_name}/laser"   # Define laser frame

        # Initialize ROS node (do this only ONCE per process)
        # Note: The agent_node.py script should ideally handle the init_node call
        # if multiple agents are meant to run in separate processes.
        # If spawn_agents.py launches each agent_node.py separately, this is fine.
        # Let's comment out the init_node here FOR NOW, assuming agent_node.py handles it.
        # rospy.init_node(self.agent_name, anonymous=True) # Defer init_node to the caller script

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
        # Keep self.obstacles for now, might be useful for other behaviours
        self.obstacles = [] 

        # Communication system
        self.comm = SwarmCommunication(agent_id)

        # Visualization system
        # Ensure SwarmVisualization doesn't also init_node if agent_node.py does
        self.vis = SwarmVisualization() # Removed agent_id if not needed by Vis constructor

        # Neighboring agents data store - Data now stored within self.comm
        # self.neighbors = {}

        # Shared data received via communication - Handled by comm
        # self.shared_data = {}

        # Publishers
        self.cmd_vel_pub = rospy.Publisher(f'/{self.agent_name}/cmd_vel', Twist, queue_size=10)

        # Subscribers
        # Use namespaced topics based on the agent's name
        rospy.Subscriber(f'/{self.agent_name}/odom', Odometry, self.odom_callback)
        rospy.Subscriber(f'/{self.agent_name}/scan', LaserScan, self.scan_callback)

        # Subscribe to swarm commands (uses register_callback)
        self.comm.register_callback('behaviour', self.behaviour_callback)
        self.comm.register_callback('formation', self.formation_callback)

        # Initialize behaviours
        self.behaviours = {
            'idle': IdleBehaviour(self), # <-- Add IdleBehaviour
            'flocking': FlockingBehaviour(self), # Pass self, behaviours access neighbours via self.agent.comm.get_neighbour_states()
            'search': SearchBehaviour(self),
            'formation': FormationBehaviour(self),
            'explore': ExploreBehaviour(self) # <-- Add ExploreBehaviour instance
        }

        # Current behaviour state - Set default to idle
        self.current_behaviour = 'idle' # <-- Change default

        # Register communication callbacks - State callback is now internal to comm
        # self.comm.register_callback('state', self.state_callback) 

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
        # Ensure orientation is in [x, y, z, w] format for euler_from_quaternion
        quaternion = (
            self.orientation[0],
            self.orientation[1],
            self.orientation[2],
            self.orientation[3]
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # euler is returned as (roll, pitch, yaw)
        return euler[2]

    def scan_callback(self, msg):
        """Process laser scan data, storing raw data and calculating obstacles."""
        # Store raw scan data for behaviours like Explore
        self.last_scan_time = msg.header.stamp
        self.last_scan_ranges = list(msg.ranges) # Store as list
        self.last_scan_angle_min = msg.angle_min
        self.last_scan_angle_increment = msg.angle_increment
        self.last_scan_range_min = msg.range_min
        self.last_scan_range_max = msg.range_max

        # --- Filter Agents from Obstacles --- #
        # Get current positions of neighbours in the odom frame
        neighbour_states = self.comm.get_neighbour_states()
        neighbour_positions_odom = []
        for agent_id, state_data in neighbour_states.items():
            pos = state_data.get('position')
            if pos:
                neighbour_positions_odom.append(np.array(pos[:2])) # Store as list of numpy arrays [x, y]

        agent_filter_radius_sq = 0.4**2 # squared distance threshold (e.g., 0.4m radius around neighbors)
        odom_frame_name = f"{self.agent_name}/odom" # Agent's odom frame
        # ------------------------------------ #

        # Recalculate obstacles in the agent's base_frame
        self.obstacles = [] # Reset obstacles each scan
        forward_arc_limit_rad = np.deg2rad(60) # +/- 60 degrees for logging

        for i, distance in enumerate(msg.ranges):
            # Filter out invalid range readings
            if self.last_scan_range_min < distance < self.last_scan_range_max:                
                current_angle = msg.angle_min + i * msg.angle_increment

                # Point in laser frame
                x_laser = distance * np.cos(current_angle)
                y_laser = distance * np.sin(current_angle)

                # Create PointStamped message in laser frame
                point_laser = PointStamped()
                point_laser.header.stamp = msg.header.stamp # Use scan timestamp
                point_laser.header.frame_id = self.laser_frame
                point_laser.point.x = x_laser
                point_laser.point.y = y_laser
                point_laser.point.z = 0.0 # Assuming laser is planar

                # --- Check if point corresponds to another agent --- #
                is_other_agent = False
                try:
                    # Transform point to ODOM frame for comparison with neighbors
                    point_odom = self.tf_buffer.transform(point_laser, odom_frame_name, timeout=rospy.Duration(0.05)) # Shorter timeout
                    point_world_pos = np.array([point_odom.point.x, point_odom.point.y])
                    
                    # Check distance to each neighbor
                    for neighbor_pos in neighbour_positions_odom:
                        dist_sq = np.sum((point_world_pos - neighbor_pos)**2)
                        if dist_sq < agent_filter_radius_sq:
                            is_other_agent = True
                            # --- DEBUG LOG ---
                            rospy.logdebug(f"Agent {self.agent_id} DBG ScanCB FilterAgent: Point {i} (World: {point_world_pos[0]:.2f},{point_world_pos[1]:.2f}) filtered as agent near {neighbor_pos}. DistSq={dist_sq:.2f}")
                            # ---------------
                            break # Stop checking neighbors for this point
                
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e_odom:
                     rospy.logwarn_throttle(10.0, f"Agent {self.agent_id} ScanCB: TF transform failed from {self.laser_frame} to {odom_frame_name}: {e_odom}")
                     # --- DEBUG LOG ---
                     rospy.logwarn(f"Agent {self.agent_id} DBG ScanCB TF Fail Odom: Laser->Odom lookup failed for agent filtering. PointIdx={i}. Error: {e_odom}")
                     # ---------------
                     # If we can't transform to odom, we can't filter, so treat as potential obstacle (conservative)
                     pass 
                # ---------------------------------------------------- #

                # --- If NOT another agent, add to obstacles list --- # 
                if not is_other_agent:
                    try:
                    # Transform point to BASE frame for obstacle list
                        point_base = self.tf_buffer.transform(point_laser, self.base_frame, timeout=rospy.Duration(0.1))
                        tx = point_base.point.x
                        ty = point_base.point.y
                        # Store relative coordinates in base_frame
                        self.obstacles.append((tx, ty))

                        # Debug logging for points in the forward arc
                        # --- DEBUG LOG ---
                        # Log only non-filtered points added as obstacles
                        rospy.logdebug(
                             f"Agent {self.agent_id} DBG ScanCB AddObstacle: PointIdx={i}, Laser=({x_laser:.2f},{y_laser:.2f}), Base=({tx:.2f},{ty:.2f})"
                        )
                        # ---------------

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e_base:
                         rospy.logwarn_throttle(5.0, f"Agent {self.agent_id} ScanCB: TF transform failed from {self.laser_frame} to {self.base_frame}: {e_base}")
                         # --- DEBUG LOG ---
                         rospy.logwarn(f"Agent {self.agent_id} DBG ScanCB TF Fail Base: Laser->Base lookup failed for obstacle list. PointIdx={i}. Error: {e_base}")
                         # ---------------

    def behaviour_callback(self, data): # Now receives parsed data directly
        """Process behaviour commands received from swarm controller."""
        try:
            behaviour = data.get('behaviour')
            if behaviour and behaviour in self.behaviours:
                if self.current_behaviour != behaviour:
                    self.current_behaviour = behaviour
                    rospy.loginfo(f"Agent {self.agent_id} switching to {behaviour} behaviour")
        except Exception as e:
            rospy.logerr(f"Error processing behaviour command: {e}")

    def formation_callback(self, data): # Now receives parsed data directly
        """Process formation commands."""
        try:
            if self.current_behaviour == 'formation':
                formation_name = data.get('formation')
                center = data.get('center', [0, 0])

                formation_behaviour = self.behaviours['formation']
                if hasattr(formation_behaviour, 'set_formation'):
                    formation_behaviour.set_formation(formation_name)
                if hasattr(formation_behaviour, 'set_reference_point'):
                    formation_behaviour.set_reference_point(center)

                rospy.loginfo(f"Agent {self.agent_id} updating formation to {formation_name} around {center}")
        except Exception as e:
            rospy.logerr(f"Error processing formation command: {e}")

    def publish_state(self):
        """Publish this agent's state for other agents via communication."""
        state_data = {
            # agent_id added automatically by self.comm.publish
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist()
            # Add orientation or other relevant state if needed by neighbours
        }
        self.comm.publish('state', state_data) # Assumes comm uses 'state' topic

    def share_data(self, topic, data):
        """Share arbitrary data with other agents (e.g., map data)."""
        self.comm.publish(topic, data) # Assumes comm handles agent_id addition

    def move(self, linear_x, angular_z):
        """Send movement commands (Twist message) to the robot's cmd_vel topic."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def update(self):
        """Main agent update loop, called periodically by the agent_node."""
        # Get current neighbour data from communication module
        # Note: Behaviours will access this via self.agent.comm.get_neighbour_states()
        # neighbour_states = self.comm.get_neighbour_states()
        
        # Compute behaviour
        behaviour_module = self.behaviours.get(self.current_behaviour)
        linear_x, angular_z = 0.0, 0.0
        if behaviour_module and hasattr(behaviour_module, 'compute'):
            try:
                linear_x, angular_z = behaviour_module.compute() # compute() method needs to use self.agent.comm.get_neighbour_states()
            except Exception as e:
                rospy.logerr(f"Error computing behaviour {self.current_behaviour} for agent {self.agent_id}: {e}")
                import traceback
                traceback.print_exc()

        # Apply movement
        self.move(linear_x, angular_z)

        # Publish state for others
        self.publish_state()

        # Clean old neighbor data - Now handled within SwarmCommunication (implicitly by overwriting or needs explicit cleaning there)
        # self._clean_old_neighbors() 

        # Visualize 
        self._visualize() 

    def _visualize(self):
        """Visualize this agent's state."""
        try:
            agent_data = {
                'position': self.position,
                'orientation': self.orientation
            }
            self.vis.visualize_single_agent(self.agent_id, agent_data)
        except Exception as e:
             rospy.logwarn_throttle(10, f"Visualization error for agent {self.agent_id}: {e}")

    def stop(self):
        """Publish a zero-velocity Twist message to stop the robot."""
        rospy.loginfo(f"Agent {self.agent_id} ({self.agent_name}) received stop command, publishing zero velocity.")
        self.move(0.0, 0.0)
        # Optionally add a small delay or publish multiple times to ensure it gets through
        rospy.sleep(0.1)
        self.move(0.0, 0.0)

# Note: The old if __name__ == '__main__': block is removed.
# This file is now intended to be imported as a module, not run directly.
# The executable logic is in nodes/agent_node.py. 