#!/usr/bin/env python3

import rospy
import numpy as np
import subprocess
import os
import time
import random # Added for randomization

# Define maze boundaries and central exclusion zone (adjust as needed)
MAZE_MIN_X = -20.0
MAZE_MAX_X = 20.0
MAZE_MIN_Y = -20.0
MAZE_MAX_Y = 20.0
CENTER_EXCLUSION_RADIUS = 5.0 # Don't spawn within this radius of (0,0)
MIN_AGENT_SEPARATION = 1.0 # Minimum distance between spawned agents

# List of approximate obstacle areas (x_min, x_max, y_min, y_max)
# Add coordinates based on the walls added in obstacle_arena.world
# These are approximate and need adjustment based on exact wall sizes/poses
OBSTACLE_ZONES = [
    (4.9, 5.1, -7.5, 7.5),   # maze_wall_1
    (-5.1, -4.9, -7.5, 7.5), # maze_wall_2
    (-2.5, 2.5, 7.4, 7.6),   # maze_wall_3
    (-2.5, 2.5, -7.6, -7.4), # maze_wall_4
    (7.4, 7.6, 2.5, 7.5),   # maze_wall_5
    (-7.6, -7.4, -7.5, -2.5), # maze_wall_6
    (5.0, 15.0, -7.6, -7.4), # maze_wall_7 (approx)
    (-15.0, -5.0, 7.4, 7.6),  # maze_wall_8 (approx)
    (1.5, 2.5, 1.5, 2.5)    # box_obstacle_1
]

def is_valid_spawn(x, y, spawned_positions):
    """Check if a spawn position is valid (not in center, not in obstacle, not too close)."""
    # Check center exclusion
    if np.sqrt(x**2 + y**2) < CENTER_EXCLUSION_RADIUS:
        return False
    
    # Check obstacle zones
    for x_min, x_max, y_min, y_max in OBSTACLE_ZONES:
        if x_min <= x <= x_max and y_min <= y <= y_max:
            return False
            
    # Check separation from other agents
    for sx, sy in spawned_positions:
        if np.sqrt((x - sx)**2 + (y - sy)**2) < MIN_AGENT_SEPARATION:
            return False
            
    return True

def spawn_agent(agent_id, x, y, z=0.05):
    """Generate URDF, set param, spawn model, and launch nodes for a single agent."""
    agent_name = f"agent_{agent_id}"
    agent_ns = f"/{agent_name}" # Namespace with leading slash

    # --- Generate URDF with namespace ---
    pkg_path = subprocess.check_output(["rospack", "find", "butler_swarm"]).strip().decode('utf-8')
    urdf_xacro_path = os.path.join(pkg_path, 'models/butler_agent.urdf.xacro')
    xacro_cmd = [
        "/opt/ros/noetic/bin/xacro", 
        urdf_xacro_path, 
        f"namespace:={agent_name}" # Pass namespace WITHOUT leading slash to xacro
    ]
    try:
        rospy.logdebug(f"Running xacro for {agent_name}: {' '.join(xacro_cmd)}")
        urdf = subprocess.check_output(xacro_cmd, stderr=subprocess.STDOUT).decode('utf-8')
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Failed to run xacro for {agent_name}: {e}")
        rospy.logerr(f"xacro output:\n{e.output.decode('utf-8')}")
        return # Don't continue if URDF generation failed

    # --- Set namespaced robot_description parameter ---
    param_name = f"{agent_ns}/robot_description"
    rospy.set_param(param_name, urdf)
    rospy.logdebug(f"Set parameter {param_name}")

    # --- Spawn model in Gazebo with namespace ---
    spawn_cmd = [
        "rosrun", "gazebo_ros", "spawn_model",
        "-urdf",
        "-param", param_name, # Use namespaced parameter
        "-model", agent_name,
        "-robot_namespace", agent_ns, # Apply namespace to Gazebo plugins
        "-x", str(x),
        "-y", str(y),
        "-z", str(z)
    ]
    try:
        rospy.loginfo(f"Spawning model {agent_name}...")
        subprocess.check_call(spawn_cmd)
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Failed to spawn model {agent_name}: {e}")
        return
    
    # --- Launch robot_state_publisher in namespace ---
    rsp_cmd = [
        "rosrun", "robot_state_publisher", "robot_state_publisher",
        f"__ns:={agent_ns}",
        f"_tf_prefix:={agent_ns}",
        "_ignore_timestamp:=true",
        "_publish_frequency:=30.0"
    ]
    try:
        rospy.logdebug(f"Launching robot_state_publisher for {agent_name}...")
        subprocess.Popen(rsp_cmd)
    except Exception as e:
         rospy.logerr(f"Failed to launch robot_state_publisher for {agent_name}: {e}")
         return

    # --- Launch agent controller node in namespace ---
    agent_node_cmd_list = [
        "rosrun", "butler_swarm", "agent_node.py",
        f"__ns:={agent_ns}", # Run agent node in namespace
        f"__name:=agent_controller", # Give the node a consistent name within the NS
        f"_agent_id:={agent_id}" # Pass ID as private param
    ]
    try:
        rospy.logdebug(f"Launching agent controller for {agent_name}...")
        subprocess.Popen(agent_node_cmd_list)
    except Exception as e:
         rospy.logerr(f"Failed to launch agent controller for {agent_name}: {e}")
         return

def main():
    """Spawn multiple agents at random valid positions."""
    rospy.init_node('spawn_agents')

    # Get parameters
    num_agents = rospy.get_param('~num_agents', 5)
    # radius = rospy.get_param('~radius', 3.0) # Radius no longer used

    spawned_positions = []
    spawn_attempts = 0
    max_spawn_attempts_per_agent = 50 # Prevent infinite loop if space is too crowded

    # Spawn agents randomly
    for i in range(num_agents):
        agent_spawned = False
        current_attempts = 0
        while not agent_spawned and current_attempts < max_spawn_attempts_per_agent:
            # Generate random position within maze boundaries
            x = random.uniform(MAZE_MIN_X, MAZE_MAX_X)
            y = random.uniform(MAZE_MIN_Y, MAZE_MAX_Y)
            
            # Check if the position is valid
            if is_valid_spawn(x, y, spawned_positions):
                rospy.loginfo(f"Initiating spawn process for agent {i} at ({x:.2f}, {y:.2f})")
                spawn_agent(i, x, y)
                spawned_positions.append((x,y))
                agent_spawned = True
                time.sleep(2) # Shorter delay might be okay now
            else:
                # rospy.logdebug(f"Spawn attempt {current_attempts+1} for agent {i} failed at ({x:.2f}, {y:.2f}). Retrying...")
                pass # Avoid flooding logs, just retry
            
            current_attempts += 1
            spawn_attempts += 1 # Total attempts across all agents

        if not agent_spawned:
            rospy.logwarn(f"Failed to find a valid spawn location for agent {i} after {max_spawn_attempts_per_agent} attempts.")

    rospy.loginfo(f"Finished initiating spawn for {len(spawned_positions)} out of {num_agents} requested agents (Total attempts: {spawn_attempts}).")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass