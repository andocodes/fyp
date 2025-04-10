#!/usr/bin/env python3

import rospy
# Import SwarmAgent from the library installed via setup.py
from butler_swarm.agent import SwarmAgent
import traceback # Import traceback for error logging

if __name__ == '__main__':
    agent = None # Initialize agent variable outside try block
    node_name = "unknown_agent_node" # Default name
    try:
        # Initialize the ROS node for this agent FIRST!
        # Using anonymous=True ensures unique node name even if __name remapping is slow
        rospy.init_node('agent_node', anonymous=True)
        
        agent_id_param = None
        # Get the actual node name assigned by ROS (e.g., /agent_0)
        # Note: rospy.get_name() requires init_node() to be called first.
        node_name = rospy.get_name()
        private_param_name = f"{node_name}/agent_id"
        
        # Wait briefly for the parameter to become available
        start_time = rospy.Time.now() # Now this call is safe
        wait_duration = rospy.Duration(2.0) 
        while agent_id_param is None and (rospy.Time.now() - start_time) < wait_duration:
            try:
                agent_id_param = rospy.get_param(private_param_name)
            except KeyError: 
                try:
                    rospy.sleep(0.1) # rospy.sleep() also requires init_node
                except rospy.ROSInterruptException:
                     rospy.loginfo(f"Node {node_name}: Interrupted during sleep while waiting for agent_id param.")
                     break
            except rospy.ROSInterruptException: 
                rospy.loginfo(f"Node {node_name}: Interrupted while waiting for agent_id param.")
                break 

        if agent_id_param is None:
            # Fallback to using ~ if direct path failed
            try:
                agent_id_param = rospy.get_param('~agent_id', None)
            except (KeyError, rospy.ROSException): # Catch potential ROSException if param server isn't ready
                pass 

        if agent_id_param is None:
            rospy.logerr(f"Agent ID not specified ({private_param_name} or ~agent_id) for node {node_name}. Shutting down.")
        else:
            rospy.loginfo(f"Node {node_name} starting with agent_id {agent_id_param}")
            
            # Initialize the SwarmAgent class instance
            agent = SwarmAgent(agent_id=int(agent_id_param))
            
            # --- Register Shutdown Hook --- 
            # Ensure the agent object exists before registering the hook
            rospy.on_shutdown(agent.stop) 
            rospy.loginfo(f"Node {node_name}: Registered shutdown hook.")

            # Set loop rate
            rate = rospy.Rate(10) # e.g., 10 Hz

            # Main loop
            while not rospy.is_shutdown():
                agent.update() 
                rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo(f"Agent node {node_name} interrupted (Ctrl+C). Shutting down cleanly.")
        # Shutdown hook (agent.stop) should have already been called by ROS
    except Exception as e:
        rospy.logerr(f"An error occurred in agent node {node_name}: {e}")
        traceback.print_exc()
    finally:
        # Ensure stop is called even if loop exits unexpectedly
        # (though on_shutdown should normally handle it)
        if agent is not None and hasattr(agent, 'stop') and callable(agent.stop):
            rospy.loginfo(f"Node {node_name}: Calling agent.stop() in finally block just in case.")
            try:
                agent.stop()
            except Exception as stop_e:
                 rospy.logerr(f"Node {node_name}: Error calling agent.stop() in finally block: {stop_e}")

        rospy.loginfo(f"Agent node {node_name} fully stopped.")
