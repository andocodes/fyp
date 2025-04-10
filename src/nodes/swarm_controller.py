#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
import json
import random

class SwarmController:
    """Central controller for coordinating swarm Behaviours."""

    def __init__(self):
        """Initialize the swarm controller."""
        # Initialize ROS node
        rospy.init_node('swarm_controller', anonymous=True)

        # Number of agents to spawn
        self.num_agents = rospy.get_param('~num_agents', 5)

        # Available Behaviours
        self.Behaviours = ['idle', 'flocking', 'search', 'formation', 'explore']
        self.current_Behaviour = 'idle'

        # Formation parameters
        self.current_formation = 'circle'
        self.formation_center = [0, 0]

        # Publishers
        self.Behaviour_pub = rospy.Publisher('/swarm/Behaviour', String, queue_size=10)
        self.formation_pub = rospy.Publisher('/swarm/formation', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/swarm/states', String, self.state_callback)

        # Agent tracking
        self.agent_states = {}

        # Timer for periodic updates
        # Disable automatic switching for now to allow manual control
        # self.timer = rospy.Timer(rospy.Duration(15), self.timer_callback) # Increased duration

        rospy.loginfo(f"Swarm controller initialized with {self.num_agents} agents. Default behaviour: {self.current_Behaviour}")

    def state_callback(self, msg):
        """Process state updates from agents."""
        try:
            data = json.loads(msg.data)
            agent_id = data.get('agent_id')
            if agent_id is not None:
                self.agent_states[agent_id] = data
        except Exception as e:
            rospy.logerr(f"Error processing agent state: {e}")

    # We can comment out the timer_callback and _update_Behaviours
    # as they are no longer triggered by the timer.
    # def timer_callback(self, event):
    #     """Periodic controller updates."""
    #     # Check if we need to switch Behaviours or formations
    #     # self._update_Behaviours()
    #     pass # Do nothing if timer is disabled

    def set_Behaviour(self, Behaviour_name):
        """Set the current Behaviour for all agents."""
        if Behaviour_name in self.Behaviours:
            self.current_Behaviour = Behaviour_name

            # Publish Behaviour command
            command = {
                'type': 'behaviour', # Use lowercase consistent with agent callback?
                'behaviour': Behaviour_name,
                'timestamp': rospy.Time.now().to_sec()
            }

            self.Behaviour_pub.publish(json.dumps(command))
            rospy.loginfo(f"Set swarm behaviour to {Behaviour_name}")
        else:
            rospy.logerr(f"Unknown behaviour: {Behaviour_name}")

    def set_formation(self, formation_name, center=None):
        """Set the current formation for all agents."""
        if center is not None:
            self.formation_center = center

        self.current_formation = formation_name

        # Publish formation command
        command = {
            'type': 'formation',
            'formation': formation_name,
            'center': self.formation_center,
            'timestamp': rospy.Time.now().to_sec()
        }

        self.formation_pub.publish(json.dumps(command))
        rospy.loginfo(f"Set swarm formation to {formation_name} at {self.formation_center}")

    # def _update_Behaviours(self):
    #     """Decide whether to update Behaviours or formations."""
    #     # This would implement the logic for when to switch Behaviours
    #     # For demonstration, we'll just randomly switch
    #     # if random.random() < 0.2:  # 20% chance to switch Behaviour
    #     #     new_Behaviour = random.choice(self.Behaviours)
    #     #     self.set_Behaviour(new_Behaviour)
    #     #
    #     # # For formation Behaviour, also update formation type sometimes
    #     # if self.current_Behaviour == 'formation' and random.random() < 0.3:  # 30% chance
    #     #     formations = ['line', 'circle', 'grid', 'arrow']
    #     #     new_formation = random.choice(formations)
    #     #
    #     #     # Also potentially change formation center
    #     #     new_center = [
    #     #         random.uniform(-5, 5),
    #     #         random.uniform(-5, 5)
    #     #     ]
    #     #
    #     #     self.set_formation(new_formation, new_center)
    #     pass # Do nothing if timer is disabled

if __name__ == '__main__':
    try:
        controller = SwarmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass