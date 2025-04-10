#!/usr/bin/env python3

import rospy

class IdleBehaviour:
    """A simple behaviour where the agent does nothing (outputs zero velocity)."""

    def __init__(self, agent):
        """Initialize the Idle behaviour."""
        self.agent = agent
        # No specific initialization needed
        rospy.logdebug(f"Agent {self.agent.agent_id}: IdleBehaviour initialized.")

    def compute(self):
        """Compute movement commands (always zero)."""
        # Return zero linear and angular velocity
        return 0.0, 0.0 