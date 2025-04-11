#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json

class SwarmCommunication:
    """Handles communication between swarm agents."""

    def __init__(self, agent_id):
        """Initialize the communication module."""
        self.agent_id = agent_id

        # Define shared topics
        self.topics = {
            'state': '/swarm/states',
            'map': '/swarm/map',
            'formation': '/swarm/formation',
            'behaviour': '/swarm/behaviour',
            'frontiers': '/swarm/frontiers',
            'task_bids': '/swarm/task_bids'
        }
        
        # Store publishers, subscribers, and user-registered callbacks
        self.publishers = {}
        self.subscribers = {}
        self.message_callbacks = {}
        self.bid_callback = None

        # Create publisher for state topic
        self.publishers['state'] = rospy.Publisher(self.topics['state'], String, queue_size=20)

        # Register internal callbacks to handle default storage for state and map
        self.register_callback('state', self._internal_state_callback)
        self.register_callback('map', self._internal_map_callback)

        # Add subscriber for task bids
        if 'task_bids' in self.topics:
             self.subscribers['task_bids'] = rospy.Subscriber(
                 self.topics['task_bids'],
                 String,
                 self._task_bid_callback
             )
             rospy.logdebug(f"Agent {self.agent_id} Comm: Created subscriber for topic 'task_bids'.")

        # Shared data repository
        self.shared_data_store = {
            'state': {},
            'map': {}
        }
        
        rospy.loginfo(f"Comm module for agent {agent_id} initialized. Topics: {list(self.topics.keys())}")

    def register_callback(self, topic_name, callback):
        """Register a callback for any defined topic."""
        if topic_name in self.topics:
            # Store the callback function provided by the user/agent
            self.message_callbacks[topic_name] = callback

            # Ensure a subscriber exists for this topic
            if topic_name not in self.subscribers:
                self.subscribers[topic_name] = rospy.Subscriber(
                    self.topics[topic_name],
                    String,
                    lambda msg, tn=topic_name: self._generic_message_callback(msg, tn)
                )
                rospy.logdebug(f"Agent {self.agent_id} Comm: Created subscriber for topic '{topic_name}'.")
        else:
            rospy.logwarn(f"Agent {self.agent_id} Comm: Cannot register callback for undefined topic '{topic_name}'")

    def _internal_state_callback(self, data):
        """Internal handler to store state data from other agents."""
        sender_id = data.get('agent_id')
        # Store data from OTHER agents
        if sender_id is not None and sender_id != self.agent_id:
            self.shared_data_store['state'][sender_id] = data

    def _internal_map_callback(self, data):
        """Internal handler to store map data from other agents."""
        sender_id = data.get('agent_id')
        if sender_id is not None:
            self.shared_data_store['map'][sender_id] = data

    def _generic_message_callback(self, msg, topic_name):
        """Generic handler for all subscribed topics."""
        try:
            data = json.loads(msg.data)

            # Call Internal Storage Handlers
            if topic_name == 'state':
                self._internal_state_callback(data)
            elif topic_name == 'map':
                self._internal_map_callback(data)

            # Call User-Registered Callback
            if topic_name in self.message_callbacks:
                self.message_callbacks[topic_name](data)

        except json.JSONDecodeError as e:
            rospy.logerr(f"Agent {self.agent_id} Comm: Error decoding JSON on topic '{topic_name}': {e}. Data: '{msg.data[:100]}...'")
        except Exception as e:
             rospy.logerr(f"Agent {self.agent_id} Comm: Error processing message on topic '{topic_name}': {e}")

    def publish(self, topic_name, data):
        """Publish data to a specific topic."""
        if topic_name not in self.topics:
            rospy.logerr(f"Agent {self.agent_id} Comm: Cannot publish to undefined topic '{topic_name}'")
            return
            
        # Ensure publisher exists
        if topic_name not in self.publishers:
             self.publishers[topic_name] = rospy.Publisher(self.topics[topic_name], String, queue_size=10)
             rospy.logwarn(f"Created publisher for {topic_name} on demand.")

        # Add agent_id to data
        if isinstance(data, dict):
            data['agent_id'] = self.agent_id

        try:
            msg = String()
            msg.data = json.dumps(data)
            self.publishers[topic_name].publish(msg)
        except TypeError as e:
            rospy.logerr(f"Agent {self.agent_id} Comm: Error serializing data for topic '{topic_name}': {e}. Data type: {type(data)}")
        except Exception as e:
            rospy.logerr(f"Agent {self.agent_id} Comm: Error publishing message on {topic_name}: {e}")

    def publish_state(self, state_data):
        """Convenience method to publish agent state."""
        self.publish('state', state_data)

    def get_neighbour_states(self):
        """Return the dictionary of currently known neighbour states."""
        return self.shared_data_store['state'].copy()

    def get_shared_map_data(self):
        """Return the dictionary of currently known shared map data, keyed by agent_id."""
        return self.shared_data_store['map'].copy()

    def _task_bid_callback(self, msg):
        """Handle incoming task bid messages."""
        try:
            data = json.loads(msg.data)
            # Check if it looks like a valid bid message
            if data.get('type') == 'BID' and 'agent_id' in data and 'task_id' in data and 'cost' in data:
                if self.bid_callback:
                    self.bid_callback(data)
                else:
                     rospy.logwarn_throttle(10, f"Agent {self.agent_id} Comm: Received BID but no bid_callback registered.")
        except json.JSONDecodeError as e:
            rospy.logerr(f"Agent {self.agent_id} Comm: Error decoding JSON on task_bids topic: {e}. Data: '{msg.data[:100]}...'")
        except Exception as e:
            rospy.logerr(f"Agent {self.agent_id} Comm: Error processing task_bid message: {e}")

    def register_bid_callback(self, callback):
        """Register the callback function for handling incoming bids."""
        if callable(callback):
            self.bid_callback = callback
            rospy.logdebug(f"Agent {self.agent_id} Comm: Registered bid callback.")
        else:
             rospy.logwarn(f"Agent {self.agent_id} Comm: Attempted to register non-callable bid callback.")