#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json

class SwarmCommunication:
    """Handles communication between swarm agents."""

    def __init__(self, agent_id):
        """Initialize the communication module."""
        self.agent_id = agent_id
        # Remove agent_name as it's not needed for shared topics
        # self.agent_name = f"agent_{agent_id}"

        # Define shared topics
        self.topics = {
            'state': '/swarm/states',       # All agents publish/subscribe here
            'map': '/swarm/map',           # Shared map data
            'formation': '/swarm/formation', # Command from controller
            'behaviour': '/swarm/behaviour', # Command from controller
            'frontiers': '/swarm/frontiers',  # Added for ExploreBehaviour
            'task_bids': '/swarm/task_bids'   # Added for TaskAllocator
        }
        
        # Store publishers, subscribers, and user-registered callbacks
        self.publishers = {}
        self.subscribers = {}
        self.message_callbacks = {} # Stores user-provided callbacks
        self.bid_callback = None    # Specific callback for task bids

        # Create publisher for state topic
        self.publishers['state'] = rospy.Publisher(self.topics['state'], String, queue_size=20) # Increased queue size
        # Create publishers for other topics dynamically if publish() is called?

        # Register internal callbacks to handle default storage for state and map
        self.register_callback('state', self._internal_state_callback)
        self.register_callback('map', self._internal_map_callback)

        # --- Add subscriber for task bids --- #
        if 'task_bids' in self.topics:
             self.subscribers['task_bids'] = rospy.Subscriber(
                 self.topics['task_bids'],
                 String,
                 self._task_bid_callback # Directly call the specific bid callback
             )
             rospy.logdebug(f"Agent {self.agent_id} Comm: Created subscriber for topic 'task_bids'.")
        # ------------------------------------ #

        # Shared data repository (consider thread safety if complex)
        self.shared_data_store = {
            'state': {},
            'map': {} # Add storage for map data keyed by sender_id
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
                    lambda msg, tn=topic_name: self._generic_message_callback(msg, tn) # Use generic handler
                )
                rospy.logdebug(f"Agent {self.agent_id} Comm: Created subscriber for topic '{topic_name}'.")
        else:
            rospy.logwarn(f"Agent {self.agent_id} Comm: Cannot register callback for undefined topic '{topic_name}'")

    # --- Internal Callbacks for Default Data Storage ---

    def _internal_state_callback(self, data):
        """Internal handler to store state data from other agents."""
        sender_id = data.get('agent_id')
        # Store data from OTHER agents
        if sender_id is not None and sender_id != self.agent_id:
             # Maybe add timestamp here? rospy.Time.now()? Or use timestamp from sender?
            self.shared_data_store['state'][sender_id] = data 
            # Directly call the agent's state update method if registered? Less flexible.

    def _internal_map_callback(self, data):
        """Internal handler to store map data from other agents."""
        sender_id = data.get('agent_id')
        # Store map data from OTHER agents (or all including self? Decide based on need)
        if sender_id is not None: # Store map data from all agents, including self?
            # We might want to store the whole data dict received
            # Or just the cells if that's all needed for integration
            self.shared_data_store['map'][sender_id] = data # Store the raw data dict
            # Optionally add receive timestamp: data['received_timestamp'] = rospy.Time.now().to_sec()

    # --- Generic Message Handler ---

    def _generic_message_callback(self, msg, topic_name):
        """Generic handler for all subscribed topics."""
        try:
            data = json.loads(msg.data)

            # --- Call Internal Storage Handlers ---
            # These handle the default behaviour of storing state/map data
            if topic_name == 'state':
                self._internal_state_callback(data)
            elif topic_name == 'map':
                self._internal_map_callback(data)

            # --- Call User-Registered Callback ---
            # If the user registered a specific callback for this topic, call it
            if topic_name in self.message_callbacks:
                # We already called the internal state/map callbacks above if needed.
                # Now call the user's callback for any additional processing.
                # Note: If the user callback *replaces* internal storage, the internal
                # calls above could be skipped or made conditional. For now, we do both.
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
            
        # Ensure publisher exists (Create if first time?) - State pub created in __init__
        if topic_name not in self.publishers:
             self.publishers[topic_name] = rospy.Publisher(self.topics[topic_name], String, queue_size=10)
             rospy.logwarn(f"Created publisher for {topic_name} on demand.")

        # Add agent_id to data
        if isinstance(data, dict):
            data['agent_id'] = self.agent_id
            # Maybe add timestamp here? data['timestamp'] = rospy.Time.now().to_sec()

        try:
            msg = String()
            msg.data = json.dumps(data)
            self.publishers[topic_name].publish(msg)
        except TypeError as e:
            rospy.logerr(f"Agent {self.agent_id} Comm: Error serializing data for topic '{topic_name}': {e}. Data type: {type(data)}")
        except Exception as e:
            rospy.logerr(f"Agent {self.agent_id} Comm: Error publishing message on {topic_name}: {e}")

    # Allow agent to retrieve neighbour states from the internal store
    def get_neighbour_states(self):
        """Return the dictionary of currently known neighbour states."""
        # Return a copy to prevent modification? Or allow direct access?
        return self.shared_data_store['state'].copy()

    # Allow agent to retrieve shared map data from the internal store
    def get_shared_map_data(self):
        """Return the dictionary of currently known shared map data, keyed by agent_id."""
        return self.shared_data_store['map'].copy()

    # Removed get_shared_data - agent should get neighbour states via get_neighbour_states
    # def get_shared_data(self, topic_name, agent_id=None):

    # --- Specific Callback for Task Bids --- #
    def _task_bid_callback(self, msg):
        """Handle incoming task bid messages."""
        try:
            data = json.loads(msg.data)
            # Check if it looks like a valid bid message (basic check)
            if data.get('type') == 'BID' and 'agent_id' in data and 'task_id' in data and 'cost' in data:
                if self.bid_callback:
                    self.bid_callback(data)
                else:
                     rospy.logwarn_throttle(10, f"Agent {self.agent_id} Comm: Received BID but no bid_callback registered.")
            # else: ignore non-BID messages on this topic
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