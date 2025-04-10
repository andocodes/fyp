#!/usr/bin/env python3

import rospy
import numpy as np
import random
import time # Might be needed, though rospy.sleep is preferred

class TaskAllocator:
    """Handles task allocation for behaviours using a bidding mechanism."""

    def __init__(self, agent, behaviour):
        """Initialize the task allocator.

        Args:
            agent: The parent agent instance.
            behaviour: The behaviour instance requesting allocation (e.g., ExploreBehaviour).
        """
        self.agent = agent
        self.behaviour = behaviour # Provides grid, pathfinding, etc.

        # Store bids received during a bidding window
        # Format: { task_id: [ {'agent_id': agent_id, 'cost': cost}, ... ], ... }
        self.received_bids = {}
        self.bid_window_start_time = None # Track when the window opens

        # Configuration
        # *** INCREASED BIDDING WINDOW and added wait after bid ***
        self.BIDDING_WINDOW_DURATION = rospy.Duration(1.5) # Further increased to 1.5s
        self.POST_BID_WAIT_DURATION = rospy.Duration(0.6) # Further increased to 0.6s
        # ------------------------------------------------------

        rospy.logdebug(f"Agent {self.agent.agent_id}: TaskAllocator initialized for behaviour {type(behaviour).__name__}")

    def allocate_task(self, candidate_frontiers):
        """Selects the best frontier, publishes bids, and determines the winner.

        Args:
            candidate_frontiers (dict): Dictionary of potential frontiers {frontier_id: frontier_data}.
                                     Frontier data should include necessary info like position.

        Returns:
            tuple: (selected_frontier_id, path_to_selected) if successful, else (None, None).
        """
        rospy.logdebug(f"Agent {self.agent.agent_id}: Starting task allocation for {len(candidate_frontiers)} candidates.")
        # Clear bids from previous rounds
        self.received_bids = {}
        self.bid_window_start_time = rospy.Time.now()

        # Store potential bids for this agent in this round
        potential_bids = {}
        my_pos_grid = self.behaviour._world_to_grid(self.agent.position[0], self.agent.position[1])

        # --- Step 1: Calculate Costs/Utilities for potential tasks --- #
        for f_id, f_data in candidate_frontiers.items():
            f_pos_world = f_data.get('position')
            if not f_pos_world:
                rospy.logwarn_throttle(5.0, f"Agent {self.agent.agent_id} Allocator: Frontier {f_id} missing position data.")
                continue
            f_pos_grid = self.behaviour._world_to_grid(f_pos_world[0], f_pos_world[1])

            # --- Pre-check Reachability (Optional but can save pathfinding time) --- #
            # Note: _find_path already handles unreachability, so this is an optimization
            # if not self.behaviour._is_reachable(my_pos_grid, f_pos_grid): 
            #     rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Skipping frontier {f_id} - pre-check unreachable.")
            #     continue
            
            # --- Calculate Cost (Pathfinding) --- #
            path, cost = self.behaviour._find_path(my_pos_grid, f_pos_grid)
            if path is None:
                rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Pathfinding failed for {f_id} (cost=inf). Skipping bid.")
                continue
            # -------------------------------------- #

            # --- Calculate Utility (Example: Inverse Cost) --- #
            utility = 1.0 / (cost + 1e-6)
            rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Candidate {f_id}, Cost={cost:.2f}, Utility={utility:.4f}")

            # Store potential bid info
            potential_bids[f_id] = {'utility': utility, 'path': path, 'cost': cost, 'data': f_data}

        if not potential_bids:
            rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: No reachable candidate frontiers found.")
            return None, None

        # --- Step 2: Publish Bids for all potentially reachable tasks --- # 
        bids_published_count = 0
        for f_id, bid_info in potential_bids.items():
            bid_message = {
                'type': 'BID',
                'agent_id': self.agent.agent_id,
                'task_id': f_id,
                'cost': bid_info['cost'],
                'bid_value': bid_info['utility'] # Include utility for potential future strategies
            }
            self.agent.comm.publish('task_bids', bid_message)
            bids_published_count += 1
        rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Published {bids_published_count} bids.")

        # --- Step 3: Wait briefly for own bids to propagate and others to respond --- #
        rospy.sleep(self.POST_BID_WAIT_DURATION)

        # --- Step 4: Continue waiting for the rest of the bidding window --- #
        rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Post-bid wait finished. Checking remaining window time.")
        time_elapsed = rospy.Time.now() - self.bid_window_start_time
        remaining_wait = self.BIDDING_WINDOW_DURATION - time_elapsed
        if remaining_wait > rospy.Duration(0):
            rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Waiting remaining {remaining_wait.to_sec():.2f}s of bidding window.")
            rospy.sleep(remaining_wait)
        # --- Log final received bids before determining winner --- #
        rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Bidding window closed. Final received bids before win determination: {self.received_bids}")
        # ------------------------------------------------------- #

        # --- Step 5: Determine Winners based on all bids received --- #
        winners = self._determine_winners(potential_bids) # Pass potential bids for own cost info
        rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Determined winners: {winners}")

        # --- Step 6: Filter tasks won by this agent --- #
        won_tasks = {tid: potential_bids[tid] for tid, winner_id in winners.items() 
                     if winner_id == self.agent.agent_id and tid in potential_bids}
        rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Tasks potentially won by me: {list(won_tasks.keys())}")

        if not won_tasks:
            rospy.loginfo(f"Agent {self.agent.agent_id}: Did not win any tasks this round.")
            return None, None

        # --- Step 7: Select the single best task among the ones won (e.g., highest utility) --- #
        best_task_id = None
        best_utility = -float('inf')
        best_path = None

        for task_id, task_info in won_tasks.items():
            # --- Recalculate utility based on current state if needed, or use stored one --- #
            # Using stored utility for now, assuming it doesn't change drastically during bidding window
            current_utility = task_info['utility'] 
            # Example using potential/distance (uncomment if preferred, but might be redundant if utility calc is stable):
            # task_pos = task_info['data'].get('position')
            # if not task_pos: continue
            # f_grid_x, f_grid_y = self.behaviour._world_to_grid(task_pos[0], task_pos[1])
            # distance = task_info['cost'] # Use path cost as distance approximation
            # potential = 0
            # for dx in range(-1, 2):
            #     for dy in range(-1, 2):
            #         if dx == 0 and dy == 0: continue
            #         nx, ny = f_grid_x + dx, f_grid_y + dy
            #         if 0 <= nx < self.behaviour.grid_dims and 0 <= ny < self.behaviour.grid_dims and self.behaviour.grid[nx, ny] == 0:
            #             potential += 1
            # current_utility = potential * 1.0 - distance * 0.5 # Or some other weighting
            # -----------------------------------------------------------------------------

            rospy.logdebug(f"Agent {self.agent.agent_id} Allocator: Evaluating won task {task_id}, Utility: {current_utility:.4f}")
            if current_utility > best_utility:
                 best_utility = current_utility
                 best_task_id = task_id
                 best_path = task_info['path']

        # --- Step 8: If a best task was chosen, PUBLISH CLAIM and return --- #
        if best_task_id:
            rospy.loginfo(f"Agent {self.agent.agent_id}: Selected task {best_task_id} (Cost: {won_tasks[best_task_id]['cost']:.2f}, Util: {best_utility:.4f}) from won tasks.")
            # --- Publish the claim NOW using comm module --- #
            claim_message = {
                'type': 'CLAIM',
                'agent_id': self.agent.agent_id,
                'task_id': best_task_id,
            }
            self.agent.comm.publish('task_bids', claim_message) # Publish claim on the same topic as bids for processing
            rospy.loginfo(f"Agent {self.agent.agent_id}: Published claim for frontier {best_task_id}")
            # -------------------------------------------- #
            # --- Optionally, update local status immediately for responsiveness --- # 
            if best_task_id in self.behaviour.known_frontiers:
                 self.behaviour.known_frontiers[best_task_id]['status'] = 'claimed'
                 self.behaviour.known_frontiers[best_task_id]['claimant_agent_id'] = self.agent.agent_id
            # -------------------------------------------------------------------- #
            return best_task_id, best_path
        else:
            # This case means the agent won bids, but the utility calculation/selection failed
            rospy.logwarn(f"Agent {self.agent.agent_id}: Won tasks ({list(won_tasks.keys())}) but failed to select a best one.")
            return None, None

    def record_bid(self, bid_data):
        """Record a bid received from another agent."""
        task_id = bid_data.get('task_id')
        agent_id = bid_data.get('agent_id')
        cost = bid_data.get('cost')

        if task_id is None or agent_id is None or cost is None:
            rospy.logwarn(f"Agent {self.agent.agent_id}: Received incomplete bid data: {bid_data}")
            return

        # Ignore own bids reflected back
        if agent_id == self.agent.agent_id:
            return

        if task_id not in self.received_bids:
            self.received_bids[task_id] = []
        
        # Avoid duplicate bids from the same agent for the same task in one round
        if not any(bid['agent_id'] == agent_id for bid in self.received_bids[task_id]):
             self.received_bids[task_id].append({'agent_id': agent_id, 'cost': cost})
             rospy.logdebug(f"Agent {self.agent.agent_id}: Recorded bid from {agent_id} for task {task_id} with cost {cost:.2f}")

    def _determine_winners(self, my_bids):
        """Determine the winning agent for each task based on lowest cost.

        Args:
            my_bids (dict): This agent's bids {task_id: {'cost': cost, ...}}

        Returns:
            dict: Winning agent for each task {task_id: winning_agent_id}.
        """
        all_task_ids = set(my_bids.keys()) | set(self.received_bids.keys())
        winners = {}

        for task_id in all_task_ids:
            bids_for_task = []
            # Add own bid if valid
            if task_id in my_bids and my_bids[task_id]['cost'] != float('inf'):
                rospy.logdebug(f"Task {task_id}: Adding own bid (cost: {my_bids[task_id]['cost']:.2f})")
                bids_for_task.append({'agent_id': self.agent.agent_id, 'cost': my_bids[task_id]['cost']})
            
            # Add received bids
            if task_id in self.received_bids:
                rospy.logdebug(f"Task {task_id}: Adding received bids: {self.received_bids[task_id]}")
                bids_for_task.extend(self.received_bids[task_id])

            if not bids_for_task:
                rospy.logdebug(f"Task {task_id}: No bids found.")
                continue # No one bid on this task

            # Find minimum cost
            rospy.logdebug(f"Task {task_id}: Determining winner from bids: {bids_for_task}")
            min_cost = min(bid['cost'] for bid in bids_for_task)

            # Find all bidders with the minimum cost
            min_cost_bidders = [bid['agent_id'] for bid in bids_for_task if bid['cost'] == min_cost]

            # Determine winner (random tie-breaking)
            if len(min_cost_bidders) == 1:
                winner_id = min_cost_bidders[0]
            elif len(min_cost_bidders) > 1:
                winner_id = random.choice(min_cost_bidders)
                rospy.logdebug(f"Agent {self.agent.agent_id}: Tie break for task {task_id} among {min_cost_bidders}. Winner: {winner_id}")
            else:
                continue # Should not happen if bids_for_task was not empty
            
            winners[task_id] = winner_id
            
        return winners 