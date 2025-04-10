# Task Allocator Plan

**Phase 2: Task Allocation Implementation**

1.  **Create `TaskAllocator` Utility (`src/butler_swarm/utils/task_allocator.py`):**
    *   Define a class `TaskAllocator`.
    *   `__init__(self, agent, behaviour)`: Takes references to the main agent instance (for ID, comms) and the controlling behaviour (for pathfinding, grid access).
    *   Internal state: Store incoming bids temporarily during the bidding window (`self.received_bids = {}`).
    *   Configuration: `BIDDING_WINDOW` (e.g., `rospy.Duration(0.5)`), `MAX_ALLOCATION_ATTEMPTS` (e.g., 3).
    *   Method `allocate_task(self, candidate_tasks)`:
        *   Takes a dictionary of candidate `frontier_id: frontier_data` (only 'open' ones).
        *   Calculates A\* path cost for each candidate using `self.behaviour._find_path()`. If path fails, cost is infinite.
        *   Stores calculated costs (`my_bids`).
        *   Clears `self.received_bids`.
        *   Broadcasts `BID` messages for reachable candidates (finite cost) via `self.agent.comm`. Message format: `{'type': 'BID', 'agent_id': self.agent.agent_id, 'task_id': frontier_id, 'cost': calculated_cost}`.
        *   `rospy.sleep(self.BIDDING_WINDOW)`.
        *   Calls `_determine_winners(my_bids)` to get a dictionary of `{frontier_id: winner_agent_id}` for all tasks bid on.
        *   Filters this to find tasks won by *this* agent (`won_tasks = {tid: data for tid, winner in winners.items() if winner == self.agent.agent_id}`).
        *   If `won_tasks` is not empty:
            *   Selects the single 'best' task from `won_tasks` using the behaviour's utility function (e.g., based on `potential * 1.0 - distance * 0.5` calculated using original `frontier_data`).
            *   If a best task is found:
                *   Publishes the claim using `self.behaviour._claim_target_frontier(best_frontier_id)`.
                *   Returns `(best_frontier_id, path_to_best_frontier)` (path was calculated earlier).
        *   If no task is won or no candidates exist, returns `(None, None)`.
    *   Method `record_bid(self, bid_data)`: Called by the communication module. Stores bid: `self.received_bids.setdefault(bid_data['task_id'], []).append({'agent_id': bid_data['agent_id'], 'cost': bid_data['cost']})`.
    *   Method `_determine_winners(self, my_bids)`: Iterates through all tasks in `my_bids` and `self.received_bids`. For each task, finds the agent(s) with the minimum cost bid. If tie, uses `random.random() > 0.5` between tied agents (or lowest ID if random is complex). Returns a dictionary `{task_id: winning_agent_id}`.

2.  **Communication Protocol (`src/butler_swarm/utils/communication.py`):**
    *   Add a new topic: `'task_bids': '/swarm/task_bids'`.
    *   Modify `SwarmCommunication.__init__`: Add `/swarm/task_bids` to `self.topics`. Add subscriber for this topic, linking it to `_task_bid_callback`.
    *   Add publisher for `/swarm/task_bids` (can be created on demand in `publish`).
    *   Add `_task_bid_callback(self, msg)`: Parses JSON. If `data['type'] == 'BID'`, checks if a bid recording callback is registered (`self.bid_callback`), and if so, calls `self.bid_callback(data)`.
    *   Add `register_bid_callback(self, callback)`: Stores the callback (e.g., `allocator.record_bid`) in `self.bid_callback`.
    *   Modify `publish`: Allow publishing `BID` messages to the `task_bids` topic.

3.  **Integrate with `ExploreBehaviour` (`src/butler_swarm/behaviours/explore.py`):**
    *   `__init__`:
        *   Import `TaskAllocator`.
        *   Instantiate `self.allocator = TaskAllocator(self.agent, self)`.
        *   Register the allocator's bid recording method: `self.agent.comm.register_bid_callback(self.allocator.record_bid)`.
        *   Add `self.path_planning_failures = {}` # Dictionary to track failures per frontier_id
    *   `compute`:
        *   When `replan_needed` is true:
            *   Filter `self.known_frontiers` to get `candidate_frontiers` (status='open').
            *   Call `selected_frontier_id, planned_path = self.allocator.allocate_task(candidate_frontiers)`.
            *   Reset `self.path_planning_failures.pop(selected_frontier_id, None)` if a task is successfully allocated.
            *   If `selected_frontier_id` is not None: Set target, path, index, publish path, update activity time.
            *   Else (no task allocated): Handle "no target" logic (check exploration complete timeout).
        *   **Remove** direct calls to `_select_target_frontier` and internal `_claim_target_frontier` from the main replan block.
    *   Path Planning Failure Handling:
        *   Inside the `if replan_needed:` block, where `self.allocator.allocate_task` is called, wrap the call to the *behaviour's* `_find_path` (inside the allocator) or handle the allocator returning `None` path:
        *   If path planning fails for a candidate frontier inside `allocate_task`, the cost should be infinite and no bid placed.
        *   If `allocate_task` returns `(None, None)` because no suitable task was won/available, proceed to "no target" logic.
    *   Stuck/Recovery Handling:
        *   In `_handle_stuck_recovery`: Before resetting state, if `self.current_target_frontier_id` is set, call `self._release_claim(self.current_target_frontier_id)`. Reset `self.path_planning_failures`.
    *   Add `_release_claim(self, frontier_id)` method:
        *   If `frontier_id` in `self.known_frontiers`:
            *   Create update data: `{'frontier_id': frontier_id, 'status': 'open', 'claimant_agent_id': -1}`.
            *   Publish via `self.agent.share_data('frontiers', update_data)`.
            *   Update local `self.known_frontiers[frontier_id]` accordingly.
            *   Log the release.
            *   Clear current target/path: `self.current_target_frontier_id = None; self.current_path = None`. 