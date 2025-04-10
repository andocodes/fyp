# Butler Swarm Obstacle Avoidance Debugging Log 

## Introduction

This document chronicles the debugging process undertaken to improve the obstacle avoidance capabilities of the Butler Swarm agents, specifically within the `ExploreBehaviour`. The primary goal was to achieve robust, reliable navigation in environments with varying obstacle densities and configurations, allowing agents to explore effectively without getting stuck, oscillating, or exhibiting unstable physical behavior.

The process involved iterative refinement, starting with simple reactive mechanisms and progressing towards more complex state-based maneuvers. Challenges encountered included parameter tuning sensitivity, physics engine instability, conflicts between reactive avoidance and path planning, and ensuring agents could navigate realistic environmental features like gaps and corners.

## Debugging Phase 1: Initial Issues & Simple Avoidance Tuning

**Problem Context:** Early testing revealed several critical issues hindering exploration:
*   **Stuck Agents:** Agents frequently became unresponsive or stuck against walls or obstacles.
*   **Avoidance Not Triggering:** Logs showed obstacles were detected by the laser scan, but the avoidance logic wasn't always activating.
*   **Physics Instability:** Particularly Agent 4 exhibited erratic behavior like flipping over or flying off the ground plane, often associated with avoidance maneuvers or high commanded velocities near obstacles.

**Attempt 1: Relaxing Trigger Conditions & Basic Logging (explore.py edit 1)**
*   **Analysis:** Examination of logs suggested the condition `0 < obs_x < self.OBSTACLE_DISTANCE_THRESHOLD` was preventing avoidance when agents were *very* close (obs_x <= 0) but still needed to react. The transformation from LaserFrame to BaseFrame seemed correct.
*   **Change:** The condition was relaxed to `obs_x < self.OBSTACLE_DISTANCE_THRESHOLD`. The calculation `max(0, obs_x)` was used for the dynamic width check to prevent errors with negative `tx`.
*   **Change:** Added `rospy.logdebug` statements to specifically log when obstacles were detected but did *not* meet the (old or new) trigger conditions, including the closest `tx` value.
*   **Outcome:** Helped trigger avoidance more readily but didn't solve fundamental maneuver issues.

**Attempt 2: Addressing Circling/Straight Backup (explore.py edit 2)**
*   **Analysis:** Agents, after detecting an obstacle, often backed up straight or turned ineffectively, leading to repeated collisions or circling behavior along walls.
*   **Change:** Introduced logic to calculate an `avoidance_turn_direction` based on the y-coordinate (`obs_y`) of the closest obstacle point relative to the agent. The goal was to make the agent turn *away* from the obstacle's perceived center.
*   **Change:** Added logging to show the closest obstacle position and the initiated turn direction.
*   **Outcome:** Some improvement, but still prone to getting stuck, and physics issues persisted.

**Attempt 3: Tuning Escape Parameters (explore.py edit 3)**
*   **Analysis:** Agent 4's instability pointed towards excessive speeds during maneuvers. The escape path calculation seemed too simplistic, potentially setting invalid targets or targets too close to the obstacle.
*   **Change:** Reduced `AVOIDANCE_TURN_SPEED`.
*   **Change:** Increased the calculated `escape_distance`.
*   **Change:** Added logging for the calculated escape target coordinates.
*   **Outcome:** Mixed results. Some agents showed slightly better behavior, but Agent 4 continued to fly off, and others still got stuck. Escape target logs were often missing.

**Attempt 4: Prioritizing Stability & Log Visibility (explore.py edit 4)**
*   **Analysis:** Persistent instability and missing logs suggested a need for a more conservative approach and guaranteed log output. The escape target calculation might be failing silently or logs weren't flushed/visible due to logger levels.
*   **Change:** Implemented logic to force the agent to STOP (`linear.x = 0`) immediately upon detecting an obstacle before initiating any backup/turn in the next cycle.
*   **Change:** Reduced backup/escape speeds further.
*   **Change:** Explicitly disabled turning during the forced backup maneuver to simplify physics interactions.
*   **Change:** Changed escape target logging from `rospy.logdebug` to `rospy.loginfo` to ensure visibility regardless of logger level settings.
*   **Outcome:** Agents were generally more stable physically (less flipping), but the core issue of getting stuck against obstacles remained. The lack of turning made them less maneuverable. The `-0.3` escape angle suggested issues in how escape direction was determined or applied. 

## Debugging Phase 2: State Machine Implementation (Scan-Assess-Maneuver - SAM)

**Problem Context:** The simple reactive backup/turn maneuvers proved insufficient. Agents got stuck in loops, unable to effectively navigate away from obstacles or towards clear space. A more structured approach was needed.

**Proposed Solution: "Stop-Turn-Backup" (Intermediate)**
*   **Concept:** Instead of a single reactive action, break down avoidance into distinct steps managed by a state machine.
    1.  **Detect & Stop:** Identify obstacle, halt motion.
    2.  **Assess & Find Angle:** Scan surroundings (`_find_best_escape_angle`) for the best direction away from the obstacle.
    3.  **Turn:** Rotate in place towards the escape angle.
    4.  **Backup/Move:** Execute a short linear motion in the escape direction.
    5.  **Re-evaluate:** Return to normal operation.
*   **Rationale:** More deliberate than simple reactions, allows for better escape angle selection, aims for stability by stopping first.

**Attempt 5: Initial State Machine & Escape Angle Function (explore.py edit 5)**
*   **Change:** Introduced `self.avoidance_state` with states: `'none'`, `'assessing'`, `'escape_turning'`, `'escape_moving'`.
*   **Change:** Added core logic to the `compute` method to handle transitions between these states based on obstacle detection and timers/completion flags.
*   **Change:** Created placeholder `_find_escape_vector` (initially named `_find_best_escape_angle`) which returned a fixed angle (straight back) for initial structure.
*   **Change:** Created placeholder `_handle_stuck_recovery` function.
*   **Change:** Removed old avoidance flags and parameters.
*   **Outcome:** Basic state machine structure implemented, but escape logic was not yet functional (used placeholder angle).

**Attempt 6: Refining State Machine (Stop & Backup Straight) (explore.py edit 6)**
*   **Analysis:** Debugging the initial state machine was complex, and physics instability (Agent 4) was still a concern. A simpler version was proposed to ensure basic stability before adding complex angle finding.
*   **Change:** Removed the `'turning'` state and associated logic (`_find_escape_vector` was temporarily unused).
*   **Change:** Directly transitioned from `'assessing'` (obstacle detected) to `'backing_up'`. The agent stopped, then backed up straight for a fixed duration.
*   **Outcome:** Improved physical stability significantly by removing turning during the maneuver, but agents were still prone to getting stuck as they couldn't actively steer away from obstacles.

**Attempt 7: Fixing Linter Errors & State Structure (explore.py edit 7)**
*   **Change:** Corrected Python indentation errors around `_handle_stuck_recovery` and the end of the `compute` method that arose during previous edits.
*   **Change:** Ensured the `compute` method had a valid return path for velocity commands after state machine logic.
*   **Outcome:** Code structure and syntax corrected, preparing for functional implementation of escape logic.

        self.escape_fallback_angle_bias_rad = np.deg2rad(15) # Avoid angles within this range of 0 when using widest gap fallback
        self.MAX_BAD_FRONTIER_CLAIMS = 3 # Max times an agent can fail to reach a frontier before marking it bad (requires global state)

        # Internal State

## Debugging Phase 3: Implementing Escape Vector & Stuck Recovery

**Problem Context:** With the state machine structure in place, the core logic for selecting an intelligent escape direction and handling situations where the agent remained stuck despite avoidance attempts was needed.

**Attempt 8: Implementing `_find_escape_vector` (explore.py edit 8)**
*   **Concept:** Analyze the laser scan (`agent.last_scan_ranges`) to find segments of clear space exceeding a minimum distance (`escape_min_clearance`). Filter these segments by angular width (`escape_min_gap_width_rad`). Select the best segment based on proximity to the current path's direction, or fallback to the widest available gap.
*   **Change:** Implemented the segment finding and filtering logic within `_find_escape_vector`.
*   **Change:** Added logic to check for an active path and calculate the direction to the next waypoint.
*   **Change:** Implemented the selection logic: find the valid clear segment whose midpoint angle is closest to the path direction. If no path exists or the path direction is too close to an obstacle, select the widest valid clear segment.
*   **Change:** Added conversion of the chosen angle (relative to laser frame) into a world frame yaw angle (`self.escape_target_yaw`) for the turning state.
*   **Change:** Included detailed debug logging for found segments, path direction, and selection reasoning.
*   **Outcome:** Agents could now calculate and attempt to turn towards escape angles based on sensor data. However, logs showed agents often getting stuck in loops, repeatedly selecting the same escape angle and triggering avoidance immediately after the maneuver.

**Attempt 9: Implementing `_handle_stuck_recovery` (explore.py edit 9)**
*   **Concept:** Provide a mechanism to reset the agent's state if it fails to make progress for a defined period (`STUCK_TIMEOUT`), likely due to complex obstacle geometry or persistent avoidance loops.
*   **Change:** Added logic to the `compute` method to track time spent without significant movement using `self.stuck_timer_start`.
*   **Change:** Implemented the `_handle_stuck_recovery` method to:
    *   Log a warning.
    *   Reset `self.avoidance_state` to `'none'`.
    *   Clear `self.current_path` and `self.current_target_frontier_id`.
    *   Publish an empty path to clear RViz visualization.
*   **Outcome:** Provided a fallback mechanism, but didn't address the root cause of the immediate re-triggering of avoidance.

**Attempt 10: Increasing Escape Duration & Debugging (explore.py edit 10)**
*   **Analysis:** The oscillation loops observed in Attempt 8 suggested the `escape_moving` duration wasn't long enough for the agent to clear the obstacle area before the next obstacle check occurred.
*   **Change:** Increased `self.escape_move_duration` from 1.5s to 2.5s.
*   **Change:** Added more detailed debug logging within `_find_escape_vector` to trace segment detection and selection logic more closely.
*   **Outcome:** Still observed oscillations. The longer move helped slightly but didn't resolve the core issue of re-triggering avoidance immediately after the maneuver completed. 