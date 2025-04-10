# Butler Swarm Exploration Refactor Plan

This document outlines the plan to refactor the obstacle avoidance and recovery logic within the `ExploreBehaviour` (`src/butler_swarm/behaviours/explore.py`) to improve robustness and reduce repetitive turning behaviors.

## Problem Statement

Currently, robots encountering obstacles utilize a simple "stop-turn-move" escape sequence. This often leads to oscillations and getting stuck in loops, especially in cluttered environments or corners. The stuck recovery mechanism involves only spinning, which is often ineffective.

## Proposed Solution

We will replace the current avoidance state machine with a potential field-based local planner and enhance the stuck recovery mechanism.

**Phase 1: Implement Potential Field Planner & Recovery Jiggle**

1.  **Refactor State Machine:** Modify the `compute` method. Remove `assessing`, `escape_turning`, `escape_moving` states. Introduce a new `'avoiding'` state.
2.  **Implement Potential Field Calculation:**
    *   Create `_calculate_potential_field_velocity()`.
    *   Calculate repulsive forces from nearby laser scan points (inverse square magnitude).
    *   Calculate an attractive force towards the next path waypoint or a default forward direction.
    *   Sum forces to get a resultant vector.
    *   Convert the vector to `linear_x`, `angular_z` commands, respecting limits and adding damping.
3.  **Integrate into `compute`:**
    *   Transition from `'none'` to `'avoiding'` on imminent obstacle detection (`AvoidCheck`).
    *   In `'avoiding'` state, use velocities from potential field calculation.
    *   Initial Exit Condition: Transition back to `'none'` after a fixed duration (`PF_AVOID_DURATION`). (Needs refinement later).
4.  **Implement Recovery "Jiggle":**
    *   Modify `_handle_stuck_recovery()` to trigger a new `'recovery_jiggle'` state.
    *   Implement the `'recovery_jiggle'` state in `compute` to execute a sequence (e.g., backward move, random turn, forward move) before returning to `'none'`.
5.  **Define Parameters:** Add necessary parameters for potential fields and recovery jiggle (gains, influence radii, durations, speeds) and provide initial tuning suggestions.

**Phase 2: Testing, Tuning & Refinement**

1.  **Test:** Observe robot behavior with the new logic.
2.  **Tune:** Adjust parameters iteratively based on performance.
3.  **Refine:** Improve potential field logic (e.g., local minima handling) and the avoidance state exit condition based on testing.

## Parameter Tuning Considerations (Initial)

*   **Potential Field:** `PF_OBSTACLE_INFLUENCE_RADIUS`, `PF_REPULSION_GAIN`, `PF_ATTRACTION_GAIN`, `PF_AVOID_DURATION`. Relative gains between repulsion and attraction are key. Influence radius determines how far obstacles affect the robot.
*   **Recovery Jiggle:** Speeds and durations for backward, turn, and forward movements.
*   **Existing:** `OBSTACLE_DISTANCE_THRESHOLD` (might need adjustment based on PF behavior), `stuck_timeout`.