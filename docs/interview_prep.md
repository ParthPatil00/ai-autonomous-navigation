# Interview Preparation Guide

This document prepares you for technical interviews about the AI-Based Autonomous Navigation System. It covers a concise project introduction, component walk-throughs, likely interview questions with model answers grounded in the actual source code, and study recommendations.

> **Honesty principle**: Only claim to understand or have implemented what is actually in the code. Interviewers often probe depth. Sections marked ⚠️ *"Study before interview"* cover areas you should review in the source before claiming full familiarity.

---

## 1. Concise Project Introduction (30–60 seconds)

> "I built a simulation-first autonomous navigation system in Python. It implements two classical path planning algorithms — A\* and Dijkstra — on a 2D occupancy grid. On top of that, I added a clearance-aware costmap that inflates obstacles to keep the robot a safe distance from walls, a geometric path smoother that reduces waypoints while guaranteeing the path stays collision-free, and a pygame visualizer for real-time viewing. Everything is test-driven — there are 161 unit tests and a 48-point physical consistency validator that checks geometry stays correct across different map resolutions."

Keep it factual. Do not claim physical hardware, ROS, or SLAM unless you add them.

---

## 2. Component Walk-Through

### 2.1 Environment

**`GridMap`** — a 2D NumPy boolean array where `True` = obstacle. Provides `is_valid`, `is_obstacle`, `is_free`, and coordinate conversion (`grid_to_world`, `world_to_grid`). World coordinates are at cell centres: `world_x = gx * resolution + resolution/2`.

**`Costmap`** — wraps `GridMap` with a layered cost array computed from the Euclidean distance transform. Four cost bands:
- `254` = lethal obstacle
- `253` = inscribed infeasible (within robot radius)
- `1–199` = soft clearance penalty (exponential or linear decay)
- `0` = free and clear

Used to steer the planner into higher-clearance corridors via a penalty on the step cost.

### 2.2 Planning

**`AStarPlanner`** — standard A\* with a binary min-heap (`heapq`). The open set stores `(f, h, counter, node)` tuples. Step costs: cardinal = 1.0, diagonal = √2. Supports three heuristics (Euclidean, Octile, Manhattan) and two connectivity modes (4- and 8-connected).

**`DijkstraPlanner`** — same structure as A\* but with `h(n) = 0`. Used as an exploration baseline.

**`PathSmoother`** — greedy shortcutting. Scans from the current waypoint forward; replaces any consecutive segment with a direct line-of-sight if `CollisionChecker.is_path_valid()` passes. Returns a `SmoothingResult` with an explicit `is_path_valid` boolean.

### 2.3 Navigation & Safety

**`CollisionChecker`** — models the robot as a circle. `is_state_valid(point, map)` checks the point with radius. `is_path_valid(path, map)` samples each segment at `step_size` intervals.

**`Navigator`** — holds the active path and current waypoint index. Calls the planner, optionally smooths the result, and checks `smooth_res.is_path_valid` before assigning the path. An invalid smoothing result clears the navigation state and returns `False`.

### 2.4 Simulation

**`Simulator`** — the main loop. Each step: sense (`ObstacleDetector`) → navigate (`Navigator.step()`) → move (`RobotModel.move_toward()`) → render → check collision/goal/timeout.

---

## 3. Interview Questions & Answers

### A\* Algorithm

**Q: Explain A\* and why it is faster than Dijkstra.**

A\* adds a heuristic function $h(n)$ to the priority: $f(n) = g(n) + h(n)$. By estimating remaining cost to the goal, it prioritises expanding nodes that are likely closer to the goal, dramatically reducing the number of nodes explored. In my implementation on the default 50×50 grid, A\* explores ~641 nodes vs. Dijkstra's ~1,809.

**Q: What makes a heuristic admissible?**

An admissible heuristic never overestimates the true remaining cost: $h(n) \leq h^*(n)$. This guarantees A\* returns an optimal path. If $h$ overestimates even once, A\* might skip over a node on the optimal path.

**Q: Is Euclidean distance always admissible?**

Yes, on any grid. The straight-line distance is always ≤ the grid path distance, because the grid constrains movement to discrete steps of cost ≥ 1.0 (or √2 for diagonals), and $\sqrt{(\Delta x)^2 + (\Delta y)^2} \leq$ any discrete path cost.

**Q: What is the Octile heuristic and when should you use it instead of Euclidean?**

Octile = $\max(\Delta x, \Delta y) + (\sqrt{2} - 1) \cdot \min(\Delta x, \Delta y)$. It is the exact unobstructed 8-connected cost to the goal, so it is the tightest admissible heuristic for 8-connected grids. Use it when you want maximum search efficiency (fewest nodes explored), but it provides no benefit on 4-connected grids.

**Q: Why is Manhattan distance inadmissible on 8-connected grids?**

For a diagonal cell at offset $(1, 1)$: Manhattan returns 2, but the actual grid cost is $\sqrt{2} \approx 1.414$. Since $2 > 1.414$, the heuristic overestimates — violating admissibility. My code raises a `UserWarning` at construction if this combination is requested.

---

### Dijkstra's Algorithm

**Q: When would you prefer Dijkstra over A\*?**

When you have no useful heuristic (e.g., a graph where nodes have no geometric meaning), or when you need shortest paths to *all* nodes from the source rather than just to a single goal. In this project, Dijkstra is the baseline and always finds the optimal path — it just explores more of the grid unnecessarily.

---

### Costmap & Collision Checking

**Q: What is costmap inflation and why does it matter?**

Costmap inflation marks cells within `robot_radius` of an obstacle as infeasible (the robot's circular footprint would collide), and cells within the `inflation_radius` as costly (close to obstacles). Planning on this costmap produces paths that stay away from walls even when they are geometrically passable. Without inflation, a planner might route a robot through a 1.01-radius gap when the robot radius is 1.0.

**Q: How does the collision checker work?**

`CollisionChecker` models the robot as a circle. For `is_state_valid(world_point, map)`, it converts the world point to a grid cell and checks whether any obstacle is within `robot_radius` metres. For `is_path_valid(path, map)`, it samples each segment at `step_size` (default 0.1 m) intervals and checks each sample point.

**Q: What's the difference between `is_path_valid` and `is_smoothed` in `SmoothingResult`?**

`is_smoothed` is `True` when the smoother actually removed waypoints. `is_path_valid` is the explicit safety status — `True` means the output path was verified collision-free; `False` means it was not (empty input, out-of-bounds point, or the input path was already invalid). You should check `is_path_valid`, not `is_smoothed`, before following the path.

---

### Path Smoothing

**Q: Explain the greedy shortcutting algorithm.**

Starting at waypoint 0, find the furthest waypoint $j$ such that the straight segment from 0 to $j$ is collision-free (verified by `CollisionChecker`). Keep $j$, skip all intermediates. Repeat from $j$. This is $O(n^2)$ in the worst case but fast in practice because most shortcuts succeed early.

**Q: What happens if the shortcutted path fails final validation?**

The smoother falls back to the original path and sets `is_path_valid=True` (because the original path was already verified). `is_smoothed` is `False`. The `Navigator` then follows the longer but safe original path.

---

### Testing & Validation

**Q: How do you know the costmap and collision checker agree with each other?**

I have a 48-point `validate_consistency.py` script that tests specific known scenarios at multiple resolutions (1.0, 0.5, 0.25 m/cell). For each scenario, it checks that `Costmap.is_traversable()` and `CollisionChecker.is_state_valid()` return the same answer for the same (point, robot_radius) pair. If they ever disagree, the script prints a `[FAIL]` line with the actual values.

---

## 4. Topics to Study Before the Interview

The following areas appear in the code. Prepare to discuss them in depth:

| Topic | Where in code | What to study |
| :--- | :--- | :--- |
| Priority queue (min-heap) | `astar.py`, `dijkstra.py` | Python `heapq` API; how tie-breaking works |
| Euclidean distance transform | `costmap.py` | Vectorized point-to-box distance computation (`_compute_obstacle_clearance`) |
| NumPy boolean array operations | `grid_map.py`, `costmap.py` | Indexing, boolean masking, `np.where` |
| `@dataclass` | `path_smoother.py`, `astar.py` | Python dataclass fields and construction |
| `unittest.mock.patch` | `test_navigator.py` | How mock patching works in pytest |
| Corner-cutting prevention | `path_utils.py` | Why both adjacent cells must be checked for diagonal moves |
| Python `heapq` tie-breaking | `astar.py` line 127 | Why a counter is added to the heap tuple |

---

## 5. Limitations to Mention Proactively

Interviewers respect honesty about limitations:

- **Simulation only** — no physical robot, no ROS.
- **Static map** — obstacles don't move; the costmap is computed once.
- **Circular footprint only** — no rectangular or polygonal robot body.
- **2D planar only** — no 3D environment support.

---

## 6. Reasonable Future Improvements (to Propose)

When asked "what would you add next?", suggest items from the actual roadmap:

1. Add DWA or VFH for local dynamic obstacle avoidance.
2. Incremental replanning (D\* Lite) when new obstacles are detected.
3. SLAM with occupancy mapping — remove the full-map assumption.
4. Non-holonomic kinematic model for differential-drive robots.
5. Gym environment wrapper for DRL training.
6. ROS 2 bridge for hardware integration.
