# Path Planning Algorithms

This document explains the algorithms implemented in the AI-Based Autonomous Navigation System, including theoretical background, implementation details, and known limitations.

---

## 1. Graph Representation

Both planners operate on the same graph abstraction provided by `planning/path_utils.py`:

- **Nodes**: Grid cells `(x, y)` where `x` is the column and `y` is the row index.
- **Edges**: Traversable adjacent cells. Traversability is determined by `is_passable()`, which dispatches to `GridMap.is_free()` or `Costmap.is_traversable()` depending on the map type.
- **Edge cost**: Depends on connectivity and whether a `Costmap` is active (see below).

---

## 2. Dijkstra's Algorithm (`planning/dijkstra.py`)

### Theory

Dijkstra's algorithm is a **uniform-cost search** that finds the shortest path from a single source node to all reachable nodes in a graph with non-negative edge weights.

$$f(n) = g(n)$$

where $g(n)$ is the exact accumulated cost from the start node to node $n$.

### Properties

| Property | Value |
| :--- | :--- |
| **Complete** | Yes — finds a path if one exists |
| **Optimal** | Yes — finds minimum-cost path for non-negative weights |
| **Time complexity** | $O((|V| + |E|) \log |V|)$ with binary min-heap |
| **Space complexity** | $O(|V|)$ |

### Role in This Project

Dijkstra is used as a **performance baseline** for comparison with A\*. On the standard 50×50 grid (density 0.20, seed 42), Dijkstra explores approximately 1,809 nodes to reach the goal, compared to A\*'s 641.

---

## 3. A\* Search (`planning/astar.py`)

### Theory

A\* extends Dijkstra by adding a **heuristic function** $h(n)$ that estimates the remaining cost to the goal:

$$f(n) = g(n) + h(n)$$

A\* explores cells in order of increasing $f(n)$, which allows it to focus search toward the goal and explore far fewer nodes than Dijkstra in practice.

### Admissibility

For A\* to guarantee an optimal path, $h(n)$ must be **admissible** — it must never overestimate the true remaining cost $h^*(n)$:

$$h(n) \leq h^*(n) \quad \text{for all nodes } n$$

### Supported Heuristics

#### Euclidean Distance (default)

$$h(n) = \sqrt{(\Delta x)^2 + (\Delta y)^2}$$

- Admissible on both 4-connected and 8-connected grids.
- Never overestimates: the straight-line distance is always $\leq$ any grid path.
- **Default heuristic** in this project.

#### Octile Distance

$$h(n) = \max(\Delta x, \Delta y) + (\sqrt{2} - 1) \cdot \min(\Delta x, \Delta y)$$

- Exact unobstructed 8-connected lower bound.
- Admissible and consistent on 8-connected grids; tightest possible heuristic for diagonal movement.
- Useful when maximum search efficiency is needed.

#### Manhattan Distance

$$h(n) = |\Delta x| + |\Delta y|$$

- Admissible **only** for 4-connected grids (no diagonal movement).
- **Inadmissible on 8-connected grids**: a diagonal cell at distance $(1, 1)$ has Manhattan value 2 but actual grid cost $\sqrt{2} \approx 1.414$. The heuristic overestimates, which can cause suboptimal paths.
- Using Manhattan with `allow_diagonal=True` raises an explicit `UserWarning` at construction time in `AStarPlanner.__init__()`.
- Safe to use when `allow_diagonal=False`.

### Implementation Notes

- A binary min-heap (`heapq`) implements the open set.
- A tie-breaking secondary key (`h` value, then insertion counter) breaks priority ties.
- Corner-cutting through obstacle diagonals is prevented: a diagonal move is only allowed when both adjacent orthogonal cells are passable.

---

## 4. Movement Costs

```
get_neighbors() in planning/path_utils.py
```

| Move type | Direction | Base cost |
| :--- | :--- | :--- |
| Cardinal | N, S, E, W | `1.0` |
| Diagonal | NE, NW, SE, SW | `math.sqrt(2) ≈ 1.41421` |

When planning on a `Costmap`, the traversal penalty for the destination cell is added:

$$\text{step\_cost}(u \to v) = \text{dist}(u, v) \times \left(1 + w \cdot \frac{c(v)}{255}\right)$$

where $c(v)$ is the cell's costmap value (0–199 for soft inflation) and $w$ is `cost_weight` (configurable, default 1.0). Lethal and inscribed cells (cost ≥ 253) are never returned as traversable neighbors.

---

## 5. Clearance-Aware Costmap (`environment/costmap.py`)

The Costmap layer adds a safety model on top of the binary `GridMap`. It follows the ROS 2 costmap convention:

### Cost Bands

| Band | Cost Value | Meaning |
| :--- | :---: | :--- |
| Free | `0` | Open cell, no nearby obstacle |
| Soft inflation | `1–199` | Within inflation radius — penalised but passable |
| Inscribed infeasible | `253` | Within `robot_radius` — impassable |
| Lethal obstacle | `254` | Physical obstacle cell |

### Clearance Computation

Clearance is computed using an exact, vectorized NumPy Euclidean distance transform (`_compute_obstacle_clearance`). Each cell receives the shortest distance (in physical metres, scaled by `resolution`) to the nearest obstacle cell boundary or map border. Because the distance computation is implemented directly in vectorized NumPy, no external SciPy dependency is needed.

### Inflation Decay Models

Soft cost penalty within the inflation band ($r_{\text{robot}} \leq d < r_{\text{robot}} + r_{\text{inflation}}$):

**Exponential (default):**

$$c(d) = c_{\text{max}} \cdot \exp\!\left(-\alpha \cdot \frac{d - r_{\text{robot}}}{r_{\text{inflation}}}\right)$$

**Linear:**

$$c(d) = c_{\text{max}} \cdot \left(1 - \frac{d - r_{\text{robot}}}{r_{\text{inflation}}}\right)$$

where $c_{\text{max}} = 199$ and $\alpha$ = `cost_scaling_factor` (default 3.0).

---

## 6. Collision-Safe Path Smoother (`planning/path_smoother.py`)

### Motivation

Planners operating on discrete grids produce paths with many small, staircase-like turns. The smoother reduces waypoints while guaranteeing the resulting path is collision-free.

### Algorithm: Greedy Shortcutting

1. Start at `i = 0` (path start).
2. Find the furthest `j > i + 1` such that the direct segment from waypoint `i` to waypoint `j` is collision-free (verified by `CollisionChecker.is_path_valid()`).
3. Keep waypoint `j`, discard all intermediates. Set `i = j`.
4. Repeat until the goal is reached.
5. Re-validate the entire shortcutted path. If re-validation fails, return the original path as fallback.

### `SmoothingResult` Fields

| Field | Type | Description |
| :--- | :--- | :--- |
| `smoothed_path` | `list` | Waypoints of the output path |
| `original_path` | `list` | Input path (preserved) |
| `is_smoothed` | `bool` | Whether any shortcutting occurred |
| **`is_path_valid`** | **`bool`** | **Explicit safety status — set at every return site** |
| `original_length` | `float` | Total length of input path (metres) |
| `smoothed_length` | `float` | Total length of output path (metres) |
| `original_waypoint_count` | `int` | |
| `smoothed_waypoint_count` | `int` | |
| `length_reduction_ratio` | `float` | `1 - smoothed/original` |

### Safety Guarantees

- `is_path_valid=False` is set for: empty paths, single points outside map bounds or inside obstacles, and multi-point paths whose input fails validation.
- `is_path_valid=True` is set only when the path was verified collision-free.
- `Navigator.plan_to_goal()` checks `is_path_valid` before assigning the smoothed path as active. An invalid result clears navigation state and returns `False`.

---

## 7. Planned Future Algorithms

The following are **not implemented** in the current codebase:

| Algorithm | Purpose |
| :--- | :--- |
| Dynamic Window Approach (DWA) | Local dynamic obstacle avoidance |
| RRT / RRT\* | Sampling-based planning for continuous spaces |
| Particle Filter SLAM | Simultaneous localization and mapping |
| Deep Reinforcement Learning (PPO/SAC) | Learning-based navigation policy |
