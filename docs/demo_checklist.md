# Demo Checklist — Screenshots & Recording Guidance

This document explains how to capture genuine, representative screenshots and a short video demonstration of the AI-Based Autonomous Navigation System.

> **Integrity note**: Do not present mockups, stock images, or edited screenshots as real application output. All demonstration assets should be captured directly from a running simulation.

---

## Prerequisites

Before capturing any assets, confirm the simulation runs correctly on your machine:

```bash
python main.py --headless     # smoke-test: exits 0 if goal is reached
pytest -q                     # 161 passed, 1 warning
```

---

## Screenshot List

Capture the following screenshots from a live pygame window. Each screenshot demonstrates a distinct feature.

---

### Screenshot 1 — Basic A\* Path Planning

**What it shows**: A complete planned path from start to goal on a randomly generated obstacle map.

**Command**:
```bash
python main.py --seed 42
```

**Wait for**: The path to appear (green cells) after planning completes. Pause or observe the first frame before the robot starts moving (the window stays open during the simulation).

**Key elements to show in the screenshot**:
- Grid cells (white = free, black = obstacle).
- Planned path highlighted in green.
- Explored nodes shown in grey.
- Robot position (coloured circle).
- Sensor rays (yellow lines radiating outward).

**Filename suggestion**: `assets/images/01_astar_basic_path.png`

---

### Screenshot 2 — Clearance-Aware Costmap Overlay

**What it shows**: The costmap inflation layers around obstacles, showing the soft-cost gradient.

**Command**:
```bash
python main.py --seed 42 --costmap
```

**Key elements to show**:
- Obstacle cells (black).
- Inscribed infeasible zone (dark red / deep inflation colour around each obstacle).
- Soft inflation gradient (lighter colours further from obstacles).
- Path curving away from obstacle edges compared to Screenshot 1.

**Filename suggestion**: `assets/images/02_costmap_overlay.png`

---

### Screenshot 3 — Path Before and After Smoothing

**What it shows**: The reduction in waypoints achieved by geometric path smoothing.

Capture **two screenshots** in sequence or side-by-side:

**Without smoothing**:
```bash
python main.py --seed 42 --planner astar
```

**With smoothing**:
```bash
python main.py --seed 42 --planner astar --smooth-path
```

**Key difference**: The smoothed path should have far fewer sharp turns (reported: 54 → 15 waypoints on the default seed-42 map).

**Filename suggestions**:
- `assets/images/03a_path_unsmoothed.png`
- `assets/images/03b_path_smoothed.png`

---

### Screenshot 4 — Dijkstra vs. A\* Exploration Comparison

**What it shows**: A\* explores many fewer cells than Dijkstra due to its heuristic guidance.

Capture the **explored nodes** overlay for each:

```bash
python main.py --seed 42 --planner astar      # ~641 explored nodes
python main.py --seed 42 --planner dijkstra   # ~1809 explored nodes
```

**Key difference**: Dijkstra should have a much larger grey (explored) region on the grid.

**Filename suggestions**:
- `assets/images/04a_explored_astar.png`
- `assets/images/04b_explored_dijkstra.png`

---

### Screenshot 5 — Goal Reached

**What it shows**: A completed simulation run where the robot has reached the goal.

Let any configuration run to completion. Capture the final frame showing the robot at the goal position.

**Filename suggestion**: `assets/images/05_goal_reached.png`

---

## Video Recording

A short (30–60 second) screen recording of a running simulation is a strong addition for GitHub and portfolio presentation.

### Recommended setup

```bash
python main.py --seed 42 --planner astar --costmap --smooth-path
```

This configuration shows:
- Costmap gradient overlay.
- Smooth path with few waypoints.
- Robot moving fluidly toward goal.

### Recording tools

| Platform | Recommended tool |
| :--- | :--- |
| Windows | Xbox Game Bar (`Win + G → Record`) or OBS Studio |
| macOS | QuickTime Player → File → New Screen Recording |
| Linux | OBS Studio or `ffmpeg` with `x11grab` |

### What to capture

- The full pygame window (700×700 px).
- The first ~30 seconds: path appears, robot begins moving.
- The final ~5 seconds: robot reaches goal.

**Filename suggestion**: `assets/videos/demo_astar_costmap_smooth.mp4`

---

## What to Avoid

- **Do not** present generated images or AI-created screenshots as real application output.
- **Do not** edit screenshots to add features that do not exist in the running code.
- **Do not** claim the simulation runs on physical hardware.
- **Do not** record a session that crashes or fails the goal and present it as a success.

---

## Adding Assets to the README

Once screenshots are captured, reference them in `README.md` like this:

```markdown
## Demo

### A* Path Planning
![A* path on 50x50 grid](assets/images/01_astar_basic_path.png)

### Costmap Inflation Overlay
![Costmap layers](assets/images/02_costmap_overlay.png)
```

Place this section after the **Features** section in the README for maximum visual impact.
