# Roadmap

This document lists planned improvements and research directions for the AI-Based Autonomous Navigation System.

> All items listed here are **not yet implemented**. The current codebase is a software-only 2D simulation. Physical hardware and ROS integration are long-term items.

---

## Near-Term (Documentation & Infrastructure)

- [ ] Create sample pre-built map files in `data/maps/` (e.g., narrow corridor, open arena, maze).
- [ ] Add `outputs/plots/` trajectory plot generation to `scripts/analyze_benchmarks.py`.
- [ ] Create Jupyter notebooks in `notebooks/` for interactive algorithm visualization.
- [ ] Add GitHub Actions CI workflow (run `pytest` and `validate_consistency.py` on push).

---

## Medium-Term (Algorithm Enhancements)

- [ ] **Local dynamic obstacle avoidance**: Implement Dynamic Window Approach (DWA) or Vector Field Histogram (VFH) to react to moving obstacles without full replanning.
- [ ] **Incremental replanning**: Detect when detected obstacles invalidate the current path and trigger automatic partial replanning (e.g., D\* Lite).
- [ ] **Weighted A\* / Anytime A\***: Add a weight parameter $\varepsilon$ to trade optimality for faster search in cluttered environments.
- [ ] **Bi-directional A\***: Search simultaneously from start and goal; useful for very long corridors.
- [ ] **Probabilistic Roadmap (PRM)**: Sampling-based global planner for comparison with grid-based approaches.
- [ ] **RRT / RRT\***: Rapidly-Exploring Random Trees for continuous configuration spaces.

---

## Medium-Term (Robot Model Enhancements)

- [ ] **Differential-drive kinematics**: Replace the current point-mass model with a two-wheel differential-drive model with turning radius constraints.
- [ ] **Velocity and acceleration limits**: Enforce maximum angular and linear acceleration constraints in the robot model.
- [ ] **Non-holonomic path planning**: Generate paths that are kinematically feasible for a differential-drive robot (e.g., Dubins curves or Reeds-Shepp paths).

---

## Long-Term (Perception & SLAM)

- [ ] **Occupancy grid SLAM**: Build the map incrementally from simulated sensor readings using a particle filter or EKF-based approach.
- [ ] **Sensor noise modelling**: Add configurable Gaussian noise to the simulated LiDAR rays.
- [ ] **Unknown-map navigation**: Remove the assumption that the planner has full global map knowledge; force it to explore unknown regions.
- [ ] **Loop closure detection**: Recognize previously visited areas and correct accumulated drift.

---

## Long-Term (Learning-Based Approaches)

- [ ] **Deep Reinforcement Learning (DRL)**: Train a navigation policy (PPO or SAC) using the existing simulation as a Gym-compatible environment.
- [ ] **Imitation learning**: Learn from the A\*/Dijkstra oracle paths.
- [ ] **Model checkpoints**: Store trained weights in `models/`.

---

## Long-Term (Hardware & Integration)

- [ ] **ROS 2 bridge**: Publish planned paths as `nav_msgs/Path` topics; subscribe to odometry and sensor topics.
- [ ] **Hardware abstraction layer**: Decouple the `RobotModel` from the simulator so it can be replaced by a real robot's state.
- [ ] **Physical robot testing**: Test algorithms on a physical differential-drive platform (e.g., TurtleBot, Raspberry Pi robot).

---

## Completed

- [x] Binary occupancy `GridMap` with configurable resolution.
- [x] Clearance-aware `Costmap` with exponential/linear decay and boundary clearance.
- [x] Procedural obstacle generator with seed control.
- [x] A\* with Euclidean, Octile, and Manhattan heuristics; 4- and 8-connectivity.
- [x] Dijkstra baseline planner.
- [x] Geometric collision-safe path smoother with explicit `is_path_valid` safety field.
- [x] Navigator validity guard (rejects invalid smoothing results before assigning active path).
- [x] Circular footprint `CollisionChecker`.
- [x] Pygame 2D renderer with headless fallback.
- [x] Evaluation metrics module.
- [x] 161-test pytest suite.
- [x] 48-point multi-resolution physical consistency validator.
