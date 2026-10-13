# Experiments and Benchmarking

This guide describes how to run and reproduce planning and navigation experiments.

## Benchmarking Script

The benchmarking suite runs comparative evaluations between planning algorithms across different obstacle density levels and map resolutions:

```bash
python scripts/run_benchmarks.py --runs 5 --density 0.20
```

## Tracked Metrics

1. **Path Length**: Cumulative Euclidean distance of the planned trajectory from start to goal.
2. **Nodes Explored**: Number of grid nodes evaluated during path planning (indicates search efficiency).
3. **Execution Time (ms)**: Computation time consumed by the path planning algorithm.
4. **Smoothness**: Path angle curvature and direction changes along waypoints.
5. **Success Rate**: Percentage of runs reaching the target destination without collisions.

Benchmark outputs are stored automatically under `outputs/metrics/` and plots under `outputs/plots/`.
