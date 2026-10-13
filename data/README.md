# Data Directory

This directory stores map definitions, obstacle benchmark maps, and scenario configurations for simulation and evaluation.

## Subdirectories

- `maps/`: Predefined 2D occupancy grid maps (saved as `.csv`, `.npy`, or `.yaml` format).
- `scenarios/`: Multi-run scenarios containing predefined start/goal pairings and obstacle layouts.

## Standard Map Format

Maps can be represented as binary occupancy grids where:
- `0`: Free traversable space
- `1`: Static obstacle / wall
