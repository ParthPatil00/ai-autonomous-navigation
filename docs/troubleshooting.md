# Troubleshooting Guide

This document covers common problems encountered when installing or running the AI-Based Autonomous Navigation System.

---

## Installation Issues

### `ModuleNotFoundError: No module named 'autonomous_navigation'`

**Cause**: The package was not installed in editable mode, so Python cannot find the `src/` layout.

**Fix**:
```bash
pip install -e .
```
Run this command from the repository root (the directory that contains `pyproject.toml`). The `-e` flag installs the package in "editable" mode, pointing directly at the `src/` directory.

---

### Is SciPy required for running the Costmap?

**Answer**: No. `Costmap` implements an exact, vectorized distance transform using pure NumPy (`_compute_obstacle_clearance`). The system runs completely out-of-the-box with only the dependencies listed in `requirements.txt` (`numpy`, `pygame`, `matplotlib`, `pyyaml`, and `pytest`).

---

### `ModuleNotFoundError: No module named 'pygame'`

**Cause**: Pygame is missing from the environment.

**Fix**:
```bash
pip install pygame>=2.5.0
```

---

### `pip install -e .` fails with "No module named pip" or "pyproject.toml not found"

**Cause**: Either pip is not installed, or the command is being run from the wrong directory.

**Fix**:
- Confirm you are in the repository root (the folder containing `pyproject.toml` and `README.md`).
- If pip is missing: `python -m ensurepip --upgrade`

---

## Running the Simulation

### `pygame.error: No video mode has been set` or blank window

**Cause**: Running in GUI mode on a system with no display (e.g., a remote server or WSL without an X server).

**Fix**: Use headless mode:
```bash
python main.py --headless
```

---

### Simulation exits immediately with "Goal blocked" or returns failure

**Cause**: The random obstacle generator placed an obstacle at or adjacent to the start or goal position.

**Fix**: Try a different seed:
```bash
python main.py --seed 100
```

Or reduce obstacle density:
```bash
python main.py --density 0.10
```

---

### Simulation runs slowly in GUI mode

**Cause**: Pygame frame rendering at 30 FPS can be slow on some machines, especially when `show_explored=true` fills the explored-nodes overlay.

**Fixes**:
- Run headless: `python main.py --headless`
- Reduce grid size via config: edit `environment.width` and `environment.height` in `configs/default.yaml`.

---

### `UserWarning: Manhattan heuristic is inadmissible on 8-connected grids`

**Cause**: You (or a config) requested `heuristic: manhattan` with `allow_diagonal: true`. Manhattan distance overestimates costs with diagonal movement, which can make A\* return suboptimal paths.

**Fix**: Change the heuristic in `configs/default.yaml`:
```yaml
planner:
  heuristic: "euclidean"   # or "octile"
  allow_diagonal: true
```

Or switch off diagonal movement:
```yaml
planner:
  heuristic: "manhattan"
  allow_diagonal: false
```

---

## Running Tests

### Tests cannot find `autonomous_navigation`

**Cause**: `pytest` is running from a directory where `src/` is not on the Python path.

**Fix**:
- Run `pip install -e .` first (see above).
- Alternatively, set `PYTHONPATH`:
  ```bash
  # Linux/macOS
  PYTHONPATH=src pytest

  # Windows PowerShell
  $env:PYTHONPATH = "src"; pytest
  ```

---

### `UserWarning: pkg_resources is deprecated` during pytest

**Cause**: This warning is emitted by `pygame.pkgdata` and is a third-party upstream issue in setuptools. It is not a defect in this project.

**Status**: Cosmetic only. All 161 tests pass. No action required.

---

### Fewer than 161 tests are collected

**Cause**: A test file was accidentally excluded, or an import error prevents a test module from loading.

**Fix**:
```bash
pytest --collect-only
```
Check for any `ERROR` lines in the collection output. Import errors will be shown with the failing module name.

---

## Consistency Validator

### `validate_consistency.py` reports failures

**Expected output**: `SUMMARY: 48 passed, 0 failed out of 48`

If failures appear, they indicate a physics/geometry inconsistency between `Costmap`, `CollisionChecker`, or coordinate conversions — likely caused by a source-code edit that changed clearance calculation or resolution handling.

Check:
1. `environment/costmap.py` — `_compute_obstacle_clearance()` and `update()`.
2. `navigation/collision_checker.py` — `is_state_valid()`.
3. `environment/grid_map.py` — `grid_to_world()` and `world_to_grid()`.
