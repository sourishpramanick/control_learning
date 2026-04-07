"""Configuration for the OCP dataset generator.

All tunable knobs are centralised here. Edit this file to change the environment
layout, OCP parameters, or generation settings — never hard-code values inside
generator.py.
"""

import os

# ── Paths ─────────────────────────────────────────────────────────────────────
_THIS_DIR: str = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT: str = os.path.dirname(os.path.dirname(_THIS_DIR))

# Release binary is used (faster than Debug, no ASAN overhead).
BINARY_PATH: str = os.path.join(_PROJECT_ROOT, "build", "Release", "control_learning")

# ── Floor space (must match space.json) ───────────────────────────────────────
FLOOR_X_MIN: float = 0.0
FLOOR_X_MAX: float = 20.0
FLOOR_Y_MIN: float = 0.0
FLOOR_Y_MAX: float = 20.0

# ── Fixed obstacles (must match obstacles.json) ───────────────────────────────
OBSTACLES: list[dict] = [
    {"x": 2.0,  "y": 2.0,  "r": 1.0},
    {"x": 7.0,  "y": 6.0,  "r": 2.0},
    {"x": 16.0, "y": 15.0, "r": 3.0},
    {"x": 5.0,  "y": 15.0, "r": 2.0},
    {"x": 15.0, "y": 7.5,  "r": 2.0},
]
SAFETY_MARGIN: float = 0.5          # must match obstacles.json → safety_margin

# ── OCP parameters (must match Optimizer::OCP_INTERVAL in Optimizer.hpp) ────────
N_INTERVALS: int = 100              # number of discretisation intervals — must equal Optimizer::OCP_INTERVAL
SIM_STEP: float = 0.2               # seconds per step

# ── Sampling constraints ───────────────────────────────────────────────────────
# Minimum Euclidean distance between sampled start and target XY positions.
# Prevents trivial (near-zero displacement) problems.
MIN_START_TARGET_DISTANCE: float = 5.0   # metres

# ── Generation settings ────────────────────────────────────────────────────────
NUM_SAMPLES: int = 100
RANDOM_SEED: int = 42
OUTPUT_PATH: str = os.path.join(_PROJECT_ROOT, "data", "dataset.npz")
NUM_WORKERS: int = os.cpu_count() - 1  # Leave one core free to keep the system responsive. Adjust if you know what you're doing.
SOLVER_TIMEOUT_S: float = 10.0      # per-solve wall-clock budget (seconds)
# Maximum number of solver attempts before giving up (guards against bad seeds).
MAX_ATTEMPTS_MULTIPLIER: int = 5    # attempts = NUM_SAMPLES * MAX_ATTEMPTS_MULTIPLIER

# ── Derived sizes (do not edit) ────────────────────────────────────────────────
N_OBSTACLES: int = len(OBSTACLES)

# Input vector layout  →  total INPUT_SIZE = 22
#   [x0, y0, θ0]              initial state       (3)
#   [xT, yT, θT]              target  state       (3)
#   [ox_k, oy_k, or_k] × 5   obstacle positions  (15)
#   [safety_margin]           scalar              (1)
INPUT_SIZE: int = 3 + 3 + 3 * N_OBSTACLES + 1   # = 22

# Output vector layout  →  total OUTPUT_SIZE = 2*(N_INTERVALS-1) + 3*N_INTERVALS
#   [v0, ω0, v1, ω1, …, v{N-2}, ω{N-2}]               controls (interleaved)  (2*(N_INTERVALS-1))
#   [x0, y0, θ0, x1, y1, θ1, …, x{N-1}, y{N-1}, θ{N-1}]  states (interleaved)  (3*N_INTERVALS)
N_CONTROLS_PER_TRAJ: int = N_INTERVALS - 1
N_STATES_PER_TRAJ:   int = N_INTERVALS       # t=0 … t=N-1
OUTPUT_SIZE: int = 2 * N_CONTROLS_PER_TRAJ + 3 * N_STATES_PER_TRAJ
