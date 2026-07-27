"""Configuration for the OCP dataset generator.

All tunable knobs are centralised here. Edit this file to change the environment
layout, OCP parameters, or generation settings — never hard-code values inside
generator.py.
"""

import json
import os

# ── Paths ─────────────────────────────────────────────────────────────────────
_THIS_DIR: str = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT: str = os.path.dirname(os.path.dirname(_THIS_DIR))

# Release binary is used (faster than Debug, no ASAN overhead).
BINARY_PATH: str = os.path.join(_PROJECT_ROOT, "build", "Release", "control_learning")

# ── Robot parameters (loaded from parameters.json) ────────────────────────────
_PARAMS_PATH: str = os.path.join(_PROJECT_ROOT, "src", "cpp", "robot", "data", "parameters.json")
with open(_PARAMS_PATH) as _f:
    _params = json.load(_f)["parameters"]

WHEEL_BASE:           float = float(_params["wheel_base"])
MAX_VELOCITY:         float = float(_params["max_velocity"])
MIN_VELOCITY:         float = float(_params["min_velocity"])
MAX_ANGULAR_VELOCITY: float = float(_params["max_angular_velocity"])
MIN_ANGULAR_VELOCITY: float = float(_params["min_angular_velocity"])

# ── Floor space (loaded from space.json) ─────────────────────────────────────
_SPACE_PATH: str = os.path.join(_PROJECT_ROOT, "src", "cpp", "robot", "environment", "space.json")
with open(_SPACE_PATH) as _f:
    _space = json.load(_f)

FLOOR_X_MIN: float = float(_space["x"]["min"])
FLOOR_X_MAX: float = float(_space["x"]["max"])
FLOOR_Y_MIN: float = float(_space["y"]["min"])
FLOOR_Y_MAX: float = float(_space["y"]["max"])

# ── Fixed obstacles (loaded from obstacles.json) ──────────────────────────────
_OBSTACLES_PATH: str = os.path.join(_PROJECT_ROOT, "src", "cpp", "robot", "environment", "obstacles.json")
with open(_OBSTACLES_PATH) as _f:
    _obs_json = json.load(_f)

SAFETY_MARGIN: float = float(_obs_json["safety_margin"])
OBSTACLES: list[dict] = [
    {"x": float(v["x"]), "y": float(v["y"]), "r": float(v["r"])}
    for k, v in _obs_json.items()
    if k != "safety_margin"
]

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
_ROBOT_PARAM_KEYS: tuple[str, ...] = (
    "wheel_base", "max_velocity", "min_velocity",
    "max_angular_velocity", "min_angular_velocity",
)
N_ROBOT_PARAMS: int = len(_ROBOT_PARAM_KEYS)
assert all(k in _params for k in _ROBOT_PARAM_KEYS), (
    f"parameters.json is missing one of {_ROBOT_PARAM_KEYS}"
)

# Input vector layout  →  total INPUT_SIZE = 27
#   [x0, y0, θ0]              initial state       (3)
#   [xT, yT, θT]              target  state       (3)
#   [ox_k, oy_k, or_k] × 5   obstacle positions  (15)
#   [safety_margin]           scalar              (1)
#   [wheel_base, max_v, min_v, max_ω, min_ω]     robot params (5)
INPUT_SIZE: int = 3 + 3 + 3 * N_OBSTACLES + 1 + N_ROBOT_PARAMS   # = 27

# Output vector layout  →  total OUTPUT_SIZE = 2*(N_INTERVALS-1) + 3*N_INTERVALS
#   [v0, ω0, v1, ω1, …, v{N-2}, ω{N-2}]               controls (interleaved)  (2*(N_INTERVALS-1))
#   [x0, y0, θ0, x1, y1, θ1, …, x{N-1}, y{N-1}, θ{N-1}]  states (interleaved)  (3*N_INTERVALS)
N_CONTROLS_PER_TRAJ: int = N_INTERVALS - 1
N_STATES_PER_TRAJ:   int = N_INTERVALS       # t=0 … t=N-1
OUTPUT_SIZE: int = 2 * N_CONTROLS_PER_TRAJ + 3 * N_STATES_PER_TRAJ
