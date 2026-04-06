# Data Generation for OCP Behavioural Cloning

## Overview

This document describes how the training dataset is generated for
**Approach B — Full Trajectory Imitation** (supervised behavioural cloning of
the OCP solver).

The goal is to learn the mapping

$$f_\theta: \mathbf{x}_\text{problem} \;\longmapsto\; \mathbf{y}_\text{trajectory}$$

so that a neural network can reproduce the optimal control trajectory that
IPOPT would compute, without running the solver at inference time.

---

## System being imitated

`Optimizer::Optimize()` in `src/cpp/robot/ocp/Optimizer.cpp` solves a
single-shot trajectory optimisation problem for a unicycle robot:

| Item | Value |
|---|---|
| Robot model | Unicycle — states `(x, y, θ)`, controls `(v, ω)` |
| Discretisation | N = 50 intervals, `sim_step = 0.2 s` |
| Floor space | `[0, 20] × [0, 20]` m |
| Obstacles | 5 fixed circular obstacles (see `environment/obstacles.json`) |
| Safety margin | 0.5 m around each obstacle radius |
| Cost function | Weighted quadratic tracking + control effort + terminal cost |
| Solver | IPOPT via CasADi 3.6.7 |

---

## Dataset schema

### Input vector — shape `(N, 22)`

| Index range | Description |
|---|---|
| 0–2 | Initial state `(x₀, y₀, θ₀)` |
| 3–5 | Target state `(xT, yT, θT)` |
| 6–8 | Obstacle 1 `(x, y, r)` |
| 9–11 | Obstacle 2 `(x, y, r)` |
| 12–14 | Obstacle 3 `(x, y, r)` |
| 15–17 | Obstacle 4 `(x, y, r)` |
| 18–20 | Obstacle 5 `(x, y, r)` |
| 21 | Safety margin (scalar) |

### Output vector — shape `(N, 248)`

| Index range | Description |
|---|---|
| 0–97 | Controls interleaved: `v₀, ω₀, v₁, ω₁, …, v₄₈, ω₄₈` (49 steps × 2) |
| 98–247 | States interleaved: `x₀, y₀, θ₀, x₁, y₁, θ₁, …, x₄₉, y₄₉, θ₄₉` (50 steps × 3) |

State index 0 in the output is the initial state (same as inputs 0–2), which
the network can use as a consistency anchor during training.

---

## Generation process

The generator (`scripts/dataset_generation/generator.py`) follows these steps
for each sample:

1. **Sample** `(x₀, y₀)` and `(xT, yT)` uniformly from the floor, rejecting
   points inside any obstacle + safety margin.  Angles `θ₀`, `θT` are drawn
   uniformly from `(-π, π)`.  Pairs where the Euclidean distance between start
   and target XY is below `MIN_START_TARGET_DISTANCE` (2 m) are rejected to
   avoid trivially easy problems.

2. **Invoke the C++ binary** with CLI flags:
   ```
   ./build/Release/control_learning \
       --init  x0 y0 theta0 \
       --target xT yT thetaT \
       --output /tmp/<unique>.json
   ```
   Each worker writes to its own temporary file so parallel runs never
   conflict.

3. **Detect convergence** by checking that the binary's stdout contains
   `"OCP solved successfully."`.  Failed solves are discarded.

4. **Parse and slice** the output JSON, discarding the trailing garbage values
   produced by an off-by-one in `Ocp::extractSolution`:
   - `controls['v'][:49]` and `controls['omega'][:49]` (valid control steps)
   - `states['x'][:50]`, `states['y'][:50]`, `states['theta'][:50]`

5. **Assemble** the 22-D input and 248-D output vectors.

6. **Save** accumulated arrays to a compressed NumPy `.npz` file plus a JSON
   sidecar with full metadata.

---

## C++ binary changes

The following changes were made to expose the CLI interface used by the
generator (no change to the OCP formulation):

| File | Change |
|---|---|
| `src/cpp/main.cpp` | Added `--init`, `--target`, `--output`, `--mpc` argument parsing |
| `src/cpp/robot/ocp/Optimizer.hpp` | Added overload `static int Optimize(initState, target, outputPath)` |
| `src/cpp/robot/ocp/Optimizer.cpp` | Implemented the new overload; refactored `Optimize()` to delegate to it |

The zero-argument `Optimize()` is unchanged in behaviour — it still reads from
the environment JSON files and writes to `ocp_solution.json`.

---

## Output files

| File | Description |
|---|---|
| `data/dataset.npz` | Compressed NumPy archive with keys `inputs (N,22)` and `outputs (N,248)` |
| `data/dataset_meta.json` | JSON metadata: column names, config snapshot, solve stats |

---

## How to run

```bash
# From project root, mpc_dl conda environment
conda run -n mpc_dl python scripts/dataset_generation/generator.py \
    --num-samples 10000 \
    --workers 4 \
    --output data/dataset.npz \
    --seed 42
```

Tune `--workers` to the number of available CPU cores.
Each solve takes ~0.1–0.5 s; 10 000 samples at 4 workers ≈ 5–20 min.

---

## Loading the dataset in Python

```python
import numpy as np
import json

data = np.load("data/dataset.npz")
X = data["inputs"]   # shape (N, 22)
Y = data["outputs"]  # shape (N, 248)

with open("data/dataset_meta.json") as f:
    meta = json.load(f)

input_cols  = meta["input_columns"]   # list of 22 names
output_cols = meta["output_columns"]  # list of 248 names
```

---

## Scaling notes for larger datasets

| N samples | Approx. time (4 workers) | File size (compressed) |
|---|---|---|
| 10 000 | ~10–20 min | ~4 MB |
| 100 000 | ~2–4 h | ~40 MB |
| 500 000 | ~10–20 h | ~200 MB |

For large runs use `--workers` equal to the number of physical cores.
The generation is embarrassingly parallel and scales linearly.
