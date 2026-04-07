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
| Discretisation | N = 100 intervals, `sim_step = 0.2 s` (total horizon 20 s) |
| Floor space | `[0, 20] × [0, 20]` m |
| Obstacles | 5 fixed circular obstacles (see `environment/obstacles.json`) |
| Safety margin | 0.5 m around each obstacle radius |
| Cost function | Weighted quadratic tracking + control effort + terminal cost (θ wrapped — see below) |
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

### Output vector — shape `(N, 498)`

| Index range | Description |
|---|---|
| 0–197 | Controls interleaved: `v₀, ω₀, v₁, ω₁, …, v₉₈, ω₉₈` (99 steps × 2) |
| 198–497 | States interleaved: `x₀, y₀, θ₀, x₁, y₁, θ₁, …, x₉₉, y₉₉, θ₉₉` (100 steps × 3) |

State index 0 in the output (`output[198:201]`) is the initial state (identical
to `input[0:3]`), which the network can use as a consistency anchor during training.
Theta values in the output are **not** wrapped — they are the raw unbounded
angles returned by IPOPT (since theta bounds were removed from the NLP; see below).

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

4. **Parse and slice** the output JSON to exactly the lengths defined by
   `config.py`:
   - `controls['v'][:99]` and `controls['omega'][:99]` (99 control steps)
   - `states['x'][:100]`, `states['y'][:100]`, `states['theta'][:100]` (100 state nodes)

5. **Assemble** the 22-D input and 498-D output vectors.

6. **Save** accumulated arrays to a compressed NumPy `.npz` file plus a JSON
   sidecar with full metadata.

---

## C++ binary changes

The following changes were made to expose the CLI interface used by the
generator (no change to the OCP formulation):

| File | Change |
|---|---|
| `src/cpp/main.cpp` | Added `--init`, `--target`, `--output`, `--horizon`, `--mpc` argument parsing |
| `src/cpp/robot/ocp/Optimizer.hpp` | Added overload `static int Optimize(initState, target, outputPath, ocp_horizon)` |
| `src/cpp/robot/ocp/Optimizer.cpp` | Implemented the new overload; refactored `Optimize()` to delegate to it |
| `src/cpp/robot/ocp/Ocp.cpp` | Removed `±π` theta bounds; added wrapped-theta cost; linear-interpolation initial guess |

---

## Cost function and theta wrapping

The OCP minimises a weighted sum of control effort and state-tracking error
over the horizon:

$$J = \sum_{k=0}^{N-2} \left[ u_k^\top R\, u_k + e_k^\top Q\, e_k \right] + e_{N-1}^\top Q_f\, e_{N-1}$$

where $u_k = (v_k, \omega_k)$ and $e_k$ is the **state error** at node $k$.

### Why naive angle error fails

For the position components of $e_k$ a raw difference $x_k - x_T$ is well-defined.  For the heading component a raw difference $\theta_k - \theta_T$ is **not**: angles are periodic with period $2\pi$, so $+350°$ and $-10°$ represent the same heading, yet their raw difference from $0°$ differs by $360°$.

With an unbounded `theta` NLP variable the cost gradient

$$\frac{\partial}{\partial\theta_k}(\theta_k - \theta_T)^2 = 2(\theta_k - \theta_T)$$

always pushes $\theta_k$ toward $\theta_T$ by the shortest *linear* path.  This can be the *long way around* the circle.  For example with $\theta_k = -177°$ and $\theta_T = 0°$: the raw error is $-177°$, so IPOPT rotates CCW through $177°$ rather than CW through $183°$ — both reach $0°$, but the gradient cannot see that CW reaches $0°$ faster once the robot has spun past $-180°$.

### The fix — geodesic (wrapped) angular error

Replace $(\theta_k - \theta_T)$ with the **shortest signed arc** on the unit circle:

$$\Delta\theta^{\text{wrapped}}_k = \operatorname{atan2}\bigl(\sin(\theta_k - \theta_T),\; \cos(\theta_k - \theta_T)\bigr)$$

This maps any real-valued angle difference to $(-\pi, \pi]$.  The gradient of $(\Delta\theta^{\text{wrapped}})^2$ always points toward the **geometrically shortest** rotation direction, so IPOPT chooses the minimal spin regardless of how many times the robot has wound around.

Implemented in `Ocp::setupOcp` via the CasADi symbolic expression:

```cpp
stateErr(2) = casadi::SX::atan2(
    casadi::SX::sin(states(2) - targetState(2)),
    casadi::SX::cos(states(2) - targetState(2)));
```

This is applied to both the **stage cost** (every node $k = 0 \ldots N-2$) and the **terminal cost** (node $N-1$, with weight $Q_f = 10Q$).

### Removing the theta bounds

The NLP variable `theta` previously had hard bounds $[-\pi, \pi]$.  With those bounds, IPOPT could not rotate the robot CW through $-\pi$ even when that was the short direction — the bound acted as a wall.  Removing the bounds (setting them to $\pm\infty$) in combination with the wrapped cost fully resolves the long-spin problem.

The zero-argument `Optimize()` is unchanged in behaviour — it still reads from
the environment JSON files and writes to `ocp_solution.json`.

---

## Output files

| File | Description |
|---|---|
| `data/dataset.npz` | Compressed NumPy archive with keys `inputs (N,22)` and `outputs (N,498)` |
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
Y = data["outputs"]  # shape (N, 498)

with open("data/dataset_meta.json") as f:
    meta = json.load(f)

input_cols  = meta["input_columns"]   # list of 22 names
output_cols = meta["output_columns"]  # list of 498 names
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
