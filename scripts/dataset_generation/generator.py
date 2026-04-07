"""OCP dataset generator — Approach B (full trajectory imitation).

For each sample the generator:
  1. Randomly samples a (init_state, target_state) pair that lies in the free
     space (outside all obstacles + safety margin).
  2. Calls the C++ Optimizer binary with --init / --target / --output flags.
  3. Parses the resulting ocp_solution JSON.
  4. Discards failed or timeout solves.
  5. Accumulates (input_vec, output_vec) rows and saves to a compressed .npz
     file together with a sidecar metadata JSON.

Input vector  (22,):
    [x0, y0, θ0, xT, yT, θT,
     obs1_x, obs1_y, obs1_r, …, obs5_x, obs5_y, obs5_r,
     safety_margin]

Output vector (cfg.OUTPUT_SIZE,):  where N = cfg.N_INTERVALS (set in config.py)
    [v0, ω0, v1, ω1, …, v{N-2}, ω{N-2}]               ← 2*(N-1) control values
    [x0, y0, θ0, x1, y1, θ1, …, x{N-1}, y{N-1}, θ{N-1}]  ← 3*N state values

Usage (from project root, mpc_dl conda env):
    python scripts/dataset_generation/generator.py [OPTIONS]

    --num-samples  N   number of successful samples to collect (default: 10000)
    --output       PATH  .npz output path   (default: data/dataset.npz)
    --workers      W   parallel worker processes (default: 4)
    --seed         S   master random seed (default: 42)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, wait as futures_wait, FIRST_COMPLETED
from typing import Optional

import numpy as np
import numpy.random as npr

# Make config importable when worker processes are spawned on non-fork platforms.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as cfg


# ── Column-name helpers ────────────────────────────────────────────────────────

def input_column_names() -> list[str]:
    """Return the ordered list of input feature names."""
    names = ["x0", "y0", "theta0", "xT", "yT", "thetaT"]
    for k in range(cfg.N_OBSTACLES):
        names += [f"obs{k + 1}_x", f"obs{k + 1}_y", f"obs{k + 1}_r"]
    names.append("safety_margin")
    return names   # length == cfg.INPUT_SIZE


def output_column_names() -> list[str]:
    """Return the ordered list of output label names."""
    ctrl = [c for k in range(cfg.N_CONTROLS_PER_TRAJ) for c in (f"v{k}", f"omega{k}")]
    state = [c for k in range(cfg.N_STATES_PER_TRAJ) for c in (f"x{k}", f"y{k}", f"theta{k}")]
    return ctrl + state   # length == cfg.OUTPUT_SIZE


# ── Free-space sampling ────────────────────────────────────────────────────────

def _in_obstacle(x: float, y: float) -> bool:
    """Return True if (x, y) lies inside any obstacle extended by the safety margin."""
    for obs in cfg.OBSTACLES:
        if math.sqrt((x - obs["x"]) ** 2 + (y - obs["y"]) ** 2) < obs["r"] + cfg.SAFETY_MARGIN:
            return True
    return False


def _sample_free_point(rng: npr.Generator) -> tuple[float, float]:
    """Uniformly sample a point in the free (non-obstacle) region of the floor."""
    while True:
        x = float(rng.uniform(cfg.FLOOR_X_MIN, cfg.FLOOR_X_MAX))
        y = float(rng.uniform(cfg.FLOOR_Y_MIN, cfg.FLOOR_Y_MAX))
        if not _in_obstacle(x, y):
            return x, y


# ── Single-sample solver call ──────────────────────────────────────────────────

def _solve_one(seed: int) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """
    Sample one problem instance, invoke the C++ binary, return (input, output).

    Returns None when:
      - the sampled start/target are too close together,
      - the binary times out or crashes,
      - IPOPT reports failure (stdout does not contain "OCP solved successfully."),
      - the JSON output has unexpected array lengths.
    """
    rng = npr.default_rng(seed)

    # ── Sample problem ──
    x0, y0 = _sample_free_point(rng)
    theta0 = float(rng.uniform(-math.pi, math.pi))

    xT, yT = _sample_free_point(rng)
    thetaT = float(rng.uniform(-math.pi, math.pi))

    if math.sqrt((x0 - xT) ** 2 + (y0 - yT) ** 2) < cfg.MIN_START_TARGET_DISTANCE:
        return None

    # ── Call binary ──
    # Each call gets its own temp file so parallel workers never conflict.
    fd, output_path = tempfile.mkstemp(suffix=".json")
    os.close(fd)

    try:
        result = subprocess.run(
            [
                cfg.BINARY_PATH,
                "--init",   str(x0),    str(y0),    str(theta0),
                "--target", str(xT),    str(yT),    str(thetaT),
                "--output", output_path,
                "--horizon", str(cfg.N_INTERVALS),
            ],
            capture_output=True,
            timeout=cfg.SOLVER_TIMEOUT_S,
        )

        # The binary prints "OCP solved successfully." to stdout on convergence.
        # We rely on this message rather than exit code because the CasADi
        # stats()["success"] cast is a known pre-existing issue in Ocp.cpp.
        if b"OCP solved successfully." not in result.stdout:
            return None

        with open(output_path) as fh:
            sol = json.load(fh)

        v_raw     = sol["controls"]["v"]
        omega_raw = sol["controls"]["omega"]
        x_raw     = sol["states"]["x"]
        y_raw     = sol["states"]["y"]
        theta_raw = sol["states"]["theta"]

        # Guard against truncated output.
        if (len(v_raw)     < cfg.N_CONTROLS_PER_TRAJ
                or len(x_raw) < cfg.N_STATES_PER_TRAJ):
            return None

        # Slice to the expected trajectory lengths defined by config.
        # The JSON may contain more values than needed (e.g. if N changed),
        # so we always take exactly N_CONTROLS_PER_TRAJ and N_STATES_PER_TRAJ.
        v     = np.asarray(v_raw[:cfg.N_CONTROLS_PER_TRAJ],     dtype=np.float32)
        omega = np.asarray(omega_raw[:cfg.N_CONTROLS_PER_TRAJ], dtype=np.float32)
        xs    = np.asarray(x_raw[:cfg.N_STATES_PER_TRAJ],       dtype=np.float32)
        ys    = np.asarray(y_raw[:cfg.N_STATES_PER_TRAJ],       dtype=np.float32)
        ts    = np.asarray(theta_raw[:cfg.N_STATES_PER_TRAJ],   dtype=np.float32)

        # ── Assemble input vector (22,) ──
        obs_flat = [val for obs in cfg.OBSTACLES for val in (obs["x"], obs["y"], obs["r"])]
        input_vec = np.asarray(
            [x0, y0, theta0, xT, yT, thetaT] + obs_flat + [cfg.SAFETY_MARGIN],
            dtype=np.float32,
        )

        # ── Assemble output vector (cfg.OUTPUT_SIZE,) ──
        # Controls interleaved: v0, ω0, v1, ω1, … (indices 0..2*cfg.N_CONTROLS_PER_TRAJ-1)
        ctrl_interleaved = np.empty(2 * cfg.N_CONTROLS_PER_TRAJ, dtype=np.float32)
        ctrl_interleaved[0::2] = v
        ctrl_interleaved[1::2] = omega

        # States interleaved: x0,y0,θ0, x1,y1,θ1, … (indices 2*cfg.N_CONTROLS_PER_TRAJ..cfg.OUTPUT_SIZE-1)
        state_interleaved = np.empty(3 * cfg.N_STATES_PER_TRAJ, dtype=np.float32)
        state_interleaved[0::3] = xs
        state_interleaved[1::3] = ys
        state_interleaved[2::3] = ts

        # Bug 4 fix: IPOPT's equality constraint state[0] = initState is satisfied
        # only up to acceptable_tol (~1e-3). Override the t=0 state triplet with
        # the exact Python-sampled values so input[0:3] == output[2*cfg.N_CONTROLS_PER_TRAJ:2*cfg.N_CONTROLS_PER_TRAJ+3] exactly.
        state_interleaved[0] = np.float32(x0)
        state_interleaved[1] = np.float32(y0)
        state_interleaved[2] = np.float32(theta0)

        output_vec = np.concatenate([ctrl_interleaved, state_interleaved])

        return input_vec, output_vec

    except (subprocess.TimeoutExpired, FileNotFoundError,
            KeyError, json.JSONDecodeError, ValueError):
        return None
    finally:
        if os.path.exists(output_path):
            os.unlink(output_path)


# ── Generation loop ────────────────────────────────────────────────────────────

def generate(
    num_samples: int,
    output_path: str,
    num_workers: int,
    seed: int,
) -> None:
    """Generate the dataset and write it to *output_path* (.npz)."""

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    if not os.path.isfile(cfg.BINARY_PATH):
        raise FileNotFoundError(
            f"C++ binary not found at: {cfg.BINARY_PATH}\n"
            "Build with:  cmake --build build/Release"
        )

    max_attempts = num_samples * cfg.MAX_ATTEMPTS_MULTIPLIER
    rng = npr.default_rng(seed)
    # Generate a large pool of unique integer seeds for workers.
    worker_seeds = rng.integers(0, 2 ** 31, size=max_attempts).tolist()

    inputs  = np.empty((num_samples, cfg.INPUT_SIZE),  dtype=np.float32)
    outputs = np.empty((num_samples, cfg.OUTPUT_SIZE), dtype=np.float32)

    n_collected = 0
    n_attempted = 0
    seed_iter   = iter(worker_seeds)
    t0          = time.perf_counter()

    # Rolling-window pool: keep at most (num_workers * 2) futures in flight.
    window = num_workers * 2

    print(f"Generating {num_samples} samples  |  workers={num_workers}  |  seed={seed}")
    print(f"Binary:  {cfg.BINARY_PATH}")
    print(f"Output:  {output_path}\n")

    with ProcessPoolExecutor(max_workers=num_workers) as pool:
        pending: set = set()

        # Prime the pool.
        for s in _take(seed_iter, window):
            pending.add(pool.submit(_solve_one, s))

        while n_collected < num_samples and (pending or n_attempted < max_attempts):
            if not pending:
                break

            done, pending = futures_wait(pending, return_when=FIRST_COMPLETED)

            for fut in done:
                n_attempted += 1
                res = fut.result()

                if res is not None and n_collected < num_samples:
                    inputs[n_collected]  = res[0]
                    outputs[n_collected] = res[1]
                    n_collected += 1

                    # Progress bar (simple inline).
                    if n_collected % 100 == 0 or n_collected == num_samples:
                        elapsed = time.perf_counter() - t0
                        rate    = n_collected / elapsed
                        pct     = n_collected / num_samples * 100
                        print(
                            f"  [{pct:5.1f}%]  {n_collected:>6}/{num_samples}"
                            f"  attempted={n_attempted}"
                            f"  success_rate={n_collected/n_attempted:.1%}"
                            f"  speed={rate:.1f} samples/s",
                            flush=True,
                        )

                # Replenish: keep the pool topped up.
                if n_collected < num_samples:
                    s = next(seed_iter, None)
                    if s is not None:
                        pending.add(pool.submit(_solve_one, s))

    inputs  = inputs[:n_collected]
    outputs = outputs[:n_collected]

    elapsed      = time.perf_counter() - t0
    success_rate = n_collected / max(n_attempted, 1)

    # ── Save ──
    np.savez_compressed(output_path, inputs=inputs, outputs=outputs)

    meta_path = output_path.replace(".npz", "_meta.json")
    metadata  = {
        "n_samples":         n_collected,
        "n_attempted":       n_attempted,
        "success_rate":      round(success_rate, 4),
        "elapsed_s":         round(elapsed, 2),
        "input_size":        cfg.INPUT_SIZE,
        "output_size":       cfg.OUTPUT_SIZE,
        "input_columns":     input_column_names(),
        "output_columns":    output_column_names(),
        "n_intervals":       cfg.N_INTERVALS,
        "sim_step":          cfg.SIM_STEP,
        "safety_margin":     cfg.SAFETY_MARGIN,
        "floor_bounds": {
            "x": [cfg.FLOOR_X_MIN, cfg.FLOOR_X_MAX],
            "y": [cfg.FLOOR_Y_MIN, cfg.FLOOR_Y_MAX],
        },
        "obstacles":         cfg.OBSTACLES,
        "seed":              seed,
        "binary":            cfg.BINARY_PATH,
    }
    with open(meta_path, "w") as fh:
        json.dump(metadata, fh, indent=2)

    print(f"\n{'─' * 60}")
    print(f"  Collected : {n_collected} / {num_samples} samples")
    print(f"  Attempted : {n_attempted}")
    print(f"  Success   : {success_rate:.1%}")
    print(f"  Time      : {elapsed:.1f} s")
    print(f"  Dataset   : {output_path}  →  inputs{inputs.shape}  outputs{outputs.shape}")
    print(f"  Metadata  : {meta_path}")
    print(f"{'─' * 60}")


# ── Utilities ──────────────────────────────────────────────────────────────────

def _take(it, n: int) -> list:
    """Consume up to *n* items from iterator *it*."""
    result = []
    for _ in range(n):
        val = next(it, None)
        if val is None:
            break
        result.append(val)
    return result


# ── CLI ────────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Generate an OCP behavioural-cloning dataset (Approach B)."
    )
    p.add_argument("--num-samples", type=int,  default=cfg.NUM_SAMPLES,
                   help=f"Number of successful samples to collect (default: {cfg.NUM_SAMPLES})")
    p.add_argument("--output",      type=str,  default=cfg.OUTPUT_PATH,
                   help=f"Output .npz path (default: {cfg.OUTPUT_PATH})")
    p.add_argument("--workers",     type=int,  default=cfg.NUM_WORKERS,
                   help=f"Parallel worker processes (default: {cfg.NUM_WORKERS})")
    p.add_argument("--seed",        type=int,  default=cfg.RANDOM_SEED,
                   help=f"Master random seed (default: {cfg.RANDOM_SEED})")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    generate(
        num_samples=args.num_samples,
        output_path=args.output,
        num_workers=args.workers,
        seed=args.seed,
    )
