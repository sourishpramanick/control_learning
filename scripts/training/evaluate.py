"""Evaluation of a trained OCP imitation model.

Loads a saved checkpoint and runs inference on the held-out test split,
then reports quantitative metrics and optionally generates trajectory plots
comparing the model's predictions against the ground-truth OCP solutions.

Metrics reported
----------------
- Overall MSE on the full 498-D output (normalised and physical space)
- Separate MSE for controls (198-D) and states (300-D)
- Per-output-type breakdown:
    • velocity v:   MSE  (m/s)
    • angular rate ω: MSE  (rad/s)
    • x position:   MSE  (m)
    • y position:   MSE  (m)
    • heading θ:    wrapped RMSE (rad) — uses geodesic error regardless of
                    which loss was used during training
- Obstacle violation rate: fraction of predicted trajectories where any
  state node falls inside an obstacle + safety margin

Usage (from project root, mpc_dl conda environment):

    conda run -n mpc_dl python scripts/training/evaluate.py \\
        --checkpoint checkpoints/run1/best.pt \\
        --dataset data/dataset4.npz

    # Also save trajectory comparison plots for the first N test samples
    conda run -n mpc_dl python scripts/training/evaluate.py \\
        --checkpoint checkpoints/run1/best.pt \\
        --dataset data/dataset4.npz \\
        --plot-samples 16 \\
        --plot-dir results/run1/
"""

from __future__ import annotations

import argparse
import sys
import os

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from training.config import TrainingConfig
from training.dataset import OcpDataset, Normalizer
from training.model import OcpMlp


class Evaluator:
    """Loads a checkpoint and evaluates the model on a dataset split.

    Parameters
    ----------
    checkpoint_path:
        Path to a .pt checkpoint produced by Trainer._save_checkpoint().
    dataset_path:
        Path to the .npz dataset. The test split is reconstructed using
        the same seed and fractions saved inside the checkpoint.
    device:
        'cuda', 'cpu', or 'auto' (default — picks CUDA if available).
    """

    def __init__(
        self,
        checkpoint_path: str,
        dataset_path: str,
        device: str = "auto",
    ) -> None:
        # Load checkpoint dict and reconstruct TrainingConfig.
        # Restore model weights and Normalizer.
        # Rebuild the test split using the saved seed / fractions so the
        # exact same rows are held out as during training.
        raise NotImplementedError

    def run(self) -> dict[str, float]:
        """Run inference on the full test set and return a metrics dict.

        The dict contains:
            mse_total, mse_ctrl, mse_state,
            mse_v, mse_omega, mse_x, mse_y, rmse_theta_wrapped,
            obstacle_violation_rate
        """
        raise NotImplementedError

    def print_report(self, metrics: dict[str, float]) -> None:
        """Print a human-readable table of metrics to stdout."""
        raise NotImplementedError

    def plot_trajectories(
        self,
        n_samples: int = 9,
        save_dir: str | None = None,
    ) -> None:
        """Plot predicted vs ground-truth trajectories for n_samples test cases.

        Each plot shows:
          - Floor map with obstacles
          - Ground-truth OCP trajectory (solid line)
          - Model-predicted trajectory (dashed line)
          - Start (green dot) and target (red dot)

        Reuses the obstacle / floor rendering logic from
        scripts/visualize_dataset.py.

        Args:
            n_samples: Number of test samples to plot.
            save_dir:  If provided, save PNG files here; otherwise display
                       interactively.
        """
        raise NotImplementedError

    def _compute_obstacle_violations(
        self,
        states: np.ndarray,
        obstacles: list[dict],
        safety_margin: float,
    ) -> np.ndarray:
        """Return boolean array (N,) — True where any node of trajectory i
        falls inside any obstacle + safety margin.

        Args:
            states:        (N, 300) state trajectories in physical units.
            obstacles:     list of {'x', 'y', 'r'} dicts.
            safety_margin: scalar margin added to each obstacle radius.
        """
        raise NotImplementedError


# ── CLI ────────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Evaluate a trained OCP imitation model.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--checkpoint",   type=str, required=True,
                   help="Path to best.pt or latest.pt checkpoint file.")
    p.add_argument("--dataset",      type=str, default=None,
                   help="Override dataset path (default: value saved in checkpoint).")
    p.add_argument("--plot-samples", type=int, default=0,
                   help="Number of test trajectories to plot (0 = skip plotting).")
    p.add_argument("--plot-dir",     type=str, default=None,
                   help="Directory to save trajectory plots (None = show interactively).")
    p.add_argument("--device",       type=str, default="auto",
                   choices=["auto", "cpu", "cuda"])
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    ev = Evaluator(
        checkpoint_path=args.checkpoint,
        dataset_path=args.dataset or "",   # empty → use checkpoint's saved path
        device=args.device,
    )
    metrics = ev.run()
    ev.print_report(metrics)

    if args.plot_samples > 0:
        ev.plot_trajectories(n_samples=args.plot_samples, save_dir=args.plot_dir)


if __name__ == "__main__":
    main()
