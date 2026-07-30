"""Orchestrator for the full OCP imitation workflow.

OcpPipeline is a thin coordinator that calls the individual pipeline
components (data generation, training, evaluation) in sequence while
ensuring shared state (config, normalizer, checkpoint path) flows correctly
between stages.

It does NOT contain any ML logic — all of that lives in the dedicated
modules (dataset.py, model.py, loss.py, trainer.py, evaluate.py).

Typical usage
-------------
From a Python script or notebook:

    from scripts.training.config import TrainingConfig
    from scripts.training.pipeline import OcpPipeline

    cfg = TrainingConfig(
        dataset_path="data/dataset.npz",
        checkpoint_dir="checkpoints/run1",
        epochs=100,
    )
    pipeline = OcpPipeline(cfg)

    # Run the full workflow end-to-end
    pipeline.run()

    # Or run stages individually
    pipeline.train()
    pipeline.evaluate()

From the command line (all stages):

    conda run -n mpc_dl python scripts/training/pipeline.py \\
        --dataset data/dataset.npz \\
        --checkpoint-dir checkpoints/run1 \\
        --epochs 100

From the command line (evaluate only):

    conda run -n mpc_dl python scripts/training/pipeline.py \\
        --checkpoint-dir checkpoints/run1 \\
        --skip-training

Design notes
------------
- Each stage is independent: train() can be called without generate_data(),
  and evaluate() can be called without train() (as long as a checkpoint
  already exists in checkpoint_dir).
- Shared objects (normalizer, model, checkpoint path) are stored as instance
  attributes so stages can access them without re-loading from disk.
- Individual CLIs (train.py, evaluate.py) remain usable standalone.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from training.config import TrainingConfig
from training.trainer import Trainer
from training.evaluate import Evaluator


class OcpPipeline:
    """End-to-end pipeline: data → train → evaluate.

    Parameters
    ----------
    config:
        A TrainingConfig instance. All stages read their parameters from it.
    """

    def __init__(self, config: TrainingConfig) -> None:
        self.config = config

        # Set after train() completes; allows evaluate() to find the checkpoint
        # without the user having to specify the path manually.
        self._best_checkpoint: str | None = os.path.join(
            config.checkpoint_dir, "best.pt"
        )

    # ── Public interface ──────────────────────────────────────────────────────

    def run(self, skip_training: bool = False) -> None:
        """Run the full pipeline sequentially.

        Stages:
          1. train()    — unless skip_training=True
          2. evaluate() — always runs; uses best.pt from checkpoint_dir

        Args:
            skip_training: If True, skip to evaluation (checkpoint must
                           already exist in config.checkpoint_dir).
        """
        if not skip_training:
            self.train()
        self.evaluate()

    def train(self) -> None:
        """Instantiate Trainer and run the training loop.

        After completion, checkpoint_dir/best.pt and normalizer.json
        are guaranteed to exist on disk and self._best_checkpoint is set.
        """
        trainer = Trainer(self.config)
        trainer.train()
        self._best_checkpoint = os.path.join(self.config.checkpoint_dir, "best.pt")
        print(f"\nTraining complete. Best checkpoint: {self._best_checkpoint}")

    def evaluate(self, plot_samples: int = 0, plot_dir: str | None = None) -> dict[str, float]:
        """Load best.pt and run evaluation on the held-out test set.

        Args:
            plot_samples: Number of trajectory comparison plots to generate
                          (0 = skip plotting).
            plot_dir:     Directory for saving plots. None = display
                          interactively.

        Returns:
            metrics dict from Evaluator.run() (see evaluate.py for keys).
        """
        if self._best_checkpoint is None or not os.path.isfile(self._best_checkpoint):
            raise FileNotFoundError(
                f"Checkpoint not found: {self._best_checkpoint}\n"
                "Run pipeline.train() first, or set config.resume=True."
            )

        ev = Evaluator(
            checkpoint_path=self._best_checkpoint,
            dataset_path=self.config.dataset_path,
        )
        metrics = ev.run()
        ev.print_report(metrics)

        if plot_samples > 0:
            ev.plot_trajectories(n_samples=plot_samples, save_dir=plot_dir)

        return metrics


# ── CLI ────────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Run the full OCP imitation pipeline (train → evaluate).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--dataset",        type=str,   default=None)
    p.add_argument("--checkpoint-dir", type=str,   default=None)
    p.add_argument("--epochs",         type=int,   default=None)
    p.add_argument("--batch-size",     type=int,   default=None)
    p.add_argument("--lr",             type=float, default=None)
    p.add_argument("--seed",           type=int,   default=None)
    p.add_argument("--hidden-dims",    type=int,   nargs="+", default=None)
    p.add_argument("--skip-training",  action="store_true", default=False,
                   help="Skip training; go straight to evaluation.")
    p.add_argument("--plot-samples",   type=int,   default=0,
                   help="Number of trajectory plots to generate after evaluation.")
    p.add_argument("--plot-dir",       type=str,   default=None)
    p.add_argument("--resume",         action="store_true", default=False)
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    cfg = TrainingConfig()
    if args.dataset        is not None: cfg.dataset_path   = args.dataset
    if args.checkpoint_dir is not None: cfg.checkpoint_dir = args.checkpoint_dir
    if args.epochs         is not None: cfg.epochs         = args.epochs
    if args.batch_size     is not None: cfg.batch_size     = args.batch_size
    if args.lr             is not None: cfg.lr             = args.lr
    if args.seed           is not None: cfg.seed           = args.seed
    if args.hidden_dims    is not None: cfg.hidden_dims    = args.hidden_dims
    if args.resume:                     cfg.resume         = True

    pipeline = OcpPipeline(cfg)
    pipeline.run(skip_training=args.skip_training)


if __name__ == "__main__":
    main()
