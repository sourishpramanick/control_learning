"""CLI entry point for training the OCP imitation model.

Usage (from project root, mpc_dl conda environment):

    # Train with all defaults (reads data/dataset.npz)
    conda run -n mpc_dl python scripts/training/train.py

    # Custom dataset, checkpoint directory, and number of epochs
    conda run -n mpc_dl python scripts/training/train.py \\
        --dataset data/dataset4.npz \\
        --checkpoint-dir checkpoints/run1 \\
        --epochs 200 \\
        --lr 5e-4 \\
        --batch-size 512

    # Resume a previous run
    conda run -n mpc_dl python scripts/training/train.py \\
        --checkpoint-dir checkpoints/run1 \\
        --resume

All arguments map 1-to-1 to TrainingConfig fields and override the
dataclass defaults. Unspecified arguments use the defaults in config.py.
"""

from __future__ import annotations

import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from training.config import TrainingConfig
from training.trainer import Trainer


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Train the OCP trajectory imitation MLP.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # Data
    p.add_argument("--dataset",        type=str,   default=None,
                   help="Path to .npz dataset file (default: TrainingConfig.dataset_path)")
    # Model
    p.add_argument("--hidden-dims",    type=int,   nargs="+", default=None,
                   help="Hidden layer widths, e.g. --hidden-dims 256 512 512 256")
    p.add_argument("--dropout",        type=float, default=None)
    # Training
    p.add_argument("--epochs",         type=int,   default=None)
    p.add_argument("--batch-size",     type=int,   default=None)
    p.add_argument("--lr",             type=float, default=None)
    p.add_argument("--weight-decay",   type=float, default=None)
    p.add_argument("--seed",           type=int,   default=None)
    # Loss
    p.add_argument("--ctrl-weight",    type=float, default=None)
    p.add_argument("--state-weight",   type=float, default=None)
    # I/O
    p.add_argument("--checkpoint-dir", type=str,   default=None)
    p.add_argument("--resume",         action="store_true", default=False,
                   help="Resume training from checkpoint-dir/latest.pt")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    # Start from dataclass defaults, override only the args that were supplied.
    cfg = TrainingConfig()
    if args.dataset        is not None: cfg.dataset_path    = args.dataset
    if args.hidden_dims    is not None: cfg.hidden_dims      = args.hidden_dims
    if args.dropout        is not None: cfg.dropout          = args.dropout
    if args.epochs         is not None: cfg.epochs           = args.epochs
    if args.batch_size     is not None: cfg.batch_size       = args.batch_size
    if args.lr             is not None: cfg.lr               = args.lr
    if args.weight_decay   is not None: cfg.weight_decay     = args.weight_decay
    if args.seed           is not None: cfg.seed             = args.seed
    if args.ctrl_weight    is not None: cfg.ctrl_weight      = args.ctrl_weight
    if args.state_weight   is not None: cfg.state_weight     = args.state_weight
    if args.checkpoint_dir is not None: cfg.checkpoint_dir  = args.checkpoint_dir
    if args.resume:                     cfg.resume           = True

    trainer = Trainer(cfg)
    trainer.train()


if __name__ == "__main__":
    main()
