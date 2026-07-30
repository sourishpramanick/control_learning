"""Training hyperparameters for the OCP imitation model.

All tunable knobs live here. Pass a TrainingConfig instance to Trainer.
Edit this file or override individual fields via train.py CLI arguments.
"""

from dataclasses import dataclass, field
import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))


@dataclass
class TrainingConfig:
    # ── Data ──────────────────────────────────────────────────────────────────
    dataset_path:   str   = os.path.join(_PROJECT_ROOT, "data", "dataset.npz")
    train_fraction: float = 0.8   # fraction of samples used for training
    val_fraction:   float = 0.1   # fraction used for validation (rest = test)

    # ── Model ─────────────────────────────────────────────────────────────────
    # Width of each hidden layer in the shared MLP backbone.
    # Deeper / wider → more capacity but slower training and inference.
    hidden_dims: list[int] = field(default_factory=lambda: [256, 512, 512, 256])
    dropout:     float     = 0.0  # dropout probability (0 = disabled)
    input_dim:   int       = 27   # must match dataset INPUT_SIZE
    n_controls:  int       = 198  # 2 * (N_INTERVALS - 1)
    n_states:    int       = 300  # 3 *  N_INTERVALS

    # ── Loss ──────────────────────────────────────────────────────────────────
    ctrl_weight:  float = 1.0   # multiplier on control MSE term
    state_weight: float = 1.0   # multiplier on state MSE term

    # ── Optimiser ─────────────────────────────────────────────────────────────
    lr:           float = 1e-3
    weight_decay: float = 1e-4
    # Learning-rate scheduler: reduce on plateau (factor=0.5, patience=10 epochs)
    lr_patience:  int   = 10
    lr_factor:    float = 0.5

    # ── Training loop ─────────────────────────────────────────────────────────
    batch_size: int = 256
    epochs:     int = 100
    seed:       int = 42

    # ── I/O ───────────────────────────────────────────────────────────────────
    checkpoint_dir: str = os.path.join(_PROJECT_ROOT, "checkpoints", "run0")
    # Set to True to resume from checkpoint_dir/latest.pt if it exists.
    resume: bool = False
