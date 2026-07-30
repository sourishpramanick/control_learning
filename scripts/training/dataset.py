"""Dataset loading and normalisation for OCP imitation training.

Classes
-------
Normalizer
    Fits z-score (mean / std) statistics on training data and applies them.
    Serialises to JSON so the same statistics can be used at inference time.

OcpDataset
    PyTorch Dataset wrapping a .npz file produced by the dataset generator.
    Returns each sample as a dict with keys:
        'input'    : float32 tensor (27,)
        'controls' : float32 tensor (198,)  — first block of the output
        'states'   : float32 tensor (300,)  — second block of the output

    The output is pre-split into controls and states so that OcpLoss can
    weight the two blocks independently without index arithmetic at loss time.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import numpy as np
import torch
from torch.utils.data import Dataset


# ── Normalizer ────────────────────────────────────────────────────────────────

class Normalizer:
    """Z-score normaliser for inputs and outputs.

    Fit on the training split; then apply identically to val and test splits
    to avoid data leakage.

    Note on theta outputs
    ---------------------
    Heading angles are periodic — their mean and std are not geometrically
    meaningful in the usual sense. For now, standard z-score is applied to all
    498 output dimensions. A future improvement would use circular statistics
    (mean direction + circular std) for the theta elements, or simply skip
    normalising the theta outputs and rely on the wrapped-angle loss instead.
    """

    def __init__(self) -> None:
        self.input_mean:  Optional[np.ndarray] = None
        self.input_std:   Optional[np.ndarray] = None
        self.output_mean: Optional[np.ndarray] = None
        self.output_std:  Optional[np.ndarray] = None

    def fit(self, X: np.ndarray, Y: np.ndarray) -> None:
        """Compute per-feature mean and std from training arrays.

        Args:
            X: (N, 27)  raw input array.
            Y: (N, 498) raw output array.
        """
        raise NotImplementedError

    def transform_input(self, X: np.ndarray) -> np.ndarray:
        """Apply z-score to input array. Shape preserved."""
        raise NotImplementedError

    def transform_output(self, Y: np.ndarray) -> np.ndarray:
        """Apply z-score to output array. Shape preserved."""
        raise NotImplementedError

    def inverse_transform_output(self, Y: np.ndarray) -> np.ndarray:
        """Reverse z-score on output array (back to physical units)."""
        raise NotImplementedError

    def save(self, path: str) -> None:
        """Serialise statistics to a JSON file at `path`."""
        raise NotImplementedError

    @classmethod
    def load(cls, path: str) -> "Normalizer":
        """Deserialise statistics from a JSON file and return a fitted Normalizer."""
        raise NotImplementedError


# ── OcpDataset ────────────────────────────────────────────────────────────────

class OcpDataset(Dataset):
    """PyTorch Dataset for OCP trajectory imitation.

    Each item is a dict:
        'input'    : float32 tensor (input_dim,)  — problem description
        'controls' : float32 tensor (n_controls,) — v/ω trajectory
        'states'   : float32 tensor (n_states,)   — x/y/θ trajectory

    Parameters
    ----------
    npz_path:
        Path to the .npz file produced by generator.py.
    normalizer:
        A *fitted* Normalizer instance. If None, raw (unnormalised) values
        are returned — useful during the fitting step itself.
    indices:
        Optional integer array selecting a subset of rows (for train/val/test
        splits). If None, all rows are used.
    """

    def __init__(
        self,
        npz_path: str,
        normalizer: Optional[Normalizer] = None,
        indices: Optional[np.ndarray] = None,
    ) -> None:
        # Load arrays from .npz; store only the requested rows.
        # Read n_controls / n_states from sidecar _meta.json so this class
        # adapts automatically if N_INTERVALS ever changes.
        raise NotImplementedError

    def __len__(self) -> int:
        raise NotImplementedError

    def __getitem__(self, idx: int) -> dict[str, torch.Tensor]:
        """Return a single sample as a dict of float32 tensors."""
        raise NotImplementedError

    @staticmethod
    def split(
        npz_path: str,
        train_frac: float = 0.8,
        val_frac: float = 0.1,
        seed: int = 42,
    ) -> tuple["OcpDataset", "OcpDataset", "OcpDataset", Normalizer]:
        """Randomly split dataset into train / val / test and fit normaliser.

        The Normalizer is fitted *only* on the training split to prevent data
        leakage into validation and test sets.

        Returns
        -------
        train_dataset, val_dataset, test_dataset, normalizer
        """
        raise NotImplementedError
