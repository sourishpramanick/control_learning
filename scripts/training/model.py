"""Neural network architecture for OCP trajectory imitation.

Architecture
------------
Shared MLP backbone → two output heads:

    Input (27,)
        │
    [Linear → ReLU → Dropout] × len(hidden_dims)   ← shared backbone
        │
        ├─── ctrl_head  → (198,)   controls  [v₀,ω₀, …, v₉₈,ω₉₈]
        └─── state_head → (300,)   states    [x₀,y₀,θ₀, …, x₉₉,y₉₉,θ₉₉]

Why two heads?
--------------
The 498 output values are not interchangeable:

  • Controls (198-D): bounded physical quantities in m/s and rad/s.
  • States   (300-D): Cartesian positions (m) and headings (rad).
    Headings are *periodic* — the same direction can be represented by
    infinitely many real values when θ is unbounded.

Using a single flat head and treating all 498 outputs identically would
mean a 1-unit error in x position and a 1-unit error in angular velocity
are penalised equally, which is physically meaningless.

Two heads give us:
  1. Independent loss weight tuning for controls vs states.
  2. The θ columns inside the state head are at fixed indices
     [2, 5, 8, …, 299], making it trivial to apply a wrapped-angle loss
     in the future without recomputing offsets.
  3. If the model is used later as an MPC warm-start calculator, only the
     control head output is needed — the state head can be skipped.
"""

from __future__ import annotations

import torch
import torch.nn as nn


class OcpMlp(nn.Module):
    """MLP with a shared backbone and two specialised output heads.

    Parameters
    ----------
    hidden_dims:
        List of hidden layer widths for the shared backbone,
        e.g. [256, 512, 512, 256].
    dropout:
        Dropout probability applied after each hidden layer's ReLU.
        Set to 0.0 to disable.
    input_dim:
        Number of input features. Must match dataset INPUT_SIZE (default 27).
    n_controls:
        Number of control outputs. Default 198 = 2 * (N_INTERVALS - 1).
    n_states:
        Number of state outputs.   Default 300 = 3 *  N_INTERVALS.
    """

    # θ column indices within the 300-D state vector (positions 2, 5, 8, …)
    THETA_INDICES_IN_STATE: list[int] = list(range(2, 300, 3))

    def __init__(
        self,
        hidden_dims: list[int] = [256, 512, 512, 256],
        dropout: float = 0.0,
        input_dim: int = 27,
        n_controls: int = 198,
        n_states: int = 300,
    ) -> None:
        super().__init__()

        # ── Shared backbone ───────────────────────────────────────────────────
        # Stack of Linear → ReLU (→ Dropout) blocks.
        # Input layer maps input_dim → hidden_dims[0].
        # Output of backbone: hidden_dims[-1]-dimensional vector.
        self.backbone: nn.Sequential
        raise NotImplementedError

        # ── Output heads ──────────────────────────────────────────────────────
        # Each head is a single Linear layer (no activation) mapping from
        # the backbone output width to the respective output dimension.
        # No activation on the output: the network should be free to predict
        # any real value; bounding happens implicitly through normalisation.
        self.ctrl_head:  nn.Linear   # backbone_out → n_controls
        self.state_head: nn.Linear   # backbone_out → n_states
        raise NotImplementedError

    def forward(
        self, x: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Forward pass.

        Args:
            x: (batch, input_dim) normalised input tensor.

        Returns:
            controls: (batch, n_controls) predicted control trajectory.
            states:   (batch, n_states)   predicted state trajectory.
        """
        raise NotImplementedError

    def predict(self, x: torch.Tensor) -> torch.Tensor:
        """Return the full 498-D concatenated output [controls | states].

        Convenience wrapper used during evaluation to compare directly
        against the raw 498-D output vectors stored in the dataset.

        Args:
            x: (batch, input_dim) normalised input tensor.

        Returns:
            y: (batch, n_controls + n_states) concatenated output.
        """
        raise NotImplementedError

    @property
    def n_parameters(self) -> int:
        """Total number of trainable parameters."""
        return sum(p.numel() for p in self.parameters() if p.requires_grad)
