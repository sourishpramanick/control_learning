"""Loss functions for OCP trajectory imitation.

Current implementation
----------------------
Weighted sum of two MSE terms — one for the control block, one for the
state block:

    L = w_ctrl  × MSE(pred_ctrl,  true_ctrl)
      + w_state × MSE(pred_state, true_state)

where MSE is the mean over both the batch dimension and the output dimension.

Why not a single flat MSE over all 498 outputs?
-----------------------------------------------
The control block (198-D) and state block (300-D) have different physical
units and different scales. A flat MSE would implicitly weight them by the
ratio of their output sizes (198 vs 300), which is arbitrary.  Separate
terms with explicit weights give full control over this trade-off.

Future extension — wrapped-angle loss
--------------------------------------
The state block contains θ values at indices [2, 5, 8, …, 299]
(every 3rd element, stored as THETA_INDICES).  Standard MSE penalises
the heading error as a linear quantity, which is wrong for periodic angles:
a predicted heading of +179° and a true heading of −179° differ by only 2°
geometrically, but MSE sees a difference of 358°.

To fix this, replace the MSE on θ columns with:

    Δθ_wrapped = atan2(sin(θ_pred − θ_true), cos(θ_pred − θ_true))
    loss_theta = mean(Δθ_wrapped²)

This is the same geodesic error used in the OCP cost function (Ocp.cpp).
The scaffolding for this extension is already in place: THETA_INDICES marks
exactly which elements need wrapping, and the forward() signature returns
the per-term losses separately so the theta term can be split out.
"""

from __future__ import annotations

import torch
import torch.nn as nn


class OcpLoss(nn.Module):
    """Weighted MSE loss with separate control and state terms.

    Parameters
    ----------
    ctrl_weight:
        Scalar multiplier on the control MSE term (default 1.0).
    state_weight:
        Scalar multiplier on the state MSE term (default 1.0).
    """

    # Indices of θ within the 300-D state vector.
    # Used to identify heading outputs for the future wrapped-angle extension.
    THETA_INDICES: list[int] = list(range(2, 300, 3))  # [2, 5, 8, …, 299]

    def __init__(
        self,
        ctrl_weight:  float = 1.0,
        state_weight: float = 1.0,
    ) -> None:
        super().__init__()
        # Store weights as buffers so they move with the module to GPU if needed
        self.register_buffer("ctrl_weight",  torch.tensor(ctrl_weight))
        self.register_buffer("state_weight", torch.tensor(state_weight))

    def forward(
        self,
        pred_ctrl:  torch.Tensor,
        pred_state: torch.Tensor,
        true_ctrl:  torch.Tensor,
        true_state: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Compute weighted loss.

        Args:
            pred_ctrl:  (batch, 198) predicted controls   (normalised space).
            pred_state: (batch, 300) predicted states     (normalised space).
            true_ctrl:  (batch, 198) ground-truth controls (normalised space).
            true_state: (batch, 300) ground-truth states  (normalised space).

        Returns:
            total:      Scalar weighted total loss — pass this to .backward().
            loss_ctrl:  Scalar control MSE (detached, for logging only).
            loss_state: Scalar state MSE   (detached, for logging only).

        Note: all tensors are expected in *normalised* space. The Normalizer
        in dataset.py scales outputs to zero-mean unit-variance before
        training, so the MSE values are dimensionless and comparable across
        the two output blocks.
        """
        raise NotImplementedError
