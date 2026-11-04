from typing import Literal

import torch
import torch.nn as nn
import torch.nn.functional as F

from .utils import get_activation


class MoeLayer(nn.Module):
    def __init__(
        self,
        input_dim,
        num_experts,
        output_dim=None,
        activation="elu",
        expert_hidden_dims=[],
        gate_hidden_dims=[],
    ):
        super().__init__()
        self.act_fn = get_activation(activation)
        self.gate = self._build_gate(input_dim, num_experts, gate_hidden_dims)
        self.experts = nn.ModuleList(
            [self._build_expert(input_dim, output_dim, expert_hidden_dims) for _ in range(num_experts)]
        )

    def _build_gate(self, input_dim, num_experts, hidden_dims):
        layers = []
        curr_dim = input_dim
        for h in hidden_dims:
            layers.append(nn.Linear(curr_dim, h))
            layers.append(self.act_fn)
            curr_dim = h
        layers.append(nn.Linear(curr_dim, num_experts))
        return nn.Sequential(*layers)

    def _build_expert(self, input_dim, output_dim, hidden_dims):
        layers = []
        curr_dim = input_dim
        for h in hidden_dims:
            layers.append(nn.Linear(curr_dim, h))
            layers.append(self.act_fn)
            curr_dim = h
        if output_dim is not None:
            layers.append(nn.Linear(curr_dim, output_dim))  # no activation for the last layer
        return nn.Sequential(*layers)

    def forward(self, x):
        gate_scores = F.softmax(self.gate(x), dim=-1)  # [batch, num_experts] # gate the expert outputs
        expert_outputs = [expert(x) for expert in self.experts]
        expert_outputs = torch.stack(expert_outputs, dim=1)  # [batch, num_experts, output_dim]
        output = torch.einsum("be,beo->bo", gate_scores, expert_outputs)  # mix the expert outputs
        return output

    def gate_weights(self, x: torch.Tensor) -> torch.Tensor:
        """Return the ``[batch, num_experts]`` softmax routing weights without running the experts."""
        return F.softmax(self.gate(x), dim=-1)


def load_balancing_loss(gate_weights: torch.Tensor) -> torch.Tensor:
    """Switch-Transformer-style load-balancing auxiliary loss (Fedus et al., 2021).

    ``N * sum_i(fraction_i * mean_weight_i)``, where ``fraction_i`` is the share of the batch
    routed (by argmax) to expert ``i`` and ``mean_weight_i`` is its average soft gate weight.
    1.0 at perfect balance, ``N`` (num_experts) at total collapse onto one expert.
    """
    num_experts = gate_weights.shape[-1]
    expert_assignments = gate_weights.argmax(dim=-1)
    fractions = torch.stack([(expert_assignments == i).float().mean() for i in range(num_experts)])
    mean_weights = gate_weights.mean(dim=0)
    return num_experts * (fractions * mean_weights).sum()


def subset_aware_load_balancing_loss(
    gate_weights: torch.Tensor,
    pinned_idxs: list,
    pinned_routing: "dict | None" = None,
):
    """Generalization of :func:`load_balancing_loss` to N experts each pinned to a routing signal.

    For each pinned expert ``i`` with a ``[B]`` (or ``[B, 1]``) routing weight in ``[0, 1]``:
      - ``lb_pinned[i] = (1 - gate[:, i])[routing_i > 0.5].mean()`` -- direct supervision pushing
        ``gate[:, i]`` toward 1 on samples where that slot's routing signal is active.
    For the remaining unpinned experts:
      - on samples where *every* pinned routing is inactive, renormalize the gate over the
        unpinned experts and apply the standard uniform-target balance, so this term doesn't
        fight the skew a pinned regime naturally creates.

    Returns ``(lb_unpinned, lb_pinned)`` where ``lb_pinned`` is keyed by expert index. Falls back
    to plain :func:`load_balancing_loss` (as ``lb_unpinned``, with ``lb_pinned={}``) if
    ``pinned_idxs`` is empty.
    """
    num_experts = gate_weights.shape[-1]
    device = gate_weights.device
    zero = torch.zeros((), device=device)

    if not pinned_idxs:
        return load_balancing_loss(gate_weights), {}

    pinned_set = {idx % num_experts for idx in pinned_idxs}
    unpinned_idx = [i for i in range(num_experts) if i not in pinned_set]
    pinned_routing = pinned_routing or {}

    walk_mask = torch.ones(gate_weights.shape[0], dtype=torch.bool, device=device)
    for idx in pinned_set:
        r = pinned_routing.get(idx)
        if r is None:
            continue
        walk_mask = walk_mask & (r.flatten().float() <= 0.5)

    if walk_mask.any() and unpinned_idx:
        gate_nr = gate_weights[walk_mask][:, unpinned_idx]
        gate_nr = gate_nr / gate_nr.sum(dim=-1, keepdim=True).clamp_min(1e-8)
        lb_unpinned = load_balancing_loss(gate_nr)
    else:
        lb_unpinned = zero

    lb_pinned = {}
    for idx in pinned_set:
        r = pinned_routing.get(idx)
        if r is None:
            lb_pinned[idx] = zero
            continue
        active_mask = r.flatten().float() > 0.5
        if active_mask.any():
            lb_pinned[idx] = (1.0 - gate_weights[active_mask][:, idx]).mean()
        else:
            lb_pinned[idx] = zero

    return lb_unpinned, lb_pinned
