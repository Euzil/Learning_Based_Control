from typing import Sequence

import numpy as np
import torch
import torch.nn as nn

from ics_rl.networks.utils import get_activation


def _build_temporal_conv(nonlinearity: str, tsteps: int, channel_size: int) -> nn.Module:
    """Small Conv1d stack over the time axis. Ported from HANDOFF's TemporalConvEncoder
    (wbc_mjlab/rl/encoders.py) -- every supported `tsteps` value collapses to a length-3
    sequence after the conv stack, so the output head size (`channel_size * 3`) stays fixed.
    """
    if tsteps == 1:
        return nn.Flatten()
    if tsteps in (10, 11):
        return nn.Sequential(
            nn.Conv1d(3 * channel_size, 2 * channel_size, kernel_size=4, stride=2),
            get_activation(nonlinearity),
            nn.Conv1d(2 * channel_size, channel_size, kernel_size=2, stride=1),
            get_activation(nonlinearity),
            nn.Flatten(),
        )
    if tsteps == 20:
        return nn.Sequential(
            nn.Conv1d(3 * channel_size, 2 * channel_size, kernel_size=6, stride=2),
            get_activation(nonlinearity),
            nn.Conv1d(2 * channel_size, channel_size, kernel_size=4, stride=2),
            get_activation(nonlinearity),
            nn.Flatten(),
        )
    if tsteps == 50:
        return nn.Sequential(
            nn.Conv1d(3 * channel_size, 2 * channel_size, kernel_size=8, stride=4),
            get_activation(nonlinearity),
            nn.Conv1d(2 * channel_size, channel_size, kernel_size=5, stride=1),
            get_activation(nonlinearity),
            nn.Conv1d(channel_size, channel_size, kernel_size=5, stride=1),
            get_activation(nonlinearity),
            nn.Flatten(),
        )
    raise ValueError(f"Unsupported Conv1dHeadModel tsteps: {tsteps}. Supported: 1, 10, 11, 20, 50.")


class Conv1dHeadModel(nn.Module):
    """1D-conv temporal encoder over a short window of stacked observation frames.

    Compresses a `[B, T, D]` window (e.g. an observation history or a future-reference lookahead
    produced via `ObservationTermCfg(history_length=T, flatten_history_dim=False)`) down to a
    single `output_size` latent: a per-frame linear projection, a small Conv1d stack over the time
    axis, then a linear head. Cheaper than `TransformerHeadModel` for short windows.

    Follows `TransformerHeadModel`'s convention of accepting multiple same-length-T components
    (`input_shapes`), so `ParallelLayer` can feed it several history/future obs terms at once.
    """

    def __init__(
        self,
        input_shapes: Sequence[torch.Size],
        output_size: int,
        nonlinearity: str = "elu",
        channel_size: int = 20,
    ):
        super().__init__()
        # NOTE: assumes all input shapes are (T, *feature_dims), sharing the same T (time) dim.
        self.tsteps = int(input_shapes[0][0])
        self.feature_size = int(sum(np.prod(s[1:]) for s in input_shapes))
        self._output_size = output_size

        self.proj = nn.Sequential(nn.Linear(self.feature_size, 3 * channel_size), get_activation(nonlinearity))
        self.temporal = _build_temporal_conv(nonlinearity, self.tsteps, channel_size)
        self.out = nn.Linear(channel_size * 3, output_size)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """x: [B, T, feature_size] (already sliced/reshaped/concatenated by the caller)."""
        projected = self.proj(x)  # [B, T, 3*channel_size]
        latent = self.temporal(projected.transpose(1, 2))  # [B, 3*channel_size, T] -> conv -> [B, 3*channel_size]
        return self.out(latent)

    @property
    def output_size(self):
        return self._output_size
