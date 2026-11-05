from __future__ import annotations

import importlib
from typing import Dict

import torch
import torch.nn as nn

import ics_rl.modules as ics_modules
from ics_rl.networks.mlp import MlpModel
from ics_rl.networks.normalizer import (
    EmpiricalDiscountedVariationNormalization,
    EmpiricalNormalization,
)
from ics_rl.utils.utils import get_subobs_size


class Discriminator(nn.Module):
    """A general implementation of returning +/- value to discriminate the input sequence."""

    is_recurrent = False

    def __init__(
        self,
        input_segment: dict[str, tuple],
        output_size=1,
        output_nonlinearity: str = None,
        use_minibatch_std: bool = False,
        normalizer_class_name: str = None,  # if provided, the normalizer will be used to normalize the input
        normalizer_kwargs: dict = None,
        encoder_class_name: str = None,
        encoder_configs: dict[str, dict] = None,
        **kwargs,
    ):
        """
        Args:
            output_size: (int) the output size of the model.
            output_nonlinearity: (str) the nonlinearity of the output layer.
            use_minibatch_std: (bool) if True, concatenate a scalar batch-statistics feature
                (mean feature-wise std across the batch) onto the trunk output before the final
                linear layer. Standard GAN trick to discourage discriminator mode collapse.
            normalizer_class_name: (str) the class name of the normalizer. If None, no normalizer will be used.
            encoder_configs: Dict[str, dict]: encoder configs for the input segment, requiring the following keys:
                - class_name: (str) the class name of the ParallelLayer or the customed encoder class.
                - {name}: {dict} encoder configs to build each specific encoder. The required keys of this dict are referred to
                    `ics_rl.networks.parallel_layer.ParallelLayer.__init__`.
        """
        super().__init__()

        self.input_segment = input_segment
        self.output_size = output_size
        self.output_nonlinearity = output_nonlinearity
        self.use_minibatch_std = use_minibatch_std
        self.normalizer_class_name = normalizer_class_name
        self.normalizer_kwargs = normalizer_kwargs
        self.encoder_class_name = encoder_class_name
        self.encoder_configs = encoder_configs

        assert (
            encoder_configs is None or normalizer_class_name is None
        ), "The encoder_configs and normalizer_class_name cannot be used at the same time (currently)."
        self.normalizer = ics_modules.build_normalizer(
            input_shape=(
                get_subobs_size(self.input_segment)
                if len(self.input_segment) > 1
                else list(self.input_segment.values())[0]
            ),  # in case of image-like non-flattened tensor
            normalizer_class_name=normalizer_class_name,
            normalizer_kwargs=normalizer_kwargs,
        )
        self.build_model(input_segment, kwargs)

        print(f"Discriminator Network Structure: {self}")

    def build_model(self, input_segment, kwargs):
        """Build the model as a trunk MLP (all hidden layers) followed by a linear head.
        Kept as two separate modules (rather than one opaque MLP) so the minibatch-std
        feature can be spliced in between them, and so the R1 gradient penalty can be
        computed with that feature detached (see ``forward``)."""
        if self.encoder_configs is not None:
            EncoderClass = (
                getattr(
                    importlib.import_module(self.encoder_class_name.split(":")[0]),
                    self.encoder_class_name.split(":")[1],
                )
                if ":" in self.encoder_class_name
                else getattr(ics_modules, self.encoder_class_name)
            )
            encoders = EncoderClass(
                input_segment=input_segment,
                block_configs=self.encoder_configs,
            )
            input_segment = encoders.output_segment
            self.encoders = encoders
        else:
            self.encoders = None

        trunk_input_size = get_subobs_size(input_segment)
        hidden_sizes = kwargs.get("hidden_sizes") or []
        if isinstance(hidden_sizes, int):
            hidden_sizes = [hidden_sizes]
        if len(hidden_sizes) == 0:
            self.trunk = nn.Identity()
            trunk_output_size = trunk_input_size
        else:
            self.trunk = MlpModel(
                input_size=trunk_input_size,
                hidden_sizes=hidden_sizes,
                output_size=None,
                nonlinearity=kwargs.get("nonlinearity", nn.ReLU),
            )
            trunk_output_size = self.trunk.output_size

        head_input_size = trunk_output_size + (1 if self.use_minibatch_std else 0)
        self.head = nn.Linear(head_input_size, self.output_size)

        self.output_activation = (
            getattr(nn, self.output_nonlinearity)() if self.output_nonlinearity is not None else None
        )

    def _minibatch_std_scalar(self, h: torch.Tensor) -> torch.Tensor:
        """Mean over feature-wise std across the batch; shape (B,1)."""
        if h.shape[0] <= 1:
            return h.new_zeros((h.shape[0], 1))
        s = h.float().std(dim=0, unbiased=False).mean()
        return s.expand(h.shape[0], 1).to(h.dtype)

    def forward(self, x, hidden_states=None, masks=None, detach_minibatch_std: bool = False):
        """NOTE: saving hidden_states and masks for future recurrent models. (Maybe not necessary)

        detach_minibatch_std: stop gradients from flowing through the batch-statistics feature.
            Used by the R1 gradient penalty, which measures the discriminator's sensitivity to a
            single sample and should not pick up a gradient contribution from the batch composition.
        """
        if self.encoders is not None:
            # The encoders will determine and update the input segment by itself.
            x = self.encoders(x)
        if self.normalizer is not None:
            # The normalizer will determine and update the mean and std of the input data by itself.
            x = self.normalizer(x)
        h = self.trunk(x)
        if self.use_minibatch_std:
            if detach_minibatch_std:
                with torch.no_grad():
                    s = self._minibatch_std_scalar(h)
            else:
                s = self._minibatch_std_scalar(h)
            h = torch.cat([h, s], dim=-1)
        out = self.head(h)
        if self.output_activation is not None:
            out = self.output_activation(out)
        return out

    def logit_layer_weights(self):
        """Get the weights of the last (head) layer of the discriminator."""
        head = self.head
        if isinstance(head, nn.parallel.DistributedDataParallel):
            head = head.module
        return head.weight
