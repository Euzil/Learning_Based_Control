"""Runtime container for one auxiliary teacher attached to a student via per-slot KL distillation.

See ``ics_rl.algorithms.multi_teacher_distill.MultiTeacherDistillMixin`` for how slots are
consumed. Generalizes ``TPPO``'s single-teacher setup to N teachers, each supervising its own
action-dimension slice and gated by its own per-env routing signal (HANDOFF-style
context-conditioned multi-teacher KL distillation, https://arxiv.org/abs/2606.06493).
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from typing import Callable, Optional

import torch
import torch.nn as nn


def resolve_callable(path: str):
    """Resolve a ``"package.module:attr"`` string into the live callable/class it names.

    Mirrors the ``":"``-split convention already used elsewhere in this codebase (e.g.
    ``ics_rl.modules.build_actor_critic``, ``AMPAlgoMixin``'s ``discriminator_class_name``).
    """
    module_name, attr_name = path.split(":")
    module = importlib.import_module(module_name)
    return getattr(module, attr_name)


@dataclass
class TeacherSlot:
    """One frozen teacher supervising (a slice of) the student's action via KL.

    Args:
        name: Unique slot name. Used to key the ``distill_<name>`` loss and (if
            ``expert_idx`` is set) the MoE routing-shaping loss.
        actor_critic: The loaded, frozen teacher network. Set to ``eval()`` with gradients
            disabled in ``__post_init__``.
        obs_components: Names of the *student's* actor obs components to feed into this
            teacher (must be a subset of the student's ``obs_segments`` — see
            ``get_subobs_by_components``). The teacher only ever sees the student's own
            observation stream, sliced down to the components it was itself trained on.
        dim_slice: Which action dimensions this teacher supervises. Defaults to the full
            action vector — use a narrower slice only if the student intentionally wants to
            discard part of a teacher's trained behavior.
        kl_coef_init / kl_coef_min: Endpoints of the cosine-annealed KL coefficient (see
            ``MultiTeacherDistillMixin.teacher_slot_anneal_steps``).
        routing_fn: ``(minibatch, obs_segments, critic_obs_segments) -> Tensor[B]``, a per-env
            weight in ``[0, 1]`` for how much this slot's KL should count on each env this
            step. ``None`` means "always fully active" (a lone/always-on teacher).
        expert_idx / pin_routing_coef: If this slot should additionally pin one MoE expert
            (its ``routing_fn`` output supervises ``gate[:, expert_idx]`` directly), set both.
    """

    name: str
    actor_critic: nn.Module
    obs_components: list[str]
    dim_slice: slice = field(default_factory=lambda: slice(None, None))
    kl_coef_init: float = 0.0
    kl_coef_min: float = 0.0
    routing_fn: Optional[Callable[..., torch.Tensor]] = None
    expert_idx: Optional[int] = None
    pin_routing_coef: float = 0.0

    def __post_init__(self):
        self.actor_critic.eval()
        for p in self.actor_critic.parameters():
            p.requires_grad_(False)
