"""Multi-teacher context-gated KL distillation into a (possibly MoE) student.

Generalizes ``TPPO``'s single-teacher action-matching distillation to N frozen teachers, each
supervising its own action-dimension slice and gated by its own per-env routing signal --
HANDOFF-style "multi-teacher KL distillation under a context-conditioned gating scheme"
(https://arxiv.org/abs/2606.06493). Unlike ``TPPO``, this never substitutes a teacher's action
into the rollout (no beta-mixing / DAgger action labels) -- it is a pure auxiliary loss term
added on top of on-policy PPO, evaluating each frozen teacher fresh on the student's own
minibatch obs every update.

Each teacher is loaded from its own experiment checkpoint directory (mirroring
``TPPO.load_teacher_policy``, generalized to a list) and fed a *subset* of the student's own
observation components -- no separate rollout storage is needed as long as every teacher's
required components are already present among the student's obs (see ``obs_components``).

Composes the same way ``AMPAlgoMixin``/``EstimatorAlgoMixin`` do::

    class MultiTeacherMoEPPO(MultiTeacherDistillMixin, PPO):
        pass
"""

from __future__ import annotations

import math
import os
import os.path as osp
from collections import OrderedDict

import torch

import ics_rl.modules as modules
from ics_rl.algorithms.ppo import PPO
from ics_rl.modules.teacher_slot import TeacherSlot, resolve_callable
from ics_rl.networks.moe import MoeLayer, subset_aware_load_balancing_loss
from ics_rl.utils.utils import get_subobs_by_components


def _gaussian_kl_per_dim(
    mu_s: torch.Tensor,
    sigma_s: torch.Tensor,
    mu_t: torch.Tensor,
    sigma_t: torch.Tensor,
) -> torch.Tensor:
    """KL(student || teacher) per action dimension, for diagonal Gaussians."""
    return torch.log(sigma_t / sigma_s) + (sigma_s.pow(2) + (mu_s - mu_t).pow(2)) / (2 * sigma_t.pow(2)) - 0.5


def _load_teacher_checkpoint(actor_critic, teacher_logdir: str, device: str) -> None:
    """Load the latest ``model_<iter>.pt`` checkpoint in ``teacher_logdir`` into ``actor_critic``.

    Mirrors ``TPPO.load_teacher_policy`` (single-teacher case), generalized so each teacher slot
    can point at its own experiment's checkpoint directory. Loads only the ``actor``/``std``
    weights, never ``critic`` -- ``compute_losses`` only ever calls ``update_distribution`` /
    reads ``action_mean``/``action_std`` on a teacher, so the critic is dead weight here. Its obs
    shape also commonly differs from the actor's own (privileged, unstacked terms) and would
    otherwise force getting an obs_format right for something that's never used.
    """
    model_files = [f for f in os.listdir(teacher_logdir) if f.endswith(".pt") and f.startswith("model_")]
    if not model_files:
        raise FileNotFoundError(f"No model_*.pt checkpoint found in {teacher_logdir}")
    model_files.sort(key=lambda f: int(f.split(".")[0].split("_")[1]))
    full_state = torch.load(osp.join(teacher_logdir, model_files[-1]), map_location="cpu")["model_state_dict"]

    actor_state = {k: v for k, v in full_state.items() if k == "std" or k.startswith("actor.")}
    result = actor_critic.load_state_dict(actor_state, strict=False)
    bad_missing = [k for k in result.missing_keys if not k.startswith("critic")]
    if bad_missing or result.unexpected_keys:
        raise RuntimeError(
            f"Teacher checkpoint at {teacher_logdir} doesn't match the reconstructed actor "
            f"architecture (check obs_components / obs_format_override / policy_cfg). "
            f"missing={bad_missing}, unexpected={result.unexpected_keys}"
        )

    actor_critic.to(device)
    actor_critic.eval()


class MultiTeacherDistillMixin:
    """Mixin adding N-teacher context-gated KL distillation to a PPO-family algorithm.

    Args (passed as ``**kwargs`` from the algorithm cfg, alongside the base algorithm's own):
        teacher_slots: list of dicts, one per teacher. Each dict's keys:
            - ``name`` (str): unique slot name.
            - ``teacher_logdir`` (str): experiment run dir containing ``model_*.pt`` checkpoints.
            - ``obs_components`` (list[str]): names of the *student's* actor obs components to
              feed this teacher -- must be a subset of the student's ``obs_segments``.
            - ``policy_cfg`` (dict): kwargs to reconstruct the teacher's network (hidden dims,
              activation, ...) -- must match what it was trained with, for the checkpoint to load.
            - ``policy_class_name`` (str, default ``"ActorCritic"``).
            - ``num_actions`` / ``num_rewards`` (int, default: student's own num_actions / 1) --
              override only if this teacher's action space differs from the student's.
            - ``dim_slice`` (slice | (start, stop)): which student action dims this teacher
              supervises. Defaults to the full action vector.
            - ``kl_coef_init`` / ``kl_coef_min`` (float): cosine-annealed KL coefficient.
            - ``routing_fn`` (callable | ``"module:func"`` str, optional): ``(minibatch,
              obs_segments, critic_obs_segments) -> Tensor[B]`` in ``[0, 1]``. ``None`` = always
              fully active.
            - ``expert_idx`` / ``pin_routing_coef`` (optional): pin an MoE expert to this slot's
              routing signal.
        teacher_slot_anneal_steps: cosine-anneal every slot's KL coef over this many updates.
        load_balance_coef: coefficient for the (unpinned-expert) MoE load-balancing loss.
        pin_routing_coef: coefficient for the summed pinned-expert routing loss.

    Each teacher's coefficient (``distill_<name>_coef``) is exposed dynamically via
    ``__getattr__`` and cosine-annealed from ``kl_coef_init`` to ``kl_coef_min`` -- this lets
    ``PPO.update()``'s existing generic ``loss += getattr(self, k + "_coef", 1.0) * v`` loop apply
    the (time-varying) per-teacher weight without this mixin having to hand-sum losses itself.
    """

    def __init__(
        self,
        *args,
        teacher_slots: list = [],
        teacher_slot_anneal_steps: int = 0,
        load_balance_coef: float = 0.0,
        pin_routing_coef: float = 0.0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.teacher_slot_anneal_steps = int(teacher_slot_anneal_steps)
        self.load_balance_coef = float(load_balance_coef)
        self.pin_routing_coef = float(pin_routing_coef)

        student_obs_segments = self.actor_critic.obs_segments
        student_num_actions = self.actor_critic.std.shape[0]

        self.teacher_slots: list[TeacherSlot] = []
        for raw_cfg in teacher_slots:
            slot_cfg = dict(raw_cfg)
            name = slot_cfg.pop("name")
            teacher_logdir = slot_cfg.pop("teacher_logdir")
            policy_class_name = slot_cfg.pop("policy_class_name", "ActorCritic")
            policy_cfg = dict(slot_cfg.pop("policy_cfg", {}))
            obs_components = slot_cfg.pop("obs_components")
            # ``.pop(key, default)`` only falls back to ``default`` when the key is *missing* --
            # cfg dataclasses (e.g. IcsRlTeacherSlotCfg) commonly default these to an
            # explicit ``None`` rather than omitting the key, so an ``or`` fallback is required
            # here, not just a pop-default (a bare pop-default silently passed `None` through to
            # ``nn.Linear(hidden, None)``).
            num_actions = slot_cfg.pop("num_actions", None) or student_num_actions
            num_rewards = slot_cfg.pop("num_rewards", None) or 1
            # Per-component shape override, e.g. {"base_ang_vel": (24,), ...}. Needed whenever a
            # teacher's checkpoint predates a change to the *student's* live env (e.g. an
            # observation term's history_length got bumped after that checkpoint was trained) --
            # auto-deriving from the student's current obs_segments would silently reconstruct
            # the wrong-shaped actor and either fail to load or (worse, if some other component
            # happened to match) load into the wrong-shaped layer. Read once from the checkpoint's
            # own logdir/params/env.yaml (observations.policy.<term>.history_length) when in doubt.
            obs_format_override = slot_cfg.pop("obs_format_override", None) or {}

            teacher_obs_format = {
                "policy": OrderedDict(
                    (k, obs_format_override.get(k, student_obs_segments[k])) for k in obs_components
                )
            }
            teacher_actor_critic = modules.build_actor_critic(
                policy_class_name, policy_cfg, teacher_obs_format, num_actions, num_rewards
            )
            _load_teacher_checkpoint(teacher_actor_critic, teacher_logdir, self.device)

            routing_fn = slot_cfg.pop("routing_fn", None)
            if isinstance(routing_fn, str):
                routing_fn = resolve_callable(routing_fn)
            dim_slice = slot_cfg.pop("dim_slice", None)
            dim_slice = slice(*dim_slice) if isinstance(dim_slice, (list, tuple)) else slice(None, None)

            self.teacher_slots.append(
                TeacherSlot(
                    name=name,
                    actor_critic=teacher_actor_critic,
                    obs_components=obs_components,
                    dim_slice=dim_slice,
                    routing_fn=routing_fn,
                    **slot_cfg,
                )
            )

    def __getattr__(self, name: str):
        # Only reached when normal attribute lookup fails. Dynamically exposes
        # ``distill_<slot.name>_coef`` so PPO.update()'s generic per-key coefficient loop can
        # pick up the (cosine-annealed, time-varying) per-teacher weight without this mixin
        # having to apply it manually.
        if name.startswith("distill_") and name.endswith("_coef"):
            teacher_slots = self.__dict__.get("teacher_slots")
            if teacher_slots is not None:
                slot_name = name[len("distill_") : -len("_coef")]
                for slot in teacher_slots:
                    if slot.name == slot_name:
                        return self._slot_kl_coef(slot)
        raise AttributeError(name)

    def _slot_kl_coef(self, slot: TeacherSlot) -> float:
        if self.teacher_slot_anneal_steps <= 0:
            return slot.kl_coef_init
        progress = min(1.0, self.current_learning_iteration / self.teacher_slot_anneal_steps)
        cosine = 0.5 * (1 + math.cos(math.pi * progress))
        return slot.kl_coef_min + (slot.kl_coef_init - slot.kl_coef_min) * cosine

    def compute_losses(self, minibatch):
        losses, inter_vars, stats = super().compute_losses(minibatch)

        mu_s = self.actor_critic.action_mean
        sigma_s = self.actor_critic.action_std
        obs_segments = self.actor_critic.obs_segments
        critic_obs_segments = self.actor_critic.critic_obs_segments

        pinned_idxs: list[int] = []
        pinned_routing: dict[int, torch.Tensor] = {}

        for slot in self.teacher_slots:
            coef = self._slot_kl_coef(slot)
            stats[f"{slot.name}_kl_coef"] = torch.tensor(coef, device=self.device)
            if coef <= 0.0:
                continue

            teacher_obs = get_subobs_by_components(minibatch.obs, slot.obs_components, obs_segments)
            with torch.no_grad():
                slot.actor_critic.update_distribution(teacher_obs)
                mu_t = slot.actor_critic.action_mean
                sigma_t = slot.actor_critic.action_std

            sl = slot.dim_slice
            kl = _gaussian_kl_per_dim(mu_s[:, sl], sigma_s[:, sl], mu_t[:, sl], sigma_t[:, sl])

            weight = None
            if slot.routing_fn is not None:
                weight = slot.routing_fn(minibatch, obs_segments, critic_obs_segments).reshape(-1, 1).clamp(0.0, 1.0)
                denom = weight.sum().clamp_min(1.0) * kl.shape[-1]
                slot_loss = (kl * weight).sum() / denom
            else:
                slot_loss = kl.mean()

            losses[f"distill_{slot.name}"] = slot_loss
            stats[f"{slot.name}_kl_raw"] = kl.detach().mean()

            if slot.expert_idx is not None and slot.pin_routing_coef > 0.0 and weight is not None:
                pinned_idxs.append(slot.expert_idx)
                pinned_routing[slot.expert_idx] = weight.detach()

        # MoE routing-shaping losses -- only if the actor is a MoeLayer.
        moe = self.actor_critic.actor
        if isinstance(moe, MoeLayer) and (self.load_balance_coef > 0.0 or pinned_idxs):
            gate_weights = moe.gate_weights(minibatch.obs)
            lb_unpinned, lb_pinned = subset_aware_load_balancing_loss(gate_weights, pinned_idxs, pinned_routing)
            if self.load_balance_coef > 0.0:
                losses["load_balance"] = lb_unpinned
            if lb_pinned:
                losses["pin_routing"] = sum(lb_pinned.values())
            stats["gate_entropy"] = -(gate_weights * (gate_weights + 1e-8).log()).sum(dim=-1).mean().detach()
            for i in range(gate_weights.shape[-1]):
                stats[f"gate_weight_e{i}"] = gate_weights[:, i].mean().detach()

        return losses, inter_vars, stats


class MultiTeacherMoEPPO(MultiTeacherDistillMixin, PPO):
    pass
