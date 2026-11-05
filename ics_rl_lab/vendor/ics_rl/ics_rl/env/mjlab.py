# SPDX-License-Identifier: BSD-3-Clause
"""Adapter that lets ics_rl train directly on mjlab environments.

This is a first-class part of ics_rl (not an external wrapper package): mjlab's
``ManagerBasedRlEnv`` returns observation groups as either a single concatenated
tensor or a dict of per-term tensors, and rewards/dones shaped for a gym-style
vector-env contract. ``MjlabVecEnv`` packs that into the flat-tensor + extras-dict
contract ``ics_rl.env.VecEnv`` (and its ``OnPolicyRunner``) expects.

Requires the optional ``mjlab`` extra (``pip install ics_rl[mjlab]``); mjlab is not
imported by the rest of ics_rl, so the base package stays framework-agnostic.
"""

from __future__ import annotations

from typing import Any, Callable

import torch
from mjlab.envs import ManagerBasedRlEnv, ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionAction
from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand

from ics_rl.env import VecEnv


##
# METADATA for mjlab Actions and Commands
##

def _per_env0(value: torch.Tensor | float) -> list[float] | float:
    return value[0].cpu().tolist() if isinstance(value, torch.Tensor) else float(value)


def _joint_position_action_metadata(term: JointPositionAction) -> dict:
    if term.cfg.entity_name != "robot":
        return {}
    return {
        # Order of the policy output vector (may differ from joint_names).
        "action_joint_names": list(term.target_names),
        "action_scale": _per_env0(term.scale),
        "action_offset": _per_env0(term.offset),
    }


def _uniform_velocity_command_metadata(term: UniformVelocityCommand) -> dict:
    return {
        "cmd_range_vx": list(term.cfg.ranges.lin_vel_x),
        "cmd_range_vy": list(term.cfg.ranges.lin_vel_y),
        "cmd_range_omega": list(term.cfg.ranges.ang_vel_z),
    }


TERM_METADATA: dict[type, Callable[[Any], dict]] = {
    JointPositionAction: _joint_position_action_metadata,
    UniformVelocityCommand: _uniform_velocity_command_metadata,
}
"""Deployment-metadata extractors for command/action term classes that can't carry a
``get_policy_metadata()`` method themselves (mjlab's stock terms). Matched along the
term's MRO, so subclasses are covered. Add entries with ``register_term_metadata``."""


def register_term_metadata(term_cls: type, fn: Callable[[Any], dict]) -> None:
    TERM_METADATA[term_cls] = fn


def term_policy_metadata(term: Any) -> dict:
    """A term's own ``get_policy_metadata()`` wins, else the closest ``TERM_METADATA`` entry."""
    fn = getattr(term, "get_policy_metadata", None)
    if callable(fn):
        return fn()
    for cls in type(term).__mro__:
        if cls in TERM_METADATA:
            return TERM_METADATA[cls](term)
    return {}


##
# Mjlab env wrapper
##


class MjlabVecEnv(VecEnv):
    """Wrap ``ManagerBasedRlEnv`` (mjlab) to satisfy the ``ics_rl.env.VecEnv`` contract."""

    def __init__(
        self,
        env: ManagerBasedRlEnv,
        *,
        policy_group: str = "policy",
        critic_group: str | None = "critic",
    ):
        self.env = env
        self.policy_group = policy_group
        self.critic_group = critic_group

        self.num_envs = self.unwrapped.num_envs
        self.device = torch.device(self.unwrapped.device)
        self.max_episode_length = self.unwrapped.max_episode_length
        self.num_actions = self.unwrapped.action_manager.total_action_dim
        self._log_defaults: dict[str, torch.Tensor] = {}

        active_groups = set(self.unwrapped.observation_manager.active_terms.keys())
        if self.critic_group is not None and self.critic_group not in active_groups:
            self.critic_group = None

        self._group_map = {"policy": self.policy_group}
        if self.critic_group is not None:
            self._group_map["critic"] = self.critic_group

        self.num_rewards = int(getattr(self.unwrapped, "num_rewards", 1))

        # ics_rl's runner doesn't call reset before rollout; mjlab expects one.
        self.env.reset()

    @property
    def cfg(self) -> ManagerBasedRlEnvCfg:
        return self.unwrapped.cfg

    @property
    def render_mode(self) -> str | None:
        return self.env.render_mode

    @property
    def observation_space(self):
        return self.env.observation_space

    @property
    def action_space(self):
        return self.env.action_space

    @classmethod
    def class_name(cls) -> str:
        return cls.__name__

    @property
    def unwrapped(self) -> ManagerBasedRlEnv:
        return self.env.unwrapped

    @property
    def episode_length_buf(self) -> torch.Tensor:
        return self.unwrapped.episode_length_buf

    @episode_length_buf.setter
    def episode_length_buf(self, value: torch.Tensor) -> None:
        self.unwrapped.episode_length_buf = value

    def seed(self, seed: int = -1) -> int:
        return self.unwrapped.seed(seed)

    def get_obs_format(self) -> dict[str, dict[str, tuple[int, ...]]]:
        active_terms = self.unwrapped.observation_manager.active_terms
        term_dims = self.unwrapped.observation_manager.group_obs_term_dim

        obs_format: dict[str, dict[str, tuple[int, ...]]] = {}
        for exposed_name, source_group in self._group_map.items():
            obs_format[exposed_name] = dict(
                zip(active_terms[source_group], term_dims[source_group], strict=False)
            )
        # Any group beyond policy/critic (e.g. "amp_policy") passes through under its
        # own name -- algorithms that need it (e.g. AMPAlgoMixin's actor_state_key)
        # look it up by that name in extras["observations"] / get_obs_format().
        for group_name in active_terms:
            if group_name in obs_format:
                continue
            obs_format[group_name] = dict(
                zip(active_terms[group_name], term_dims[group_name], strict=False)
            )
        return obs_format

    def get_observations(self) -> tuple[torch.Tensor, dict]:
        obs_dict = self.unwrapped.observation_manager.compute()
        packed_obs = self._pack_observations(obs_dict)
        return packed_obs["policy"], {"observations": packed_obs}

    def reset(self) -> tuple[torch.Tensor, dict]:
        obs_dict, extras = self.env.reset()
        packed_obs = self._pack_observations(obs_dict)
        extras = dict(extras)
        self._stabilize_log(extras)
        extras["observations"] = packed_obs
        return packed_obs["policy"], extras

    def step(self, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, dict]:
        obs_dict, rewards, terminated, truncated, extras = self.env.step(actions)
        packed_obs = self._pack_observations(obs_dict)
        dones = (terminated | truncated).to(dtype=torch.long)

        extras = dict(extras)
        self._stabilize_log(extras)
        extras["observations"] = packed_obs
        if not self.unwrapped.cfg.is_finite_horizon:
            extras["time_outs"] = truncated

        if isinstance(rewards, dict):
            rewards = torch.stack(list(rewards.values()), dim=-1)
        if rewards.ndim == 1:
            rewards = rewards.unsqueeze(1)
        return packed_obs["policy"], rewards, dones, extras

    def close(self) -> None:
        self.env.close()

    """
    Policy export
    """

    def get_policy_metadata(self) -> dict:
        """Deployment metadata embedded into the exported ONNX policy.

        Everything generic is read from the live env (the compiled MuJoCo model and
        the managers), not from the cfg, so it reflects what the policy was actually
        run with. Command/action terms contribute via ``term_policy_metadata``: their
        own ``get_policy_metadata() -> dict`` method (e.g. the AMP run task's hull
        velocity command) or a ``TERM_METADATA`` entry for mjlab's stock terms. Term
        entries are merged in last and may override the generic ones.
        """
        env = self.unwrapped
        metadata: dict = {
            "decimation": env.cfg.decimation,
            "dt": env.physics_dt,
            "step_dt": env.step_dt,
        }

        # -- robot: PD gains / effort limits from the compiled model, in joint order
        if "robot" in env.scene.entities:
            robot = env.scene["robot"]
            ctrl_id_by_joint: dict[str, int] = {}
            for act in robot.actuators:
                for name, ctrl_id in zip(act.target_names, act.global_ctrl_ids.tolist(), strict=True):
                    ctrl_id_by_joint[name] = ctrl_id
            joint_names = [n for n in robot.joint_names if n in ctrl_id_by_joint]
            ctrl_ids = [ctrl_id_by_joint[n] for n in joint_names]
            mj_model = env.sim.mj_model
            joint_ids = [robot.joint_names.index(n) for n in joint_names]
            metadata["joint_names"] = joint_names
            metadata["joint_stiffness"] = mj_model.actuator_gainprm[ctrl_ids, 0].tolist()
            metadata["joint_damping"] = (-mj_model.actuator_biasprm[ctrl_ids, 2]).tolist()
            metadata["joint_effort_limits"] = mj_model.actuator_forcerange[ctrl_ids, 1].tolist()
            # Reflected rotor inertia: set from the actuator cfg, not part of the robot XML,
            # so a standalone MuJoCo sim2sim needs it to reproduce the joint dynamics.
            dof_adr = robot.indexing.joint_v_adr[joint_ids].cpu().numpy()
            metadata["joint_armature"] = mj_model.dof_armature[dof_adr].tolist()
            metadata["default_joint_pos"] = robot.data.default_joint_pos[0, joint_ids].cpu().tolist()

        # -- action/command terms (see TERM_METADATA / term_policy_metadata)
        action_manager, command_manager = env.action_manager, env.command_manager
        for key, manager in (("action_types", action_manager), ("command_types", command_manager)):
            metadata[key] = {name: type(manager.get_term(name).cfg).__name__ for name in manager.active_terms}
        term_metadata: dict = {}
        for manager in (action_manager, command_manager):
            for name in manager.active_terms:
                term_metadata.update(term_policy_metadata(manager.get_term(name)))

        # -- policy observations
        obs_manager = env.observation_manager
        group_name = self.policy_group
        term_names = obs_manager.active_terms[group_name]
        term_dims = obs_manager.group_obs_term_dim[group_name]
        group_cfg = env.cfg.observations[group_name]
        metadata["observation_names"] = list(term_names)
        metadata["observation_dims"] = [int(torch.tensor(d).prod().item()) for d in term_dims]
        metadata["observation_dim"] = sum(metadata["observation_dims"])
        scales, clips, history_lengths = [], [], []
        for name in term_names:
            term_cfg = group_cfg.terms[name]
            scale = term_cfg.scale
            if isinstance(scale, torch.Tensor):
                scale = scale.cpu().tolist()
            scales.append(scale if scale is not None else 1.0)
            clips.append(term_cfg.clip[1] if term_cfg.clip is not None else 10000.0)
            history_lengths.append(term_cfg.history_length or 1)
        metadata["observation_scales"] = scales
        metadata["observation_clips"] = clips
        metadata["observation_history_lengths"] = history_lengths
        metadata["observation_history_length"] = group_cfg.history_length or 1

        metadata.update(term_metadata)
        return metadata

    def _stabilize_log(self, extras: dict) -> None:
        """Keep extras['log'] keys stable across steps (some only appear on reset)."""
        log_info = extras.get("log")
        if not isinstance(log_info, dict):
            return
        stable_log = dict(log_info)
        for key, value in stable_log.items():
            self._log_defaults[key] = self._zero_like(value)
        for key, default_value in self._log_defaults.items():
            stable_log.setdefault(key, default_value)
        extras["log"] = stable_log

    def _zero_like(self, value: object) -> torch.Tensor:
        if isinstance(value, torch.Tensor):
            return torch.zeros_like(value)
        if isinstance(value, bool):
            return torch.tensor(False, device=self.device)
        if isinstance(value, int):
            return torch.tensor(0, device=self.device)
        return torch.tensor(0.0, device=self.device)

    def _pack_observations(self, obs_dict: dict) -> dict[str, torch.Tensor]:
        packed: dict[str, torch.Tensor] = {}
        for exposed_name, source_group in self._group_map.items():
            packed[exposed_name] = self._flatten_group(source_group, obs_dict[source_group])
        for group_name, group_obs in obs_dict.items():
            if group_name in self._group_map.values():
                continue
            packed[group_name] = self._flatten_group(group_name, group_obs)
        return packed

    def _flatten_group(
        self, group_name: str, group_obs: torch.Tensor | dict[str, torch.Tensor]
    ) -> torch.Tensor:
        if isinstance(group_obs, torch.Tensor):
            return group_obs.flatten(start_dim=1)
        active_terms = self.unwrapped.observation_manager.active_terms[group_name]
        return torch.cat([group_obs[term].flatten(start_dim=1) for term in active_terms], dim=1)
