import importlib
from collections import OrderedDict
from typing import Literal

import torch
import torch.distributed as dist
import torch.nn as nn
import torch.optim as optim

import ics_rl.modules as ics_modules
from ics_rl.algorithms.ppo import PPO
from ics_rl.networks.normalizer import EmpiricalNormalization
from ics_rl.storage.amp_storage import AMPReplayBuffer
from ics_rl.utils.buffer import buffer_func
from ics_rl.utils.utils import get_subobs_size


class AMPAlgoMixin:
    """AMP algorithm mixin with persistent replay buffer and empirical normalisation.

    * **Persistent replay buffer** (``amp_replay_buffer_size`` transitions, default 100 k):
      the discriminator trains on a diverse history of policy behaviour instead of only the
      current rollout, which prevents the discriminator from cycling with the policy.

    * **Empirical normalisation** (``amp_normalize_obs=True``): running mean/variance
      normalisation is applied to AMP observations before the discriminator, improving
      stability especially early in training.

    * **Task/style reward blending** (``task_reward_lerp``): by default the discriminator's
      style reward is simply added on top of the full task reward (``PPO.process_env_step``,
      weighted by ``discriminator_reward_coef``). Setting ``task_reward_lerp`` (e.g. ``0.7``)
      scales the task reward down to that fraction before the style term is added, so the two
      terms are blended rather than the style term being a small additive bonus on top of an
      unscaled task reward: ``reward = task_reward_lerp * task_reward + discriminator_reward_coef
      * style_reward``. Leave at ``None`` (default) to keep the pure-additive behavior.

    The algorithm still builds ``(s_{t-1}, s_t)`` state-transition pairs internally from
    single-frame ``actor_state_key`` observations so the env does not need a history buffer.
    Expert reference pairs are sampled from ``motion_loader``.
    """

    def __init__(
        self,
        *args,
        actor_state_key: str = "amp_policy",
        motion_loader=None,
        discriminator_class_name: str = "Discriminator",
        discriminator_kwargs: dict = {},
        discriminator_optimizer_class_name: str = "AdamW",
        discriminator_optimizer_kwargs: dict = {},
        discriminator_reward_coef: float = 1.0,
        discriminator_reward_type: Literal["log", "quad", "wasserstein"] = "log",
        discriminator_loss_func: Literal["WassersteinLoss", "BCEWithLogitsLoss", "MSELoss"] = "BCEWithLogitsLoss",
        discriminator_loss_coef: float = 1.0,
        discriminator_gradient_penalty_coef: float = 10.0,
        amp_replay_buffer_size: int = 100_000,
        amp_normalize_obs: bool = True,
        task_reward_lerp: float | None = None,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.actor_state_key = actor_state_key
        self.motion_loader = motion_loader
        self.discriminator_class_name = discriminator_class_name
        self.discriminator_kwargs = discriminator_kwargs
        self.discriminator_optimizer_class_name = discriminator_optimizer_class_name
        self.discriminator_optimizer_kwargs = discriminator_optimizer_kwargs
        self.discriminator_reward_coef = discriminator_reward_coef
        self.discriminator_reward_type = discriminator_reward_type
        self.discriminator_loss_func = discriminator_loss_func
        self.discriminator_loss_coef = discriminator_loss_coef
        self.discriminator_gradient_penalty_coef = discriminator_gradient_penalty_coef
        self.amp_replay_buffer_size = amp_replay_buffer_size
        self.amp_normalize_obs = amp_normalize_obs
        self.task_reward_lerp = task_reward_lerp

    def init_storage(self, num_envs, num_transitions_per_env, obs_format, num_actions, num_rewards=1):
        super().init_storage(num_envs, num_transitions_per_env, obs_format, num_actions, num_rewards)

        # Build discriminator with doubled (s_{t-1}, s_t) input segment
        if ":" in self.discriminator_class_name:
            DiscriminatorClass = getattr(
                importlib.import_module(self.discriminator_class_name.split(":")[0]),
                self.discriminator_class_name.split(":")[1],
            )
        else:
            DiscriminatorClass = getattr(ics_modules, self.discriminator_class_name)

        single_size = get_subobs_size(obs_format[self.actor_state_key])
        pair_segment = OrderedDict()
        for k, v in obs_format[self.actor_state_key].items():
            pair_segment[f"prev_{k}"] = v
        for k, v in obs_format[self.actor_state_key].items():
            pair_segment[f"next_{k}"] = v

        self.discriminator = DiscriminatorClass(input_segment=pair_segment, **self.discriminator_kwargs).to(self.device)
        if "lr" not in self.discriminator_optimizer_kwargs:
            self.discriminator_optimizer_kwargs = dict(lr=self.learning_rate, **self.discriminator_optimizer_kwargs)
        self.discriminator_optimizer = getattr(optim, self.discriminator_optimizer_class_name)(
            self.discriminator.parameters(), **self.discriminator_optimizer_kwargs
        )

        # Persistent circular replay buffer — survives across rollouts
        self.amp_replay_buffer = AMPReplayBuffer(single_size, self.amp_replay_buffer_size, device=self.device)

        # Optional empirical normaliser for single-frame AMP observations
        if self.amp_normalize_obs:
            self.amp_normalizer = EmpiricalNormalization(shape=[single_size]).to(self.device)
        else:
            self.amp_normalizer = None

    def _normalize(self, x: torch.Tensor) -> torch.Tensor:
        """Normalise a single-frame AMP observation (no-op when normalisation is off)."""
        if self.amp_normalizer is not None:
            return self.amp_normalizer(x)
        return x

    def process_env_step(self, rewards, dones, infos, next_obs, next_critic_obs):
        if self.actor_state_key not in infos["observations"]:
            raise ValueError(
                "actor_state_key '{}' not found in infos['observations']".format(self.actor_state_key)
            )

        s_t = infos["observations"][self.actor_state_key]  # (N, D)

        if not hasattr(self, "_prev_actor_state"):
            self._prev_actor_state = s_t.detach().clone()
        elif dones.any():
            self._prev_actor_state[dones] = s_t[dones].detach()

        # Insert raw (unnormalised) pairs — normaliser keeps its own running stats
        self.amp_replay_buffer.insert(self._prev_actor_state, s_t)

        if self.task_reward_lerp is not None:
            rewards = rewards * self.task_reward_lerp

        # super() calls compute_auxiliary_reward before we update _prev_actor_state
        super().process_env_step(rewards, dones, infos, next_obs, next_critic_obs)

        self._prev_actor_state = s_t.detach()

    @torch.no_grad()
    def compute_auxiliary_reward(self, obs_pack: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        super_reward = super().compute_auxiliary_reward(obs_pack)
        s_t = obs_pack[self.actor_state_key]
        prev = getattr(self, "_prev_actor_state", s_t)
        # Score without updating the normaliser's running stats: this is called on every env, every
        # rollout step (pure policy data, no expert). Left in train mode, it would recalibrate the
        # normaliser almost entirely off the policy's own behaviour instead of a policy/expert
        # balance — collapsing the learned std for low-variance channels (e.g. arms, which have no
        # direct task reward and so little rollout variance until the policy learns to use them) and
        # amplifying normalised deviations there well beyond what the discriminator was trained on.
        # Stats are only updated in compute_amp_losses, on a balanced policy+expert batch.
        normalizer = self.amp_normalizer
        was_training = normalizer is not None and normalizer.training
        if normalizer is not None and was_training:
            normalizer.eval()
        try:
            disc = self.discriminator(torch.cat([self._normalize(prev), self._normalize(s_t)], dim=-1)).detach()
        finally:
            if normalizer is not None and was_training:
                normalizer.train()
        if self.discriminator_reward_type == "log":
            reward = -torch.log(1 - torch.clamp(torch.sigmoid(disc), 1e-6, 1 - 1e-6))
        elif self.discriminator_reward_type == "quad":
            reward = torch.clamp(1 - (1 / 4) * torch.square(disc - 1), min=0)
        elif self.discriminator_reward_type == "wasserstein":
            reward = disc
        super_reward["discriminator_reward"] = reward
        return super_reward

    def update(self, *args, **kwargs):
        if self.motion_loader is None:
            raise RuntimeError("AMPAlgoMixin requires motion_loader to be set before update().")

        mean_losses, average_stats = super().update(*args, **kwargs)

        mini_batch_size = self.storage.num_envs * self.storage.num_transitions_per_env // self.num_mini_batches
        mini_batch_size = max(mini_batch_size, 1)
        num_updates = self.num_learning_epochs * self.num_mini_batches

        amp_policy_generator = self.amp_replay_buffer.mini_batch_generator(num_updates, mini_batch_size)
        amp_expert_generator = self.motion_loader.mini_batch_generator(num_updates, mini_batch_size)

        for policy_pair, expert_pair in zip(amp_policy_generator, amp_expert_generator):
            losses, _, stats = self.compute_amp_losses(policy_pair, expert_pair)

            loss = 0.0
            for k, v in losses.items():
                loss += getattr(self, k + "_coef", 1.0) * v
                mean_losses[k] = mean_losses.get(k, torch.zeros(1, device=self.device)[0]) + v.detach() / num_updates
            mean_losses["amp_total_loss"] = mean_losses.get(
                "amp_total_loss", torch.zeros(1, device=self.device)[0]
            ) + loss.detach() / num_updates
            for k, v in stats.items():
                average_stats[k] = average_stats.get(
                    k, torch.zeros(1, device=self.device)[0]
                ) + v.detach() / num_updates

            self._discriminator_gradient_step(loss)

        return mean_losses, average_stats

    def compute_amp_losses(
        self,
        policy_pair: tuple[torch.Tensor, torch.Tensor],
        expert_pair: tuple[torch.Tensor, torch.Tensor],
    ):
        losses, stats, inter_vars = dict(), dict(), dict()

        policy_states, policy_next_states = policy_pair
        expert_states, expert_next_states = expert_pair

        # Normalise all four batches under one frozen snapshot of the running stats, then update
        # the stats once afterwards on the raw tensors. Calling _normalize() on each of the 4
        # batches back-to-back in train mode (the previous behaviour) would auto-update stats
        # after every call — so policy_states, policy_next_states, expert_states and
        # expert_next_states would each be scored through a slightly different, drifting affine
        # map within the same minibatch, a normalization-scale leak the discriminator could exploit
        # instead of learning real motion content. Matches rsl_rl's explicit
        # update_normalization(...), which always runs on raw tensors after scoring.
        normalizer = self.amp_normalizer
        was_training = normalizer is not None and normalizer.training
        if normalizer is not None and was_training:
            normalizer.eval()
        policy_d_input = torch.cat([self._normalize(policy_states), self._normalize(policy_next_states)], dim=-1)
        expert_d_input = torch.cat([self._normalize(expert_states), self._normalize(expert_next_states)], dim=-1)
        if normalizer is not None and was_training:
            normalizer.train()
            with torch.no_grad():
                normalizer.update(policy_states)
                normalizer.update(policy_next_states)
                normalizer.update(expert_states)
                normalizer.update(expert_next_states)

        # Score policy and expert samples in a single forward pass (not two separate calls): when
        # use_minibatch_std is enabled, the batch-std feature must be computed over a batch containing
        # both populations, otherwise it trivially leaks which population a sample came from (a policy-only
        # batch and an expert-only batch generally have different internal diversity), letting the
        # discriminator "cheat" via batch statistics instead of learning per-sample motion features.
        batch_size = policy_d_input.shape[0]
        combined_d = self.discriminator(torch.cat([policy_d_input, expert_d_input], dim=0))
        policy_d, expert_d = combined_d[:batch_size], combined_d[batch_size:]

        if self.discriminator_loss_func == "WassersteinLoss":
            policy_d_loss = policy_d.mean()
            expert_d_loss = -expert_d.mean()
        elif self.discriminator_loss_func == "BCEWithLogitsLoss":
            policy_d_loss = torch.nn.BCEWithLogitsLoss()(policy_d, torch.zeros_like(policy_d))
            expert_d_loss = torch.nn.BCEWithLogitsLoss()(expert_d, torch.ones_like(expert_d))
        elif self.discriminator_loss_func == "MSELoss":
            policy_d_loss = torch.nn.MSELoss()(policy_d, -torch.ones_like(policy_d))
            expert_d_loss = torch.nn.MSELoss()(expert_d, torch.ones_like(expert_d))
        else:
            raise NotImplementedError(f"discriminator loss function {self.discriminator_loss_func} not implemented")
        losses["discriminator_loss"] = (policy_d_loss + expert_d_loss) * 0.5

        if self.discriminator_gradient_penalty_coef > 0:
            losses["discriminator_gradient_penalty"] = self._gradient_penalty(policy_d_input, expert_d_input)
        else:
            losses["discriminator_gradient_penalty"] = torch.zeros(1, device=self.device)[0]

        stats["discriminator_policy"] = policy_d.mean()
        stats["discriminator_expert"] = expert_d.mean()

        if self.discriminator_loss_func in ("BCEWithLogitsLoss", "MSELoss"):
            policy_prob = torch.sigmoid(policy_d)
            expert_prob = torch.sigmoid(expert_d)
            stats["accuracy_policy"] = (torch.round(policy_prob) == torch.zeros_like(policy_prob)).float().mean()
            stats["accuracy_expert"] = (torch.round(expert_prob) == torch.ones_like(expert_prob)).float().mean()
        elif self.discriminator_loss_func == "WassersteinLoss":
            stats["accuracy_policy"] = (policy_d < 0).float().mean()
            stats["accuracy_expert"] = (expert_d > 0).float().mean()

        return losses, inter_vars, stats

    def _gradient_penalty(self, policy_input: torch.Tensor, expert_input: torch.Tensor) -> torch.Tensor:
        """Gradient penalty regularising the discriminator, on already-normalised concatenated
        pair inputs.

        BCEWithLogitsLoss / MSELoss: standard R1 penalty — squared gradient norm of D w.r.t.
        real/expert samples only (fake/policy samples are not penalised). The minibatch-std
        feature, if enabled, is detached so the penalty measures per-sample sensitivity only,
        not sensitivity to batch composition.
        WassersteinLoss: WGAN-GP — penalises deviation from unit gradient norm along the line
        between policy and expert samples.
        """
        if self.discriminator_loss_func == "WassersteinLoss":
            alpha = torch.rand(expert_input.shape[0], 1, device=self.device).expand_as(expert_input)
            interpolated = (alpha * expert_input + (1 - alpha) * policy_input).detach().requires_grad_(True)
            disc = self.discriminator(interpolated, detach_minibatch_std=True)
            grad = torch.autograd.grad(
                outputs=disc,
                inputs=interpolated,
                grad_outputs=torch.ones_like(disc),
                create_graph=True,
                retain_graph=True,
                only_inputs=True,
            )[0]
            return (grad.norm(2, dim=1) - 1.0).pow(2).mean()

        data = expert_input.detach().requires_grad_(True)
        disc = self.discriminator(data, detach_minibatch_std=True)
        grad = torch.autograd.grad(
            outputs=disc.sum(),
            inputs=data,
            create_graph=True,
            retain_graph=True,
            only_inputs=True,
        )[0]
        return grad.pow(2).sum(dim=1).mean()

    def _discriminator_gradient_step(self, loss: torch.Tensor):
        self.discriminator_optimizer.zero_grad()
        loss.backward()
        if dist.is_initialized():
            world_size = dist.get_world_size()
            for param in self.discriminator.parameters():
                if param.grad is not None:
                    dist.all_reduce(param.grad.data, op=dist.ReduceOp.SUM)
                    param.grad.data /= world_size
        self.discriminator_optimizer.step()

    def state_dict(self):
        sd = super().state_dict()
        sd["discriminator"] = self.discriminator.state_dict()
        sd["discriminator_optimizer"] = self.discriminator_optimizer.state_dict()
        if self.amp_normalizer is not None:
            sd["amp_normalizer"] = self.amp_normalizer.state_dict()
        return sd

    def load_state_dict(self, state_dict):
        super().load_state_dict(state_dict)
        if "discriminator" in state_dict:
            self.discriminator.load_state_dict(state_dict["discriminator"])
        else:
            print("[Warning] discriminator state_dict not found in checkpoint")
        if "discriminator_optimizer" in state_dict:
            self.discriminator_optimizer.load_state_dict(state_dict["discriminator_optimizer"])
        else:
            print("[Warning] discriminator_optimizer state_dict not found in checkpoint")
        if self.amp_normalizer is not None and "amp_normalizer" in state_dict:
            self.amp_normalizer.load_state_dict(state_dict["amp_normalizer"])


class AMPAlgoPPO(AMPAlgoMixin, PPO):
    pass
