# ics_rl

PyTorch on-policy RL (PPO and friends) for legged robots. A fork of
[Instinct RL](https://github.com/project-instinct/instinct_rl), itself built on the RSL-RL code base,
extended to train directly on [mjlab](https://github.com/mujocolab/mjlab) environments. It is the
training library of [ics_rl_lab](https://github.com/TUM-ICS/ics_rl_lab), which
includes it as a git submodule.

## Package layout

```
ics_rl/
  algorithms/   PPO and the algorithms built on it (see below)
  modules/      actor-critic networks, discriminator, build_actor_critic / build_normalizer
  networks/     building blocks: MLP, conv, transformer, MoE layer, VAE, VQ-VAE, normalizers
  runners/      OnPolicyRunner (training loop, checkpoints, ONNX export), TwoStageRunner,
                DemonstrationSaver / DaggerSaver (roll out and store trajectories)
  storage/      rollout storage, AMP replay buffer, rollout datasets on disk
  env/          VecEnv interface; mjlab.py = MjlabVecEnv (needs the mjlab extra)
  utils/        ONNX metadata, wandb writer, checkpoint surgery, misc
```

## Training on mjlab

```python
from ics_rl.env.mjlab import MjlabVecEnv
from ics_rl.runners import OnPolicyRunner

env = MjlabVecEnv(ManagerBasedRlEnv(cfg=env_cfg, device="cuda:0"))
runner = OnPolicyRunner(env, train_cfg, log_dir=log_dir, device="cuda:0")
runner.learn(num_learning_iterations=10_000)
runner.export_onnx_checkpoint()  # <log_dir>/exported/model_<iter>/<run_name>.onnx
```

`train_cfg` is a dict with `policy`, `algorithm` and runner keys (`num_steps_per_env`,
`save_interval`, `normalizers`, `logger`, ...). ics_rl_lab builds it from dataclasses in
`ics_rl_lab/rl/config.py`; see `scripts/train.py` there for a complete example.

**Observations.** Each mjlab observation group becomes one flat tensor; `get_obs_format()` reports
the per-term shapes (see below), so networks and normalizers know the layout.

**Deployment metadata.** `MjlabVecEnv.get_policy_metadata()` collects the robot's joint names, PD
gains, effort limits, armature and default pose from the compiled model, plus the action and
observation layout, and `export_as_onnx` attaches it to every exported file. Command and action
terms add their own entries through a `get_policy_metadata()` method; for terms you can't edit
(mjlab's stock terms), register an extractor with `register_term_metadata(TermClass, fn)`.

**AMP.** If the algorithm has a `motion_loader` attribute and the scene has a `motionloader`
entity, the runner connects the two, so expert transitions come straight from the env.

**Logging.** `logger="tensorboard"` (default) or `"wandb"`. With wandb, each experiment gets its
own project, `<wandb_project>_<experiment_name>`, and each run is named after its log directory.
Set `WANDB_USERNAME` to log to a specific entity.

## Observation format

- **obs_format**: an ordered dict from observation group name (e.g. `policy`, `critic`) to its
  obs_segment.
- **obs_segment**: an ordered dict from observation term name to the term's shape.
- **obs_pack**: a dict from observation group name to the flattened tensor of that group, which is
  what the algorithms consume.

## Swapping algorithms and networks

The runner builds the policy and the algorithm from `class_name` entries in the config:

```python
actor_critic = modules.build_actor_critic(
    self.policy_cfg.pop("class_name"),
    self.policy_cfg,
    obs_format,
    num_actions=env.num_actions,
    num_rewards=env.num_rewards,
).to(self.device)
```

`class_name` is either a class exported by `ics_rl.modules` (policies) / `ics_rl.algorithms`
(algorithms), or a full import path `package.module:ClassName` to your own class (also for
normalizers). The remaining config entries are passed to the constructor, together with
`obs_format`, `num_actions` and `num_rewards`.

## Algorithms

| Class | Description |
|---|---|
| `PPO` | Clipped-surrogate PPO with value loss and entropy bonus; multiple epochs and mini-batches per update. |
| `AMPAlgoPPO` | PPO plus an adversarial motion prior (AMP, WASABI-style). Discriminator loss: `BCEWithLogitsLoss`, `WassersteinLoss` or `MSELoss`; style reward: `log`, `quad` or `wasserstein`; gradient penalty. Persistent replay buffer of policy transitions, empirical normalization of AMP observations, optional `task_reward_lerp` blending of task and style reward. |
| `EstimatorPPO` / `EstimatorTPPO` | PPO / TPPO plus a supervised loss that trains a state estimator inside the policy. `AMPEstimatorPPO` combines it with AMP. |
| `TPPO` | Teacher-student distillation on top of PPO (DAgger-style), with configurable teacher action probability and distillation loss. |
| `VaeDistill` | Distillation into a VAE student, which adds a latent distribution separate from the action distribution. |
| `MultiTeacherMoEPPO` | Distills several frozen teachers into one (MoE) student with context-gated KL losses. |
| `LipschitzPPO` | PPO plus a gradient penalty that keeps the policy network smooth. |

## Network modules

| Class | Description |
|---|---|
| `ActorCritic` / `ActorCriticRecurrent` | Separate actor and critic MLPs; the recurrent variant adds GRU/LSTM memory. |
| `MoEActorCritic` | Mixture-of-experts actor-critic with a gating network. |
| `VaeActorCritic` | Actor that acts from a VAE latent of the observation. |
| `EncoderActorCritic(Recurrent)` | Actor-critic with encoder backbones for high-dimensional inputs (e.g. height maps, images). |
| `EstimatorActorCritic(Recurrent)` | Actor-critic with a state-estimator head (used by the estimator algorithms). |
| `EncoderStateAc`, `EncoderMoEActorCritic`, `EncoderVaeActorCritic` | Combinations of the above (`all_mixer.py`). |
| `Discriminator` | AMP discriminator. |

Normalizers (`runner_cfg["normalizers"]`): `EmpiricalNormalization` and
`EmpiricalDiscountedVariationNormalization`.

## License

The repository is licensed under [CC BY-NC 4.0](LICENSE), inherited from Instinct RL: you may not
use it for commercial purposes (e.g. demos advertising commercial products, or wrapping the code
into a commercial product). Files derived from RSL-RL keep their BSD-3-Clause headers. Licenses of
dependencies are in [`licenses/`](licenses).