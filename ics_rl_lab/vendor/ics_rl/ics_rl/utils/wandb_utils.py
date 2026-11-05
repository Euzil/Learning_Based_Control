from __future__ import annotations

import os
from dataclasses import asdict

from torch.utils.tensorboard import SummaryWriter

try:
    import wandb
except ModuleNotFoundError:
    raise ModuleNotFoundError("wandb package is required to log to Weights and Biases.") from None


class WandbSummaryWriter(SummaryWriter):
    """Summary writer that mirrors all scalar logs to Weights & Biases."""

    def __init__(self, log_dir: str, flush_secs: int, cfg: dict) -> None:
        super().__init__(log_dir, flush_secs)

        # One wandb project per experiment ("<wandb_project>_<experiment_name>"), the
        # run named by its log dir ("<timestamp>[_<run_name>]").
        run_name = os.path.split(log_dir)[-1]

        try:
            project = cfg["wandb_project"]
        except KeyError:
            raise KeyError("Please specify wandb_project in the runner config.") from None
        if experiment_name := cfg.get("experiment_name"):
            project = f"{project}_{experiment_name}"

        entity = os.environ.get("WANDB_USERNAME", None)

        wandb.init(project=project, entity=entity, name=run_name)
        wandb.config.update({"log_dir": log_dir})

    def store_config(self, env_cfg, train_cfg: dict) -> None:
        wandb.config.update({"runner_cfg": train_cfg})
        if "policy" in train_cfg:
            wandb.config.update({"policy_cfg": train_cfg["policy"]})
        if "algorithm" in train_cfg:
            wandb.config.update({"alg_cfg": train_cfg["algorithm"]})
        try:
            wandb.config.update({"env_cfg": env_cfg.to_dict()})
        except Exception:
            try:
                wandb.config.update({"env_cfg": asdict(env_cfg)})
            except Exception:
                pass

    def add_scalar(
        self,
        tag: str,
        scalar_value: float,
        global_step: int | None = None,
        walltime: float | None = None,
        new_style: bool = False,
    ) -> None:
        super().add_scalar(tag, scalar_value, global_step=global_step, walltime=walltime, new_style=new_style)
        # Skip time-axis variants (e.g. "Train/time/mean_reward") — wandb timestamps every entry natively
        if "/time/" in tag:
            return
        wandb.log({tag: scalar_value}, step=int(global_step) if global_step is not None else None)

    def stop(self) -> None:
        wandb.finish()

    def save_model(self, model_path: str, it: int) -> None:
        wandb.save(model_path, base_path=os.path.dirname(model_path))

    def save_file(self, path: str) -> None:
        wandb.save(path, base_path=os.path.dirname(path))
