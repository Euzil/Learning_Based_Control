import numpy as np

from engineai_deploy.agent.pm01_agent import PM01Agent
from engineai_deploy.agent.robot_state import RobotState
from engineai_deploy.model.actor_critic_mlp import Policy
from engineai_deploy.observations.observations import ObservationManager
from engineai_deploy.agent.control_input import ControlInput
from engineai_deploy.commands import COMMAND_REGISTRY
from engineai_deploy.actions import ACTION_REGISTRY


class PolicySkill:
    """Encapsulates a single policy with its own command, action, and
    observation manager, all selected from the policy metadata.

    The skill speaks policy order internally and returns robot-order targets,
    so the runner stays policy-agnostic. Call activate() when this skill
    becomes the active one to push its gains to the agent.
    """
    def __init__(self,
                 name: str,
                 model_path: str,
                 agent: PM01Agent,
                 verbose: bool = False):
        self._name = name
        self._agent = agent

        # --- Policy
        self._policy = Policy(model_path, verbose=verbose)
        self._metadata = self._policy.metadata

        self._dt_control = float(self._metadata["dt"]) * int(self._metadata["decimation"])

        # --- Command
        command_cls_name = next(iter(self._metadata["command_types"].values()))
        command_cls = COMMAND_REGISTRY.get(command_cls_name)
        if command_cls is None:
            raise ValueError(f"[PolicySkill]: '{name}' unknown command type '{command_cls_name}'")
        print(f"[PolicySkill]: '{name}' command type '{command_cls_name}'")
        self._command = command_cls(self._metadata)

        # --- Action
        action_cls_name = next(iter(self._metadata["action_types"].values()))
        action_cls = ACTION_REGISTRY.get(action_cls_name)
        if action_cls is None:
            raise ValueError(f"[PolicySkill]: '{name}' unknown action type '{action_cls_name}'")
        print(f"[PolicySkill]: '{name}' action type '{action_cls_name}'")
        self._joint_action = action_cls(self._metadata, agent, command=self._command)
        self._joint_mapping = self._joint_action.joint_mapping
        num_policy_dof = self._joint_action.num_policy_dof

        # --- Observations
        self._state = RobotState(
            dof_pos=self._joint_action.policy_default_pos.copy(),
            dof_vel=np.zeros(num_policy_dof),
        )
        self._update_state()

        self._observation_manager = ObservationManager(
            self._metadata,
            self._state,
            self._command,
            agent.config.max_obs_value,
        )

        expected_dim = self._policy.observation_dimension
        obs_dim = self._observation_manager.obs_dim
        if expected_dim != obs_dim:
            raise ValueError(
                f"[PolicySkill]: '{name}' expects obs dim={expected_dim} "
                f"but got obs dim={obs_dim}"
            )

        self._action = np.zeros(num_policy_dof)

    @property
    def name(self) -> str:
        return self._name

    @property
    def dt_control(self) -> float:
        return self._dt_control

    @property
    def robot_stiffness(self) -> np.ndarray:
        return self._joint_action.robot_stiffness

    @property
    def robot_damping(self) -> np.ndarray:
        return self._joint_action.robot_damping

    def activate(self):
        """Push this skill's joint gains to the agent. Call on skill switch."""
        self._agent.set_gains(self.robot_stiffness, self.robot_damping)

    def reset(self):
        """Clear observation history and last action."""
        self._observation_manager.reset()
        self._action[:] = 0.0

    def zero_last_action(self):
        """Zero the last action fed back into the observations (startup warmup)."""
        self._action[:] = 0.0

    def step(self, input: ControlInput) -> tuple[np.ndarray, np.ndarray]:
        """Sync state, update command, run inference.
        Returns (target_robot_pos, target_robot_vel) in agent joint order.
        """
        self._update_state()

        self._command.compute(input)

        self._observation_manager.set_last_action(self._action)
        obs = self._observation_manager.compute_obs()

        self._action = self._policy.forward(obs).flatten()

        # apply() returns internal buffers, copy so callers can blend safely
        robot_pos, robot_vel = self._joint_action.apply(self._action)
        return robot_pos.copy(), robot_vel.copy()

    def _update_state(self):
        s = self._agent.sensor_state
        self._state.dof_pos[:] = s.dof_pos[self._joint_mapping]
        self._state.dof_vel[:] = s.dof_vel[self._joint_mapping]
        self._state.sensors = s
