import numpy as np


class JointPositionAction:
    """Owns all joint-position policy setup and per-step action application.

    Handles joint mapping, gain/default merging from policy metadata into
    robot-order arrays, and the per-step action → (robot_pos, robot_vel) transform.
    """

    def __init__(self, metadata: dict, agent, command=None):
        policy_joint_names = metadata["joint_names"]
        self._joint_mapping = self._build_joint_mapping(agent.joint_names, policy_joint_names)

        num_robot_dof = agent.num_dof
        policy_default_pos = np.array(metadata["default_joint_pos"])
        policy_stiffness   = np.array(metadata["joint_stiffness"])
        policy_damping     = np.array(metadata["joint_damping"])
        # scalar or per-joint (policy order), broadcast to all policy joints
        num_policy_dof = len(policy_joint_names)
        action_scale = np.asarray(metadata["action_scale"], dtype=float).reshape(-1)
        if action_scale.size == 1:
            action_scale = np.full(num_policy_dof, action_scale[0])
        elif action_scale.size != num_policy_dof:
            raise ValueError(
                f"[JointPositionAction]: action_scale has {action_scale.size} entries, "
                f"expected 1 or {num_policy_dof}"
            )

        self._robot_stiffness  = agent.joint_default_stiffness
        self._robot_damping    = agent.joint_default_damping
        self._robot_default_pos = agent.joint_default_pos

        for i, robot_idx in enumerate(self._joint_mapping):
            self._robot_stiffness[robot_idx]   = policy_stiffness[i]
            self._robot_damping[robot_idx]     = policy_damping[i]
            self._robot_default_pos[robot_idx] = policy_default_pos[i]

        self._default_dof_pos = policy_default_pos
        self._action_scale    = action_scale
        self._robot_cmd_pos   = self._robot_default_pos.copy()
        self._robot_cmd_vel   = np.zeros(num_robot_dof)

    @property
    def joint_mapping(self) -> list:
        return self._joint_mapping

    @property
    def num_policy_dof(self) -> int:
        return len(self._default_dof_pos)

    @property
    def policy_default_pos(self) -> np.ndarray:
        return self._default_dof_pos

    @property
    def robot_stiffness(self) -> np.ndarray:
        return self._robot_stiffness

    @property
    def robot_damping(self) -> np.ndarray:
        return self._robot_damping

    def apply(self, raw_action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Returns (target_robot_pos, target_robot_vel) for agent.write().
        Output is in agent joint order.
        """
        policy_target = self._default_dof_pos + raw_action * self._action_scale
        self._robot_cmd_pos[self._joint_mapping] = policy_target
        return self._robot_cmd_pos, self._robot_cmd_vel

    @staticmethod
    def _build_joint_mapping(robot_joint_names: list, policy_joint_names: list) -> list:
        mapping = []
        for name in policy_joint_names:
            if name not in robot_joint_names:
                raise ValueError(f"[JointPositionAction]: Joint '{name}' not found in robot joints")
            mapping.append(robot_joint_names.index(name))
        return mapping
