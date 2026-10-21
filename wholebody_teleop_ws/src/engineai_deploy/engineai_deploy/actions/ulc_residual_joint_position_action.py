import numpy as np

from .joint_position_action import JointPositionAction

from engineai_deploy.commands.ulc_command import ULCCommand


class ULCResidualJointPositionAction(JointPositionAction):
    """Joint position action with residual feedforward from ULC arm targets.

    For all joints:  q_target = default + action * scale
    For arm joints:  q_target += arm_delta  (arm_target − arm_default)
    Net for arms:    q_target = action * scale + arm_target
    """

    def __init__(self, metadata: dict, agent, command: ULCCommand):
        super().__init__(metadata, agent)
        
        if not isinstance(command, ULCCommand):
            raise ValueError("[ULCResidualJointPositionAction]: wrong command type, expect 'ULCCommand'")
        
        self._ulc_command = command

    def apply(self, raw_action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        policy_target = self._default_dof_pos + raw_action * self._action_scale
        policy_target += self._ulc_command.arm_delta    
        self._robot_cmd_pos[self._joint_mapping] = policy_target
        return self._robot_cmd_pos, self._robot_cmd_vel
