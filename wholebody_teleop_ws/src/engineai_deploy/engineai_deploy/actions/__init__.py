from .joint_position_action import JointPositionAction
from .ulc_residual_joint_position_action import ULCResidualJointPositionAction

ACTION_REGISTRY = {
    "JointPositionActionCfg":            JointPositionAction,
    "ULCResidualJointPositionActionCfg": ULCResidualJointPositionAction,
}
