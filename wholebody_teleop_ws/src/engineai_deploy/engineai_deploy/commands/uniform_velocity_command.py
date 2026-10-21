import numpy as np

from .commands_base import CommandBase

from interface_protocol.msg import GamepadKeys
from engineai_deploy.agent.control_input import ControlInput

class UniformVelocityCommand(CommandBase):
    """
    UniformVelocityCommand
    Planar Velocity Command (vx, vy, omega).
    """
    def __init__(self, metadata: dict):

        self._cmd = np.zeros(3)
        self._lb = np.zeros(3)
        self._ub = np.zeros(3)

        cmd_keys = ["cmd_range_vx", "cmd_range_vy", "cmd_range_omega"]
        for i, key in enumerate(cmd_keys):
            if key in metadata:
                limits = metadata[key]
                self._lb[i] = abs(limits[0])
                self._ub[i] = abs(limits[1])
    
    def compute(self, input: ControlInput) -> np.ndarray:
        """ Reads gamepad and scales inputs based on configured limits.
        """     
        analog = input.analog        
        
        raw_cmd = np.array([
            analog[GamepadKeys.LEFT_STICK_X],
            analog[GamepadKeys.LEFT_STICK_Y],
            analog[GamepadKeys.RIGHT_STICK_Y],
        ])
        sign = np.sign(raw_cmd)
        
        limit = np.where(sign >= 0, self._ub, self._lb)
        self._cmd[:] = raw_cmd * limit
        return self._cmd