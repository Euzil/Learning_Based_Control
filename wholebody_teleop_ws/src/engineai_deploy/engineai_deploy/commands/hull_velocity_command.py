import numpy as np

from .commands_base import CommandBase

from interface_protocol.msg import GamepadKeys
from engineai_deploy.agent.control_input import ControlInput

class HullVelocityCommand(CommandBase):
    """
    HullVelocityCommand
    Planar Velocity Command (vx, vy, omega).
    vy and omega are clamped based on speed vx.
    """
    def __init__(self, metadata: dict):
        
        self._cmd = np.zeros(3)
        self._is_moving = False
        
        vy_pts = np.array(metadata["vy_bound_points"])
        self.vy_xp = vy_pts[:, 0]
        self.vy_yp = vy_pts[:, 1]
        
        omega_pts = np.array(metadata["omega_bound_points"])
        self.omega_xp = omega_pts[:, 0]
        self.omega_yp = omega_pts[:, 1]
        
        self.min_vx = max(self.vy_xp[0], self.omega_xp[0])
        self.max_vx = min(self.vy_xp[-1], self.omega_xp[-1])
        
    def compute(self, input: ControlInput) -> np.ndarray:
        analog = input.analog

        # read raw commands        
        raw_vx = analog[GamepadKeys.LEFT_STICK_X]
        if not self._is_moving and raw_vx > 0.0:
            self._is_moving = True
            raw_vx = 0.3
        elif self._is_moving and raw_vx < 0.3:
            self._is_moving = False
            raw_vx = 0.0
        raw_vy = 0.0
        joy_omega = analog[GamepadKeys.RIGHT_STICK_Y]
        
        # forward velocity scaled
        if raw_vx >= 0:
            vx = raw_vx * self.max_vx
        else:
            vx = raw_vx * abs(self.min_vx) 
            
        # Interpolate dynamic max bounds based on the scaled vx
        max_vy = np.interp(vx, self.vy_xp, self.vy_yp)
        max_omega = np.interp(vx, self.omega_xp, self.omega_yp)
        
        # Scale lateral and angular velocities
        vy = raw_vy * max_vy
        omega = joy_omega * max_omega
        
        self._cmd[:] = np.array([vx, vy, omega])
        
        return self._cmd