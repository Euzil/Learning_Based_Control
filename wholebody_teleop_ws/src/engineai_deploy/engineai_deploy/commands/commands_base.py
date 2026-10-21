import numpy as np
from engineai_deploy.agent.control_input import ControlInput

class CommandBase:
    def __init__(self):
        self._cmd = np.array([])

    def compute(self, input: ControlInput) -> np.ndarray:
        raise NotImplementedError

    @property
    def command(self) -> np.ndarray: return self._cmd
