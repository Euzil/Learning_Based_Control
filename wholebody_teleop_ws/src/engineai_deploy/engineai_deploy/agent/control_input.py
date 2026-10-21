import numpy as np
from typing import List
from interface_protocol.msg import GamepadKeys

class ControlInput:
    """Extracts inputs from gamepad msg.
    Provides basic logic keys and analog and digital states 
    """
    def __init__(self):        
        self._analog_states = []
        self._digital_states = []
        self._mode = 0
        self._start = 0
        self._stop = 0
        
        self._prev_mode = False
    
    def update(self, gamepad: GamepadKeys, dt: float):
        """Compute the inputs"""

        # special command keys for logic
        self._start = gamepad.digital_states[GamepadKeys.START]
        self._stop = gamepad.digital_states[GamepadKeys.BACK]
        self._mode = gamepad.digital_states[GamepadKeys.Y]
        
        # raw analog and digital states
        self._analog_states = gamepad.analog_states[:]
        self._digital_states = gamepad.digital_states[:]
            
    @property
    def analog(self) -> List[float]: 
        return self._analog_states
    
    @property
    def digital(self) -> List[float]: 
        return self._digital_states
    
    @property
    def start(self) -> int:
        return self._start
    
    @property
    def stop(self) -> int:
        return self._stop
    
    @property
    def mode(self) -> int: 
        return self._mode
    