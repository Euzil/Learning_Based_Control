import numpy as np
import pinocchio as pin

from engineai_deploy.agent.config_loader import ConfigLoader

class RobotModel:
    """Computes robot forward kinematics and dynamics.
    """
    def __init__(self, 
                 config: ConfigLoader):
        
        # build the model
        model, _, _ = pin.buildModelsFromUrdf(
            
        )