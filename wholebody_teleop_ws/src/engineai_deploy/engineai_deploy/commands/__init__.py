from .uniform_velocity_command import UniformVelocityCommand
from .hull_velocity_command import HullVelocityCommand
from .ulc_command import ULCCommand

COMMAND_REGISTRY = {
    "UniformVelocityCommandCfg": UniformVelocityCommand,
    "HullVelocityCommandCfg":    HullVelocityCommand,
    "ULCCommandCfg":             ULCCommand,
}
