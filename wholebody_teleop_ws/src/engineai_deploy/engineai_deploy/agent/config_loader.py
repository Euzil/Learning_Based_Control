import yaml
import numpy as np
from pathlib import Path
from typing import List, Dict, Any, Optional


class SensorTopic:
    name: str = ""
    topic: str = ""


class ConfigLoader:
    """Loads per-robot configuration from a YAML file.

    Supports format:

    robots:
        - name: pm01
          joints: { ... }
          sensors: { ... }
          frames: { ... }

    Relative topics (not starting with '/') are automatically prefixed with /<robot_name>/.
    """

    def __init__(self, config_path: str, name: str = "pm01"):
        self.config_path = Path(config_path)
        self._raw = self._load_file()

        robot_node, prefix = self._find_robot(name)

        # --- Safty Limits
        safty_cfg = robot_node.get('safty', {})
        self.max_joint_delta: float = float(safty_cfg.get('max_joint_delta'))
        self.max_joint_vel: float = float(safty_cfg.get('max_joint_vel'))
        self.max_imu_tilt: float = float(safty_cfg.get('max_imu_tilt'))
        self.max_obs_value: float = float(safty_cfg.get('max_obs_value'))
    
        # --- Joints
        joints_cfg  = robot_node.get('joints', {})
        self.joint_state_topic: str = self._resolve(
            joints_cfg.get('state_topic', ''), prefix)
        self.joint_command_topic: str = self._resolve(
            joints_cfg.get('command_topic', ''), prefix)
        self.joint_names: List[str] = joints_cfg.get('names', [])

        n = len(self.joint_names)
        self.joint_default_stiffness: np.ndarray = np.array(
            joints_cfg.get('default_stiffness', []), dtype=float)
        self.joint_default_damping: np.ndarray = np.array(
            joints_cfg.get('default_damping', []), dtype=float)
        self.joint_default_position: np.ndarray = np.array(
            joints_cfg.get('default_position', np.zeros(n).tolist()), dtype=float)
        
        self._validate_joints()

        # --- Frames
        frames_cfg  = robot_node.get('frames')
        self.base_frame: str       = frames_cfg.get('base', '')
        self.left_foot_frame: str  = frames_cfg.get('left_foot', '')
        self.right_foot_frame: str = frames_cfg.get('right_foot', '')
        self._validate_frames()

        # --- Sensors
        sensors_cfg = robot_node.get('sensors', {})
        self.imu_sensors:  List[SensorTopic] = self._load_sensors(
            sensors_cfg.get('imu_sensors', []), prefix)
        self.ft_sensors:   List[SensorTopic] = self._load_sensors(
            sensors_cfg.get('ft_sensors', []), prefix)
        self.odom_sensors: List[SensorTopic] = self._load_sensors(
            sensors_cfg.get('odom_sensors', []), prefix)
        self.skin_sensors: List[SensorTopic] = self._load_sensors(
            sensors_cfg.get('skin_sensors', []), prefix)
        self.ray_sensors:  List[SensorTopic] = self._load_sensors(
            sensors_cfg.get('ray_caster_sensors', []), prefix)
        self._validate_sensors()

    # -------------------------------------------------------------------------
    # Private helpers
    # -------------------------------------------------------------------------

    def _load_file(self) -> Dict[str, Any]:
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)

    def _find_robot(self, name: str):
        """Return (robot_node, prefix) for the requested robot name.

        Multi-robot format: search robots list for matching name.
        Legacy flat format: return top-level data with empty prefix.
        """
        if 'robots' in self._raw:
            robots = self._raw['robots']
            if not isinstance(robots, list):
                raise ValueError("'robots' must be a list")
            for robot in robots:
                if robot.get('name', '') == name:
                    return robot, name
            available = [r.get('name', '') for r in robots]
            raise ValueError(
                f"Robot '{name}' not found in config. Available: {available}")
        # Legacy flat format – treat whole file as one unnamed robot
        return self._raw, ''

    @staticmethod
    def _resolve(topic: str, prefix: str) -> str:
        """Apply robot name prefix to a relative topic."""
        if not topic or topic.startswith('/'):
            return topic
        if not prefix:
            return '/' + topic
        return f'/{prefix}/{topic}'

    def _load_sensors(self, cfg_elements: list, prefix: str) -> List[SensorTopic]:
        sensors = []
        for elem in cfg_elements:
            s = SensorTopic()
            s.name  = elem['name']
            s.topic = self._resolve(elem['topic'], prefix)
            sensors.append(s)
        return sensors

    def _validate_joints(self):
        n = len(self.joint_names)
        for attr, arr in (
            ('default_stiffness', self.joint_default_stiffness),
            ('default_damping',   self.joint_default_damping),
            ('default_position',  self.joint_default_position),
        ):
            if len(arr) != n:
                raise ValueError(
                    f"Joint config mismatch: '{attr}' has {len(arr)} entries "
                    f"but 'names' has {n}.")

    def _validate_sensors(self):
        if not self.imu_sensors:
            raise ValueError("Config error: no IMU sensors found.")

    def _validate_frames(self):
        missing = [k for k, v in (
            ('base',        self.base_frame),
            ('left_foot',   self.left_foot_frame),
            ('right_foot',  self.right_foot_frame),
        ) if not v]
        if missing:
            raise ValueError(
                f"Config error: missing required frame mappings: {', '.join(missing)}")
