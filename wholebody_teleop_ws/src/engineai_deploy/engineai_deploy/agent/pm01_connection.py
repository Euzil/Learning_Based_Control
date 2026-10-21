from typing import Dict, List
from functools import partial

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from engineai_deploy.agent.config_loader import ConfigLoader

from skin_msgs.msg import SkinFeature, SkinPatchInfo, RayCasterData
from geometry_msgs.msg import WrenchStamped, Wrench
from nav_msgs.msg import Odometry

from interface_protocol.msg import (
    GamepadKeys,
    ImuInfo,
    JointCommand,
    JointState,
    MotionState,
)

class PM01Connection:
    """
    Manages the connection state and data flow for the PM01 agent.
    Real robot topics (mendatory):
     - /hardware/imu_info
     - /hardware/joint_state
     - /motion/motion_state
     - /hardware/gamepad_keys
    Additional topics (optional, only loaded if in config)
     - /pm01/<patch>/info
     - /pm01/<patch>/features
    """

    def __init__(self, node: Node, config: ConfigLoader, is_dry_run: bool = False):
        self._node = node
        self._config = config
        self._is_dry_run = is_dry_run

        # Connection flags: parallel lists so callbacks use fast int indexing
        # while missing_topics can report by name without extra overhead.
        self._is_connected = False
        self._msg_flags: List[bool] = []
        self._topic_names: List[str] = []

        # ROS handles
        self._clock = node.get_clock()
        self._connection_timer = None
        self._last_connection_time = self._clock.now()

        # Data caches
        self._gamepad: GamepadKeys = GamepadKeys()
        self._imu: ImuInfo = ImuInfo()
        self._joint_state: JointState = JointState()
        self._motion_state: MotionState = MotionState()
        self._skin_poses: Dict[str, SkinPatchInfo] = {}
        self._skin_features: Dict[str, SkinFeature] = {}

        # ----------------------------------------------------------------------
        # requried topics

        # Define QoS Profile
        qos = QoSProfile(
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers
        base_imu = config.imu_sensors[0]
        self._imu_idx = self._register_topic(base_imu.topic)
        self._imu_sub = node.create_subscription(
            ImuInfo,
            base_imu.topic,
            self._imu_info_callback,
            qos
        )

        self._joint_state_idx = self._register_topic(config.joint_state_topic)
        self._joint_state_sub = node.create_subscription(
            JointState,
            config.joint_state_topic,
            self._joint_state_callback,
            qos
        )

        self._motion_state_idx = self._register_topic("/motion/motion_state")
        self._motion_state_sub = node.create_subscription(
            MotionState,
            "/motion/motion_state",
            self._motion_state_callback,
            1
        )

        self._gamepad_idx = self._register_topic("/hardware/gamepad_keys")
        self._gamepad_sub = node.create_subscription(
            GamepadKeys,
            "/hardware/gamepad_keys",
            self._gamepad_keys_callback,
            qos
        )

        # ----------------------------------------------------------------------
        # optional topics (skin, ft, odom, etc, ...)
        # will be loaded only if specified in config file

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # skins
        self._skin_info_subs = []
        self._skin_feature_subs = []
        self._skin_infos = {}
        self._skin_features = {}
        for skin in config.skin_sensors:
            info_topic = skin.topic + "/info"
            idx = self._register_topic(info_topic)
            skin_info_sub = node.create_subscription(
                SkinPatchInfo,
                info_topic,
                partial(self._skin_info_callback, name=skin.name, idx=idx),
                latched_qos
            )
            self._skin_info_subs.append(skin_info_sub)
            self._skin_infos[skin.name] = None

            features_topic = skin.topic + "/features"
            idx = self._register_topic(features_topic)
            skin_features_sub = node.create_subscription(
                SkinFeature,
                skin.topic + '/features',
                partial(self._skin_features_callback, name=skin.name, idx=idx),
                qos
            )
            self._skin_feature_subs.append(skin_features_sub)
            self._skin_features[skin.name] = None

        # ft sensors (note only saving wrench)
        self._ft_sensor_subs = []
        self._ft_sensors = {}
        for ft_sensor in config.ft_sensors:
            idx = self._register_topic(ft_sensor.topic)
            ft_sub = node.create_subscription(
                WrenchStamped,
                ft_sensor.topic,
                partial(self._ft_sensor_callback, name=ft_sensor.name, idx=idx),
                qos
            )
            self._ft_sensor_subs.append(ft_sub)
            self._ft_sensors[ft_sensor.name] = None
            
        # odom sensors (note only saving pose and twist)
        self._odom_sensor_subs = []
        self._odom_sensors = {}
        for odom_sensor in config.odom_sensors:
            idx = self._register_topic(odom_sensor.topic)
            odom_sub = node.create_subscription(
                Odometry,
                odom_sensor.topic,
                partial(self._odom_sensor_callback, name=odom_sensor.name, idx=idx),
                qos
            )
            self._odom_sensor_subs.append(odom_sub)
            self._odom_sensors[odom_sensor.name] = None
            
        self._ray_caster_subs = []
        self._ray_caster_sensors = {}
        for ray_sensor in config.ray_sensors:
            idx = self._register_topic(ray_sensor.topic)
            ray_sub = node.create_subscription(
                RayCasterData,
                ray_sensor.topic,
                partial(self._ray_caster_sensor_callback, name=odom_sensor.name, idx=idx),
                qos
            )
            self._ray_caster_subs.append(ray_sub)
            self._ray_caster_sensors[ray_sensor.name] = None
            
        # ----------------------------------------------------------------------
        # Publisher

        self._joint_cmd_pub = node.create_publisher(
            JointCommand,
            config.joint_command_topic,
            qos
        )

        # Whatchdog timer (50ms)
        self._connection_timer = node.create_timer(
            0.05,
            self._connection_timer_callback
        )

    def _register_topic(self, topic: str) -> int:
        """Appends topic to the parallel name/flag lists and returns its index."""
        idx = len(self._topic_names)
        self._topic_names.append(topic)
        self._msg_flags.append(False)
        return idx

    def send_command(self, msg: JointCommand):
        """
        Publishes a JointCommand message.
        """
        # publish the robot command
        if self._joint_cmd_pub is not None:
            self._joint_cmd_pub.publish(msg)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def missing_topics(self) -> List[str]:
        return [name for name, ok in zip(self._topic_names, self._msg_flags) if not ok]

    @property
    def joint_state(self) -> JointState | None:
        return self._joint_state

    @property
    def gamepad_keys(self) -> GamepadKeys | None:
        return self._gamepad

    @property
    def imu_info(self) -> ImuInfo | None:
        return self._imu

    @property
    def motion_state(self) -> MotionState | None:
        return self._motion_state

    @property
    def skin_infos(self) -> Dict[str, SkinPatchInfo]:
        return self._skin_infos
    @property
    def skin_features(self) -> Dict[str, SkinFeature]:
        return self._skin_features
    
    @property
    def ft_sensors(self) -> Dict[str, Wrench]:
        return self._ft_sensors
    
    @property
    def odom_sensors(self) -> Dict[str, Odometry]:
        return self._odom_sensors
    
    @property
    def ray_caster_sensors(self) -> Dict[str, RayCasterData]:
        return self._ray_caster_sensors

    def _connection_timer_callback(self):
        """ Internal timer callback to manage connection state.
        """
        if self._motion_state is None:
            # Cannot check connection logic without motion state
            return

        correct_motion_state = self._is_dry_run or self._motion_state.current_motion_task == "joint_bridge"

        if not self._is_connected:
            all_msgs_arrived = all(self._msg_flags)            
            if all_msgs_arrived and correct_motion_state:
                self._is_connected = True
                self._last_connection_time = self._clock.now()

        if self._is_connected:
            time_diff = (self._clock.now() - self._last_connection_time).nanoseconds / 1e9
            if time_diff > 0.5 or not correct_motion_state:
                # Lost connection
                self._is_connected = False
                self._msg_flags = [False] * len(self._msg_flags)

    def _imu_info_callback(self, msg: ImuInfo):
        self._msg_flags[self._imu_idx] = True
        self._imu = msg

    def _joint_state_callback(self, msg: JointState):
        self._msg_flags[self._joint_state_idx] = True
        self._joint_state = msg
        # Update heartbeat on joint state arrival
        self._last_connection_time = self._clock.now()

    def _motion_state_callback(self, msg: MotionState):
        self._msg_flags[self._motion_state_idx] = True
        self._motion_state = msg

    def _gamepad_keys_callback(self, msg: GamepadKeys):
        self._msg_flags[self._gamepad_idx] = True
        self._gamepad = msg

    def _skin_info_callback(self, msg: SkinPatchInfo, name: str, idx: int):
        self._msg_flags[idx] = True
        self._skin_infos[name] = msg

    def _skin_features_callback(self, msg: SkinFeature, name: str, idx: int):
        self._msg_flags[idx] = True
        self._skin_features[name] = msg

    def _ft_sensor_callback(self, msg: WrenchStamped, name: str, idx: int):
        # Note: only saving ft sensor
        self._msg_flags[idx] = True
        self._ft_sensors[name] = msg.wrench
        
    def _odom_sensor_callback(self, msg: Odometry, name: str, idx: int):
        # Note: only saving the pose and twist
        self._msg_flags[idx] = True
        self._odom_sensors[name] = msg
        
    def _ray_caster_sensor_callback(self, msg: RayCasterData, name: str, idx: int):
        self._msg_flags[idx] = True
        self._ray_caster_sensors[name] = msg