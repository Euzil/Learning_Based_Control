import numpy as np

from engineai_deploy.agent.robot_state import RobotState
from engineai_deploy.commands.commands_base import CommandBase

from engineai_deploy.utils.math import quat_rotate_inverse


class HistoryBuffer:
    """ HistoryBuffer
    Append observations in circular buffer
    buffer = [o(t-N-1), ..., o(t-1), o(t)]
    """
    def __init__(self, history_length: int, feature_dim: int):
        self.history_length = history_length
        self.feature_dim = feature_dim
        self.buffer = np.zeros((history_length, feature_dim))
        self.initialized = False

    def reset(self):
        self.buffer[:] = 0.0
        self.initialized = False

    def append(self, obs: np.ndarray):
        """Add new observation and shift old ones left."""
        if not self.initialized:
            self.buffer[:] = obs
            self.initialized = True
        else:
            self.buffer[:-1] = self.buffer[1:]
            self.buffer[-1] = obs

    def get_flattened(self):
        return self.buffer.flatten()


class ObservationManager:
    def __init__(
        self, 
        metadata: dict, 
        state: RobotState, 
        command: CommandBase, 
        max_obs_value: float = 1e6
    ):

        self._state: RobotState = state
        self._command: CommandBase = command
        self._max_obs_value = max_obs_value

        # Read observations settings
        self._obs_clips = metadata["observation_clips"]
        self._obs_scales = metadata["observation_scales"]

        # check group and term history length
        group_history_lengths = int(metadata["observation_history_length"])
        if group_history_lengths != 1:
            observation_history_lengths = [group_history_lengths] * len(self._obs_clips)
        else:
            observation_history_lengths = np.array(metadata["observation_history_lengths"], dtype=int)

        # Default DOF (policy order)
        self._default_dof_pos = np.array(metadata["default_joint_pos"], dtype=float)
        self._default_dof_vel = np.zeros_like(self._default_dof_pos)

        # Action (updated by setter)
        self._action = np.zeros(len(self._default_dof_pos))

        # Setup Observations and observation history buffer
        self._obs_names, self._obs_getters, self._obs_hist_buffers = [], [], []
        for name, history_length in zip(metadata["observation_names"], observation_history_lengths):
            getter_name = f"_get_{name}"
            if hasattr(self, getter_name):
                getter_fnc = getattr(self, getter_name)
                self._obs_getters.append(getter_fnc)
                self._obs_names.append(name)
                self._obs_hist_buffers.append(
                    HistoryBuffer(
                        history_length, 
                        getter_fnc().shape[-1]
                    )
                )
            else:
                raise ValueError(f"[ObservationManager]: No getter found for observation '{name}'.")

        self._obs_dim, obs_dims = self._compute_obs_dim()
        
        print(f"[ObservationManager]: Total dim {self._obs_dim}")
        for obs_name, obs_dim in zip(self._obs_names, obs_dims):
            print(f"- {obs_name} ({obs_dim})")

    # --------------------------------------------------------------------------
    # setters/getters

    def set_last_action(self, action: np.ndarray):
        self._action = action

    @property
    def obs_dim(self) -> int:
        return self._obs_dim  

    # --------------------------------------------------------------------------
    # update

    def reset(self):
        """Clear all buffers back to zero."""
        for buf in self._obs_hist_buffers:
            buf.reset()

    def compute_obs(self) -> np.ndarray:
        """Constructs the observation vector and updates the history buffer."""
        for name, fnc, buf, scale, clip in zip(
                self._obs_names,
                self._obs_getters,
                self._obs_hist_buffers,
                self._obs_scales,
                self._obs_clips
            ):
            value = fnc()
            
            # check inputs
            max_val = np.max(np.abs(value))
            if max_val > self._max_obs_value:
                raise ValueError(
                    f"[ObservationManager]: Invalid observation '{name}' has val={max_val:.4f} > {self._max_obs_value}"
                )

            # process
            processed_obs = scale * np.clip(value, -clip, clip)
            buf.append(processed_obs)

        return np.concatenate([buf.get_flattened() for buf in self._obs_hist_buffers])

    # --------------------------------------------------------------------------
    # base observations

    def _get_base_ang_vel(self):
        """base ang vel from imu"""
        return self._state.sensors.root_ang_vel_b  # 3

    def _get_projected_gravity(self):
        """projected gravity vector into body frame from imu"""
        return quat_rotate_inverse(self._state.sensors.root_quat_w, np.array([0, 0, -1]))  # 3

    def _get_joint_pos_rel(self):
        """relative joint position of robot"""
        return self._state.dof_pos - self._default_dof_pos  # num_dof

    def _get_joint_vel_rel(self):
        """relative joint velocity of robot"""
        return self._state.dof_vel - self._default_dof_vel  # num_dof

    def _get_last_action(self):
        """last relative joint command to the robot"""
        return self._action  # num_dof

    def _get_feet_contact_binary(self):
        """feet contact"""
        # return np.array([
        #     abs(self._state.sensors.left_ft[2]),
        #     abs(self._state.sensors.right_ft[2])
        # ]) > 5.0
        return np.array([1.0, 1.0])                                             # TODO: Bad hack for real robot!


    # --------------------------------------------------------------------------
    # commands
    
    def _get_commands(self):
        """get basic commands"""
        return self._command.command

    def _get_loco_command(self):
        """get ulc loco command"""
        return self._command.loco_command
    
    def _get_arm_delta(self):
        """get ulc arm command"""
        return self._command.arm_delta

    # --------------------------------------------------------------------------
    # skin observations
    
    def _get_left_foot_mean_prox(self):
        """mean proximity for patches on left foot"""
        features = self._state.sensors.skin_features
        return np.array([features["p1e"].avg_prox, features["p2e"].avg_prox])

    def _get_right_foot_mean_prox(self):
        """mean proximity for patches on right foot"""
        features = self._state.sensors.skin_features
        return np.array([features["p3e"].avg_prox, features["p4e"].avg_prox])

    def _get_left_foot_acc(self):
        """acceleration from patches on the left foot"""
        features = self._state.sensors.skin_features
        acc1, acc2 = features["p1e"].link_acc, features["p2e"].link_acc
        return 0.5 * np.array([acc1.x + acc2.x, acc1.y + acc2.y, acc1.z + acc2.z])

    def _get_right_foot_acc(self):
        """acceleration from patches on the right foot"""
        features = self._state.sensors.skin_features
        acc1, acc2 = features["p3e"].link_acc, features["p4e"].link_acc
        return 0.5 * np.array([acc1.x + acc2.x, acc1.y + acc2.y, acc1.z + acc2.z])

    def _get_left_foot_skin(self):
        """proximity sensor readings from patches on the left foot, normalized"""
        features = self._state.sensors.skin_features
        return np.concatenate([features["p1e"].prox_distribution, features["p2e"].prox_distribution])

    def _get_right_foot_skin(self):
        """proximity sensor readings from patches on the right foot, normalized"""
        features = self._state.sensors.skin_features
        return np.concatenate([features["p3e"].prox_distribution, features["p4e"].prox_distribution])

    # --------------------------------------------------------------------------
    # priviliged information 
    
    def _get_height_scan(self):
        """height scan from scan points on the ground"""
        height_scan = np.clip(self._state.sensors.height_scan - 0.5, -1.0, 5.0)
        return height_scan
    
    def _get_base_height(self):
        """height of base above the ground"""
        return np.array([self._state.sensors.odom_pose[2]])
    
    def _get_base_lin_vel(self):
        """base linear velocity in robot base frame"""
        base_lin_vel = self._state.sensors.odom_twist[0:3]
        return base_lin_vel
    
    def _get_feet_contact(self):
        """scaled contact force with the ground"""
        lf = np.linalg.norm(self._state.sensors.left_ft[0:3])
        rf = np.linalg.norm(self._state.sensors.right_ft[0:3])
        return np.array([lf, rf]) * 0.1 * 0.01                                  # TODO fix the scale in the engineai_lab
        
    # --------------------------------------------------------------------------
    # utils

    def _compute_obs_dim(self) -> int:
        total_dim = 0
        obs_dims = []
        for fnc, buf in zip(self._obs_getters, self._obs_hist_buffers):
            value = fnc()
            total_dim += buf.history_length * value.shape[-1]
            obs_dims.append(value.shape[-1])
        return total_dim, obs_dims
