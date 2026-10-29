"""Simulation-only Walk/Run/WBC inference with named Quest upper-body references."""
import ast
from pathlib import Path

import numpy as np
import onnxruntime as ort


DEFAULT_WBC_MODEL = (Path(__file__).resolve().parents[1] / 'models' /
                     'pm01_wbc' / '20260721_211647.onnx')
UPPER_NAMES = tuple(
    f'{joint}_{side}_joint' for side in ('l', 'r') for joint in
    ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_pitch', 'elbow_yaw')
) + ('head_yaw_joint', 'sim_head_pitch_joint',
     'sim_wrist_flex_l_joint', 'sim_wrist_flex_r_joint')


def parse_metadata(value):
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError):
        return value.split(',') if ',' in value else value


def vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{label}: expected {size} finite values')
    return result


class QuestPolicy:
    """The actor keeps its own raw last_action, as in the existing deployment.

    WBC receives Quest through arm_delta and retains learned residual actions.
    Walk/Run retain their velocity observations and replace upper-body targets
    after inference. The three simulation extensions are always driven directly.
    """
    def __init__(self, joints, model_path=DEFAULT_WBC_MODEL, mode="wbc"):
        if mode not in ("walk", "run", "wbc"):
            raise ValueError(f"Unknown policy mode: {mode}")
        if Path(model_path).suffix.lower() != '.onnx':
            raise ValueError('Quest policy mode currently supports ONNX exports only, not JIT checkpoints')
        self.mode = mode
        self.is_wbc = mode == "wbc"
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model_path), options,
                                            providers=['CPUExecutionProvider'])
        self.meta = {k: parse_metadata(v) for k, v in
                     self.session.get_modelmeta().custom_metadata_map.items()}
        m = self.meta
        command_types = m.get('command_types', {})
        action_types = m.get('action_types', {})
        expected_action = 'ULCResidualJointPositionActionCfg' if self.is_wbc else 'JointPositionActionCfg'
        if action_types and list(action_types.values()) != [expected_action]:
            raise ValueError(f'{mode}: incompatible action type {action_types}')
        allowed = ('ULCCommandCfg',) if self.is_wbc else ('UniformVelocityCommandCfg', 'HullVelocityCommandCfg')
        if command_types and next(iter(command_types.values())) not in allowed:
            raise ValueError(f'{mode}: incompatible command type {command_types}')
        self.names = list(joints['names'])
        self.policy_names = list(m['joint_names'])
        self.action_names = list(m.get('action_joint_names', self.policy_names))
        if len(self.names) != len(set(self.names)) or len(set(self.policy_names)) != 24:
            raise ValueError('Expected unique scene names and 24 unique policy joints')
        if set(self.action_names) != set(self.policy_names) or len(self.action_names) != 24:
            raise ValueError('Action joints must cover all 24 policy joints exactly once')
        self.mapping = np.array([self.names.index(n) for n in self.policy_names])
        self.action_mapping = np.array([self.names.index(n) for n in self.action_names])
        self.upper_names = tuple(n for n in UPPER_NAMES if n in self.names)
        self.upper_ids = np.array([self.names.index(n) for n in self.upper_names])
        self.arm_names = list(m['arm_joint_names']) if self.is_wbc else []
        self.arm_ids = np.array([self.policy_names.index(n) for n in self.arm_names], dtype=int)
        self.arm_ranges = np.array([m['arm_joint_ranges'][n] for n in self.arm_names]).reshape(-1, 2)
        self.extra_names = tuple(n for n in self.upper_names if n not in self.policy_names)
        self.default = vector(m['default_joint_pos'], 24, 'default_joint_pos')
        action_order = [self.policy_names.index(n) for n in self.action_names]
        self.offset = vector(m.get('action_offset', self.default[action_order]), 24, 'action_offset')
        self.scale = np.broadcast_to(np.asarray(m['action_scale'], dtype=float), (24,)).copy()
        if not np.isfinite(self.scale).all() or np.any(self.scale <= 0):
            raise ValueError('Invalid action_scale')
        self.robot_default = vector(joints['default_position'], len(self.names), 'scene default').copy()
        self.robot_default[self.mapping] = self.default
        self.kp = vector(joints['default_stiffness'], len(self.names), 'scene stiffness').copy()
        self.kd = vector(joints['default_damping'], len(self.names), 'scene damping').copy()
        self.kp[self.mapping] = vector(m['joint_stiffness'], 24, 'policy stiffness')
        self.kd[self.mapping] = vector(m['joint_damping'], 24, 'policy damping')
        self.dt = float(m['dt']) * int(m['decimation'])
        if not np.isclose(self.dt, .02):
            raise ValueError('Quest policy expects a 50 Hz policy')
        self.obs_names = list(m['observation_names'])
        expected = ['base_ang_vel', 'projected_gravity', 'joint_pos_rel',
                    'joint_vel_rel', 'last_action', 'loco_command', 'arm_delta']
        if not self.is_wbc:
            expected = ['base_ang_vel', 'projected_gravity', 'commands',
                        'joint_pos_rel', 'joint_vel_rel', 'last_action']
        term_count = len(expected)
        if self.obs_names != expected:
            raise ValueError(f'Unsupported policy observations: {self.obs_names}')
        self.obs_scales = vector(m['observation_scales'], term_count, 'observation_scales')
        self.obs_clips = vector(m['observation_clips'], term_count, 'observation_clips')
        history = int(m['observation_history_length'])
        lengths = ([history] * term_count if history != 1 else
                   [int(h) for h in m['observation_history_lengths']])
        if len(lengths) != term_count or min(lengths) < 1:
            raise ValueError('Invalid observation history')
        dims = (3, 3, 24, 24, 24, 7, 24) if self.is_wbc else (3, 3, 3, 24, 24, 24)
        self.buffers = [np.zeros((h, d)) for h, d in zip(lengths, dims)]
        self.input = self.session.get_inputs()[0]
        self.obs_dim = sum(b.size for b in self.buffers)
        if self.input.shape != [1, self.obs_dim] or self.session.get_outputs()[0].shape != [1, 24]:
            raise ValueError('ONNX shape does not match metadata')
        self.hull = not self.is_wbc and 'vy_bound_points' in m
        if self.hull:
            self.vy_points = np.asarray(m['vy_bound_points'], dtype=float)
            self.yaw_points = np.asarray(m['omega_bound_points'], dtype=float)
            self.command_bounds = np.array([
                [max(self.vy_points[0, 0], self.yaw_points[0, 0]),
                 min(self.vy_points[-1, 0], self.yaw_points[-1, 0])],
                [-np.max(self.vy_points[:, 1]), np.max(self.vy_points[:, 1])],
                [-np.max(self.yaw_points[:, 1]), np.max(self.yaw_points[:, 1])]])
        else:
            keys = (('range_lin_vel_x', 'range_lin_vel_y', 'range_ang_vel_z') if self.is_wbc
                    else ('cmd_range_vx', 'cmd_range_vy', 'cmd_range_omega'))
            if not all(k in m for k in keys):
                raise ValueError(f'{mode}: model is missing velocity bounds {keys}; choose a compatible export')
            self.command_bounds = np.array([m[k] for k in keys], dtype=float)
        if self.command_bounds.shape != (3, 2) or not np.isfinite(self.command_bounds).all():
            raise ValueError('Invalid command bounds')
        self.reset()

    def reset(self):
        self.last_action = np.zeros(24)
        self.upper_target = self.robot_default[self.upper_ids].copy()
        self.arm_delta = np.zeros(24)
        self.loco_command = np.array([0., 0., 0., float(self.meta.get('default_height', .8)), 0., 0., 0.])
        self.initialized = False
        for buf in self.buffers:
            buf.fill(0.)

    def observation(self, position, velocity, quaternion_xyzw, angular_velocity, command):
        q = vector(position, len(self.names), 'joint position')[self.mapping]
        dq = vector(velocity, len(self.names), 'joint velocity')[self.mapping]
        quat = vector(quaternion_xyzw, 4, 'IMU quaternion')
        norm = np.linalg.norm(quat)
        if norm < .1:
            raise ValueError('Invalid IMU quaternion norm')
        x, y, z, w = quat / norm
        gravity = np.array([2*(w*y-x*z), -2*(y*z+w*x), 2*(x*x+y*y)-1])
        cmd = np.clip(vector(command, 3, 'velocity command'),
                      self.command_bounds[:, 0], self.command_bounds[:, 1])
        if self.hull:
            vy = np.interp(cmd[0], self.vy_points[:, 0], self.vy_points[:, 1])
            wz = np.interp(cmd[0], self.yaw_points[:, 0], self.yaw_points[:, 1])
            cmd[1:] = np.clip(cmd[1:], [-vy, -wz], [vy, wz])
        self.loco_command[:3] = cmd
        values = (vector(angular_velocity, 3, 'angular velocity'), gravity,
                  q-self.default, dq, self.last_action, self.loco_command, self.arm_delta)
        if not self.is_wbc:
            values = (vector(angular_velocity, 3, 'angular velocity'), gravity, cmd,
                      q-self.default, dq, self.last_action)
        for buf, value, scale, clip in zip(self.buffers, values, self.obs_scales, self.obs_clips):
            value = scale * np.clip(value, -clip, clip)
            if not self.initialized:
                buf[:] = value
            else:
                buf[:-1] = buf[1:]
                buf[-1] = value
        self.initialized = True
        return np.concatenate([b.ravel() for b in self.buffers]).astype(np.float32)

    def step(self, position, velocity, quaternion_xyzw, angular_velocity, command,
             reference=None):
        if reference is not None:
            desired = vector(reference, len(self.upper_ids), 'Quest upper reference').copy()
            for name, bounds in zip(self.arm_names, self.arm_ranges):
                if name in self.upper_names:
                    i = self.upper_names.index(name)
                    desired[i] = np.clip(desired[i], *bounds)
            # Bound takeover and recovery before feeding the reference to WBC.
            self.upper_target += np.clip(desired-self.upper_target, -1.5*self.dt, 1.5*self.dt)
        arm_target = self.default.copy()
        for name in self.arm_names:
            if name in self.upper_names:
                arm_target[self.policy_names.index(name)] = self.upper_target[self.upper_names.index(name)]
        arm_target[self.arm_ids] = np.clip(arm_target[self.arm_ids],
                                          self.arm_ranges[:, 0], self.arm_ranges[:, 1])
        self.arm_delta = arm_target-self.default
        obs = self.observation(position, velocity, quaternion_xyzw, angular_velocity, command)
        action = self.session.run(None, {self.input.name: obs[None]})[0][0]
        self.last_action = vector(action, 24, 'policy action').copy()
        target = self.robot_default.copy()
        delta = self.arm_delta[[self.policy_names.index(n) for n in self.action_names]]
        target[self.action_mapping] = self.offset + self.scale * self.last_action + delta
        direct_names = self.extra_names if self.is_wbc else self.upper_names
        for name in direct_names:
            target[self.names.index(name)] = self.upper_target[self.upper_names.index(name)]
        return target


class WBCPolicy(QuestPolicy):
    """Preserve the original WBC Python entry point."""
    def __init__(self, joints, model_path=DEFAULT_WBC_MODEL):
        super().__init__(joints, model_path, mode='wbc')


def main():
    import argparse
    import time
    import mujoco
    import yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--velocity', type=float, nargs=3, default=(0., 0., 0.))
    args = parser.parse_args()
    requested_velocity = vector(args.velocity, 3, 'requested velocity')
    cfg = yaml.safe_load(Path(args.config).read_text())
    robot = cfg['robots'][0]
    joints = robot['joints']
    reference_topic = cfg['quest_reference_topic']
    imu_topic = robot['sensors']['imu_sensors'][0]['topic']
    if not all(t.startswith('/simulation/') for t in
               (joints['command_topic'], joints['state_topic'], reference_topic, imu_topic)):
        parser.error('Policy controller is restricted to /simulation/ topics')
    if reference_topic == joints['command_topic']:
        parser.error('Reference and actuator topics must be different')
    mode = cfg.get('policy_mode', 'wbc')
    policy = QuestPolicy(joints, cfg.get('policy_model', cfg.get('wbc_model')), mode)
    model = mujoco.MjModel.from_xml_path(cfg['ik_scene'])
    if model.neq:
        parser.error('Policy scene must have no equality constraints')
    limits = model.jnt_range[[model.joint(n).id for n in policy.names]].copy()

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from interface_protocol.msg import JointCommand, JointState, ImuInfo
    from sensor_msgs.msg import JointState as NamedJointState
    from rclpy.executors import ExternalShutdownException
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node(f'quest_{mode}_controller')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(JointCommand, joints['command_topic'], qos)
    state = {'q': None, 'dq': None, 'quat': None, 'gyro': None,
             'joint_time': -np.inf, 'imu_time': -np.inf,
             'reference': None, 'reference_time': -np.inf,
             'clock': None, 'twist': None, 'twist_time': -np.inf,
             'fault': False, 'tracking': False, 'started': False}

    def on_joints(msg):
        try:
            q = vector(msg.position, len(policy.names), 'joint position')
            dq = vector(msg.velocity, len(policy.names), 'joint velocity')
        except ValueError as exc:
            node.get_logger().error(str(exc))
            return
        state.update(q=q, dq=dq, joint_time=time.monotonic())

    def on_imu(msg):
        q, g = msg.quaternion, msg.angular_velocity
        quat, gyro = np.array([q.x, q.y, q.z, q.w]), np.array([g.x, g.y, g.z])
        if np.isfinite(quat).all() and np.isfinite(gyro).all() and np.linalg.norm(quat) > .1:
            state.update(quat=quat/np.linalg.norm(quat), gyro=gyro, imu_time=time.monotonic())

    def on_reference(msg):
        try:
            if len(msg.name) != len(set(msg.name)) or set(msg.name) != set(policy.upper_names):
                raise ValueError('Quest reference must name exactly the 14 upper-body joints')
            values = vector(msg.position, len(msg.name), 'upper reference')
            by_name = dict(zip(msg.name, values))
            ref = np.array([by_name[n] for n in policy.upper_names])
            ref = np.clip(ref, limits[policy.upper_ids, 0], limits[policy.upper_ids, 1])
        except ValueError as exc:
            node.get_logger().error(str(exc))
            return
        state.update(reference=ref, reference_time=time.monotonic())

    def on_twist(msg):
        cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if np.isfinite(cmd).all():
            state.update(twist=cmd, twist_time=time.monotonic())

    def on_clock(msg):
        stamp = msg.clock.sec + msg.clock.nanosec*1e-9
        if state['clock'] is not None and stamp < state['clock']:
            policy.reset()
            state.update(q=None, quat=None, reference=None, reference_time=-np.inf,
                         fault=False, started=False, tracking=False, twist=None, twist_time=-np.inf)
            node.get_logger().info('Simulation reset: cleared policy history and Quest reference')
        state['clock'] = stamp

    def publish(target, passive=False):
        msg = JointCommand()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.position = [float(x) for x in target]
        msg.velocity = msg.torque = msg.feed_forward_torque = [0.] * len(policy.names)
        msg.stiffness = ([0.] * len(policy.names) if passive else policy.kp.tolist())
        msg.damping = policy.kd.tolist()
        pub.publish(msg)

    def update():
        now = time.monotonic()
        if state['q'] is None or state['quat'] is None:
            return  # C++ simulator holds its initial state until the first command.
        if min(now-state['joint_time'], now-state['imu_time']) < 0:
            return
        if max(now-state['joint_time'], now-state['imu_time']) > .3:
            if state['started'] and not state['fault']:
                node.get_logger().error('Robot feedback stale: entering passive damping')
                state['fault'] = True
        if state['fault']:
            publish(state['q'], passive=True)
            return
        x, y, z, w = state['quat']
        if 1-2*(x*x+y*y) < np.cos(.9):
            state['fault'] = True
            node.get_logger().error('Robot tilted over 0.9 rad: reset simulation to restart')
            publish(state['q'], passive=True)
            return
        fresh = now-state['reference_time'] <= .3
        if fresh != state['tracking']:
            node.get_logger().info(f'Quest active: {mode.upper()} ' +
                                   ('references' if policy.is_wbc else 'upper-body overrides') if fresh else
                                   'Quest stale: holding upper targets; policy continues with zero velocity')
            state['tracking'] = fresh
        cmd = requested_velocity.copy()
        if state['twist'] is not None:
            cmd = state['twist'] if now-state['twist_time'] <= .3 else np.zeros(3)
        if not fresh:
            cmd = np.zeros(3)
        try:
            target = policy.step(state['q'], state['dq'], state['quat'], state['gyro'], cmd,
                                 state['reference'] if fresh else None)
            target = np.clip(target, limits[:, 0], limits[:, 1])
            publish(target)
            state['started'] = True
        except (ValueError, RuntimeError) as exc:
            state['fault'] = True
            node.get_logger().error(f'{mode.upper()} inference failed: {exc}')
            publish(state['q'], passive=True)

    node.create_subscription(JointState, joints['state_topic'], on_joints, qos)
    node.create_subscription(ImuInfo, imu_topic, on_imu, qos)
    node.create_subscription(NamedJointState, reference_topic, on_reference, qos)
    node.create_subscription(Clock, '/clock', on_clock, qos)
    node.create_subscription(Twist, '/simulation/quest/cmd_vel', on_twist, qos)
    node.create_timer(policy.dt, update)
    node.get_logger().info(f'{mode.upper()} {policy.obs_dim}->24 at {1/policy.dt:.0f} Hz; '
                           f'Quest supplies {len(policy.upper_ids)} named references. '
                           f'Velocity request {requested_velocity.tolist()}; waiting for simulation feedback.')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok() and state['q'] is not None:
            publish(state['q'], passive=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
