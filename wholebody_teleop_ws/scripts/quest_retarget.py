"""Quest upper-body retargeting; ROS imports are confined to main()."""
import argparse
import logging
import math
from pathlib import Path
import socket
import time

import mujoco
import numpy as np
import yaml

from quest_body_profile import BodyProfile
from quest_tracking import body_features
from quest_calibration import CalibrationSession
from quest_orientation import WristOrientation, DEFAULT_WRIST_OFFSETS_DEG


class Retargeter:
    def __init__(self, config, hand_height_bias=None, body_profile=None, wrist_offsets_deg=DEFAULT_WRIST_OFFSETS_DEG):
        self.profile = BodyProfile.load(body_profile) if body_profile is not None else None
        if hand_height_bias is None:
            hand_height_bias = 0. if self.profile else .15
        if not math.isfinite(hand_height_bias) or not -.5 <= hand_height_bias <= 0.5:
            raise ValueError("hand_height_bias must be finite and between -0.5 and 0.5")
        self.hand_height_bias = hand_height_bias
        path = Path(config)
        cfg = yaml.safe_load(path.read_text())
        self.reference_topic = cfg.get('quest_reference_topic')
        self.joints = cfg['robots'][0]['joints']
        self.model = mujoco.MjModel.from_xml_path(str((path.parent / cfg['ik_scene']).resolve()))
        self.data = mujoco.MjData(self.model)
        self.ids = np.array([self.model.joint(n).id for n in self.joints['names']])
        self.qadr = self.model.jnt_qposadr[self.ids]
        self.dadr = self.model.jnt_dofadr[self.ids]
        self.q = np.array(self.joints['default_position'], dtype=float)
        self.data.qpos[self.qadr] = self.q
        mujoco.mj_forward(self.model, self.data)
        self.arms = []
        self.hand_local = []
        for side, human in [('l', (10, 11, 19)), ('r', (15, 16, 45))]:
            indices = [self.joints['names'].index(f'{joint}_{side}_joint') for joint in
                       ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_pitch')]
            bodies = [self.model.body(f'{joint}_{side}_link').id for joint in
                      ('shoulder_roll', 'elbow_pitch', 'elbow_yaw')]
            local = (self.profile.arms[len(self.arms)].robot_hand_local_m.copy()
                     if self.profile else np.array([.03, 0., -.14]))
            self.hand_local.append(local)
            shoulder, elbow, wrist = self.data.xpos[bodies].copy()
            wrist += self.data.xmat[bodies[2]].reshape(3, 3) @ local
            upper_len, lower_len = np.linalg.norm(elbow-shoulder), np.linalg.norm(wrist-elbow)
            if min(upper_len, lower_len) < .05:
                raise ValueError('Robot shoulder/elbow/hand reference points must be separated by >= 0.05 m')
            self.arms.append((human, indices, bodies, upper_len, lower_len))
        self.head = self.joints['names'].index('head_yaw_joint')
        self.head_pitch = (self.joints['names'].index('sim_head_pitch_joint')
                           if 'sim_head_pitch_joint' in self.joints['names'] else None)
        self.filtered_pitch = None
        self.wrists = WristOrientation(self.model, self.joints['names'], self.ids, self.dadr, wrist_offsets_deg)
        self.rest = self.q.copy()
        self.filtered_directions = None
        self.filtered_head = None
        self.filtered_positions = None
        self.position_errors_m = None

    def solve(self, poses, dt, head_pose=None):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('dt must be finite and nonnegative')
        absolute_z = self.profile is not None and self.profile.calibration is not None
        basis, segments, shoulder_heights, _ = body_features(poses, gravity_aligned=absolute_z)
        forward, left, up = basis
        directions, positions = [], []
        for arm_index, (human, indices, bodies, upper_len, lower_len) in enumerate(self.arms):
            upper, lower = segments[arm_index]
            upper_direction = upper / np.linalg.norm(upper)
            lower_direction = lower / np.linalg.norm(lower)
            # Dimensionless lift in torso space, after human arm normalization.
            # This preserves segment length and leaves vertical arms vertical.
            lower_direction[2] += self.hand_height_bias
            lower_direction /= np.linalg.norm(lower_direction)
            directions.extend([upper_direction, lower_direction])
            if self.profile:
                positions.append(self.profile.targets(
                    arm_index, upper, lower_direction*np.linalg.norm(lower),
                    (upper_len, lower_len), shoulder_heights[arm_index]))
        # Filter target directions in torso coordinates, not raw world positions.
        alpha = 1. - math.exp(-min(max(dt, 0.), .05) / .08)
        directions = np.array(directions)
        if self.filtered_directions is None:
            self.filtered_directions = directions.copy()
        else:
            self.filtered_directions += alpha * (directions-self.filtered_directions)
        if self.profile:
            positions = np.asarray(positions)
            if self.filtered_positions is None:
                self.filtered_positions = positions.copy()
            else:
                self.filtered_positions += alpha*(positions-self.filtered_positions)
        previous = self.q.copy()
        # Set original wrist rotation first so arm IK accounts for the moving endpoint.
        self.wrists.solve(poses, basis, self.q, self.qadr, self.data, alpha)
        for arm_index, (_, indices, bodies, upper_len, lower_len) in enumerate(self.arms):
            dofs = self.dadr[indices]
            for _ in range(12):
                mujoco.mj_forward(self.model, self.data)
                # The shoulder roll pivot moves with
                # shoulder pitch; subtract its Jacobian rather than pinning it.
                # The forearm endpoint is beyond the elbow-yaw pivot.
                points = [self.data.xpos[bodies[0]], self.data.xpos[bodies[1]],
                          self.data.xpos[bodies[2]] +
                          self.data.xmat[bodies[2]].reshape(3, 3) @ self.hand_local[arm_index]]
                jacs = []
                for body, point in zip(bodies, points):
                    jac = np.zeros((3, self.model.nv))
                    mujoco.mj_jac(self.model, self.data, jac, None, point, body)
                    jacs.append(jac[:, dofs])
                errors, jacobians = [], []
                if self.profile:
                    # Shoulder-relative position IK. Normalize residuals by a
                    # fixed robot reach, never by the current target magnitude.
                    reach = upper_len + lower_len
                    for k, weight in enumerate((self.profile.elbow_weight,
                                                self.profile.hand_weight)):
                        scale = math.sqrt(weight)/reach
                        current = points[k+1]-points[0]
                        relative_jac = jacs[k+1]-jacs[0]
                        if absolute_z:
                            current[2] = points[k+1][2]
                            relative_jac[2] = jacs[k+1][2]
                        errors.extend(scale*(self.filtered_positions[arm_index, k]-current))
                        jacobians.append(scale*relative_jac)
                else:
                    for k in range(2):
                        segment = points[k+1]-points[k]
                        length = np.linalg.norm(segment)
                        direction = segment/length
                        target = self.filtered_directions[2*arm_index+k]
                        target = target/max(np.linalg.norm(target), 1e-6)
                        errors.extend(target-direction)
                        jacobians.append((np.eye(3)-np.outer(direction,direction)) @
                                         (jacs[k+1]-jacs[k])/length)
                jac = np.vstack(jacobians)
                # A small rest-pose preference removes twist ambiguity when
                # the arm is straight; the previous pose keeps IK continuous.
                regularizer = .015
                delta = np.linalg.solve(jac.T @ jac + .04*np.eye(4),
                    jac.T @ errors + regularizer*(self.rest[indices]-self.q[indices]))
                self.q[indices] += np.clip(delta, -.10, .10)
                limits = self.model.jnt_range[self.ids[indices]]
                self.q[indices] = np.clip(self.q[indices], limits[:, 0], limits[:, 1])
                self.data.qpos[self.qadr] = self.q
        # Camera Head pose uses local +Z; Meta Body_Head is a bone frame
        # whose local +Y is forward. These are not interchangeable frames.
        quat = poses[7, 3:7] if head_pose is None else np.asarray(head_pose)[3:7]
        norm = np.linalg.norm(quat)
        if np.isfinite(quat).all() and norm > .1:
            x, y, z, w = quat / norm
            if head_pose is None:
                gaze = np.array([2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)])
            else:
                gaze = np.array([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])
            lateral, frontal = np.dot(gaze, left), np.dot(gaze, forward)
            if self.head_pitch is not None:
                pitch = -math.atan2(np.dot(gaze, up), math.hypot(lateral, frontal))
                low, high = self.model.jnt_range[self.ids[self.head_pitch]]
                pitch = np.clip(pitch, low, high)
                if self.filtered_pitch is None:
                    self.filtered_pitch = previous[self.head_pitch]
                self.filtered_pitch += alpha*(pitch-self.filtered_pitch)
                self.q[self.head_pitch] = self.filtered_pitch
            # Looking almost vertically has no well-defined yaw: hold it.
            if math.hypot(lateral, frontal) > .25:
                yaw = math.atan2(lateral, frontal)
                lower, upper = self.model.jnt_range[self.ids[self.head]]
                yaw = np.clip(yaw, lower, upper)
                if self.filtered_head is None:
                    self.filtered_head = previous[self.head]
                self.filtered_head += alpha*(yaw-self.filtered_head)
                self.q[self.head] = self.filtered_head
        limits = self.model.jnt_range[self.ids]
        self.q = np.clip(self.q, limits[:, 0], limits[:, 1])
        # Maximum commanded joint speed: 1.5 rad/s, including tracking recovery.
        step = 1.5 * min(max(dt, 0.), .05)
        self.q = previous + np.clip(self.q-previous, -step, step)
        self.data.qpos[self.qadr] = self.q
        mujoco.mj_forward(self.model, self.data)
        if self.profile:
            self.position_errors_m = []
            for i, (_, _, bodies, _, _) in enumerate(self.arms):
                shoulder, elbow, hand = self.data.xpos[bodies].copy()
                hand += self.data.xmat[bodies[2]].reshape(3, 3) @ self.hand_local[i]
                current = np.array([elbow-shoulder, hand-shoulder])
                if absolute_z:
                    current[:, 2] = [elbow[2], hand[2]]
                self.position_errors_m.append(np.linalg.norm(
                    self.filtered_positions[i]-current, axis=1))
            self.position_errors_m = np.asarray(self.position_errors_m)
        return self.q.copy()


def parse_pose(line):
    label, sep, payload = line.partition(':')
    if not sep or label.split('|', 1)[0].strip().lower() != 'body joints_pose':
        return None
    fields = payload.strip().lstrip(',').split(',')
    try:
        values = np.array([float(v) for v in fields])
    except ValueError:
        return None
    if values.size != 490 or not np.isfinite(values).all():
        return None
    return values.reshape(70, 7)


def parse_head(line):
    label, sep, payload = line.partition(':')
    if not sep or label.split('|', 1)[0].strip().lower() != 'head pose':
        return None
    try:
        values = np.array([float(v) for v in payload.strip().lstrip(',').split(',')])
    except ValueError:
        return None
    if values.shape != (7,) or not np.isfinite(values).all() or np.linalg.norm(values[3:]) < .1:
        return None
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--config', required=True)
    parser.add_argument('--port', type=int, default=9010, help='Forwarded UDP pose port')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--body-profile', help='Personal calibration YAML; enables position IK')
    mode.add_argument('--calibrate', metavar='OUTPUT_YAML',
                      help='Robot-led four-pose calibration, then start tracking with saved profile')
    parser.add_argument('--overwrite-calibration', action='store_true',
                        help='Explicitly replace an existing calibration output')
    parser.add_argument('--tracking-ready-file', help=argparse.SUPPRESS)
    parser.add_argument('--check-profile', action='store_true',
                        help='Validate configuration and print model lengths without starting ROS')
    parser.add_argument('--hand-height-bias', type=float, default=None,
                        help='Torso-up direction bias [-0.5, 0.5]; default 0 with profile, otherwise 0.15')
    parser.add_argument('--wrist-offset-deg', nargs=2, type=float, default=DEFAULT_WRIST_OFFSETS_DEG,
                        metavar=('LEFT', 'RIGHT'),
                        help='Original wrist joint zero corrections in degrees, left then right; default 90 -90; +/-150')
    args = parser.parse_args()
    if args.overwrite_calibration and not args.calibrate:
        parser.error('--overwrite-calibration requires --calibrate')
    if args.calibrate and args.hand_height_bias not in (None, 0.):
        parser.error('Calibration requires --hand-height-bias 0 (or omit it)')
    try:
        solver = Retargeter(args.config, hand_height_bias=0. if args.calibrate else args.hand_height_bias,
                            body_profile=args.body_profile, wrist_offsets_deg=args.wrist_offset_deg)
        if solver.reference_topic and args.calibrate:
            raise ValueError('Run robot-led calibration in fixed-base mode, then use --body-profile in policy mode')
        calibration = (CalibrationSession(solver, args.calibrate, args.overwrite_calibration)
                       if args.calibrate else None)
    except (ValueError, OSError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    logging.warning('Arm mapping: %s; hand height bias %.3f',
                    'robot-led calibration' if calibration else (
                        'personal position calibration' if solver.profile else 'segment directions'),
                    solver.hand_height_bias)
    for i, (_, _, _, upper, lower) in enumerate(solver.arms):
        logging.warning('%s robot upper %.4f m, forearm to configured endpoint %.4f m',
                        ('Left', 'Right')[i], upper, lower)
        if solver.profile:
            arm = solver.profile.arms[i]
            logging.warning('Human upper %.4f m, forearm %.4f m; effective scales %.3f / %.3f',
                            arm.upper_arm_m, arm.forearm_m,
                            upper/arm.upper_arm_m*arm.upper_gain,
                            lower/arm.forearm_m*arm.forearm_gain)
    if solver.profile and solver.profile.calibration is not None:
        logging.warning('Fixed standing-height reference active; preserve the Quest tracking origin. '
                        'Recalibrate after recentering. Vertical shoulder gain %.3f',
                        solver.profile.calibration['vertical_scale'])
    logging.warning('Wrist zero corrections (left/right): %.1f / %.1f degrees', *args.wrist_offset_deg)
    if solver.head_pitch is not None:
        logging.warning('Original elbow_yaw wrist rotation active; head pitch and palm flexion are simulation extensions. '
                        'Keep wrists straight on the first tracking frame (orientation zero).')
    if args.check_profile:
        return
    import rclpy
    from interface_protocol.msg import JointCommand, JointState
    from sensor_msgs.msg import JointState as NamedJointState
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('quest_sim_retarget')
    reference_only = solver.reference_topic is not None
    topic = solver.reference_topic if reference_only else solver.joints['command_topic']
    if not topic.startswith('/simulation/') or not solver.joints['state_topic'].startswith('/simulation/'):
        raise ValueError('This bridge requires an isolated /simulation/ command topic')
    pub = node.create_publisher(NamedJointState if reference_only else JointCommand, topic,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    if reference_only:
        # Keep this module independent of ONNX Runtime in fixed-base mode.
        upper_names = [n for n in solver.joints['names'] if n.startswith(
            ('shoulder_', 'elbow_', 'head_', 'sim_head_', 'sim_wrist_'))]
        upper_ids = [solver.joints['names'].index(n) for n in upper_names]
        logging.warning('Policy mode: publishing named upper-body references only to %s', topic)
    feedback, feedback_time = None, -math.inf
    def on_state(msg):
        nonlocal feedback, feedback_time
        q = np.asarray(msg.position, dtype=float)
        if q.shape == solver.q.shape and np.isfinite(q).all():
            feedback, feedback_time = q, time.monotonic()
    if calibration:
        node.create_subscription(JointState, solver.joints['state_topic'], on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    if calibration:
        logging.warning('机器人引导标定：请在 MuJoCo 全身观察窗口查看示范。'
                        '共 4 个采样姿态，采样后直接使用，不做姿态匹配或误差验收。'
                        '期间不跟随真人；追踪或反馈中断会暂停。')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', args.port))
    sock.setblocking(False)
    last_valid = None
    previous_solve = time.monotonic()
    previous_control = previous_solve
    stale = True
    last_error_log = -math.inf
    head_pose, head_time = None, -math.inf
    logging.warning('Waiting for forwarded Body joints_pose on 127.0.0.1:%d', args.port)
    try:
        while rclpy.ok():
            start = time.monotonic()
            latest = None
            solved = False
            # Bound draining so incoming traffic cannot starve control.
            for _ in range(256):
                try:
                    packet = sock.recv(65535).decode('utf-8', errors='replace')
                except BlockingIOError:
                    break
                for line in packet.splitlines():
                    head = parse_head(line)
                    if head is not None:
                        head_pose, head_time = head, start
                    pose = parse_pose(line)
                    if pose is not None:
                        latest = pose
            calibrating = calibration is not None and not calibration.done
            if calibrating:
                calibration.step(start, start-previous_control, latest, feedback, feedback_time)
                if calibration.done and args.tracking_ready_file:
                    Path(args.tracking_ready_file).write_text('tracking ready\n')
                previous_solve = start
            previous_control = start
            if latest is not None and not calibrating:
                try:
                    solver.solve(latest, start-previous_solve,
                                 head_pose if start-head_time < .15 else None)
                    if solver.profile and start-last_error_log >= 5.:
                        logging.warning('Commanded-pose IK residual cm (elbow, hand): left %s; right %s',
                                        np.round(100*solver.position_errors_m[0], 1),
                                        np.round(100*solver.position_errors_m[1], 1))
                        last_error_log = start
                    previous_solve = start
                    last_valid = start
                    solved = True
                    if stale:
                        logging.warning('Tracking active: arms and head following')
                    stale = False
                except ValueError as exc:
                    logging.debug('Rejected tracking: %s', exc)
            if last_valid is not None and start-last_valid > .3 and not stale:
                logging.warning('Tracking stale: holding last commanded pose')
                stale = True
            if reference_only:
                # Only fresh, successfully solved tracking produces a reference.
                # Re-publishing held values would conceal a Quest disconnect.
                if solved:
                    msg = NamedJointState()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.name = upper_names
                    msg.position = solver.q[upper_ids].tolist()
                    pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.)
                time.sleep(max(0., 1/60-(time.monotonic()-start)))
                continue
            msg = JointCommand()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.position = solver.q.tolist()
            msg.velocity = [0.] * len(solver.q)
            msg.torque = [0.] * len(solver.q)
            msg.feed_forward_torque = [0.] * len(solver.q)
            msg.stiffness = [float(x) for x in solver.joints['default_stiffness']]
            msg.damping = [float(x) for x in solver.joints['default_damping']]
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.)
            time.sleep(max(0., 1/60-(time.monotonic()-start)))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
