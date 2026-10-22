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


class Retargeter:
    def __init__(self, config, hand_height_bias=.15):
        if not math.isfinite(hand_height_bias) or not -.5 <= hand_height_bias <= 1:
            raise ValueError("hand_height_bias must be finite and between -0.5 and 0.5")
        self.hand_height_bias = hand_height_bias
        path = Path(config)
        cfg = yaml.safe_load(path.read_text())
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
        for side, human in [('l', (10, 11, 19)), ('r', (15, 16, 45))]:
            indices = [self.joints['names'].index(f'{joint}_{side}_joint') for joint in
                       ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_pitch')]
            bodies = [self.model.body(f'{joint}_{side}_link').id for joint in
                      ('shoulder_roll', 'elbow_pitch', 'elbow_yaw')]
            shoulder, elbow, wrist = self.data.xpos[bodies].copy()
            self.arms.append((human, indices, bodies, np.linalg.norm(elbow-shoulder),
                              np.linalg.norm(wrist-elbow)))
        self.head = self.joints['names'].index('head_yaw_joint')
        self.rest = self.q.copy()
        self.filtered_directions = None
        self.filtered_head = None

    def solve(self, poses, dt, head_pose=None):
        if poses.shape != (70, 7) or not np.isfinite(poses).all():
            raise ValueError('Expected 70 finite joint poses')
        p = poses[:, :3]
        left = p[10] - p[15]
        up = p[5] - p[1]
        if min(np.linalg.norm(left), np.linalg.norm(up)) < .05:
            raise ValueError('Degenerate torso tracking')
        up /= np.linalg.norm(up)
        left -= up * np.dot(left, up)
        if np.linalg.norm(left) < .05:
            raise ValueError('Degenerate shoulder tracking')
        left /= np.linalg.norm(left)
        forward = np.cross(up, left)  # Unity is left handed.
        basis = np.array([forward, left, up])
        directions = []
        for human, indices, bodies, upper_len, lower_len in self.arms:
            shoulder, elbow, wrist = p[list(human)]
            upper, lower = elbow-shoulder, wrist-elbow
            if min(np.linalg.norm(upper), np.linalg.norm(lower)) < .05:
                raise ValueError('Degenerate arm tracking')
            upper_direction = basis @ upper / np.linalg.norm(upper)
            lower_direction = basis @ lower / np.linalg.norm(lower)
            # Dimensionless lift in torso space, after human arm normalization.
            # This preserves segment length and leaves vertical arms vertical.
            lower_direction[2] += self.hand_height_bias
            lower_direction /= np.linalg.norm(lower_direction)
            directions.extend([upper_direction, lower_direction])
        # Filter target directions in torso coordinates, not raw world positions.
        alpha = 1. - math.exp(-min(max(dt, 0.), .05) / .08)
        directions = np.array(directions)
        if self.filtered_directions is None:
            self.filtered_directions = directions.copy()
        else:
            self.filtered_directions += alpha * (directions-self.filtered_directions)
        previous = self.q.copy()
        for arm_index, (_, indices, bodies, _, _) in enumerate(self.arms):
            dofs = self.dadr[indices]
            for _ in range(12):
                mujoco.mj_forward(self.model, self.data)
                # Use segment directions. The shoulder roll pivot moves with
                # shoulder pitch; subtract its Jacobian rather than pinning it.
                # The forearm endpoint is beyond the elbow-yaw pivot.
                points = [self.data.xpos[bodies[0]], self.data.xpos[bodies[1]],
                          self.data.xpos[bodies[2]] +
                          self.data.xmat[bodies[2]].reshape(3, 3) @ np.array([.03, 0., -.14])]
                jacs = []
                for body, point in zip(bodies, points):
                    jac = np.zeros((3, self.model.nv))
                    mujoco.mj_jac(self.model, self.data, jac, None, point, body)
                    jacs.append(jac[:, dofs])
                errors, jacobians = [], []
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--port', type=int, default=9010, help='Forwarded UDP pose port')
    parser.add_argument('--hand-height-bias', type=float, default=.15,
                        help='Torso-up forearm direction bias [-0.5, 0.5]; default 0.15, 0 disables')
    args = parser.parse_args()
    if not math.isfinite(args.hand_height_bias) or not -.5 <= args.hand_height_bias <= 1:
        parser.error('--hand-height-bias must be between -0.5 and 0.5')
    import rclpy
    from interface_protocol.msg import JointCommand
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('quest_sim_retarget')
    solver = Retargeter(args.config, hand_height_bias=args.hand_height_bias)
    topic = solver.joints['command_topic']
    if not topic.startswith('/simulation/'):
        raise ValueError('This bridge requires an isolated /simulation/ command topic')
    pub = node.create_publisher(JointCommand, topic,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', args.port))
    sock.setblocking(False)
    last_valid = None
    previous_solve = time.monotonic()
    stale = True
    head_pose, head_time = None, -math.inf
    logging.warning('Waiting for forwarded Body joints_pose on 127.0.0.1:%d', args.port)
    try:
        while rclpy.ok():
            start = time.monotonic()
            latest = None
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
            if latest is not None:
                try:
                    solver.solve(latest, start-previous_solve,
                                 head_pose if start-head_time < .15 else None)
                    previous_solve = start
                    last_valid = start
                    if stale:
                        logging.warning('Tracking active: arms and head following')
                    stale = False
                except ValueError as exc:
                    logging.debug('Rejected tracking: %s', exc)
            if last_valid is not None and start-last_valid > .3 and not stale:
                logging.warning('Tracking stale: holding last commanded pose')
                stale = True
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
