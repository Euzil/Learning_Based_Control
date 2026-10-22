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
    def __init__(self, config):
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
                      ('shoulder_pitch', 'elbow_pitch', 'elbow_yaw')]
            shoulder, elbow, wrist = self.data.xpos[bodies].copy()
            self.arms.append((human, indices, bodies, np.linalg.norm(elbow-shoulder),
                              np.linalg.norm(wrist-elbow)))
        self.head = self.joints['names'].index('head_yaw_joint')

    def solve(self, poses, dt):
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
        targets = []
        for human, indices, bodies, upper_len, lower_len in self.arms:
            shoulder, elbow, wrist = p[list(human)]
            upper, lower = elbow-shoulder, wrist-elbow
            if min(np.linalg.norm(upper), np.linalg.norm(lower)) < .05:
                raise ValueError('Degenerate arm tracking')
            origin = self.data.xpos[bodies[0]].copy()
            elbow_target = origin + basis @ upper / np.linalg.norm(upper) * upper_len
            wrist_target = elbow_target + basis @ lower / np.linalg.norm(lower) * lower_len
            targets.append((indices, bodies[1:], [elbow_target, wrist_target]))
        previous = self.q.copy()
        for indices, bodies, points in targets:
            dofs = self.dadr[indices]
            for _ in range(10):
                mujoco.mj_forward(self.model, self.data)
                errors, jacobians = [], []
                for body, point in zip(bodies, points):
                    jac = np.zeros((3, self.model.nv))
                    mujoco.mj_jac(self.model, self.data, jac, None, self.data.xpos[body], body)
                    errors.extend(point - self.data.xpos[body])
                    jacobians.append(jac[:, dofs])
                jac = np.vstack(jacobians)
                delta = np.linalg.solve(jac.T @ jac + .002 * np.eye(4), jac.T @ errors)
                self.q[indices] += np.clip(delta, -.15, .15)
                limits = self.model.jnt_range[self.ids[indices]]
                self.q[indices] = np.clip(self.q[indices], limits[:, 0], limits[:, 1])
                self.data.qpos[self.qadr] = self.q
        # Head local +Z in Unity, expressed in the current torso basis.
        quat = poses[7, 3:7]
        norm = np.linalg.norm(quat)
        if norm > .1:
            x, y, z, w = quat / norm
            gaze = np.array([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])
            self.q[self.head] = math.atan2(np.dot(gaze, left), np.dot(gaze, forward))
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
    if not sep or label.strip().lower() != 'body joints_pose':
        return None
    fields = payload.strip().lstrip(',').split(',')
    try:
        values = np.array([float(v) for v in fields])
    except ValueError:
        return None
    if values.size != 490 or not np.isfinite(values).all():
        return None
    return values.reshape(70, 7)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--port', type=int, default=9010, help='Forwarded UDP pose port')
    args = parser.parse_args()
    import rclpy
    from interface_protocol.msg import JointCommand
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('quest_sim_retarget')
    solver = Retargeter(args.config)
    topic = solver.joints['command_topic']
    if not topic.startswith('/simulation/'):
        raise ValueError('This bridge requires an isolated /simulation/ command topic')
    pub = node.create_publisher(JointCommand, topic,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', args.port))
    sock.setblocking(False)
    last_valid = None
    previous_tick = time.monotonic()
    stale = True
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
                    pose = parse_pose(line)
                    if pose is not None:
                        latest = pose
            if latest is not None:
                try:
                    solver.solve(latest, start-previous_tick)
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
            previous_tick = start
            time.sleep(max(0., 1/60-(time.monotonic()-start)))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
