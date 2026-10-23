"""ROS/UDP calibration integration with synthetic feedback; no physical simulator/Quest.

Run in the ROS container after sourcing ROS and install/setup.bash.
Uses an isolated ROS domain, a random UDP port, and temporary output files.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '193'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from interface_protocol.msg import JointCommand, JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_retarget import Retargeter
from quest_calibration import robot_points, demo_pose, relative_targets
from quest_body_profile import BodyProfile
from test_quest_retarget import sample


def main():
    scripts = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        subprocess.run([sys.executable, str(scripts/'prepare_quest_teleop.py'), directory], check=True)
        solver = Retargeter(root/'config.yaml')
        robot_ref = robot_points(solver, demo_pose(solver, 0))[:, 0, 2]
        output = root/'calibrated.yaml'
        ready = root/'tracking-ready'
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
        sock.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rclpy.init()
        node = rclpy.create_node('calibration_integration')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub = node.create_publisher(JointState, solver.joints['state_topic'], qos)
        q = solver.q.copy()
        commands = []
        def receive(msg):
            nonlocal q
            q = np.array(msg.position)
            commands.append(q.copy())
        node.create_subscription(JointCommand, solver.joints['command_topic'], receive, qos)
        with (root/'bridge.log').open('w+') as log:
            proc = subprocess.Popen([sys.executable, str(scripts/'quest_retarget.py'),
                                     '--config', str(root/'config.yaml'), '--port', str(port),
                                     '--calibrate', str(output), '--tracking-ready-file', str(ready)], stdout=log, stderr=log)
            try:
                start = time.monotonic()
                completed_at = None
                while time.monotonic()-start < 120:
                    if proc.poll() is not None:
                        raise RuntimeError(f'Bridge exited: {proc.returncode}')
                    rclpy.spin_once(node, timeout_sec=.001)
                    state = JointState()
                    state.header.stamp = node.get_clock().now().to_msg()
                    state.position = q.tolist()
                    pub.publish(state)
                    targets = relative_targets(robot_points(solver, q), robot_ref)
                    upper = (targets[:, 0]-[.005, 0, -.005])/.6
                    lower = (targets[:, 1]-targets[:, 0]-[0, .003, .004])/1.1
                    p = sample()
                    for i, (shoulder, elbow, hand) in enumerate(((10, 11, 19), (15, 16, 45))):
                        p[elbow, :3] = p[shoulder, :3]+[-upper[i, 1], upper[i, 2], upper[i, 0]]
                        p[hand, :3] = p[elbow, :3]+[-lower[i, 1], lower[i, 2], lower[i, 0]]
                    sock.sendto(('Body joints_pose:,'+','.join(map(str, p.ravel()))).encode(), ('127.0.0.1', port))
                    if ready.exists():
                        assert output.exists(), 'Video released before calibration passed'
                    if output.exists():
                        if completed_at is None:
                            completed_at = time.monotonic()
                            BodyProfile.load(output)
                        elif time.monotonic()-completed_at > .5:
                            break
                    time.sleep(.015)
                log.flush()
                log.seek(0)
                contents = log.read()
                assert output.exists(), contents
                assert Path(str(output)+'.report.yaml').exists(), contents
                assert 'Tracking active' in contents, contents
                assert ready.is_file(), 'Video readiness marker missing after calibration'
                assert len(commands) > 100, len(commands)
                assert np.isfinite(np.array(commands)).all()
                print('PASS: four poses through ROS JointState/JointCommand + UDP; saved sampled profile; tracking resumed.')
                print((Path(str(output)+'.report.yaml')).read_text())
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                sock.close()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
