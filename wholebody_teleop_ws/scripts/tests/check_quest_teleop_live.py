"""Integration check: run in the ROS container with a working DISPLAY."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import numpy as np
import rclpy
from interface_protocol.msg import JointState, JointCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy
from test_quest_retarget import sample, yaw_quat
from test_quest_orientation import axis_quat, multiply

workspace = Path(__file__).resolve().parents[2]
os.environ['ROS_DOMAIN_ID'] = '87'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
log = open('/tmp/quest-teleop-integration.log', 'w')
proc = subprocess.Popen(['bash', str(workspace/'scripts/run_quest_teleop.sh')],
                        stdout=log, stderr=log, start_new_session=True)
rclpy.init()
node = rclpy.create_node('quest_teleop_integration')
state, commands = [], []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(JointState, '/simulation/quest/joint_state', lambda m: state.append(m.position), qos)
node.create_subscription(JointCommand, '/simulation/quest/joint_command', lambda m: commands.append(m.position), qos)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def run(seconds, pose=None):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        if proc.poll() is not None:
            raise RuntimeError('Launcher exited; inspect /tmp/quest-teleop-integration.log')
        if pose is not None:
            data = 'Body joints_pose:,' + ','.join(map(str,pose.ravel()))
            sock.sendto(data.encode(), ('127.0.0.1',9010))
        rclpy.spin_once(node, timeout_sec=.01)
        time.sleep(.01)
try:
    run(8, sample())
    assert state and commands, 'No ROS states/commands'
    before = np.array(state[-1])
    p = sample()
    p[11,:3] = [-.2,1.4,.3]
    p[19,:3] = [-.2,1.4,.55]
    p[7,3:] = yaw_quat(p[7,3:], -.4)
    run(5,p)
    after = np.array(state[-1])
    change = np.linalg.norm(after[13:17]-before[13:17])
    assert change > .5, change
    head = 23
    assert abs(after[head]-.4) < .05, ('head tracking error', after[head])
    state.clear()
    run(1,p)
    head_span = np.ptp(np.array(state)[:,head])
    assert head_span < .05, ('head oscillation', head_span)
    # Return arms down, then exercise the original wrists and simulation-only neck pitch.
    p = sample()
    run(3, p)
    wrist_before = np.array(state[-1])[17:18]
    p[19, 3:] = axis_quat([0, 1, 0], .4)
    p[7, 3:] = multiply(axis_quat([0, 1, 0], -.4),
                       multiply(axis_quat([1, 0, 0], .3), p[7, 3:]))
    run(3, p)
    extended = np.array(state[-1])
    assert len(extended) == 27, ('extended joint count', len(extended))
    wrist_change = np.linalg.norm(extended[17:18]-wrist_before)
    assert wrist_change > .2, ('left wrist did not turn', wrist_change)
    assert abs(extended[24]-.3) < .05, ('head pitch tracking error', extended[24])
    assert abs(extended[23]-.4) < .05, ('combined head yaw tracking error', extended[23])
    run(1)
    held = np.array(commands[-1])
    run(.5)
    np.testing.assert_equal(commands[-1], held)
    assert np.isfinite(after).all()
    print('PASS: actual simulated left arm moved', change, 'rad; head yaw', after[head], 'rad; settled head span',head_span,'rad; wrist change', wrist_change, 'rad; head pitch', extended[24], 'rad; stale command held')
finally:
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    sock.close()
    node.destroy_node()
    rclpy.shutdown()
    log.close()
