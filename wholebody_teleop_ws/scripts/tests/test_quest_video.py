"""Run with the video venv after sourcing ROS: python -m unittest discover -s scripts/tests."""
import sys
import json
import os
import socket
import subprocess
import tempfile
import time
from urllib.request import urlopen
from pathlib import Path
import unittest
from types import SimpleNamespace
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_video_server import image_array


class ImageConversionTests(unittest.TestCase):
    def test_padded_bgr(self):
        msg = SimpleNamespace(encoding="bgr8", width=2, height=2, step=8,
                              data=bytes([3, 2, 1, 6, 5, 4, 0, 0, 9, 8, 7, 12, 11, 10, 0, 0]))
        np.testing.assert_array_equal(image_array(msg),
                                      [[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]])

    def test_rgba(self):
        msg = SimpleNamespace(encoding="rgba8", width=1, height=1, step=4,
                              data=bytes([20, 30, 40, 255]))
        np.testing.assert_array_equal(image_array(msg), [[[20, 30, 40]]])

    def test_invalid_image(self):
        for encoding, step, data in [("32FC1", 4, bytes(4)), ("rgb8", 2, bytes(2)),
                                     ("rgb8", 3, bytes(2))]:
            with self.subTest(encoding=encoding, step=step, size=len(data)):
                with self.assertRaises(ValueError):
                    image_array(SimpleNamespace(encoding=encoding, width=1, height=1,
                                                step=step, data=data))


class CalibrationVideoTests(unittest.TestCase):
    def test_no_listener_until_calibration_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory)/'tracking-ready'
            with socket.socket() as sock:
                sock.bind(('127.0.0.1', 0))
                port = sock.getsockname()[1]
            env = dict(os.environ, ROS_DOMAIN_ID='194', ROS_LOCALHOST_ONLY='1')
            log_path = Path(directory)/'video.log'
            with log_path.open('w') as log:
                process = subprocess.Popen([
                    sys.executable, str(Path(__file__).resolve().parents[1]/'quest_video_server.py'),
                    '--host', '127.0.0.1', '--port', str(port), '--wait-for-tracking', str(ready)],
                    stdout=log, stderr=log, env=env)
                try:
                    deadline = time.monotonic()+10
                    while 'robot video paused' not in log_path.read_text():
                        self.assertIsNone(process.poll(), log_path.read_text())
                        if time.monotonic() > deadline:
                            self.fail(log_path.read_text())
                        time.sleep(.05)
                    with self.assertRaises(OSError):
                        socket.create_connection(('127.0.0.1', port), timeout=.2)
                    ready.write_text('tracking ready\n')
                    deadline = time.monotonic()+10
                    while True:
                        self.assertIsNone(process.poll(), log_path.read_text())
                        try:
                            with urlopen(f'http://127.0.0.1:{port}/health', timeout=.2) as response:
                                health = json.load(response)
                            self.assertFalse(health['source_ready'])
                            break
                        except OSError:
                            if time.monotonic() > deadline:
                                self.fail(log_path.read_text())
                            time.sleep(.05)
                finally:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
