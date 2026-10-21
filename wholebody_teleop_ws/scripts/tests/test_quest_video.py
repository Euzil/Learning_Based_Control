"""Run with the video venv after sourcing ROS: python -m unittest discover -s scripts/tests."""
import sys
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
