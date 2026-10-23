"""Personal arm mapping, with optional fixed standing-height calibration."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


def scalar(value, name, lower, upper):
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a number')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a number') from exc
    if not np.isfinite(number) or not lower <= number <= upper:
        raise ValueError(f'{name} must be finite and in [{lower}, {upper}]')
    return number


def vector(value, name, bound):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must contain three numbers') from exc
    if result.shape != (3,) or not np.isfinite(result).all() or np.any(abs(result) > bound):
        raise ValueError(f'{name} must contain three finite values within +/-{bound} m')
    return result


def mapping(value, name, allowed):
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be a mapping')
    unknown = set(value)-set(allowed)
    if unknown:
        raise ValueError(f'Unknown {name} fields: {sorted(unknown, key=str)}')
    return value


@dataclass
class ArmProfile:
    upper_arm_m: float
    forearm_m: float
    upper_gain: float
    forearm_gain: float
    hand_offset_m: np.ndarray
    robot_hand_local_m: np.ndarray
    elbow_offset_m: np.ndarray


@dataclass
class BodyProfile:
    arms: list
    elbow_weight: float
    hand_weight: float
    calibration: dict | None = None

    @classmethod
    def load(cls, path):
        cfg = mapping(yaml.safe_load(Path(path).read_text()), 'profile',
                      ('left', 'right', 'elbow_weight', 'hand_weight', 'calibration'))
        raw_calibration = cfg.get('calibration')
        sampled = isinstance(raw_calibration, dict) and raw_calibration.get('fit_mode') == 'sampled'
        length_bounds = (np.finfo(float).tiny, float('inf')) if sampled else (.10, .60)
        gain_bounds = (-float('inf'), float('inf')) if sampled else (.5, 1.5)
        offset_bound = float('inf') if sampled else .15
        arms = []
        for side in ('left', 'right'):
            arm = mapping(cfg.get(side), side,
                          ('upper_arm_m', 'forearm_m', 'upper_gain', 'forearm_gain',
                           'hand_offset_m', 'robot_hand_local_m', 'elbow_offset_m'))
            arms.append(ArmProfile(
                scalar(arm.get('upper_arm_m'), f'{side}.upper_arm_m', *length_bounds),
                scalar(arm.get('forearm_m'), f'{side}.forearm_m', *length_bounds),
                scalar(arm.get('upper_gain', 1.), f'{side}.upper_gain', *gain_bounds),
                scalar(arm.get('forearm_gain', 1.), f'{side}.forearm_gain', *gain_bounds),
                vector(arm.get('hand_offset_m', [0, 0, 0]), f'{side}.hand_offset_m', offset_bound),
                vector(arm.get('robot_hand_local_m', [.03, 0, -.14]),
                       f'{side}.robot_hand_local_m', .30),
                vector(arm.get('elbow_offset_m', [0, 0, 0]), f'{side}.elbow_offset_m', offset_bound)))
        calibration = cfg.get('calibration')
        if calibration is not None:
            calibration = dict(mapping(calibration, 'calibration',
                               ('version', 'human_shoulder_y_m', 'robot_shoulder_z_m', 'vertical_scale', 'fit_mode')))
            if calibration.get('fit_mode') not in (None, 'sampled'):
                raise ValueError('Unsupported calibration fit mode')
            if calibration.get('version') != 1:
                raise ValueError('Unsupported calibration version')
            for key in ('human_shoulder_y_m', 'robot_shoulder_z_m'):
                value = np.asarray(calibration.get(key), dtype=float)
                if value.shape != (2,) or not np.isfinite(value).all():
                    raise ValueError(f'calibration.{key} must contain two finite values')
                calibration[key] = value
            calibration['vertical_scale'] = scalar(calibration.get('vertical_scale', 1.),
                                                    'vertical_scale', .25, 2.)
        return cls(arms,
                   scalar(cfg.get('elbow_weight', .5), 'elbow_weight', .01, 10.),
                   scalar(cfg.get('hand_weight', 1.), 'hand_weight', .01, 10.), calibration)

    def targets(self, arm_index, upper, lower, robot_lengths, shoulder_y=None):
        """Input vectors are in torso axes/metres. Calibrated output Z is absolute.
        Without calibration, all output axes are relative to the shoulder.

        Deliberately do not normalize: otherwise personal lengths and gains cancel.
        """
        arm = self.arms[arm_index]
        elbow = (upper * (robot_lengths[0] / arm.upper_arm_m) * arm.upper_gain
                 + arm.elbow_offset_m)
        hand = (elbow + lower * (robot_lengths[1] / arm.forearm_m) * arm.forearm_gain
                + arm.hand_offset_m)
        targets = np.array([elbow, hand])
        if self.calibration is not None:
            if shoulder_y is None or not np.isfinite(shoulder_y):
                raise ValueError('Calibrated mapping requires tracked shoulder height')
            # X/Y remain shoulder-relative; Z is an absolute model-world target.
            targets[:, 2] += (self.calibration['robot_shoulder_z_m'][arm_index] +
                              self.calibration['vertical_scale'] *
                              (shoulder_y-self.calibration['human_shoulder_y_m'][arm_index]))
        return targets
