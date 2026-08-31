#!/usr/bin/env python3
"""Visualize a G1 reference qpos sequence in MuJoCo.

This is a kinematic viewer for files produced by convert_84_to_g1_reference.py.
It writes qpos_ref into MuJoCo and calls mj_forward, so it is for checking the
reference pose quality before sending anything to a controller.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


DEFAULT_XML = Path("../TWIST2/assets/g1/g1_mocap_29dof.xml")


def load_reference(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not path.exists():
        raise SystemExit(f"Reference file not found: {path}")

    data = np.load(path, allow_pickle=False)
    if "qpos_ref" in data.files:
        qpos = np.asarray(data["qpos_ref"], dtype=np.float64)
    elif "qpos" in data.files:
        qpos = np.asarray(data["qpos"], dtype=np.float64)
    else:
        raise SystemExit(f"{path} must contain qpos_ref or qpos.")

    if "timestamps" in data.files:
        timestamps = np.asarray(data["timestamps"], dtype=np.float64)
    else:
        timestamps = np.arange(len(qpos), dtype=np.float64) / 30.0

    if qpos.ndim != 2:
        raise SystemExit(f"Expected qpos_ref [N,nq], got {qpos.shape}")
    if timestamps.shape[0] != qpos.shape[0]:
        raise SystemExit(
            f"timestamps length {timestamps.shape[0]} does not match qpos frames {qpos.shape[0]}"
        )
    return timestamps, qpos


def update_camera(viewer, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    if pelvis_id >= 0:
        viewer.cam.lookat[:] = data.xpos[pelvis_id]
    viewer.cam.distance = 2.2
    viewer.cam.elevation = -15
    viewer.cam.azimuth = -140


def replay(
    xml_path: Path,
    reference_path: Path,
    speed: float,
    loop: bool,
    frame_step: int,
) -> None:
    timestamps, qpos_seq = load_reference(reference_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    if qpos_seq.shape[1] != model.nq:
        raise SystemExit(f"qpos width {qpos_seq.shape[1]} does not match model.nq {model.nq}")

    speed = max(speed, 1e-6)
    frame_step = max(frame_step, 1)
    print(f"xml: {xml_path}")
    print(f"reference: {reference_path}")
    print(f"frames: {len(qpos_seq)} nq: {model.nq}")

    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        frame = 0
        previous_timestamp = timestamps[0]
        while viewer.is_running():
            data.qpos[:] = qpos_seq[frame]
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            update_camera(viewer, model, data)
            viewer.sync()

            next_frame = frame + frame_step
            if next_frame >= len(qpos_seq):
                if not loop:
                    break
                next_frame = 0

            if next_frame > frame:
                dt = timestamps[next_frame] - previous_timestamp
            else:
                dt = 1.0 / 30.0
            previous_timestamp = timestamps[next_frame]
            time.sleep(max(dt / speed, 0.0))
            frame = next_frame


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize G1 qpos_ref in MuJoCo.")
    parser.add_argument("reference", type=Path, help="Input *_g1_reference.npz")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="G1 MuJoCo XML path.")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier.")
    parser.add_argument("--loop", action="store_true", help="Loop until the viewer is closed.")
    parser.add_argument("--frame-step", type=int, default=1, help="Advance by this many frames.")
    args = parser.parse_args()

    replay(
        xml_path=args.xml.expanduser(),
        reference_path=args.reference.expanduser(),
        speed=args.speed,
        loop=args.loop,
        frame_step=args.frame_step,
    )


if __name__ == "__main__":
    main()
