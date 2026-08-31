#!/usr/bin/env python3
"""Stream G1 reference mimic_obs to the TWIST2 low-level policy simulator.

Input:
    *_g1_reference.npz produced by convert_84_to_g1_reference.py

Required field:
    mimic_obs[N, 35]

Redis keys written:
    action_body_unitree_g1_with_hands
    action_hand_left_unitree_g1_with_hands
    action_hand_right_unitree_g1_with_hands
    action_neck_unitree_g1_with_hands
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import redis


ROBOT_KEY = "unitree_g1_with_hands"

DEFAULT_MIMIC_OBS = np.concatenate(
    [
        np.zeros(2, dtype=np.float32),
        np.array([0.793], dtype=np.float32),
        np.zeros(2, dtype=np.float32),
        np.zeros(1, dtype=np.float32),
        np.array(
            [
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.4, 0.0, 1.2, 0.0, 0.0, 0.0,
                0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0,
            ],
            dtype=np.float32,
        ),
    ]
)


def load_reference(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not path.exists():
        raise SystemExit(f"Reference file not found: {path}")

    data = np.load(path, allow_pickle=False)
    required = {"timestamps", "mimic_obs"}
    missing = required.difference(data.files)
    if missing:
        raise SystemExit(f"{path} is missing required arrays: {sorted(missing)}")

    timestamps = np.asarray(data["timestamps"], dtype=np.float64)
    mimic_obs = np.asarray(data["mimic_obs"], dtype=np.float32)
    if mimic_obs.ndim != 2 or mimic_obs.shape[1] != 35:
        raise SystemExit(f"Expected mimic_obs shape [N,35], got {mimic_obs.shape}")
    if timestamps.shape[0] != mimic_obs.shape[0]:
        raise SystemExit(
            f"timestamps length {timestamps.shape[0]} does not match mimic_obs frames {mimic_obs.shape[0]}"
        )
    return timestamps, mimic_obs


def process_mimic_obs(
    mimic_obs: np.ndarray,
    command_scale: float,
    max_dof_offset: float,
    zero_root_motion: bool,
    lowpass_alpha: float,
) -> np.ndarray:
    out = mimic_obs.copy()

    if zero_root_motion:
        out[:, 0:2] = 0.0
        out[:, 3:6] = 0.0
        out[:, 2] = DEFAULT_MIMIC_OBS[2]

    dof = out[:, 6:]
    delta = dof - DEFAULT_MIMIC_OBS[6:]
    delta *= command_scale
    if max_dof_offset > 0.0:
        delta = np.clip(delta, -max_dof_offset, max_dof_offset)
    out[:, 6:] = DEFAULT_MIMIC_OBS[6:] + delta

    if lowpass_alpha > 0.0:
        alpha = min(lowpass_alpha, 0.99)
        for i in range(1, len(out)):
            out[i] = alpha * out[i - 1] + (1.0 - alpha) * out[i]

    return out.astype(np.float32)


def publish(client: redis.Redis, obs: np.ndarray) -> None:
    pipe = client.pipeline()
    pipe.set(f"action_body_{ROBOT_KEY}", json.dumps(obs.tolist()))
    pipe.set(f"action_hand_left_{ROBOT_KEY}", json.dumps(np.zeros(7, dtype=float).tolist()))
    pipe.set(f"action_hand_right_{ROBOT_KEY}", json.dumps(np.zeros(7, dtype=float).tolist()))
    pipe.set(f"action_neck_{ROBOT_KEY}", json.dumps(np.zeros(2, dtype=float).tolist()))
    pipe.set("t_action", int(time.time() * 1000))
    pipe.execute()


def hold(client: redis.Redis, obs: np.ndarray, seconds: float, rate_hz: float = 50.0) -> None:
    steps = max(0, int(seconds * rate_hz))
    for _ in range(steps):
        publish(client, obs)
        time.sleep(1.0 / rate_hz)


def blend(client: redis.Redis, start: np.ndarray, end: np.ndarray, seconds: float, rate_hz: float = 50.0) -> None:
    steps = max(1, int(seconds * rate_hz))
    for i in range(steps):
        a = (i + 1) / steps
        publish(client, ((1.0 - a) * start + a * end).astype(np.float32))
        time.sleep(1.0 / rate_hz)


def stream(
    reference: Path,
    redis_host: str,
    redis_port: int,
    speed: float,
    loop: bool,
    warmup: float,
    blend_in: float,
    command_scale: float,
    max_dof_offset: float,
    zero_root_motion: bool,
    lowpass_alpha: float,
) -> None:
    timestamps, mimic_obs = load_reference(reference)
    mimic_obs = process_mimic_obs(
        mimic_obs,
        command_scale=command_scale,
        max_dof_offset=max_dof_offset,
        zero_root_motion=zero_root_motion,
        lowpass_alpha=lowpass_alpha,
    )

    client = redis.Redis(host=redis_host, port=redis_port, db=0)
    try:
        client.ping()
    except redis.exceptions.ConnectionError as exc:
        raise SystemExit(
            f"Cannot connect to Redis at {redis_host}:{redis_port}. "
            "Start it with: redis-server --daemonize yes"
        ) from exc

    speed = max(speed, 1e-6)
    print(f"reference: {reference}")
    print(f"frames: {len(mimic_obs)} mimic_obs: {mimic_obs.shape}")
    print(f"redis: {redis_host}:{redis_port}")
    print(f"command_scale: {command_scale:g}")
    print(f"max_dof_offset: {max_dof_offset:g}")
    print(f"zero_root_motion: {zero_root_motion}")
    print(f"lowpass_alpha: {lowpass_alpha:g}")

    hold(client, DEFAULT_MIMIC_OBS, warmup)
    blend(client, DEFAULT_MIMIC_OBS, mimic_obs[0], blend_in)

    while True:
        for i, obs in enumerate(mimic_obs):
            t0 = time.time()
            publish(client, obs)
            if i + 1 < len(mimic_obs):
                dt = max((timestamps[i + 1] - timestamps[i]) / speed, 0.0)
            else:
                dt = 1.0 / 50.0
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
        if not loop:
            break


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream G1 reference mimic_obs to TWIST2 policy sim.")
    parser.add_argument("reference", type=Path, help="Input *_g1_reference.npz")
    parser.add_argument("--redis-host", default="localhost")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--blend-in", type=float, default=3.0)
    parser.add_argument("--command-scale", type=float, default=0.6)
    parser.add_argument("--max-dof-offset", type=float, default=0.7)
    parser.add_argument("--allow-root-motion", action="store_true")
    parser.add_argument("--lowpass-alpha", type=float, default=0.85)
    args = parser.parse_args()

    stream(
        reference=args.reference.expanduser(),
        redis_host=args.redis_host,
        redis_port=args.redis_port,
        speed=args.speed,
        loop=args.loop,
        warmup=args.warmup,
        blend_in=args.blend_in,
        command_scale=args.command_scale,
        max_dof_offset=args.max_dof_offset,
        zero_root_motion=not args.allow_root_motion,
        lowpass_alpha=args.lowpass_alpha,
    )


if __name__ == "__main__":
    main()
