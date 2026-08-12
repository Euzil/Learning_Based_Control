"""Record full-body landmark streams from Unity/Quest to an NPZ file.

The recorder expects the body messages produced by BodyLandmarkStreamer:

    Body root:, x, y, z, qx, qy, qz, qw
    Body joints_pose:, [x, y, z, qx, qy, qz, qw] * 84

Examples:
    python scripts/record_body_stream.py --protocol tcp --host localhost --port 8000
    python scripts/record_body_stream.py --protocol udp --host 0.0.0.0 --port 9000 --duration 10
"""

from __future__ import annotations

import argparse
import logging
import select
import signal
import socket
import time
from pathlib import Path
from typing import Iterable, Optional, Tuple

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Missing dependency. Please install numpy: pip install numpy") from exc


BODY_JOINT_COUNT = 84
ROOT_STRIDE = 7
JOINT_POSE_STRIDE = 7


def _default_host(protocol: str) -> str:
    return "0.0.0.0" if protocol == "udp" else "localhost"


def _default_port(protocol: str) -> int:
    return 9000 if protocol == "udp" else 8000


def _timestamped_output_path(output: Optional[str]) -> Path:
    if output:
        return Path(output).expanduser()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("recordings") / f"body_motion_{stamp}.npz"


def _parse_floats(parts: Iterable[str]) -> Tuple[float, ...]:
    values = []
    for part in parts:
        part = part.strip()
        if not part:
            continue
        try:
            values.append(float(part))
        except ValueError:
            continue
    return tuple(values)


def parse_body_line(line: str) -> Optional[Tuple[str, np.ndarray]]:
    """Parse a Unity CSV line into ("root"|"joints_pose"|"joints", values)."""
    parts = [part.strip() for part in line.split(",")]
    if not parts:
        return None

    label = parts[0].lower()
    if "body root" in label:
        values = np.asarray(_parse_floats(parts[1:]), dtype=np.float32)
        if values.size >= ROOT_STRIDE:
            return ("root", values[:ROOT_STRIDE])
        return None

    if "body joints_pose" in label:
        values = np.asarray(_parse_floats(parts[1:]), dtype=np.float32)
        usable = values.size - (values.size % JOINT_POSE_STRIDE)
        if usable >= JOINT_POSE_STRIDE:
            return ("joints_pose", values[:usable].reshape((-1, JOINT_POSE_STRIDE)))
        return None

    if "body joints" in label:
        values = np.asarray(_parse_floats(parts[1:]), dtype=np.float32)
        usable = values.size - (values.size % 3)
        if usable >= 3:
            return ("joints", values[:usable].reshape((-1, 3)))

    return None


class BodyRecorder:
    """Collect root and joint pose messages into timestamped frames."""

    def __init__(self, expected_joints: int = BODY_JOINT_COUNT) -> None:
        self.expected_joints = expected_joints
        self.root: Optional[np.ndarray] = None
        self.timestamps = []
        self.root_poses = []
        self.joint_poses = []
        self.joint_positions = []
        self.first_sample_time: Optional[float] = None
        self.start_time = time.monotonic()

    def update(self, kind: str, values: np.ndarray) -> bool:
        """Update latest data. Returns True when a full frame was recorded."""
        now = time.monotonic()
        if kind == "root":
            self.root = values.astype(np.float32, copy=True)
            return False

        if kind == "joints_pose":
            if values.shape[0] < self.expected_joints:
                logging.warning(
                    "Skipping joints_pose with only %d joints; expected %d",
                    values.shape[0],
                    self.expected_joints,
                )
                return False
            if self.root is None:
                logging.debug("Skipping joints_pose before first Body root")
                return False

            if self.first_sample_time is None:
                self.first_sample_time = now

            self.timestamps.append(now - self.first_sample_time)
            self.root_poses.append(self.root.copy())
            self.joint_poses.append(values[: self.expected_joints].astype(np.float32, copy=True))
            return True

        if kind == "joints":
            if values.shape[0] >= self.expected_joints:
                self.joint_positions.append(
                    values[: self.expected_joints].astype(np.float32, copy=True)
                )

        return False

    @property
    def frame_count(self) -> int:
        return len(self.timestamps)

    def save(self, output_path: Path, metadata: dict) -> None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            output_path,
            timestamps=np.asarray(self.timestamps, dtype=np.float64),
            root_poses=np.asarray(self.root_poses, dtype=np.float32),
            joint_poses=np.asarray(self.joint_poses, dtype=np.float32),
            joint_positions=np.asarray(self.joint_positions, dtype=np.float32),
            joint_count=np.asarray([self.expected_joints], dtype=np.int32),
            metadata=np.asarray([repr(metadata)]),
        )


def _install_stop_handlers() -> callable:
    running = {"value": True}

    def _handle_signal(signum, frame):
        del signum, frame
        running["value"] = False

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    return lambda: running["value"]


def _handle_message(message: str, recorder: BodyRecorder) -> int:
    recorded = 0
    for line in message.splitlines():
        parsed = parse_body_line(line)
        if parsed is None:
            continue
        kind, values = parsed
        if recorder.update(kind, values):
            recorded += 1
    return recorded


def run_udp(
    host: str,
    port: int,
    recorder: BodyRecorder,
    duration: Optional[float],
) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.setblocking(False)
    logging.info("Listening for UDP on %s:%d", host, port)

    keep_running = _install_stop_handlers()
    start = time.monotonic()
    next_report = start + 1.0
    frames_this_second = 0

    try:
        while keep_running():
            if duration is not None and time.monotonic() - start >= duration:
                break

            ready, _, _ = select.select([sock], [], [], 0.1)
            if ready:
                data, _addr = sock.recvfrom(262144)
                try:
                    frames_this_second += _handle_message(data.decode("utf-8"), recorder)
                except UnicodeDecodeError:
                    logging.warning("Skipping non UTF-8 UDP packet")

            now = time.monotonic()
            if now >= next_report:
                logging.info(
                    "recorded frames: %d total, %d in last second",
                    recorder.frame_count,
                    frames_this_second,
                )
                frames_this_second = 0
                next_report += 1.0
    finally:
        sock.close()


def run_tcp(
    host: str,
    port: int,
    recorder: BodyRecorder,
    duration: Optional[float],
) -> None:
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen(1)
    server_sock.settimeout(0.5)

    logging.info("TCP server listening on %s:%d", host, port)
    logging.info("If using USB, run: adb reverse tcp:%d tcp:%d", port, port)
    logging.info("Waiting for Unity/Quest connection...")

    keep_running = _install_stop_handlers()
    start = time.monotonic()
    conn = None
    frames_this_second = 0
    next_report = start + 1.0
    buffer = ""

    try:
        while keep_running() and conn is None:
            if duration is not None and time.monotonic() - start >= duration:
                return
            try:
                conn, addr = server_sock.accept()
                conn.setblocking(False)
                logging.info("Accepted connection from %s", addr)
            except socket.timeout:
                continue

        while keep_running() and conn is not None:
            if duration is not None and time.monotonic() - start >= duration:
                break

            ready, _, _ = select.select([conn], [], [], 0.1)
            if ready:
                data = conn.recv(262144)
                if not data:
                    logging.info("TCP connection closed")
                    break
                try:
                    buffer += data.decode("utf-8")
                except UnicodeDecodeError:
                    logging.warning("Skipping non UTF-8 TCP chunk")
                    continue

                lines = buffer.split("\n")
                buffer = lines.pop()
                frames_this_second += _handle_message("\n".join(lines), recorder)

            now = time.monotonic()
            if now >= next_report:
                logging.info(
                    "recorded frames: %d total, %d in last second",
                    recorder.frame_count,
                    frames_this_second,
                )
                frames_this_second = 0
                next_report += 1.0
    finally:
        if conn is not None:
            conn.close()
        server_sock.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="record_body_stream",
        description="Record Unity/Quest BodyLandmarkStreamer output to a compressed NPZ file.",
    )
    parser.add_argument(
        "--protocol",
        choices=("udp", "tcp"),
        default="tcp",
        help="Transport protocol to listen on (default: tcp).",
    )
    parser.add_argument(
        "--host",
        default=None,
        help="Host/IP to bind to (default: localhost for TCP, 0.0.0.0 for UDP).",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=None,
        help="Port to listen on (default: 8000 for TCP, 9000 for UDP).",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output .npz path (default: recordings/body_motion_YYYYmmdd_HHMMSS.npz).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Recording duration in seconds. If omitted, press Ctrl+C to stop.",
    )
    parser.add_argument(
        "--expected-joints",
        type=int,
        default=BODY_JOINT_COUNT,
        help="Expected full-body joint count (default: 84).",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        help="Logging verbosity (default: INFO).",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(message)s",
    )

    host = args.host or _default_host(args.protocol)
    port = args.port or _default_port(args.protocol)
    output_path = _timestamped_output_path(args.output)
    recorder = BodyRecorder(expected_joints=args.expected_joints)

    metadata = {
        "protocol": args.protocol,
        "host": host,
        "port": port,
        "expected_joints": args.expected_joints,
        "created_unix_time": time.time(),
        "format": "timestamps[N], root_poses[N,7], joint_poses[N,84,7]",
    }

    try:
        if args.protocol == "udp":
            run_udp(host, port, recorder, args.duration)
        else:
            run_tcp(host, port, recorder, args.duration)
    finally:
        if recorder.frame_count == 0:
            logging.warning("No complete body frames recorded; output file was not written.")
        else:
            recorder.save(output_path, metadata)
            logging.info("Saved %d frames to %s", recorder.frame_count, output_path)


if __name__ == "__main__":
    main()
