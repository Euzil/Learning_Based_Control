#!/usr/bin/env python3
"""ROS RGB Image -> Quest HTS WebRTC. No robot commands are published."""
import argparse
import asyncio
from fractions import Fraction
import json
import logging
from pathlib import Path
import threading
import time

from aiohttp import web, WSMsgType
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.sdp import candidate_from_sdp
from av import VideoFrame
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

LOG = logging.getLogger("quest_video")


def image_array(msg):
    """Decode ROS RGB/BGR images, including row padding, without cv_bridge."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels:
        raise ValueError("Invalid image dimensions/stride")
    if len(msg.data) < msg.height * msg.step:
        raise ValueError("Truncated image")
    rows = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height * msg.step)
    pixels = rows.reshape(msg.height, msg.step)[:, :msg.width * channels]
    pixels = pixels.reshape(msg.height, msg.width, channels)[:, :, :3]
    if msg.encoding.startswith("bgr"):
        pixels = pixels[:, :, ::-1]
    return np.ascontiguousarray(pixels)


class ImageSource(Node):
    def __init__(self, topic):
        super().__init__("quest_video_source")
        self.lock = threading.Lock()
        self.latest = None
        self.received_at = 0.0
        self.sequence = 0
        self.last_error_log = 0.0
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        try:
            array = image_array(msg)
        except ValueError as exc:
            now = time.monotonic()
            if now - self.last_error_log > 5:
                LOG.warning("%s", exc)
                self.last_error_log = now
            return
        with self.lock:
            # Keep only the newest image; never build a latency-producing queue.
            self.latest = array
            self.received_at = time.monotonic()
            self.sequence += 1

    def snapshot(self):
        with self.lock:
            return self.latest, self.received_at, self.sequence


class RosVideoTrack(VideoStreamTrack):
    def __init__(self, source, fps=30):
        super().__init__()
        self.source = source
        self.period = 1 / fps
        self.started = time.monotonic()
        self.next_frame = self.started
        self.last_pts = -1
        self.last_sequence = 0
        self.frames = 0
        self.drops = 0

    async def recv(self):
        await asyncio.sleep(max(0, self.next_frame - time.monotonic()))
        while True:
            image, received, sequence = self.source.snapshot()
            if image is not None and sequence != self.last_sequence and time.monotonic() - received < 2:
                break
            await asyncio.sleep(0.02)
        now = time.monotonic()
        self.next_frame = now + self.period
        frame = VideoFrame.from_ndarray(image, format="rgb24")
        frame.pts = max(self.last_pts + 1, int((now - self.started) * 90000))
        frame.time_base = Fraction(1, 90000)
        self.last_pts = frame.pts
        if self.last_sequence:
            self.drops += max(0, sequence - self.last_sequence - 1)
        self.last_sequence = sequence
        self.frames += 1
        return frame


class VideoServer:
    def __init__(self, source, fps):
        self.source = source
        self.fps = fps
        self.sockets = set()

    async def preview(self, request):
        return web.FileResponse(Path(__file__).with_name("quest_video_preview.html"))

    async def health(self, request):
        image, received, sequence = self.source.snapshot()
        age = time.monotonic() - received if image is not None else None
        return web.json_response({"source_ready": age is not None and age < 2,
                                  "image_age_seconds": age, "frames_received": sequence,
                                  "width": image.shape[1] if image is not None else None,
                                  "height": image.shape[0] if image is not None else None})

    async def websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=1024 * 1024)
        await ws.prepare(request)
        self.sockets.add(ws)
        peer = None
        track = None
        monitor = None
        session = None
        pending_ice = []

        async def send(kind, payload):
            if not ws.closed:
                await ws.send_json({"type": kind, "session_id": session, "payload": payload})

        async def add_ice(payload):
            raw = payload.get("candidate", "")
            if not raw:
                await peer.addIceCandidate(None)
                return
            candidate = candidate_from_sdp(raw.removeprefix("candidate:"))
            candidate.sdpMid = payload.get("sdpMid")
            candidate.sdpMLineIndex = payload.get("sdpMLineIndex", 0)
            await peer.addIceCandidate(candidate)

        async def monitor_source():
            started = last = time.monotonic()
            previous_frames = previous_bytes = 0
            while True:
                await asyncio.sleep(1)
                now = time.monotonic()
                _, received, _ = self.source.snapshot()
                if now - started > 10 and (received == 0 or now - received > 3):
                    await send("error", {"message": "No fresh ROS camera image. Check simulator and image topic."})
                    await ws.close()
                    return
                stats = await peer.getStats()
                sent = sum(getattr(s, "bytesSent", 0) for s in stats.values()
                           if s.type == "outbound-rtp")
                await send("stats", {"fps": (track.frames - previous_frames) / (now - last),
                                     "bitrate_kbps": 8 * (sent - previous_bytes) / (now - last) / 1000,
                                     "frame_drops": track.drops, "rtt_ms": None})
                last, previous_frames, previous_bytes = now, track.frames, sent

        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    continue
                envelope = json.loads(message.data)
                incoming_session = envelope.get("session_id")
                if not incoming_session or (session and session != incoming_session):
                    raise ValueError("Missing or inconsistent session_id")
                session = incoming_session
                kind = envelope.get("type")
                payload = envelope.get("payload", {})
                if kind == "hello":
                    await send("hello_ack", {"video_preset": "source", "source": "ros_image"})
                elif kind == "start_video":
                    await send("video_state", {"state": "starting"})
                elif kind == "offer":
                    if peer is not None:
                        raise ValueError("Create a new session before sending another offer")
                    peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
                    track = RosVideoTrack(self.source, self.fps)

                    @peer.on("connectionstatechange")
                    async def state_change():
                        LOG.info("session=%s peer=%s", session, peer.connectionState)
                        if peer.connectionState == "failed":
                            await send("error", {"message": "WebRTC ICE failed. Check LAN routing/firewall."})
                            await ws.close()

                    await peer.setRemoteDescription(RTCSessionDescription(sdp=payload["sdp"], type="offer"))
                    peer.addTrack(track)
                    for candidate in pending_ice:
                        await add_ice(candidate)
                    pending_ice.clear()
                    await peer.setLocalDescription(await peer.createAnswer())
                    # The completed answer contains host ICE candidates (non-trickle).
                    await send("answer", {"sdp": peer.localDescription.sdp})
                    monitor = asyncio.create_task(monitor_source())
                elif kind == "ice_candidate":
                    if peer is None:
                        if len(pending_ice) >= 256:
                            raise ValueError("Too many early ICE candidates")
                        pending_ice.append(payload)
                    else:
                        await add_ice(payload)
                elif kind == "ping":
                    await send("pong", {})
                elif kind == "stop_video":
                    await send("video_state", {"state": "stopped"})
                    break
        except Exception as exc:
            LOG.exception("Video session failed")
            await send("error", {"message": str(exc)})
        finally:
            if monitor:
                monitor.cancel()
                await asyncio.gather(monitor, return_exceptions=True)
            if peer:
                await peer.close()
            if track:
                track.stop()
            self.sockets.discard(ws)
            await ws.close()
        return ws

    async def shutdown(self, app):
        await asyncio.gather(*(ws.close() for ws in list(self.sockets)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/simulation/head/image_raw")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--telemetry-port", type=int, default=0,
                        help="Optional TCP sink for display-only testing; discards poses, sends no robot commands")
    args = parser.parse_args()
    if not 1 <= args.fps <= 60:
        parser.error("--fps must be between 1 and 60")
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=[])
    source = ImageSource(args.topic)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(source)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    server = VideoServer(source, args.fps)
    app = web.Application()
    app.router.add_get("/", server.websocket)
    app.router.add_get("/health", server.health)
    app.router.add_get("/preview", server.preview)
    app.on_shutdown.append(server.shutdown)
    if args.telemetry_port:
        writers = set()

        async def discard_poses(reader, writer):
            writers.add(writer)
            try:
                while await reader.read(65536):
                    pass
            except (ConnectionError, asyncio.CancelledError):
                pass
            finally:
                writers.discard(writer)
                writer.close()
                try:
                    await writer.wait_closed()
                except ConnectionError:
                    pass

        async def telemetry_lifetime(app):
            listener = await asyncio.start_server(discard_poses, args.host, args.telemetry_port)
            LOG.info("Display-only TCP pose sink on port %s (poses discarded)", args.telemetry_port)
            try:
                yield
            finally:
                listener.close()
                await listener.wait_closed()
                for writer in list(writers):
                    writer.close()

        app.cleanup_ctx.append(telemetry_lifetime)
    LOG.info("ROS source=%s, signaling ws://%s:%s", args.topic, args.host, args.port)
    try:
        web.run_app(app, host=args.host, port=args.port)
    finally:
        executor.shutdown()
        ros_thread.join(timeout=3)
        source.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
