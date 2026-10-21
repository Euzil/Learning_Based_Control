#!/usr/bin/env python3
"""Receive real WebRTC frames with the Quest signaling protocol (no headset needed)."""
import argparse
import asyncio
import time
import uuid
from aiohttp import ClientSession
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription


async def check(url, frame_count):
    peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    peer.addTransceiver("video", direction="recvonly")
    incoming = asyncio.Queue()
    peer.on("track", incoming.put_nowait)
    session = str(uuid.uuid4())
    try:
        async with ClientSession() as http:
            async with http.ws_connect(url) as ws:
                async def send(kind, payload):
                    await ws.send_json({"type": kind, "session_id": session, "payload": payload})
                await send("hello", {"video_preset": "720p30", "client_name": "smoke_test"})
                await send("start_video", {})
                await peer.setLocalDescription(await peer.createOffer())
                await send("offer", {"sdp": peer.localDescription.sdp})
                while True:
                    msg = await ws.receive_json(timeout=15)
                    if msg["type"] == "error":
                        raise RuntimeError(msg["payload"])
                    if msg["type"] == "answer":
                        await peer.setRemoteDescription(RTCSessionDescription(msg["payload"]["sdp"], "answer"))
                        break
                track = await asyncio.wait_for(incoming.get(), 10)
                started = time.monotonic()
                previous_pts = -1
                for index in range(frame_count):
                    frame = await asyncio.wait_for(track.recv(), 15)
                    assert frame.pts > previous_pts, "Non-monotonic frame timestamps"
                    previous_pts = frame.pts
                    if index == 0:
                        print(f"First frame: {frame.width}x{frame.height}, format={frame.format.name}")
                        assert frame.to_ndarray(format="rgb24").std() > 1, "Blank image"
                print(f"Received {frame_count} frames in {time.monotonic()-started:.2f}s; peer={peer.connectionState}")
                await send("stop_video", {})
    finally:
        await peer.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:8765")
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()
    asyncio.run(check(args.url, args.frames))
