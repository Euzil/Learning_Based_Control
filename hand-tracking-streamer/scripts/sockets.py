"""TCP/UDP listener for Hand Tracking Streamer (HTS).

The script can either:
    - Log each UTF-8 message received, split by newline, or
    - Tally how many packets arrive per second (``--tally`` mode).

Examples:
    python sockets.py --protocol udp --host 0.0.0.0 --port 9000
    python sockets.py --protocol tcp --host localhost --port 8000
"""

from __future__ import annotations

import argparse
import logging
import os
import select
import signal
import socket
import threading
import time


class PoseForwarder:
    """Forward complete body/head lines without parsing or changing their values."""
    def __init__(self, port):
        self.target = ('127.0.0.1', port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, line):
        label = line.split(b':', 1)[0].split(b'|', 1)[0].strip().lower()
        if label in (b'body joints_pose', b'head pose'):
            try:
                self.sock.sendto(line, self.target)
            except OSError:
                pass  # Offline/slow simulation must not block Quest reception.

    def close(self):
        self.sock.close()


def run_udp_listener(host: str, port: int, tally: bool, forwarder=None) -> None:
    """Listen for UDP packets and log or tally them."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.setblocking(False)

    logging.info("Listening for UDP on %s:%d", host, port)

    running = True

    def _handle_sigint(signum, frame):
        """Signal handler that requests a clean shutdown."""
        del signum, frame
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _handle_sigint)
    signal.signal(signal.SIGTERM, _handle_sigint)

    msgs_this_second = 0
    next_report = time.monotonic() + 1.0

    try:
        while running:
            timeout = max(0.0, next_report - time.monotonic())
            ready, _, _ = select.select([sock], [], [], timeout)
            if ready:
                try:
                    data, addr = sock.recvfrom(65536)
                except BlockingIOError:
                    continue

                if forwarder is not None:
                    for line in data.splitlines():
                        forwarder.send(line)
                if tally:
                    msgs_this_second += 1
                else:
                    try:
                        message = data.decode("utf-8")
                        for line in message.split("\n"):
                            if line:
                                logging.info("Message from %s: %s", addr, line)
                    except UnicodeDecodeError:
                        logging.info("Message from %s: %s", addr, data)

            now = time.monotonic()
            if now >= next_report:
                if tally:
                    logging.info("packets/sec: %d", msgs_this_second)
                msgs_this_second = 0
                next_report = now + 1.0
    finally:
        try:
            sock.close()
        except OSError:
            pass


def handle_tcp_connection(conn, addr, tally: bool, forwarder=None) -> None:
    """Reassemble newline-delimited records across TCP packet boundaries."""
    with conn:
        logging.info("Accepted connection from %s", addr)
        buffer = b''
        count = 0
        next_report = time.monotonic()+1.
        conn.settimeout(1.)
        try:
            while True:
                try:
                    data = conn.recv(65536)
                except socket.timeout:
                    data = None
                if data == b'':
                    logging.info("Connection from %s closed", addr)
                    break  # Discard an incomplete final record.
                if data:
                    buffer += data
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        line = line.rstrip(b'\r')
                        if not line:
                            continue
                        count += 1
                        if forwarder is not None:
                            forwarder.send(line)
                        if not tally:
                            logging.info("Message from %s: %s", addr,
                                         line.decode('utf-8', errors='replace'))
                    if len(buffer) > 1024*1024:
                        logging.warning("Closing oversized incomplete record from %s", addr)
                        break
                now = time.monotonic()
                if now >= next_report:
                    if tally:
                        logging.info("messages/sec from %s: %d", addr, count)
                    count = 0
                    next_report = now+1.
        except OSError as exc:
            logging.warning("Connection from %s ended: %s", addr, exc)


def run_tcp_server(host: str, port: int, tally: bool, forwarder=None) -> None:
    """Listen for TCP connections from HTS and handle each in a separate thread."""
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen(5)  # Allow multiple pending connections

    logging.info("TCP server listening on %s:%d", host, port)
    logging.info("Waiting for connections from HTS (Quest)...")
    logging.info("Remember to setup TCP reverse first: 'adb reverse tcp:%d tcp:%d'", port, port)

    try:
        while True:
            conn, addr = server_sock.accept()
            # Handle each connection in a separate thread
            thread = threading.Thread(
                target=handle_tcp_connection,
                args=(conn, addr, tally, forwarder),
                daemon=True
            )
            thread.start()
    finally:
        server_sock.close()


def _default_host(protocol: str) -> str:
    return "0.0.0.0" if protocol == "udp" else "localhost"


def _default_port(protocol: str) -> int:
    return 9000 if protocol == "udp" else 8000


def main() -> None:
    """Parse command-line arguments and start the listener."""
    parser = argparse.ArgumentParser(
        prog="sockets",
        description=(
            "TCP/UDP message listener for Hand Tracking Streamer (HTS); "
            "either prints messages or tallies packets per second."
        ),
    )
    parser.add_argument(
        "--protocol",
        choices=("udp", "tcp"),
        default="udp",
        help="Transport protocol to listen on (default: udp).",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=None,
        help="Port to listen on (default: 9000 for UDP, 8000 for TCP).",
    )
    parser.add_argument(
        "--host",
        default=None,
        help="Host/IP to bind to (default: 0.0.0.0 for UDP, localhost for TCP).",
    )
    parser.add_argument(
        "--tally",
        action="store_true",
        help="Only count messages per second instead of printing each message.",
    )
    parser.add_argument('--forward-port', type=int, default=None,
                        help='Forward body/head poses to localhost UDP; defaults to HTS_SIM_FORWARD_PORT.')
    args = parser.parse_args()
    try:
        forward_port = (args.forward_port if args.forward_port is not None else
                        int(os.environ.get('HTS_SIM_FORWARD_PORT', '0')))
    except ValueError:
        parser.error('HTS_SIM_FORWARD_PORT must be an integer')
    if not 0 <= forward_port <= 65535:
        parser.error('Forward port must be 1-65535, or 0 to disable')

    host = args.host or _default_host(args.protocol)
    port = args.port or _default_port(args.protocol)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    if args.protocol == 'udp' and forward_port == port:
        parser.error('UDP receiving and forwarding ports must differ')
    forwarder = PoseForwarder(forward_port) if forward_port else None
    if forwarder:
        logging.info('Forwarding Body joints_pose and Head pose to UDP 127.0.0.1:%d', forward_port)
    try:
        if args.protocol == "udp":
            run_udp_listener(host, port, args.tally, forwarder)
        else:
            run_tcp_server(host, port, args.tally, forwarder)
    except KeyboardInterrupt:
        pass
    finally:
        if forwarder:
            forwarder.close()


if __name__ == "__main__":
    main()
