"""ZMQ frame receiver — newest frame wins.

Accepts a frame in ANY of three wire framings, so publisher changes never
black out the stream:
    1. multipart:  send(header, SNDMORE); send(payload)     ← current streamer
    2. combined:   send(header + payload)                    (one message)
    3. legacy:     send(header); send(payload)               (two messages)

LATENCY RULE — the fix for "stream lags reality" lives in the PUBLISHER:
    sock.setsockopt(zmq.SNDHWM, 1)   # BEFORE bind — queue at most one frame;
                                     # PUB drops stale frames instead of
                                     # queueing them behind the slow pipe
Any queue between camera and screen adds standing latency equal to its depth
× frame transfer time. Color frames are far larger than depth frames, so a
queued color pipe trails a live depth pipe by exactly that backlog. The
receiver here keeps its own side drained (small RCVHWM + skip-to-newest), but
it can only skip frames that already crossed TCP — publisher-side queueing
must be capped at the publisher.

NOTE: ZMQ_CONFLATE is deliberately NOT used — libzmq aborts the whole process
(assertion in fq.cpp) when a conflating socket receives a multipart message,
so it would turn a publisher framing choice into a crash loop here.
"""

import time
import zlib

import cv2
import zmq
import numpy as np
import struct
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable

HEADER_FMT  = "<BIHHBI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Header "type" byte (see stream_packet.h in the Astra streamer)
TYPE_COLOR_RAW  = 0   # raw RGB888
TYPE_DEPTH_RAW  = 1   # raw int16 mm
TYPE_COLOR_JPEG = 2   # JPEG -> decoded to RGB888
TYPE_DEPTH_ZLIB = 3   # zlib( per-row delta of uint16 depth ) -> int16 mm, lossless


def decode_payload(stream_type, w, h, channels, buffer, offset):
    """Turn a frame payload into the same arrays raw frames always produced:
    colour (h, w, 3) uint8 RGB, depth (h, w) int16 mm. Returns None if the
    payload can't be decoded (frame is then dropped, not crashed on)."""
    if stream_type == TYPE_COLOR_RAW:
        return np.frombuffer(buffer, dtype=np.uint8, offset=offset).reshape(h, w, channels)
    if stream_type == TYPE_DEPTH_RAW:
        return np.frombuffer(buffer, dtype=np.int16, offset=offset).reshape(h, w)
    if stream_type == TYPE_COLOR_JPEG:
        jpg = np.frombuffer(buffer, dtype=np.uint8, offset=offset)
        bgr = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
        if bgr is None or bgr.shape[:2] != (h, w):
            return None
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if stream_type == TYPE_DEPTH_ZLIB:
        raw = zlib.decompress(memoryview(buffer)[offset:])
        if len(raw) != w * h * 2:
            return None
        delta = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
        return np.cumsum(delta, axis=1, dtype=np.uint16).view(np.int16)
    return None

COLOR_ENDPOINT = "tcp://host.docker.internal:5550"
DEPTH_ENDPOINT = "tcp://host.docker.internal:5551"


@dataclass
class Frame:
    index: int
    width: int
    height: int
    data: np.ndarray

@dataclass
class DualFrame: 
    depth_frame: Frame
    color_frame: Frame


class CameraStream:
    """
    Runs color and depth ZMQ receivers on background threads.
    Frames are always available via latest_color and latest_depth,
    or you can register callbacks to be notified on every new frame.

    Usage:
        stream = CameraStream()
        stream.start()

        # Option 1 — poll latest frame whenever you need it
        frame = stream.latest_color
        frame = stream.latest_depth

        # Option 2 — register a callback
        stream.on_color(lambda f: print(f.index))
        stream.on_depth(lambda f: print(f.index))

        # Option 3 — block until the next frame arrives
        frame = stream.wait_for_color()
        frame = stream.wait_for_depth()

        stream.stop()

    Can also be used as a context manager:
        with CameraStream() as stream:
            frame = stream.latest_color
    """

    def __init__(
        self,
        color_endpoint: str = COLOR_ENDPOINT,
        depth_endpoint: str = DEPTH_ENDPOINT,
    ):
        self._color_endpoint = color_endpoint
        self._depth_endpoint = depth_endpoint

        self._latest_color: Optional[Frame] = None
        self._latest_depth: Optional[Frame] = None

        self._color_lock = threading.Lock()
        self._depth_lock = threading.Lock()

        # Events so wait_for_color/depth can block until first frame
        self._color_event = threading.Event()
        self._depth_event = threading.Event()

        self._color_callbacks: list[Callable[[Frame], None]] = []
        self._depth_callbacks: list[Callable[[Frame], None]] = []

        self._stop_event = threading.Event()

        self._color_thread = threading.Thread(
            target=self._receive_loop,
            args=(self._color_endpoint, "COLOR"),
            daemon=True,
            name="camera-color",
        )
        self._depth_thread = threading.Thread(
            target=self._receive_loop,
            args=(self._depth_endpoint, "DEPTH"),
            daemon=True,
            name="camera-depth",
        )

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def start(self) -> "CameraStream":
        self._stop_event.clear()
        self._color_thread.start()
        self._depth_thread.start()
        return self

    def stop(self):
        self._stop_event.set()
        self._color_thread.join(timeout=2)
        self._depth_thread.join(timeout=2)

    @property
    def latest_color(self) -> Optional[Frame]:
        """Most recent color frame, or None if none received yet."""
        with self._color_lock:
            return self._latest_color

    @property
    def latest_depth(self) -> Optional[Frame]:
        """Most recent depth frame, or None if none received yet."""
        with self._depth_lock:
            return self._latest_depth

    def wait_for_color(self, timeout: float = 5.0) -> Optional[Frame]:
        """Block until a color frame is available and return it."""
        self._color_event.wait(timeout=timeout)
        return self.latest_color

    def wait_for_depth(self, timeout: float = 5.0) -> Optional[Frame]:
        """Block until a depth frame is available and return it."""
        self._depth_event.wait(timeout=timeout)
        return self.latest_depth

    def on_color(self, callback: Callable[[Frame], None]):
        """Register a callback fired on every new color frame."""
        self._color_callbacks.append(callback)

    def on_depth(self, callback: Callable[[Frame], None]):
        """Register a callback fired on every new depth frame."""
        self._depth_callbacks.append(callback)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    @staticmethod
    def _recv_frame(sock, flags: int = 0):
        """One frame in any supported framing → (header, buffer, offset).

        The pixel data lives in `buffer` starting at `offset` (kept as
        buffer+offset so the combined framing stays zero-copy).
        Raises zmq.Again if nothing is available (respecting `flags`/RCVTIMEO).
        """
        msg = sock.recv(flags)

        if sock.getsockopt(zmq.RCVMORE):
            # Multipart: header part + payload part (extra parts tolerated,
            # last one wins). Parts of a multipart message arrive atomically.
            payload = sock.recv()
            while sock.getsockopt(zmq.RCVMORE):
                payload = sock.recv()
            return msg, payload, 0

        if len(msg) == HEADER_SIZE:
            # Legacy: header and payload as two separate messages — the pair
            # follows on the wire (RCVTIMEO bounds the wait).
            payload = sock.recv()
            return msg, payload, 0

        # Combined: header + payload in one message.
        return msg[:HEADER_SIZE], msg, HEADER_SIZE

    def _receive_loop(self, endpoint: str, label: str):
        ctx  = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVTIMEO, 500)   # ms — unblocks to check stop_event
        # Small receive queue: at most ~one frame waits here; anything that
        # does queue is skipped by the drain below. (No CONFLATE — see module
        # docstring; it aborts the process on multipart input.)
        sock.setsockopt(zmq.RCVHWM, 2)
        sock.connect(endpoint)
        sock.setsockopt(zmq.SUBSCRIBE, b"")

        is_color = label == "COLOR"

        try:
            while not self._stop_event.is_set():
                try:
                    hdr_bytes, buffer, offset = self._recv_frame(sock)
                except zmq.Again:
                    continue  # timeout — loop back and check stop_event

                # Skip to the newest fully-arrived frame. With RCVHWM=2 this
                # loop rarely runs more than once; it exists so a hiccup can
                # never leave the display trailing a local backlog.
                while True:
                    try:
                        hdr_bytes, buffer, offset = self._recv_frame(sock, zmq.NOBLOCK)
                    except zmq.Again:
                        break

                if len(hdr_bytes) != HEADER_SIZE:
                    print(f"[{label}] malformed header ({len(hdr_bytes)} bytes) — dropped", flush=True)
                    continue

                stream_type, frame_index, w, h, channels, size = struct.unpack(
                    HEADER_FMT, hdr_bytes
                )

                if len(buffer) - offset != size:
                    print(
                        f"[{label}] size mismatch (header says {size}, "
                        f"payload is {len(buffer) - offset}) — dropped",
                        flush=True,
                    )
                    continue

                # Raw types: zero-copy view. Compressed types (2/3): decoded
                # here, only for the newest frame (stale ones skipped above).
                try:
                    img = decode_payload(stream_type, w, h, channels, buffer, offset)
                except Exception as e:
                    img = None
                    print(f"[{label}] decode error: {e}", flush=True)
                if img is None:
                    print(f"[{label}] could not decode frame type {stream_type} — dropped", flush=True)
                    continue

                frame = Frame(index=frame_index, width=w, height=h, data=img)

                if is_color:
                    with self._color_lock:
                        self._latest_color = frame
                    self._color_event.set()
                    for cb in self._color_callbacks:
                        try:
                            cb(frame)
                        except Exception as e:
                            print(f"[{label}] callback error: {e}")
                else:
                    with self._depth_lock:
                        self._latest_depth = frame
                    self._depth_event.set()
                    for cb in self._depth_callbacks:
                        try:
                            cb(frame)
                        except Exception as e:
                            print(f"[{label}] callback error: {e}")

        finally:
            sock.close()
            ctx.term()

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "CameraStream":
        return self.start()

    def __exit__(self, *_):
        self.stop()


# ----------------------------------------------------------------------
# Example usage
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import time

    with CameraStream() as stream:

        # Wait for first frames before doing anything
        print("Waiting for first frames...")
        stream.wait_for_color()
        stream.wait_for_depth()
        print("Streams live.")

        # Register callbacks
        stream.on_color(lambda f: print(f"[COLOR] frame {f.index}  {f.width}x{f.height}"))
        stream.on_depth(lambda f: print(f"[DEPTH] frame {f.index}  value at centre: "
                                        f"{f.data[f.height // 2, f.width // 2]} mm"))

        # Simulate a large task running while stream continues in background
        print("Running large task...")
        time.sleep(5)

        # Poll the latest frame directly after the task
        color = stream.latest_color
        depth = stream.latest_depth
        print(f"After task — latest color frame: {color.index}, depth frame: {depth.index}")