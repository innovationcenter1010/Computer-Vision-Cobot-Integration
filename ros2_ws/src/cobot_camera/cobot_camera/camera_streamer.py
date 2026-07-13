import time

import zmq
import numpy as np
import struct
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable

HEADER_FMT  = "<BIHHBI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

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

    def _receive_loop(self, endpoint: str, label: str):
        ctx  = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVTIMEO, 500)   # ms — unblocks to check stop_event
        sock.connect(endpoint)
        sock.setsockopt(zmq.SUBSCRIBE, b"")

        is_color = label == "COLOR"

        skipped_total = 0
        last_skip_report = time.monotonic()

        try:
            while not self._stop_event.is_set():
                try:
                    hdr_bytes = sock.recv()
                    payload   = sock.recv()
                except zmq.Again:
                    continue  # timeout — loop back and check stop_event

                # ---- drain-to-latest ----
                # Keep ONLY the newest frame pair. Without this the loop decodes
                # every queued frame in FIFO order, and ZMQ's default HWM
                # (1000 msgs) can hold ~30s of video — the consumer then trails
                # reality by the full backlog, forever.
                skipped = 0
                while True:
                    try:
                        newer_hdr = sock.recv(zmq.NOBLOCK)
                        newer_payload = sock.recv()  # its pair follows (RCVTIMEO applies)
                        hdr_bytes, payload = newer_hdr, newer_payload
                        skipped += 1
                    except zmq.Again:
                        break

                if skipped:
                    skipped_total += skipped
                    now = time.monotonic()
                    if now - last_skip_report > 5.0:
                        print(
                            f"[{label}] running behind: skipped {skipped_total} "
                            f"stale frames in the last {now - last_skip_report:.0f}s",
                            flush=True,
                        )
                        skipped_total = 0
                        last_skip_report = now

                # ---- pairing guards ----
                # If a header/payload pair ever desyncs (e.g. a lone header at
                # a timeout boundary), discard until a valid pair lines up.
                if len(hdr_bytes) != HEADER_SIZE:
                    print(f"[{label}] malformed header ({len(hdr_bytes)} bytes) — resyncing", flush=True)
                    continue

                stream_type, frame_index, w, h, channels, size = struct.unpack(
                    HEADER_FMT, hdr_bytes
                )

                if len(payload) != size:
                    print(
                        f"[{label}] header/payload desync "
                        f"(expected {size} bytes, got {len(payload)}) — resyncing",
                        flush=True,
                    )
                    continue

                if stream_type == 0:
                    img = np.frombuffer(payload, dtype=np.uint8).reshape(h, w, channels)
                else:
                    img = np.frombuffer(payload, dtype=np.int16).reshape(h, w)

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