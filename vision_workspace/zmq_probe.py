"""Delay probe: prints colour frames as they ARRIVE inside Docker.

Compare the frame numbers printed here with the 'color frameIndex' lines
in the AstraStreamer window at the same moment. Same number (within a few)
= the Windows -> Docker crossing is fast. Far behind = the delay is there.

Run (system can be running or not):
  docker exec -it <container-name> python3 /ws/vision_workspace/zmq_probe.py
Stops by itself after 30 seconds (or press Ctrl+C).
"""
import struct
import time

import numpy as np
import zmq
try:
    import cv2
except ImportError:
    cv2 = None

HEADER_FMT = "<BIHHBI"          # type, frame_index, width, height, channels, size
HEADER_SIZE = struct.calcsize(HEADER_FMT)
ENDPOINT = "tcp://host.docker.internal:5550"

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.connect(ENDPOINT)
sock.setsockopt(zmq.SUBSCRIBE, b"")
print(f"listening on {ENDPOINT} ... (30 s)", flush=True)

start = time.time()
count = 0
while time.time() - start < 30:
    try:
        hdr = sock.recv()
        payload = sock.recv() if sock.getsockopt(zmq.RCVMORE) else b""
    except zmq.Again:
        print("no frame for 2 s - is AstraStreamer running?", flush=True)
        continue
    except KeyboardInterrupt:
        break
    if len(hdr) != HEADER_SIZE:
        continue
    _type, idx, w, h, ch, size = struct.unpack(HEADER_FMT, hdr)
    count += 1
    # centre pixel brightness, so covering the lens shows up here too
    bright = -1
    if _type == 2 and cv2 is not None:          # compressed (JPEG) colour
        img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            bright = int(img[h // 2, w // 2].mean())
    elif _type == 0:                             # raw colour
        mid = (h // 2 * w + w // 2) * ch
        if len(payload) >= mid + ch:
            bright = sum(payload[mid:mid + ch]) // max(ch, 1)
    kind = "jpeg" if _type == 2 else "raw " if _type == 0 else f"t{_type}"
    print(f"{time.strftime('%H:%M:%S')}  arrived frame {idx:6d}  {kind} {len(payload) // 1024:4d} KB"
          f"  centre brightness {bright:3d}  ({count / (time.time() - start):.1f} frames/s)", flush=True)

print(f"done: {count} frames in {time.time() - start:.0f} s", flush=True)
