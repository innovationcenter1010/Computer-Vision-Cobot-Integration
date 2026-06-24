import zmq
import numpy as np
import struct
import threading

HEADER_FMT  = "<BIHHBBI"  # note: no space in format string
HEADER_SIZE = struct.calcsize(HEADER_FMT)

COLOR_ENDPOINT = "tcp://host.docker.internal:5550"
DEPTH_ENDPOINT = "tcp://host.docker.internal:5551"

ctx = zmq.Context()

def receive_stream(endpoint, label):
    sock = ctx.socket(zmq.SUB)
    sock.connect(endpoint)
    sock.setsockopt(zmq.SUBSCRIBE, b"")

    while True:
        hdr_bytes = sock.recv()
        payload   = sock.recv()

        stream_type, frame_index, w, h, channels, size = struct.unpack(HEADER_FMT, hdr_bytes)

        if stream_type == 0:  # color — uint8 RGB
            img = np.frombuffer(payload, dtype=np.uint8).reshape(h, w, channels)
        else:                  # depth — int16 mm
            img = np.frombuffer(payload, dtype=np.int16).reshape(h, w)

        print(f"[{label}] frame {frame_index}  {w}x{h}  {len(payload)} bytes")

# Run both streams concurrently
color_thread = threading.Thread(target=receive_stream, args=(COLOR_ENDPOINT, "COLOR"), daemon=True)
depth_thread = threading.Thread(target=receive_stream, args=(DEPTH_ENDPOINT, "DEPTH"), daemon=True)

color_thread.start()
depth_thread.start()

color_thread.join()
depth_thread.join()