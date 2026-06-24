import cv2
import numpy as np
import os
from datetime import datetime

import asyncio

from python.cobot_application.shared_data import Py_network, Config, package_ID, P2P_command, vision_commands, cobot_commands, camera_commands
from cobot_application.vision_system.run_operation_scan import run_YOLO


class Vision_Controller():
    def __init__(self,network: Py_network,configurations):
        self.network = network
        self.id = package_ID.vision_id
        self.command_queue = network.command_queues[self.id.value]
        self.configurations = configurations

        self.command_map = {

            vision_commands.detect.value: self.scan,
            vision_commands.convert_to_jpeg.value: self.convert_to_jpeg

        }

        print("vision system awake",flush=True)

    async def loop(self):
        while True:
            print("Vision waiting for command...", flush=True)

            #wait for a command to come in
            command: P2P_command = await self.command_queue.get()

            #map the command from the command map to run the actual function
            handle = self.command_map.get(command.message.value)
            
            #only run the command if it is properly mapped
            if handle: await handle(command)

            else: raise RuntimeError("Unknown command:", command.message)



    async def scan(self, command: P2P_command):

        print("Starting YOLO scan...", flush=True)

        detections = await asyncio.to_thread(
            run_YOLO,
            None,
            self.configurations.as_dict()["vision_system"]
        )

        print("YOLO scan complete", flush=True)
        print("Detections:", detections, flush=True)


        layer_command = P2P_command(
            sender_ID=self.id,
            receiver_ID=package_ID.cobot_id,
            message=cobot_commands.pick_layer,
            value=detections
        )

        await self.network.send(layer_command)

    async def convert_to_jpeg(self,command):

        output_dir="snapshots"

        os.makedirs(output_dir, exist_ok=True)

        # Extract DualFrame from command
        dual_frame = command.value

        if dual_frame is None:
            print("[snapshot] No frame data in command")
            return

        color_frame = dual_frame.color_frame
        depth_frame = dual_frame.depth_frame

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # ------------------------------------------------------------
        # Save COLOR frame
        # ------------------------------------------------------------
        if color_frame is not None and color_frame.data is not None:
            color_img = color_frame.data

            # Ensure correct type for OpenCV
            if color_img.dtype != np.uint8:
                color_img = np.clip(color_img, 0, 255).astype(np.uint8)

            color_path = os.path.join(
                output_dir,
                f"color_{timestamp}_idx{color_frame.index}.jpg"
            )

            cv2.imwrite(color_path, cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR))

        # ------------------------------------------------------------
        # Save DEPTH frame (normalize for visualization)
        # ------------------------------------------------------------
        if depth_frame is not None and depth_frame.data is not None:
            depth_img = depth_frame.data.astype(np.float32)

            # Normalize depth → 0–255 for JPEG visualization
            depth_norm = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
            depth_uint8 = depth_norm.astype(np.uint8)

            depth_path = os.path.join(
                output_dir,
                f"depth_{timestamp}_idx{depth_frame.index}.jpg"
            )

            cv2.imwrite(depth_path, depth_uint8)

        print(f"[snapshot] Saved frames at {output_dir}")








