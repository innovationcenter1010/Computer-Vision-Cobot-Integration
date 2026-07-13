"""cobot_node — the high-level controller. Single node, Python, pybind underneath.

Direct descendant of the legacy Cobot_Controller command_map: inputs arrive
and the controller decides which operation to run.

  in:  /vision/targets (CanTargetArray)  — a published layer triggers pick_layer
       /cobot/start | /cobot/stop | /cobot/calibrate (std_srvs/Trigger)
  out: /cobot/status (CobotStatus, latched)
       /cobot/layer_report (LayerReport)

Threading model: pick operations run in a background thread driving the
existing async Cobot choreography with asyncio.run(). Services live in their
own callback group so /cobot/stop is reachable WHILE a pick runs (requires the
MultiThreadedExecutor set up in main()). One operation at a time — a target
array arriving mid-pick is rejected with a warning.
"""

import asyncio
import json
import threading
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_srvs.srv import Trigger

from cobot_interfaces.msg import CanTargetArray, CobotStatus, LayerReport

from cobot_control.cobot import Cobot, Cobot_Operation_Status


LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CobotNode(Node):
    def __init__(self):
        super().__init__("cobot_node")

        self.declare_parameter("robot_ip", "192.168.1.31")
        self.declare_parameter("tool_head_length_mm", 620.0)
        self.declare_parameter("tool_head_xy_offset", 0.0)
        self.declare_parameter("tool_head_compression_distance_mm", 50.0)
        self.declare_parameter("trajectories_file", "")

        trajectories_file = self.get_parameter("trajectories_file").value
        if not trajectories_file:
            raise RuntimeError(
                "trajectories_file parameter is required "
                "(see cobot_bringup/config/trajectories.json)"
            )
        trajectories = json.loads(Path(trajectories_file).read_text(encoding="utf-8"))

        # Same shape the legacy Cobot consumed from config.JSON.
        cobot_configs = {
            "cobot_ip": self.get_parameter("robot_ip").value,
            "tool_head_length_mm": self.get_parameter("tool_head_length_mm").value,
            "tool_head_xy_offset": self.get_parameter("tool_head_xy_offset").value,
            "tool_head_compression_distance_mm": self.get_parameter(
                "tool_head_compression_distance_mm"
            ).value,
            "cobot_place_trajectories": trajectories,
        }

        self._status_pub = self.create_publisher(CobotStatus, "/cobot/status", LATCHED_QOS)
        self._report_pub = self.create_publisher(LayerReport, "/cobot/layer_report", 10)

        self._operation_lock = threading.Lock()  # one operation at a time
        self._operation_thread: threading.Thread | None = None
        self._stop_requested = threading.Event()

        self.get_logger().info(f"Connecting to Motoman at {cobot_configs['cobot_ip']} ...")
        try:
            self._cobot = Cobot(cobot_configs)
            self._publish_status(CobotStatus.STATE_IDLE, "connected")
        except Exception as e:  # noqa: BLE001 — node must come up even if robot is off
            self._cobot = None
            self.get_logger().error(f"Cobot connection failed: {e}")
            self._publish_status(CobotStatus.STATE_DISCONNECTED, str(e))

        # Services in their own group: stop must interleave with a running pick.
        service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "/cobot/start", self._on_start, callback_group=service_group)
        self.create_service(Trigger, "/cobot/stop", self._on_stop, callback_group=service_group)
        self.create_service(
            Trigger, "/cobot/calibrate", self._on_calibrate, callback_group=service_group
        )

        self.create_subscription(CanTargetArray, "/vision/targets", self._on_targets, 10)

        # The controller asks vision to sense when it needs data (§5 of the
        # architecture doc): start → scan; tier-1 checks per pick come later.
        self._scan_client = self.create_client(Trigger, "/vision/scan")

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------

    def _publish_status(self, state: int, detail: str = ""):
        msg = CobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.detail = detail
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # High-level controller: decide what to run based on input
    # ------------------------------------------------------------------

    def _on_targets(self, msg: CanTargetArray):
        if self._cobot is None:
            self.get_logger().error("Targets received but robot is not connected")
            return

        if not msg.targets:
            self.get_logger().info("Empty target layer received — nothing to pick")
            return

        if not self._operation_lock.acquire(blocking=False):
            self.get_logger().warn(
                f"Rejecting layer of {len(msg.targets)} targets: operation already running"
            )
            return

        self._stop_requested.clear()
        self._operation_thread = threading.Thread(
            target=self._run_pick_layer, args=(list(msg.targets),), daemon=True
        )
        self._operation_thread.start()

    def _run_pick_layer(self, targets: list):
        """Owns self._operation_lock (acquired by _on_targets); releases on exit."""
        try:
            self._publish_status(CobotStatus.STATE_PICKING, f"{len(targets)} targets")

            report = LayerReport()
            report.overall_ok = True

            for index, can in enumerate(targets):
                if self._stop_requested.is_set():
                    self.get_logger().warn(f"Stop requested — abandoning layer at can {index}")
                    report.overall_ok = False
                    break

                self.get_logger().info(
                    f"Picking can {index + 1}/{len(targets)} at "
                    f"({can.real_x:.1f}, {can.real_y:.1f}, rz={can.real_rz:.1f})"
                )

                # Drive the existing async choreography from this worker thread.
                operation_report = asyncio.run(self._cobot.pick_and_place(can))

                if operation_report.overall_status == Cobot_Operation_Status.job_success:
                    report.successes += 1
                else:
                    report.failures += 1
                    report.overall_ok = False
                    self.get_logger().warn(f"Pick {index + 1} failed:\n{operation_report}")

            report.header.stamp = self.get_clock().now().to_msg()
            self._report_pub.publish(report)
            self._publish_status(
                CobotStatus.STATE_IDLE,
                f"layer done: {report.successes} ok / {report.failures} failed",
            )

        except Exception as e:  # noqa: BLE001 — a failed layer must not kill the node
            self.get_logger().error(f"pick_layer crashed: {e}")
            self._publish_status(CobotStatus.STATE_ERROR, str(e))

        finally:
            self._operation_lock.release()

    # ------------------------------------------------------------------
    # Trigger services
    # ------------------------------------------------------------------

    def _on_start(self, request, response):
        """Kick off a cycle: ask vision to scan; targets arrive via topic."""
        if self._cobot is None:
            response.success = False
            response.message = "robot not connected"
            return response

        if not self._scan_client.service_is_ready():
            response.success = False
            response.message = "/vision/scan not available"
            return response

        self._scan_client.call_async(Trigger.Request())
        response.success = True
        response.message = "scan requested — picking starts when targets arrive"
        return response

    def _on_stop(self, request, response):
        self._stop_requested.set()
        self._publish_status(CobotStatus.STATE_STOPPING, "stop requested")
        response.success = True
        response.message = "stop requested (takes effect after the current can)"
        return response

    def _on_calibrate(self, request, response):
        # TODO: calibration routine (legacy defined the command but no handler).
        response.success = False
        response.message = "calibrate not implemented yet"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
