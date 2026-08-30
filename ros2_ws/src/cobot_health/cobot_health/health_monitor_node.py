"""health_monitor — aggregates per-node heartbeats into /health/summary.

Subscribes /health/heartbeat (every node publishes one every ~2 s, see
cobot_health.heartbeat) and publishes a latched HealthSummary at
summary_period_s. Per node:

    never seen                          -> LEVEL_UNKNOWN (age -1)
    age > stale_factor * hb.period_s    -> LEVEL_STALE  (last metrics kept)
    otherwise                           -> the node's own level/detail/metrics

expected_nodes lists what MUST appear; anything else that publishes is
appended so a new node shows up without a config change. Level transitions
are logged once (not every tick).
"""

import threading
import time

import rclpy
from rclpy.node import Node

from cobot_interfaces.msg import HealthSummary, Heartbeat, NodeHealth

from cobot_health.heartbeat import (
    HEARTBEAT_QOS,
    HEARTBEAT_TOPIC,
    LATCHED_QOS,
    SUMMARY_TOPIC,
    HeartbeatPublisher,
)

DEFAULT_EXPECTED = [
    "camera_node",
    "yolo_node",
    "localization_node",
    "cobot_node",
    "task_manager",
    "web_node",
    "health_monitor",
]

LEVEL_NAMES = {
    NodeHealth.LEVEL_OK: "OK",
    NodeHealth.LEVEL_WARN: "WARN",
    NodeHealth.LEVEL_ERROR: "ERROR",
    NodeHealth.LEVEL_STALE: "STALE",
    NodeHealth.LEVEL_UNKNOWN: "UNKNOWN",
}

# Severity order for the summary level: STALE/UNKNOWN are as bad as ERROR.
SEVERITY = {
    NodeHealth.LEVEL_OK: 0,
    NodeHealth.LEVEL_WARN: 1,
    NodeHealth.LEVEL_ERROR: 2,
    NodeHealth.LEVEL_STALE: 2,
    NodeHealth.LEVEL_UNKNOWN: 2,
}
SEVERITY_TO_LEVEL = {0: Heartbeat.LEVEL_OK, 1: Heartbeat.LEVEL_WARN, 2: Heartbeat.LEVEL_ERROR}


class HealthMonitorNode(Node):
    def __init__(self):
        super().__init__("health_monitor")

        self.declare_parameter("expected_nodes", DEFAULT_EXPECTED)
        self.declare_parameter("stale_factor", 3.0)
        self.declare_parameter("summary_period_s", 1.0)

        self._lock = threading.Lock()
        self._seen: dict[str, tuple[float, Heartbeat]] = {}
        self._last_level: dict[str, int] = {}
        self._stale_count = 0

        self.create_subscription(Heartbeat, HEARTBEAT_TOPIC, self._on_heartbeat, HEARTBEAT_QOS)
        self._summary_pub = self.create_publisher(HealthSummary, SUMMARY_TOPIC, LATCHED_QOS)

        period = float(self.get_parameter("summary_period_s").value)
        self.create_timer(max(period, 0.2), self._publish_summary)

        self._hb = HeartbeatPublisher(self, self._health_report)
        self.get_logger().info("health_monitor ready")

    def _on_heartbeat(self, msg: Heartbeat):
        with self._lock:
            self._seen[msg.node_name] = (time.monotonic(), msg)

    def _publish_summary(self):
        now = time.monotonic()
        stale_factor = float(self.get_parameter("stale_factor").value)
        with self._lock:
            seen = dict(self._seen)

        names = list(self.get_parameter("expected_nodes").value)
        names += [n for n in seen if n not in names]

        out = HealthSummary()
        out.header.stamp = self.get_clock().now().to_msg()
        worst = 0
        stale = 0

        for name in names:
            nh = NodeHealth()
            nh.node_name = name
            if name not in seen:
                nh.level = NodeHealth.LEVEL_UNKNOWN
                nh.detail = "no heartbeat received"
                nh.age_s = -1.0
            else:
                rx, hb = seen[name]
                age = now - rx
                nh.age_s = float(age)
                nh.metrics = list(hb.metrics)
                nh.detail = hb.detail
                if age > stale_factor * max(float(hb.period_s), 0.5):
                    nh.level = NodeHealth.LEVEL_STALE
                    nh.detail = f"stale ({age:.1f}s): {hb.detail}"
                    stale += 1
                else:
                    nh.level = int(hb.level)

            self._log_transition(name, nh.level)
            worst = max(worst, SEVERITY.get(nh.level, 2))
            out.nodes.append(nh)

        self._stale_count = stale
        out.level = SEVERITY_TO_LEVEL[worst]
        self._summary_pub.publish(out)

    def _log_transition(self, name: str, level: int):
        prev = self._last_level.get(name)
        if prev == level:
            return
        self._last_level[name] = level
        if prev is None and level == NodeHealth.LEVEL_UNKNOWN:
            return  # startup: nothing seen yet, not worth a log line
        line = f"{name}: {LEVEL_NAMES.get(prev, '?')} -> {LEVEL_NAMES.get(level, '?')}"
        if SEVERITY.get(level, 2) >= 1:
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)

    def _health_report(self):
        with self._lock:
            tracked = len(self._seen)
        return (
            Heartbeat.LEVEL_OK,
            f"tracking {tracked} nodes",
            {"tracked_nodes": tracked, "stale_nodes": self._stale_count},
        )


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
