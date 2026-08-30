"""HeartbeatPublisher — one timer + one publisher per node.

    from cobot_health.heartbeat import HeartbeatPublisher

    class MyNode(Node):
        def __init__(self):
            ...
            self._hb = HeartbeatPublisher(self, self._health_report)   # last line of __init__

        def _health_report(self) -> tuple[int, str, dict]:
            return Heartbeat.LEVEL_OK, "streaming", {"color_hz": 14.9, "connected": True}

The report callback runs on an executor thread and MUST NOT block: read
cached numbers only. Exceptions inside it are caught and published as
LEVEL_ERROR so a metrics bug can never kill the node.

The timer lives in its own MutuallyExclusiveCallbackGroup so, on a
MultiThreadedExecutor, it is never queued behind a long-running callback
(e.g. the 120 s calibration handlers in localization_node). On a
single-threaded executor the group is harmless.
"""

import threading
import time
from typing import Callable

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from cobot_interfaces.msg import Heartbeat, KeyValue

HEARTBEAT_TOPIC = "/health/heartbeat"
SUMMARY_TOPIC = "/health/summary"

HEARTBEAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

DEFAULT_PERIOD_S = 2.0

ReportFn = Callable[[], tuple[int, str, dict]]


def format_metric(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


class HeartbeatPublisher:
    def __init__(
        self,
        node: Node,
        report: ReportFn,
        period_s: float | None = None,
        name: str | None = None,
    ):
        self._node = node
        self._report = report
        self._name = name or node.get_name()

        if period_s is None:
            if not node.has_parameter("heartbeat_period_s"):
                node.declare_parameter("heartbeat_period_s", DEFAULT_PERIOD_S)
            period_s = float(node.get_parameter("heartbeat_period_s").value)
        self._period = max(float(period_s), 0.2)

        self._pub = node.create_publisher(Heartbeat, HEARTBEAT_TOPIC, HEARTBEAT_QOS)
        self._group = MutuallyExclusiveCallbackGroup()
        self._timer = node.create_timer(self._period, self._tick, callback_group=self._group)

    @property
    def period_s(self) -> float:
        return self._period

    def _tick(self):
        try:
            level, detail, metrics = self._report()
        except Exception as e:  # noqa: BLE001 — a metrics bug must not kill the node
            level, detail, metrics = Heartbeat.LEVEL_ERROR, f"health report failed: {e}", {}

        msg = Heartbeat()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.node_name = self._name
        msg.level = int(level)
        msg.detail = str(detail)
        msg.period_s = float(self._period)
        msg.metrics = [
            KeyValue(key=str(k), value=format_metric(v)) for k, v in (metrics or {}).items()
        ]
        self._pub.publish(msg)


class RateMeter:
    """Count events; report (hz, age_s) over the window since the last read.

    Thread-safe: tick() from any thread, read() from the heartbeat timer.
    hz is averaged over the read window (== the heartbeat period); age_s is
    None until the first tick, so callers can tell "never" from "old".
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._count = 0
        self._total = 0
        self._window_start = time.monotonic()
        self._last: float | None = None

    def tick(self, n: int = 1):
        with self._lock:
            self._count += n
            self._total += n
            self._last = time.monotonic()

    def read(self) -> tuple[float, float | None]:
        with self._lock:
            now = time.monotonic()
            hz = self._count / max(now - self._window_start, 1e-6)
            age = None if self._last is None else now - self._last
            self._count = 0
            self._window_start = now
            return hz, age

    @property
    def total(self) -> int:
        with self._lock:
            return self._total

    def age(self) -> float | None:
        with self._lock:
            return None if self._last is None else time.monotonic() - self._last
