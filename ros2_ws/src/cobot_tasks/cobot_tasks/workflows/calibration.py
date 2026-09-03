"""calibration — camera→robot extrinsic calibration (movement loop only;
every millimetre of math lives in localization_node).

  /vision/calibration_begin(mode, model) → per grid pose: /cobot/move_to_pose,
  settle, /vision/calibration_sample(x, y, z) → /vision/calibration_compute.

Four registered variants share this code — two MANUAL ones (the operator
clicks the marker centre on the dashboard feed at every pose) and their
AUTOMATIC twins (the detector finds the marker; no operator input, so the
grid is denser for a better fit):

  calibration               manual · calibration_grid_size / calibration_z_levels_mm
  auto_calibration          automatic · auto_calibration_grid_size / _z_levels_mm
  startup_calibration       manual · startup_calibration_* (task_manager runs the
                            configured startup workflow at boot, see
                            startup_calibration_workflow)
  auto_startup_calibration  automatic · auto_startup_calibration_*

How the marker pixel is found is pinned per RUN in the CalibrationBegin
request (mode / model_id / class_id / samples_per_pose), so the twins coexist
without touching localization's manual_dot_selection parameter. Every sample
is echoed on /vision/calibration_marker for the dashboard's verification
overlay.

All variants read the shared x/y ranges and orientation; [0,0] ranges mean
"not configured" and check_ready refuses (the arm must never drive an
unmeasured grid). No rclpy here: everything goes through the WorkflowContext.
"""

from __future__ import annotations

from std_srvs.srv import Trigger

from cobot_interfaces.action import MoveToPose
from cobot_interfaces.msg import TaskStatus
from cobot_interfaces.srv import CalibrationBegin, CalibrationSample

from cobot_tasks.system_state import CAP_OPERATE
from cobot_tasks.workflows.base import ClientSpec, ParamSpec, Workflow

MOVE_TIMEOUT_SEC = 120.0        # one calibration grid move
# calibration_begin lazy-loads the marker model on the vision side.
CAL_BEGIN_TIMEOUT_SEC = 120.0
CAL_COMPUTE_TIMEOUT_SEC = 60.0

GRID_NOT_CONFIGURED = (
    "calibration grid not configured — set calibration_x_range_mm / "
    "calibration_y_range_mm / {z_param} to safe bounds"
)

MODE_MANUAL = CalibrationBegin.Request.MODE_MANUAL
MODE_AUTOMATIC = CalibrationBegin.Request.MODE_AUTOMATIC


def spread(lo: float, hi: float, n: int) -> list[float]:
    """linspace by hand — no numpy in cobot_tasks."""
    if n <= 1:
        return [(lo + hi) / 2.0]
    step = (hi - lo) / (n - 1)
    return [lo + i * step for i in range(n)]


class CalibrationWorkflow(Workflow):
    id = "calibration"
    label = "Camera calibration (full grid · manual click)"
    description = (
        "Drive the marker through the full calibration grid (with the active "
        "tool); you click the marker centre on the color feed at every pose, "
        "then the camera→robot transform is refitted."
    )
    requires = CAP_OPERATE  # moves the arm AND uses the camera (variants inherit)
    grid_size_param = "calibration_grid_size"
    z_levels_param = "calibration_z_levels_mm"
    mode = MODE_MANUAL
    # Automatic variants: which detector model / class finds the marker and
    # how many reads are taken per pose (params, so a retrained model is a
    # yaml change).
    model_param: str | None = None
    class_param: str | None = None
    reads_param: str | None = None

    params = (
        ParamSpec("calibration_x_range_mm", [0.0, 0.0], "[min, max] robot X of the grid; [0,0] = unset"),
        ParamSpec("calibration_y_range_mm", [0.0, 0.0], "[min, max] robot Y of the grid; [0,0] = unset"),
        ParamSpec("calibration_z_levels_mm", [0.0], "Z heights (floats); [0.0] = unset"),
        ParamSpec("calibration_grid_size", [3, 2], "nx, ny poses per z level"),
        ParamSpec("calibration_orientation_deg", [180.0, 0.0, 0.0], "rx, ry, rz for every pose"),
        ParamSpec("calibration_settle_sec", 0.5, "wait after each move before sampling"),
        ParamSpec("calibration_sample_timeout_sec", 30.0,
                  "budget per calibration_sample; must exceed localization's manual click timeout"),
    )
    clients = (
        ClientSpec("move", "action", MoveToPose, "/cobot/move_to_pose"),
        ClientSpec("cal_begin", "service", CalibrationBegin, "/vision/calibration_begin"),
        ClientSpec("cal_sample", "service", CalibrationSample, "/vision/calibration_sample"),
        ClientSpec("cal_compute", "service", Trigger, "/vision/calibration_compute"),
    )

    # ------------------------------------------------------------------

    @property
    def automatic(self) -> bool:
        return self.mode == MODE_AUTOMATIC

    def grid_configured(self, ctx) -> bool:
        """True once the shared x/y ranges and this variant's z levels are set."""
        x = list(ctx.get_param("calibration_x_range_mm"))
        y = list(ctx.get_param("calibration_y_range_mm"))
        z = list(ctx.get_param(self.z_levels_param))
        return not (x == [0.0, 0.0] or y == [0.0, 0.0] or z == [0.0])

    def not_configured_reason(self) -> str:
        return GRID_NOT_CONFIGURED.format(z_param=self.z_levels_param)

    def check_ready(self, ctx) -> tuple[bool, str]:
        if not self.grid_configured(ctx):
            return False, self.not_configured_reason()
        return super().check_ready(ctx)

    def poses(self, ctx) -> list[tuple[float, float, float]]:
        """(x, y, z) grid over the shared x/y ranges at this variant's
        z levels and density."""
        x_min, x_max = ctx.get_param("calibration_x_range_mm")
        y_min, y_max = ctx.get_param("calibration_y_range_mm")
        z_levels = list(ctx.get_param(self.z_levels_param))
        nx, ny = (int(v) for v in ctx.get_param(self.grid_size_param))
        return [
            (x, y, z)
            for z in z_levels
            for y in spread(y_min, y_max, ny)
            for x in spread(x_min, x_max, nx)
        ]

    def begin_request(self, ctx) -> CalibrationBegin.Request:
        """How localization must find the marker for this whole run."""
        req = CalibrationBegin.Request()
        req.mode = int(self.mode)
        if self.automatic:
            req.model_id = str(ctx.get_param(self.model_param)) if self.model_param else ""
            req.class_id = int(ctx.get_param(self.class_param)) if self.class_param else -1
            req.samples_per_pose = int(ctx.get_param(self.reads_param)) if self.reads_param else 0
        return req

    # ------------------------------------------------------------------

    def run(self, ctx) -> None:
        label = self.label
        poses = self.poses(ctx)
        rx, ry, rz = ctx.get_param("calibration_orientation_deg")
        settle = float(ctx.get_param("calibration_settle_sec"))
        sample_timeout = float(ctx.get_param("calibration_sample_timeout_sec"))

        ctx.publish_status(
            TaskStatus.STATE_CALIBRATING, f"{label} starting: {len(poses)} poses", 0, len(poses)
        )

        # Vision clears its sample buffer, pins the marker-finding mode for
        # the run and (automatic) warms up the marker model.
        begin = ctx.call("cal_begin", self.begin_request(ctx), CAL_BEGIN_TIMEOUT_SEC)
        if not begin.success:
            ctx.publish_status(TaskStatus.STATE_ERROR, f"{label} begin refused: {begin.message}")
            return

        collected = 0
        for index, (x, y, z) in enumerate(poses):
            if ctx.stop_requested():
                ctx.publish_status(
                    TaskStatus.STATE_IDLE,
                    f"{label} aborted by stop at pose {index + 1}/{len(poses)}",
                )
                return

            pose_label = f"pose {index + 1}/{len(poses)} ({x:.0f}, {y:.0f}, {z:.0f})"
            ctx.publish_status(
                TaskStatus.STATE_CALIBRATING,
                f"{pose_label} — {collected} samples so far",
                index + 1,
                len(poses),
            )

            # tool_number -1 = the ACTIVE tool file: the grid is commanded in
            # it and localization adds that tool's marker_offset_mm.
            goal = MoveToPose.Goal(x=float(x), y=float(y), z=float(z),
                                   rx=float(rx), ry=float(ry), rz=float(rz),
                                   tool_number=-1)
            move = ctx.run_action("move", goal, MOVE_TIMEOUT_SEC)
            if not move.success:
                ctx.log.warn(f"{pose_label}: move failed — skipped ({move.message})")
                continue

            ctx.sleep(settle)  # let vibration die before vision measures

            sample = ctx.call(
                "cal_sample",
                CalibrationSample.Request(x=float(x), y=float(y), z=float(z)),
                sample_timeout,
            )
            if sample.success:
                collected = sample.sample_count
            else:
                ctx.log.warn(f"{pose_label}: no sample ({sample.message})")

        ctx.publish_status(
            TaskStatus.STATE_CALIBRATING,
            f"{label}: grid done, {collected} samples — computing transform",
            len(poses),
            len(poses),
        )

        compute = ctx.call("cal_compute", Trigger.Request(), CAL_COMPUTE_TIMEOUT_SEC)
        if compute.success:
            ctx.publish_status(TaskStatus.STATE_IDLE, f"{label} complete: {compute.message}")
        else:
            ctx.publish_status(TaskStatus.STATE_ERROR, f"{label} failed: {compute.message}")


class StartupCalibrationWorkflow(CalibrationWorkflow):
    id = "startup_calibration"
    label = "Camera calibration (startup grid · manual click)"
    description = (
        "Small extrinsic grid for a quick re-fit against camera drift; the "
        "operator clicks the marker at each pose. Selectable as the boot-time "
        "run via task_manager.startup_calibration_workflow."
    )
    grid_size_param = "startup_calibration_grid_size"
    z_levels_param = "startup_calibration_z_levels_mm"

    # Shared x/y/orientation/settle/timeout params come from the parent
    # (the node declares each name once).
    params = CalibrationWorkflow.params + (
        ParamSpec("startup_calibration_grid_size", [2, 2], "nx, ny for the boot grid"),
        ParamSpec("startup_calibration_z_levels_mm", [-200.0, 200.0], "Z heights for the boot grid"),
    )


# ---------------------------------------------------------------------------
# Automatic twins — the green-light detector replaces the operator click, so
# the grids are denser (nothing to wait for per pose but a few inferences).
# ---------------------------------------------------------------------------

AUTO_PARAMS = (
    ParamSpec("auto_calibration_model_id", "green_light_v2",
              "detector model (models.yaml id) that finds the marker in automatic runs"),
    ParamSpec("auto_calibration_class_id", -1,
              "keep only this class of the model's detections; -1 = any (single-class models)"),
    ParamSpec("auto_calibration_samples_per_pose", 5,
              "detector reads per pose (median of the hits; majority must see the marker)"),
)


class AutoCalibrationWorkflow(CalibrationWorkflow):
    id = "auto_calibration"
    label = "Camera calibration (dense grid · automatic)"
    description = (
        "Automatic twin of the full calibration: the green-light detector finds "
        "the marker at every pose (no clicking), so a denser grid is driven for "
        "a better fit. Each detection is shown on the color feed for verification."
    )
    grid_size_param = "auto_calibration_grid_size"
    z_levels_param = "auto_calibration_z_levels_mm"
    mode = MODE_AUTOMATIC
    model_param = "auto_calibration_model_id"
    class_param = "auto_calibration_class_id"
    reads_param = "auto_calibration_samples_per_pose"

    params = CalibrationWorkflow.params + AUTO_PARAMS + (
        ParamSpec("auto_calibration_grid_size", [4, 3], "nx, ny per z level (denser than manual)"),
        ParamSpec("auto_calibration_z_levels_mm", [0.0], "Z heights (floats); [0.0] = unset"),
    )


class AutoStartupCalibrationWorkflow(CalibrationWorkflow):
    id = "auto_startup_calibration"
    label = "Camera calibration (startup grid · automatic)"
    description = (
        "Automatic twin of the startup calibration: detector-found marker over a "
        "denser boot grid. Set task_manager.startup_calibration_workflow to this "
        "id to run it unattended at launch."
    )
    grid_size_param = "auto_startup_calibration_grid_size"
    z_levels_param = "auto_startup_calibration_z_levels_mm"
    mode = MODE_AUTOMATIC
    model_param = "auto_calibration_model_id"
    class_param = "auto_calibration_class_id"
    reads_param = "auto_calibration_samples_per_pose"

    params = CalibrationWorkflow.params + AUTO_PARAMS + (
        ParamSpec("auto_startup_calibration_grid_size", [3, 3], "nx, ny for the automatic boot grid"),
        ParamSpec("auto_startup_calibration_z_levels_mm", [-200.0, 200.0],
                  "Z heights for the automatic boot grid"),
    )
