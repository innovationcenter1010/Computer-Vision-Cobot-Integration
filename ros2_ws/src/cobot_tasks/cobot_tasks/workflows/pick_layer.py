"""pick_layer — scan the bin, then pick every target it found.

  1. /cobot/hide (Hide action, served by cobot_operation) — parks the arm at
     the remembered "hide" teach point, 100% out of the camera's view,
     BEFORE the scan runs. task_manager never talks to cobot_control
     directly for this: cobot_operation owns every remembered position and
     the busy lock that serialises them with picks (see choreography.py).
  2. /vision/scan (Scan action)  → ordered robot-frame targets (result.targets)
  3. per target: /cobot/pick_can (PickCan action) — one goal at a time, stop
     cancels the in-flight goal natively (cobot_operation halts the arm)
  4. per successful pick: /vision/verify_target ("second depth scan") — runs
     ONLY after the PickCan result, i.e. after the place: during the pick the
     arm covers the camera. The service re-samples the depth at the pixel the
     target was detected at (Target.pixel_x/y, depth_mm). The verdict is
     reported on /task/pick_event only; it does not gate the layer or trigger
     a rescan yet (UI keeps that behind a flag)
  5. /task/layer_report + TaskStatus IDLE

No rclpy here: everything goes through the WorkflowContext. The choreography
(cobot_operation) knows nothing about verification.
"""

from __future__ import annotations

from cobot_interfaces.action import Hide, PickCan, Scan
from cobot_interfaces.msg import LayerReport, PickEvent, TaskStatus
from cobot_interfaces.srv import VerifyTarget

from cobot_tasks.system_state import CAP_OPERATE
from cobot_tasks.workflows.base import ClientSpec, ParamSpec, Workflow

# One remembered-position move before the scan — much shorter than a full
# pick+place, but generous enough to cover a slow/queued controller.
HIDE_TIMEOUT_SEC = 60.0
PICK_TIMEOUT_SEC = 300.0        # one full pick+place choreography
# Post-place depth verification ("second depth scan"): cheap, no ML. Never
# move this before the PickCan result — the arm occludes the camera mid-pick.
VERIFY_TIMEOUT_SEC = 5.0


class PickLayerWorkflow(Workflow):
    id = "pick_layer"
    label = "Pick layer"
    description = (
        "Scan the bin, then pick and place every detected can (one PickCan "
        "goal each) with a post-pick depth check."
    )
    requires = CAP_OPERATE  # moves the arm AND uses vision
    params = (
        ParamSpec("scan_timeout_sec", 30.0, "budget for the /vision/scan action (detect + localize)"),
        ParamSpec("scan_model_id", "", "detector model id for the scan; '' = localization default"),
    )
    clients = (
        ClientSpec("hide", "action", Hide, "/cobot/hide"),
        ClientSpec("scan", "action", Scan, "/vision/scan"),
        ClientSpec("pick", "action", PickCan, "/cobot/pick_can"),
        ClientSpec("verify", "service", VerifyTarget, "/vision/verify_target", required=False),
    )

    def run(self, ctx) -> None:
        ctx.publish_status(TaskStatus.STATE_SCANNING, "moving to hide position (clearing camera view)")

        hide = ctx.run_action("hide", Hide.Goal(), HIDE_TIMEOUT_SEC)

        if ctx.stop_requested():
            ctx.publish_status(TaskStatus.STATE_IDLE, "stopped while moving to hide position")
            return
        if not hide.success:
            ctx.publish_status(TaskStatus.STATE_ERROR, f"move to hide position failed: {hide.message}")
            return

        ctx.publish_status(TaskStatus.STATE_SCANNING, "requesting scan")

        timeout = float(ctx.get_param("scan_timeout_sec"))
        goal = Scan.Goal(model_id=str(ctx.get_param("scan_model_id") or ""))
        scan = ctx.run_action("scan", goal, timeout)

        if ctx.stop_requested():
            ctx.publish_status(TaskStatus.STATE_IDLE, "stopped while waiting for scan")
            return
        if not scan.success:
            ctx.publish_status(TaskStatus.STATE_ERROR, f"scan failed: {scan.message}")
            return

        targets = list(scan.targets.targets)
        if not targets:
            ctx.publish_status(TaskStatus.STATE_IDLE, "scan found no cans")
            return

        report = LayerReport()
        report.overall_ok = True

        for index, can in enumerate(targets):
            if ctx.stop_requested():
                ctx.log.warn(f"stop — abandoning layer at can {index + 1}")
                report.overall_ok = False
                break

            ctx.publish_status(
                TaskStatus.STATE_PICKING,
                f"picking can at ({can.real_x:.1f}, {can.real_y:.1f}, rz={can.real_rz:.1f})",
                index + 1,
                len(targets),
            )

            result = ctx.run_action("pick", PickCan.Goal(target=can), PICK_TIMEOUT_SEC)

            if result.success:
                report.successes += 1
            else:
                report.failures += 1
                report.overall_ok = False
                ctx.log.warn(f"pick {index + 1} failed: {result.message}")

            self._pick_event(ctx, index + 1, len(targets), can, result)

        ctx.publish_layer_report(report)
        ctx.publish_status(
            TaskStatus.STATE_IDLE,
            f"layer done: {report.successes} ok / {report.failures} failed",
        )

    def _pick_event(self, ctx, index: int, total: int, can, result) -> None:
        """Report one can's outcome, then run the post-place depth check.
        success && !present confirms the pick; !success is inconclusive (no
        measurement — never counted as verified). Verification must never
        break the loop: any failure is folded into the event message."""
        event = PickEvent()
        event.index = int(index)
        event.total = int(total)
        event.target = can
        event.detection_index = int(can.detection_index)
        event.measured_z = -1.0
        event.verify_delta_mm = 0.0
        event.verify_message = ""
        event.message = result.message
        if result.success:
            event.outcome = PickEvent.OUTCOME_PICKED
        elif ctx.stop_requested():
            event.outcome = PickEvent.OUTCOME_SKIPPED
        else:
            event.outcome = PickEvent.OUTCOME_FAILED

        if result.success:
            if not ctx.client_ready("verify"):
                event.verify_message = "verify_target unavailable"
                event.message += " (verify_target unavailable)"
            else:
                try:
                    verify = ctx.call("verify", VerifyTarget.Request(target=can), VERIFY_TIMEOUT_SEC)
                    if not verify.success:
                        event.verify_message = str(verify.message)
                        event.message += f" (verify inconclusive: {verify.message})"
                        ctx.log.warn(f"pick {index}: verify_target could not measure: {verify.message}")
                    else:
                        event.verify_ran = True
                        event.verified = not verify.present
                        event.measured_z = float(verify.measured_z)
                        event.verify_delta_mm = float(verify.delta_mm)
                        if verify.present:
                            event.message += (
                                f" (WARNING: depth still shows a can at the target, "
                                f"delta {verify.delta_mm:+.0f} mm)"
                            )
                            ctx.log.warn(
                                f"pick {index}: verify_target says the can is still present "
                                f"(delta {verify.delta_mm:+.1f} mm, measured z {verify.measured_z:.1f} mm)"
                            )
                except TimeoutError as e:
                    event.verify_message = f"timed out: {e}"
                    event.message += f" (verify timed out: {e})"
                except Exception as e:  # noqa: BLE001
                    event.verify_message = f"failed: {e}"
                    event.message += f" (verify failed: {e})"

        ctx.publish_pick_event(event)
