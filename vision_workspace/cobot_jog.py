#!/usr/bin/env python3
"""cobot_jog.py — careful, one-move-at-a-time robot test tool.

Talks to the real robot driver (cobot_control) exactly the way the pick code
does (/cobot/move_to_pose), but only when YOU type a command and confirm it.
Nothing runs on its own: no workflows, no vision, no gripper.

Run inside the running container (system started with cobot:=true tasks:=false):

  docker exec -it <container-name> bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && python3 /ws/vision_workspace/cobot_jog.py"

Commands (mm and degrees, robot base frame — same numbers the pendant shows
for the active tool):
  p                      show where the robot is + its status
  rel DX DY DZ           small relative move, e.g.  rel 0 0 20   (up 20 mm)
  rel DX DY DZ DRX DRY DRZ   relative move with rotation
  go X Y Z RX RY RZ      move to an absolute pose (as shown by 'p')
  save NAME              remember the current pose (e.g. save above_bin)
  back NAME              return to a remembered pose ('start' = where we began)
  list                   show remembered poses
  speed N                linear speed in mm/s (default 50, max 250)
  mode linear|joint      straight-line moves (default) or joint moves
  stop                   stop the arm now
  q                      quit
During a move, press Ctrl+C to stop the arm immediately.

Safety limits in this tool (on top of the driver's own reachability and
joint-limit checks): one relative step <= 150 mm and <= 30 deg; an absolute
'go' must be within 400 mm of the current position.

Wrist note: cobot_control adds mount_yaw_offset_deg (90 in cobot_setup.yaml)
to the rz of EVERY move. This tool subtracts it first, so the pose you type
is the pose the robot ends up at.
"""
import math
import threading
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from cobot_interfaces.action import MoveToPose
from cobot_interfaces.msg import CobotState
from cobot_interfaces.srv import MotionStop

SETUP_FILE = "/ws/install/share/cobot_bringup/config/cobot_setup.yaml"
MAX_STEP_MM = 150.0
MAX_STEP_DEG = 30.0
MAX_GO_DIST_MM = 400.0
MAX_SPEED_MMS = 250.0
MODES = {"TEACH": 0, "PLAY": 1, "REMOTE": 2}
MODE_NAMES = {v: k for k, v in MODES.items()}


def wrap180(a):
    """Angle into [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def fmt(p):
    return ("x={:8.1f} y={:8.1f} z={:8.1f}  rx={:7.1f} ry={:7.1f} rz={:7.1f}"
            .format(*p))


def read_mount_offset():
    try:
        with open(SETUP_FILE) as f:
            return float((yaml.safe_load(f) or {}).get("mount_yaw_offset_deg", 0.0))
    except Exception as e:  # noqa: BLE001
        print(f"!! could not read {SETUP_FILE} ({e}); assuming offset 90")
        return 90.0


class Jog(Node):
    def __init__(self):
        super().__init__("cobot_jog")
        self.state = None
        self.create_subscription(CobotState, "/cobot/state", self._on_state, 1)
        self.move = ActionClient(self, MoveToPose, "/cobot/move_to_pose")
        self.stop_cli = self.create_client(MotionStop, "/cobot/motion_stop")

    def _on_state(self, msg):
        self.state = msg

    def pose(self):
        s = self.state
        return None if s is None else [s.x, s.y, s.z, s.rx, s.ry, s.rz]


def wait(fut, timeout=None):
    t0 = time.time()
    while not fut.done():
        if timeout is not None and time.time() - t0 > timeout:
            return False
        time.sleep(0.05)
    return True


def show_status(node):
    s = node.state
    if s is None:
        print("no /cobot/state yet — is cobot_control running (cobot:=true)?")
        return
    print(f"pose    : {fmt(node.pose())}   (tool {s.pose_tool_number} '{s.active_tool_name}')")
    if len(s.joints_deg):
        print("joints  : " + "  ".join(f"{n}={v:.1f}" for n, v in zip("SLURBT", s.joints_deg)))
    print(f"status  : connected={s.connected} servo_on={s.servo_on} "
          f"mode={MODE_NAMES.get(s.control_mode, s.control_mode)} hold={s.in_hold} "
          f"alarm={s.alarming} busy={s.busy}")
    for a in s.alarms:
        print(f"  ALARM : {a}")


def ready_problems(node):
    s = node.state
    if s is None:
        return "no robot state received yet"
    if not s.connected:
        return "robot not connected (check cable / IP 192.168.1.31)"
    if s.alarming:
        return "controller has an alarm — clear it on the pendant first"
    if not s.servo_on:
        return "servos are off"
    if s.control_mode == MODES["TEACH"]:
        return "pendant is in TEACH mode — switch the key to REMOTE"
    if s.in_hold:
        return "controller is in HOLD"
    if s.busy:
        return "a move is already running"
    return None


def stop_arm(node):
    if node.stop_cli.wait_for_service(timeout_sec=1.0):
        req = MotionStop.Request()
        req.clear_trajectory = True
        wait(node.stop_cli.call_async(req), 3.0)
        print("stop sent")
    else:
        print("!! /cobot/motion_stop not available — use the pendant / e-stop")


def do_move(node, target, offset, speed, linear):
    problem = ready_problems(node)
    if problem:
        print(f"not moving: {problem}")
        return
    cur = node.pose()
    dist = math.dist(cur[:3], target[:3])
    rot = max(abs(wrap180(t - c)) for t, c in zip(target[3:], cur[3:]))
    print(f"  from: {fmt(cur)}")
    print(f"  to  : {fmt(target)}")
    print(f"  ({dist:.0f} mm, max rotation {rot:.0f} deg, "
          f"{'linear %.0f mm/s' % speed if linear else 'joint 10 %'})")
    if input("  type y + Enter to move (anything else cancels): ").strip().lower() != "y":
        print("  cancelled")
        return
    if not node.move.wait_for_server(timeout_sec=2.0):
        print("!! /cobot/move_to_pose not available — is cobot_control running?")
        return

    g = MoveToPose.Goal()
    g.x, g.y, g.z, g.rx, g.ry = target[:5]
    g.rz = wrap180(target[5] - offset)      # driver adds the offset back
    g.interpolation = MoveToPose.Goal.INTERP_LINEAR if linear else MoveToPose.Goal.INTERP_JOINT
    g.speed = speed if linear else 10.0
    g.accuracy_mm = 0
    g.tool_number = -1

    gh_fut = node.move.send_goal_async(g)
    wait(gh_fut, 5.0)
    gh = gh_fut.result() if gh_fut.done() else None
    if gh is None or not gh.accepted:
        print("!! move REJECTED by cobot_control (see its log: unreachable, servos, mode...)")
        return
    res_fut = gh.get_result_async()
    try:
        while not res_fut.done():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nCtrl+C — stopping the arm")
        gh.cancel_goal_async()
        stop_arm(node)
        wait(res_fut, 5.0)
    if res_fut.done():
        r = res_fut.result().result
        print(f"  result: success={r.success} code={r.status_code} {r.message}")
    time.sleep(0.3)
    end = node.pose()
    if end:
        err = math.dist(end[:3], target[:3])
        print(f"  now : {fmt(end)}   (position error {err:.1f} mm)")


def main():
    # Keep Ctrl+C for OUR handler (stop the arm), not rclpy's shutdown.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Jog()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    offset = read_mount_offset()
    speed, linear = 50.0, True
    print(f"cobot_jog — mount_yaw_offset_deg = {offset} (handled for you)")
    print("waiting for /cobot/state ...")
    t0 = time.time()
    while node.state is None and time.time() - t0 < 10:
        time.sleep(0.1)
    show_status(node)
    saved = {}
    if node.pose():
        saved["start"] = node.pose()
        print("remembered the current pose as 'start'")
    print("type a command (p, rel, go, save, back, list, speed, mode, stop, q)")

    while True:
        try:
            line = input("jog> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        cmd, *args = line.split()
        cmd = cmd.lower()
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd in ("p", "pose", "status"):
                show_status(node)
            elif cmd == "stop":
                stop_arm(node)
            elif cmd == "speed":
                speed = max(5.0, min(MAX_SPEED_MMS, float(args[0])))
                print(f"linear speed = {speed:.0f} mm/s")
            elif cmd == "mode":
                linear = args[0].lower().startswith("l")
                print("mode = " + ("linear (straight line)" if linear else "joint (10 %)"))
            elif cmd == "save":
                saved[args[0]] = node.pose()
                print(f"saved '{args[0]}': {fmt(saved[args[0]])}")
            elif cmd == "list":
                for k, v in saved.items():
                    print(f"  {k:12s} {fmt(v)}")
            elif cmd == "back":
                if args[0] not in saved:
                    print(f"no pose named '{args[0]}' (try: list)")
                    continue
                tgt = saved[args[0]]
                if math.dist(tgt[:3], node.pose()[:3]) > MAX_GO_DIST_MM:
                    print(f"refused: more than {MAX_GO_DIST_MM:.0f} mm away — go in steps")
                    continue
                do_move(node, tgt, offset, speed, linear)
            elif cmd == "rel":
                d = [float(a) for a in args] + [0.0] * (6 - len(args))
                if len(args) not in (3, 6):
                    print("usage: rel DX DY DZ   or   rel DX DY DZ DRX DRY DRZ")
                    continue
                if math.dist(d[:3], (0, 0, 0)) > MAX_STEP_MM or max(map(abs, d[3:])) > MAX_STEP_DEG:
                    print(f"refused: step limit is {MAX_STEP_MM:.0f} mm / {MAX_STEP_DEG:.0f} deg")
                    continue
                cur = node.pose()
                tgt = [c + x for c, x in zip(cur, d)]
                tgt[3:] = [wrap180(a) for a in tgt[3:]]
                do_move(node, tgt, offset, speed, linear)
            elif cmd == "go":
                if len(args) != 6:
                    print("usage: go X Y Z RX RY RZ   (copy the numbers from 'p')")
                    continue
                tgt = [float(a) for a in args]
                if math.dist(tgt[:3], node.pose()[:3]) > MAX_GO_DIST_MM:
                    print(f"refused: more than {MAX_GO_DIST_MM:.0f} mm from here — go in steps")
                    continue
                do_move(node, tgt, offset, speed, linear)
            else:
                print("unknown command — p, rel, go, save, back, list, speed, mode, stop, q")
        except TypeError:
            print("no robot pose yet — check 'p' (is the robot connected?)")
        except (IndexError, ValueError):
            print("couldn't read that — check the numbers (see the list at the top of the file)")

    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
