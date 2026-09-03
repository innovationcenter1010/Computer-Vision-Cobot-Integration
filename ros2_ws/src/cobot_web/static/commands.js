// command types
// must match python decoder

export const VisionCommands = Object.freeze({
    DETECT: "detect",
    // TEMPORARY manual centre select — payload {u, v} in COLOR image pixels.
    CLICK_CENTER: "click_center",
    // TEMPORARY manual centre select — drop this pose (marker out of frame).
    SKIP_CENTER: "skip_center",
});

export const CobotCommands = Object.freeze({
    // Main path: payload {workflow: id} from the dashboard selector.
    START_WORKFLOW: "start_workflow",
    // Tooling card: payload {name} → /cobot/set_active_tool (cobot_control).
    SET_ACTIVE_TOOL: "set_active_tool",
    // Re-express every remembered position in the active tool file
    // (controller kinematics) — manual retry; normally automatic.
    CONVERT_TRAJECTORIES: "convert_trajectories",
    // payload {key, index} → overwrite one remembered position with the
    // current pose + joints.
    TEACH_POINT: "teach_point",
    // Trigger aliases still served by task_manager.
    PICKUP: "pickup",
    CALIBRATE: "calibrate",
    STOP: "stop",
    START: "start"
});

export const CameraCommands = Object.freeze({
    SNAPSHOT: "snapshot",
    SAVE: "save",
});

// Session recorder (server-side, web_node): start/stop capture every
// websocket payload; a stop exports a standalone replay HTML to /results.
export const RecordCommands = Object.freeze({
    START: "start",
    STOP: "stop",
    STATUS: "status",
});

export const Receiver = Object.freeze({
    CAMERA: "CAMERA",
    COBOT: "COBOT",
    VISION: "VISION",
    RECORD: "RECORD"
});

export const ReceiverCommands = Object.freeze({
    [Receiver.CAMERA]: CameraCommands,
    [Receiver.COBOT]: CobotCommands,
    [Receiver.VISION]: VisionCommands,
    [Receiver.RECORD]: RecordCommands,
});

