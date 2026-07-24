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
    PICKUP: "pickup",
    CALIBRATE: "calibrate",
    STOP: "stop",
    START: "start"
});

export const CameraCommands = Object.freeze({
    SNAPSHOT: "snapshot",
    SAVE: "save",
});

export const Receiver = Object.freeze({
    CAMERA: "CAMERA",
    COBOT: "COBOT",
    VISION: "VISION"
});

export const ReceiverCommands = Object.freeze({
    [Receiver.CAMERA]: CameraCommands,
    [Receiver.COBOT]: CobotCommands,
    [Receiver.VISION]: VisionCommands,
});

