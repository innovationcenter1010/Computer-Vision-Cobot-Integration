// command types
// must match python decoder

export const VisionCommands = Object.freeze({
    DETECT: "detect",
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

