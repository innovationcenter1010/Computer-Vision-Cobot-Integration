// Query param busts the module cache when commands change (bump with app.js version).
import { VisionCommands, CobotCommands, CameraCommands, Receiver, ReceiverCommands } from "./commands.js?v=4";


function setFieldValue(id, value) {
    const element = document.getElementById(id);
    if (element && value !== undefined && value !== null) {
        element.value = value;
    }
}

async function loadConfig() {
    const response = await fetch("/api/config");
    const config = await response.json();

    // Config is the bringup params YAML: {node_name: {ros__parameters: {...}}}
    const cobotParams = config.cobot_node?.ros__parameters ?? {};
    const yoloParams = config.yolo_node?.ros__parameters ?? {};

    setFieldValue("robot_ip", cobotParams.robot_ip);
    setFieldValue("confidence_threshold", yoloParams.confidence);
}

async function saveConfig() {
    const config = {
        robot_ip: document.getElementById("robot_ip").value,
        camera_ip: document.getElementById("camera_ip").value,
        save_folder: document.getElementById("save_folder").value,
        confidence_threshold: parseFloat(document.getElementById("confidence_threshold").value),
        enable_motion: document.getElementById("enable_motion").checked
    };

    console.log("Saving config:", config);

    const response = await fetch("/api/config", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify(config)
    });

    const result = await response.json();

    console.log("Server response:", result);

    document.getElementById("save_status").innerText = "Saved!";
}

/*
async function loadLatestImage() {
    const response = await fetch("/api/latest-image?t=" + new Date().getTime());
    const result = await response.json();

    console.log("Latest image result:", result);

    const img = document.getElementById("latest_image");

    if (result.image === null) {
        img.src = "";
        img.alt = "No image found";
    } else {
        img.src = result.image + "?t=" + new Date().getTime();
        img.alt = "Latest image";
    }
}

*/

let socket = null;

// ------------------- live camera / depth feed -------------------

let lastFrameTime = null;

function setText(id, text) {
    const element = document.getElementById(id);
    if (element) element.innerText = text;
}

function updateFeedImage(id, base64Jpeg) {
    if (!base64Jpeg) return; // keep the last good frame on screen
    const img = document.getElementById(id);
    if (img) img.src = "data:image/jpeg;base64," + base64Jpeg;
}

function handleCameraFrame(data) {
    updateFeedImage("rgb_image", data.rgb);
    updateFeedImage("depth_image", data.depth);

    if (data.rgb || data.depth) {
        lastFrameTime = Date.now();

        let status = "live";
        if (!data.rgb) status += " (no color)";
        if (!data.depth) status += " (no depth)";
        setText("stream_status", status);
    }
}

// Mark the feed as stalled if frames stop arriving.
setInterval(function () {
    if (lastFrameTime !== null && Date.now() - lastFrameTime > 3000) {
        setText("stream_status", "stream stalled");
    }
}, 1000);

// ------- TEMPORARY: manual calibration-marker centre select -------
// While localization is waiting, clicking the color feed sends the clicked
// COLOR-IMAGE pixel back as the marker centre, replacing the red-dot model.
// Delete this block (and the markup/CSS it touches) when the model is trusted.

let awaitingMarkerClick = false;

function setMarkerPrompt(waiting) {
    awaitingMarkerClick = waiting;

    const image = document.getElementById("rgb_image");
    if (image) image.classList.toggle("awaiting-click", waiting);

    const banner = document.getElementById("marker_click_prompt");
    if (banner) banner.style.display = waiting ? "flex" : "none";
}

function handleColorFeedClick(event) {
    if (!awaitingMarkerClick) return;

    const image = event.currentTarget;

    // naturalWidth/Height are the true frame size (640x480); the element is
    // CSS-scaled, so map the click back before sending pixels.
    if (!image.naturalWidth || !image.naturalHeight) return;

    const bounds = image.getBoundingClientRect();
    if (!bounds.width || !bounds.height) return;

    const u = (event.clientX - bounds.left) / bounds.width * image.naturalWidth;
    const v = (event.clientY - bounds.top) / bounds.height * image.naturalHeight;

    sendCommand(Receiver.VISION, VisionCommands.CLICK_CENTER, {
        u: Math.round(u),
        v: Math.round(v)
    });

    // Optimistic: localization confirms by dropping the request flag.
    setMarkerPrompt(false);
}

function skipMarkerPose() {
    if (!awaitingMarkerClick) return;
    sendCommand(Receiver.VISION, VisionCommands.SKIP_CENTER);
    setMarkerPrompt(false);
}

window.skipMarkerPose = skipMarkerPose;

// -----------------------------------------------------------------

function connectWebSocket() {
    const wsProtocol = window.location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(`${wsProtocol}://${window.location.host}/ws`);

    socket.onopen = function () {
        console.log("WebSocket connected");
        document.getElementById("robot_command_status").innerText = "WebSocket connected";
    };

    socket.onmessage = function (event) {
        const data = JSON.parse(event.data);
        console.log("WebSocket message:", data);

        if (data.type === "command_result") {
            setText("robot_command_status", data.result.message);
            return;
        }

        if (data.type === "camera_frame") {
            handleCameraFrame(data);
            return;
        }

        if (data.type === "cobot_status") {
            setText(
                "cobot_feedback",
                "state: " + data.state + (data.detail ? " — " + data.detail : "")
            );
            return;
        }

        if (data.type === "layer_report") {
            setText(
                "cobot_feedback",
                "layer done: " + data.successes + " ok / " + data.failures +
                " failed" + (data.overall_ok ? "" : " (FAILED)")
            );
            return;
        }

        if (data.type === "targets") {
            setText("robot_command_status", "vision: " + data.count + " targets found");
            return;
        }

        // TEMPORARY manual centre select
        if (data.type === "manual_dot_request") {
            setMarkerPrompt(data.waiting);
            return;
        }
    };

    socket.onclose = function () {
        console.log("WebSocket disconnected");
        document.getElementById("robot_command_status").innerText = "WebSocket disconnected";
    };

    socket.onerror = function (error) {
        console.log("WebSocket error:", error);
        document.getElementById("robot_command_status").innerText = "WebSocket error";
    };
}

/**
 * @typedef {"CAMERA" | "COBOT" | "VISION"} ReceiverType
 */

/**
 * @param {ReceiverType} receiver
 */
function sendCommand(receiver,command,payload = null) {
    console.log("Sending command:", command);


    if(!isValidCommand(receiver,command)) {

        console.log("command invalid");
        return;
    }


    if (!socket || socket.readyState !== WebSocket.OPEN) {
        console.log("WebSocket is not connected");

        document.getElementById("robot_command_status").innerText =
            "WebSocket is not connected";

        return;
    }

    const outgoing = {
        receiver: receiver,
        command: command,
        payload: payload
    };

    socket.send(JSON.stringify(outgoing));
}

function isValidCommand(receiver, command) {

    if (!Object.values(Receiver).includes(receiver)) {
        throw new Error(`Invalid receiver: ${receiver}`);
    }
    
    const commands = ReceiverCommands[receiver];

    if (!commands) {
        return false;
    }

    return Object.values(commands).includes(command);
}

/*
function sendRobotCommand(command) {
    console.log("Sending command:", command);

    if (!socket || socket.readyState !== WebSocket.OPEN) {
        console.log("WebSocket is not connected");

        document.getElementById("robot_command_status").innerText =
            "WebSocket is not connected";

        return;
    }

    const payload = {
        command: command
    };

    socket.send(JSON.stringify(payload));
}

function sendRobotCommandWithValue(command, commandValue) {
    console.log("Sending command:", command);
    console.log("Command value:", commandValue);

    if (!socket || socket.readyState !== WebSocket.OPEN) {
        console.log("WebSocket is not connected");

        document.getElementById("robot_command_status").innerText =
            "WebSocket is not connected";

        return;
    }

    const payload = {
        command: command,
        value: commandValue
    };

    socket.send(JSON.stringify(payload));
}

*/

window.sendCommand = sendCommand;
window.Receiver = Receiver;
window.VisionCommands = VisionCommands;
window.CobotCommands = CobotCommands;
window.CameraCommands = CameraCommands;

let speedCommandTimer = null;
let lastSentSpeedLevel = null;

const SPEED_COMMAND_DELAY_MS = 1500;

function getSpeedLabel(speedLevel) {
    const speedLabels = {
        1: "Slow",
        2: "Medium",
        3: "Fast",
        4: "Max"
    };

    return speedLabels[speedLevel] || "Unknown";
}

function updateSpeedDisplay() {
    const slider = document.getElementById("cobot_speed_slider");
    const speedLevel = parseInt(slider.value);

    document.getElementById("cobot_speed_status").innerText =
        "Selected speed: " + getSpeedLabel(speedLevel) + " - waiting...";
}

function handleSpeedSliderInput() {
    updateSpeedDisplay();

    if (speedCommandTimer !== null) {
        clearTimeout(speedCommandTimer);
    }

    speedCommandTimer = setTimeout(function () {
        sendSpeedCommand();
    }, SPEED_COMMAND_DELAY_MS);
}

function sendSpeedCommand() {
    const slider = document.getElementById("cobot_speed_slider");
    const speedLevel = parseInt(slider.value);

    if (speedLevel === lastSentSpeedLevel) {
        document.getElementById("cobot_speed_status").innerText =
            "Selected speed: " + getSpeedLabel(speedLevel) + " - already sent";
        return;
    }

    lastSentSpeedLevel = speedLevel;

    document.getElementById("cobot_speed_status").innerText =
        "Selected speed: " + getSpeedLabel(speedLevel) + " - sent";

    sendRobotCommandWithValue("set_cobot_speed", speedLevel);
}

connectWebSocket();
loadConfig();

// TEMPORARY manual centre select — wire the color feed once at load.
const colorFeed = document.getElementById("rgb_image");
if (colorFeed) colorFeed.addEventListener("click", handleColorFeedClick);



