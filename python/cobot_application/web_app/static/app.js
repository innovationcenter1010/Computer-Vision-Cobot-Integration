async function loadConfig() {
    const response = await fetch("/api/config");
    const config = await response.json();

    document.getElementById("robot_ip").value = config.robot_ip;
    document.getElementById("camera_ip").value = config.camera_ip;
    document.getElementById("save_folder").value = config.save_folder;
    document.getElementById("confidence_threshold").value = config.confidence_threshold;
    document.getElementById("enable_motion").checked = config.enable_motion;
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

let socket = null;

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
            document.getElementById("robot_command_status").innerText =
                data.result.message;
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
loadLatestImage();
updateSpeedDisplay();
setInterval(loadLatestImage, 5000);
