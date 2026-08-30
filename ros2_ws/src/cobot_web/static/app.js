// Query param busts the module cache when commands change (bump with app.js version).
import { VisionCommands, CobotCommands, CameraCommands, Receiver, ReceiverCommands } from "./commands.js?v=4";

// Pick monitor gate. When false, a "picked" event colors its box green
// regardless of the post-pick depth verification; when true, a pick is only
// shown green once /vision/verify_target confirmed the can is gone (amber
// otherwise). Flip once the depth check is trusted on the real cell.
const USE_VERIFY_GATE = false;

const STREAM_STALL_MS = 3000;
const HEALTH_STALL_MS = 3500;

let socket = null;

// ------------------- small DOM helpers -------------------

function byId(id) {
    return document.getElementById(id);
}

function setText(id, text) {
    const element = byId(id);
    if (element) element.innerText = text;
}

function setPill(id, level, text) {
    const element = byId(id);
    if (!element) return;
    element.className = "pill " + (level ? "lvl-" + level : "pill-neutral");
    element.innerHTML = '<span class="dot"></span>' + escapeHtml(text);
}

function escapeHtml(value) {
    return String(value)
        .replace(/&/g, "&amp;")
        .replace(/</g, "&lt;")
        .replace(/>/g, "&gt;")
        .replace(/"/g, "&quot;");
}

function cssVar(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

// ------------------- live camera / depth feed -------------------

let lastFrameTime = null;
let lastColorTime = null;
let lastDepthTime = null;
let frameCount = 0;
let frameWindowStart = null;
let measuredHz = 0;

function updateFeedImage(id, base64Jpeg) {
    if (!base64Jpeg) return; // keep the last good frame on screen
    const img = byId(id);
    if (img) img.src = "data:image/jpeg;base64," + base64Jpeg;
}

function handleCameraFrame(data) {
    updateFeedImage("rgb_image", data.rgb);
    updateFeedImage("depth_image", data.depth);

    const now = Date.now();
    if (data.rgb) lastColorTime = now;
    if (data.depth) lastDepthTime = now;

    if (data.rgb || data.depth) {
        lastFrameTime = now;
        if (frameWindowStart === null) frameWindowStart = now;
        frameCount += 1;
        if (now - frameWindowStart >= 1000) {
            measuredHz = frameCount / ((now - frameWindowStart) / 1000);
            frameCount = 0;
            frameWindowStart = now;
        }
        refreshStreamStatus();
    }
    drawOverlay();
}

function refreshStreamStatus() {
    const now = Date.now();
    if (lastFrameTime === null) return;

    const colorAge = lastColorTime === null ? null : (now - lastColorTime) / 1000;
    const depthAge = lastDepthTime === null ? null : (now - lastDepthTime) / 1000;
    const colorImg = byId("rgb_image");
    const depthImg = byId("depth_image");

    setText("color_meta", describeFeed(colorImg, colorAge));
    setText("depth_meta", describeFeed(depthImg, depthAge));

    if (now - lastFrameTime > STREAM_STALL_MS) {
        setPill("stream_status", "error", "stream stalled");
        return;
    }
    const missing = [];
    if (colorAge === null || colorAge > STREAM_STALL_MS / 1000) missing.push("no color");
    if (depthAge === null || depthAge > STREAM_STALL_MS / 1000) missing.push("no depth");
    if (missing.length) {
        setPill("stream_status", "warn", "live (" + missing.join(", ") + ")");
    } else {
        setPill("stream_status", "ok", "live · " + measuredHz.toFixed(1) + " Hz");
    }
}

function describeFeed(img, age) {
    if (age === null) return "no frames";
    const size = img && img.naturalWidth ? img.naturalWidth + "×" + img.naturalHeight + " · " : "";
    return size + "age " + age.toFixed(2) + " s";
}

setInterval(refreshStreamStatus, 1000);

// ------------------- detection overlay (latest scan) -------------------

let lastDetections = null;                 // {image_width, image_height, detections[]}
const pickStateByDetection = new Map();    // detection_index -> "picked" | "unverified" | "failed" | "skipped"

function handleDetections(data) {
    lastDetections = data;
    pickStateByDetection.clear();          // a new scan invalidates old pick colors
    pickEvents = [];
    renderPickMonitor();
    drawOverlay();
}

function boxState(detectionIndex) {
    return pickStateByDetection.get(detectionIndex) || "pending";
}

function colorForState(state) {
    switch (state) {
        case "picked": return cssVar("--ok");
        case "unverified": return cssVar("--warn");
        case "failed": return cssVar("--error");
        case "skipped": return cssVar("--stale");
        default: return cssVar("--primary");
    }
}

function drawOverlay() {
    const canvas = byId("rgb_overlay");
    const img = byId("rgb_image");
    if (!canvas || !img) return;

    const ctx = canvas.getContext("2d");
    const width = img.clientWidth;
    const height = img.clientHeight;
    if (!width || !height) return;
    if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
    }
    ctx.clearRect(0, 0, width, height);

    if (!lastDetections || !lastDetections.detections.length) return;
    if (!img.naturalWidth || !img.naturalHeight) return;

    // The <img> is object-fit: contain inside a 4:3 frame — map image pixels
    // to the rendered rectangle, not the element box.
    const scale = Math.min(width / img.naturalWidth, height / img.naturalHeight);
    const drawnW = img.naturalWidth * scale;
    const drawnH = img.naturalHeight * scale;
    const offsetX = (width - drawnW) / 2;
    const offsetY = (height - drawnH) / 2;
    const sx = drawnW / (lastDetections.image_width || img.naturalWidth);
    const sy = drawnH / (lastDetections.image_height || img.naturalHeight);

    ctx.lineWidth = 2.5;
    ctx.font = "500 11px 'IBM Plex Mono', ui-monospace, monospace";

    for (const det of lastDetections.detections) {
        const color = colorForState(boxState(det.index));
        const pts = [];
        for (let i = 0; i < 8; i += 2) {
            pts.push([offsetX + det.corners_px[i] * sx, offsetY + det.corners_px[i + 1] * sy]);
        }

        ctx.strokeStyle = color;
        ctx.beginPath();
        ctx.moveTo(pts[0][0], pts[0][1]);
        for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
        ctx.closePath();
        ctx.stroke();

        const label = "#" + (det.index + 1) + "  " + det.confidence.toFixed(2);
        const lx = Math.min(...pts.map(p => p[0]));
        const ly = Math.min(...pts.map(p => p[1]));
        const tw = ctx.measureText(label).width + 10;
        ctx.fillStyle = color;
        ctx.fillRect(lx, Math.max(0, ly - 18), tw, 18);
        ctx.fillStyle = "#fff";
        ctx.fillText(label, lx + 5, Math.max(13, ly - 5));
    }
}

window.addEventListener("resize", drawOverlay);

// ------------------- pick monitor -------------------

let pickEvents = [];   // ordered {index,total,detection_index,outcome,verify_ran,verified,measured_z,message}

function pickState(event) {
    if (event.outcome === "picked") {
        if (USE_VERIFY_GATE && !event.verified) return "unverified";
        if (!USE_VERIFY_GATE && event.verify_ran && !event.verified) return "unverified";
        return "picked";
    }
    if (event.outcome === "failed") return "failed";
    if (event.outcome === "skipped") return "skipped";
    return "pending";
}

function handlePickEvent(event) {
    pickEvents = pickEvents.filter(e => e.index !== event.index);
    pickEvents.push(event);
    pickEvents.sort((a, b) => a.index - b.index);
    if (event.detection_index >= 0) {
        pickStateByDetection.set(event.detection_index, pickState(event));
    }
    renderPickMonitor();
    drawOverlay();
}

function renderPickMonitor() {
    const list = byId("pick_list");
    if (!list) return;

    setText("pick_gate", USE_VERIFY_GATE ? "gate: depth verify ON" : "gate: depth verify (flag off)");

    const total = lastDetections ? lastDetections.detections.length : 0;
    if (!total) {
        setPill("pick_summary", null, "no scan yet");
        list.innerHTML = "";
        return;
    }

    const picked = pickEvents.filter(e => e.outcome === "picked").length;
    const failed = pickEvents.filter(e => e.outcome === "failed").length;
    setPill("pick_summary", null, total + " detected · " + picked + " picked · " + failed + " failed");

    const byIndex = new Map(pickEvents.map(e => [e.index, e]));
    const layerTotal = pickEvents.length ? pickEvents[0].total : total;
    const cards = [];
    for (let i = 1; i <= layerTotal; i++) {
        const event = byIndex.get(i);
        const state = event ? pickState(event) : "pending";
        let line = "awaiting pick";
        let note = "";
        if (event) {
            if (event.outcome === "picked") {
                line = state === "picked" ? "picked · verified" : "picked · depth not confirmed";
                if (event.verify_ran) {
                    note = "z " + event.measured_z.toFixed(0) + " mm" + (event.verified ? " ✓" : " — still present");
                } else {
                    note = "no depth verify";
                }
            } else if (event.outcome === "failed") {
                line = "pick failed";
                note = event.message;
            } else {
                line = "skipped";
                note = event.message;
            }
        }
        const det = event && event.detection_index >= 0 ? " · det " + (event.detection_index + 1) : "";
        cards.push(
            '<div class="pick ' + state + '" title="' + escapeHtml(event ? event.message : "") + '">' +
            '<span class="idx">#' + i + escapeHtml(det) + '</span>' +
            '<span class="out">' + escapeHtml(line) + '</span>' +
            '<span class="z">' + escapeHtml(note) + '</span>' +
            '</div>'
        );
    }
    list.innerHTML = cards.join("");
}

// ------------------- system health -------------------

let lastHealthTime = null;
let lastHealth = null;

const LEVEL_LABEL = { ok: "OK", warn: "Warn", error: "Error", stale: "Stale", unknown: "Unknown" };

function handleHealth(data) {
    lastHealthTime = Date.now();
    lastHealth = data;
    renderHealth();
}

function renderHealth() {
    const list = byId("health_list");
    if (!list || !lastHealth) return;

    const rows = lastHealth.nodes.map(node => {
        const chips = Object.entries(node.metrics || {}).map(([k, v]) => {
            const bad = v === "false" && /connected|calibrated|model_loaded|intrinsics/.test(k);
            return '<span class="chip' + (bad ? " bad" : "") + '">' + escapeHtml(k) + "=<b>" + escapeHtml(v) + "</b></span>";
        }).join("");
        const age = node.age_s < 0 ? "never" : node.age_s.toFixed(1) + " s";
        return (
            '<div class="node is-' + node.level + '">' +
            '<span class="name">' + escapeHtml(node.name) + "</span>" +
            '<span class="pill lvl-' + node.level + '"><span class="dot"></span>' + (LEVEL_LABEL[node.level] || node.level) + "</span>" +
            '<span class="detail">' + escapeHtml(node.detail) + "</span>" +
            '<span class="age">' + age + "</span>" +
            '<div class="metrics">' + chips + "</div>" +
            "</div>"
        );
    });
    list.innerHTML = rows.join("");

    const counts = { warn: 0, error: 0 };
    for (const node of lastHealth.nodes) {
        if (node.level === "warn") counts.warn++;
        if (node.level === "error" || node.level === "stale" || node.level === "unknown") counts.error++;
    }
    if (counts.error) {
        setPill("overall_health", "error", counts.error + " node" + (counts.error > 1 ? "s" : "") + " need attention");
    } else if (counts.warn) {
        setPill("overall_health", "warn", counts.warn + " warning" + (counts.warn > 1 ? "s" : ""));
    } else {
        setPill("overall_health", "ok", "all systems ok");
    }
    refreshHealthAge();
}

function refreshHealthAge() {
    if (lastHealthTime === null) return;
    const age = (Date.now() - lastHealthTime) / 1000;
    if (age * 1000 > HEALTH_STALL_MS) {
        setPill("health_age", "error", "health monitor offline (" + age.toFixed(0) + " s)");
        setPill("overall_health", "stale", "health monitor offline");
    } else {
        setPill("health_age", null, "summary " + age.toFixed(1) + " s ago");
    }
}

setInterval(refreshHealthAge, 1000);

// ------------------- task status -------------------

function handleTaskStatus(data) {
    setText("task_state", data.state || "—");
    const progress = byId("task_progress");
    const bar = byId("task_bar");
    if (data.progress_total) {
        progress.style.display = "";
        setPill("task_progress", null, data.progress_current + " / " + data.progress_total);
        if (bar) bar.style.width = (100 * data.progress_current / data.progress_total) + "%";
    } else {
        progress.style.display = "none";
        if (bar) bar.style.width = data.state === "idle" || data.state === "disconnected" ? "0%" : "100%";
    }
    setText("cobot_feedback", "state: " + data.state + (data.detail ? " — " + data.detail : ""));
}

// ------- TEMPORARY: manual calibration-marker centre select -------
// While localization is waiting, clicking the color feed sends the clicked
// COLOR-IMAGE pixel back as the marker centre, replacing the red-dot model.
// Delete this block (and the markup/CSS it touches) when the model is trusted.

let awaitingMarkerClick = false;

function setMarkerPrompt(waiting) {
    awaitingMarkerClick = waiting;

    const image = byId("rgb_image");
    if (image) image.classList.toggle("awaiting-click", waiting);

    const banner = byId("marker_click_prompt");
    if (banner) banner.style.display = waiting ? "flex" : "none";
}

function handleColorFeedClick(event) {
    if (!awaitingMarkerClick) return;

    const image = event.currentTarget;
    if (!image.naturalWidth || !image.naturalHeight) return;

    // object-fit: contain — map the click into the rendered image rectangle.
    const bounds = image.getBoundingClientRect();
    if (!bounds.width || !bounds.height) return;
    const scale = Math.min(bounds.width / image.naturalWidth, bounds.height / image.naturalHeight);
    const drawnW = image.naturalWidth * scale;
    const drawnH = image.naturalHeight * scale;
    const offsetX = (bounds.width - drawnW) / 2;
    const offsetY = (bounds.height - drawnH) / 2;

    const u = (event.clientX - bounds.left - offsetX) / scale;
    const v = (event.clientY - bounds.top - offsetY) / scale;
    if (u < 0 || v < 0 || u > image.naturalWidth || v > image.naturalHeight) return;

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
        setPill("ws_status", null, "connected");
    };

    socket.onmessage = function (event) {
        const data = JSON.parse(event.data);

        switch (data.type) {
            case "camera_frame":
                handleCameraFrame(data);
                return;
            case "command_result":
                setText("robot_command_status", data.command + " → " + data.result.message);
                return;
            case "cobot_status":
                handleTaskStatus(data);
                return;
            case "layer_report":
                setText(
                    "cobot_feedback",
                    "layer done: " + data.successes + " ok / " + data.failures +
                    " failed" + (data.overall_ok ? "" : " (FAILED)")
                );
                return;
            case "targets":
                setText("robot_command_status", "vision: " + data.count + " targets found");
                return;
            case "detections":
                handleDetections(data);
                return;
            case "health":
                handleHealth(data);
                return;
            case "pick_event":
                handlePickEvent(data);
                return;
            case "manual_dot_request":   // TEMPORARY manual centre select
                setMarkerPrompt(data.waiting);
                return;
            default:
                console.log("WebSocket message:", data);
        }
    };

    socket.onclose = function () {
        console.log("WebSocket disconnected — retrying in 2 s");
        setPill("ws_status", "error", "disconnected");
        setTimeout(connectWebSocket, 2000);
    };

    socket.onerror = function (error) {
        console.log("WebSocket error:", error);
        setPill("ws_status", "error", "error");
    };
}

/**
 * @typedef {"CAMERA" | "COBOT" | "VISION"} ReceiverType
 */

/**
 * @param {ReceiverType} receiver
 */
function sendCommand(receiver, command, payload = null) {
    if (!isValidCommand(receiver, command)) {
        console.log("command invalid", receiver, command);
        return;
    }

    if (!socket || socket.readyState !== WebSocket.OPEN) {
        setText("robot_command_status", "WebSocket is not connected");
        return;
    }

    socket.send(JSON.stringify({ receiver: receiver, command: command, payload: payload }));
}

function isValidCommand(receiver, command) {
    if (!Object.values(Receiver).includes(receiver)) {
        throw new Error(`Invalid receiver: ${receiver}`);
    }
    const commands = ReceiverCommands[receiver];
    if (!commands) return false;
    return Object.values(commands).includes(command);
}

window.sendCommand = sendCommand;
window.Receiver = Receiver;
window.VisionCommands = VisionCommands;
window.CobotCommands = CobotCommands;
window.CameraCommands = CameraCommands;

connectWebSocket();
renderPickMonitor();

// TEMPORARY manual centre select — wire the color feed once at load.
const colorFeed = byId("rgb_image");
if (colorFeed) {
    colorFeed.addEventListener("click", handleColorFeedClick);
    colorFeed.addEventListener("load", drawOverlay);
}
