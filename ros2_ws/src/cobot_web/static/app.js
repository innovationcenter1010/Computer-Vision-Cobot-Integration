// Query param busts the module cache when commands change (bump with app.js version).
import { VisionCommands, CobotCommands, CameraCommands, RecordCommands, Receiver, ReceiverCommands } from "./commands.js?v=7";

// Cobot panel: a /cobot/state sample older than this is shown as stale.
const COBOT_STALE_MS = 2000;
const JOINT_NAMES = ["S", "L", "U", "R", "B", "T"];

// Pick monitor gate. When false, a "picked" event colors its box green
// regardless of the post-pick depth verification; when true, a pick is only
// shown green once /vision/verify_target confirmed the can is gone (amber
// otherwise). Flip once the depth check is trusted on the real cell.
const USE_VERIFY_GATE = false;

const STREAM_STALL_MS = 3000;
const HEALTH_STALL_MS = 3500;

let socket = null;

// Virtual clock: live pages get Date.now(); a standalone replay export
// (replay.js) installs window.__cobotNow so frame ages, stall pills and
// marker expiry follow the recording's timeline (and freeze on pause).
const nowMs = () => (window.__cobotNow ? window.__cobotNow() : Date.now());

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
let frameWindowStart = null;
const frameCounts = { all: 0, color: 0, depth: 0 };
const wsHz = { all: 0, color: 0, depth: 0 };     // websocket frames/s, per stream

function updateFeedImage(id, base64Jpeg) {
    if (!base64Jpeg) return; // keep the last good frame on screen
    const img = byId(id);
    if (img) img.src = "data:image/jpeg;base64," + base64Jpeg;
}

function handleCameraFrame(data) {
    updateFeedImage("rgb_image", data.rgb);
    updateFeedImage("depth_image", data.depth);

    const now = nowMs();
    if (data.rgb) { lastColorTime = now; frameCounts.color += 1; }
    if (data.depth) { lastDepthTime = now; frameCounts.depth += 1; }

    if (data.rgb || data.depth) {
        lastFrameTime = now;
        if (frameWindowStart === null) frameWindowStart = now;
        frameCounts.all += 1;
        if (now - frameWindowStart >= 1000) {
            const span = (now - frameWindowStart) / 1000;
            for (const k of Object.keys(frameCounts)) {
                wsHz[k] = frameCounts[k] / span;
                frameCounts[k] = 0;
            }
            frameWindowStart = now;
        }
        refreshStreamStatus();
    }
    drawOverlay();
}

// Camera-side numbers (color_hz / depth_hz / *_dropped / publish_rate_hz)
// come from camera_node's heartbeat via the health summary (1 Hz). The
// browser itself only knows the websocket rate and the age of the last
// frame it received — both are shown, labelled apart.
const SPARK_SAMPLES = 60;
const cameraSpark = { color: [], depth: [] };   // [{hz, drop}] newest last
const lastDropped = { color: null, depth: null };
let cameraMetrics = null;

function noteCameraHealth(summary) {
    const node = (summary.nodes || []).find(n => n.name === "camera_node");
    cameraMetrics = node ? (node.metrics || {}) : null;
    for (const stream of ["color", "depth"]) {
        const hz = cameraMetrics ? parseFloat(cameraMetrics[stream + "_hz"]) : NaN;
        const dropped = cameraMetrics ? parseInt(cameraMetrics[stream + "_dropped"], 10) : NaN;
        const drop = Number.isFinite(dropped) && lastDropped[stream] !== null && dropped > lastDropped[stream];
        if (Number.isFinite(dropped)) lastDropped[stream] = dropped;
        const series = cameraSpark[stream];
        series.push({ hz: Number.isFinite(hz) ? hz : null, drop });
        if (series.length > SPARK_SAMPLES) series.shift();
    }
    refreshStreamStatus();
}

function refreshStreamStatus() {
    const now = nowMs();
    const colorAge = lastColorTime === null ? null : (now - lastColorTime) / 1000;
    const depthAge = lastDepthTime === null ? null : (now - lastDepthTime) / 1000;
    const colorImg = byId("rgb_image");
    const depthImg = byId("depth_image");

    setText("color_meta", describeFeed(colorImg));
    setText("depth_meta", describeFeed(depthImg));
    renderFeedHud("color", colorImg, colorAge);
    renderFeedHud("depth", depthImg, depthAge);
    renderFeedTelemetry();

    if (lastFrameTime === null) return;
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
        setPill("stream_status", "ok", "live · " + wsHz.all.toFixed(1) + " Hz");
    }
}

function describeFeed(img) {
    return img && img.naturalWidth ? img.naturalWidth + "×" + img.naturalHeight : "";
}

function metric(name) {
    return cameraMetrics && cameraMetrics[name] !== undefined ? cameraMetrics[name] : null;
}

function renderFeedTelemetry() {
    const box = byId("feed_telemetry");
    if (!box) return;
    const chip = (label, value) => '<span class="chip">' + label + " <b>" + escapeHtml(value) + "</b></span>";
    const camHz = (s) => metric(s + "_hz") !== null ? Number(metric(s + "_hz")).toFixed(1) + " Hz" : "—";
    box.innerHTML = chip("color", camHz("color")) + chip("depth", camHz("depth")) + chip("ws", wsHz.all.toFixed(1) + " Hz");
}

function renderFeedHud(stream, img, age) {
    const live = age !== null && age * 1000 <= STREAM_STALL_MS;
    const frame = byId(stream + "_frame");
    if (frame) {
        frame.classList.toggle("live", live);
        frame.classList.toggle("stalled", age !== null && !live);
    }
    const liveEl = byId(stream + "_hud_live");
    if (liveEl) {
        liveEl.textContent = age === null ? "no frames" : live ? "live" : "stalled";
        liveEl.className = "hud-live " + (age === null ? "" : live ? "on" : "off");
    }
    const hz = metric(stream + "_hz");
    setText(stream + "_hud_hz", hz === null ? "—" : Number(hz).toFixed(1));
    setText(stream + "_hud_age", age === null ? "—" : age < 10 ? age.toFixed(2) + " s" : age.toFixed(0) + " s");
    const dropped = metric(stream + "_dropped");
    setText(stream + "_hud_drops", dropped === null ? "—" : dropped);
    const size = describeFeed(img);
    setText(stream + "_foot_right", (size ? size + " · " : "") + "ws " + wsHz[stream].toFixed(1) + " Hz");
    drawSpark(stream);
}

function drawSpark(stream) {
    const canvas = byId(stream + "_spark");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width;
    const H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    const samples = cameraSpark[stream];
    const values = samples.map(s => s.hz).filter(v => v !== null);
    const note = byId(stream + "_foot");
    if (!values.length) {
        if (note) note.textContent = "camera Hz · waiting for health";
        return;
    }
    const target = parseFloat(metric("publish_rate_hz"));
    const top = (Math.max(Number.isFinite(target) ? target : 0, ...values) * 1.05) || 1;
    const offset = SPARK_SAMPLES - samples.length;              // newest sample sits at the right edge
    const X = (i) => (i / (SPARK_SAMPLES - 1)) * W;
    const Y = (v) => H - 3 - (v / top) * (H - 6);

    if (Number.isFinite(target)) {
        ctx.strokeStyle = "rgba(76, 140, 242, .3)";
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(0, Y(target)); ctx.lineTo(W, Y(target)); ctx.stroke();
    }

    const firstIdx = samples.findIndex(s => s.hz !== null);
    ctx.beginPath();
    let started = false;
    samples.forEach((s, i) => {
        if (s.hz === null) return;
        const x = X(offset + i);
        const y = Y(s.hz);
        if (started) ctx.lineTo(x, y); else ctx.moveTo(x, y);
        started = true;
    });
    ctx.strokeStyle = cssVar("--primary-hi");
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.lineTo(W, H);
    ctx.lineTo(X(offset + firstIdx), H);
    ctx.closePath();
    const grad = ctx.createLinearGradient(0, 0, 0, H);
    grad.addColorStop(0, "rgba(35, 97, 196, .55)");
    grad.addColorStop(1, "rgba(35, 97, 196, 0)");
    ctx.fillStyle = grad;
    ctx.fill();

    ctx.fillStyle = cssVar("--warn");
    samples.forEach((s, i) => {
        if (!s.drop || s.hz === null) return;
        ctx.beginPath(); ctx.arc(X(offset + i), Y(s.hz), 3, 0, Math.PI * 2); ctx.fill();
    });
    ctx.fillStyle = "#ffffff";
    ctx.beginPath(); ctx.arc(W - 1, Y(values[values.length - 1]), 2.5, 0, Math.PI * 2); ctx.fill();

    if (note) {
        note.textContent = "camera Hz · " + values.length + " s · min " + Math.min(...values).toFixed(1) +
            " · max " + Math.max(...values).toFixed(1);
    }
}

setInterval(refreshStreamStatus, 1000);

// ------------------- stream layout: split / color focus / depth focus -------------------

const FEED_LAYOUTS = ["split", "focus-color", "focus-depth"];
let feedLayout = "split";

function setFeedLayout(layout) {
    if (!FEED_LAYOUTS.includes(layout)) layout = "split";
    feedLayout = layout;
    const grid = byId("vision_grid");
    if (grid) grid.className = "vision-grid" + (layout === "split" ? "" : " " + layout);
    document.querySelectorAll("#feed_layout button").forEach(b => b.classList.toggle("on", b.dataset.layout === layout));
    try { localStorage.setItem("cobot.feedLayout", layout); } catch (e) { /* storage unavailable */ }
    requestAnimationFrame(drawOverlay);
}
window.setFeedLayout = setFeedLayout;

function toggleFeedFocus(stream) {
    // Clicking a stream focuses it; clicking the focused stream restores the
    // split. Never while localization waits for the manual marker click —
    // that click belongs to the calibration pose (handleColorFeedClick stops
    // its propagation as well).
    if (awaitingMarkerClick) return;
    const want = "focus-" + stream;
    setFeedLayout(feedLayout === want ? "split" : want);
}

function initFeedLayout() {
    let saved = "split";
    try { saved = localStorage.getItem("cobot.feedLayout") || "split"; } catch (e) { /* storage unavailable */ }
    setFeedLayout(saved);
    for (const stream of ["color", "depth"]) {
        const frame = byId(stream + "_frame");
        if (frame) frame.addEventListener("click", () => toggleFeedFocus(stream));
    }
    // The overlay canvas must follow the <img> through layout transitions,
    // not just window resizes.
    const img = byId("rgb_image");
    if (img && "ResizeObserver" in window) new ResizeObserver(() => drawOverlay()).observe(img);
}

// ------------------- theme: light / dark -------------------
// index.html stamps data-theme on <html> before first paint (stored choice,
// else OS preference); here we only label the button and handle the toggle.
// A toggle is persisted; a merely OS-derived theme is not, so the page keeps
// following the OS until the user chooses.

function themeButtonLabel() {
    const theme = document.documentElement.dataset.theme === "dark" ? "dark" : "light";
    const btn = byId("theme_toggle");
    if (btn) btn.textContent = theme === "dark" ? "light theme" : "dark theme";
}

function toggleTheme() {
    const theme = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
    document.documentElement.dataset.theme = theme;
    try { localStorage.setItem("cobot.theme", theme); } catch (e) { /* storage unavailable */ }
    themeButtonLabel();
    // canvases sample the CSS tokens at draw time — repaint under the new theme
    drawOverlay();
    refreshStreamStatus();
}
window.toggleTheme = toggleTheme;

// ------------------- detection overlay (latest scan) -------------------

let lastDetections = null;                 // {image_width, image_height, detections[]}
const pickStateByDetection = new Map();    // detection_index -> "picked" | "unverified" | "failed" | "skipped"

function handleDetections(data) {
    if (data.calibration) {
        // Marker-model detect (calibration): show it as live marker boxes for
        // a few seconds; never touch the can overlay or the pick monitor.
        liveMarkerDetections = data;
        liveMarkerTime = nowMs();
        drawOverlay();
        return;
    }
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

    if (!img.naturalWidth || !img.naturalHeight) return;
    drawCalibrationOverlay(ctx, img, width, height);
    if (!lastDetections || !lastDetections.detections.length) return;

    // The <img> is object-fit: contain inside a 4:3 frame — map image pixels
    // to the rendered rectangle, not the element box.
    const scale = Math.min(width / img.naturalWidth, height / img.naturalHeight);
    const drawnW = img.naturalWidth * scale;
    const drawnH = img.naturalHeight * scale;
    const offsetX = (width - drawnW) / 2;
    const offsetY = (height - drawnH) / 2;
    const sx = drawnW / (lastDetections.image_width || img.naturalWidth);
    const sy = drawnH / (lastDetections.image_height || img.naturalHeight);

    ctx.lineWidth = 2;
    ctx.font = "500 11px 'JetBrains Mono', ui-monospace, monospace";

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
        ctx.fillStyle = "#02060f";   // dark text on the colored tag reads better on the dark feed
        ctx.fillText(label, lx + 5, Math.max(13, ly - 5));
    }
}

window.addEventListener("resize", drawOverlay);

// ------------------- calibration verification overlay -------------------
// localization echoes every calibration_sample on /vision/calibration_marker
// (web_node → {"type": "calibration_marker"}): accepted markers accumulate
// for the run (a reset starts a new one), the last rejection is shown until
// the next sample, and calibration-model detects arrive as {"type":
// "detections", calibration: true} → dashed live boxes for a few seconds.

const LIVE_MARKER_MS = 6000;
// The marker overlay draws over live video, not the page surface, so it does
// not follow the theme tokens: --ok (#0e8a50 in light) is far too dark to see
// against a camera image. Fixed bright green + a dark casing stroke keeps the
// marks visible on bright and dark pixels alike, in both themes.
const MARKER_GREEN = "#3dff88";
const MARKER_CASING = "rgba(2, 10, 6, 0.85)";
function markerColor() { return MARKER_GREEN; }

// Accepted samples are shaded by their sweep z layer (TCP z): the lowest
// layer draws dark green, the highest light green, so the layers of a
// multi-level calibration read apart on the overlay. A single-layer run
// keeps the plain bright green.
const MARKER_LAYER_DARK = [10, 100, 52];     // #0a6434
const MARKER_LAYER_LIGHT = [141, 255, 183];  // #8dffb7
const MARKER_LAYER_TOL_MM = 2;               // settle jitter within one layer

function markerLayerZs() {
    const zs = [];
    for (const m of calMarkers) {
        const z = (m.tcp && m.tcp[2]) || 0;
        if (!zs.some(v => Math.abs(v - z) <= MARKER_LAYER_TOL_MM)) zs.push(z);
    }
    return zs.sort((a, b) => a - b);
}

function markerLayerColor(m, zs) {
    if (zs.length < 2) return MARKER_GREEN;
    const z = (m.tcp && m.tcp[2]) || 0;
    let idx = zs.findIndex(v => Math.abs(v - z) <= MARKER_LAYER_TOL_MM);
    if (idx < 0) idx = 0;
    const t = idx / (zs.length - 1);
    const c = MARKER_LAYER_DARK.map((d, i) => Math.round(d + (MARKER_LAYER_LIGHT[i] - d) * t));
    return "rgb(" + c.join(", ") + ")";
}

let calMarkers = [];              // accepted samples of the current run
let lastCalMarker = null;         // latest sample (accepted or rejected)
let liveMarkerDetections = null;  // latest calibration-model detect
let liveMarkerTime = null;

function handleCalibrationMarker(data) {
    if (data.reset) {
        calMarkers = [];
        lastCalMarker = null;
        liveMarkerDetections = null;
    } else {
        if (data.accepted) calMarkers.push(data);
        lastCalMarker = data;
        if (data.detections && data.detections.length) {
            liveMarkerDetections = data;
            liveMarkerTime = nowMs();
        }
    }
    renderCalibrationBar(data);
    drawOverlay();
}

function renderCalibrationBar(data) {
    const bar = byId("cal_marker_bar");
    if (!bar) return;
    bar.style.display = "flex";
    const mode = data.mode ? data.mode + " calibration" : "calibration";
    if (data.reset) {
        setPill("cal_marker_pill", null, mode + " started");
        setText("cal_marker_text", data.model_id ? "marker model: " + data.model_id : "");
        return;
    }
    const where = "(" + data.u.toFixed(0) + ", " + data.v.toFixed(0) + ") px";
    const reads = data.reads ? " · seen " + data.hits + "/" + data.reads : "";
    const conf = data.confidence ? " · conf " + data.confidence.toFixed(2) : "";
    if (data.accepted) {
        setPill("cal_marker_pill", "ok", mode + " · sample " + data.sample_index + " accepted");
        setText("cal_marker_text", "marker at " + where + " · depth " + data.depth_mm.toFixed(0) + " mm" + reads + conf +
            " · TCP (" + data.tcp.map(v => v.toFixed(0)).join(", ") + ")");
    } else {
        setPill("cal_marker_pill", "error", mode + " · sample rejected");
        setText("cal_marker_text", data.message + reads + " · TCP (" + data.tcp.map(v => v.toFixed(0)).join(", ") + ")");
    }
}

function clearCalibrationMarkers() {
    calMarkers = [];
    lastCalMarker = null;
    liveMarkerDetections = null;
    const bar = byId("cal_marker_bar");
    if (bar) bar.style.display = "none";
    drawOverlay();
}
window.clearCalibrationMarkers = clearCalibrationMarkers;

function imageToCanvas(img, width, height, imageWidth, imageHeight) {
    // Same object-fit: contain mapping drawOverlay uses for can boxes.
    const scale = Math.min(width / img.naturalWidth, height / img.naturalHeight);
    const drawnW = img.naturalWidth * scale;
    const drawnH = img.naturalHeight * scale;
    const offsetX = (width - drawnW) / 2;
    const offsetY = (height - drawnH) / 2;
    const sx = drawnW / (imageWidth || img.naturalWidth);
    const sy = drawnH / (imageHeight || img.naturalHeight);
    return (x, y) => [offsetX + x * sx, offsetY + y * sy];
}

function casedStroke(ctx, color) {
    // dark casing pass under a bright pass so the mark reads on any pixels
    ctx.lineWidth = 5;
    ctx.strokeStyle = MARKER_CASING;
    ctx.stroke();
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = color;
    ctx.stroke();
}

function casedText(ctx, text, x, y, color) {
    ctx.lineWidth = 3;
    ctx.lineJoin = "round";
    ctx.strokeStyle = MARKER_CASING;
    ctx.strokeText(text, x, y);
    ctx.fillStyle = color;
    ctx.fillText(text, x, y);
}

function drawCalibrationOverlay(ctx, img, width, height) {
    const errorColor = cssVar("--error");
    ctx.font = "600 12px 'JetBrains Mono', ui-monospace, monospace";
    ctx.lineCap = "round";

    // live detector boxes (dashed), fade out after LIVE_MARKER_MS
    if (liveMarkerDetections && liveMarkerTime && nowMs() - liveMarkerTime < LIVE_MARKER_MS) {
        const map = imageToCanvas(img, width, height, liveMarkerDetections.image_width, liveMarkerDetections.image_height);
        ctx.save();
        ctx.setLineDash([6, 4]);
        for (const det of liveMarkerDetections.detections) {
            const pts = [];
            for (let i = 0; i < 8; i += 2) pts.push(map(det.corners_px[i], det.corners_px[i + 1]));
            ctx.beginPath();
            ctx.moveTo(pts[0][0], pts[0][1]);
            for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
            ctx.closePath();
            casedStroke(ctx, markerColor());
            const [cx, cy] = map(det.center_x, det.center_y);
            ctx.setLineDash([]);
            casedText(ctx, "marker " + det.confidence.toFixed(2), cx + 8, cy - 8, markerColor());
            ctx.setLineDash([6, 4]);
        }
        ctx.restore();
    }

    // accepted markers of the run: crosshair + sample number
    const map = imageToCanvas(img, width, height, img.naturalWidth, img.naturalHeight);
    ctx.save();
    const layerZs = markerLayerZs();
    for (const m of calMarkers) {
        const color = markerLayerColor(m, layerZs);
        const [x, y] = map(m.u, m.v);
        ctx.beginPath(); ctx.arc(x, y, 8, 0, Math.PI * 2);
        casedStroke(ctx, color);
        ctx.beginPath(); ctx.moveTo(x - 13, y); ctx.lineTo(x + 13, y); ctx.moveTo(x, y - 13); ctx.lineTo(x, y + 13);
        casedStroke(ctx, color);
        casedText(ctx, "#" + m.sample_index, x + 11, y + 16, color);
    }
    // last rejection: red X where the marker was (if it had a pixel at all)
    if (lastCalMarker && !lastCalMarker.accepted && (lastCalMarker.u || lastCalMarker.v)) {
        const [x, y] = map(lastCalMarker.u, lastCalMarker.v);
        ctx.beginPath(); ctx.moveTo(x - 9, y - 9); ctx.lineTo(x + 9, y + 9); ctx.moveTo(x + 9, y - 9); ctx.lineTo(x - 9, y + 9);
        casedStroke(ctx, errorColor);
    }
    ctx.restore();
}

setInterval(function () {
    // let the dashed live boxes expire even with no new frames
    if (liveMarkerDetections && liveMarkerTime && nowMs() - liveMarkerTime >= LIVE_MARKER_MS) {
        liveMarkerDetections = null;
        drawOverlay();
    }
}, 1000);

// ------------------- pick monitor -------------------

// ordered {index,total,detection_index,outcome,verify_ran,verified,measured_z,
//          verify_delta_mm,verify_message,message}
// verify_ran=false with a non-empty verify_message = the post-place depth
// check could not measure (inconclusive) — never shown as confirmed.
let pickEvents = [];

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
                if (event.verify_ran) {
                    const delta = Number(event.verify_delta_mm || 0);
                    const deltaText = "Δ " + (delta >= 0 ? "+" : "") + delta.toFixed(0) + " mm";
                    if (event.verified) {
                        line = "picked · verified";
                        note = deltaText + " · z " + event.measured_z.toFixed(0) + " mm ✓";
                    } else {
                        line = "picked · depth not confirmed";
                        note = deltaText + " — still present";
                    }
                } else {
                    line = "picked · verify inconclusive";
                    note = event.verify_message ? "verify inconclusive: " + event.verify_message : "no depth verify";
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
    lastHealthTime = nowMs();
    lastHealth = data;
    noteCameraHealth(data);   // feeds the in-frame stream telemetry + sparklines
    renderHealth();
}

function renderHealth() {
    const list = byId("health_list");
    if (!list || !lastHealth) return;

    const rows = lastHealth.nodes.map(node => {
        const chips = Object.entries(node.metrics || {}).map(([k, v]) => {
            const bad = v === "false" && /connected|calibrated|model_loaded|intrinsics|trajectories_ready|primitives_ready/.test(k);
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
    renderTooling();

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
    const age = (nowMs() - lastHealthTime) / 1000;
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
    setText("task_state", (data.state || "—") + (data.active_task ? " · " + data.active_task : ""));
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
    taskRunning = !(data.state === "idle" || data.state === "disconnected");
    renderTooling();
}

// ------------------- workflow selector -------------------
// task_manager publishes its registry (latched /task/workflows); web_node
// forwards it as {"type": "workflows"} and replays it on connect.

let workflows = [];

function handleWorkflows(data) {
    workflows = data.workflows || [];
    const select = byId("workflow_select");
    if (!select) return;

    const previous = select.value;
    select.innerHTML = "";
    for (const wf of workflows) {
        const option = document.createElement("option");
        option.value = wf.id;
        option.textContent = wf.label + " · needs " + (wf.requires || "?") +
            (wf.ready ? "" : " (not ready)");
        option.disabled = !wf.ready;
        if (!wf.ready && wf.reason) option.title = wf.reason;
        select.appendChild(option);
    }

    const keep = workflows.some(wf => wf.id === previous) ? previous : data.default;
    if (keep) select.value = keep;
    onWorkflowSelected();
}

function onWorkflowSelected() {
    const select = byId("workflow_select");
    const wf = workflows.find(w => w.id === (select ? select.value : ""));
    const start = byId("workflow_start");
    if (start) start.disabled = !wf || !wf.ready;
    let text = wf ? ("requires " + (wf.requires || "?") + " — " + wf.description) : "";
    if (wf && !wf.ready && wf.reason) text += " — not ready: " + wf.reason;
    setText("workflow_description", text);
}

function startSelectedWorkflow() {
    const select = byId("workflow_select");
    const id = select ? select.value : "";
    if (!id) {
        setText("robot_command_status", "no workflow selected");
        return;
    }
    sendCommand(Receiver.COBOT, CobotCommands.START_WORKFLOW, { workflow: id });
}

window.onWorkflowSelected = onWorkflowSelected;
window.startSelectedWorkflow = startSelectedWorkflow;

// ------------------- tooling -------------------
// cobot_control reads the YRC1000 profile ONCE after connect (system info,
// TOOL.CND tool files, UFRAME.CND user frames — read-only) and latches it on
// /cobot/profile; web_node forwards it as {"type": "tool_profile"}. The
// active tool (the tool file every pose is expressed in) is selected here via
// ("COBOT", "set_active_tool") and shows up on /cobot/state → "tool_state".

let toolProfile = null;
let toolState = null;
let lastSystemState = null;
let taskRunning = false;
let toolSelectDirty = false;

function handleToolProfile(data) {
    toolProfile = data;
    renderTooling();
}

function handleToolState(data) {
    toolState = data;
    renderTooling();
}

function fmt(v, digits = 1) {
    return (typeof v === "number" && isFinite(v)) ? v.toFixed(digits) : "—";
}

function renderTooling() {
    const select = byId("tool_select");
    if (!select) return;

    const active = toolState ? toolState.active_tool_number : (toolProfile ? toolProfile.active_tool_number : -1);
    const activeName = toolState ? toolState.active_tool_name : (toolProfile ? toolProfile.active_tool_name : "");
    const connected = !!(toolState && toolState.connected);
    const profileLoaded = !!(toolProfile && toolProfile.loaded && toolProfile.tools.length);

    // header pills
    if (active >= 0 && activeName) {
        const verified = toolProfile ? toolProfile.active_tool_verified : false;
        setPill("tool_active_pill", connected ? (profileLoaded && !verified ? "warn" : "ok") : "stale",
            "active: " + activeName + " (tool " + active + ")" + (profileLoaded && !verified ? " · not in TOOL.CND" : ""));
    } else {
        setPill("tool_active_pill", "stale", "active: ?");
    }

    // controller line + errors
    if (toolProfile) {
        const parts = [];
        if (toolProfile.model_name) parts.push(toolProfile.model_name);
        if (toolProfile.software_version) parts.push(toolProfile.software_version);
        parts.push(toolProfile.loaded ? ("profile " + toolProfile.source + " · read " + (toolProfile.read_at || "?") + " (#" + toolProfile.read_count + ")") : "controller profile not read yet");
        setText("tool_controller", parts.join(" · "));
        const errs = byId("tool_errors");
        if (errs) {
            errs.innerHTML = "";
            for (const e of toolProfile.errors || []) {
                const pill = document.createElement("span");
                pill.className = "pill sm lvl-warn";
                pill.innerHTML = '<span class="dot"></span>' + escapeHtml(e);
                errs.appendChild(pill);
            }
        }
    }

    // selector: none + every configured tool; disabled when the controller
    // does not have that tool file (or the profile is not loaded yet)
    const known = new Set((toolProfile ? toolProfile.tools : []).map(t => t.number));
    const configured = toolProfile ? toolProfile.configured : [];
    const options = [{ name: "none", tool_number: 0 }].concat(configured.filter(c => c.name !== "none"));
    const previous = select.value;
    select.innerHTML = "";
    for (const opt of options) {
        const o = document.createElement("option");
        o.value = opt.name;
        const onController = opt.tool_number === 0 || known.has(opt.tool_number);
        o.textContent = opt.name + " · tool file " + opt.tool_number + (onController ? "" : " (not on controller)");
        o.disabled = profileLoaded && !onController;
        if (o.disabled) o.title = "tool file " + opt.tool_number + " is not in the controller's TOOL.CND";
        select.appendChild(o);
    }
    if (toolSelectDirty && options.some(o => o.name === previous)) {
        select.value = previous;
    } else {
        select.value = options.some(o => o.name === activeName) ? activeName : "none";
        toolSelectDirty = false;
    }
    const canApply = connected && toolState && !toolState.busy && !taskRunning;
    select.disabled = !canApply;
    const apply = byId("tool_apply");
    if (apply) {
        apply.disabled = !canApply || select.value === activeName;
        apply.title = !connected ? "robot not connected" : (toolState && toolState.busy ? "a move is executing" :
            (taskRunning ? "a task is running" : ""));
    }

    // remembered positions vs active tool (cobot_operation heartbeat metrics)
    const op = lastHealth ? (lastHealth.nodes || []).find(n => n.name === "cobot_operation") : null;
    const m = op ? (op.metrics || {}) : {};
    const trajReady = m.trajectories_ready;
    const convertBtn = byId("tool_convert");
    if (!op) {
        setPill("tool_traj_pill", "stale", "positions: ?");
    } else if (trajReady === "True" || trajReady === "true") {
        setPill("tool_traj_pill", "ok", "positions in tool " + (m.trajectories_tool_numbers || "?") +
            " (" + (m.trajectories_count || "?") + ", " + (m.trajectories_with_joints || 0) + " with joints)");
    } else {
        setPill("tool_traj_pill", "warn", "positions in tool " + (m.trajectories_tool_numbers || "?") +
            " ≠ active " + active + " — picks refused until converted");
    }
    const pill = byId("tool_traj_pill");
    if (pill && m.last_convert) pill.title = m.last_convert;
    if (convertBtn) {
        convertBtn.disabled = !(op && connected && !taskRunning && (m.convert_ready === "True" || m.convert_ready === "true"));
    }

    // calibration tool badge (task_manager system_state)
    const ss = lastSystemState;
    if (!ss || ss.calibration_tool_number === undefined) {
        setPill("tool_cal_pill", "stale", "cal: ?");
    } else if (ss.calibration_tool_number < 0) {
        setPill("tool_cal_pill", "stale", "cal: tool unknown");
    } else if (ss.calibration_tool_mismatch) {
        setPill("tool_cal_pill", "warn", "cal: tool " + ss.calibration_tool_number + " ≠ active " + ss.active_tool_number);
    } else {
        setPill("tool_cal_pill", "ok", "cal: tool " + ss.calibration_tool_number);
    }

    // tool table — TOOL.CND holds a slot for every tool file, most of them
    // blank; only show slots that carry a name, plus anything meaningful even
    // when unnamed (the active tool and tools referenced by a cell profile).
    const tbody = byId("tool_table") ? byId("tool_table").querySelector("tbody") : null;
    if (tbody) {
        const byNumber = {};
        for (const c of configured) byNumber[c.tool_number] = c.name;
        const allTools = (toolProfile ? toolProfile.tools : []);
        const tools = allTools.filter(t =>
            (t.name && t.name.trim()) || t.number === active || byNumber[t.number]);
        if (active === 0 && !tools.some(t => t.number === 0)) {
            tools.unshift({ number: 0, name: "(flange / none)", x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0, weight_kg: 0, is_identity: true });
        }
        if (!tools.length) {
            tbody.innerHTML = '<tr><td colspan="10" class="muted">' +
                (toolProfile && toolProfile.errors && toolProfile.errors.length ? "tool files not readable — see errors above" :
                    (allTools.length ? "no named tool files (" + allTools.length + " blank slots hidden)" : "no tool files read yet")) +
                "</td></tr>";
        } else {
            tbody.innerHTML = tools.map(t => {
                const cls = (t.number === active ? "active" : "") + (t.is_identity && t.number !== 0 ? " identity" : "");
                const tag = t.number === active ? "active" : (byNumber[t.number] ? "cell: " + byNumber[t.number] : (t.is_identity && t.number !== 0 ? "identity" : ""));
                return '<tr class="' + cls.trim() + '">' +
                    "<td>" + t.number + "</td>" +
                    "<td>" + escapeHtml(t.name || "") + "</td>" +
                    "<td>" + fmt(t.x) + "</td><td>" + fmt(t.y) + "</td><td>" + fmt(t.z) + "</td>" +
                    "<td>" + fmt(t.rx, 2) + "</td><td>" + fmt(t.ry, 2) + "</td><td>" + fmt(t.rz, 2) + "</td>" +
                    "<td>" + fmt(t.weight_kg, 2) + "</td>" +
                    '<td class="tag">' + escapeHtml(tag) + "</td>" +
                    "</tr>";
            }).join("");
        }
    }

    // user frames — same story: hide unnamed slots
    const frames = (toolProfile ? toolProfile.user_frames : []).filter(f => f.name && f.name.trim());
    setText("tool_frames_count", String(frames.length));
    const fbody = byId("tool_frames") ? byId("tool_frames").querySelector("tbody") : null;
    if (fbody) {
        fbody.innerHTML = frames.map(f => {
            const p = f.pose && f.pose.length === 6 ? f.pose : [null, null, null, null, null, null];
            return "<tr>" +
                "<td>" + f.number + "</td><td>" + escapeHtml(f.name || "") + "</td><td>" + (f.tool_number >= 0 ? f.tool_number : "—") + "</td>" +
                p.map((v, i) => "<td>" + fmt(v, i < 3 ? 1 : 2) + "</td>").join("") +
                "</tr>";
        }).join("");
    }

    // live pose line
    if (toolState && toolState.connected && toolState.pose) {
        const p = toolState.pose;
        setText("tool_pose", "TCP (tool " + toolState.pose_tool_number + "): " + p.slice(0, 3).map(v => fmt(v)).join(", ") +
            " | " + p.slice(3).map(v => fmt(v, 2)).join(", ") +
            (toolState.joints_deg && toolState.joints_deg.length ? "  ·  J " + toolState.joints_deg.map(v => fmt(v, 1)).join(" ") : ""));
    } else {
        setText("tool_pose", "");
    }
}

function onToolSelected() {
    toolSelectDirty = true;
    renderTooling();
}

function applySelectedTool() {
    const select = byId("tool_select");
    const name = select ? select.value : "";
    if (!name) {
        setText("tool_status", "no tool selected");
        return;
    }
    setText("tool_status", "switching active tool to " + name + "…");
    toolSelectDirty = false;
    sendCommand(Receiver.COBOT, CobotCommands.SET_ACTIVE_TOOL, { name: name });
}

function convertTrajectories() {
    setText("tool_status", "converting remembered positions via the controller kinematics…");
    sendCommand(Receiver.COBOT, CobotCommands.CONVERT_TRAJECTORIES);
}

window.onToolSelected = onToolSelected;
window.applySelectedTool = applySelectedTool;
window.convertTrajectories = convertTrajectories;

// ------------------- cobot panel -------------------
// web_node digests /cobot/state (cobot_control, 10 Hz) into
// {"type": "cobot_state"} at cobot_state_hz: link / servo / hold / alarms /
// control mode / named IO, the TCP pose and every joint in degrees, plus the
// per-joint range (joint_limits_deg) the bars are drawn against.

let cobotState = null;
let cobotStateTime = null;

function handleCobotState(data) {
    cobotState = data;
    cobotStateTime = nowMs();
    renderCobot();
}

function cobotSampleAge() {
    if (cobotStateTime === null) return null;
    const local = (nowMs() - cobotStateTime) / 1000;
    const bridge = cobotState && typeof cobotState.age_s === "number" ? cobotState.age_s : 0;
    return local + bridge;
}

function renderCobot() {
    const s = cobotState;
    if (!s) return;
    const age = cobotSampleAge();
    const fresh = age !== null && age * 1000 <= COBOT_STALE_MS;
    setText("cobot_age", age === null ? "" : "sample " + age.toFixed(1) + " s ago · poll " + fmt(s.poll_ms, 0) + " ms");

    // header pill: the one line an operator reads first
    if (!fresh) {
        setPill("cobot_link_pill", "stale", "state stale (" + (age === null ? "?" : age.toFixed(0)) + " s)");
    } else if (!s.connected) {
        setPill("cobot_link_pill", "error", "not connected");
    } else if (s.alarming) {
        setPill("cobot_link_pill", "error", "ALARM · " + (s.alarms.length ? s.alarms[0].name || ("code " + s.alarms[0].code) : "see list"));
    } else if (s.erroring) {
        setPill("cobot_link_pill", "error", "controller error");
    } else if (!s.servo_on) {
        setPill("cobot_link_pill", "warn", "servos off");
    } else if (s.control_mode === "teach") {
        setPill("cobot_link_pill", "warn", "teach mode — motion blocked");
    } else if (s.in_hold) {
        setPill("cobot_link_pill", "warn", "hold");
    } else if (s.paused) {
        setPill("cobot_link_pill", "warn", "paused");
    } else if (s.busy || s.running) {
        setPill("cobot_link_pill", "ok", s.running ? "moving" : "busy");
    } else {
        setPill("cobot_link_pill", "ok", "connected · idle");
    }

    // robot-specific health facts (grey = unknown because the sample is stale)
    const st = (good, bad) => fresh ? (good ? "ok" : bad) : "stale";
    const facts = [
        ["Link", st(s.connected, "error"), "YMConnect link to the controller"],
        ["Servos " + (s.servo_on ? "on" : "off"), st(s.servo_on, "error"), "servo power"],
        ["Mode: " + (s.control_mode || "-"), fresh ? (s.control_mode === "teach" ? "error" : s.control_mode === "unknown" ? "stale" : "ok") : "stale", "controller mode (teach blocks remote motion)"],
        ["Alarm", st(!s.alarming, "error"), "active controller alarm"],
        ["Error", st(!s.erroring, "error"), "controller error flag"],
        ["Hold", st(!s.in_hold, "warn"), "controller HOLD"],
        ["Paused", st(!s.paused, "warn"), "cobot_control pause latch"],
        [s.running ? "Moving" : "Stationary", fresh ? "ok" : "stale", "controller reports motion in progress"],
        ["Goal " + (s.busy ? "executing" : "none"), fresh ? "ok" : "stale", "a MoveToPose goal is executing"],
        ["Tool: " + (s.active_tool_name || "-") + " (" + s.active_tool_number + ")", fresh && s.active_tool_number >= 0 ? "ok" : "stale", "active tool file"],
    ];
    const box = byId("cobot_facts");
    if (box) {
        box.innerHTML = "";
        for (const [label, level, tip] of facts) {
            const pill = document.createElement("span");
            pill.className = "pill sm lvl-" + level;
            pill.innerHTML = '<span class="dot"></span>' + escapeHtml(label);
            pill.title = tip;
            box.appendChild(pill);
        }
    }

    // alarms
    const alarms = byId("cobot_alarms");
    if (alarms) {
        alarms.innerHTML = (s.alarms || []).map(a =>
            '<div class="alarm"><span class="code">' + escapeHtml(a.code + (a.subcode ? "." + a.subcode : "")) + "</span>" +
            "<span>" + escapeHtml(a.name || "alarm") + "</span>" +
            '<span class="mono">' + escapeHtml(a.time || "") + "</span></div>"
        ).join("");
    }

    // joints: one bar per axis, value positioned inside its configured range
    const joints = byId("joint_list");
    if (joints) {
        const vals = s.joints_deg || [];
        if (!vals.length) {
            joints.innerHTML = '<p class="muted">' + (s.connected ? "joint read failed or poll_joints=false" : "no joint data (robot not connected)") + "</p>";
        } else {
            joints.innerHTML = vals.map((v, i) => {
                const lim = (s.joint_limits_deg && s.joint_limits_deg[i]) || [-180, 180];
                const span = Math.max(lim[1] - lim[0], 1e-6);
                const pct = Math.min(100, Math.max(0, (v - lim[0]) / span * 100));
                const zero = Math.min(100, Math.max(0, (0 - lim[0]) / span * 100));
                const margin = Math.min(v - lim[0], lim[1] - v);
                const cls = margin <= 5 ? " near-limit" : (margin <= 15 ? " close" : "");
                return '<div class="joint' + (fresh ? "" : " stale") + cls + '">' +
                    '<span class="jname">' + (JOINT_NAMES[i] || ("J" + (i + 1))) + "</span>" +
                    '<div class="jbar"><i class="zero" style="left:' + zero.toFixed(1) + '%"></i><i class="fill" style="left:' + Math.min(zero, pct).toFixed(1) + "%;width:" + Math.abs(pct - zero).toFixed(1) + '%"></i><b style="left:' + pct.toFixed(1) + '%"></b></div>' +
                    '<span class="jval">' + fmt(v, 2) + "°</span>" +
                    '<span class="jlim">' + fmt(lim[0], 0) + " … " + fmt(lim[1], 0) + "</span>" +
                    "</div>";
            }).join("");
        }
    }

    // pose
    setText("pose_tool", s.connected ? "· tool " + s.pose_tool_number : "");
    const pose = byId("pose_list");
    if (pose) {
        const labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"];
        const units = ["mm", "mm", "mm", "°", "°", "°"];
        pose.innerHTML = (s.pose || []).map((v, i) =>
            '<div class="pv' + (fresh ? "" : " stale") + '"><span class="k">' + labels[i] + "</span><span class=\"v\">" + fmt(v, i < 3 ? 1 : 2) + '</span><span class="u">' + units[i] + "</span></div>"
        ).join("");
    }

    // named IO from cobot_setup.yaml
    const io = byId("cobot_io");
    if (io) {
        io.innerHTML = "";
        const add = (name, on, kind) => {
            const pill = document.createElement("span");
            pill.className = "pill sm " + (fresh ? (on ? "lvl-ok" : "pill-neutral") : "lvl-stale");
            pill.innerHTML = '<span class="dot"></span>' + escapeHtml(name);
            pill.title = kind + (on ? " = on" : " = off");
            io.appendChild(pill);
        };
        for (const [n, v] of Object.entries(s.sensors || {})) add(n, v, "sensor");
        for (const [n, v] of Object.entries(s.outputs || {})) add(n + " ⏻", v, "output");
        if (!io.childElementCount) io.innerHTML = '<span class="muted">no named IO</span>';
    }
}

setInterval(function () {
    // keep the stale marking honest even if the bridge stops pushing
    if (cobotState) renderCobot();
}, 1000);

// ------------------- derived system state -------------------
// task_manager aggregates /cobot/state + /health/summary into a read-only
// capability (none/see/move/operate) with the individual facts behind it;
// web_node forwards it as {"type": "system_state"} and replays on connect.

const CAPABILITY_LEVELS = { operate: "ok", see: "warn", move: "warn", none: "error" };

function handleSystemState(data) {
    lastSystemState = data;
    const level = CAPABILITY_LEVELS[data.capability] || "unknown";
    setPill("capability_pill", level, data.capability || "?");
    const pill = byId("capability_pill");
    if (pill) {
        const blockers = (data.move_blockers || []).concat(data.see_blockers || []);
        pill.title = blockers.join("\n");
    }
    renderSystemDots(data);
    renderTooling();
}

function renderSystemDots(data) {
    const box = byId("system_dots");
    if (!box) return;

    const robotFresh = data.robot_state_fresh;
    const healthFresh = data.health_fresh;
    // [label, level, tooltip] — lvl-stale (grey) = the owning side is not
    // fresh, so the value is unknown rather than bad.
    const mode = data.control_mode || "-";
    const facts = [
        ["Connected", robotFresh ? (data.connected ? "ok" : "error") : "stale", "YMConnect link"],
        ["Servos", robotFresh ? (data.servo_on ? "ok" : "error") : "stale", "servo power"],
        ["Mode: " + mode, !robotFresh || mode === "-" ? "stale" : (mode === "teach" ? "error" : "ok"),
            "YRC1000 control mode (teach blocks motion)"],
        ["Alarm", robotFresh ? (data.alarming ? "error" : "ok") : "stale", "controller alarm"],
        ["Paused", robotFresh ? (data.paused ? "warn" : "ok") : "stale", "pause latch"],
        ["Camera", healthFresh ? (data.camera_ok ? "ok" : "error") : "stale", "color+depth streams"],
        ["Calibrated", healthFresh ? (data.calibrated ? "ok" : "error") : "stale", "camera→robot transform"],
        ["Depth", healthFresh ? (data.depth_fresh ? "ok" : "warn") : "stale", "depth buffer freshness"],
        ["Detector", healthFresh ? (data.detector_ok ? "ok" : "warn") : "stale", "ONNX detector"],
        ["Tool: " + (robotFresh && data.active_tool_name ? data.active_tool_name : "-"),
            robotFresh && data.active_tool_number >= 0 ? "ok" : "stale", "active YRC1000 tool file (Tooling card)"],
        ["Cal tool", !healthFresh || data.calibration_tool_number === undefined || data.calibration_tool_number < 0 ? "stale" :
            (data.calibration_tool_mismatch ? "warn" : "ok"),
            "tool file the camera calibration was captured with (mismatch = warning, recalibrate if picks drift)"],
    ];

    box.innerHTML = "";
    for (const [label, level, tip] of facts) {
        const pill = document.createElement("span");
        pill.className = "pill sm lvl-" + level;
        pill.innerHTML = '<span class="dot"></span>' + escapeHtml(label);
        pill.title = tip;
        box.appendChild(pill);
    }
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
    event.stopPropagation();   // this click is the marker centre, not a "focus stream" click

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

// ------------------- session recording (server-side) -------------------
// web_node records every broadcast while armed; stop exports a standalone
// replay HTML into /results. This block only mirrors the recorder status
// ({"type": "recording"}) onto the topbar button + download link.

let recordingState = null;
let recordingBaseMs = null;
let recordingTick = null;

function renderRecordButton() {
    const btn = byId("record_btn");
    if (!btn || !recordingState || !recordingState.active) return;
    const s = Math.max(0, Math.round((Date.now() - recordingBaseMs) / 1000));
    btn.textContent = "■ stop " + Math.floor(s / 60) + ":" + String(s % 60).padStart(2, "0");
}

function handleRecording(data) {
    recordingState = data;
    const btn = byId("record_btn");
    const link = byId("record_link");
    if (data.active) {
        if (link) link.style.display = "none";
        if (btn) btn.classList.add("danger");
        recordingBaseMs = Date.now() - (data.duration_s || 0) * 1000;
        if (recordingTick === null) recordingTick = setInterval(renderRecordButton, 1000);
        renderRecordButton();
        return;
    }
    if (recordingTick !== null) { clearInterval(recordingTick); recordingTick = null; }
    if (btn) { btn.classList.remove("danger"); btn.textContent = "● Record"; }
    if (!link) return;
    if (data.file) {
        link.href = data.file;
        link.download = data.file.split("/").pop();
        const why = data.reason && data.reason !== "stopped" ? " (" + data.reason + ")" : "";
        link.className = "pill lvl-ok";
        link.innerHTML = '<span class="dot"></span>' + escapeHtml("replay ready" + why + " — download");
        link.style.display = "";
    } else if (data.reason === "export_failed" || data.reason === "write_error") {
        link.removeAttribute("href");
        link.className = "pill lvl-error";
        link.innerHTML = '<span class="dot"></span>' + escapeHtml("recording failed: " + data.reason);
        link.style.display = "";
    }
}

function toggleRecording() {
    const active = recordingState && recordingState.active;
    sendCommand(Receiver.RECORD, active ? RecordCommands.STOP : RecordCommands.START);
}
window.toggleRecording = toggleRecording;

// -----------------------------------------------------------------

function dispatchMessage(data) {
    switch (data.type) {
        case "camera_frame":
            handleCameraFrame(data);
            return;
        case "command_result":
            if (data.command === "set_active_tool" || data.command === "convert_trajectories" || data.command === "teach_point") {
                setText("tool_status", data.command + " → " + data.result.message);
            } else {
                setText("robot_command_status", data.command + " → " + data.result.message);
            }
            return;
        case "cobot_status":
            handleTaskStatus(data);
            return;
        case "workflows":
            handleWorkflows(data);
            return;
        case "system_state":
            handleSystemState(data);
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
        case "calibration_marker":
            handleCalibrationMarker(data);
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
        case "tool_profile":
            handleToolProfile(data);
            return;
        case "tool_state":
            handleToolState(data);
            return;
        case "cobot_state":
            handleCobotState(data);
            return;
        case "recording":
            handleRecording(data);
            return;
        default:
            console.log("WebSocket message:", data);
    }
}

function connectWebSocket() {
    const wsProtocol = window.location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(`${wsProtocol}://${window.location.host}/ws`);

    socket.onopen = function () {
        console.log("WebSocket connected");
        setPill("ws_status", null, "connected");
    };

    socket.onmessage = function (event) {
        dispatchMessage(JSON.parse(event.data));
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

// Standalone replay support: everything incremental the message handlers
// accumulate, re-initialized so replay.js can seek by re-dispatching the
// recorded prefix from t=0 into a clean UI.
function resetForReplay() {
    lastDetections = null;
    pickStateByDetection.clear();
    pickEvents = [];
    calMarkers = [];
    lastCalMarker = null;
    liveMarkerDetections = null;
    liveMarkerTime = null;
    const calBar = byId("cal_marker_bar");
    if (calBar) calBar.style.display = "none";
    lastFrameTime = lastColorTime = lastDepthTime = null;
    frameWindowStart = null;
    for (const k of Object.keys(frameCounts)) { frameCounts[k] = 0; wsHz[k] = 0; }
    cameraSpark.color.length = 0;
    cameraSpark.depth.length = 0;
    lastDropped.color = null;
    lastDropped.depth = null;
    cameraMetrics = null;
    lastHealthTime = null;
    lastHealth = null;
    cobotState = null;
    cobotStateTime = null;
    toolProfile = null;
    toolState = null;
    lastSystemState = null;
    taskRunning = false;
    toolSelectDirty = false;
    workflows = [];
    setMarkerPrompt(false);
    setText("robot_command_status", "");
    setText("tool_status", "");
    setText("cobot_feedback", "No cobot messages yet.");
    renderPickMonitor();
    drawOverlay();
}

/**
 * @typedef {"CAMERA" | "COBOT" | "VISION"} ReceiverType
 */

/**
 * @param {ReceiverType} receiver
 */
function sendCommand(receiver, command, payload = null) {
    if (window.__COBOT_REPLAY__) {
        // Standalone replay export — there is no server to talk to.
        setText("robot_command_status", "replay mode — commands disabled");
        return;
    }

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
window.RecordCommands = RecordCommands;

// Replay hooks: in a standalone export (session_recorder.py) replay.js is
// concatenated above this module and window.__COBOT_REPLAY__ holds the
// recording — hand it the dispatch/reset entry points and skip the socket.
// On the live page neither global exists and this is a plain no-op path.
window.__cobotDispatch = dispatchMessage;
window.__cobotResetUI = resetForReplay;
if (window.__COBOT_REPLAY__ && window.__cobotStartReplay) {
    window.__cobotStartReplay(window.__COBOT_REPLAY__);
} else {
    connectWebSocket();
}
renderPickMonitor();

// TEMPORARY manual centre select — wire the color feed once at load.
const colorFeed = byId("rgb_image");
if (colorFeed) {
    colorFeed.addEventListener("click", handleColorFeedClick);
    colorFeed.addEventListener("load", drawOverlay);
}

initFeedLayout();
themeButtonLabel();
refreshStreamStatus();
