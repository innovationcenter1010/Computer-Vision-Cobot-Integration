// Developer dashboard (/dev): a long list of the PHYSICAL tunables
// (cobot_params.yaml, saved to disk AND live-applied via set_parameters — no
// restart), the cell setup values (cobot_setup.yaml, applies on restart) and
// the remembered trajectories (edit / capture / save per point, persisted
// through cobot_operation so its in-memory store stays the truth).
//
// Deliberately NO streams and NO job controls: the websocket connects with
// frames=off so the server never sends this page camera frames — tuning here
// can't degrade the operator panel's stream. Run jobs from the operator panel.

let socket = null;

// ---------------------------------------------------------------- helpers

function byId(id) { return document.getElementById(id); }
function setText(id, text) { const el = byId(id); if (el) el.innerText = text; }

function setPill(id, level, text) {
    const el = byId(id);
    if (!el) return;
    el.className = "pill " + (level ? "lvl-" + level : "pill-neutral");
    el.innerHTML = '<span class="dot"></span>' + escapeHtml(text);
}

function escapeHtml(v) {
    return String(v)
        .replace(/&/g, "&amp;").replace(/</g, "&lt;")
        .replace(/>/g, "&gt;").replace(/"/g, "&quot;");
}

function setRowStatus(el, ok, text) {
    el.className = "pstat " + (ok === null ? "" : ok ? "ok" : "err");
    el.innerText = text;
    el.title = text;
}

function fmtNum(v) {
    const n = Number(v);
    return Number.isInteger(n) ? String(n) : String(Math.round(n * 1000) / 1000);
}

// ---------------------------------------------------------------- theme

function toggleTheme() {
    const theme = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
    document.documentElement.dataset.theme = theme;
    try { localStorage.setItem("cobot.theme", theme); } catch (e) { /* storage unavailable */ }
    themeButtonLabel();
}

function themeButtonLabel() {
    const btn = byId("theme_toggle");
    if (btn) btn.innerText = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
}

// ---------------------------------------------------------------- websocket

function connectWebSocket() {
    const wsProtocol = window.location.protocol === "https:" ? "wss" : "ws";
    // frames=off: the server keeps camera frames off this socket entirely.
    socket = new WebSocket(`${wsProtocol}://${window.location.host}/ws?frames=off`);

    socket.onopen = () => setPill("ws_status", null, "connected");

    socket.onmessage = function (event) {
        const data = JSON.parse(event.data);
        switch (data.type) {
            case "cobot_status": handleTaskStatus(data); return;
            case "cobot_state": handleCobotState(data); return;
            default: return; // detections / health / … — operator-panel concerns
        }
    };

    socket.onclose = function () {
        setPill("ws_status", "error", "disconnected");
        setTimeout(connectWebSocket, 2000);
    };
    socket.onerror = function () { setPill("ws_status", "error", "error"); };
}

function handleTaskStatus(data) {
    const level = data.state === "error" ? "error"
        : data.state === "idle" || data.state === "disconnected" ? null : "ok";
    setPill("task_pill", level, data.state + (data.active_task ? " · " + data.active_task : ""));
}

function handleCobotState(data) {
    const p = data.pose || [];
    if (p.length === 6) {
        setText("live_pose",
            `x ${fmtNum(p[0])}  y ${fmtNum(p[1])}  z ${fmtNum(p[2])}  ` +
            `rx ${fmtNum(p[3])}  ry ${fmtNum(p[4])}  rz ${fmtNum(p[5])}  ` +
            `· tool ${data.pose_tool_number} (${data.active_tool_name || "?"})`);
    }
}

// ---------------------------------------------------------------- tunable params (cobot_params.yaml)
// PHYSICAL parameters only — geometry, tolerances, limits and the calibration
// sweep volume. Networking / rates / paths stay in the YAML.

const PARAM_SPECS = [
    { section: "Vision Geometry", node: "localization_node", params: [
        { name: "can_height_mm", label: "Can Height", unit: "mm" },
        { name: "depth_at_top_layer_mm", label: "Depth At Top Layer", unit: "mm" },
        { name: "pathing_row_tolerance_mm", label: "Pathing Row Tolerance", unit: "mm" },
        { name: "depth_sample_window_px", label: "Depth Sample Window", unit: "px", int: true },
        { name: "verify_tolerance_mm", label: "Verify Tolerance", unit: "mm" },
        { name: "verify_max_depth_age_s", label: "Verify Max Depth Age", unit: "s" },
        { name: "calibration_max_rms_mm", label: "Calibration Max RMS", unit: "mm" },
    ]},
    { section: "Joint Limits", node: "cobot_control", params: [
        { name: "joint_limit_margin_deg", label: "Joint Limit Margin", unit: "deg" },
        { name: "joint_limits_min_deg", label: "Joint Limits Min", unit: "deg · S L U R B T", list: 6 },
        { name: "joint_limits_max_deg", label: "Joint Limits Max", unit: "deg · S L U R B T", list: 6 },
    ]},
    { section: "Calibration Sweep", node: "task_manager", params: [
        { name: "calibration_x_range_mm", label: "Calibration X Range", unit: "mm · min max", list: 2 },
        { name: "calibration_y_range_mm", label: "Calibration Y Range", unit: "mm · min max", list: 2 },
        { name: "calibration_z_levels_mm", label: "Calibration Z Levels", unit: "mm · any count", list: -1 },
        { name: "calibration_grid_size", label: "Calibration Grid", unit: "cols rows", list: 2, int: true },
        { name: "calibration_orientation_deg", label: "Calibration Orientation", unit: "deg · Rx Ry Rz", list: 3 },
        { name: "calibration_settle_sec", label: "Calibration Settle", unit: "s" },
    ]},
];

function parseInputValue(spec, text) {
    const parts = String(text).trim().split(/[\s,]+/).filter(s => s.length);
    const nums = parts.map(Number);
    if (nums.some(n => !Number.isFinite(n))) throw new Error("not a number: " + text);
    if (spec.int && nums.some(n => !Number.isInteger(n))) throw new Error("integers only");
    if (spec.list === undefined) {
        if (nums.length !== 1) throw new Error("expected one number");
        return nums[0];
    }
    if (spec.list > 0 && nums.length !== spec.list) {
        throw new Error(`expected ${spec.list} numbers, got ${nums.length}`);
    }
    if (nums.length === 0) throw new Error("expected numbers");
    return nums;
}

function paramRow(spec, currentValue, meta, onSave) {
    const row = document.createElement("div");
    row.className = "prow";

    const name = document.createElement("div");
    name.className = "pname";
    name.innerHTML = escapeHtml(spec.label) +
        '<span class="pmeta">' + escapeHtml(meta) + (spec.unit ? " · " + escapeHtml(spec.unit) : "") + "</span>";

    const input = document.createElement("input");
    input.className = "pinput";
    input.type = "text";
    input.spellcheck = false;
    input.value = Array.isArray(currentValue) ? currentValue.map(fmtNum).join(", ")
        : currentValue === undefined || currentValue === null ? "" : fmtNum(currentValue);
    input.addEventListener("input", () => { row.classList.add("dirty"); });

    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "btn sm";
    btn.innerText = "Save";

    const stat = document.createElement("div");
    stat.className = "pstat";

    btn.addEventListener("click", async () => {
        let value;
        try { value = parseInputValue(spec, input.value); }
        catch (e) { setRowStatus(stat, false, e.message); return; }
        btn.disabled = true;
        setRowStatus(stat, null, "saving…");
        try {
            const outcome = await onSave(value);
            setRowStatus(stat, outcome.ok, outcome.message);
            if (outcome.ok) row.classList.remove("dirty");
        } catch (e) {
            setRowStatus(stat, false, String(e));
        } finally {
            btn.disabled = false;
        }
    });
    input.addEventListener("keydown", e => { if (e.key === "Enter") btn.click(); });

    row.append(name, input, btn, stat);
    return row;
}

function sectionHead(title, file) {
    const head = document.createElement("div");
    head.className = "dev-section";
    head.innerHTML = '<span class="eyebrow">' + escapeHtml(title) + '</span>' +
        (file ? '<span class="file">' + escapeHtml(file) + "</span>" : "");
    return head;
}

async function loadParams() {
    const list = byId("param_list");
    let config;
    try {
        config = await (await fetch("/api/config")).json();
    } catch (e) {
        list.innerHTML = '<p class="muted">failed to load /api/config: ' + escapeHtml(e) + "</p>";
        return;
    }
    list.innerHTML = "";
    for (const group of PARAM_SPECS) {
        const section = (config[group.node] || {}).ros__parameters || {};
        list.appendChild(sectionHead(group.section, group.node));
        for (const spec of group.params) {
            list.appendChild(paramRow(spec, section[spec.name], spec.name, async (value) => {
                const res = await fetch("/api/param", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ node: group.node, name: spec.name, value }),
                });
                const data = await res.json();
                if (data.status !== "saved") return { ok: false, message: data.message || "save failed" };
                return { ok: true, message: "saved · " + (data.applied || "") };
            }));
        }
    }
}

// ---------------------------------------------------------------- cell setup (cobot_setup.yaml)
// Physical per-tool cell values + motion profiles, keyed by whatever the YAML
// actually holds. Saved to disk; the nodes load this file at startup.

const TOOL_FIELDS = [
    { key: "tip_offset_mm", label: "Tip Offset", unit: "mm · x y z", list: 3 },
    { key: "marker_offset_mm", label: "Marker Offset", unit: "mm · x y z", list: 3 },
    { key: "compression_mm", label: "Compression", unit: "mm" },
    { key: "pick_tip_z_mm", label: "Pick Tip Z", unit: "mm" },
    { key: "approach_tip_z_mm", label: "Approach Tip Z", unit: "mm" },
];

const PROFILE_FIELDS = [
    { key: "speed", label: "Speed", unit: "joint: % · linear: mm/s" },
    { key: "accel_ratio", label: "Accel Ratio", unit: "%" },
    { key: "decel_ratio", label: "Decel Ratio", unit: "%" },
    { key: "accuracy_mm", label: "Accuracy", unit: "mm · -1 round, 0 exact", int: true },
];

function setupSaver(path) {
    return async (value) => {
        const res = await fetch("/api/setup-param", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ path, value }),
        });
        const data = await res.json();
        if (data.status !== "saved") return { ok: false, message: data.message || "save failed" };
        return { ok: true, message: "saved · applies on restart" };
    };
}

async function loadSetup() {
    const list = byId("setup_list");
    let setup;
    try {
        setup = await (await fetch("/api/setup")).json();
    } catch (e) {
        list.innerHTML = '<p class="muted">failed to load /api/setup: ' + escapeHtml(e) + "</p>";
        return;
    }
    list.innerHTML = "";
    if (!setup || Object.keys(setup).length === 0) {
        list.innerHTML = '<p class="muted">cobot_setup.yaml not available — is setup_file set on web_node?</p>';
        return;
    }
    for (const [profileName, profile] of Object.entries(setup.motion_profiles || {})) {
        list.appendChild(sectionHead("Motion Profile · " + profileName,
            "interpolation: " + (profile.interpolation || "?")));
        for (const f of PROFILE_FIELDS) {
            if (!(f.key in profile)) continue;
            list.appendChild(paramRow(f, profile[f.key], "motion_profiles." + profileName + "." + f.key,
                setupSaver(["motion_profiles", profileName, f.key])));
        }
    }
    for (const [toolName, tool] of Object.entries(setup.tools || {})) {
        list.appendChild(sectionHead("Tool · " + toolName, "tool_number: " + tool.tool_number));
        for (const f of TOOL_FIELDS) {
            if (!(f.key in tool)) continue;
            list.appendChild(paramRow(f, tool[f.key], "tools." + toolName + "." + f.key,
                setupSaver(["tools", toolName, f.key])));
        }
    }
}

// ---------------------------------------------------------------- trajectories

const AXES = ["x", "y", "z", "rx", "ry", "rz"];

function trajectoryRow(key, index, point) {
    const row = document.createElement("div");
    row.className = "trow";

    const head = document.createElement("div");
    head.className = "tkey";
    head.innerHTML = '<span class="pname">' + escapeHtml(key) + "[" + index + "]</span>" +
        '<span class="pmeta">tool ' + escapeHtml(point.tool_number ?? "?") +
        (point.joints_deg ? " · joints ✓" : " · no joints") +
        (point.taught_at ? " · " + escapeHtml(point.taught_at) : "") + "</span>";

    const vec = document.createElement("div");
    vec.className = "tvec";
    const inputs = {};
    for (const axis of AXES) {
        const ax = document.createElement("label");
        ax.className = "ax";
        const cap = document.createElement("span");
        cap.innerText = axis;
        const input = document.createElement("input");
        input.type = "text";
        input.spellcheck = false;
        input.value = fmtNum(point[axis] ?? 0);
        input.addEventListener("input", () => { row.classList.add("dirty"); });
        inputs[axis] = input;
        ax.append(cap, input);
        vec.appendChild(ax);
    }

    const stat = document.createElement("div");
    stat.className = "pstat";

    const btns = document.createElement("div");
    btns.className = "btns";

    const capture = document.createElement("button");
    capture.type = "button";
    capture.className = "btn ghost sm";
    capture.innerText = "Capture";
    capture.title = "Overwrite this point with the robot's CURRENT pose + joints (saved to disk)";
    capture.addEventListener("click", async () => {
        capture.disabled = true;
        setRowStatus(stat, null, "capturing…");
        try {
            const res = await fetch("/api/trajectories/capture", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ key, index }),
            });
            const data = await res.json();
            setRowStatus(stat, data.success, data.success ? "captured & saved" : data.message);
            if (data.success && Array.isArray(data.pose)) {
                AXES.forEach((axis, i) => { inputs[axis].value = fmtNum(data.pose[i]); });
                row.classList.remove("dirty");
                loadTrajectories(); // refresh tool / joints / taught_at metadata
            }
        } catch (e) {
            setRowStatus(stat, false, String(e));
        } finally {
            capture.disabled = false;
        }
    });

    const save = document.createElement("button");
    save.type = "button";
    save.className = "btn sm";
    save.innerText = "Save";
    save.title = "Save the edited pose to disk (joints are cleared — it was typed, not taught)";
    save.addEventListener("click", async () => {
        const body = { key, index, tool_number: point.tool_number ?? -1 };
        for (const axis of AXES) {
            const n = Number(inputs[axis].value);
            if (!Number.isFinite(n)) { setRowStatus(stat, false, axis + " is not a number"); return; }
            body[axis] = n;
        }
        save.disabled = true;
        setRowStatus(stat, null, "saving…");
        try {
            const res = await fetch("/api/trajectories/save", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(body),
            });
            const data = await res.json();
            setRowStatus(stat, data.success, data.success ? "saved to disk" : data.message);
            if (data.success) {
                row.classList.remove("dirty");
                loadTrajectories(); // refresh metadata (joints cleared, taught_at)
            }
        } catch (e) {
            setRowStatus(stat, false, String(e));
        } finally {
            save.disabled = false;
        }
    });

    btns.append(capture, save);
    row.append(head, vec, btns, stat);
    return row;
}

async function loadTrajectories() {
    const list = byId("traj_list");
    let data;
    try {
        data = await (await fetch("/api/trajectories")).json();
    } catch (e) {
        list.innerHTML = '<p class="muted">failed to load /api/trajectories: ' + escapeHtml(e) + "</p>";
        return;
    }
    setText("traj_meta", data.exists
        ? data.path + (data.updated_at ? " · updated " + data.updated_at : "")
        : "trajectories.json not found — is trajectories_file set on web_node?");
    const keys = Object.keys(data.points || {});
    if (!keys.length) { list.innerHTML = '<p class="muted">no trajectory points</p>'; return; }
    list.innerHTML = "";
    for (const key of keys) {
        (data.points[key] || []).forEach((point, index) => {
            list.appendChild(trajectoryRow(key, index, point));
        });
    }
}

// ---------------------------------------------------------------- boot

window.toggleTheme = toggleTheme;

connectWebSocket();
themeButtonLabel();
loadParams();
loadSetup();
loadTrajectories();
