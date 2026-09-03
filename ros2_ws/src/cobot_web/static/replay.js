// replay.js — playback driver for standalone session exports.
//
// NEVER loaded by the live page. session_recorder.py concatenates this file
// between commands.js and app.js inside the exported single-file HTML and
// embeds the recording as window.__COBOT_REPLAY__; app.js's bottom guard then
// calls window.__cobotStartReplay(recording) instead of opening a websocket.
//
// It reaches back into app.js only through three window hooks:
//   __cobotDispatch(msg)  — the live switch(data.type) dispatcher
//   __cobotResetUI()      — re-initialize all accumulated UI state
//   __cobotNow            — installed HERE: virtual clock so ages/stall pills
//                           follow the recording's timeline and freeze on pause
//
// Seek (both directions) = reset + instantly re-dispatch the recorded prefix.
// Every handler is either latest-wins or accumulate-in-order, so replaying the
// ordered prefix into a clean UI reproduces the exact live state; camera
// frames are latest-wins per stream, so only the last rgb/depth are decoded.

(function () {
    "use strict";

    const SPEEDS = [0.5, 1, 2, 4];

    let records = [];
    let duration = 0;
    let cursor = 0;        // next record to dispatch
    let virtualT = 0;      // seconds on the recording's timeline
    let playing = false;
    let speedIdx = 1;      // 1×
    let epochMs = 0;
    let lastWallMs = null;

    function el(id) { return document.getElementById(id); }

    function fmt(t) {
        t = Math.max(0, Math.round(t));
        return Math.floor(t / 60) + ":" + String(t % 60).padStart(2, "0");
    }

    function dispatch(msg) {
        try {
            window.__cobotDispatch(msg);
        } catch (e) {
            console.log("replay: dispatch failed", e, msg && msg.type);
        }
    }

    function updateBar() {
        const scrub = el("rp_scrub");
        if (scrub && document.activeElement !== scrub) scrub.value = String(virtualT);
        const time = el("rp_time");
        if (time) time.textContent = fmt(virtualT) + " / " + fmt(duration);
        const play = el("rp_play");
        if (play) play.textContent = playing ? "⏸ pause" : "▶ play";
        const speed = el("rp_speed");
        if (speed) speed.textContent = SPEEDS[speedIdx] + "×";
    }

    function seek(target) {
        target = Math.max(0, Math.min(target, duration));
        window.__cobotResetUI();
        virtualT = target;
        cursor = 0;
        let lastRgb = null;
        let lastDepth = null;
        while (cursor < records.length && records[cursor].t <= target) {
            const m = records[cursor].msg;
            if (m.type === "camera_frame") {
                // latest-wins per stream; a payload may carry only one of them
                if (m.rgb) lastRgb = m;
                if (m.depth) lastDepth = m;
            } else {
                dispatch(m);
            }
            cursor++;
        }
        // after the JSON state so overlays draw onto the restored frame
        if (lastDepth && lastDepth !== lastRgb) dispatch(lastDepth);
        if (lastRgb) dispatch(lastRgb);
        updateBar();
    }

    function tick(nowWallMs) {
        requestAnimationFrame(tick);
        if (!playing) {
            lastWallMs = nowWallMs;
            return;
        }
        if (lastWallMs === null) lastWallMs = nowWallMs;
        const dt = (nowWallMs - lastWallMs) / 1000;
        lastWallMs = nowWallMs;
        virtualT = Math.min(virtualT + dt * SPEEDS[speedIdx], duration);
        while (cursor < records.length && records[cursor].t <= virtualT) {
            dispatch(records[cursor].msg);
            cursor++;
        }
        if (virtualT >= duration) playing = false;
        updateBar();
    }

    function togglePlay() {
        if (!playing && virtualT >= duration) seek(0);  // replay from the top
        playing = !playing;
        updateBar();
    }

    function cycleSpeed() {
        speedIdx = (speedIdx + 1) % SPEEDS.length;
        updateBar();
    }

    function buildBar(meta) {
        const style = document.createElement("style");
        style.textContent = [
            "#replay_bar { position: fixed; left: 0; right: 0; bottom: 0; z-index: 1000;",
            "  display: flex; align-items: center; gap: 10px; padding: 8px 14px;",
            "  background: var(--surface); border-top: 1px solid var(--border);",
            "  color: var(--ink); font-family: var(--mono); font-size: 12px; }",
            "#replay_bar .rp-banner { color: var(--primary-hi); font-weight: 600;",
            "  letter-spacing: .04em; white-space: nowrap; }",
            "#replay_bar .rp-meta { color: var(--muted); white-space: nowrap; }",
            "#rp_scrub { flex: 1; min-width: 120px; accent-color: var(--primary); }",
            "#rp_time { color: var(--muted); white-space: nowrap; min-width: 8ch; text-align: right; }",
            "body { padding-bottom: 56px; }",
        ].join("\n");
        document.head.appendChild(style);

        const bar = document.createElement("div");
        bar.id = "replay_bar";
        const recorded = meta.started_at ? " · recorded " + meta.started_at : "";
        bar.innerHTML =
            '<span class="rp-banner">REPLAY</span>' +
            '<span class="rp-meta">' + recorded.replace(/</g, "&lt;") + "</span>" +
            '<button type="button" id="rp_play" class="btn ghost sm">▶ play</button>' +
            '<button type="button" id="rp_speed" class="btn ghost sm" title="playback speed">1×</button>' +
            '<input id="rp_scrub" type="range" min="0" max="' + (duration || 0.001) + '" step="0.05" value="0"' +
            '       aria-label="Replay timeline">' +
            '<span id="rp_time">0:00 / ' + fmt(duration) + "</span>";
        document.body.appendChild(bar);

        el("rp_play").addEventListener("click", togglePlay);
        el("rp_speed").addEventListener("click", cycleSpeed);
        const scrub = el("rp_scrub");
        // seek on release ("change"); while dragging only preview the label —
        // a full prefix re-dispatch per pixel of drag would jank long sessions
        scrub.addEventListener("input", function (e) {
            const time = el("rp_time");
            if (time) time.textContent = fmt(parseFloat(e.target.value)) + " / " + fmt(duration);
        });
        scrub.addEventListener("change", function (e) {
            seek(parseFloat(e.target.value));
            e.target.blur();
        });
        document.addEventListener("keydown", function (e) {
            if (e.target && /INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
            if (e.code === "Space") { e.preventDefault(); togglePlay(); }
            else if (e.code === "ArrowLeft") seek(virtualT - 5);
            else if (e.code === "ArrowRight") seek(virtualT + 5);
        });
    }

    window.__cobotStartReplay = function (recording) {
        records = (recording.records || []).slice().sort(function (a, b) { return a.t - b.t; });
        duration = records.length ? records[records.length - 1].t : 0;
        epochMs = (recording.meta && recording.meta.started_at_ms) || Date.now();

        // Virtual clock BEFORE the first dispatch: app.js nowMs() follows the
        // recording's timeline (scaled by speed, frozen while paused).
        window.__cobotNow = function () { return epochMs + virtualT * 1000; };

        // live-only topbar controls make no sense against a recording
        const recBtn = el("record_btn");
        if (recBtn) recBtn.style.display = "none";
        const recLink = el("record_link");
        if (recLink) recLink.style.display = "none";
        const ws = el("ws_status");
        if (ws) { ws.className = "pill pill-neutral"; ws.innerHTML = '<span class="dot"></span>replay'; }

        buildBar(recording.meta || {});

        // let app.js finish its bootstrap tail (feed layout, theme label)
        // before the first dispatch storm, then autoplay
        requestAnimationFrame(function () {
            seek(0);
            playing = true;
            lastWallMs = null;
            updateBar();
            requestAnimationFrame(tick);
        });
    };
})();
