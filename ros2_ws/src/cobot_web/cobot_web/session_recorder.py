"""session_recorder — record websocket traffic, export a standalone replay.

SessionRecorder taps WebNode.broadcast(): while armed, every outbound payload
is appended to a JSONL file in results_dir (camera_frame decimated to
frame_hz; "recording" status messages excluded so a replay never re-triggers
the record UI). Recording is server-side — it keeps capturing with zero
browsers connected and survives page refreshes.

On stop (user, max-duration or max-size) an export thread assembles ONE
self-contained HTML file: index.html with style.css inlined, the recording
embedded as window.__COBOT_REPLAY__, and commands.js + replay.js + app.js
concatenated into a single module. app.js's bottom guard sees the replay
payload and starts the replay driver instead of opening a websocket, so live
and replay run the identical app.js. The file opens from file:// with zero
network requests; the raw .jsonl is kept alongside for re-export.

JSONL format: line 1 is {"meta": {...}}; every other line is
{"t": <seconds since start>, "msg": <payload>}. A truncated trailing line
(node crash) is tolerated and skipped by the export parser.

Threading: record() is called from rclpy executor threads AND the uvicorn
asyncio thread — all mutable state sits behind one lock. Export runs on a
daemon thread, never inline in a ROS callback.
"""

import json
import re
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable


def _fail_unless(condition: bool, message: str):
    if not condition:
        raise RuntimeError(f"replay export: {message}")


class SessionRecorder:
    FLUSH_INTERVAL_S = 2.0

    def __init__(
        self,
        results_dir: Path,
        share_dir_resolver: Callable[[], Path],
        frame_hz: float,
        max_seconds: float,
        max_bytes: int,
        on_status_change: Callable[[dict], None],
        logger,
    ):
        self._results_dir = Path(results_dir)
        self._share_dir_resolver = share_dir_resolver
        self._frame_interval = 1.0 / max(float(frame_hz), 0.1)
        self._max_seconds = float(max_seconds)
        self._max_bytes = int(max_bytes)
        self._on_status_change = on_status_change
        self._logger = logger

        self._lock = threading.Lock()
        self._active = False
        self._file = None
        self._jsonl_path: Path | None = None
        self._started_mono: float | None = None
        self._started_at: str | None = None
        self._bytes = 0
        self._frames = 0
        self._last_frame_mono: float | None = None
        self._last_flush_mono = 0.0
        self._last_file_url: str | None = None
        self._last_reason: str | None = None
        self._stopped_duration = 0.0

    # ------------------------------------------------------------------
    # control
    # ------------------------------------------------------------------

    def start(self, snapshot: list[dict]) -> dict:
        with self._lock:
            if self._active:
                return {"result": False, "message": "already recording"}
            stamp = datetime.now()
            name = stamp.strftime("%Y%m%d_%H%M%S") + "_session"
            self._results_dir.mkdir(parents=True, exist_ok=True)
            self._jsonl_path = self._results_dir / f"{name}.jsonl"
            try:
                self._file = open(self._jsonl_path, "w", encoding="utf-8", buffering=1024 * 1024)
            except OSError as e:
                self._file = None
                self._jsonl_path = None
                return {"result": False, "message": f"cannot open recording file: {e}"}
            self._started_mono = time.monotonic()
            self._started_at = stamp.strftime("%Y-%m-%d %H:%M:%S")
            self._bytes = 0
            self._frames = 0
            self._last_frame_mono = None
            self._last_flush_mono = self._started_mono
            self._last_file_url = None
            self._last_reason = None
            meta = {
                "meta": {
                    "version": 1,
                    "started_at": self._started_at,
                    "started_at_ms": int(stamp.timestamp() * 1000),
                    "frame_hz": round(1.0 / self._frame_interval, 3),
                    "node": "web_node",
                }
            }
            self._write_line(meta)
            self._active = True
            # t=0 snapshot: the same latched state a newly connected browser
            # gets, so a recording armed mid-session opens correctly.
            for payload in snapshot:
                self._record_locked(payload, t=0.0)
        self._logger.info(f"recording started → {self._jsonl_path}")
        return {"result": True, "message": f"recording to {self._jsonl_path.name}"}

    def stop(self, reason: str = "stopped") -> dict:
        with self._lock:
            if not self._active:
                return {"result": False, "message": "not recording"}
            self._finalize_locked(reason)
        return {"result": True, "message": "recording stopped — exporting replay…"}

    def status(self) -> dict:
        with self._lock:
            if self._active:
                duration = time.monotonic() - self._started_mono
            else:
                duration = self._stopped_duration
            return {
                "type": "recording",
                "active": self._active,
                "started_at": self._started_at if self._active else None,
                "duration_s": round(duration, 1),
                "bytes": self._bytes,
                "frames": self._frames,
                "file": self._last_file_url,
                "reason": self._last_reason,
            }

    # ------------------------------------------------------------------
    # the broadcast tap
    # ------------------------------------------------------------------

    def wants_frame(self) -> bool:
        """True when active and the camera-frame decimation gate is open.

        Lets _stream_frames encode at record_frame_hz with zero browsers
        connected instead of skipping frames entirely.
        """
        if not self._active:
            return False
        last = self._last_frame_mono
        return last is None or time.monotonic() - last >= self._frame_interval

    def record(self, payload: dict) -> None:
        if not self._active:  # cheap unlocked fast path; re-checked under the lock
            return
        if payload.get("type") == "recording":
            return  # never record our own status — a replay must not re-arm the UI
        with self._lock:
            if not self._active:
                return
            self._record_locked(payload, t=time.monotonic() - self._started_mono)

    def _record_locked(self, payload: dict, t: float) -> None:
        if payload.get("type") == "recording":
            return
        mono = time.monotonic()
        if payload.get("type") == "camera_frame":
            if (
                self._last_frame_mono is not None
                and mono - self._last_frame_mono < self._frame_interval
            ):
                return
            self._last_frame_mono = mono
            self._frames += 1
        try:
            self._write_line({"t": round(t, 3), "msg": payload})
        except OSError as e:
            self._logger.error(f"recording write failed: {e}")
            self._finalize_locked("write_error")
            return
        if mono - self._last_flush_mono >= self.FLUSH_INTERVAL_S:
            self._last_flush_mono = mono
            try:
                self._file.flush()
            except OSError:
                pass
        if t > self._max_seconds:
            self._finalize_locked("max_duration")
        elif self._bytes > self._max_bytes:
            self._finalize_locked("max_size")

    def _write_line(self, obj: dict) -> None:
        line = json.dumps(obj, ensure_ascii=True, separators=(",", ":")) + "\n"
        self._file.write(line)
        self._bytes += len(line)

    # ------------------------------------------------------------------
    # finalize + export (never inline in a ROS callback)
    # ------------------------------------------------------------------

    def _finalize_locked(self, reason: str) -> None:
        self._active = False
        self._last_reason = reason
        self._stopped_duration = (
            0.0 if self._started_mono is None else time.monotonic() - self._started_mono
        )
        try:
            self._file.close()
        except OSError:
            pass
        self._file = None
        jsonl_path = self._jsonl_path
        threading.Thread(
            target=self._export_and_notify, args=(jsonl_path, reason), daemon=True
        ).start()

    def _export_and_notify(self, jsonl_path: Path, reason: str) -> None:
        try:
            out_path = jsonl_path.with_suffix(".html")
            export_session_html(jsonl_path, self._share_dir_resolver(), out_path)
            with self._lock:
                self._last_file_url = f"/results/{out_path.name}"
            self._logger.info(f"replay exported → {out_path}")
        except Exception as e:  # noqa: BLE001
            with self._lock:
                self._last_reason = "export_failed"
            self._logger.error(f"replay export failed: {e}")
        self._on_status_change(self.status())


# ----------------------------------------------------------------------
# export: JSONL → one self-contained replay HTML
# ----------------------------------------------------------------------

_STYLE_LINK_RE = re.compile(r'<link rel="stylesheet" href="/static/style\.css\?v=\d+">')
_APP_SCRIPT_RE = re.compile(r'<script type="module" src="/static/app\.js\?v=\d+"></script>')
_IMPORT_RE = re.compile(r'^import \{[^}]*\} from "\./commands\.js[^"]*";\s*$', re.MULTILINE)


def export_session_html(jsonl_path: Path, share_dir: Path, out_path: Path) -> Path:
    """Assemble the standalone replay page. Anchored replacements assert
    exactly-one match so an asset edit breaks the export loudly, not silently."""
    index_html = (share_dir / "templates" / "index.html").read_text(encoding="utf-8")
    style_css = (share_dir / "static" / "style.css").read_text(encoding="utf-8")
    commands_js = (share_dir / "static" / "commands.js").read_text(encoding="utf-8")
    replay_js = (share_dir / "static" / "replay.js").read_text(encoding="utf-8")
    app_js = (share_dir / "static" / "app.js").read_text(encoding="utf-8")

    recording = _parse_jsonl(jsonl_path)

    # Script-safe embed: "<" only occurs inside JSON strings, where < is a
    # legal escape — defuses </script> and <!-- in one move. Base64 frames
    # contain no "<" so the multi-MB strings are untouched.
    payload_js = json.dumps(recording, ensure_ascii=True, separators=(",", ":")).replace(
        "<", "\\u003c"
    )

    # commands.js: module → plain consts for the concatenated single module.
    exports = commands_js.count("export const ")
    _fail_unless(exports == 6, f"commands.js: expected 6 'export const', found {exports}")
    commands_body = commands_js.replace("export const ", "const ")

    # app.js: drop the import — commands.js consts are concatenated above it.
    imports = _IMPORT_RE.findall(app_js)
    _fail_unless(len(imports) == 1, f"app.js: expected 1 commands.js import, found {len(imports)}")
    app_body = _IMPORT_RE.sub("// (commands.js inlined above for standalone replay)", app_js)

    # Order matters: consts → replay driver (defines window.__cobotStartReplay)
    # → app.js, whose bottom guard hands control to the driver.
    module_body = "\n".join([commands_body, replay_js, app_body])

    _fail_unless(len(_STYLE_LINK_RE.findall(index_html)) == 1, "style.css link tag not found")
    html = _STYLE_LINK_RE.sub(lambda _: "<style>\n" + style_css + "\n</style>", index_html)

    _fail_unless(len(_APP_SCRIPT_RE.findall(html)) == 1, "app.js script tag not found")
    replacement = (
        "<script>window.__COBOT_REPLAY__ = " + payload_js + ";</script>\n"
        '<script type="module">\n' + module_body + "\n</script>"
    )
    html = _APP_SCRIPT_RE.sub(lambda _: replacement, html)

    started = recording.get("meta", {}).get("started_at", "")
    html = html.replace(
        "<title>Cobot Control Panel</title>",
        f"<title>Cobot Replay {started}</title>",
        1,
    )

    tmp = out_path.with_suffix(".html.tmp")
    tmp.write_text(html, encoding="utf-8")
    tmp.replace(out_path)
    return out_path


def _parse_jsonl(jsonl_path: Path) -> dict:
    meta = {}
    records = []
    with open(jsonl_path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue  # truncated trailing line from a crash — skip
            if i == 0 and "meta" in obj:
                meta = obj["meta"]
            elif "msg" in obj:
                records.append({"t": obj.get("t", 0.0), "msg": obj["msg"]})
    return {"meta": meta, "records": records}
