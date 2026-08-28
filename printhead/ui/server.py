"""
Web server
==========

FastAPI app behind the UI. Endpoints:

  ``GET  /``                    the UI
  ``GET  /view``                the print-view page: large live coverage
                                 canvas + position/pixel-count/percent,
                                 meant for its own tab while a print runs
  ``GET  /coverage_view.js``    canvas logic shared by / and /view
  ``WS   /ws``                  live stream: log lines, position, coverage, status
  ``GET  /api/state``           what is running right now
  ``POST /api/run``             run an action (print / test / diagnostic)
  ``POST /api/stop``            stop the running action
  ``POST /api/shutdown``        stop everything and terminate the server
  ``POST /api/preview``         render a preview PNG (dry run, no BLE)
  ``GET  /api/preview.png``     fetch it
  ``GET  /api/record.png``      fetch the last pass's coverage reconstruction
  ``POST /api/sensor/start``    start the idle position stream
  ``POST /api/sensor/stop``     stop it
  ``POST /api/calibration/...`` compute / save / load a page calibration
  ``GET  /api/tests``           the catalogue of one-click test actions

Everything runs as a real ``main.py`` subprocess (see ``runner.py``), so the UI
cannot drift away from what the CLI actually does.

The tracker is a single USB device and cannot be opened twice. The idle
position stream (``--pos --pos-json``) therefore has to yield to a print pass,
which needs the tracker itself. :meth:`Hub.run_action` handles that
automatically -- it suspends the stream, runs the action, and brings the stream
back afterwards -- so "the position is always live" holds without the operator
having to think about which process owns the sensor. During the pass the
position comes from the pass's own ``--progress-json`` events, which carry the
same fields (see ``PrintController._print_freehand_pass``).
"""

from __future__ import annotations

import asyncio
import json
import os
import signal
import tempfile
import warnings
from pathlib import Path
from typing import List, Optional, Set

import numpy as np
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
from pydantic import BaseModel

from ..calibration import (
    CalibrationAngleWarning, CalibrationQualityWarning, PageCalibration, calibrate_page,
)
from .runner import CommandProcess

STATIC_DIR = Path(__file__).resolve().parent / "static"
PREVIEW_PATH = Path(tempfile.gettempdir()) / "printhead_ui_preview.png"
RECORD_PATH = Path(tempfile.gettempdir()) / "printhead_ui_record.png"

# Events from --progress-json that describe pass progress rather than log text.
_COVERAGE_EVENTS = {"coverage_start", "coverage", "coverage_done"}


def _try_parse_json(line: str) -> Optional[dict]:
    """One stdout line as NDJSON, or ``None`` for ordinary log text. Both the
    sensor stream and a running action need to tell a structured event
    (``{"event": "position"|"coverage"|...}``) apart from a plain message."""
    try:
        return json.loads(line)
    except (json.JSONDecodeError, ValueError):
        return None


# ============================================================== test catalogue
# The measurement series from TESTS.md as one-click actions. Kept as data, not
# as buttons hard-coded in the HTML, so the UI and the documented protocol
# cannot drift apart: each entry names the test it belongs to, and `args` is
# exactly the command line TESTS.md tells the operator to type.
#
# `needs_calibration` entries get --page-calibration spliced in from the form;
# `dry` entries never touch hardware and are safe to click at any time.
TEST_ACTIONS = [
    {"id": "nozzle_test", "test": "V1", "label": "Düsentest",
     "help": "Alle 152 Düsen, dann einzeln durchlaufen. Findet tote Düsen, "
             "die sonst in Test 1 als Scharte und in Test 4 als Streifen "
             "auftauchen und dem Tracking angelastet würden.",
     "args": ["--nozzle-test"], "needs_calibration": False, "dry": False},
    {"id": "ble_benchmark", "test": "V2", "label": "BLE-Benchmark",
     "help": "Durchsatz und Round-Trip-Latenz der BLE-Strecke, dazu die "
             "abgeleitete Maximalgeschwindigkeit. Unabhängige Gegenprobe zu "
             "Test 7.",
     "args": ["--ble-benchmark"], "needs_calibration": False, "dry": False},
    {"id": "calibration_check", "test": "V4", "label": "Kalibrierung prüfen",
     "help": "Wagen flach OHNE Drehung ≥50 mm schieben, dann stoppen. Meldet "
             "Gierwinkel-Spanne und Korrelation gegen u/v.",
     "args": ["--calibration-check"], "needs_calibration": True, "dry": False},
    {"id": "pos", "test": "2a", "label": "Position aufzeichnen",
     "help": "Roher Sensorstrom (--pos --pos-json). Für die Rauschmessung den "
             "Wagen festklemmen und die Ausgabe in eine Datei umleiten.",
     "args": ["--pos", "--pos-json"], "needs_calibration": False, "dry": False},
    {"id": "precision_check", "test": "3", "label": "precision-check drucken",
     "help": "Linien parallel zur Düsenleiste mit verdoppelnden Abständen. "
             "Auswertung mit funktionen/precision_check_auswertung.py.",
     "args": ["--pattern", "precision-check", "--pattern-gap-start", "1",
              "--pattern-line-cols", "1", "--pattern-length-mm", "60"],
     "needs_calibration": True, "dry": False},
    {"id": "h_stripes", "test": "1", "label": "h-stripes (Kante längs)",
     "help": "Kante längs zur Fahrt — rein geometrisch, kein Timing. Der "
             "Bodenwert, gegen den v-stripes verglichen wird.",
     "args": ["--pattern", "h-stripes", "--pattern-length-mm", "80"],
     "needs_calibration": True, "dry": False},
    {"id": "v_stripes", "test": "1", "label": "v-stripes (Kante quer)",
     "help": "Kante quer zur Fahrt — Position und Timing. Die Differenz zu "
             "h-stripes ist der Tracking-Anteil.",
     "args": ["--pattern", "v-stripes", "--pattern-square-mm", "5",
              "--pattern-length-mm", "80"],
     "needs_calibration": True, "dry": False},
    {"id": "checkerboard", "test": "6", "label": "Schachbrett",
     "help": "Für die Rechtwinkligkeit. Winkel an mehreren Positionen messen — "
             "konstant heißt kalibrierbar, ortsabhängig heißt Feldverzerrung.",
     "args": ["--pattern", "checkerboard", "--pattern-square-mm", "10",
              "--pattern-square-height-mm", "10", "--pattern-length-mm", "120"],
     "needs_calibration": True, "dry": False},
    {"id": "solid_speed", "test": "7", "label": "Vollfläche (Geschwindigkeit)",
     "help": "Dabei absichtlich von langsam nach schnell beschleunigen. "
             "Auswertung mit funktionen/geschwindigkeit_profil.py.",
     "args": ["--pattern", "solid", "--pattern-length-mm", "200"],
     "needs_calibration": True, "dry": False},
]


# ================================================================ calibration
# Plain functions, not Hub methods: a calibration is a pure computation over
# samples the browser already buffered from the live sensor stream, so it needs
# none of Hub's process state. Factored out of the handlers so the project's
# plain-function tests can call them without an HTTP client.
def compute_calibration(col_samples, row_samples,
                        sheet_width_mm=None, sheet_height_mm=None,
                        boresight_quat=None) -> dict:
    """Fit a PageCalibration from two traced edges and report its quality."""
    try:
        col = np.asarray(col_samples, dtype=float)
        row = np.asarray(row_samples, dtype=float)
    except (TypeError, ValueError) as exc:
        return {"ok": False, "error": f"malformed samples: {exc}"}
    if col.ndim != 2 or col.shape[1] != 3 or row.ndim != 2 or row.shape[1] != 3:
        return {"ok": False, "error": "samples must be lists of [x, y, z]"}

    quat = np.asarray(boresight_quat, dtype=float) if boresight_quat else None
    # Winkel- und Güte-Warnung bleiben GETRENNT, nicht in einer Liste
    # zusammengefasst: sie bedeuten Verschiedenes (schiefe Kanten gegen zu
    # kurze/verrauschte Kanten) und führen zu verschiedenen Gegenmaßnahmen.
    warning_msg = None
    quality_warning_msg = None
    try:
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always", CalibrationAngleWarning)
            warnings.simplefilter("always", CalibrationQualityWarning)
            cal = calibrate_page(col, row,
                                 sheet_width_mm=sheet_width_mm,
                                 sheet_height_mm=sheet_height_mm,
                                 boresight_quat=quat)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    for w in caught:
        if issubclass(w.category, CalibrationAngleWarning):
            warning_msg = str(w.message)
        elif issubclass(w.category, CalibrationQualityWarning):
            quality_warning_msg = str(w.message)

    quality = {
        "col_trace_length_mm": cal.col_trace_length_mm,
        "row_trace_length_mm": cal.row_trace_length_mm,
        "col_rms_residual_mm": cal.col_rms_residual_mm,
        "row_rms_residual_mm": cal.row_rms_residual_mm,
        "col_sample_count": cal.col_sample_count,
        "row_sample_count": cal.row_sample_count,
        "normal_tilt_deg": cal.normal_tilt_deg,
    }
    return {"ok": True, "angle_error_deg": cal.angle_error_deg,
            "scale_col": cal.scale_col, "scale_row": cal.scale_row,
            "warning": warning_msg, "quality_warning": quality_warning_msg,
            "quality": quality,
            "has_boresight": cal.boresight_quat is not None,
            "calibration": cal.to_dict()}


def save_calibration(calibration: dict, path: str) -> dict:
    try:
        cal = PageCalibration.from_dict(calibration)
    except Exception as exc:
        return {"ok": False, "error": f"not a valid calibration: {exc}"}
    try:
        cal.save(path)
    except OSError as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "path": str(Path(path).resolve())}


def load_calibration(path: str) -> dict:
    """Read back a saved calibration's summary, e.g. to confirm one before
    printing with it.

    Every ``quality`` entry may be ``None``: a calibration saved before the
    fit-quality feature existed (the operator has one) carries no measured
    quality to report -- see ``PageCalibration``'s docstring."""
    try:
        cal = PageCalibration.load(path)
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    quality = {
        "col_trace_length_mm": cal.col_trace_length_mm,
        "row_trace_length_mm": cal.row_trace_length_mm,
        "col_rms_residual_mm": cal.col_rms_residual_mm,
        "row_rms_residual_mm": cal.row_rms_residual_mm,
        "col_sample_count": cal.col_sample_count,
        "row_sample_count": cal.row_sample_count,
        "normal_tilt_deg": cal.normal_tilt_deg,
    }
    return {"ok": True, "path": str(Path(path).resolve()),
            "angle_error_deg": cal.angle_error_deg,
            "scale_col": cal.scale_col, "scale_row": cal.scale_row,
            "quality": quality,
            "calibration": cal.to_dict()}


# ===================================================================== models
class RunRequest(BaseModel):
    args: List[str] = []


class CalibrationComputeRequest(BaseModel):
    col_samples: List[List[float]]
    row_samples: List[List[float]]
    sheet_width_mm: Optional[float] = None
    sheet_height_mm: Optional[float] = None
    boresight_quat: Optional[List[float]] = None


class CalibrationSaveRequest(BaseModel):
    calibration: dict
    path: str


class CalibrationLoadRequest(BaseModel):
    path: str


# ======================================================================== hub
class Hub:
    """Holds the WebSocket clients and the managed subprocesses."""

    def __init__(self) -> None:
        self.clients: Set[WebSocket] = set()
        self.action: Optional[CommandProcess] = None
        self.sensor: Optional[CommandProcess] = None
        # Args the sensor stream was last started with, so a suspend/resume
        # round trip brings back the SAME stream (same axis, same scale).
        self._sensor_args: Optional[List[str]] = None
        # Sensor args to restore after an action finishes. None means the
        # stream was not running when the action started, so nothing is
        # resumed -- an action must never silently turn the tracker on.
        self._sensor_resume: Optional[List[str]] = None
        # The most recent `coverage_start` event's payload, kept only while
        # that pass is still running (cleared on `coverage_done`). Lets a
        # client that connects or reconnects MID-PASS -- the whole point of
        # /view, opened after a print has already started -- initialise its
        # canvas immediately instead of sitting blank until the NEXT pass's
        # coverage_start, which may be minutes away or may never come if
        # this is the only pass. See register() below.
        self._last_coverage_start: Optional[dict] = None

    # -- websocket fan-out --------------------------------------------------
    async def register(self, ws: WebSocket) -> None:
        await ws.accept()
        self.clients.add(ws)
        await ws.send_json({"type": "status", **self.status()})
        if self._last_coverage_start is not None:
            # `replay` marks this apart from a live coverage_start so the
            # client can say "opened mid-pass" instead of implying it saw
            # the pass from the start -- cells inked before this connection
            # existed are NOT recoverable (the hub never buffers them, a
            # full-resolution print can be millions of pixels) and will
            # never appear on this client's canvas, only from here on.
            await ws.send_json({"type": "coverage_event",
                                **self._last_coverage_start, "replay": True})

    def unregister(self, ws: WebSocket) -> None:
        self.clients.discard(ws)

    async def broadcast(self, msg: dict) -> None:
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send_json(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    def status(self) -> dict:
        return {
            "action_running": self.action.running if self.action else False,
            "action_cmd": self.action.command_str() if self.action else None,
            "sensor_running": self.sensor.running if self.sensor else False,
        }

    async def _status_broadcast(self) -> None:
        await self.broadcast({"type": "status", **self.status()})

    async def _log(self, line: str, stream: str = "action") -> None:
        await self.broadcast({"type": "log", "stream": stream, "line": line})

    # -- actions ------------------------------------------------------------
    async def run_action(self, args: List[str]) -> dict:
        """
        Run one ``main.py`` action.

        Suspends the idle position stream first: the Amfitrack is a single USB
        device, and a pass that needs it would fail to open it while ``--pos``
        still holds it. The stream is restarted when the action exits, so from
        the operator's side the live readout simply keeps working -- during the
        pass it is fed by the pass's own --progress-json events instead.
        """
        if self.action and self.action.running:
            return {"ok": False, "error": "es läuft bereits eine Aktion"}

        # A previous action's coverage_start could otherwise leak into a
        # client that connects during THIS action, before it has (or ever
        # has, if this isn't page mode) sent its own coverage_start -- wrong
        # width/height, or claiming a pass is running when none is.
        self._last_coverage_start = None

        # Remember whether to bring the sensor back, then release the device.
        if self.sensor and self.sensor.running:
            self._sensor_resume = list(self._sensor_args or [])
            await self._suspend_sensor()
        else:
            self._sensor_resume = None

        async def on_line(line: str) -> None:
            obj = _try_parse_json(line)
            if isinstance(obj, dict) and obj.get("event") == "position":
                await self.broadcast({"type": "position", **obj})
            elif isinstance(obj, dict) and obj.get("event") in _COVERAGE_EVENTS:
                # Tracked so register() can replay it into a client that
                # connects mid-pass -- see _last_coverage_start's docstring.
                if obj["event"] == "coverage_start":
                    self._last_coverage_start = dict(obj)
                elif obj["event"] == "coverage_done":
                    self._last_coverage_start = None
                await self.broadcast({"type": "coverage_event", **obj})
            else:
                await self._log(line)

        async def on_exit(code: int) -> None:
            # Safety net for a process that dies mid-pass (crash, kill)
            # without ever emitting coverage_done: without this, a start
            # payload from a pass that no longer exists would keep getting
            # replayed into new connections after the action has already
            # ended.
            self._last_coverage_start = None
            await self.broadcast({"type": "action_done", "code": code})
            await self._status_broadcast()
            if self._sensor_resume is not None:
                resume = self._sensor_resume
                self._sensor_resume = None
                await self.start_sensor(resume)

        self.action = CommandProcess(args, on_line, on_exit)
        await self.action.start()
        await self._log(f"$ {self.action.command_str()}")
        await self._status_broadcast()
        return {"ok": True, "cmd": self.action.command_str()}

    async def stop_action(self) -> dict:
        if self.action:
            await self.action.stop()
        await self._status_broadcast()
        return {"ok": True}

    async def run_print(self, args: List[str]) -> dict:
        """A print pass, always instrumented: --progress-json feeds the live
        readout and the coverage canvas, --record reconstructs what landed."""
        try:
            RECORD_PATH.unlink()
        except FileNotFoundError:
            pass
        return await self.run_action([*args, "--progress-json",
                                      "--record", str(RECORD_PATH)])

    # -- preview ------------------------------------------------------------
    async def run_preview(self, args: List[str]) -> dict:
        """
        Render what would be printed to a PNG, without touching hardware.

        Runs its own short-lived subprocess rather than going through
        run_action, so a preview never counts as "an action is running", never
        suspends the sensor stream, and can be re-rendered while a pass is in
        progress.
        """
        try:
            PREVIEW_PATH.unlink()
        except FileNotFoundError:
            pass
        full = [*args, "--dry-run", "--preview", str(PREVIEW_PATH)]
        done = asyncio.Event()
        fehler: List[str] = []

        async def on_line(line: str) -> None:
            # Preview output is noise in the main log unless it failed, so it
            # is buffered and only surfaced when the render produced nothing.
            fehler.append(line)

        async def on_exit(code: int) -> None:
            done.set()

        proc = CommandProcess(full, on_line, on_exit)
        await proc.start()
        try:
            await asyncio.wait_for(done.wait(), timeout=30.0)
        except asyncio.TimeoutError:
            await proc.stop()
        if PREVIEW_PATH.exists():
            return {"ok": True}
        # Der letzte Ausgabezeile ist bei einem Argument-/Dateifehler die
        # eigentliche Meldung ("cannot load page calibration ..."). Sie geht
        # mit zurück, damit sie IM Vorschaufeld steht statt nur im Log --
        # dort steht sie sonst zwischen dem Rauschen der laufenden Aktion.
        for line in fehler[-6:]:
            await self._log(f"[vorschau] {line}")
        letzte = next((l for l in reversed(fehler) if l.strip()), "")
        return {"ok": False,
                "error": letzte or "Vorschau konnte nicht erzeugt werden"}

    # -- sensor stream ------------------------------------------------------
    async def start_sensor(self, extra: List[str]) -> dict:
        if self.sensor and self.sensor.running:
            return {"ok": False, "error": "Sensorstrom läuft bereits"}
        args = ["--pos", "--pos-json", *extra]
        self._sensor_args = list(extra)
        proc_box: dict = {}

        async def on_line(line: str) -> None:
            # A superseded process must not push stale samples.
            if self.sensor is not proc_box.get("p"):
                return
            obj = _try_parse_json(line)
            if not isinstance(obj, dict):
                await self._log(line, stream="sensor")
                return
            if obj.get("event") == "position":
                await self.broadcast({"type": "position", **obj})
            else:
                await self.broadcast({"type": "sensor_event", **obj})

        async def on_exit(code: int) -> None:
            if self.sensor is proc_box.get("p"):
                await self.broadcast({"type": "sensor_stopped", "code": code})
                await self._status_broadcast()

        proc = CommandProcess(args, on_line, on_exit)
        proc_box["p"] = proc
        self.sensor = proc
        await proc.start()
        await self._status_broadcast()
        return {"ok": True, "cmd": proc.command_str()}

    async def _suspend_sensor(self) -> None:
        """Stop the stream without announcing it as an operator-visible stop --
        it is coming back as soon as the action finishes."""
        old = self.sensor
        self.sensor = None            # detach so old on_exit stays quiet
        if old is not None:
            await old.stop()

    async def stop_sensor(self) -> dict:
        if self.sensor:
            await self.sensor.stop()
        self._sensor_resume = None    # an explicit stop is not resumed later
        await self._status_broadcast()
        return {"ok": True}

    async def restart_sensor(self, extra: List[str]) -> dict:
        await self._suspend_sensor()
        return await self.start_sensor(extra)


hub = Hub()
app = FastAPI(title="Printhead")


# ==================================================================== routes
@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return (STATIC_DIR / "index.html").read_text(encoding="utf-8")


@app.get("/view", response_class=HTMLResponse)
async def view() -> str:
    """The print-view page: a large live coverage canvas plus position and
    pixel-count/percent readouts, meant to be opened in its own tab/window
    while a print runs (see index.html's "Druckansicht" link) -- unlike the
    main page it carries no print controls, only the live monitor."""
    return (STATIC_DIR / "view.html").read_text(encoding="utf-8")


@app.get("/coverage_view.js")
async def coverage_view_js():
    """The canvas logic shared by index.html and view.html (see that file's
    own doc comment for why it is factored out). A dedicated route, not a
    generic StaticFiles mount, matching how every other static asset here
    (index.html, view.html) is served -- this app has exactly three static
    files and no build step; a mount would be infrastructure for a fourth
    that will not arrive."""
    return FileResponse(str(STATIC_DIR / "coverage_view.js"),
                        media_type="application/javascript")


@app.get("/api/state")
async def state() -> dict:
    return hub.status()


@app.get("/api/tests")
async def tests() -> dict:
    return {"tests": TEST_ACTIONS}


@app.post("/api/run")
async def run(req: RunRequest) -> dict:
    return await hub.run_action(req.args)


@app.post("/api/print")
async def run_print(req: RunRequest) -> dict:
    return await hub.run_print(req.args)


@app.post("/api/stop")
async def stop() -> dict:
    return await hub.stop_action()


@app.post("/api/shutdown")
async def shutdown() -> dict:
    """Stop the action + sensor stream (clean SIGINT so subprocesses blank the
    printhead and close the tracker), then terminate this server.

    Termination is deferred until just after this coroutine returns, so the
    HTTP response reaches the browser before the process goes away. uvicorn
    installs its own SIGTERM handler and shuts down gracefully, so this
    signals rather than calling os._exit."""
    hub._sensor_resume = None         # nothing to resume, we are going away
    await hub.stop_action()
    await hub.stop_sensor()

    async def _terminate() -> None:
        await asyncio.sleep(0.3)
        os.kill(os.getpid(), signal.SIGTERM)

    asyncio.create_task(_terminate())
    return {"ok": True}


@app.post("/api/preview")
async def preview(req: RunRequest) -> dict:
    return await hub.run_preview(req.args)


@app.get("/api/preview.png")
async def preview_png():
    if not PREVIEW_PATH.exists():
        raise HTTPException(status_code=404, detail="noch keine Vorschau")
    return FileResponse(str(PREVIEW_PATH), media_type="image/png",
                        headers={"Cache-Control": "no-store"})


@app.get("/api/record.png")
async def record_png():
    if not RECORD_PATH.exists():
        raise HTTPException(status_code=404, detail="noch keine Aufzeichnung")
    return FileResponse(str(RECORD_PATH), media_type="image/png",
                        headers={"Cache-Control": "no-store"})


@app.post("/api/calibration/compute")
async def calibration_compute_endpoint(req: CalibrationComputeRequest) -> dict:
    return compute_calibration(req.col_samples, req.row_samples,
                               req.sheet_width_mm, req.sheet_height_mm,
                               req.boresight_quat)


@app.post("/api/calibration/save")
async def calibration_save_endpoint(req: CalibrationSaveRequest) -> dict:
    return save_calibration(req.calibration, req.path)


@app.post("/api/calibration/load")
async def calibration_load_endpoint(req: CalibrationLoadRequest) -> dict:
    return load_calibration(req.path)


@app.post("/api/sensor/start")
async def sensor_start(req: RunRequest) -> dict:
    return await hub.start_sensor(req.args)


@app.post("/api/sensor/stop")
async def sensor_stop() -> dict:
    return await hub.stop_sensor()


@app.post("/api/sensor/restart")
async def sensor_restart(req: RunRequest) -> dict:
    return await hub.restart_sensor(req.args)


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await hub.register(ws)
    try:
        while True:
            await ws.receive_text()          # keep open; input is ignored
    except WebSocketDisconnect:
        hub.unregister(ws)
    except Exception:
        hub.unregister(ws)
