"""
Web server
==========

FastAPI app that serves the single-page UI and exposes:
  * ``GET  /``                 -> the UI
  * ``WS   /ws``               -> live stream (log lines, position samples, status)
  * ``POST /api/run``          -> run an action command (print/calibrate/pattern/...)
  * ``POST /api/stop``         -> stop the running action
  * ``POST /api/sensor/start`` -> start the continuous position stream
  * ``POST /api/sensor/stop``  -> stop the position stream
  * ``GET  /api/state``        -> current running state
  * ``POST /api/shutdown``     -> stop everything and terminate this server process

Actions and the position stream are ``main.py`` subprocesses (see runner.py);
their stdout is broadcast to every connected browser over the WebSocket.
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

from ..calibration import CalibrationAngleWarning, PageCalibration, calibrate_page
from .runner import CommandProcess

STATIC_DIR = Path(__file__).resolve().parent / "static"
PREVIEW_PATH = Path(tempfile.gettempdir()) / "printhead_ui_preview.png"
RECORD_PATH = Path(tempfile.gettempdir()) / "printhead_ui_record.png"


def _try_parse_json(line: str) -> Optional[dict]:
    """Parse one subprocess stdout line as NDJSON, or return ``None`` if it
    isn't valid JSON (a plain log line). Shared by the sensor stream and the
    action-run handlers below, which both need to tell a structured progress
    event apart from ordinary log text: ``{"event": "position", ...}`` from
    ``--pos-json``, or ``{"event": "coverage"|"coverage_start"|"coverage_done",
    ...}`` from a page-mode pass run with ``--progress-json`` (see
    ``PrintController._print_freehand_pass``)."""
    try:
        return json.loads(line)
    except (json.JSONDecodeError, ValueError):
        return None


_COVERAGE_EVENTS = {"coverage_start", "coverage", "coverage_done"}


# ================================================================ calibration
# Plain functions rather than methods on Hub: calibration is a pure
# computation over samples the browser already buffered from the live
# --pos-json sensor stream (see the Calibration tab), not a managed
# subprocess, so it needs none of Hub's process/broadcast state. Factored out
# from the @app.post handlers below so they're testable without spinning up
# FastAPI or a test HTTP client -- this project's tests are all plain
# functions, no pytest/httpx.
def compute_calibration(col_samples, row_samples,
                        sheet_width_mm: Optional[float] = None,
                        sheet_height_mm: Optional[float] = None,
                        boresight_quat=None) -> dict:
    """Business logic behind ``POST /api/calibration/compute``.

    ``boresight_quat``, if given (the browser's most recently captured
    ``(qx, qy, qz, qw)`` from the live ``--pos-json`` sensor stream -- see
    the Calibration tab's "Capture boresight" button), is threaded straight
    into ``calibrate_page()`` and stored on the resulting calibration.
    ``has_boresight`` in the return value mirrors that back so the UI can
    confirm what actually got saved, not just what it thinks it sent."""
    try:
        col = np.asarray(col_samples, dtype=float)
        row = np.asarray(row_samples, dtype=float)
        bore = np.asarray(boresight_quat, dtype=float) if boresight_quat is not None else None
    except (TypeError, ValueError) as exc:
        return {"ok": False, "error": f"invalid samples: {exc}"}

    warning_msg = None
    try:
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always", CalibrationAngleWarning)
            cal = calibrate_page(col, row, sheet_width_mm=sheet_width_mm,
                                 sheet_height_mm=sheet_height_mm,
                                 boresight_quat=bore)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    for w in caught:
        if issubclass(w.category, CalibrationAngleWarning):
            warning_msg = str(w.message)

    return {"ok": True, "angle_error_deg": cal.angle_error_deg,
            "scale_col": cal.scale_col, "scale_row": cal.scale_row,
            "warning": warning_msg, "has_boresight": cal.boresight_quat is not None,
            "calibration": cal.to_dict()}


def save_calibration(calibration: dict, path: str) -> dict:
    """Business logic behind ``POST /api/calibration/save``. ``calibration``
    is the ``calibration`` dict a prior ``compute_calibration()`` call
    returned -- the browser round-trips it rather than the server holding
    calibration state between requests."""
    try:
        PageCalibration.from_dict(calibration).save(path)
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "path": path}


def load_calibration(path: str) -> dict:
    """Business logic behind ``POST /api/calibration/load``: read back a
    previously saved calibration's summary, e.g. to confirm one before
    printing with it."""
    try:
        cal = PageCalibration.load(path)
    except Exception as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "angle_error_deg": cal.angle_error_deg,
            "scale_col": cal.scale_col, "scale_row": cal.scale_row,
            "calibration": cal.to_dict()}


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


class Hub:
    """Holds the WebSocket clients and the two managed subprocesses."""

    def __init__(self) -> None:
        self.clients: Set[WebSocket] = set()
        self.action: Optional[CommandProcess] = None
        self.sensor: Optional[CommandProcess] = None

    # -- websocket fan-out --------------------------------------------------
    async def register(self, ws: WebSocket) -> None:
        await ws.accept()
        self.clients.add(ws)
        await ws.send_json({"type": "status", **self.status()})

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

    # -- actions ------------------------------------------------------------
    async def run_action(self, args: List[str]) -> dict:
        if self.action and self.action.running:
            return {"ok": False, "error": "an action is already running"}

        async def on_line(line: str) -> None:
            obj = _try_parse_json(line)
            if isinstance(obj, dict) and obj.get("event") == "position":
                await self.broadcast({"type": "position", **obj})
            elif isinstance(obj, dict) and obj.get("event") in _COVERAGE_EVENTS:
                await self.broadcast({"type": "coverage_event", **obj})
            else:
                await self.broadcast({"type": "log", "stream": "action", "line": line})

        async def on_exit(code: int) -> None:
            await self.broadcast({"type": "action_done", "code": code})
            await self._status_broadcast()

        self.action = CommandProcess(args, on_line, on_exit)
        await self.action.start()
        await self.broadcast({"type": "log", "stream": "action",
                              "line": f"$ {self.action.command_str()}"})
        await self._status_broadcast()
        return {"ok": True, "cmd": self.action.command_str()}

    async def stop_action(self) -> dict:
        if self.action:
            await self.action.stop()
        await self._status_broadcast()
        return {"ok": True}

    async def run_record(self, args: List[str]) -> dict:
        """Run a print pass that also reconstructs what was sent (--record)."""
        try:
            RECORD_PATH.unlink()
        except FileNotFoundError:
            pass
        return await self.run_action([*args, "--record", str(RECORD_PATH)])

    # -- preview ------------------------------------------------------------
    async def run_preview(self, args: List[str]) -> dict:
        """Render the image to a PNG (dry-run, no BLE) and report when it's ready.
        The generated file is served by GET /api/preview.png."""
        try:
            PREVIEW_PATH.unlink()
        except FileNotFoundError:
            pass
        full = [*args, "--dry-run", "--preview", str(PREVIEW_PATH)]
        done = asyncio.Event()

        async def on_line(line: str) -> None:
            await self.broadcast({"type": "log", "stream": "action", "line": line})

        async def on_exit(code: int) -> None:
            done.set()

        proc = CommandProcess(full, on_line, on_exit)
        await self.broadcast({"type": "log", "stream": "action",
                              "line": f"$ {proc.command_str()}"})
        await proc.start()
        try:
            await asyncio.wait_for(done.wait(), timeout=30.0)
        except asyncio.TimeoutError:
            await proc.stop()
        return {"ok": PREVIEW_PATH.exists()}

    # -- sensor stream ------------------------------------------------------
    async def start_sensor(self, extra: List[str]) -> dict:
        if self.sensor and self.sensor.running:
            return {"ok": False, "error": "sensor stream already running"}
        args = ["--pos", "--pos-json", *extra]
        proc_box: dict = {}

        async def on_line(line: str) -> None:
            # A superseded (refreshed) process must not push stale samples.
            if self.sensor is not proc_box.get("p"):
                return
            obj = _try_parse_json(line)
            if not isinstance(obj, dict):
                await self.broadcast({"type": "log", "stream": "sensor", "line": line})
                return
            if obj.get("event") == "position":
                await self.broadcast({"type": "position", **obj})
            else:
                await self.broadcast({"type": "sensor_event", **obj})

        async def on_exit(code: int) -> None:
            # Only report "stopped" if this is still the active stream, so a
            # refresh (stop old + start new) does not clobber the new stream.
            if self.sensor is proc_box.get("p"):
                await self.broadcast({"type": "sensor_stopped", "code": code})
                await self._status_broadcast()

        proc = CommandProcess(args, on_line, on_exit)
        proc_box["p"] = proc
        self.sensor = proc
        await proc.start()
        await self._status_broadcast()
        return {"ok": True, "cmd": proc.command_str()}

    async def stop_sensor(self) -> dict:
        if self.sensor:
            await self.sensor.stop()
        await self._status_broadcast()
        return {"ok": True}

    async def restart_sensor(self, extra: List[str]) -> dict:
        """Stop the current stream (if any) and start a fresh one with new args,
        so a changed advance axis / scale takes effect immediately."""
        old = self.sensor
        self.sensor = None                 # detach so the old on_exit stays quiet
        if old is not None:
            await old.stop()
        return await self.start_sensor(extra)


hub = Hub()
app = FastAPI(title="Printhead Control")


@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return (STATIC_DIR / "index.html").read_text(encoding="utf-8")


@app.get("/api/state")
async def state() -> dict:
    return hub.status()


@app.post("/api/run")
async def run(req: RunRequest) -> dict:
    return await hub.run_action(req.args)


@app.post("/api/stop")
async def stop() -> dict:
    return await hub.stop_action()


@app.post("/api/shutdown")
async def shutdown() -> dict:
    """Stop the running action + sensor stream (clean SIGINT so subprocesses
    blank the printhead / close the tracker), then terminate this server
    process itself -- unlike /api/stop, which only cancels the current action.

    The actual termination is deferred to just after this coroutine returns,
    so the HTTP response reaches the browser before the process (and its
    connections) go away. uvicorn installs its own SIGTERM handler and shuts
    down gracefully, so we signal rather than os._exit."""
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
        raise HTTPException(status_code=404, detail="no preview generated yet")
    return FileResponse(str(PREVIEW_PATH), media_type="image/png",
                        headers={"Cache-Control": "no-store"})


@app.post("/api/record")
async def record(req: RunRequest) -> dict:
    return await hub.run_record(req.args)


@app.get("/api/record.png")
async def record_png():
    if not RECORD_PATH.exists():
        raise HTTPException(status_code=404, detail="no recording yet")
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
            await ws.receive_text()          # keep the socket open; ignore input
    except WebSocketDisconnect:
        hub.unregister(ws)
    except Exception:
        hub.unregister(ws)
