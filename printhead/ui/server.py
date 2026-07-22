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

Actions and the position stream are ``main.py`` subprocesses (see runner.py);
their stdout is broadcast to every connected browser over the WebSocket.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

from .runner import CommandProcess

STATIC_DIR = Path(__file__).resolve().parent / "static"


class RunRequest(BaseModel):
    args: List[str] = []


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
            try:
                obj = json.loads(line)
            except (json.JSONDecodeError, ValueError):
                await self.broadcast({"type": "log", "stream": "sensor", "line": line})
                return
            event = obj.get("event")
            if event == "position":
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
