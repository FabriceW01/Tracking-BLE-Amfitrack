"""
Subprocess runner
=================

Runs ``python -u main.py <args>`` as a child process and streams its stdout
lines to an async callback. Used both for one-shot actions (print / calibrate /
benchmark / ...) and for the long-lived position stream (``--pos --pos-json``).

Running the real CLI (rather than re-implementing it in-process) means the UI
automatically covers every command the CLI supports, and a failing connection
just shows up in the log instead of taking down the server.
"""

from __future__ import annotations

import asyncio
import os
import signal
import sys
from pathlib import Path
from typing import Awaitable, Callable, List, Optional

# repo root = the directory that contains main.py (two levels up from this file)
REPO_ROOT = Path(__file__).resolve().parents[2]

LineCallback = Callable[[str], Awaitable[None]]
ExitCallback = Callable[[int], Awaitable[None]]


class CommandProcess:
    """A single managed ``main.py`` subprocess."""

    def __init__(self, args: List[str], on_line: LineCallback,
                 on_exit: Optional[ExitCallback] = None):
        self.args = [str(a) for a in args]
        self._on_line = on_line
        self._on_exit = on_exit
        self._proc: Optional[asyncio.subprocess.Process] = None
        self._pump: Optional[asyncio.Task] = None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.returncode is None

    def command_str(self) -> str:
        return "python main.py " + " ".join(self.args)

    async def start(self) -> None:
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        self._proc = await asyncio.create_subprocess_exec(
            sys.executable, "-u", "main.py", *self.args,
            cwd=str(REPO_ROOT), env=env,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        self._pump = asyncio.create_task(self._read_output())

    async def _read_output(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        try:
            async for raw in self._proc.stdout:
                line = raw.decode("utf-8", "replace").rstrip("\r\n")
                if line:
                    await self._on_line(line)
        finally:
            code = await self._proc.wait()
            if self._on_exit is not None:
                await self._on_exit(code)

    async def stop(self) -> None:
        """Ask the child to stop (SIGINT so --pos / passes exit cleanly)."""
        if self._proc is None or self._proc.returncode is not None:
            return
        try:
            self._proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            await asyncio.wait_for(self._proc.wait(), timeout=3.0)
        except asyncio.TimeoutError:
            try:
                self._proc.kill()
            except ProcessLookupError:
                pass
