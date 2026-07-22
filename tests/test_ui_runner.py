"""
UI subprocess-runner test (no hardware, no web deps).

`printhead.ui.runner` only uses the stdlib, so this runs without FastAPI. It
drives a real (dry-run, simulated) `main.py` through CommandProcess and checks
that its stdout is streamed line by line and the exit code is reported.

Run with:  python tests/test_ui_runner.py
"""

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.ui.runner import CommandProcess      # noqa: E402


def test_command_process_streams_and_exits():
    lines = []
    exit_code = {}

    async def on_line(line):
        lines.append(line)

    async def on_exit(code):
        exit_code["code"] = code

    async def run():
        proc = CommandProcess(
            ["--pattern", "solid", "--dry-run", "--simulate",
             "--pattern-length-mm", "10"],
            on_line, on_exit)
        assert proc.command_str().startswith("python main.py --pattern solid")
        await proc.start()
        # wait for it to finish (dry-run sim completes quickly)
        for _ in range(200):                 # up to ~10s
            if "code" in exit_code:
                break
            await asyncio.sleep(0.05)

    asyncio.run(run())

    assert exit_code.get("code") == 0, exit_code
    joined = "\n".join(lines)
    assert any("Rendered" in ln for ln in lines), joined
    assert any("Dry run" in ln for ln in lines), joined


if __name__ == "__main__":
    test_command_process_streams_and_exits()
    print("OK: ui runner streams output and reports exit code.")
