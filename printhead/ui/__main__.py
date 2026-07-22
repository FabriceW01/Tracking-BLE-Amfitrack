"""
Launch the web UI:  python -m printhead.ui  [--host H] [--port P] [--no-browser]
"""

from __future__ import annotations

import argparse
import threading
import webbrowser


def main() -> None:
    ap = argparse.ArgumentParser(prog="printhead.ui",
                                 description="Local web UI for the printhead CLI.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true",
                    help="Do not open a browser window automatically")
    args = ap.parse_args()

    try:
        import uvicorn
    except ImportError:
        raise SystemExit("The UI needs extra packages: "
                         "pip install -r requirements-ui.txt")

    url = f"http://{args.host}:{args.port}"
    print(f"Printhead Control UI -> {url}")
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    uvicorn.run("printhead.ui.server:app", host=args.host, port=args.port,
                log_level="warning")


if __name__ == "__main__":
    main()
