#!/usr/bin/env python3
"""
Entry point for the HP302 printhead controller.

Equivalent to ``python -m printhead``. See ``python main.py --help`` and the
README for usage.
"""

from printhead.cli import main

if __name__ == "__main__":
    main()
