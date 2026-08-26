"""
printhead.ui
============

Local web UI for the printhead rig, built around the two things the machine is
actually used for:

  * **Drucken** -- print an image or a test pattern, with a live preview of
    exactly what will be printed.
  * **Tests** -- run the measurement series from ``TESTS.md`` as one-click
    actions instead of hand-typed command lines.

Two panels are always on screen, whatever else you are doing:

  * the **live position**, carrying the same quantities ``--verbose`` prints
    (raw x/y/z, page u/v, row/col, yaw/roll/pitch, and covered/total during a
    pass);
  * the **print preview**, re-rendered whenever a setting that affects it
    changes.

Everything the UI runs is a real ``main.py`` subprocess (see ``runner.py``), so
the UI can never disagree with the CLI about what a command does, and the exact
command line is always shown and copyable.

Run it with::

    pip install -r requirements-ui.txt
    python -m printhead.ui            # opens http://127.0.0.1:8000
"""
