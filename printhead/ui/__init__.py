"""
printhead.ui
============

A local web UI that drives the ``printhead`` CLI: it builds ``main.py`` commands
from forms, runs them, streams their output live, and shows a continuous readout
of the Amfitrack sensor position while connected.

Run it with::

    pip install -r requirements-ui.txt
    python -m printhead.ui            # opens http://127.0.0.1:8000 in the browser
"""
