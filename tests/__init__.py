"""Runs with nothing installed: python3 -m unittest discover -s tests -t ."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
