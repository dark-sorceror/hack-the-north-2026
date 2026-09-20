#!/usr/bin/env python3
"""Launch retriever.voice.qnx from a source checkout."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from retriever.voice.qnx import main

if __name__ == "__main__":
    raise SystemExit(main())
