"""Keep the automated suite out of this directory.

The scripts here are run by hand on the board - they enumerate real USB, audio
and RealSense devices, and `test_hardware_detection.py` shells out to `lsusb`
at import time. Collecting them on a laptop is an error, not a failure, so
pytest is told to skip the whole directory. `unittest discover` already skips
it, because there is deliberately no `__init__.py` here.
"""

collect_ignore_glob = ["*.py"]
