"""Pure-Python safety facade retained for the pre-ROS module path."""
from robot_app.safety import QuickDecisionMaker, run_with_watchdog

__all__ = ["QuickDecisionMaker", "run_with_watchdog"]
