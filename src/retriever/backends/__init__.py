"""Robot backends: the one seam between the software and a body.

Everything above this package talks to a `RobotBackend` and cannot tell whether the
robot is real, simulated, or reached through the Pi bridge. That is what lets every skill
be exercised on a laptop with no hardware before it ever moves a motor.
"""
