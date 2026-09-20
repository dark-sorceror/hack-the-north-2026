"""Perception: turning what a camera sees into the metres and radians every other layer speaks.

Nothing in here decides anything. It answers one question — where is that, relative to the
base — so navigation and the skills can go on reasoning in the frame they already use.
Stdlib only, so the geometry imports on the Pi with nothing installed; anything that needs
a numerical library keeps that import inside the function that uses it.
"""
