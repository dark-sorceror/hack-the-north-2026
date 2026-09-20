"""The contract every robot body satisfies: observe, act, close.

It is a structural `Protocol` rather than a base class so the bridge client, the fakes and
any future simulator conform by shape alone, without importing each other. It is
`runtime_checkable` so wiring code can assert what it was handed at startup, instead of
failing on the first `act()` mid-mission.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from retriever.types import Action, Observation


@runtime_checkable
class RobotBackend(Protocol):
    """A robot body: real, fake or simulated; nothing above backends/ knows which."""

    def observe(self) -> Observation:
        """Return one synchronous snapshot of the robot."""
        ...

    def act(self, action: Action) -> None:
        """Apply one commanded state."""
        ...

    def close(self) -> None:
        """Release the body; the robot must be stopped afterwards."""
        ...
