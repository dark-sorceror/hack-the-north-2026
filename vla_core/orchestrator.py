"""Pure-Python orchestration facade retained for older launch commands."""
from robot_app.coordinator import Coordinator


class VLAOrchestrator:
    """Compose task decomposition and deterministic robot execution."""

    def __init__(self, coordinator=None):
        self.coordinator = coordinator or Coordinator()

    async def execute(self, target):
        return await self.coordinator.fetch(target)


__all__ = ["Coordinator", "VLAOrchestrator"]
