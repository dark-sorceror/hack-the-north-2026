"""The planner: a language model choosing which skill runs next.

It decides WHAT, never HOW. Control stays in deterministic skills with
timeouts; the model only routes between them.
"""

from retriever.planner.loop import PlanEvent, Planner, PlanResult, format_event
from retriever.planner.niches import FETCH, NICHES, SORT, NicheConfig, get_niche
from retriever.planner.providers import (
    AnthropicProvider,
    OpenAICompatibleProvider,
    PlannerTurn,
    Provider,
    ScriptedProvider,
    ToolCall,
)

__all__ = [
    "AnthropicProvider",
    "FETCH",
    "NICHES",
    "NicheConfig",
    "OpenAICompatibleProvider",
    "PlanEvent",
    "PlanResult",
    "Planner",
    "PlannerTurn",
    "Provider",
    "SORT",
    "ScriptedProvider",
    "ToolCall",
    "format_event",
    "get_niche",
]
