"""Voice + vision front end: the robot sees, hears and speaks through one OMNI model.

    senses = Senses(OmniClient(), camera=WebcamCamera(0), mic=PushToTalkMic())
    turn = senses.turn(listen=True)
    turn.said, turn.intent   # -> hand the Intent to the planner; it decides

Importing this package needs no numpy, sounddevice or network -- those load
only when a function that needs them runs.
"""

from retriever.voice.omni import (
    MockOmniClient,
    OmniBackend,
    OmniClient,
    OmniConfig,
    OmniError,
    OmniReply,
    build_messages,
    build_request,
)
from retriever.voice.senses import (
    Intent,
    Senses,
    StaticCamera,
    Turn,
    parse_intent,
    strip_intent,
)

__all__ = [
    "Intent",
    "MockOmniClient",
    "OmniBackend",
    "OmniClient",
    "OmniConfig",
    "OmniError",
    "OmniReply",
    "Senses",
    "StaticCamera",
    "Turn",
    "build_messages",
    "build_request",
    "parse_intent",
    "strip_intent",
]
