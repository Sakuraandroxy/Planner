"""ProjectAirSim adapters.

The package deliberately has no eager imports so that pure modules
(``coordinates``, ``depth``, ``camera``) can be unit-tested without the
ProjectAirSim client installed.

The single eager name below is the HTTP model transport: the assembly builds
detectors and planners from it, and a cross-layer import has to stay on this
entry instead of reaching into ``planner.adapters.multimodal_chat``.
"""

from .multimodal_chat import MultimodalChatClient

__all__ = ["MultimodalChatClient"]
