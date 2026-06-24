from .config import PlannerConfig
from .trajectory import AtomicAction, Delta, Trajectory, compute_delta, add_noise
from web.shared_state import SharedState
from web.app import create_app

# Lazy imports for agent (avoids circular: prompt_builder -> planner -> agent -> planner)
import importlib

def __getattr__(name):
    lazy = {
        "VLMPlanner": ("agent.planner", "Planner"),
        "ConversationHistory": ("agent.context_manager", "ContextManager"),
        "PromptBuilder": ("agent.prompt_builder", "PromptBuilder"),
        "ResponseParser": ("agent.response_parser", "ResponseParser"),
        "VLMClient": ("agent.vlm_client", "VLMClient"),
    }
    if name in lazy:
        mod_path, cls_name = lazy[name]
        mod = importlib.import_module(mod_path)
        result = getattr(mod, cls_name)
        globals()[name] = result
        return result
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "PlannerConfig", "VLMPlanner", "ConversationHistory",
    "SharedState", "create_app",
    "AtomicAction", "Delta", "Trajectory",
    "compute_delta", "add_noise",
    "PromptBuilder", "ResponseParser", "VLMClient",
]
