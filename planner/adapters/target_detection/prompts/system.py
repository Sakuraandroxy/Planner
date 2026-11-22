"""Detection-only bbox and surface-pixel protocol, separate from planning."""
from .render import load_fragment

TARGET_DETECTION_SYSTEM_PROMPT = load_fragment("system")
