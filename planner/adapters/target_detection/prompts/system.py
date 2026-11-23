"""Detection-only bbox and surface-pixel protocol, separate from planning."""

from planner.adapters.prompts import render_prompt

TARGET_DETECTION_SYSTEM_PROMPT = render_prompt("target_detection")
