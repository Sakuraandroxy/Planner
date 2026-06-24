"""Conversation history management for multi-step VLM reasoning."""
from typing import List, Dict


class ContextManager:
    """Manages VLM conversation history, auto-trimming to prevent overflow."""

    def __init__(self, max_steps: int = 5):
        self.messages: List[Dict] = []
        self.max_steps = max_steps

    def add_step(self, step_num: int, actions: List[str],
                 old_pos, new_pos, yaw_deg: float, collided: bool):
        """Record one step result as a user/assistant message pair."""
        summary = (
            f"【Step {step_num}】动作: {actions}. "
            f"位置 ({old_pos[0]:.2f}, {old_pos[1]:.2f}, {old_pos[2]:.2f}) -> "
            f"({new_pos[0]:.2f}, {new_pos[1]:.2f}, {new_pos[2]:.2f}), "
            f"偏航 {yaw_deg:.1f}deg."
        )
        if collided:
            summary += " 发生碰撞！"

        self.messages.append({"role": "user", "content": summary})
        self.messages.append({
            "role": "assistant",
            "content": f"已执行{actions}，继续下一步。"
        })

        max_pairs = self.max_steps * 2
        if len(self.messages) > max_pairs:
            self.messages = self.messages[-max_pairs:]

    def get_messages(self) -> List[Dict]:
        return self.messages

    def clear(self):
        self.messages = []
