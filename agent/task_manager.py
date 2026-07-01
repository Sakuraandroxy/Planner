"""Multi-stage task management for closed-loop AirSim navigation.

This module is intentionally independent from bbox tracking.  It only decides
what the current sub-task is and whether that sub-task needs target detection.
Concrete target identity, depth, and relocalization stay in planner/context
modules.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class TaskStage:
    """One executable stage parsed from a user instruction."""

    index: int
    instruction: str
    mode: str = "target"  # target | detect | action
    target_query: str = ""
    actions: List[str] = field(default_factory=list)
    completed: bool = False
    completion_reason: str = ""
    completion_condition: str = ""

    @property
    def requires_target(self) -> bool:
        return self.mode in ("target", "detect")

    @property
    def allow_relocalize(self) -> bool:
        return self.mode in ("target", "detect")

    @property
    def is_direct_action(self) -> bool:
        return self.mode == "action" and bool(self.actions)


class TaskManager:
    """Parse a long instruction into sequential navigation stages."""

    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.root_instruction: str = ""
        self.stages: List[TaskStage] = []
        self.current_index: int = 0

    def start(self, instruction: str):
        self.root_instruction = (instruction or "").strip()
        self.stages = []
        self.current_index = 0

    def start_with_stages(self, instruction: str, parsed_stages: list):
        """Start a task from VLM-parsed stage dictionaries."""
        self.root_instruction = (instruction or "").strip()
        stages = self._stages_from_dicts(parsed_stages)
        if not stages:
            raise ValueError("no valid parsed task stages")
        self.stages = stages
        self.current_index = 0

    def reset(self):
        self.root_instruction = ""
        self.stages = []
        self.current_index = 0

    def current_stage(self) -> Optional[TaskStage]:
        if 0 <= self.current_index < len(self.stages):
            return self.stages[self.current_index]
        return None

    def is_done(self) -> bool:
        return bool(self.stages) and self.current_index >= len(self.stages)

    def complete_current(self, reason: str = "") -> Optional[TaskStage]:
        stage = self.current_stage()
        if stage is None:
            return None
        stage.completed = True
        stage.completion_reason = reason or "completed"
        self.current_index += 1
        return stage

    def current_prompt(self) -> str:
        stage = self.current_stage()
        if stage is None:
            return self.root_instruction
        completed = [
            f"{item.index + 1}. {item.instruction}（{item.completion_reason or '已完成'}）"
            for item in self.stages
            if item.completed
        ]
        pending = [
            f"{item.index + 1}. {item.instruction}"
            for item in self.stages
            if not item.completed
        ]
        stage_rule = self._stage_rule(stage)
        return (
            f"原始任务：{self.root_instruction}\n"
            f"任务阶段：{stage.index + 1}/{len(self.stages)}\n"
            f"已完成阶段：{'；'.join(completed) if completed else '无'}\n"
            f"待完成阶段：{'；'.join(pending) if pending else '无'}\n"
            f"当前只执行这一阶段：{stage.instruction}\n"
            f"{stage_rule}\n"
            "不要提前执行后续阶段；当前阶段完成时输出 done=true。"
        )

    def summary(self) -> str:
        if not self.stages:
            return "任务阶段：未拆解。"
        parts = []
        for stage in self.stages:
            marker = "✓" if stage.completed else ("→" if stage.index == self.current_index else "·")
            parts.append(f"{marker}{stage.index + 1}:{stage.instruction}")
        return "任务阶段：" + " | ".join(parts)

    def inject_stages(self, vlm_stages: list, root_instruction: str = ""):
        """Inject stages parsed by the VLM at the first bbox call."""
        if root_instruction:
            self.root_instruction = (root_instruction or "").strip()
        stages = self._stages_from_dicts(vlm_stages)
        if stages:
            self.stages = stages
            self.current_index = 0

    def _stages_from_dicts(self, raw_stages: list) -> List[TaskStage]:
        stages = []
        mode_map = {
            "forward": "action", "back": "action", "backward": "action",
            "left": "action", "right": "action",
            "up": "action", "down": "action",
            "前进": "action", "后退": "action",
            "左转": "action", "右转": "action",
            "上升": "action", "下降": "action",
        }
        for item in raw_stages or []:
            if not isinstance(item, dict):
                continue
            instruction = str(item.get("instruction", "") or "").strip()
            if not instruction:
                continue
            mode_hint = str(item.get("mode", "") or "").strip().lower()
            action = self._normalize_action_name(str(item.get("action", "") or "").strip().lower())
            value = item.get("value")
            unit = str(item.get("unit", "") or "").strip()
            target = str(item.get("target", item.get("target_query", "")) or "").strip()
            relation = str(item.get("relation", "") or "").strip()
            completion_condition = str(item.get("completion_condition", "") or "").strip()

            if action in mode_map:
                mode = "action"
                if value is None:
                    defaults = {"forward": "10", "backward": "5", "left": "90", "right": "90", "up": "5", "down": "5"}
                    value = defaults.get(action, "1")
                actions = [f"{action} {self._format_action_value(value)}"]
            elif mode_hint in ("target", "detect"):
                mode = mode_hint
                actions = []
                actions = []
            elif target and relation:
                mode = "target"
                actions = []
            else:
                mode = "target"
                actions = []
            if mode in ("target", "detect") and not target:
                target = instruction

            stage = TaskStage(
                index=len(stages),
                instruction=instruction,
                mode=mode,
                target_query=target if mode in ("target", "detect") else "",
                actions=actions,
                completion_condition=completion_condition,
            )
            stages.append(stage)
        return stages

    @staticmethod
    def _format_action_value(value) -> str:
        match = re.search(r"-?\d+(?:\.\d+)?", str(value))
        if not match:
            return "1"
        number = float(match.group(0))
        return str(int(number)) if number.is_integer() else str(number)

    @staticmethod
    def _normalize_action_name(action: str) -> str:
        aliases = {
            "back": "backward",
            "后退": "backward",
            "前进": "forward",
            "左转": "left",
            "右转": "right",
            "上升": "up",
            "下降": "down",
        }
        return aliases.get(action, action)

    @staticmethod
    def _stage_rule(stage: TaskStage) -> str:
        if stage.mode == "action":
            return "当前阶段是固定动作阶段，程序会直接执行，不需要VLM规划。"
        if stage.mode == "detect":
            return (
                f"当前阶段是寻找并锁定目标：{stage.target_query or stage.instruction}。"
                "如果当前帧已看到该目标，只需确认/对准目标并输出 done=true；不要飞向其他实例。"
            )
        return (
            f"当前阶段是实体目标导航：{stage.target_query or stage.instruction}。"
            "必须围绕当前锁定目标实例完成相对位置任务，不要切换到同类远处目标。"
        )
