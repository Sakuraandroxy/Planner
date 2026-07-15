"""Multi-stage task management for closed-loop AirSim navigation.

Parser 的 TaskStage 是唯一数据源，运行时状态（completed / reason）由
_stage_state dict 独立追踪，不在 TaskStage 上挂脏字段。
"""
from __future__ import annotations

import re
from typing import Dict, List, Optional

from agent.task_parser.base import TaskStage


class TaskManager:
    """管理 VLM 解析后的多阶段任务。"""

    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.root_instruction: str = ""
        self.stages: List[TaskStage] = []
        self.current_index: int = 0
        # { stage.index: {"completed": bool, "reason": str} }
        self._stage_state: Dict[int, dict] = {}

    # ── lifecycle ──────────────────────────────────────────

    def start(self, instruction: str):
        self.root_instruction = (instruction or "").strip()
        self.stages = []
        self._stage_state = {}
        self.current_index = 0

    def start_with_stages(self, instruction: str, parsed_stages: list):
        """直接注入 parser 输出的 TaskStage 列表（或旧格式 dict）。"""
        self.root_instruction = (instruction or "").strip()
        if parsed_stages and hasattr(parsed_stages[0], 'index') and not isinstance(parsed_stages[0], dict):
            # 已经是 parser 的 TaskStage —— 直接使用
            stages = list(parsed_stages)
        else:
            # 旧格式 dict → TaskStage
            stages = self._stages_from_dicts(parsed_stages)
        if not stages:
            raise ValueError("no valid parsed task stages")
        self.stages = [self._repair_stage_instruction(stage) for stage in stages]
        self._stage_state = {}
        self.current_index = 0

    def reset(self):
        self.root_instruction = ""
        self.stages = []
        self._stage_state = {}
        self.current_index = 0

    # ── queries ────────────────────────────────────────────

    def current_stage(self) -> Optional[TaskStage]:
        if 0 <= self.current_index < len(self.stages):
            return self.stages[self.current_index]
        return None

    def is_done(self) -> bool:
        return bool(self.stages) and self.current_index >= len(self.stages)

    def is_stage_completed(self, index: int) -> bool:
        return self._stage_state.get(index, {}).get("completed", False)

    def stage_completion_reason(self, index: int) -> str:
        return self._stage_state.get(index, {}).get("reason", "")

    # ── mutations ──────────────────────────────────────────

    def complete_current(self, reason: str = "") -> Optional[TaskStage]:
        stage = self.current_stage()
        if stage is None:
            return None
        self._stage_state[stage.index] = {"completed": True, "reason": reason or ""}
        self.current_index += 1
        return stage

    # ── display ────────────────────────────────────────────

    def current_prompt(self) -> str:
        stage = self.current_stage()
        if stage is None:
            return self.root_instruction
        completed = [
            f"{s.index + 1}. {s.instruction}（{self.stage_completion_reason(s.index) or '已完成'}）"
            for s in self.stages if self.is_stage_completed(s.index)
        ]
        pending = [
            f"{s.index + 1}. {s.instruction}"
            for s in self.stages if not self.is_stage_completed(s.index)
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
        for s in self.stages:
            if self.is_stage_completed(s.index):
                marker = "✓"
            elif s.index == self.current_index:
                marker = "→"
            else:
                marker = "·"
            parts.append(f"{marker}{s.index + 1}:{s.instruction}")
        return "任务阶段：" + " | ".join(parts)

    # ── legacy support ─────────────────────────────────────

    def inject_stages(self, vlm_stages: list, root_instruction: str = ""):
        """兼容旧接口：注入 VLM 解析的阶段。"""
        if root_instruction:
            self.root_instruction = root_instruction.strip()
        if vlm_stages and hasattr(vlm_stages[0], 'index') and not isinstance(vlm_stages[0], dict):
            stages = list(vlm_stages)
        else:
            stages = self._stages_from_dicts(vlm_stages)
        if stages:
            self.stages = [self._repair_stage_instruction(stage) for stage in stages]
            self._stage_state = {}
            self.current_index = 0

    # ── internal ───────────────────────────────────────────

    _STAGE_RULES = {
        "action": "Current stage is a fixed action — execute directly without VLM planning.",
        "detect": (
            "Current stage is to find and lock onto target: {target}. "
            "If target is visible in current frame, confirm/lock and output done=true. Do NOT fly to other instances."
        ),
    }
    _STAGE_RULES_DEFAULT = (
        "Current stage: navigate to target entity: {target}. "
        "Plan around the locked target instance to achieve the spatial relation. Do not switch to distant same-class targets."
    )

    @staticmethod
    def _stage_rule(stage: TaskStage) -> str:
        target = stage.target_query or stage.instruction
        template = TaskManager._STAGE_RULES.get(
            stage.mode, TaskManager._STAGE_RULES_DEFAULT
        )
        return template.format(target=target)

    # ── dict → TaskStage 转换（兼容旧格式） ──

    _ACTION_MODE_MAP = {
        "forward": "action", "back": "action", "backward": "action",
        "left": "action", "right": "action",
        "up": "action", "down": "action",
        "前进": "action", "后退": "action",
        "左转": "action", "右转": "action",
        "上升": "action", "下降": "action",
    }
    _ACTION_DEFAULTS = {
        "forward": 10, "backward": 5, "left": 90, "right": 90, "up": 5, "down": 5,
    }

    @staticmethod
    def _stages_from_dicts(raw_stages: list) -> List[TaskStage]:
        stages: List[TaskStage] = []
        for item in raw_stages or []:
            if not isinstance(item, dict):
                continue
            instruction = str(item.get("instruction", "") or "").strip()
            if not instruction:
                continue

            mode_hint = str(item.get("mode", "") or "").strip().lower()
            action_raw = TaskManager._normalize_action_name(
                str(item.get("action", "") or "").strip().lower()
            )

            if action_raw in TaskManager._ACTION_MODE_MAP:
                mode = "action"
                value = item.get("value")
                if value is None:
                    value = TaskManager._ACTION_DEFAULTS.get(action_raw, 1)
                else:
                    try:
                        value = float(value)
                    except (ValueError, TypeError):
                        value = TaskManager._ACTION_DEFAULTS.get(action_raw, 1)
            elif mode_hint in ("target", "detect"):
                mode = mode_hint
                action_raw = ""
                value = None
            else:
                mode = "target"
                action_raw = ""
                value = None

            target = str(item.get("target", item.get("target_query", "")) or "").strip()
            if mode in ("target", "detect") and not target:
                target = instruction

            stages.append(TaskStage(
                index=len(stages),
                instruction=instruction,
                mode=mode,
                target=target if mode in ("target", "detect") else "",
                action=action_raw if mode == "action" else "",
                value=value if mode == "action" else None,
                unit=str(item.get("unit", "") or "").strip(),
                relation=str(item.get("relation", "") or "").strip(),
                completion_condition=str(item.get("completion_condition", "") or "").strip(),
            ))
        return stages

    @staticmethod
    def _normalize_action_name(action: str) -> str:
        aliases = {
            "back": "backward",
            "后退": "backward", "前进": "forward",
            "左转": "left", "右转": "right",
            "上升": "up", "下降": "down",
        }
        return aliases.get(action, action)

    @staticmethod
    def _repair_stage_instruction(stage: TaskStage) -> TaskStage:
        """Keep target instructions aligned with parser relation metadata."""
        if stage.mode != "target" or not stage.target:
            return stage

        relation = (stage.relation or "").strip().lower()
        instruction_low = (stage.instruction or "").strip().lower()
        if relation in {"above", "over", "on top", "on top of"} and not any(
            token in instruction_low for token in ("above", "over", "on top")
        ):
            stage.instruction = f"Fly above {TaskManager._article_for(stage.target)}{stage.target}"
        return stage

    @staticmethod
    def _article_for(noun: str) -> str:
        low = (noun or "").strip().lower()
        if not low:
            return ""
        if low.startswith(("the ", "a ", "an ")):
            return ""
        return "the "
