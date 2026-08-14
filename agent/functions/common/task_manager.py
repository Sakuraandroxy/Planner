"""Multi-stage task management for closed-loop AirSim navigation."""

from __future__ import annotations

import re
from typing import Dict, List, Optional

from agent.functions.task_parser.base import TaskStage


class TaskManager:
    """Manage parsed task stages and runtime completion state."""

    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.root_instruction: str = ""
        self.stages: List[TaskStage] = []
        self.current_index: int = 0
        self._stage_state: Dict[int, dict] = {}

    def start(self, instruction: str):
        self.root_instruction = (instruction or "").strip()
        self.stages = []
        self._stage_state = {}
        self.current_index = 0

    def start_with_stages(self, instruction: str, parsed_stages: list):
        self.root_instruction = (instruction or "").strip()
        if parsed_stages and hasattr(parsed_stages[0], "index") and not isinstance(parsed_stages[0], dict):
            stages = list(parsed_stages)
        else:
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

    def current_stage(self) -> Optional[TaskStage]:
        if 0 <= self.current_index < len(self.stages):
            return self.stages[self.current_index]
        return None

    def is_done(self) -> bool:
        return bool(self.stages) and self.current_index >= len(self.stages)

    def is_stage_completed(self, index: int) -> bool:
        return self._stage_state.get(index, {}).get("completed", False)

    def is_stage_failed(self, index: int) -> bool:
        return self._stage_state.get(index, {}).get("failed", False)

    def stage_completion_reason(self, index: int) -> str:
        return self._stage_state.get(index, {}).get("reason", "")

    def complete_current(self, reason: str = "") -> Optional[TaskStage]:
        stage = self.current_stage()
        if stage is None:
            return None
        self._stage_state[stage.index] = {"completed": True, "reason": reason or ""}
        self.current_index += 1
        return stage

    def fail_current(self, reason: str = "") -> Optional[TaskStage]:
        """Record a terminal stage failure without advancing to future stages."""
        stage = self.current_stage()
        if stage is None:
            return None
        self._stage_state[stage.index] = {"completed": False, "failed": True, "reason": reason or ""}
        return stage

    def current_prompt(self) -> str:
        stage = self.current_stage()
        if stage is None:
            return self.root_instruction

        completed = [
            f"{s.index + 1}. {s.instruction} ({self.stage_completion_reason(s.index) or 'done'})"
            for s in self.stages
            if self.is_stage_completed(s.index)
        ]
        pending = [
            f"{s.index + 1}. {s.instruction}"
            for s in self.stages
            if not self.is_stage_completed(s.index)
        ]
        return (
            f"Original task: {self.root_instruction}\n"
            f"Task stage: {stage.index + 1}/{len(self.stages)}\n"
            f"Completed stages: {'; '.join(completed) if completed else 'none'}\n"
            f"Pending stages: {'; '.join(pending) if pending else 'none'}\n"
            f"Current executable stage: {stage.instruction}\n"
            f"{self._stage_rule(stage)}\n"
            "Do not execute future stages early. Output done=true only when the current stage is complete."
        )

    def summary(self) -> str:
        if not self.stages:
            return "Task stages: not parsed"
        parts = []
        for stage in self.stages:
            if self.is_stage_completed(stage.index):
                marker = "done"
            elif self.is_stage_failed(stage.index):
                marker = "failed"
            elif stage.index == self.current_index:
                marker = "current"
            else:
                marker = "pending"
            parts.append(f"{marker}{stage.index + 1}:{stage.instruction}")
        return "Task stages: " + " | ".join(parts)

    def inject_stages(self, vlm_stages: list, root_instruction: str = ""):
        if root_instruction:
            self.root_instruction = root_instruction.strip()
        if vlm_stages and hasattr(vlm_stages[0], "index") and not isinstance(vlm_stages[0], dict):
            stages = list(vlm_stages)
        else:
            stages = self._stages_from_dicts(vlm_stages)
        if stages:
            self.stages = [self._repair_stage_instruction(stage) for stage in stages]
            self._stage_state = {}
            self.current_index = 0

    _STAGE_RULES = {
        "action": "Current stage is a fixed action; execute it directly without VLM planning.",
        "detect": (
            "Current stage is to find and lock onto target: {target}. "
            "If target is visible in current frame, confirm/lock and output done=true. "
            "Do not fly to other instances."
        ),
    }
    _STAGE_RULES_DEFAULT = (
        "Current stage: navigate to target entity: {target}. "
        "Plan around the locked target instance to achieve the spatial relation. "
        "Do not switch to distant same-class targets."
    )

    @staticmethod
    def _stage_rule(stage: TaskStage) -> str:
        target = stage.target_query or stage.instruction
        template = TaskManager._STAGE_RULES.get(stage.mode, TaskManager._STAGE_RULES_DEFAULT)
        rule = template.format(target=target)
        if getattr(stage, "ordinal", None):
            rule += f" The target instance is encounter/order #{int(stage.ordinal)}; keep this identity stable."
        elif getattr(stage, "selection_rule", ""):
            rule += f" Target selection rule: {stage.selection_rule}."
        if getattr(stage, "auxiliary_targets", None):
            anchors = ", ".join(str(t) for t in stage.auxiliary_targets)
            rule += f" Use these landmark qualifiers to disambiguate the target: {anchors}."
        if getattr(stage, "view_relative", False):
            rule += " Resolve and keep the target instance selected from the view captured when this stage became active."
        if getattr(stage, "return_target", False):
            rule += " Reuse the previously remembered target instance; do not renumber targets from the current view."
        return rule

    _ACTION_DEFAULTS = {
        "forward": 10,
        "backward": 5,
        "left": 90,
        "right": 90,
        "up": 5,
        "down": 5,
        "land": 0,
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

            action_raw = TaskManager._normalize_action_name(
                str(item.get("action", "") or "").strip().lower()
            )
            mode_hint = str(item.get("mode", "") or "").strip().lower()

            if action_raw in TaskManager._ACTION_DEFAULTS:
                mode = "action"
                value = item.get("value")
                if action_raw == "land":
                    value = None
                else:
                    try:
                        value = float(value) if value is not None else TaskManager._ACTION_DEFAULTS[action_raw]
                    except (TypeError, ValueError):
                        value = TaskManager._ACTION_DEFAULTS[action_raw]
            elif mode_hint in {"target", "detect"}:
                mode = mode_hint
                action_raw = ""
                value = None
            else:
                mode = "target"
                action_raw = ""
                value = None

            target = str(item.get("target", item.get("target_query", "")) or "").strip()
            ordinal = TaskManager._coerce_ordinal(item.get("ordinal"))
            if ordinal and target:
                target = TaskManager._strip_ordinal_from_target(target)
            if mode in {"target", "detect"} and not target:
                target = instruction
            auxiliary_targets = TaskManager._coerce_auxiliary_targets(item.get("auxiliary_targets"))

            stages.append(
                TaskStage(
                    index=len(stages),
                    instruction=instruction,
                    mode=mode,
                    target=target if mode in {"target", "detect"} else "",
                    action=action_raw if mode == "action" else "",
                    value=value if mode == "action" else None,
                    unit=str(item.get("unit", "") or "").strip(),
                    relation=str(item.get("relation", "") or "").strip(),
                    completion_condition=str(item.get("completion_condition", "") or "").strip(),
                    # 这两个字段供 MissionMemory 维护多实例身份，例如“第2辆红车”。
                    ordinal=ordinal,
                    selection_rule=str(item.get("selection_rule", "") or "").strip().lower(),
                    stage_kind=str(item.get("stage_kind", "") or "").strip().lower(),
                    view_relative=bool(item.get("view_relative", False)),
                    return_target=bool(item.get("return_target", False)),
                    auxiliary_targets=auxiliary_targets if mode in {"target", "detect"} else [],
                )
            )
        return stages

    @staticmethod
    def _normalize_action_name(action: str) -> str:
        aliases = {
            "back": "backward",
            "move_forward": "forward",
            "move_backward": "backward",
            "turn_left": "left",
            "turn_right": "right",
            "ascend": "up",
            "descend": "down",
            "landing": "land",
            "touchdown": "land",
            "touch_down": "land",
        }
        return aliases.get(action, action)

    @staticmethod
    def _coerce_ordinal(value) -> int | None:
        try:
            if value in (None, ""):
                return None
            ordinal = int(value)
            return ordinal if ordinal > 0 else None
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _strip_ordinal_from_target(target: str) -> str:
        text = str(target or "").strip()
        text = re.sub(r"\b(first|second|third|fourth|fifth|\d+(?:st|nd|rd|th))\b", " ", text, flags=re.IGNORECASE)
        text = re.sub(r"第\s*[0-9一二两三四五六七八九]+\s*[个辆台座只架]?", " ", text)
        return re.sub(r"\s+", " ", text).strip()

    @staticmethod
    def _coerce_auxiliary_targets(value) -> List[str]:
        if isinstance(value, list):
            raw = value
        elif isinstance(value, str):
            raw = re.split(r"[,;/，、]| and ", value)
        else:
            raw = []
        seen = set()
        out: List[str] = []
        for item in raw:
            text = re.sub(r"\s+", " ", str(item or "").strip())
            key = re.sub(r"[^0-9a-zA-Z\u4e00-\u9fff]+", " ", text.lower()).strip()
            compact = key.replace(" ", "")
            # left front/右前方只是方位限定，不是后续要检测和记忆的实体锚点。
            if key in {"left", "right", "front", "left front", "right front", "front left", "front right"}:
                continue
            if compact in {"左", "右", "前", "左前", "右前", "左前方", "右前方", "前方"}:
                continue
            if text and key and key not in seen:
                seen.add(key)
                out.append(text)
        return out

    @staticmethod
    def _repair_stage_instruction(stage: TaskStage) -> TaskStage:
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
        if not low or low.startswith(("the ", "a ", "an ")):
            return ""
        return "the "
