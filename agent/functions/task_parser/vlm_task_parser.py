"""VLM-backed task parser for multi-stage UAV navigation instructions."""

from __future__ import annotations

import json
import re
import time
from typing import Any, Dict, List

from agent.functions.task_parser import register_task_parser
from agent.functions.task_parser.base import BaseTaskParser, TaskStage

_ACTION_VALUE_DEFAULTS = {
    "forward": 10.0,
    "backward": 5.0,
    "left": 90.0,
    "right": 90.0,
    "up": 5.0,
    "down": 5.0,
    "land": 0.0,
}

_SPATIAL_DESCRIPTOR_KEYS = {
    "left",
    "right",
    "front",
    "back",
    "behind",
    "ahead",
    "forward",
    "left front",
    "front left",
    "right front",
    "front right",
    "left side",
    "right side",
    "左",
    "右",
    "前",
    "后",
    "後",
    "左前",
    "右前",
    "左前方",
    "右前方",
    "左边",
    "右边",
    "前方",
    "后方",
    "後方",
}

TASK_PARSER_SYSTEM_PROMPT = """You are a UAV navigation task parser.
Return only one valid JSON object. Do not include markdown or explanations.

All output fields must be English, even when the user instruction is Chinese.

Split the user instruction into ordered executable stages. Use:
- mode="action" only for fixed ego-motion commands: forward/backward/left/right/up/down.
- mode="action" for land only when the current stage is an actual landing/touch-down command.
- mode="detect" for find/search/locate target without flying to it.
- mode="target" for navigation to a physical target or spatial relation such as beside/near/above/on top.

When the instruction contains "然后/then/and then/first...then", split into multiple stages.
  Example: "右转，然后飞到红色车旁" -> task_type="multi" with:
    stage 1: {"instruction": "Turn right", "mode": "action", "action": "right", "value": 90, "unit": "degree"}
    stage 2: {"instruction": "Fly to the red car", "mode": "target", "target": "red car", "relation": "beside"}
  Example: "先左转45度再飞到房子附近" -> task_type="multi" with:
    stage 1: {"instruction": "Turn left 45 degrees", "mode": "action", "action": "left", "value": 45, "unit": "degree"}
    stage 2: {"instruction": "Fly to the house", "mode": "target", "target": "house", "relation": "near"}
  Example: "到路口右转，然后到遇到的一堆木箱旁降落" -> task_type="multi" with:
    stage 1: {"instruction": "Fly to the intersection", "mode": "target", "target": "intersection", "relation": "near"}
    stage 2: {"instruction": "Turn right", "mode": "action", "action": "right", "value": 90, "unit": "degree"}
    stage 3: {"instruction": "Fly near the pile of wooden boxes", "mode": "target", "target": "pile of wooden boxes", "relation": "near"}
    stage 4: {"instruction": "Land", "mode": "action", "action": "land", "value": null, "unit": ""}

The "instruction" field is a simple English command. Examples:
  "飞到红色车旁" -> {"instruction": "Fly to the red car", "target": "red car", "relation": "beside"}
  "飞到房子附近" -> {"instruction": "Fly to the house", "target": "house", "relation": "near"}
  "飞到房子上方" -> {"instruction": "Fly above the house", "target": "house", "relation": "above"}
  "飞到第2辆红车旁" -> {"instruction": "Fly to the second red car", "target": "red car", "relation": "beside", "ordinal": 2, "selection_rule": "ordinal"}
  "飞到灌木丛旁边的红车旁" -> {"instruction": "Fly to the red car near the bushes", "target": "red car", "relation": "beside", "auxiliary_targets": ["bushes"], "selection_rule": "anchored"}
  "原地左转后，以新视角为准飞到前方可见的第二栋楼旁" -> {"instruction": "Fly near the second building visible ahead in the new view", "target": "building", "relation": "near", "ordinal": 2, "selection_rule": "ordinal", "view_relative": true, "return_target": false}
  "飞回第一次经过的白车旁边" -> {"instruction": "Fly back to the first previously visited white car", "target": "white car", "relation": "near", "ordinal": 1, "selection_rule": "ordinal", "view_relative": false, "return_target": true}

Target binding rules:
- Set view_relative=true when the target identity or ordinal is defined by the view at the start of that stage, for example "current view", "new view", "after turning", "visible ahead", or "the first building on the left after the turn".
- A view-relative target must not be resolved from the mission-start view or from an earlier stage.
- Set return_target=true for explicit revisits such as "fly back", "return to", "previously visited", "飞回", "返回", or "回到".
- return_target and view_relative are mutually exclusive. A return target must use view_relative=false even if the sentence also contains a direction.
- Stable descriptions that do not redefine the target from a later view use view_relative=false and return_target=false.

JSON schema:
{
  "task_type": "single|multi",
  "stages": [
    {
      "index": 1,
      "instruction": "English task description",
      "mode": "target|detect|action",
      "target": "English target name",
      "relation": "beside|near|above|on top|",
      "action": "forward|backward|left|right|up|down|",
      "value": null,
      "unit": "",
      "requires_target": false,
      "allow_relocalize": false,
      "completion_condition": "English completion condition",
      "ordinal": null,
      "selection_rule": "stable|ordinal|nearest|anchored",
      "stage_kind": "navigation|landing|search|action",
      "view_relative": false,
      "return_target": false,
      "auxiliary_targets": ["English landmark or qualifier targets, e.g. bushes"]
    }
  ]
}
"""

TASK_PARSER_USER_PROMPT = """User task:
{task}

Return only the task parsing JSON."""


@register_task_parser("vlm_task_parser")
@register_task_parser("vlm_parser")
class TaskParser(BaseTaskParser):
    """Parse a natural-language task into TaskStage objects."""

    def __init__(self, vlm=None, max_tokens: int = 2048):
        self.vlm = vlm
        self.max_tokens = max_tokens
        self.last_timing: Dict[str, Any] = {}

    def parse(self, instruction: str) -> List[TaskStage]:
        started = time.perf_counter()
        messages = [
            {"role": "system", "content": TASK_PARSER_SYSTEM_PROMPT},
            {"role": "user", "content": TASK_PARSER_USER_PROMPT.format(task=(instruction or "").strip())},
        ]

        if self.vlm is not None:
            response_text, _reasoning = self.vlm.call(messages, max_tokens=self.max_tokens)
            elapsed = time.perf_counter() - started
            self.last_timing = dict(getattr(self.vlm, "last_call_info", {}) or {})
            self.last_timing["name"] = "task_parse"
            self.last_timing["elapsed"] = self.last_timing.get("elapsed", elapsed)
        else:
            response_text = self._call_api_directly(messages)
            elapsed = time.perf_counter() - started
            self.last_timing = {"name": "task_parse", "elapsed": elapsed}

        return parse_task_parser_to_stages(response_text or "", original_instruction=instruction)

    def _call_api_directly(self, messages) -> str:
        from config import cfg
        from openai import OpenAI
        from agent.functions.common.config_access import first_value, function_section

        fc = function_section(cfg, "TASK_PARSER")
        ag_cfg = cfg.get("AGENT", {}) or {}
        client = OpenAI(
            base_url=first_value(fc.get("URL"), ag_cfg.get("TASK_PARSER_URL"), default=""),
            api_key=first_value(fc.get("API_KEY"), ag_cfg.get("TASK_API_KEY"), default="no-key"),
        )
        resp = client.chat.completions.create(
            model=first_value(fc.get("MODEL_NAME"), ag_cfg.get("TASK_PARSER_MODEL"), default="Qwen3-VL-4B-Instruct"),
            messages=messages,
            max_tokens=int(first_value(fc.get("MAX_TOKENS"), ag_cfg.get("TASK_PARSER_MAX_TOKENS"), default=1024)),
            temperature=0.0,
            extra_body={"thinking": {"type": "disabled"}},
        )
        msg = resp.choices[0].message
        raw = msg.content or ""
        if not raw.strip():
            raw = getattr(msg, "reasoning_content", None) or ""
        return raw


def parse_task_parser_to_stages(response_text: str, original_instruction: str = "") -> List[TaskStage]:
    data = _load_json_object(response_text)
    raw_stages = data.get("stages") if isinstance(data, dict) else None
    if not isinstance(raw_stages, list):
        raise ValueError("task parser response does not contain stages list")

    stages: List[TaskStage] = []
    single_stage_original = original_instruction if len(raw_stages) == 1 else ""
    for item in raw_stages:
        if not isinstance(item, dict):
            continue
        instruction = str(item.get("instruction", "") or "").strip()
        mode = str(item.get("mode", "") or "").strip().lower()
        if not instruction or not mode:
            continue

        action = _normalize_action(str(item.get("action", "") or "").strip().lower())
        target = str(item.get("target", "") or "").strip()
        relation = str(item.get("relation", "") or "").strip().lower()
        mode, target, relation = _coerce_spatial_action_stage(
            mode=mode,
            action=action,
            target=target,
            relation=relation,
            instruction=instruction,
            original_instruction=single_stage_original,
        )

        value = item.get("value")
        ordinal = _coerce_ordinal(item.get("ordinal"))
        if ordinal is None:
            ordinal = _infer_ordinal_from_text(
                " ".join([
                    instruction,
                    target,
                    str(item.get("completion_condition", "") or ""),
                    original_instruction or "",
                ])
            )
        selection_rule = str(item.get("selection_rule", "") or "").strip().lower()
        if not selection_rule:
            selection_rule = _infer_selection_rule(
                " ".join([instruction, target, original_instruction or ""]),
                ordinal=ordinal,
            )
        completion_condition = str(item.get("completion_condition", "") or "").strip()
        binding_text = " ".join([instruction, target, completion_condition])
        return_target = bool(
            _coerce_bool(item.get("return_target"))
            or _has_return_to_intent(binding_text)
        )
        view_relative = bool(
            _coerce_bool(item.get("view_relative"))
            or _has_view_relative_intent(binding_text, ordinal=ordinal)
        )
        if return_target:
            view_relative = False
        auxiliary_targets = _coerce_auxiliary_targets(item.get("auxiliary_targets"))
        split_target, split_aux = _split_target_and_auxiliary(
            target,
            instruction=instruction,
            completion_condition=completion_condition,
            original_instruction=original_instruction,
        )
        if split_target:
            target = split_target
        auxiliary_targets = _merge_unique(auxiliary_targets + split_aux)
        if auxiliary_targets and selection_rule == "stable":
            selection_rule = "anchored"
        if ordinal and target:
            target = _strip_ordinal_from_target(target)

        if mode == "action":
            if action not in _ACTION_VALUE_DEFAULTS:
                action = _infer_action_from_instruction(instruction)
            value = _coerce_action_value(value, action)
            target = ""
            relation = ""
        else:
            action = ""
            value = None
            if not target:
                target = instruction

        stages.append(
            TaskStage(
                index=len(stages),
                instruction=_repair_instruction(instruction, target, relation, mode),
                mode=mode,
                target=target if mode in {"target", "detect"} else "",
                relation=relation if mode == "target" else "",
                action=action if mode == "action" else "",
                value=value if mode == "action" else None,
                unit=str(item.get("unit", "") or "").strip(),
                requires_target=bool(item.get("requires_target", mode in {"target", "detect"})),
                allow_relocalize=bool(item.get("allow_relocalize", mode in {"target", "detect"})),
                completion_condition=completion_condition,
                # ordinal/selection_rule 用于memory锁定“第N个目标”，避免多实例场景重新编号。
                ordinal=ordinal,
                selection_rule=selection_rule if mode in {"target", "detect"} else "",
                stage_kind=str(item.get("stage_kind", "") or "").strip().lower(),
                view_relative=view_relative if mode in {"target", "detect"} else False,
                return_target=return_target if mode in {"target", "detect"} else False,
                auxiliary_targets=auxiliary_targets if mode in {"target", "detect"} else [],
            )
        )

    if not stages:
        raise ValueError("task parser returned no valid stages")
    return _split_landing_stages(stages)


def parse_task_parser_response(response_text: str, original_instruction: str = "") -> List[TaskStage]:
    return parse_task_parser_to_stages(response_text, original_instruction=original_instruction)


def _load_json_object(text: str) -> Dict[str, Any]:
    cleaned = (text or "").strip()
    if cleaned.startswith("```"):
        cleaned = re.sub(r"^```(?:json)?\s*", "", cleaned)
        cleaned = re.sub(r"\s*```$", "", cleaned)
    try:
        return json.loads(cleaned)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", cleaned, flags=re.S)
        if not match:
            raise
        return json.loads(match.group(0))


def _normalize_action(action: str) -> str:
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


def _infer_action_from_instruction(instruction: str) -> str:
    text = (instruction or "").lower()
    for name in ("forward", "backward", "left", "right", "up", "down", "land"):
        if name in text:
            return name
    if "turn left" in text:
        return "left"
    if "turn right" in text:
        return "right"
    if "ascend" in text:
        return "up"
    if "descend" in text:
        return "down"
    if "降落" in text or "着陆" in text or "touch down" in text:
        return "land"
    return ""


def _coerce_action_value(value, action: str) -> float | None:
    if action not in _ACTION_VALUE_DEFAULTS:
        return None
    if action == "land":
        return None
    try:
        return float(value) if value is not None else _ACTION_VALUE_DEFAULTS[action]
    except (TypeError, ValueError):
        return _ACTION_VALUE_DEFAULTS[action]


def _coerce_spatial_action_stage(
    *,
    mode: str,
    action: str,
    target: str,
    relation: str,
    instruction: str,
    original_instruction: str,
):
    source = f"{original_instruction} {instruction}".lower()
    if _has_above_relation(source) and target:
        return "target", target, "above"
    if _has_return_to_intent(source) and target:
        # "previously passed/经过的" identifies an old instance.  The desired
        # spatial relation is returning near it, not passing it again.
        return "target", target, "near"
    if mode not in {"action", "detect", "target"}:
        mode = "target"
    if mode == "action" and action not in _ACTION_VALUE_DEFAULTS:
        mode = "target"
    return mode, target, relation


def _has_above_relation(text: str) -> bool:
    lower = (text or "").lower()
    return any(token in lower for token in ("above", "over", "on top", "top of", "上方", "上面", "顶部"))


def _has_return_to_intent(text: str) -> bool:
    lower = (text or "").lower()
    return bool(
        re.search(r"\b(?:fly|go|come|head|navigate)?\s*back\s+to\b", lower)
        or re.search(r"\breturn\s+to\b", lower)
        or any(
            token in lower
            for token in (
                "previously visited",
                "visited before",
                "previously passed",
                "飞回",
                "返回",
                "回到",
                "回去",
                "之前经过",
                "先前经过",
            )
        )
    )


def _coerce_bool(value, *, default: bool = False) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "y"}:
        return True
    if normalized in {"0", "false", "no", "n", ""}:
        return False
    return bool(default)


def _has_view_relative_intent(text: str, *, ordinal: int | None = None) -> bool:
    normalized = re.sub(r"[-_/]+", " ", str(text or "").lower())
    normalized = re.sub(r"\s+", " ", normalized).strip()
    compact = normalized.replace(" ", "")
    explicit_tokens = (
        "current view",
        "current perspective",
        "new view",
        "new perspective",
        "view after the turn",
        "after turning",
        "after the turn",
        "visible ahead",
        "visible in front",
        "when this stage starts",
        "at the start of this stage",
    )
    explicit_zh = (
        "当前视野",
        "当前视角",
        "新视野",
        "新视角",
        "转向后",
        "转弯后",
        "转向完成后",
        "前方可见",
        "阶段开始时",
    )
    if any(token in normalized for token in explicit_tokens):
        return True
    if any(token in compact for token in explicit_zh):
        return True
    if not ordinal:
        return False
    directional_tokens = (
        "front right",
        "right front",
        "front left",
        "left front",
        "on the left",
        "on the right",
        "to the left",
        "to the right",
        "ahead",
        "in front",
        "左前方",
        "右前方",
        "左侧",
        "右侧",
        "前方",
    )
    return any(token in normalized or token in compact for token in directional_tokens)


def _has_landing_intent(text: str) -> bool:
    lower = (text or "").lower()
    return any(token in lower for token in (" land", "landing", "touch down", "降落", "着陆", "落地"))


def _split_landing_stages(stages: List[TaskStage]) -> List[TaskStage]:
    out: List[TaskStage] = []
    for stage in stages:
        if (
            stage.mode == "target"
            and _has_landing_intent(f" {stage.instruction} {stage.completion_condition}")
        ):
            # 降落不能由完成判定“判定”出来，必须拆成导航到目标旁 + 真实land动作。
            stage.instruction = re.sub(
                r"\b(and\s+)?(then\s+)?(land|landing|touch down)\b",
                "",
                stage.instruction,
                flags=re.IGNORECASE,
            ).strip() or _repair_instruction(stage.instruction, stage.target, stage.relation, stage.mode)
            stage.completion_condition = re.sub(
                r"\b(and\s+)?(then\s+)?(land|landing|touch down)\b",
                "",
                stage.completion_condition,
                flags=re.IGNORECASE,
            ).strip()
            out.append(stage)
            out.append(
                TaskStage(
                    index=0,
                    instruction="Land",
                    mode="action",
                    action="land",
                    value=None,
                    unit="",
                    stage_kind="landing",
                )
            )
        else:
            out.append(stage)
    for index, stage in enumerate(out):
        stage.index = index
    return out


def _coerce_ordinal(value) -> int | None:
    try:
        if value in (None, ""):
            return None
        ordinal = int(value)
        return ordinal if ordinal > 0 else None
    except (TypeError, ValueError):
        return None


def _infer_ordinal_from_text(text: str) -> int | None:
    lower = (text or "").lower()
    mapping = {
        "first": 1,
        "1st": 1,
        "second": 2,
        "2nd": 2,
        "third": 3,
        "3rd": 3,
        "fourth": 4,
        "4th": 4,
        "fifth": 5,
        "5th": 5,
    }
    for token, ordinal in mapping.items():
        if re.search(rf"\b{re.escape(token)}\b", lower):
            return ordinal
    zh_match = re.search(r"第\s*([0-9一二两三四五六七八九]+)", lower)
    if zh_match:
        raw = zh_match.group(1)
        if raw.isdigit():
            return int(raw)
        return {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}.get(raw)
    return None


def _infer_selection_rule(text: str, ordinal: int | None = None) -> str:
    lower = (text or "").lower()
    if ordinal:
        return "ordinal"
    if any(token in lower for token in ("nearest", "closest", "最近")):
        return "nearest"
    return "stable"


def _coerce_auxiliary_targets(value) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        raw_items = re.split(r"[,;/，、]| and ", value)
    elif isinstance(value, list):
        raw_items = value
    else:
        raw_items = []
    return _merge_unique(
        item
        for item in (str(item or "").strip() for item in raw_items)
        if item and not _is_spatial_descriptor(item)
    )


def _split_target_and_auxiliary(
    target: str,
    *,
    instruction: str = "",
    completion_condition: str = "",
    original_instruction: str = "",
) -> tuple[str, List[str]]:
    """Split target qualifiers such as 'red car near bushes'.

    这里把“主目标”和“锚点目标”拆开，便于 memory 在执行白车阶段时也提前记住红车和灌木。
    """
    target_text = str(target or "").strip()
    aux: List[str] = []
    primary = target_text
    pattern = (
        r"\b(.+?)\s+"
        r"(?:near|beside|next to|by|adjacent to|close to|in front of|behind)\s+"
        r"(?:the\s+|a\s+|an\s+)?(.+)$"
    )
    match = re.search(pattern, target_text, flags=re.IGNORECASE)
    if match:
        primary = match.group(1).strip()
        aux.append(_clean_auxiliary_target(match.group(2)))

    source = " ".join([instruction or "", completion_condition or ""])
    if primary:
        escaped = re.escape(primary)
        inst_match = re.search(
            rf"{escaped}\s+(?:near|beside|next to|by|adjacent to|close to)\s+"
            r"(?:the\s+|a\s+|an\s+)?([a-zA-Z][a-zA-Z\s-]{1,40})",
            source,
            flags=re.IGNORECASE,
        )
        if inst_match:
            aux.append(_clean_auxiliary_target(inst_match.group(1)))

    original = str(original_instruction or "")
    stage_source = f"{target_text} {instruction}".lower()
    if "red" in stage_source and any(token in original for token in ("灌木", "草丛", "树丛")):
        aux.append("bushes")
    aux = [
        item for item in _merge_unique(aux)
        if normalize_simple(item) != normalize_simple(primary)
        and not _is_spatial_descriptor(item)
    ]
    return primary.strip(), aux


def _clean_auxiliary_target(text: str) -> str:
    cleaned = re.sub(
        r"\b(?:near|beside|next to|by|and|then|finally|land|landing|touch down)\b.*$",
        "",
        str(text or "").strip(),
        flags=re.IGNORECASE,
    )
    cleaned = re.sub(r"\b(the|a|an)\b", " ", cleaned, flags=re.IGNORECASE)
    cleaned = re.sub(r"\s+", " ", cleaned).strip(" .,:;")
    return "" if _is_spatial_descriptor(cleaned) else cleaned


def _merge_unique(items) -> List[str]:
    seen = set()
    out: List[str] = []
    for item in items:
        cleaned = re.sub(r"\s+", " ", str(item or "").strip())
        if not cleaned:
            continue
        key = normalize_simple(cleaned)
        if not key or key in seen:
            continue
        seen.add(key)
        out.append(cleaned)
    return out


def normalize_simple(text: str) -> str:
    return re.sub(r"[^0-9a-zA-Z\u4e00-\u9fff]+", " ", str(text or "").lower()).strip()


def _is_spatial_descriptor(text: str) -> bool:
    """过滤纯方位词：left front/右前方是目标限定方向，不是可检测锚点物体。"""
    normalized = normalize_simple(text)
    compact = normalized.replace(" ", "")
    if not normalized:
        return True
    if normalized in _SPATIAL_DESCRIPTOR_KEYS or compact in _SPATIAL_DESCRIPTOR_KEYS:
        return True
    tokens = set(normalized.split())
    direction_tokens = {"left", "right", "front", "back", "behind", "ahead", "forward", "side"}
    return bool(tokens and tokens.issubset(direction_tokens))


def _strip_ordinal_from_target(target: str) -> str:
    text = str(target or "").strip()
    text = re.sub(r"\b(first|second|third|fourth|fifth|\d+(?:st|nd|rd|th))\b", " ", text, flags=re.IGNORECASE)
    text = re.sub(r"第\s*[0-9一二两三四五六七八九]+\s*[个辆台座只架]?", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def _repair_instruction(instruction: str, target: str, relation: str, mode: str) -> str:
    if mode == "target" and target and relation in {"above", "over", "on top", "on top of"}:
        low = instruction.lower()
        if not any(token in low for token in ("above", "over", "on top")):
            article = "" if target.lower().startswith(("the ", "a ", "an ")) else "the "
            return f"Fly above {article}{target}"
    return instruction
