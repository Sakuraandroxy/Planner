"""VLM-backed task parsing for multi-stage navigation instructions."""
from __future__ import annotations

import json
import re
import time
from typing import Any, Dict, List

from agent.task_parser.base import BaseTaskParser, TaskStage

# Import register decorator HERE for the @register decorator pattern
# (imported in __init__.py which triggers auto-registration)
from agent.task_parser import register_task_parser

# action 阶段数值默认值（VLM prompt 中声明，代码兜底）
_ACTION_VALUE_DEFAULTS = {
    "forward": 10.0, "backward": 5.0,
    "left": 90.0, "right": 90.0,
    "up": 5.0, "down": 5.0,
}


TASK_PARSER_SYSTEM_PROMPT = """你是无人机任务解析器。把用户的自然语言导航任务拆解成有序、可执行的阶段。
**重要：所有输出字段必须使用英文。** 无论用户输入是中文还是英文，instruction、target、relation 等全部输出英文。
例如用户输入"飞到红色汽车旁边" → instruction="Fly to the red car", target="red car", relation="beside"
例如用户输入"find the white truck and fly above it" → instruction1="Find the white truck",mode="detect", instruction2="Fly above the white truck", target="white truck", relation="above"

必须只输出一个合法JSON对象，不要Markdown，不要解释，不要思考过程。

输出格式：
{
  "task_type": "single|multi",
  "stages": [
    {
      "index": 1,
      "instruction": "English task description for this stage",
      "mode": "target|detect|action",
      "target": "English target name for detection",
      "relation": "English spatial relation e.g. beside/above/near/on top",
      "action": "",
      "value": null,
      "unit": "",
      "requires_target": false,
      "allow_relocalize": false,
      "completion_condition": "English completion condition"
    }
  ]
}

mode定义：
核心判定原则：
- 先判断短语里是否有“目标实体/语义地标/可框选区域”。如果有，并且用户要求飞到、靠近、到达、位于它的旁边/上方/附近/顶部，则必须输出 mode="target"。
- action 只表示无人机自身的纯运动指令，不依赖任何外部目标。例如"上升5m""左转40度""直走20m"。
- 如果短语同时包含目标实体和空间关系，绝对不要把空间关系改写成直接动作。例如"飞到电线杆上方"不是"up 5m"，而是 target="power pole", relation="above"。
- instruction 必须保留空间关系。不要把 "Fly above the power pole" 简化成 "Fly to the power pole"；不要把 "Fly beside the red car" 简化成 "Fly to the red car"。

1. action：明确固定动作，例如"前进10m""右转90度""上升5m"。
   - action只能是 forward/backward/left/right/up/down
   - value为数字；如果用户没给数值，默认：右转/左转=90度，前进=10m，后退=5m，上升/下降=5m
   - 只有用户明确要求无人机自身执行动作时才用action，例如"上升5m""向上飞一点""左转"。
   - "飞到X上面/上方/顶部/屋顶/楼顶"不是action up，而是target，relation="above"或"on top"。
   - requires_target=false，allow_relocalize=false

2. detect：寻找、发现、搜索、定位某个目标，只要求看到并锁定目标，不要求飞到旁边。
   - 例如"找到汽车""搜索红色房子"→ "Find the car""Search for the red house"
   - target填写英文目标名，如 "car""red house"
   - relation为空
   - requires_target=true，allow_relocalize=true

3. target：围绕具体实体、语义地标或可框选区域完成导航任务。
   - 例如"飞到汽车旁边""靠近树""到房子上方""飞到较远车辆附近"
   - 例如"飞到电线杆上面"→ instruction="Fly above the power pole", target="power pole", relation="above"
   - 例如"飞到屋顶上面"→ instruction="Fly above the roof", target="roof", relation="above"
   - instruction输出完整英文任务，如 "Fly to the red car"
   - target输出英文检测目标，如 "red car"（给 GroundingDINO 用）
   - relation输出英文空间关系，如 "beside""above""near""on top"
   - 也包括道路结构任务："飞到三岔路口"→ instruction="Fly to the three-way intersection", target="three-way intersection"
   - requires_target=true，allow_relocalize=true

拆分规则：
1. 按时间顺序拆分。"然后、之后、接着、随后、再、并且、并、逗号、分号、句号"是阶段边界。
2. 如果一个短语同时包含动作和后续目标，例如"右转找到汽车"，拆成"右转"和"找到汽车"。
3. 如果出现 "find/search/locate/detect X and fly/go/move above/beside/near/to it"，必须拆成两个阶段：
   - stage1: mode="detect", instruction="Find X", target="X"
   - stage2: mode="target", instruction="Fly above/beside/near/to X", target="X", relation=对应空间关系
   例如 "find the white truck and fly above it" 必须输出 detect("white truck") + target("white truck", relation="above")。
4. 如果出现 "turn left/right and fly/go/move to/above/beside/near X" 或中文"左转/右转飞到X"，必须拆成两个阶段：
   - stage1: mode="action", action="left/right", 默认 value=90, unit="degree"
   - stage2: mode="target", target="X", relation=对应空间关系
   例如 "左转飞到房屋上方" 必须输出 left action + "Fly above the house" target。
5. 如果后续阶段出现"它、目标、旁边、附近、上方"等省略表达，应继承最近一次明确target。
6. 不要臆造用户没有要求的目标、距离或转角。
7. 如果任务存在歧义，优先拆成更保守、更短的阶段。
8. 输出的 stages 必须覆盖完整原始任务，不能遗漏。
9. 除 action 阶段外，不要生成具体飞行动作。

正确划分例子（严格模仿这些模式）：
例1 用户任务："飞到红色汽车旁，然后左转飞到房屋上方"
正确 stages:
[
  {"index":1,"instruction":"Fly to the red car","mode":"target","target":"red car","relation":"beside","action":"","value":null,"unit":"","requires_target":true,"allow_relocalize":true,"completion_condition":"reach beside the red car"},
  {"index":2,"instruction":"Turn left 90 degrees","mode":"action","target":"","relation":"","action":"left","value":90,"unit":"degree","requires_target":false,"allow_relocalize":false,"completion_condition":"turn left"},
  {"index":3,"instruction":"Fly above the house","mode":"target","target":"house","relation":"above","action":"","value":null,"unit":"","requires_target":true,"allow_relocalize":true,"completion_condition":"reach above the house"}
]
错误：不要输出 {"mode":"action","instruction":"Turn left and fly above the house"}，因为一个 stage 里混合了转向和目标导航。

例2 用户任务："find the white truck and fly above it"
正确 stages:
[
  {"index":1,"instruction":"Find the white truck","mode":"detect","target":"white truck","relation":"","action":"","value":null,"unit":"","requires_target":true,"allow_relocalize":true,"completion_condition":"white truck is detected"},
  {"index":2,"instruction":"Fly above the white truck","mode":"target","target":"white truck","relation":"above","action":"","value":null,"unit":"","requires_target":true,"allow_relocalize":true,"completion_condition":"reach above the white truck"}
]
错误：不要合并为 "Find the white truck and fly above it"。

例3 用户任务："右转找到蓝色摩托车，然后飞到它旁边"
正确 stages:
[
  {"index":1,"instruction":"Turn right 90 degrees","mode":"action","target":"","relation":"","action":"right","value":90,"unit":"degree","requires_target":false,"allow_relocalize":false,"completion_condition":"turn right"},
  {"index":2,"instruction":"Find the blue motorcycle","mode":"detect","target":"blue motorcycle","relation":"","action":"","value":null,"unit":"","requires_target":true,"allow_relocalize":true,"completion_condition":"blue motorcycle is detected"},
  {"index":3,"instruction":"Fly to the blue motorcycle","mode":"target","target":"blue motorcycle","relation":"beside","action":"","value":null,"unit":"","requires_target":true,"allow_relocalize":true,"completion_condition":"reach beside the blue motorcycle"}
]

例4 用户任务："飞到电线杆上方，然后前进20m"
正确 stages:
[
  {"index":1,"instruction":"Fly above the power pole","mode":"target","target":"power pole","relation":"above","action":"","value":null,"unit":"","requires_target":true,"allow_relocalize":true,"completion_condition":"reach above the power pole"},
  {"index":2,"instruction":"Move forward 20 meters","mode":"action","target":"","relation":"","action":"forward","value":20,"unit":"meter","requires_target":false,"allow_relocalize":false,"completion_condition":"move forward 20 meters"}
]
错误：不要把 "Fly above the power pole" 解析成 action="up"。"""


TASK_PARSER_USER_PROMPT = """用户任务：
{task}

请只输出任务解析JSON。"""


@register_task_parser("vlm_parser")
class TaskParser(BaseTaskParser):
    """Parse task text through the existing VLM API. Implements BaseTaskParser."""

    def __init__(self, vlm=None, max_tokens: int = 2048):
        self.vlm = vlm
        self.max_tokens = max_tokens
        self.last_timing: Dict[str, Any] = {}

    def parse(self, instruction: str) -> List[TaskStage]:
        """Parse instruction into a list of TaskStage objects.
        
        Returns List[TaskStage] as required by BaseTaskParser interface.
        """
        started = time.perf_counter()
        messages = [
            {"role": "system", "content": TASK_PARSER_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": TASK_PARSER_USER_PROMPT.format(task=(instruction or "").strip()),
            },
        ]
        
        if self.vlm is not None:
            response_text, reasoning_content = self.vlm.call(messages, max_tokens=self.max_tokens)
            elapsed = time.perf_counter() - started
            self.last_timing = dict(self.vlm.last_call_info)
            self.last_timing["name"] = "task_parse"
            self.last_timing["elapsed"] = self.last_timing.get("elapsed", elapsed)
        else:
            # Fallback: direct API call using config
            response_text = self._call_api_directly(messages)
            elapsed = time.perf_counter() - started
            self.last_timing = {"name": "task_parse", "elapsed": elapsed}
        
        return parse_task_parser_to_stages(response_text or "", original_instruction=instruction)

    def _call_api_directly(self, messages) -> str:
        """Call task parser via dedicated lightweight VLM (Qwen3-VL-4B on GPU 3).

        Uses TASK_PARSER_URL from config (separate from the heavy Qwen planner server).
        The 4B model handles text-only prompts in ~1s.
        """
        from config import cfg
        from openai import OpenAI
        ag_cfg = cfg["AGENT"]
        base_url = ag_cfg["TASK_PARSER_URL"]
        model_name = ag_cfg.get("TASK_PARSER_MODEL", "Qwen3-VL-4B-Instruct")
        max_tok = int(ag_cfg.get("TASK_PARSER_MAX_TOKENS", 1024))
        api_key = ag_cfg.get("TASK_API_KEY", "no-key")
        client = OpenAI(base_url=base_url, api_key=api_key)
        resp = client.chat.completions.create(
            model=model_name,
            messages=messages,
            max_tokens=max_tok,
            temperature=0.0,
            extra_body={"thinking": {"type": "disabled"}},
        )
        msg = resp.choices[0].message
        raw = msg.content or ""
        # MiMo 思考模式可能把内容放在 reasoning_content
        if not raw.strip():
            reasoning = getattr(msg, 'reasoning_content', None) or ""
            if reasoning.strip():
                raw = reasoning
        if not raw.strip():
            finish = getattr(resp.choices[0], 'finish_reason', 'unknown')
            print(f"  [TaskParser] empty response! finish_reason={finish}, "
                  f"model={model_name}, content_len={len(msg.content or '')}, "
                  f"reasoning_len={len(getattr(msg, 'reasoning_content', '') or '')}")
        return raw


def parse_task_parser_to_stages(response_text: str, original_instruction: str = "") -> List[TaskStage]:
    """Parse LLM response text into List[TaskStage].

    不再硬编码 mode 白名单 —— mode 由 VLM prompt 约束，此处只做基本校验。
    """
    data = _load_json_object(response_text)
    raw_stages = data.get("stages") if isinstance(data, dict) else None
    if not isinstance(raw_stages, list):
        raise ValueError("task parser response does not contain stages list")

    stages = []
    single_stage_original = original_instruction if len(raw_stages) == 1 else ""
    for i, item in enumerate(raw_stages):
        if not isinstance(item, dict):
            continue
        instruction = str(item.get("instruction", "") or "").strip()
        mode = str(item.get("mode", "") or "").strip().lower()
        if not instruction or not mode:
            continue
        target = str(item.get("target", "") or "").strip()
        mode, target, relation_override = _coerce_spatial_action_stage(
            mode=mode,
            action=str(item.get("action", "") or "").strip().lower(),
            instruction=instruction,
            target=target,
            relation=str(item.get("relation", "") or "").strip(),
            original_instruction=original_instruction,
        )
        if mode in ("target", "detect") and not target:
            target = instruction
        if mode == "target":
            relation_override, instruction = _normalize_spatial_target_instruction(
                relation=relation_override,
                instruction=instruction,
                target=target,
                original_instruction=single_stage_original,
            )

        value = item.get("value")
        try:
            value = float(value) if value is not None else None
        except (ValueError, TypeError):
            value = None

        # action 阶段默认值兜底：VLM 可能未填 value → 按 action 类型补默认
        if mode == "action" and value is None:
            value = _ACTION_VALUE_DEFAULTS.get(str(item.get("action", "")).strip().lower(), 0.0)
        if mode != "action":
            value = None

        stage = TaskStage(
            index=i,
            instruction=instruction,
            mode=mode,
            target=target if mode in ("target", "detect") else "",
            relation=relation_override,
            action=str(item.get("action", "") or "").strip().lower() if mode == "action" else "",
            value=value,
            unit=str(item.get("unit", "") or "").strip() if mode == "action" else "",
            requires_target=True if mode in ("target", "detect") else bool(item.get("requires_target", False)),
            allow_relocalize=True if mode in ("target", "detect") else bool(item.get("allow_relocalize", False)),
            completion_condition=str(item.get("completion_condition", "") or "").strip(),
        )
        stages.append(stage)

    if not stages:
        raise ValueError("task parser response did not yield valid stages")
    stages = _split_compound_turn_target_stages(stages)
    stages = _apply_original_clause_relations(stages, original_instruction)
    for idx, stage in enumerate(stages):
        stage.index = idx
    return stages


def _coerce_spatial_action_stage(
    mode: str,
    action: str,
    instruction: str,
    target: str,
    relation: str,
    original_instruction: str = "",
) -> tuple[str, str, str]:
    """Fix common parser confusion: "fly to X above" is target, not direct up."""
    if mode != "action" or action not in {"up", "down"}:
        return mode, target, relation

    raw = f"{original_instruction} {instruction}".lower()
    direct_action_words = ("上升", "升高", "下降", "降低", "ascend", "descend", "climb", "rise")
    if any(word in raw for word in direct_action_words):
        return mode, target, relation

    text = instruction.strip()
    low = text.lower()
    nav_prefixes = (
        "fly to ", "fly above ", "go to ", "move to ", "navigate to ",
        "approach ", "reach ", "head to ",
    )
    if not any(low.startswith(prefix) for prefix in nav_prefixes):
        return mode, target, relation

    inferred_target = target or _infer_target_from_navigation_instruction(text)
    inferred_relation = relation or ("below" if action == "down" else "above")
    return "target", inferred_target or text, inferred_relation


def _infer_target_from_navigation_instruction(instruction: str) -> str:
    text = instruction.strip()
    patterns = [
        r"^fly\s+to\s+the\s+(.+)$",
        r"^fly\s+to\s+(.+)$",
        r"^fly\s+above\s+the\s+(.+)$",
        r"^fly\s+above\s+(.+)$",
        r"^go\s+to\s+the\s+(.+)$",
        r"^go\s+to\s+(.+)$",
        r"^move\s+to\s+the\s+(.+)$",
        r"^move\s+to\s+(.+)$",
        r"^navigate\s+to\s+the\s+(.+)$",
        r"^navigate\s+to\s+(.+)$",
        r"^approach\s+the\s+(.+)$",
        r"^approach\s+(.+)$",
    ]
    for pattern in patterns:
        match = re.match(pattern, text, flags=re.IGNORECASE)
        if match:
            return match.group(1).strip().rstrip(".")
    return ""


def _normalize_spatial_target_instruction(
    relation: str,
    instruction: str,
    target: str,
    original_instruction: str = "",
) -> tuple[str, str]:
    """Recover spatial relations that small parsers often drop in translation."""
    if re.match(r"^\s*turn\s+(left|right)\b", instruction, flags=re.IGNORECASE):
        return relation, instruction
    raw = f"{instruction} {original_instruction or ''}".lower()
    above_words = ("上方", "上面", "上空", "顶部", "顶上", "屋顶上", "楼顶上")
    above_english = (" above ", " on top", " over ")
    if any(word in raw for word in above_words) or any(word in f" {raw} " for word in above_english):
        relation = "above"
        if target:
            noun = target.strip()
            article = "" if noun.lower().startswith(("the ", "a ", "an ")) else "the "
            instruction = f"Fly above {article}{noun}"
    return relation, instruction


def _split_compound_turn_target_stages(stages: List[TaskStage]) -> List[TaskStage]:
    """Split generic "turn left/right and fly ..." stages into action + target."""
    out: List[TaskStage] = []
    pattern = re.compile(
        r"^turn\s+(left|right)(?:\s+([0-9]+(?:\.[0-9]+)?)\s*degrees?)?\s+and\s+"
        r"(fly|go|move|navigate)\s+(.*)$",
        flags=re.IGNORECASE,
    )
    for stage in stages:
        match = pattern.match(stage.instruction.strip())
        if match:
            direction = match.group(1).lower()
            value = float(match.group(2)) if match.group(2) else _ACTION_VALUE_DEFAULTS[direction]
            nav_tail = match.group(4).strip()
            target_instruction = _nav_tail_to_instruction(nav_tail)
            target = stage.target or _infer_target_from_navigation_instruction(target_instruction)
            relation = stage.relation or _relation_from_english_instruction(target_instruction)
            out.append(TaskStage(
                index=len(out),
                instruction=f"Turn {direction} {value:g} degrees",
                mode="action",
                target="",
                relation="",
                action=direction,
                value=value,
                unit="degree",
                requires_target=False,
                allow_relocalize=False,
                completion_condition="",
            ))
            out.append(TaskStage(
                index=len(out),
                instruction=target_instruction,
                mode="target",
                target=target,
                relation=relation,
                action="",
                value=None,
                unit="",
                requires_target=True,
                allow_relocalize=True,
                completion_condition=stage.completion_condition,
            ))
            continue
        out.append(stage)
    return out


def _relation_from_english_instruction(instruction: str) -> str:
    low = f" {instruction.lower()} "
    if " above " in low or " on top" in low or " over " in low:
        return "above"
    if " beside " in low or " next to " in low:
        return "beside"
    if " near " in low:
        return "near"
    return ""


def _apply_original_clause_relations(stages: List[TaskStage], original_instruction: str) -> List[TaskStage]:
    """Align target-stage relations with original text clauses by order.

    This prevents a later clause like "飞到房屋上方" from changing an earlier
    target like "飞到红色汽车旁" into "above".
    """
    if not original_instruction:
        return stages
    clauses = [c.strip() for c in re.split(r"(?:然后|之后|接着|随后|再|，|,|；|;|。|\.)", original_instruction) if c.strip()]
    if not clauses:
        return stages

    target_indices = [i for i, stage in enumerate(stages) if stage.mode == "target"]
    for order, stage_idx in enumerate(target_indices):
        if order >= len(clauses):
            break
        clause = clauses[order]
        relation = _relation_from_original_clause(clause)
        if not relation:
            continue
        stage = stages[stage_idx]
        stage.relation = relation
        if relation == "above":
            stage.instruction = _spatial_instruction("above", stage.target, stage.instruction)
        elif relation in {"beside", "near"} and "above" in stage.instruction.lower():
            stage.instruction = _spatial_instruction(relation, stage.target, stage.instruction)
    return stages


def _relation_from_original_clause(clause: str) -> str:
    if any(word in clause for word in ("上方", "上面", "上空", "顶部", "顶上", "屋顶上", "楼顶上")):
        return "above"
    if any(word in clause for word in ("旁边", "旁", "边上", "旁侧")):
        return "beside"
    if any(word in clause for word in ("附近", "靠近", "近处")):
        return "near"
    return ""


def _spatial_instruction(relation: str, target: str, fallback: str) -> str:
    if not target:
        return fallback
    noun = target.strip()
    article = "" if noun.lower().startswith(("the ", "a ", "an ")) else "the "
    if relation == "above":
        return f"Fly above {article}{noun}"
    if relation == "beside":
        return f"Fly to {article}{noun}"
    if relation == "near":
        return f"Fly near {article}{noun}"
    return fallback


def _nav_tail_to_instruction(nav_tail: str) -> str:
    tail = nav_tail.strip().rstrip(".")
    low = tail.lower()
    if low.startswith(("to ", "above ", "near ", "beside ", "toward ", "towards ")):
        return "Fly " + tail
    return "Fly to " + tail


# Keep old parse function for backward compatibility
def parse_task_parser_response(response_text: str) -> List[Dict[str, Any]]:
    """Legacy: Return raw dict list (use parse_task_parser_to_stages for new code)."""
    data = _load_json_object(response_text)
    raw_stages = data.get("stages") if isinstance(data, dict) else None
    if not isinstance(raw_stages, list):
        raise ValueError("task parser response does not contain stages list")

    stages = []
    for item in raw_stages:
        if not isinstance(item, dict):
            continue
        instruction = str(item.get("instruction", "") or "").strip()
        mode = str(item.get("mode", "") or "").strip().lower()
        if not instruction or not mode:
            continue
        target = str(item.get("target", "") or "").strip()
        if mode in {"target", "detect"} and not target:
            target = instruction
        normalized = {
            "instruction": instruction,
            "mode": mode,
            "target": target,
            "relation": str(item.get("relation", "") or "").strip(),
            "action": str(item.get("action", "") or "").strip().lower(),
            "value": item.get("value"),
            "unit": str(item.get("unit", "") or "").strip(),
            "requires_target": bool(item.get("requires_target", mode in ("target", "detect"))),
            "allow_relocalize": bool(item.get("allow_relocalize", mode in ("target", "detect"))),
            "completion_condition": str(item.get("completion_condition", "") or "").strip(),
        }
        stages.append(normalized)
    if not stages:
        raise ValueError("task parser response did not yield valid stages")
    return stages


def _load_json_object(response_text: str) -> Dict[str, Any]:
    """Extract the first valid JSON object from model output.

    Uses json.JSONDecoder.raw_decode() which stops at the end of the
    first complete JSON object, ignoring any trailing text/explanation.
    This is robust against models that append commentary after the JSON.
    """
    text = (response_text or "").strip()
    # Strip markdown code fences
    text = re.sub(r"```json\s*", "", text, flags=re.IGNORECASE)
    text = re.sub(r"```\s*", "", text)

    # Find the first '{' and try raw_decode from there
    # Find the first '{' (server now returns only generated text, no prompt)
    idx = text.find("{")
    if idx < 0:
        raise ValueError(f"no JSON object found in response (len={len(text)})")

    decoder = json.JSONDecoder()
    try:
        data, end = decoder.raw_decode(text[idx:])
        if not isinstance(data, dict):
            raise ValueError(f"expected JSON object, got {type(data).__name__}")
        return data
    except json.JSONDecodeError:
        raise ValueError(f"cannot parse task parser JSON: {text[:200]}")
