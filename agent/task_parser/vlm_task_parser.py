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

你不是路径规划器，不要根据场景猜测路线；你不是目标检测器，不要输出bbox；你不需要看图像。只根据用户任务文本进行语义拆分。

**重要：所有输出字段必须使用英文。** 无论用户输入是中文还是英文，instruction、target、relation 等全部输出英文。
例如用户输入"飞到红色汽车旁边" → instruction="Fly to the red car", target="red car", relation="beside"
例如用户输入"find the white truck and fly above it" → instruction="Find the white truck and fly above it", target="white truck", relation="above"

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
1. action：明确固定动作，例如"前进10m""右转90度""上升5m"。
   - action只能是 forward/backward/left/right/up/down
   - value为数字；如果用户没给数值，默认：右转/左转=90度，前进=10m，后退=5m，上升/下降=5m
   - requires_target=false，allow_relocalize=false

2. detect：寻找、发现、搜索、定位某个目标，只要求看到并锁定目标，不要求飞到旁边。
   - 例如"找到汽车""搜索红色房子"→ "Find the car""Search for the red house"
   - target填写英文目标名，如 "car""red house"
   - relation为空
   - requires_target=true，allow_relocalize=true

3. target：围绕具体实体、语义地标或可框选区域完成导航任务。
   - 例如"飞到汽车旁边""靠近树""到房子上方""飞到较远车辆附近"
   - instruction输出完整英文任务，如 "Fly to the red car"
   - target输出英文检测目标，如 "red car"（给 GroundingDINO 用）
   - relation输出英文空间关系，如 "beside""above""near""on top"
   - 也包括道路结构任务："飞到三岔路口"→ instruction="Fly to the three-way intersection", target="three-way intersection"
   - requires_target=true，allow_relocalize=true

拆分规则：
1. 按时间顺序拆分。"然后、之后、接着、随后、再、并且、并、逗号、分号、句号"是阶段边界。
2. 如果一个短语同时包含动作和后续目标，例如"右转找到汽车"，拆成"右转"和"找到汽车"。
3. 如果后续阶段出现"它、目标、旁边、附近、上方"等省略表达，应继承最近一次明确target。
4. 不要臆造用户没有要求的目标、距离或转角。
5. 如果任务存在歧义，优先拆成更保守、更短的阶段。
6. 输出的 stages 必须覆盖完整原始任务，不能遗漏。
7. 除 action 阶段外，不要生成具体飞行动作。"""


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
        
        return parse_task_parser_to_stages(response_text or "")

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


def parse_task_parser_to_stages(response_text: str) -> List[TaskStage]:
    """Parse LLM response text into List[TaskStage].

    不再硬编码 mode 白名单 —— mode 由 VLM prompt 约束，此处只做基本校验。
    """
    data = _load_json_object(response_text)
    raw_stages = data.get("stages") if isinstance(data, dict) else None
    if not isinstance(raw_stages, list):
        raise ValueError("task parser response does not contain stages list")

    stages = []
    for i, item in enumerate(raw_stages):
        if not isinstance(item, dict):
            continue
        instruction = str(item.get("instruction", "") or "").strip()
        mode = str(item.get("mode", "") or "").strip().lower()
        if not instruction or not mode:
            continue
        target = str(item.get("target", "") or "").strip()
        if mode in ("target", "detect") and not target:
            target = instruction

        value = item.get("value")
        try:
            value = float(value) if value is not None else None
        except (ValueError, TypeError):
            value = None

        # action 阶段默认值兜底：VLM 可能未填 value → 按 action 类型补默认
        if mode == "action" and value is None:
            value = _ACTION_VALUE_DEFAULTS.get(str(item.get("action", "")).strip().lower(), 0.0)

        stage = TaskStage(
            index=i,
            instruction=instruction,
            mode=mode,
            target=target if mode in ("target", "detect") else "",
            relation=str(item.get("relation", "") or "").strip(),
            action=str(item.get("action", "") or "").strip().lower(),
            value=value,
            unit=str(item.get("unit", "") or "").strip(),
            requires_target=bool(item.get("requires_target", mode in ("target", "detect"))),
            allow_relocalize=bool(item.get("allow_relocalize", mode in ("target", "detect"))),
            completion_condition=str(item.get("completion_condition", "") or "").strip(),
        )
        stages.append(stage)

    if not stages:
        raise ValueError("task parser response did not yield valid stages")
    return stages


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
