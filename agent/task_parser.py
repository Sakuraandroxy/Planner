"""VLM-backed task parsing for multi-stage navigation instructions."""
from __future__ import annotations

import json
import re
import time
from typing import Any, Dict, List


TASK_PARSER_SYSTEM_PROMPT = """你是无人机任务解析器。你的唯一任务是把用户的自然语言导航任务拆解成有序、可执行的阶段。

你不是路径规划器，不要根据场景猜测路线；你不是目标检测器，不要输出bbox；你不需要看图像。只根据用户任务文本进行语义拆分。

必须只输出一个合法JSON对象，不要Markdown，不要解释，不要思考过程。

输出格式：
{
  "task_type": "single|multi",
  "stages": [
    {
      "index": 1,
      "instruction": "当前阶段的自然语言描述",
      "mode": "target|detect|action",
      "target": "",
      "relation": "",
      "action": "",
      "value": null,
      "unit": "",
      "requires_target": false,
      "allow_relocalize": false,
      "completion_condition": "该阶段完成条件"
    }
  ]
}

mode定义：
1. action：明确固定动作，不需要视觉判断，例如“前进10m”“右转90度”“上升5m”。
   - action只能是 forward/back/left/right/up/down
   - value为数字；如果用户没给数值，使用常见默认值：右转/左转=90度，前进=10m，后退=5m，上升/下降=5m
   - requires_target=false，allow_relocalize=false

2. detect：寻找、发现、搜索、定位某个目标，只要求看到并锁定目标，不要求飞到旁边。
   - 例如“找到汽车”“搜索红色房子”“发现电线杆”
   - target填写要找的目标
   - relation为空
   - requires_target=true，allow_relocalize=true

3. target：围绕具体实体、语义地标或可框选区域完成导航任务。
   - 例如“飞到汽车旁边”“靠近树”“到房子上方”“飞到较远车辆附近”
   - 也包括“飞到三岔路口”“飞到十字路口”“沿道路前进到尽头”“穿过路口”“飞到空旷区域”
   - target填写要检测/到达的实体或语义区域，如“汽车”“较远车辆”“红色房子”“三岔路口区域”“道路尽头”“空旷区域”
   - relation填写关系，如“旁边”“附近”“上方”“靠近”“顶部”“到达”“穿过”
   - requires_target=true，allow_relocalize=true

拆分规则：
1. 按时间顺序拆分，不要合并本应先后执行的阶段。
2. “然后、之后、接着、随后、再、并且、并、逗号、分号、句号”通常表示阶段边界。
3. 如果一个短语同时包含动作和后续目标，例如“右转找到汽车”，拆成“右转”和“找到汽车”。
4. 如果后续阶段出现“它、目标、旁边、附近、上方”等省略表达，应继承最近一次明确target。
5. 不要臆造用户没有要求的目标、距离或转角。
6. 如果任务存在歧义，优先拆成更保守、更短的阶段。
7. 输出的 stages 必须覆盖完整原始任务，不能遗漏。
8. 除 action 阶段外，不要生成具体飞行动作。"""


TASK_PARSER_USER_PROMPT = """用户任务：
{task}

请只输出任务解析JSON。"""


class TaskParser:
    """Parse task text through the existing VLM API."""

    def __init__(self, vlm, max_tokens: int = 2048):
        self.vlm = vlm
        self.max_tokens = max_tokens
        self.last_timing: Dict[str, Any] = {}

    def parse(self, task: str) -> List[Dict[str, Any]]:
        started = time.perf_counter()
        messages = [
            {"role": "system", "content": TASK_PARSER_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": TASK_PARSER_USER_PROMPT.format(task=(task or "").strip()),
            },
        ]
        response_text, reasoning_content = self.vlm.call(messages, max_tokens=self.max_tokens)
        elapsed = time.perf_counter() - started
        self.last_timing = dict(self.vlm.last_call_info)
        self.last_timing["name"] = "task_parse"
        self.last_timing["elapsed"] = self.last_timing.get("elapsed", elapsed)
        return parse_task_parser_response(response_text or reasoning_content)


def parse_task_parser_response(response_text: str) -> List[Dict[str, Any]]:
    """Return normalized stage dictionaries from a task parser response."""
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
        if not instruction or mode not in {"target", "detect", "action"}:
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
    text = re.sub(r"```json\s*", "", response_text or "", flags=re.IGNORECASE)
    text = re.sub(r"```\s*", "", text)
    match = re.search(r"\{.*\}", text, re.DOTALL)
    if not match:
        raise ValueError(f"cannot extract task parser JSON: {text[:200]}")
    return json.loads(match.group(), strict=False)
