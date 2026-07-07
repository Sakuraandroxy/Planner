"""API 预热工具 —— 读 config yaml，遍历所有 OpenAI 端点发送 warmup 请求。

不依赖任何业务模块。入口只需 import 并调用 warmup_from_config()。
新增端点只需在 yaml 加配置，不需要写任何代码。
"""

import time as _time
from openai import OpenAI


def _warmup_one(name: str, base_url: str, api_key: str, model: str):
    """向一个 OpenAI 兼容端点发送极短 warmup 请求。"""
    _t0 = _time.perf_counter()
    try:
        client = OpenAI(base_url=base_url, api_key=api_key or "no-key")
        client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": "OK"}],
            max_tokens=2,
            temperature=0.0,
            extra_body={"thinking": {"type": "disabled"}},
        )
        _elapsed = _time.perf_counter() - _t0
        print(f"  [Warmup] {name} ready in {_elapsed:.2f}s  ({base_url} / {model})")
    except Exception as exc:
        _elapsed = _time.perf_counter() - _t0
        print(f"  [Warmup] {name} skipped ({_elapsed:.2f}s): {exc}")


def warmup_from_config():
    """读 config/default.yaml，遍历所有 API 端点并预热。

    自动去重：相同 (base_url, model) 的端点只预热一次。
    """
    from config import cfg

    print("\n" + "=" * 55)
    print("  API Warmup")
    print("=" * 55)

    ag = cfg["AGENT"]
    seen = set()

    def _warmup_if_new(name, url, key, model):
        if not url or not model:
            return
        dedup_key = (url, model)
        if dedup_key in seen:
            print(f"  [Warmup] {name} skipped (same endpoint as previous)")
            return
        seen.add(dedup_key)
        _warmup_one(name, url, key, model)

    # Task Parser
    _warmup_if_new(
        "task_parser",
        ag.get("TASK_PARSER_URL", ""),
        ag.get("TASK_API_KEY", ""),
        ag.get("TASK_PARSER_MODEL", ""),
    )

    # Detector（DETECTOR_URL 为空则复用 PLANNER_URL）
    det_url = ag.get("DETECTOR_URL") or ag.get("PLANNER_URL", "")
    det_model = ag.get("DETECTOR_MODEL") or ag.get("PLANNER_MODEL", "")
    det_key = ag.get("DETECTOR_API_KEY") or ag.get("PLANNER_API_KEY", "")
    _warmup_if_new("detector", det_url, det_key, det_model)

    # Planner
    _warmup_if_new(
        "planner",
        ag.get("PLANNER_URL", ""),
        ag.get("PLANNER_API_KEY", ""),
        ag.get("PLANNER_MODEL", ""),
    )

    print("=" * 55)
    print()
