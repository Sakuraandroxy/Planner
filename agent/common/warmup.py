"""API 预热工具 —— 读 config yaml，遍历所有 OpenAI 端点发送 warmup 请求。

不依赖任何业务模块。入口只需 import 并调用 warmup_from_config()。
新增端点只需在 yaml 加配置，不需要写任何代码。
"""

import time as _time
from urllib.parse import urlparse, urlunparse

import requests
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


def _replace_path(url: str, new_path: str) -> str:
    parts = urlparse(url)
    return urlunparse((parts.scheme, parts.netloc, new_path, "", "", ""))


def _warmup_http_health(name: str, url: str):
    _t0 = _time.perf_counter()
    try:
        resp = requests.get(url, timeout=5)
        resp.raise_for_status()
        _elapsed = _time.perf_counter() - _t0
        print(f"  [Warmup] {name} ready in {_elapsed:.2f}s  ({url})")
    except Exception as exc:
        _elapsed = _time.perf_counter() - _t0
        print(f"  [Warmup] {name} skipped ({_elapsed:.2f}s): {exc}")


def _warmup_http_reachable(name: str, url: str, method: str = "GET", json_body=None):
    _t0 = _time.perf_counter()
    try:
        method = (method or "GET").upper()
        if method == "POST":
            resp = requests.post(url, json=json_body, timeout=5)
        else:
            resp = requests.get(url, timeout=5)
        _elapsed = _time.perf_counter() - _t0
        print(f"  [Warmup] {name} reachable in {_elapsed:.2f}s  ({url} status={resp.status_code})")
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

    # Detector:
    # - groundingdino 走自定义 /detect 接口，做轻量 reachability 检查
    # - vlm_detector 才走 OpenAI 兼容端点
    detector_name = str(ag.get("DETECTOR", "")).strip().lower()
    if detector_name == "groundingdino":
        det_url = ag.get("GROUNDINGDINO_URL", "")
        if det_url:
            _warmup_http_reachable("detector", det_url, method="POST", json_body={})
        else:
            print("  [Warmup] detector skipped (groundingdino url missing)")
    elif detector_name in {"none", "noop"}:
        print(f"  [Warmup] detector skipped ({detector_name})")
    else:
        det_url = ag.get("DETECTOR_URL") or ag.get("PLANNER_URL", "")
        det_model = ag.get("DETECTOR_MODEL") or ag.get("PLANNER_MODEL", "")
        det_key = ag.get("DETECTOR_API_KEY") or ag.get("PLANNER_API_KEY", "")
        _warmup_if_new("detector", det_url, det_key, det_model)

    # Planner:
    # - qwen_planner 走自定义 /plan 接口，对应服务用 /health 探活
    # - 其他规划器走 OpenAI 兼容 warmup
    planner_name = str(ag.get("PLANNER", "")).strip().lower()
    if planner_name == "qwen_planner":
        planner_url = ag.get("PLANNER_URL", "")
        if planner_url:
            _warmup_http_health("planner", _replace_path(planner_url, "/health"))
    else:
        _warmup_if_new(
            "planner",
            ag.get("PLANNER_URL", ""),
            ag.get("PLANNER_API_KEY", ""),
            ag.get("PLANNER_MODEL", ""),
        )

    print("=" * 55)
    print()
