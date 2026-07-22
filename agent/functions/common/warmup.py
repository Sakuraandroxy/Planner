"""API warmup helpers driven by config."""

from __future__ import annotations

import time as _time
from urllib.parse import urlparse, urlunparse

import requests
from openai import OpenAI


def _warmup_one(name: str, base_url: str, api_key: str, model: str):
    started = _time.perf_counter()
    try:
        client = OpenAI(base_url=base_url, api_key=api_key or "no-key")
        client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": "OK"}],
            max_tokens=2,
            temperature=0.0,
            extra_body={"thinking": {"type": "disabled"}},
        )
        elapsed = _time.perf_counter() - started
        print(f"  [Warmup] {name} ready in {elapsed:.2f}s  ({base_url} / {model})")
    except Exception as exc:
        elapsed = _time.perf_counter() - started
        print(f"  [Warmup] {name} skipped ({elapsed:.2f}s): {exc}")


def _replace_path(url: str, new_path: str) -> str:
    parts = urlparse(url)
    return urlunparse((parts.scheme, parts.netloc, new_path, "", "", ""))


def _warmup_http_health(name: str, url: str):
    started = _time.perf_counter()
    try:
        resp = requests.get(url, timeout=5)
        resp.raise_for_status()
        elapsed = _time.perf_counter() - started
        print(f"  [Warmup] {name} ready in {elapsed:.2f}s  ({url})")
    except Exception as exc:
        elapsed = _time.perf_counter() - started
        print(f"  [Warmup] {name} skipped ({elapsed:.2f}s): {exc}")


def _warmup_http_reachable(name: str, url: str, method: str = "GET", json_body=None):
    started = _time.perf_counter()
    try:
        method = (method or "GET").upper()
        if method == "POST":
            resp = requests.post(url, json=json_body, timeout=5)
        else:
            resp = requests.get(url, timeout=5)
        elapsed = _time.perf_counter() - started
        print(f"  [Warmup] {name} reachable in {elapsed:.2f}s  ({url} status={resp.status_code})")
    except Exception as exc:
        elapsed = _time.perf_counter() - started
        print(f"  [Warmup] {name} skipped ({elapsed:.2f}s): {exc}")


def warmup_from_config():
    """Warm up configured model/API endpoints once per unique endpoint."""
    from config import cfg
    from agent.functions.common.config_access import first_value, function_section

    print("\n" + "=" * 55)
    print("  API Warmup")
    print("=" * 55)

    ag = cfg.get("AGENT", {}) or {}
    task_cfg = function_section(cfg, "TASK_PARSER")
    perception_cfg = function_section(cfg, "PERCEPTION")
    planning_cfg = function_section(cfg, "PLANNING")
    sw = planning_cfg
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

    _warmup_if_new(
        "task_parser",
        first_value(task_cfg.get("URL"), ag.get("TASK_PARSER_URL"), default=""),
        first_value(task_cfg.get("API_KEY"), ag.get("TASK_API_KEY"), default=""),
        first_value(task_cfg.get("MODEL_NAME"), ag.get("TASK_PARSER_MODEL"), default=""),
    )

    detector_name = str(first_value(perception_cfg.get("MODEL"), ag.get("DETECTOR"), default="")).strip().lower()
    if detector_name == "groundingdino":
        det_url = first_value(perception_cfg.get("URL"), ag.get("GROUNDINGDINO_URL"), default="")
        if det_url:
            _warmup_http_reachable("detector", det_url, method="POST", json_body={})
        else:
            print("  [Warmup] detector skipped (groundingdino url missing)")
    elif detector_name in {"none", "noop"}:
        print(f"  [Warmup] detector skipped ({detector_name})")
    else:
        _warmup_if_new(
            "detector",
            first_value(perception_cfg.get("URL"), ag.get("DETECTOR_URL"), ag.get("PLANNER_URL"), default=""),
            first_value(perception_cfg.get("API_KEY"), ag.get("DETECTOR_API_KEY"), ag.get("PLANNER_API_KEY"), default=""),
            first_value(perception_cfg.get("MODEL_NAME"), ag.get("DETECTOR_MODEL"), ag.get("PLANNER_MODEL"), default=""),
        )

    def _sliding_window_url():
        explicit = str(first_value(planning_cfg.get("URL"), sw.get("PLANNER_URL"), default="")).strip()
        if explicit:
            return explicit
        host = str(first_value(planning_cfg.get("SERVER_IP"), sw.get("SERVER_IP"), sw.get("HOST"), default="")).strip()
        port = str(first_value(planning_cfg.get("SERVER_PORT"), sw.get("SERVER_PORT"), sw.get("PORT"), default="")).strip()
        path = str(first_value(planning_cfg.get("CHAT_PATH"), sw.get("CHAT_PATH"), default="/v1/chat/completions")).strip() or "/v1/chat/completions"
        if not host:
            return ag.get("PLANNER_URL", "")
        base = host.rstrip("/")
        if not base.startswith(("http://", "https://")):
            base = f"http://{base}"
        if port and ":" not in base.rsplit("/", 1)[-1]:
            base = f"{base}:{port}"
        return f"{base}{path if path.startswith('/') else '/' + path}"

    planner_name = str(first_value(planning_cfg.get("MODEL"), ag.get("PLANNER"), default="")).strip().lower()
    if planner_name == "qwen_planner":
        planner_url = first_value(planning_cfg.get("URL"), ag.get("PLANNER_URL"), default="")
        if planner_url:
            _warmup_http_health("planner", _replace_path(planner_url, "/health"))
    elif planner_name in {"qwen_sliding_window_planner", "sliding_window", "qwen_incremental"}:
        _warmup_if_new(
            "planner",
            _sliding_window_url(),
            first_value(planning_cfg.get("API_KEY"), sw.get("PLANNER_API_KEY"), ag.get("PLANNER_API_KEY"), default=""),
            first_value(planning_cfg.get("MODEL_NAME"), sw.get("PLANNER_MODEL"), ag.get("PLANNER_MODEL"), default=""),
        )
    else:
        _warmup_if_new(
            "planner",
            first_value(planning_cfg.get("URL"), ag.get("PLANNER_URL"), default=""),
            first_value(planning_cfg.get("API_KEY"), ag.get("PLANNER_API_KEY"), default=""),
            first_value(planning_cfg.get("MODEL_NAME"), ag.get("PLANNER_MODEL"), default=""),
        )

    print("=" * 55)
    print()
