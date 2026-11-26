"""大模型评语：把评测或对比结果交给模型，产出一段约 500 字的纯文本报告。

模型端点取 ``config/eval.yml`` 的 ``ai_report`` 段，字段与 ``config/core/*/base.yaml``
的模型段一致，密钥默认读环境变量（与规划器共用 ``VLN_API_KEY``）。

落盘位置：
- 单批次：``<run>/metrics.json`` 的 ``ai_report`` 字段，跟该批次的评测记录同处一个文件；
- 对比：``output_dir/_ai_reports.json`` 的索引，键为「场景::排序后的批次名」。
"""
from __future__ import annotations

import glob
import json
import os
import sys
import tempfile
import time

from .config import REPO_ROOT

INDEX_FILE = "_ai_reports.json"

_SYSTEM = (
    "你是无人机视觉语言导航（UAV-VLN）评测报告的撰写者，只输出中文纯文本。"
    "不写标题、不写编号列表、不用 Markdown 标记、不排版成表格。"
)

_INSTRUCTIONS = """请基于以上数据写一段约 {words} 字的中文评述，放在评测报告末尾。要求：
- 纯文本，一段到底，不要标题、编号、项目符号、Markdown 标记；
- 提到整体表现、成功率与可达率的差距、末端距离反映的定位精度、路径效率、失败与碰撞的集中点；
- 只使用上面给出的数据，不要编造数字、场景名或者不存在的对比；
- 语气客观，最后给出可执行的改进方向，不要断言系统已经足够好。"""


def generate(cfg, kind: str, payload: dict) -> dict:
    """调模型生成评语，返回 ``{text, model, created}``，不落盘。"""
    client, timeout_s = _client(cfg)
    prompt = (
        _run_prompt(cfg, payload)
        if kind == "run"
        else _compare_prompt(cfg, payload)
    )
    text = client.request(
        _SYSTEM, prompt, (), timeout_s=timeout_s, operation="AI 评语"
    )
    text = " ".join(str(text or "").split())
    if not text:
        raise RuntimeError("模型返回空文本")
    return {"text": text, "model": client.model, "created": _now()}


def save(cfg, kind: str, payload: dict, text: str, model: str = "") -> dict:
    """把评语写进已有记录：单批次进 metrics.json，对比进索引文件。"""
    record = {"text": text, "model": model, "created": _now()}
    if kind == "run":
        path = os.path.join(_run_dir(cfg, payload.get("run")), "metrics.json")
        data = _read_json(path, None)
        if not isinstance(data, dict) or not data:
            raise RuntimeError(f"批次记录不存在: {path}")
        data["ai_report"] = record
        _write_json(path, data)
        return record
    path = os.path.join(cfg.output_dir, INDEX_FILE)
    data = _read_json(path, {})
    if not isinstance(data, dict):
        data = {}
    data[_compare_key(payload)] = {
        **record,
        "scene": str(payload.get("scene") or ""),
        "runs": sorted(str(x) for x in payload.get("runs") or []),
    }
    _write_json(path, data)
    return record


def load(cfg, kind: str, payload: dict) -> dict | None:
    """读已保存的评语，没有就返回 None。"""
    if kind == "run":
        report = _read_json(
            os.path.join(_run_dir(cfg, payload.get("run")), "metrics.json"), {}
        )
        record = report.get("ai_report") if isinstance(report, dict) else None
    else:
        index = _read_json(os.path.join(cfg.output_dir, INDEX_FILE), {})
        record = index.get(_compare_key(payload)) if isinstance(index, dict) else None
    if isinstance(record, dict) and str(record.get("text") or "").strip():
        return record
    return None


# ---------- 模型端点 ----------

def _client(cfg):
    settings = cfg.ai_report
    url = str(settings.url or "").strip()
    model = str(settings.model or "").strip()
    if not url or not model:
        raise RuntimeError("config/eval.yml 的 ai_report 段缺少 url 或 model")
    api_key = str(settings.api_key or "").strip()
    if not api_key:
        env_name = str(settings.api_key_env or "VLN_API_KEY").strip()
        api_key = os.environ.get(env_name, "").strip()
        if not api_key:
            raise RuntimeError(
                f"缺少模型密钥：设置环境变量 {env_name}，或填 ai_report.api_key"
            )
    root = str(REPO_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)
    from planner.adapters.multimodal_chat import MultimodalChatClient

    timeout_s = float(settings.timeout_s)
    return MultimodalChatClient(
        url,
        model,
        api_key,
        timeout_s,
        max_completion_tokens=int(settings.max_completion_tokens),
        reasoning_effort=settings.reasoning_effort,
        request_format=settings.request_format,
    ), timeout_s


# ---------- 提示词 ----------

def _words(cfg) -> int:
    try:
        return max(120, int(cfg.ai_report.words))
    except (TypeError, ValueError):
        return 500


def _run_prompt(cfg, payload: dict) -> str:
    run = str(payload.get("run") or "").strip()
    report = _read_json(os.path.join(_run_dir(cfg, run), "metrics.json"), {})
    if not isinstance(report, dict) or not report:
        raise RuntimeError(f"批次还没有评测结果: {run}")
    facts = [_run_facts(run, report)]
    return "\n".join(facts) + "\n\n" + _INSTRUCTIONS.format(words=_words(cfg))


def _compare_prompt(cfg, payload: dict) -> str:
    scene = str(payload.get("scene") or "").strip()
    runs = [str(x) for x in payload.get("runs") or []]
    if len(runs) < 2:
        raise RuntimeError("对比报告至少要两个批次")
    facts = [f"评测场景：{scene}", f"参与对比的批次：{len(runs)} 个"]
    episodes = {}
    for run in runs:
        report = _read_json(os.path.join(_run_dir(cfg, run), "metrics.json"), {})
        if not isinstance(report, dict) or not report:
            raise RuntimeError(f"批次还没有评测结果: {run}")
        facts.append("")
        facts.append(_run_facts(run, report))
        episodes[run] = {ep.get("uuid"): ep for ep in report.get("episodes") or []}
    facts.append("")
    facts.append("逐用例结果（同一行是同一个用例在各批次下的判定与末端距离）：")
    uuids = []
    for run in runs:
        for uuid in episodes[run]:
            if uuid and uuid not in uuids:
                uuids.append(uuid)
    for uuid in uuids[:40]:
        cells = []
        for run in runs:
            ep = episodes[run].get(uuid)
            if not ep:
                cells.append("—")
                continue
            status = "成功" if ep.get("success") else ("碰撞" if ep.get("collided") else "未完成")
            ne = ep.get("final_ne")
            dist = f"{float(ne):.2f}m" if isinstance(ne, (int, float)) else "—"
            cells.append(f"{status} {dist}")
        facts.append(f"{uuid}: " + " | ".join(cells))
    if len(uuids) > 40:
        facts.append(f"（另有 {len(uuids) - 40} 个用例未列出）")
    return "\n".join(facts) + "\n\n" + _INSTRUCTIONS.format(words=_words(cfg))


def _run_facts(run: str, report: dict) -> str:
    episodes = report.get("episodes") or []
    metrics = report.get("metrics") or {}
    dataset = report.get("dataset") or {}
    scenes = dataset.get("scenes")
    if isinstance(scenes, list):
        scenes = "、".join(str(x) for x in scenes)
    total = len(episodes)
    success = sum(1 for ep in episodes if ep.get("success"))
    collided = sum(1 for ep in episodes if ep.get("collided"))
    distances = [
        float(ep["final_ne"])
        for ep in episodes
        if isinstance(ep.get("final_ne"), (int, float))
    ]
    lines = [
        f"批次：{run}",
        f"评测时间：{report.get('time') or '未知'}",
        f"规划配置：{report.get('planner') or '未知'}",
        f"数据集切分：{dataset.get('split') or '未知'}；场景：{scenes or '未知'}；"
        f"成功半径：{dataset.get('success_radius_m') or '未知'} 米",
        f"用例 {total} 个，成功 {success} 个，碰撞 {collided} 个",
        f"SR {_num(metrics.get('sr'))}%，OSR {_num(metrics.get('osr'))}%，"
        f"NE 均值 {_num(metrics.get('ne_mean'))} 米，SPL {_num(metrics.get('spl'))}%",
    ]
    if distances:
        lines.append(
            f"末端距离范围 {min(distances):.2f} ~ {max(distances):.2f} 米"
        )
    failed = [ep for ep in episodes if not ep.get("success")]
    for ep in failed[:8]:
        reason = "碰撞" if ep.get("collided") else (ep.get("error") or "未到达")
        ne = ep.get("final_ne")
        dist = f"{float(ne):.2f} 米" if isinstance(ne, (int, float)) else "未知"
        lines.append(
            f"未成功用例 {ep.get('uuid') or '?'}：{reason}，末端距离 {dist}，"
            f"步数 {ep.get('steps') if ep.get('steps') is not None else '未知'}"
        )
    if len(failed) > 8:
        lines.append(f"（另有 {len(failed) - 8} 个未成功用例未列出）")
    return "\n".join(lines)


def _num(value) -> str:
    if isinstance(value, (int, float)):
        return f"{float(value):.2f}"
    return "未知"


# ---------- 文件 ----------

def _run_dir(cfg, name) -> str:
    name = str(name or "").strip()
    if not name:
        raise RuntimeError("缺少批次名")
    for path in glob.glob(os.path.join(cfg.output_dir, "run_*")):
        if os.path.isdir(path) and os.path.basename(path) == name:
            return path
    raise RuntimeError(f"批次目录不存在: {name}")


def _compare_key(payload: dict) -> str:
    scene = str(payload.get("scene") or "").strip()
    runs = ",".join(sorted(str(x) for x in payload.get("runs") or []))
    return f"{scene}::{runs}"


def _read_json(path: str, default):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def _write_json(path: str, data) -> None:
    """原子写：同目录临时文件 + os.replace，避免轮询读到半个文件。"""
    directory = os.path.dirname(path) or "."
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=".ai_report-", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp, path)
    except Exception:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def _now() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")