"""AI 评语：参数归一、落盘位置、提示词内容、缺密钥的报错。"""
import json
from types import SimpleNamespace

import pytest

from eval.evalcore import ai_report, dashboard


def _cfg(tmp_path):
    output_dir = tmp_path / "runs"
    run_dir = output_dir / "run_20250101_120000"
    run_dir.mkdir(parents=True)
    (run_dir / "metrics.json").write_text(
        json.dumps(
            {
                "time": "2025-01-01 12:00:00",
                "planner": "default",
                "dataset": {"split": "test", "scenes": ["Carla_Town15"], "success_radius_m": 5.0},
                "metrics": {"sr": 50.0, "osr": 75.0, "ne_mean": 6.5, "spl": 40.0},
                "episodes": [
                    {"uuid": "u000", "success": True, "collided": False, "final_ne": 3.0, "steps": 60},
                    {"uuid": "u001", "success": False, "collided": True, "final_ne": 11.5, "steps": 60},
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return SimpleNamespace(
        output_dir=str(output_dir),
        ai_report=SimpleNamespace(
            url="http://127.0.0.1:8099",
            model="fake-model",
            api_key="",
            api_key_env="VLN_API_TEST_KEY",
            timeout_s=5.0,
            max_completion_tokens=256,
            reasoning_effort="none",
            request_format="anthropic",
            words=500,
        ),
    )


def test_ai_payload_normalizes_query_string_and_json_body():
    query = {"kind": ["compare"], "scene": ["Carla_Town15"], "runs": ["run_a,run_b"]}
    assert dashboard._ai_payload(query) == {
        "kind": "compare",
        "scene": "Carla_Town15",
        "runs": ["run_a", "run_b"],
    }
    body = {"kind": "compare", "scene": "Carla_Town15", "runs": ["run_a", "run_b"]}
    assert dashboard._ai_payload(body)["runs"] == ["run_a", "run_b"]
    assert dashboard._ai_payload({"kind": "run", "run": "run_x"}) == {
        "kind": "run",
        "run": "run_x",
    }


def test_run_report_is_saved_into_metrics_json_and_read_back(tmp_path):
    cfg = _cfg(tmp_path)
    payload = {"kind": "run", "run": "run_20250101_120000"}
    assert ai_report.load(cfg, "run", payload) is None

    ai_report.save(cfg, "run", payload, "整批评测表现尚可。", "fake-model")
    record = ai_report.load(cfg, "run", payload)
    assert record["text"] == "整批评测表现尚可。"
    assert record["model"] == "fake-model"

    stored = json.loads((tmp_path / "runs/run_20250101_120000/metrics.json").read_text("utf-8"))
    assert stored["ai_report"]["text"] == "整批评测表现尚可。"
    assert stored["metrics"]["sr"] == 50.0          # 原字段不被覆盖


def test_compare_report_is_keyed_by_scene_and_sorted_runs(tmp_path):
    cfg = _cfg(tmp_path)
    payload = {"kind": "compare", "scene": "Carla_Town15", "runs": ["run_a", "run_b"]}
    ai_report.save(cfg, "compare", payload, "两种配置差异集中在末端阶段。", "fake-model")

    index = json.loads((tmp_path / "runs/_ai_reports.json").read_text("utf-8"))
    assert list(index) == ["Carla_Town15::run_a,run_b"]
    assert index["Carla_Town15::run_a,run_b"]["runs"] == ["run_a", "run_b"]
    assert ai_report.load(cfg, "compare", payload)["text"] == "两种配置差异集中在末端阶段。"


def test_run_summary_reports_saved_ai_text(tmp_path):
    cfg = _cfg(tmp_path)
    run_path = str(tmp_path / "runs/run_20250101_120000")
    assert dashboard._run_summary(run_path)["has_ai_report"] is False
    ai_report.save(cfg, "run", {"kind": "run", "run": "run_20250101_120000"}, "已生成。")
    assert dashboard._run_summary(run_path)["has_ai_report"] is True


def test_prompt_carries_metrics_and_failed_cases(tmp_path):
    cfg = _cfg(tmp_path)
    prompt = ai_report._run_prompt(cfg, {"kind": "run", "run": "run_20250101_120000"})
    assert "SR 50.00%" in prompt
    assert "u001" in prompt
    assert "约 500 字" in prompt


def test_missing_api_key_names_the_environment_variable(tmp_path, monkeypatch):
    cfg = _cfg(tmp_path)
    monkeypatch.delenv("VLN_API_TEST_KEY", raising=False)
    with pytest.raises(RuntimeError) as excinfo:
        ai_report._client(cfg)
    assert "VLN_API_TEST_KEY" in str(excinfo.value)