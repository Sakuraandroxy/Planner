"""结果落盘：per-episode CSV + 聚合 metrics.json + 供仪表盘轮询的 progress.json。"""
from __future__ import annotations

import csv
import json
import os
import time

_CSV_FIELDS = ["uuid", "scene", "success", "collided", "final_ne", "steps", "path_length", "error"]


def write_reports(run_dir: str, results: list[dict], metrics: dict | None, cfg) -> None:
    # per-episode CSV
    csv_path = os.path.join(run_dir, "per_episode.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=_CSV_FIELDS, extrasaction="ignore")
        w.writeheader()
        for r in results:
            row = {k: r.get(k) for k in _CSV_FIELDS}
            if row["path_length"] is not None:
                row["path_length"] = round(float(row["path_length"]), 3)
            if row["final_ne"] is not None:
                row["final_ne"] = round(float(row["final_ne"]), 3)
            w.writerow(row)

    # 聚合指标
    summary = {
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "mode": None,
        "num_episodes": len(results),
        "num_success": sum(1 for r in results if r.get("success")),
        "num_collided": sum(1 for r in results if r.get("collided")),
        "metrics": metrics,
        "dataset": {
            "split": cfg.split,
            "scenes": cfg.scenes or "all",
            "success_radius_m": cfg.success_radius_m,
        },
        "planner": cfg.planner.profile,
        "episodes": [
            {k: r.get(k) for k in _CSV_FIELDS} for r in results
        ],
    }
    with open(os.path.join(run_dir, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(f"[eval] 报告已写入: {csv_path} 和 metrics.json")


def write_progress(run_dir: str, *, total: int, done: int, current: str | None,
                   metrics: dict | None = None, state: str = "running") -> None:
    """仪表盘轮询用的轻量状态文件。"""
    data = {
        "state": state,
        "total": total,
        "done": done,
        "current": current,
        "metrics": metrics,
        "updated": time.time(),
    }
    tmp = os.path.join(run_dir, "progress.json.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)
    os.replace(tmp, os.path.join(run_dir, "progress.json"))
