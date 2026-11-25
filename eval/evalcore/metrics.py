"""打分：直接调用官方 metric.py，并解析其 stdout。

官方口径（见 3DG-VLN-main/utils/metric.py）：
  SR  = 目录名以 success_ 前缀的 episode 比例
  OSR = 轨迹任一点 ≤ success_radius 的比例
  NE  = 末帧距离均值（剔除 > max_ne_distance）
  SPL = S·L*/max(L*,P)，L* 用专家 log 路径长 − success_radius
"""
from __future__ import annotations

import os
import re
import subprocess
import sys


def run_metric(cfg, eval_save_path: str) -> dict:
    """调官方 metric.py，返回 {'ne_mean','osr','sr','spl','num'}。"""
    ds = cfg.dataset_root
    cmd = [
        sys.executable, cfg.metric_py,
        "--eval_save_path", eval_save_path,
        "--eval_test_path", os.path.join(ds, "test"),
        "--eval_unscene_path", os.path.join(ds, "unscene"),
        "--eval_unobject_path", os.path.join(ds, "unobject"),
        "--object_info_path", cfg.object_info_path,
        "--success_radius", str(cfg.success_radius_m),
        "--max_ne_distance", str(cfg.max_ne_distance_m),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    print(proc.stdout)
    if proc.returncode != 0:
        print(proc.stderr, file=sys.stderr)
        raise RuntimeError(f"metric.py 退出码 {proc.returncode}")
    return parse_metric_stdout(proc.stdout)


_NUM = r"([-+]?\d+(?:\.\d+)?)"


def parse_metric_stdout(text: str) -> dict:
    def grab(key):
        m = re.search(rf"^{key}:\s*{_NUM}", text, re.MULTILINE)
        return float(m.group(1)) if m else None

    num = re.search(r"^num_trajectories:\s*(\d+)", text, re.MULTILINE)
    return {
        "num": int(num.group(1)) if num else None,
        "ne_mean": grab("ne_mean"),
        "osr": grab("oracle_success_rate"),
        "sr": grab("success_rate"),
        "spl": grab("avg_spl"),
    }
