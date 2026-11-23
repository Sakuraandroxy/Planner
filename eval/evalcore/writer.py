"""磁盘契约写入器：产出 metric.py 能直接吃的目录结构。

    <out>/<success_?><uuid>/
        log/000000.json ...        # {"frame": idx, "sensors": {"state": {"position": [...], "orientation": [...]}}}
        ori_info.json              # {"ori_traj_dir": "<数据集 episode 绝对路径>"}
        result.json                # 我方附加元数据（metric.py 忽略）

成功时目录名前缀 success_，与官方 check_batch_termination 一致。
"""
from __future__ import annotations

import json
import os
import shutil


class TrajectoryWriter:
    def __init__(self, out_root: str, uuid: str, ori_traj_dir: str):
        self.out_root = out_root
        self.uuid = uuid
        self.ori_traj_dir = ori_traj_dir
        # 先写到无前缀目录，finalize 时按成败改名
        self.dir = os.path.join(out_root, uuid)
        self.log_dir = os.path.join(self.dir, "log")
        if os.path.isdir(self.dir):
            shutil.rmtree(self.dir)
        os.makedirs(self.log_dir, exist_ok=True)
        self._idx = 0

    def write_frame(self, state: dict, collision: bool = False, extra: dict | None = None) -> None:
        rec = {
            "frame": self._idx,
            "sensors": {
                "state": {
                    "position": list(state["position"]),
                    "orientation": list(state["orientation"]),
                    "linear_velocity": list(state.get("linear_velocity", [0.0, 0.0, 0.0])),
                },
                "collision": {"has_collided": bool(collision)},
            },
        }
        if extra:
            rec["meta"] = extra
        path = os.path.join(self.log_dir, f"{self._idx:06d}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(rec, f)
        self._idx += 1

    def finalize(self, success: bool, summary: dict) -> str:
        with open(os.path.join(self.dir, "ori_info.json"), "w", encoding="utf-8") as f:
            json.dump({"ori_traj_dir": self.ori_traj_dir}, f)
        with open(os.path.join(self.dir, "result.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        if success:
            new_dir = os.path.join(self.out_root, f"success_{self.uuid}")
            if os.path.isdir(new_dir):
                shutil.rmtree(new_dir)
            os.rename(self.dir, new_dir)
            self.dir = new_dir
        return self.dir
