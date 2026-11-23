"""数据集加载：对照 3DG-VLN env_uav.py 的 load_my_datasets 复刻 episode 构造。

每个 episode 需要的量与官方 eval 完全一致：
  - 起始位姿 = log/ 排序后第一帧的 position/orientation
  - 目标物体 = mark.target.position 最近 spawn-area 的
      position=[9:12]  quat(x,y,z,w)=[13,14,15,12]  scale=[17]  asset=[16]
  - 指令 = obj_des.json[0] （退化到 meta/instructions.json[uuid]）
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class Episode:
    scene: str
    uuid: str
    instruction: str
    start_position: list[float]          # [x,y,z] NED 世界系
    start_orientation: list[float]       # [x,y,z,w] 四元数
    object_position: list[float]         # [x,y,z] NED（spawn-area [9:12]）
    object_quat_xyzw: list[float]        # [x,y,z,w]（spawn-area [13,14,15,12]）
    object_scale: float                  # spawn-area [17]
    asset_name: str                      # spawn-area [16]
    ori_traj_dir: str                    # 数据集原 episode 目录
    gt_positions: list[list[float]] = field(default_factory=list)


def _find_closest_area(coord, areas):
    """与 metric.py / env_uav.py 完全一致的最近 spawn-area 匹配。"""
    min_distance = float("inf")
    closest = None
    for area in areas:
        if len(area) < 18:
            continue
        true_area = [area[0] + 1.0, area[1] + 1.0, area[2] + 0.5]
        distance = math.dist(coord, true_area)
        if distance < min_distance:
            min_distance = distance
            closest = area
    return closest


def _sorted_logs(log_dir: str) -> list[str]:
    files = [f for f in os.listdir(log_dir)
             if f.lower().endswith(".json") and os.path.isfile(os.path.join(log_dir, f))]
    # 数字帧号按数值排序（0,1,...,10）
    def key(name):
        stem = os.path.splitext(name)[0]
        return (0, int(stem)) if stem.isdigit() else (1, stem)
    files.sort(key=key)
    return files


def _read_instruction(ep_dir: str, uuid: str, meta_instructions: dict) -> str:
    obj_des = os.path.join(ep_dir, "obj_des.json")
    if os.path.isfile(obj_des):
        try:
            with open(obj_des, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, list) and data:
                return str(data[0])
        except Exception:
            pass
    return str(meta_instructions.get(uuid, ""))


def load_episodes(cfg) -> list[Episode]:
    """读取数据集全部 episode（可选按 cfg.scenes 过滤）。"""
    split_root = cfg.split_dir
    meta_dir = cfg.meta_dir

    with open(os.path.join(meta_dir, "map_spawnarea_info.json"), "r", encoding="utf-8") as f:
        map_area = json.load(f)
    meta_instructions = {}
    ins_path = os.path.join(meta_dir, "instructions.json")
    if os.path.isfile(ins_path):
        with open(ins_path, "r", encoding="utf-8") as f:
            meta_instructions = json.load(f)

    scenes = cfg.scenes or sorted(
        d for d in os.listdir(split_root)
        if os.path.isdir(os.path.join(split_root, d)) and d not in ("meta", "tools")
    )

    episodes: list[Episode] = []
    for scene in scenes:
        scene_dir = os.path.join(split_root, scene)
        if not os.path.isdir(scene_dir):
            print(f"[dataset] 跳过不存在的场景: {scene}")
            continue
        if scene not in map_area:
            print(f"[dataset] 跳过无 spawn-area 的场景: {scene}")
            continue
        for uuid in sorted(os.listdir(scene_dir)):
            ep_dir = os.path.join(scene_dir, uuid)
            if not os.path.isdir(ep_dir):
                continue
            mark_path = os.path.join(ep_dir, "mark.json")
            log_dir = os.path.join(ep_dir, "log")
            if not (os.path.isfile(mark_path) and os.path.isdir(log_dir)):
                continue
            logs = _sorted_logs(log_dir)
            if not logs:
                continue
            with open(os.path.join(log_dir, logs[0]), "r", encoding="utf-8") as f:
                st = json.load(f)["sensors"]["state"]
            start_pos = list(st["position"])
            start_ori = list(st["orientation"])
            with open(mark_path, "r", encoding="utf-8") as f:
                mark = json.load(f)
            target = mark["target"]["position"]
            area = _find_closest_area(target, map_area[scene])
            if area is None:
                print(f"[dataset] 跳过无匹配 area: {scene}/{uuid}")
                continue
            ep = Episode(
                scene=scene,
                uuid=uuid,
                instruction=_read_instruction(ep_dir, uuid, meta_instructions),
                start_position=start_pos,
                start_orientation=start_ori,
                object_position=[area[9], area[10], area[11]],
                object_quat_xyzw=[area[13], area[14], area[15], area[12]],
                object_scale=float(area[17]),
                asset_name=str(area[16]),
                ori_traj_dir=os.path.abspath(ep_dir),
            )
            episodes.append(ep)

    remaining = list(episodes)
    if cfg.uuids:
        wanted = set(cfg.uuids)
        remaining = [e for e in remaining if e.uuid in wanted]
    if cfg.limit_episodes and cfg.limit_episodes > 0:
        remaining = remaining[: cfg.limit_episodes]
    return remaining
