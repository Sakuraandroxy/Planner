"""单 episode 回合执行器：reset → spawn → loop(观测→规划→飞→记日志→判成功)。"""
from __future__ import annotations

import math

import numpy as np

from .writer import TrajectoryWriter


def _ne(pos, target) -> float:
    return float(np.linalg.norm(np.asarray(pos, dtype=float) - np.asarray(target, dtype=float)))


def _save_thumbs(ep_dir, rgbs) -> None:
    """把最新一帧 front/down 存成缩略图，供仪表盘轮询显示（覆盖写，只留最新）。"""
    try:
        import os

        import cv2
    except Exception:
        return
    names = [("frame_front.png", 0), ("frame_down.png", 1)]
    for fname, idx in names:
        if idx < len(rgbs) and rgbs[idx] is not None:
            try:
                cv2.imwrite(os.path.join(ep_dir, fname),
                            cv2.cvtColor(rgbs[idx], cv2.COLOR_RGB2BGR))
            except Exception:
                pass


def run_episode(scene, episode, cfg, policy, out_root: str, on_step=None) -> dict:
    """在已拉起的 scene 上跑一个 episode，返回汇总 dict。"""
    if getattr(cfg.planner, "mode", "sanity") == "real":
        from .planner_runtime import run_episode_real

        return run_episode_real(scene, episode, cfg, out_root, on_step=on_step)

    scene.clear_objects()
    scene.set_pose(episode.start_position, episode.start_orientation)
    scene.spawn_object(episode.asset_name, episode.object_position,
                       episode.object_quat_xyzw, episode.object_scale)

    writer = TrajectoryWriter(out_root, episode.uuid, episode.ori_traj_dir)
    target = np.asarray(episode.object_position, dtype=float)

    success = False
    collided = False
    ne = _ne(episode.start_position, target)
    steps_done = 0
    total_len = 0.0
    prev_pos = np.asarray(episode.start_position, dtype=float)

    for step in range(cfg.max_steps):
        steps_done = step + 1
        rgbs = scene.get_rgb()
        _save_thumbs(writer.dir, rgbs)
        state = scene.get_state()
        obs = {
            "position": state["position"],
            "quat": state["orientation"],
            "rgb_front": rgbs[0] if len(rgbs) > 0 else None,
            "rgb_down": rgbs[1] if len(rgbs) > 1 else None,
            "instruction": episode.instruction,
            "target_object_position": episode.object_position,
        }
        world_pts = policy.plan(obs)
        visited, final_state, collided = scene.fly(world_pts)

        for p in visited:
            writer.write_frame({"position": p, "orientation": final_state["orientation"],
                                "linear_velocity": [0.0, 0.0, 0.0]},
                               collision=False, extra={"step": step})
            total_len += _ne(p, prev_pos)
            prev_pos = np.asarray(p, dtype=float)

        ne = _ne(final_state["position"], target)
        if on_step is not None:
            on_step(step=step, ne=ne, collided=collided)

        if collided:
            break
        # 末帧进入成功球 → 本 chunk 成功（对齐参考：末距离≤阈值且无碰撞）
        if ne <= cfg.success_radius_m:
            success = True
            break

    summary = {
        "uuid": episode.uuid,
        "scene": episode.scene,
        "instruction": episode.instruction,
        "success": bool(success and not collided),
        "collided": bool(collided),
        "final_ne": ne,
        "steps": steps_done,
        "path_length": total_len,
        "target_position": episode.object_position,
    }
    out_dir = writer.finalize(summary["success"], summary)
    summary["dir"] = out_dir
    return summary
