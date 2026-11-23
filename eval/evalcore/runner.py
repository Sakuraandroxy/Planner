"""单 episode 回合执行器。

评测没有独立规划器：每个 episode 都交给 :func:`evalcore.planner_runtime.run_episode_real`，
在进程内复用 Planner 的 visual-goal 工作流（真检测+真VLM规划+完成判定+滑窗）。
"""
from __future__ import annotations


def run_episode(scene, episode, cfg, out_root: str, on_step=None) -> dict:
    """在已拉起的 scene 上跑一个 episode，返回汇总 dict。"""
    from .planner_runtime import run_episode_real

    return run_episode_real(scene, episode, cfg, out_root, on_step=on_step)
