"""评测子系统 CLI 入口。

用法（在 vln-ue4 环境）：
    python run.py one   --scenes TropicalIsland --limit-episodes 1
    python run.py batch --scenes TropicalIsland,ModularPark
    python run.py serve                      # 启动评测仪表盘
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from evalcore.config import parse_args          # noqa: E402
from evalcore.dataset import load_episodes      # noqa: E402
from evalcore.metrics import run_metric         # noqa: E402
from evalcore.report import write_progress, write_reports  # noqa: E402
from evalcore.runner import run_episode         # noqa: E402


def _group_by_scene(episodes):
    groups: dict[str, list] = {}
    for ep in episodes:
        groups.setdefault(ep.scene, []).append(ep)
    return groups


def _new_run_dir(cfg) -> str:
    ts = time.strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(cfg.output_dir, f"run_{ts}")
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


def run_batch(cfg, episodes, run_dir, gpu_for=None, total=None) -> list[dict]:
    from evalcore.sim import open_scene

    results = []
    total = total or len(episodes)
    write_progress(run_dir, total=total, done=0, current=None, state="running")
    for scene_id, eps in _group_by_scene(episodes).items():
        gpu = gpu_for(scene_id) if gpu_for else cfg.gpus[0]
        print(f"\n[eval] ===== scene={scene_id} episodes={len(eps)} gpu={gpu} "
              f"profile={cfg.planner.profile} =====")
        with open_scene(scene_id, cfg, gpu=gpu) as scene:
            for i, ep in enumerate(eps, 1):
                print(f"[eval] ({i}/{len(eps)}) {ep.uuid}")
                write_progress(run_dir, total=total, done=len(results),
                               current=ep.uuid, state="running")
                try:
                    r = run_episode(scene, ep, cfg, run_dir,
                                    on_step=lambda **kw: print(
                                        f"    step={kw['step']} ne={kw['ne']:.2f} "
                                        f"coll={kw['collided']}"))
                except Exception as exc:  # 单集失败不影响整批
                    print(f"[eval] episode {ep.uuid} 失败: {exc}")
                    r = {"uuid": ep.uuid, "scene": scene_id, "error": str(exc),
                         "success": False, "final_ne": float("nan")}
                results.append(r)
        # 关闭这张图的真实规划器栈（会话持有连接）；要在进程被 kill 前做。
        from evalcore.planner_runtime import close_scene
        close_scene(scene)
    return results


def main(argv=None) -> int:
    args = parse_args(argv)
    cfg, mode = args.cfg, args.mode

    if mode == "serve":
        from evalcore.dashboard import serve
        serve(cfg, host=cfg.dashboard_host,
              parent_pipe=sys.stdin.buffer if os.environ.get("AVLN_EVAL_PARENT_PIPE") == "1" else None)
        return 0

    episodes = load_episodes(cfg)
    if not episodes:
        print("[eval] 没有可跑 episode，检查 --scenes / --dataset-root / --split")
        return 1

    run_dir = _new_run_dir(cfg)
    total = len(episodes)
    print(f"[eval] mode={mode} episodes={total} out={run_dir}")

    if mode == "one":
        episodes = episodes[:1]
        # 单集时让 eval_save_path 只含这一集，metric.py 才会只算这一条
        results = run_batch(cfg, episodes, run_dir, total=1)
    else:
        results = run_batch(cfg, episodes, run_dir, total=total)

    ok = sum(1 for r in results if r.get("success"))
    print(f"\n[eval] 完成 {len(results)} 集，success={ok}")
    for r in results:
        print(f"    {r['uuid'][:8]} scene={r.get('scene')} "
              f"success={r.get('success')} ne={r.get('final_ne')}")

    print("\n[eval] ===== 官方 metric.py 打分 =====")
    metrics = None
    try:
        metrics = run_metric(cfg, run_dir)
        print(f"[eval] SR={metrics['sr']}%  OSR={metrics['osr']}%  NE={metrics['ne_mean']}m  "
              f"SPL={metrics['spl']}%  n={metrics['num']}")
    except Exception as exc:
        print(f"[eval] metric.py 失败: {exc}")

    write_reports(run_dir, results, metrics, cfg)
    write_progress(run_dir, total=len(results), done=len(results), current=None,
                   metrics=metrics, state="done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
