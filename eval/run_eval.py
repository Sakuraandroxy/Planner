"""
eval/run_eval.py — 通用离线评估器（闭环，需 AirSim）。

用法:
    # 单场景手动模式 (不变)
    python eval/run_eval.py --dataset ./data/test --scene ModularEuropean

    # 多场景自动切换模式 (新增, 需 TravelUAV 环境)
    python eval/run_eval.py --dataset ./data/test --env_root /data/sakura/data/TravelUAV_env [--gpu 0]

数据集格式（兼容 3DG-VLN UAV-VLN-FOV）:
    dataset/
    └── scene_name/
        └── episode_name/
            ├── mark.json      (start, target.position)
            └── obj_des.json   (instruction)

流程:
    1. 遍历所有 episode
    2. 对于每个: 连接 AirSim → 放到起点 → 运行 Agent → 执行 → 记录轨迹
    3. 全部跑完后出指标 SR / OSR / NE / SPL
"""

import os, json, sys, time, math
from pathlib import Path
from typing import List, Dict, Optional
import numpy as np

# 项目路径
_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_root))

from config import cfg
from agent.detector import build_detector
from agent.planner import build_planner
from agent.direction import build_direction_estimator
from eval.metrics import MetricsTracker
from sim.airsim_client import AirSimClient


SUCCESS_RADIUS = 10.0  # 终点在目标 10m 内 = 成功


def discover_episodes(dataset_path: str) -> List[Dict]:
    """扫描数据集目录，返回所有 episode 的元信息。

    兼容 3DG-VLN UAV-VLN-FOV 格式:
        Scene/traj/mark.json  +  obj_des.json
    """
    episodes = []
    dataset_path = Path(dataset_path)
    if not dataset_path.exists():
        raise FileNotFoundError(f"数据集不存在: {dataset_path}")

    for scene_dir in sorted(dataset_path.iterdir()):
        if not scene_dir.is_dir():
            continue
        scene_name = scene_dir.name
        for ep_dir in sorted(scene_dir.iterdir()):
            if not ep_dir.is_dir():
                continue
            mark_file = ep_dir / "mark.json"
            if not mark_file.exists():
                continue
            with open(mark_file, "r") as f:
                meta = json.load(f)

            # 3DG-VLN 格式: mark.json 含 start/end/target
            start_pos = meta.get("start", [0, 0, 0])
            target_info = meta.get("target", {})
            target_pos = target_info.get("position", [0, 0, 0]) if isinstance(target_info, dict) else target_info

            # 指令从 obj_des.json 读取
            obj_des_file = ep_dir / "obj_des.json"
            instruction = ""
            target_obj = ""
            if obj_des_file.exists():
                with open(obj_des_file, "r") as f:
                    obj_des = json.load(f)
                    if isinstance(obj_des, list) and len(obj_des) > 0:
                        instruction = obj_des[0]
                    elif isinstance(obj_des, str):
                        instruction = obj_des
                    target_obj = meta.get("object_name", "")

            episodes.append({
                "name": ep_dir.name,
                "scene": scene_name,
                "start": start_pos,
                "target": target_pos,
                "instruction": instruction,
                "target_obj": target_obj,
            })
    return episodes


def evaluate(episodes: List[Dict], scene_filter: str = None,
             scene_manager=None, current_scene: str = ""):
    """批量评估。

    Args:
        episodes: discover_episodes() 的输出
        scene_filter: 只跑特定场景（如 "Town01"）
        scene_manager: SceneManager 实例（可选，用于多场景自动切换）
        current_scene: 当前已启动的场景名
    """
    if scene_filter:
        episodes = [ep for ep in episodes if ep["scene"] == scene_filter]

    print(f"评估 {len(episodes)} 条 episode...")
    if not episodes:
        print("没有要跑的 episode")
        return

    # 初始化各模块（由 config 决定实现）
    detector = build_detector()
    planner = build_planner()
    direction_est = build_direction_estimator()

    # 连接 AirSim
    client = AirSimClient()
    client.connect()
    client.warmup_capture()

    all_metrics = {
        "sr": 0, "osr": 0,
        "ne_list": [], "spl_list": [],
        "trajectory_lengths": [],
    }

    for idx, ep in enumerate(episodes):
        print(f"\n[{idx+1}/{len(episodes)}] {ep['scene']}/{ep['name']}")
        print(f"  指令: {ep['instruction']}")
        print(f"  目标: {ep['target']}")

        # ── 场景切换检测 ──
        if scene_manager is not None and ep["scene"] != current_scene:
            current_scene = ep["scene"]
            if not scene_manager.start(current_scene):
                print(f"  [SKIP] 场景 '{current_scene}' 不可用")
                continue
            # 重新连接 AirSim (旧连接已失效)
            try:
                client.cleanup()
            except Exception:
                pass
            time.sleep(2)
            client = AirSimClient()
            client.connect()
            client.warmup_capture()
            print(f"  [SceneManager] 已切换到 {current_scene}")

        # 创建指标跟踪器
        metrics = MetricsTracker(ep["target"])
        task_done = False

        # ── 初始化无人机到轨迹起始位姿 ──
        start = ep["start"]
        print(f"  起始位姿: {start}")
        try:
            client.enable_api_control(True)
            client.arm(True)
            client.move_to_position(start[0], start[1], start[2], velocity=5.0, timeout=30.0)
            time.sleep(1.0)
        except Exception as e:
            print(f"  [WARN] 初始化位姿失败: {e}")
            continue

        # 主循环
        for step in range(100):
            try:
                pos = client.get_pose()[0]
            except Exception:
                break

            # 抓帧
            front_rgb, down_rgb, front_depth = client.get_dual_view()
            if front_rgb is None:
                time.sleep(0.1)
                continue

            # 检测
            detect_caption = ep.get("target_obj", ep["instruction"])
            detection = detector.detect(front_rgb, detect_caption, front_depth)

            # 方向
            direction = ""
            if detection.visible and detection.bbox:
                direction = direction_est.estimate(
                    detection.bbox, 0, front_rgb.size
                )

            # 停止判断
            if detection.visible and detection.depth_median is not None:
                if detection.depth_median < cfg["AGENT"]["STOP_DEPTH_THRESHOLD"]:
                    print(f"  [Done] depth={detection.depth_median:.1f}m < threshold")
                    task_done = True
                    break

            # 规划
            result = planner.plan(
                front_rgb, down_rgb,
                instruction=ep["instruction"],
                direction=direction,
                detected_bbox=detection.bbox if detection.visible else None,
                depth_meters=front_depth,
            )

            # 执行
            if result.waypoints and not all(all(v == 0 for v in wp) for wp in result.waypoints):
                pos_final, _, collided = client.execute_waypoints(result.waypoints)
                metrics.record_step(pos_final)
                if step % 5 == 0:
                    print(f"  Step {step}: dist={metrics.current_distance:.1f}m")
            else:
                print(f"  Step {step}: no valid waypoints, stopping")
                break

            if result.done:
                task_done = True
                break

        # 记录指标
        ne = metrics.ne
        sr = task_done or (ne is not None and ne < SUCCESS_RADIUS)
        osr_flag = metrics.osr
        spl = metrics.spl

        all_metrics["sr"] += 1 if sr else 0
        all_metrics["osr"] += 1 if osr_flag else 0
        if ne is not None:
            all_metrics["ne_list"].append(ne)
        all_metrics["spl_list"].append(spl)
        all_metrics["trajectory_lengths"].append(metrics.trajectory_length)

        print(f"  NE={ne:.1f}m SR={'✅' if sr else '❌'} OSR={osr_flag} SPL={spl:.2f}")

    # 汇总
    n = len(episodes)
    if n > 0:
        print("\n" + "=" * 50)
        print(f"评估完成: {n} 条")
        print(f"  SR  (Success Rate):          {all_metrics['sr']/n*100:.2f}%")
        print(f"  OSR (Oracle Success Rate):   {all_metrics['osr']/n*100:.2f}%")
        if all_metrics['ne_list']:
            print(f"  NE  (Navigation Error):      {np.mean(all_metrics['ne_list']):.2f}m")
        print(f"  SPL (Path Length Weighted):  {np.mean(all_metrics['spl_list']):.4f}")
        print(f"  平均路径长度:                {np.mean(all_metrics['trajectory_lengths']):.1f}m")
        print("=" * 50)

    client.cleanup()


def _group_by_scene(episodes: List[Dict]) -> Dict[str, List[Dict]]:
    """按场景分组，保持数据集原始顺序。"""
    groups: Dict[str, List[Dict]] = {}
    for ep in episodes:
        groups.setdefault(ep["scene"], []).append(ep)
    return groups


def evaluate_multi_scene(episodes: List[Dict], env_root: str, gpu_id: int = 0):
    """多场景自动切换评估。

    按场景分组 → 逐场景启动 UE4 → 跑该场景所有轨迹 → 切换下一场景。
    """
    from eval.scene_manager import SceneManager

    scene_groups = _group_by_scene(episodes)
    scene_order = sorted(scene_groups.keys())
    print(f"\n数据集含 {len(scene_order)} 个场景: {scene_order}")

    manager = SceneManager(env_root, gpu_id=gpu_id)
    available = manager.available_scenes
    print(f"可用 UE4 环境 {len(available)} 个场景: {available[:5]}...")

    all_scene_metrics = {}

    for scene_name in scene_order:
        eps = scene_groups[scene_name]
        print(f"\n{'='*50}")
        print(f"  场景: {scene_name} ({len(eps)} 条)")
        print(f"{'='*50}")

        # 按场景排序 → 同场景的 episode 已连续排列
        # evaluate() 内部检测到新场景会自动调用 manager.start()
        sorted_eps = sorted(eps, key=lambda e: e["name"])

        if not manager.start(scene_name):
            print(f"  [SKIP] 场景 '{scene_name}' 的 UE4 环境不可用, 跳过 {len(eps)} 条")
            continue

        evaluate(sorted_eps, scene_filter=None,
                 scene_manager=manager, current_scene=scene_name)

    manager.stop()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="闭环评测器（需 AirSim）")
    parser.add_argument("--dataset", required=True, help="数据集根目录 (如 UAV-VLN-FOV/test)")
    parser.add_argument("--scene", default=None, help="只跑指定场景（单场景模式）")
    parser.add_argument("--env_root", default=None,
                        help="TravelUAV 环境根目录（启用多场景自动切换）")
    parser.add_argument("--gpu", type=int, default=0,
                        help="GPU ID (多场景模式, 默认 0)")
    args = parser.parse_args()

    episodes = discover_episodes(args.dataset)

    if args.env_root:
        # 多场景自动切换模式
        evaluate_multi_scene(episodes, args.env_root, gpu_id=args.gpu)
    else:
        # 单场景/手动模式（保持原有行为）
        evaluate(episodes, scene_filter=args.scene)
