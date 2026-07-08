"""
eval/openloop_eval.py — 开环离线评测：只看数据集第一帧 + 深度，直接出指标。

不需要 AirSim、不需要 UE4 环境、不需要 Linux。
只调 API（detector + planner），纯数学算指标。

用法:
    python eval/openloop_eval.py --dataset /path/to/UAV-VLN-FOV/test [--split test|unobject|unscene]

流程:
    每条轨迹:
        1. 读 FrontCamera/000000.png + DownCamera/000000.png
        2. 读 mark.json → 起始位姿 + GT 目标
        3. 读 obj_des.json → 指令
        4. detector.detect(前视, target) → bbox
        5. planner.plan(前视, 下视, 指令, bbox) → waypoints
        6. waypoint 终点 → 世界坐标
        7. 对比 GT → NE/SR/SPL
"""

import os, json, sys, math, time
from pathlib import Path
from typing import List, Dict
import numpy as np
from scipy.spatial.transform import Rotation as R

_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_root))

from config import cfg, get_cfg
get_cfg(str(_root / "config" / "default.yaml"))

from agent.detector import build_detector
from agent.planner import build_planner
from PIL import Image

SUCCESS_RADIUS = float(cfg.get("EVAL", {}).get("SUCCESS_RADIUS", 10.0))


def quat_to_rot(q: list) -> np.ndarray:
    """四元数 [x,y,z,w] → 3x3 旋转矩阵 body→world."""
    return R.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()


def load_episodes(dataset_path: str) -> List[Dict]:
    """扫描数据集目录，返回所有 episode。兼容 3DG-VLN UAV-VLN-FOV 格式。"""
    episodes = []
    root = Path(dataset_path)
    if not root.exists():
        raise FileNotFoundError(f"数据集不存在: {root}")

    for scene_dir in sorted(root.iterdir()):
        if not scene_dir.is_dir():
            continue
        for traj_dir in sorted(scene_dir.iterdir()):
            if not traj_dir.is_dir():
                continue

            mark_file = traj_dir / "mark.json"
            if not mark_file.exists():
                continue
            with open(mark_file) as f:
                mark = json.load(f)

            obj_des_file = traj_dir / "obj_des.json"
            instruction = ""
            if obj_des_file.exists():
                with open(obj_des_file) as f:
                    raw = json.load(f)
                    instruction = raw[0] if isinstance(raw, list) and raw else str(raw)

            # 找第一帧（不同轨迹起始帧号不同，取 FrontCamera 下最小的 png 文件名）
            front_dir = traj_dir / "FrontCamera"
            front_imgs = sorted([f for f in front_dir.iterdir() if f.suffix == ".png"]) if front_dir.exists() else []
            if not front_imgs:
                continue
            first_frame_stem = front_imgs[0].stem  # e.g. "000080"
            front_img = front_imgs[0]

            down_dir = traj_dir / "DownCamera"
            down_imgs = sorted([f for f in down_dir.iterdir() if f.suffix == ".png"]) if down_dir.exists() else []
            down_img = down_imgs[0] if down_imgs else front_imgs[0]

            # 起始位姿：从对应的 log/frame.json 获取
            log_file = traj_dir / "log" / f"{first_frame_stem}.json"
            start_pos = mark.get("start", [0, 0, 0])
            start_orientation = [0, 0, 0, 1]
            if log_file.exists():
                with open(log_file) as f:
                    log = json.load(f)
                    state = log.get("sensors", {}).get("state", {})
                    if "position" in state:
                        start_pos = state["position"]
                    if "orientation" in state:
                        start_orientation = state["orientation"]

            target_info = mark.get("target", {})
            gt_target = target_info.get("position", [0, 0, 0]) if isinstance(target_info, dict) else target_info

            episodes.append({
                "name": traj_dir.name,
                "scene": scene_dir.name,
                "start_pos": start_pos,
                "start_orientation": start_orientation,
                "gt_target": gt_target,
                "instruction": instruction,
                "front_img": str(front_img),
                "down_img": str(down_img) if down_img.exists() else str(front_img),
            })

    return episodes


def evaluate(episodes: List[Dict]):
    """开环批量评估。"""
    detector = build_detector()
    planner = build_planner()

    results = {"ne": [], "sr": 0, "osr": 0, "spl": [], "total": len(episodes)}
    waypoints_all = []

    for idx, ep in enumerate(episodes):
        print(f"\n[{idx+1}/{len(episodes)}] {ep['scene']}/{ep['name']}")
        print(f"  指令: {ep['instruction'][:80]}")

        # 加载图像
        try:
            front_rgb = Image.open(ep["front_img"]).convert("RGB")
            down_rgb = Image.open(ep["down_img"]).convert("RGB")
        except Exception as e:
            print(f"  [SKIP] 图像读取失败: {e}")
            # 仍然记录以便计数
            results["ne"].append(float("nan"))
            continue

        # 深度：只有真正从数据集读到才传, 不给假数据误导模型
        depth = _load_depth(Path(ep["front_img"]).parent.parent)
        # depth=None 时 planner 内部会处理：_build_depth_info 返回 "未获取到有效深度数据"

        # 检测
        target_obj = ep["instruction"].split(".")[0] if ep["instruction"] else "target"
        t0 = time.time()
        try:
            detection = detector.detect_with_fallback(
                front_rgb,
                down_rgb,
                target_obj,
                front_depth_meters=depth,
                down_depth_meters=None,
            )
        except Exception as e:
            print(f"  [SKIP] 检测失败: {e}")
            results["ne"].append(float("nan"))
            continue
        t1 = time.time()

        if not detection.visible or not detection.bbox:
            print(f"  [DETECT] 目标不可见，跳过")
            results["ne"].append(float("nan"))
            continue

        depth_str = f"depth={detection.depth_median:.1f}m" if detection.depth_median else "depth=N/A"
        print(f"  [DETECT] {detection.camera} bbox={detection.bbox} {depth_str} ({t1-t0:.1f}s)")

        # 规划
        try:
            plan_result = planner.plan(
                front_rgb, down_rgb,
                instruction=ep["instruction"],
                detected_bbox=detection.bbox,
                depth_meters=depth,
                detection=detection,
                down_depth_meters=None,
            )
        except Exception as e:
            print(f"  [SKIP] 规划失败: {e}")
            results["ne"].append(float("nan"))
            continue

        t2 = time.time()
        wps = plan_result.waypoints
        non_zero = sum(1 for wp in wps if not all(abs(v) < 1e-6 for v in wp))
        print(f"  [PLAN] {len(wps)} waypoints ({non_zero} non-zero, {t2-t1:.1f}s)")

        # waypoints 终点 → 世界坐标
        if not wps or non_zero == 0:
            print(f"  [SKIP] 空 waypoints")
            results["ne"].append(float("nan"))
            continue

        wp_final = wps[-1]  # 最后一个 waypoint [dx, dy, dz]，机体坐标
        if all(abs(v) < 1e-6 for v in wp_final):
            # 找最后一个非零 waypoint
            for wp in reversed(wps):
                if not all(abs(v) < 1e-6 for v in wp):
                    wp_final = wp
                    break

        # 机体坐标 → 世界坐标
        start_pos = np.array(ep["start_pos"])
        R_start = quat_to_rot(ep["start_orientation"])
        world_final = start_pos + R_start @ np.array(wp_final[:3])

        gt_target = np.array(ep["gt_target"])
        ne = float(np.linalg.norm(world_final - gt_target))
        sr = ne <= SUCCESS_RADIUS
        sl = float(np.linalg.norm(start_pos - gt_target))

        # TL 用 waypoints 累计路径长度估算
        prev = np.zeros(3)
        tl = 0.0
        for wp in wps:
            curr = np.array(wp[:3])
            tl += float(np.linalg.norm(curr - prev))
            prev = curr

        spl = (sl / max(tl, sl)) if sr else 0.0

        results["ne"].append(ne)
        results["sr"] += 1 if sr else 0
        results["spl"].append(spl)
        waypoints_all.append(wps)

        print(f"  [RESULT] NE={ne:.1f}m SR={'✅' if sr else '❌'} SPL={spl:.3f} "
              f"TL={tl:.1f}m SL={sl:.1f}m")

    # 汇总
    valid_ne = [n for n in results["ne"] if not math.isnan(n)]
    valid_spl = results["spl"]
    n_total = results["total"]

    print("\n" + "=" * 60)
    print(f"  开环评测完成: {n_total} 条 ({len(valid_ne)} 有效)")
    if n_total == 0:
        print("  (无数据，请检查 --dataset 路径)")
        print("=" * 60)
        return
    print(f"  SR  (Success Rate):          {results['sr']/n_total*100:.2f}%")
    print(f"  NE  (Navigation Error):      {np.mean(valid_ne):.2f}m" if valid_ne else "  NE: N/A")
    print(f"  SPL (Path Length Weighted):  {np.mean(valid_spl):.4f}" if valid_spl else "  SPL: N/A")
    print(f"  无效轨迹(检测/规划失败):      {n_total - len(valid_ne)}")
    print("=" * 60)


def _load_depth(traj_dir: Path) -> np.ndarray | None:
    """尝试从数据集加载深度图。3DG-VLN 的深度以 8-bit PNG 存储，值×100/255=米。"""
    depth_file = traj_dir / "FrontCamera" / "000000_depth.png"
    if not depth_file.exists():
        # 尝试 log 中的深度数据
        return None
    depth_img = Image.open(depth_file)
    depth = np.array(depth_img, dtype=np.float32) * 100.0 / 255.0
    return depth


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="开环离线评测（不需要 AirSim）")
    parser.add_argument("--dataset", required=True, help="数据集根目录 (如 UAV-VLN-FOV/test)")
    parser.add_argument("--scene", default=None, help="只跑指定场景")
    args = parser.parse_args()

    episodes = load_episodes(args.dataset)
    if args.scene:
        episodes = [e for e in episodes if e["scene"] == args.scene]

    print(f"加载 {len(episodes)} 条轨迹")
    evaluate(episodes)
