"""
从 3DG-VLN 原始轨迹数据中提取 Qwen2.5-VL 微调样本。

用法:
    python extract_training_samples.py \
        --dataset /path/to/UAV-VLN-FOV/train \
        --meta /path/to/UAV-VLN-FOV/meta \
        --output ./finetune_data \
        --waypoints 5

输出格式（LLaMA-Factory 兼容）:
    finetune_data/
    ├── images/
    │   ├── traj001_step00_front.png
    │   ├── traj001_step00_down.png
    │   └── ...
    └── dataset.json          # [{messages, images}, ...]
"""
import argparse, json, os, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image


def load_instructions(meta_dir: Path) -> dict:
    """加载 meta/instructions.json: trajectory_name -> instruction_text."""
    path = meta_dir / "instructions.json"
    if not path.exists():
        print(f"[WARN] {path} not found, instructions will be empty")
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_trajectory(traj_dir: Path) -> list[dict]:
    """读取一条轨迹的所有 log 帧, 按顺序返回."""
    log_dir = traj_dir / "log"
    if not log_dir.exists():
        return []
    frames = []
    for fname in sorted(os.listdir(log_dir)):
        with open(log_dir / fname, "r") as f:
            frames.append(json.load(f))
    return frames


def _read_instruction(traj_dir: Path, centralized: dict = None,
                      traj_name: str = "") -> str:
    """从轨迹目录读取指令：优先 obj_des.json，回退到集中的 instructions dict。"""
    obj_des = traj_dir / "obj_des.json"
    if obj_des.exists():
        with open(obj_des, "r", encoding="utf-8") as f:
            arr = json.load(f)
            if isinstance(arr, list) and len(arr) > 0:
                return arr[0]
    if centralized:
        inst = centralized.get(traj_name, "")
        if inst:
            return inst
    return ""


def quat_to_rot_matrix(q: list) -> np.ndarray:
    """四元数 [x,y,z,w] -> 3x3 旋转矩阵 (body->world)。"""
    r = R.from_quat([q[0], q[1], q[2], q[3]])
    return r.as_matrix()


def compute_waypoints(frames: list[dict], start_idx: int, num_wp: int = 5
                      ) -> list[list[float]]:
    """从 start_idx 帧开始, 取后续 num_wp 个轨迹点。

    每个 waypoint [dx,dy,dz] 是相对于起始位置的机体坐标系累积偏移量，
    与 execute_waypoints 的执行逻辑一致：

        waypoints[i] = R_0^T @ (P_i - P_0)

    即所有 waypoint 都从起点算，不是增量。
    """
    n_remaining = len(frames) - start_idx - 1
    if n_remaining <= 0:
        return [[0.0, 0.0, 0.0]] * num_wp

    start_pos = np.array(frames[start_idx]["sensors"]["state"]["position"])
    ori = frames[start_idx]["sensors"]["imu"]["orientation"]
    start_rot = quat_to_rot_matrix(ori)

    indices = np.linspace(start_idx + 1, len(frames) - 1, min(num_wp, n_remaining),
                          dtype=int)

    wp = []
    for idx in indices:
        target_pos = np.array(frames[idx]["sensors"]["state"]["position"])
        delta_world = target_pos - start_pos       # 从起始位置到目标点（累积）
        delta_body = start_rot.T @ delta_world     # world -> body frame
        wp.append([round(float(delta_body[0]), 2),
                    round(float(delta_body[1]), 2),
                    round(float(delta_body[2]), 2)])

    while len(wp) < num_wp:
        wp.append([0.0, 0.0, 0.0])
    return wp


def build_sample(traj_name: str, instruction: str, step: int,
                 front_img_path: str, down_img_path: str,
                 waypoints: list) -> dict:
    """构建一个 Qwen2.5-VL 格式的对话样本.

    prompt 格式：Instruction: xxx
    不包含任何硬编码假字段（Stage/Previous displacement/Current position）。
    模型只需理解：前视图+下视图+指令 → 输出 5 个机体坐标系累积位移 waypoint。
    """
    wp_str = json.dumps(waypoints, ensure_ascii=False)
    return {
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": front_img_path},
                    {"type": "image", "image": down_img_path},
                    {"type": "text",
                     "text": f"Instruction: {instruction}"}
                ]
            },
            {
                "role": "assistant",
                "content": wp_str
            }
        ],
        "images": [front_img_path, down_img_path]
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True, help="训练集路径, 如 UAV-VLN-FOV/train/")
    parser.add_argument("--meta", required=True, help="meta 目录路径")
    parser.add_argument("--output", required=True, help="输出目录")
    parser.add_argument("--waypoints", type=int, default=5)
    parser.add_argument("--step_interval", type=int, default=5,
                        help="每隔多少帧取一个训练样本 (避免相邻帧过于相似)")
    args = parser.parse_args()

    dataset_dir = Path(args.dataset)
    meta_dir = Path(args.meta)
    output_dir = Path(args.output)
    img_out = output_dir / "images"
    img_out.mkdir(parents=True, exist_ok=True)

    instructions = load_instructions(meta_dir)
    all_samples = []

    # 遍历所有场景/轨迹
    for scene_dir in sorted(dataset_dir.iterdir()):
        if not scene_dir.is_dir():
            continue
        for traj_dir in sorted(scene_dir.iterdir()):
            if not traj_dir.is_dir():
                continue
            traj_name = traj_dir.name
            instruction = _read_instruction(traj_dir, instructions, traj_name)
            if not instruction:
                print(f"  [SKIP] {traj_name}: no instruction")
                continue

            frames = load_trajectory(traj_dir)
            n_frames = len(frames)
            if n_frames < args.waypoints + 2:
                print(f"  [SKIP] {traj_name}: too few frames ({n_frames})")
                continue

            print(f"  {scene_dir.name}/{traj_name}: {n_frames} frames, "
                  f"instruction='{instruction[:50]}...'")

            for step in range(0, n_frames - args.waypoints, args.step_interval):
                frame_num = frames[step]["frame"]
                frame_str = f"{frame_num:06d}"

                front_src = traj_dir / "FrontCamera" / f"{frame_str}.png"
                down_src = traj_dir / "DownCamera" / f"{frame_str}.png"

                if not front_src.exists() or not down_src.exists():
                    continue

                front_dst_name = f"{traj_name}_step{frame_str}_front.png"
                down_dst_name = f"{traj_name}_step{frame_str}_down.png"
                front_dst = img_out / front_dst_name
                down_dst = img_out / down_dst_name

                if not front_dst.exists():
                    Image.open(front_src).save(front_dst)
                if not down_dst.exists():
                    Image.open(down_src).save(down_dst)

                wp = compute_waypoints(frames, step, args.waypoints)

                sample = build_sample(
                    traj_name, instruction, step,
                    str(front_dst.relative_to(output_dir)),
                    str(down_dst.relative_to(output_dir)),
                    wp
                )
                all_samples.append(sample)

    dataset_file = output_dir / "dataset.json"
    with open(dataset_file, "w", encoding="utf-8") as f:
        json.dump(all_samples, f, ensure_ascii=False, indent=2)

    print(f"\nDone: {len(all_samples)} samples -> {dataset_file}")
    print(f"Images in: {img_out}")


if __name__ == "__main__":
    main()
