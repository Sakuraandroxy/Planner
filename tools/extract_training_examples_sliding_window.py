"""
Extract sliding-window Qwen2.5-VL fine-tuning samples from 3DG-VLN trajectories.

The original extractor samples five waypoints by splitting the whole remaining
trajectory into five segments. This script keeps the online sliding-window
setting instead:

    input  = front image + down image + instruction + <=3 pending waypoints
    output = up to 5 additional incremental body-frame waypoints

All pending and output waypoints are incremental displacements expressed in the
current body/front-view frame:

    cumulative_i = R_current^T @ (P_future_i - P_current)
    waypoint_1   = cumulative_1
    waypoint_i   = cumulative_i - cumulative_{i-1}

Example:
    python tools/extract_training_examples_sliding_window.py \
        --dataset /path/to/UAV-VLN-FOV/train \
        --meta /path/to/UAV-VLN-FOV/meta \
        --output ./finetune_sliding \
        --future_stride 3
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import random
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


def load_instructions(meta_dir: Path) -> dict:
    path = meta_dir / "instructions.json"
    if not path.exists():
        print(f"[WARN] {path} not found, instructions will be empty")
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_trajectory(traj_dir: Path) -> list[dict]:
    log_dir = traj_dir / "log"
    if not log_dir.exists():
        return []

    frames = []
    for fname in sorted(os.listdir(log_dir)):
        with open(log_dir / fname, "r", encoding="utf-8") as f:
            frames.append(json.load(f))
    return frames


def iter_trajectory_dirs(dataset_dir: Path):
    """Yield (scene_name, trajectory_dir) for both scene/traj and direct layouts."""
    for first_level in sorted(dataset_dir.iterdir()):
        if not first_level.is_dir():
            continue
        if (first_level / "log").is_dir():
            yield dataset_dir.name, first_level
            continue
        for traj_dir in sorted(first_level.iterdir()):
            if traj_dir.is_dir() and (traj_dir / "log").is_dir():
                yield first_level.name, traj_dir


def read_instruction(traj_dir: Path, centralized: dict | None = None,
                     traj_name: str = "") -> str:
    obj_des = traj_dir / "obj_des.json"
    if obj_des.exists():
        with open(obj_des, "r", encoding="utf-8") as f:
            arr = json.load(f)
            if isinstance(arr, list) and arr:
                return str(arr[0])

    if centralized:
        inst = centralized.get(traj_name, "")
        if inst:
            return str(inst)
    return ""


def quat_to_rot_matrix(q: list[float]) -> np.ndarray:
    return R.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()


def format_coord(value: float) -> float:
    rounded = round(float(value), 2)
    if abs(rounded) < 1e-6:
        return 0.0
    return rounded


def format_waypoints_json(waypoints: list[list[float]]) -> str:
    rows = []
    for waypoint in waypoints:
        coords = [format_coord(coord) for coord in waypoint]
        rows.append("[" + ", ".join(f"{coord:.2f}" for coord in coords) + "]")
    return "[" + ", ".join(rows) + "]"


def parse_pending_counts(raw: str, max_pending: int) -> list[int]:
    values = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        value = int(item)
        if value < 0 or value > max_pending:
            raise ValueError(
                f"pending count {value} outside valid range [0, {max_pending}]"
            )
        values.append(value)
    return sorted(set(values))


def sampled_future_indices(
    start_idx: int,
    end_idx: int,
    future_stride: int,
    count: int,
) -> list[int]:
    indices = []
    for step_id in range(1, count + 1):
        idx = start_idx + step_id * future_stride
        if idx > end_idx:
            break
        indices.append(idx)
    return indices


def compute_incremental_body_waypoints(
    frames: list[dict],
    start_idx: int,
    future_indices: list[int],
) -> list[list[float]]:
    start_pos = np.array(frames[start_idx]["sensors"]["state"]["position"],
                         dtype=float)
    ori = frames[start_idx]["sensors"]["imu"]["orientation"]
    start_rot = quat_to_rot_matrix(ori)

    cumulative = []
    for idx in future_indices:
        target_pos = np.array(frames[idx]["sensors"]["state"]["position"],
                              dtype=float)
        delta_world = target_pos - start_pos
        delta_body = start_rot.T @ delta_world
        cumulative.append(delta_body.astype(float))

    waypoints = []
    prev = np.zeros(3, dtype=float)
    for point in cumulative:
        delta = point - prev
        waypoints.append([
            format_coord(delta[0]),
            format_coord(delta[1]),
            format_coord(delta[2]),
        ])
        prev = point
    return waypoints


def build_prompt(
    instruction: str,
    pending: list[list[float]],
    max_additional: int,
) -> str:
    lines = [f"Instruction: {instruction}"]
    if pending:
        lines.append(
            "Pending incremental body-frame waypoints: "
            f"{format_waypoints_json(pending)}"
        )
    lines.extend([
        (
            f"Output up to {max_additional} additional incremental "
            "body-frame waypoints as a JSON list."
        ),
        (
            "Each waypoint must be [dx, dy, dz], where the first pending "
            "or output waypoint is relative to the current drone position/front-view frame "
            "and each following waypoint is relative to the previous waypoint."
        ),
        "Do not output any other text.",
    ])
    return "\n".join(lines)


def build_sample(
    instruction: str,
    front_img_path: str,
    down_img_path: str,
    pending: list[list[float]],
    additional: list[list[float]],
    max_additional: int,
) -> dict:
    return {
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": front_img_path},
                    {"type": "image", "image": down_img_path},
                    {
                        "type": "text",
                        "text": build_prompt(
                            instruction=instruction,
                            pending=pending,
                            max_additional=max_additional,
                        ),
                    },
                ],
            },
            {
                "role": "assistant",
                "content": format_waypoints_json(additional),
            },
        ],
        "images": [front_img_path, down_img_path],
    }


def materialize_image(src: Path, dst: Path, mode: str) -> None:
    if dst.exists():
        return
    dst.parent.mkdir(parents=True, exist_ok=True)

    if mode == "reference":
        return

    if mode in {"hardlink", "auto"}:
        try:
            os.link(src, dst)
            return
        except OSError:
            if mode == "hardlink":
                raise

    if mode == "symlink":
        os.symlink(src, dst)
        return

    # File copy is much faster than PNG decode/re-encode and preserves bytes.
    shutil.copy2(src, dst)


def trajectory_step_stats(frames: list[dict]) -> tuple[float, float]:
    if len(frames) < 2:
        return 0.0, 0.0

    dists = []
    dts = []
    for prev, cur in zip(frames, frames[1:]):
        prev_pos = np.array(prev["sensors"]["state"]["position"], dtype=float)
        cur_pos = np.array(cur["sensors"]["state"]["position"], dtype=float)
        dists.append(float(np.linalg.norm(cur_pos - prev_pos)))

        prev_ts = prev["sensors"]["state"].get("timestamp")
        cur_ts = cur["sensors"]["state"].get("timestamp")
        if prev_ts is not None and cur_ts is not None:
            dts.append((int(cur_ts) - int(prev_ts)) / 1000.0)

    avg_dist = float(np.mean(dists)) if dists else 0.0
    avg_dt = float(np.mean(dts)) if dts else 0.0
    return avg_dist, avg_dt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True,
                        help="Dataset path, e.g. UAV-VLN-FOV/train")
    parser.add_argument("--meta", required=True, help="Meta directory path")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--future_stride", type=int, default=3,
                        help="Frame interval between adjacent sampled waypoints")
    parser.add_argument("--step_interval", type=int, default=5,
                        help="Frame interval between training examples")
    parser.add_argument("--max_pending", type=int, default=3)
    parser.add_argument("--max_additional", type=int, default=5)
    parser.add_argument("--pending_counts", default="0,1,2,3",
                        help="Comma-separated pending counts to generate")
    parser.add_argument(
        "--pending_count_mode",
        choices=["all", "random"],
        default="all",
        help=(
            "all = generate one sample for every pending count; "
            "random = sample one pending count per step from pending_counts"
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed used when --pending_count_mode=random",
    )
    parser.add_argument(
        "--image_mode",
        choices=["auto", "copy", "hardlink", "symlink", "reference"],
        default="auto",
        help=(
            "How to place images in output/images. auto tries hardlink first "
            "and falls back to file copy. reference stores original image "
            "paths without copying files."
        ),
    )
    args = parser.parse_args()

    if args.future_stride <= 0:
        raise ValueError("--future_stride must be positive")
    if args.step_interval <= 0:
        raise ValueError("--step_interval must be positive")
    if args.max_pending < 0:
        raise ValueError("--max_pending must be non-negative")
    if args.max_additional <= 0:
        raise ValueError("--max_additional must be positive")

    pending_counts = parse_pending_counts(args.pending_counts, args.max_pending)
    rng = random.Random(args.seed)

    dataset_dir = Path(args.dataset)
    meta_dir = Path(args.meta)
    output_dir = Path(args.output)
    img_out = output_dir / "images"
    img_out.mkdir(parents=True, exist_ok=True)

    instructions = load_instructions(meta_dir)
    all_samples = []
    total_traj = 0
    total_skipped = 0
    dist_stats = []
    dt_stats = []

    max_future_count = args.max_pending + args.max_additional

    for scene_name, traj_dir in iter_trajectory_dirs(dataset_dir):
            traj_name = traj_dir.name
            instruction = read_instruction(traj_dir, instructions, traj_name)
            if not instruction:
                print(f"  [SKIP] {traj_name}: no instruction")
                total_skipped += 1
                continue

            frames = load_trajectory(traj_dir)
            n_frames = len(frames)
            min_required = args.future_stride + 1
            if n_frames < min_required:
                print(f"  [SKIP] {traj_name}: too few frames ({n_frames})")
                total_skipped += 1
                continue

            avg_dist, avg_dt = trajectory_step_stats(frames)
            if avg_dist > 0:
                dist_stats.append(avg_dist)
            if avg_dt > 0:
                dt_stats.append(avg_dt)

            total_traj += 1
            print(
                f"  {scene_name}/{traj_name}: {n_frames} frames, "
                f"avg_frame_dist={avg_dist:.2f}m, avg_dt={avg_dt:.2f}s, "
                f"instruction='{instruction[:50]}...'"
            )

            for step in range(0, n_frames - 1, args.step_interval):
                frame_num = frames[step]["frame"]
                frame_str = f"{frame_num:06d}"

                front_src = traj_dir / "FrontCamera" / f"{frame_str}.png"
                down_src = traj_dir / "DownCamera" / f"{frame_str}.png"
                if not front_src.exists() or not down_src.exists():
                    continue

                future_indices = sampled_future_indices(
                    start_idx=step,
                    end_idx=n_frames - 1,
                    future_stride=args.future_stride,
                    count=max_future_count,
                )
                if not future_indices:
                    continue

                if args.image_mode == "reference":
                    front_img_path = str(front_src)
                    down_img_path = str(down_src)
                else:
                    front_dst_name = f"{traj_name}_step{frame_str}_front.png"
                    down_dst_name = f"{traj_name}_step{frame_str}_down.png"
                    front_dst = img_out / front_dst_name
                    down_dst = img_out / down_dst_name
                    materialize_image(front_src, front_dst, args.image_mode)
                    materialize_image(down_src, down_dst, args.image_mode)
                    front_img_path = str(front_dst.relative_to(output_dir))
                    down_img_path = str(down_dst.relative_to(output_dir))

                waypoints = compute_incremental_body_waypoints(
                    frames=frames,
                    start_idx=step,
                    future_indices=future_indices,
                )

                selected_pending_counts = pending_counts
                if args.pending_count_mode == "random":
                    if not pending_counts:
                        continue
                    selected_pending_counts = [rng.choice(pending_counts)]

                for pending_count in selected_pending_counts:
                    if len(waypoints) <= pending_count:
                        continue
                    pending = waypoints[:pending_count]
                    additional = waypoints[
                        pending_count:pending_count + args.max_additional
                    ]
                    if not additional:
                        continue

                    sample = build_sample(
                        instruction=instruction,
                        front_img_path=front_img_path,
                        down_img_path=down_img_path,
                        pending=pending,
                        additional=additional,
                        max_additional=args.max_additional,
                    )
                    all_samples.append(sample)

    dataset_file = output_dir / "dataset.json"
    with open(dataset_file, "w", encoding="utf-8") as f:
        json.dump(all_samples, f, ensure_ascii=False, indent=2)

    print(f"\nDone: {len(all_samples)} samples -> {dataset_file}")
    print(f"Images in: {img_out}")
    print(f"Trajectories used: {total_traj}, skipped: {total_skipped}")
    if dist_stats:
        print(f"Avg per-frame distance: {float(np.mean(dist_stats)):.2f}m")
        print(
            "Approx waypoint spacing: "
            f"{float(np.mean(dist_stats)) * args.future_stride:.2f}m "
            f"(future_stride={args.future_stride})"
        )
    if dt_stats:
        print(
            "Approx expert waypoint interval: "
            f"{float(np.mean(dt_stats)) * args.future_stride:.2f}s "
            f"(future_stride={args.future_stride})"
        )


if __name__ == "__main__":
    main()
