"""AirSim 抓图方式测速脚本。

对比三种模式：
1. front_depth: 前视 RGB + 前视深度
2. dual_batch:  单次 RPC 批量抓前/下视 RGB + 前/下视深度
3. dual_parallel4: 4 个独立 client 并发抓 4 路图（实验）

用法:
    python tools/benchmark_airsim_capture.py
    python tools/benchmark_airsim_capture.py --runs 3 (--local/--ip xx.xx.xx.xx)
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim.airsim_client import AirSimClient  # noqa: E402


def _summarize(name: str, values: list[float]):
    if not values:
        return
    avg = statistics.mean(values)
    best = min(values)
    worst = max(values)
    print(f"{name:<16} avg={avg:.3f}s  min={best:.3f}s  max={worst:.3f}s")


def main():
    parser = argparse.ArgumentParser(description="Benchmark AirSim capture modes")
    parser.add_argument("--runs", type=int, default=3, help="每种模式重复次数")
    parser.add_argument("--local", action="store_true", help="忽略 config/default.yaml 中的 SIM.AIRSIM_IP，强制连接本地 localhost")
    parser.add_argument("--ip", default="", help="显式指定 AirSim IP；设置后优先级高于 --local 和配置文件")
    parser.add_argument("--port", type=int, default=41451, help="AirSim 端口，默认 41451")
    args = parser.parse_args()

    if args.ip:
        client = AirSimClient(ip=args.ip, port=args.port, use_config_ip=False)
        target = f"{args.ip}:{args.port}"
    elif args.local:
        client = AirSimClient(port=args.port, use_config_ip=False)
        target = f"localhost:{args.port}"
    else:
        client = AirSimClient(port=args.port)
        from config import cfg
        sim_cfg = cfg.get("SIM", {})
        cfg_ip = sim_cfg.get("AIRSIM_IP", "")
        target = f"{cfg_ip or 'localhost'}:{int(sim_cfg.get('AIRSIM_PORT', args.port))}"

    print(f"[Connect] target={target}")
    try:
        client.connect()
    except Exception as exc:
        print(f"[Connect] failed: {exc}")
        print("提示：")
        print("  1. 如果你当前跑的是本地 AirSim，请加 --local")
        print("  2. 如果你要测远端 AirSim，请加 --ip <server_ip>")
        print("  3. 确认 AirSim RPC 端口已启动，默认是 41451")
        raise
    client.warmup_capture()

    front_times = []
    batch_times = []
    parallel_times = []

    for idx in range(args.runs):
        print(f"\n=== Run {idx + 1}/{args.runs} ===")

        t0 = time.perf_counter()
        front_rgb, front_depth = client.get_scene_and_depth_meters()
        t1 = time.perf_counter()
        front_elapsed = t1 - t0
        front_times.append(front_elapsed)
        print(
            f"[front_depth]  total={front_elapsed:.3f}s  "
            f"rgb={front_rgb.size if front_rgb is not None else 'None'}  "
            f"depth={front_depth.shape if front_depth is not None else 'None'}"
        )

        t0 = time.perf_counter()
        front_rgb, down_rgb, front_depth, down_depth = client.get_dual_view()
        t1 = time.perf_counter()
        batch_elapsed = t1 - t0
        batch_times.append(batch_elapsed)
        print(
            f"[dual_batch]   total={batch_elapsed:.3f}s  "
            f"front={front_rgb.size if front_rgb is not None else 'None'}  "
            f"down={down_rgb.size if down_rgb is not None else 'None'}  "
            f"depth_front={front_depth.shape if front_depth is not None else 'None'}  "
            f"depth_down={down_depth.shape if down_depth is not None else 'None'}"
        )

        t0 = time.perf_counter()
        front_rgb, down_rgb, front_depth, down_depth, timing = client.get_dual_view_parallel_experimental()
        t1 = time.perf_counter()
        parallel_elapsed = t1 - t0
        parallel_times.append(parallel_elapsed)
        print(
            f"[dual_parallel4] total={parallel_elapsed:.3f}s  "
            f"wall={timing['wall_s']:.3f}s  "
            f"front={front_rgb.size if front_rgb is not None else 'None'}  "
            f"down={down_rgb.size if down_rgb is not None else 'None'}  "
            f"depth_front={front_depth.shape if front_depth is not None else 'None'}  "
            f"depth_down={down_depth.shape if down_depth is not None else 'None'}"
        )

    print("\n=== Summary ===")
    _summarize("front_depth", front_times)
    _summarize("dual_batch", batch_times)
    _summarize("dual_parallel4", parallel_times)

    if batch_times and parallel_times:
        avg_batch = statistics.mean(batch_times)
        avg_parallel = statistics.mean(parallel_times)
        delta = avg_parallel - avg_batch
        ratio = (avg_parallel / avg_batch) if avg_batch > 1e-6 else float("inf")
        print(
            f"parallel_vs_batch delta={delta:.3f}s  ratio={ratio:.2f}x"
        )


if __name__ == "__main__":
    main()
