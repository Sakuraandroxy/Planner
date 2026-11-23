"""评测子系统配置。

完全独立于主 Planner 仓库：只用标准库 + 可选 pyyaml。默认值即可直接跑；
需要覆盖时用命令行参数。config 文件（eval.yaml）仅在装了 pyyaml 时才解析。
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path


# 场景名 -> 相对 root_path 的启动脚本（对照 3DG-VLN AirVLNSimulatorServerTool.env_exec_path_dict）
_CLOSELOOP = ["NYCEnvironmentMegapa", "TropicalIsland", "NewYorkCity",
              "ModularPark", "ModularEuropean", "ModernCityMap"]
_CARLA_TOWNS = ["Town01", "Town02", "Town03", "Town04", "Town05",
                "Town06", "Town07", "Town10HD", "Town15"]


def scene_launcher_map() -> dict[str, str]:
    m = {name: f"closeloop_envs/{name}.sh" for name in _CLOSELOOP}
    for t in _CARLA_TOWNS:
        m[f"Carla_{t}"] = f"carla_town_envs/{t}/LinuxNoEditor/CarlaUE4.sh"
    return m


@dataclass
class MoveSettings:
    velocity: float = 5.0
    lookahead: int = 3
    adaptive_lookahead: int = 1
    chunk_timeout_s: float = 45.0
    reach_tol_m: float = 1.0


@dataclass
class CaptureSettings:
    width: int = 256
    height: int = 256
    fov: int = 90


@dataclass
class PlannerSettings:
    # sanity = 直线朝目标（Phase 0 验证用）
    # real   = 在进程内复用 Planner 的 visual-goal 工作流（真检测+真VLM规划+完成判定+滑窗）
    # http/import = 预留（把观测 POST 到外部规划器端点）
    mode: str = "sanity"
    endpoint: str = "http://127.0.0.1:8000"
    timeout_s: float = 30.0
    # real 模式：位姿采样频率（写 metric 帧）；关闭滑窗时工作流走 stopped-segments（更简单）
    sampler_hz: float = 5.0
    disable_sliding_window: bool = False


@dataclass
class Config:
    # 环境
    root_path: str = "/data/sakura/data/TravelUAV_env"
    dataset_root: str = "/data/sakura/data/UAV-VLN-FOV"
    split: str = "test"                # 数据集切分：test / unscene / unobject
    output_dir: str = "/data/sakura/AVLN/Planner/eval/output"
    dashboard_port: int = 8010         # Phase 2 仪表盘端口
    # 0.0.0.0 = 局域网可访问（控制台用 LAN IP 打开时，跳转 location.hostname:port 才通）
    dashboard_host: str = "0.0.0.0"
    scenes: list[str] = field(default_factory=list)   # 空 = 数据集里全部场景
    # 空 = 不按 uuid 过滤；否则只跑这些 episode（"点哪个跑哪个"）
    uuids: list[str] = field(default_factory=list)
    gpus: list[int] = field(default_factory=lambda: [0])

    # 实验参数（与 3DG-VLN / metric.py 对齐）
    success_radius_m: float = 10.0
    max_ne_distance_m: float = 500.0
    max_steps: int = 40                # 每回合最多飞几个 5-航点 chunk
    chunk_size: int = 5
    seconds_between_episodes: float = 0.0

    # 子配置
    move: MoveSettings = field(default_factory=MoveSettings)
    capture: CaptureSettings = field(default_factory=CaptureSettings)
    planner: PlannerSettings = field(default_factory=PlannerSettings)
    clock_speed: int = 1               # >1 会让本 build 物理时间步失控（自由落体），保持 1
    lockstep: bool = False             # ClockSpeed=1 下实时运行即可

    # 官方打分脚本
    metric_py: str = "/data/sakura/AVLN/3DG-VLN-main/utils/metric.py"

    # 控制器
    limit_episodes: int = 0            # 0 = 不限

    @property
    def meta_dir(self) -> str:
        return str(Path(self.dataset_root) / "meta")

    @property
    def split_dir(self) -> str:
        return str(Path(self.dataset_root) / self.split)

    @property
    def object_info_path(self) -> str:
        return str(Path(self.dataset_root) / "meta" / "map_spawnarea_info.json")

    def resolve_scenes(self) -> dict[str, str]:
        return scene_launcher_map()


def _apply_yaml(cfg: Config, path: str) -> None:
    try:
        import yaml  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise SystemExit(f"--config 需要 pyyaml，但导入失败: {exc}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    for key, value in data.items():
        if key == "move" and isinstance(value, dict):
            for k, v in value.items():
                setattr(cfg.move, k, v)
        elif key == "capture" and isinstance(value, dict):
            for k, v in value.items():
                setattr(cfg.capture, k, v)
        elif key == "planner" and isinstance(value, dict):
            for k, v in value.items():
                setattr(cfg.planner, k, v)
        elif hasattr(cfg, key):
            setattr(cfg, key, value)


def parse_args(argv=None) -> CoordinateArgs:
    p = argparse.ArgumentParser(description="UAV-VLN 评测子系统")
    p.add_argument("--config", default=None, help="eval.yaml 路径（需 pyyaml）")
    p.add_argument("--root-path", default=None)
    p.add_argument("--dataset-root", default=None)
    p.add_argument("--output-dir", default=None)
    p.add_argument("--dashboard-port", type=int, default=None, help="仪表盘 HTTP 端口（默认 8010）")
    p.add_argument("--dashboard-host", default=None, help="仪表盘绑定地址（默认 0.0.0.0，局域网可访问）")
    p.add_argument("--scenes", default=None, help="逗号分隔；缺省=数据集全部场景")
    p.add_argument("--uuids", default=None, help="逗号分隔；只跑指定 episode（点哪个跑哪个）")
    p.add_argument("--gpus", default=None, help="逗号分隔，如 0 或 0,1")
    p.add_argument("--success-radius", type=float, default=None)
    p.add_argument("--max-steps", type=int, default=None)
    p.add_argument("--limit-episodes", type=int, default=None, help="只跑前 N 个 episode（0=不限）")
    p.add_argument("--planner-mode", choices=["sanity", "real", "http", "import"], default=None)
    p.add_argument("--planner-endpoint", default=None)
    p.add_argument("--move-velocity", type=float, default=None)
    p.add_argument("--clock-speed", type=int, default=None)
    p.add_argument("mode", choices=["one", "batch", "serve"], nargs="?",
                   default="batch", help="one=单 episode；batch=批量；serve=只起仪表盘")

    ns = p.parse_args(argv)
    cfg = Config()
    if ns.config:
        _apply_yaml(cfg, ns.config)
    if ns.root_path:
        cfg.root_path = ns.root_path
    if ns.dataset_root:
        cfg.dataset_root = ns.dataset_root
    if ns.output_dir:
        cfg.output_dir = ns.output_dir
    if ns.dashboard_port is not None:
        cfg.dashboard_port = ns.dashboard_port
    if ns.dashboard_host:
        cfg.dashboard_host = ns.dashboard_host
    if ns.scenes:
        cfg.scenes = [s.strip() for s in ns.scenes.split(",") if s.strip()]
    if ns.uuids:
        cfg.uuids = [s.strip() for s in ns.uuids.split(",") if s.strip()]
    if ns.gpus:
        cfg.gpus = [int(s) for s in ns.gpus.split(",") if s.strip()]
    if ns.success_radius is not None:
        cfg.success_radius_m = ns.success_radius
    if ns.max_steps is not None:
        cfg.max_steps = ns.max_steps
    if ns.limit_episodes is not None:
        cfg.limit_episodes = ns.limit_episodes
    if ns.planner_mode:
        cfg.planner.mode = ns.planner_mode
    if ns.planner_endpoint:
        cfg.planner.endpoint = ns.planner_endpoint
    if ns.move_velocity is not None:
        cfg.move.velocity = ns.move_velocity
    if ns.clock_speed is not None:
        cfg.clock_speed = ns.clock_speed

    return CoordinateArgs(cfg=cfg, mode=ns.mode)


@dataclass
class CoordinateArgs:
    cfg: Config
    mode: str
