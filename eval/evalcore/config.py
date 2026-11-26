"""评测配置。默认读取 config/eval.yml，命令行参数可以覆盖文件值。"""
from __future__ import annotations

import argparse
import os
from dataclasses import dataclass, field
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "eval.yml"


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
    # 评测不再有独立规划器：始终在进程内复用 Planner 的 visual-goal 工作流
    # （真检测+真VLM规划+完成判定+滑窗），见 :mod:`evalcore.planner_runtime`。
    # profile 选择哪套 config/core/<profile>/base.yaml（世界模型=default、普通VLM=vlm）。
    # 这是主界面当前 profile 的只读镜像：起评测时由 dashboard 读主服务 /api/profile 覆盖。
    profile: str = "default"
    # 位姿采样频率（写 metric 帧）；关闭滑窗时工作流走 stopped-segments（更简单）
    sampler_hz: float = 5.0
    disable_sliding_window: bool = False


@dataclass
class Config:
    # 外部资源相对 root，运行产物相对仓库根。
    root: str = "."
    root_path: str = "TravelUAV_env"
    dataset_root: str = "UAV-VLN-FOV"
    split: str = "test"                # 数据集切分：test / unscene / unobject
    output_dir: str = "eval/output"
    saved_dir: str = "eval/saved"
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

    config_path: Path = field(default=DEFAULT_CONFIG_PATH, init=False)

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


def _apply_yaml(cfg: Config, path: Path) -> None:
    import yaml

    with path.open(encoding="utf-8") as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise ValueError(f"无法解析评测配置 {path}: {exc}") from exc
    if data is None:
        return
    if not isinstance(data, dict):
        raise ValueError(f"评测配置必须是 YAML 映射: {path}")
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
        elif key != "config_path" and hasattr(cfg, key):
            setattr(cfg, key, value)


def _absolute_path(value: str | Path, base: Path) -> Path:
    return (base / Path(value).expanduser()).resolve()


def load_config(path: str | Path | None = None) -> Config:
    config_path = _absolute_path(
        path or os.environ.get("AVLN_EVAL_CONFIG") or DEFAULT_CONFIG_PATH,
        Path.cwd(),
    )
    cfg = Config()
    _apply_yaml(cfg, config_path)
    cfg.config_path = config_path
    root = _absolute_path(cfg.root, config_path.parent)
    cfg.root = str(root)
    cfg.root_path = str(_absolute_path(cfg.root_path, root))
    cfg.dataset_root = str(_absolute_path(cfg.dataset_root, root))
    cfg.output_dir = str(_absolute_path(cfg.output_dir, REPO_ROOT))
    cfg.saved_dir = str(_absolute_path(
        os.environ.get("AVLN_EVAL_SAVED") or cfg.saved_dir, REPO_ROOT
    ))
    cfg.dashboard_port = int(os.environ.get("AVLN_EVAL_PORT") or cfg.dashboard_port)
    return cfg


def parse_args(argv=None) -> CoordinateArgs:
    p = argparse.ArgumentParser(description="UAV-VLN 评测子系统")
    p.add_argument("--config", default=None, help="评测配置路径（默认 config/eval.yml）")
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
    p.add_argument("--planner-profile", default=None,
                   help="规划器 profile（config/core/<profile>），缺省跟随主服务当前 profile")
    p.add_argument("--move-velocity", type=float, default=None)
    p.add_argument("--clock-speed", type=int, default=None)
    p.add_argument("mode", choices=["one", "batch", "serve"], nargs="?",
                   default="batch", help="one=单 episode；batch=批量；serve=只起仪表盘")

    ns = p.parse_args(argv)
    cfg = load_config(ns.config)
    for name in ("root_path", "dataset_root", "output_dir"):
        value = getattr(ns, name)
        if value:
            setattr(cfg, name, str(_absolute_path(value, Path.cwd())))
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
    if ns.planner_profile:
        cfg.planner.profile = ns.planner_profile
    if ns.move_velocity is not None:
        cfg.move.velocity = ns.move_velocity
    if ns.clock_speed is not None:
        cfg.clock_speed = ns.clock_speed

    return CoordinateArgs(cfg=cfg, mode=ns.mode)


@dataclass
class CoordinateArgs:
    cfg: Config
    mode: str
