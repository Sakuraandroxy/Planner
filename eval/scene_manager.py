"""
eval/scene_manager.py — 轻量 UE4 场景管理器。

替代 3DG-VLN 的 AirVLNSimulatorServerTool，单进程串行管理 UE4 生命周期。
不依赖 msgpack-rpc、不依赖 multiprocessing。

用法:
    manager = SceneManager(env_root="/data/sakura/data/TravelUAV_env", gpu_id=0)
    manager.start("ModularEuropean")  # 启动 UE4, 等15s加载
    # ... 跑评测 ...
    manager.start("Carla_Town01")     # 自动杀掉旧进程, 启动新场景
    manager.stop()

场景映射:
    自动从 env_root 扫描所有 .sh 文件, 建立场景名 → .sh 路径映射。
    数据集里的 scene 目录名必须能匹配到映射中的 key。
"""

import os
import subprocess
import time
import signal
from pathlib import Path
from typing import Dict, List, Optional


def _scan_scenes(env_root: str) -> Dict[str, str]:
    """递归扫描 env_root 下所有 .sh 文件, 建立 {场景名: .sh路径} 映射。

    场景名从 .sh 所在目录的父目录名提取 (如 ModularEuropean/LinuxNoEditor/ModularEuropean.sh → ModularEuropean)。
    如果 key 重复, 后发现的覆盖前者。
    """
    scene_map = {}
    root = Path(env_root)
    if not root.exists():
        return scene_map

    for sh_file in root.rglob("*.sh"):
        # LinuxNoEditor/xxx.sh → 取 LinuxNoEditor 的父目录名作为场景名
        parent = sh_file.parent
        grandparent = parent.parent
        scene_name = grandparent.name if parent.name == "LinuxNoEditor" else parent.name
        scene_map[scene_name] = str(sh_file)

    return scene_map


class SceneManager:
    """管理单个 UE4 场景的生命周期: 启动 → 等待就绪 → 杀死。

    特性:
      - 同一场景连续调用 start() 不重启 (复用当前进程)
      - 不同场景自动 kill 旧进程再启动新进程
      - stop() 清理残留进程
    """

    def __init__(self, env_root: str, gpu_id: int = 0,
                 startup_wait: int = 15, airsim_port: int = 41451):
        self.env_root = env_root
        self.gpu_id = gpu_id
        self.startup_wait = startup_wait
        self.airsim_port = airsim_port
        self._proc: Optional[subprocess.Popen] = None
        self._current_scene: str = ""
        self._scene_map = _scan_scenes(env_root)

    @property
    def available_scenes(self) -> List[str]:
        """返回所有可用场景名。"""
        return sorted(self._scene_map.keys())

    def get_matching_scenes(self, dataset_scenes: List[str]) -> Dict[str, str]:
        """将数据集的 scene 目录名匹配到 .sh 路径。

        匹配规则: 优先精确匹配, 回退到模糊匹配 (如 "Carla_Town01" 匹配包含 "Town01" 的 key)。
        返回 {数据集scene名: .sh路径}。
        """
        matched = {}
        for ds_scene in dataset_scenes:
            if ds_scene in self._scene_map:
                matched[ds_scene] = self._scene_map[ds_scene]
                continue
            # 模糊匹配
            for key, path in self._scene_map.items():
                if ds_scene.lower() in key.lower() or key.lower() in ds_scene.lower():
                    matched[ds_scene] = path
                    break
        return matched

    def start(self, scene_name: str) -> bool:
        """启动指定场景的 UE4 进程。

        如果当前已是同一场景, 直接返回 True (复用)。
        如果场景名查找失败, 返回 False。
        """
        if scene_name == self._current_scene and self._is_alive():
            print(f"  [SceneManager] 复用当前场景: {scene_name}")
            return True

        # 查找场景路径
        sh_path = self._scene_map.get(scene_name)
        if sh_path is None:
            # 模糊查找
            for key, path in self._scene_map.items():
                if scene_name.lower() in key.lower() or key.lower() in scene_name.lower():
                    sh_path = path
                    break
        if sh_path is None:
            print(f"  [SceneManager] ⚠️ 未找到场景 '{scene_name}', 可用: {list(self._scene_map.keys())[:5]}...")
            return False

        # 杀掉旧进程
        self.stop()

        # 启动新进程
        print(f"  [SceneManager] 启动场景: {scene_name} ({sh_path})")
        try:
            self._proc = subprocess.Popen(
                ["bash", sh_path, "-RenderOffscreen",
                 f"-GraphicsAdapter={self.gpu_id}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,  # 独立进程组, 方便 kill
            )
        except FileNotFoundError:
            print(f"  [SceneManager] ⚠️ bash 不可用, 尝试直接执行")
            self._proc = subprocess.Popen(
                [sh_path, "-RenderOffscreen",
                 f"-GraphicsAdapter={self.gpu_id}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )

        self._current_scene = scene_name
        print(f"  [SceneManager] 等待 UE4 加载 ({self.startup_wait}s)...")
        time.sleep(self.startup_wait)
        print(f"  [SceneManager] 就绪, AirSim 端口 {self.airsim_port}")
        return True

    def stop(self):
        """杀掉当前 UE4 进程及其子进程。"""
        if self._proc is not None:
            print(f"  [SceneManager] 关闭场景: {self._current_scene}")
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
                self._proc.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except Exception as e:
                print(f"  [SceneManager] 关闭异常: {e}")
            self._proc = None
            self._current_scene = ""
            time.sleep(2)

    def _is_alive(self) -> bool:
        """检查当前进程是否存活。"""
        if self._proc is None:
            return False
        return self._proc.poll() is None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.stop()

    def __del__(self):
        self.stop()
