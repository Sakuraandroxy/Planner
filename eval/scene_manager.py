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
"""

import os
import shlex
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path, PurePosixPath
from typing import Dict, List, Optional

from config import cfg
from sim.airsim_settings import build_airsim_settings_with_overrides, LOCAL_AIRSIM_SETTINGS_PATH


def _scene_name_from_path(path_str: str, remote: bool = False) -> str:
    path_obj = PurePosixPath(path_str) if remote else Path(path_str)
    parent = path_obj.parent
    grandparent = parent.parent
    return grandparent.name if parent.name == "LinuxNoEditor" else parent.name


def _scan_scenes(env_root: str) -> Dict[str, str]:
    """递归扫描 env_root 下所有 .sh 文件, 建立 {场景名: .sh路径} 映射。"""
    scene_map = {}
    root = Path(env_root)
    if not root.exists():
        return scene_map
    for sh_file in root.rglob("*.sh"):
        scene_name = _scene_name_from_path(str(sh_file))
        scene_map[scene_name] = str(sh_file)
    return scene_map


class SceneManager:

    def __init__(self, env_root: str, gpu_id: int = 0,
                 startup_wait: int | None = None, airsim_port: int | None = None,
                 remote_host: str | None = None, remote_user: str | None = None,
                 remote_port: int = 22):
        if startup_wait is None:
            startup_wait = int(cfg.get("EVAL", {}).get("SCENE_STARTUP_WAIT", 25))
        if airsim_port is None:
            airsim_port = int(cfg.get("SIM", {}).get("AIRSIM_PORT", 41451))
        self.env_root = env_root
        self.gpu_id = gpu_id
        self.startup_wait = startup_wait
        self.airsim_port = airsim_port
        self.remote_host = (remote_host or "").strip()
        self.remote_user = (remote_user or "").strip()
        self.remote_port = int(remote_port)
        self.remote_enabled = bool(self.remote_host)
        self._proc: Optional[subprocess.Popen] = None
        self._current_scene: str = ""
        self._remote_state_dir = "/tmp/unilavira_scene_manager"
        self._remote_pid_path = f"{self._remote_state_dir}/current_scene.pid"
        self._remote_log_path = f"{self._remote_state_dir}/current_scene.log"
        self._remote_settings_path = f"{self._remote_state_dir}/settings.json"
        self._tunnel_proc: Optional[subprocess.Popen] = None
        self._auto_ssh_tunnel = bool(cfg.get("EVAL", {}).get("AUTO_SSH_TUNNEL", True))
        if self.remote_enabled:
            self._scene_map = self._scan_remote_scenes(env_root)
        else:
            self._scene_map = _scan_scenes(env_root)
        self._settings_dir = tempfile.mkdtemp(prefix="airsim_settings_")

    @property
    def available_scenes(self) -> List[str]:
        return sorted(self._scene_map.keys())

    @property
    def _ssh_target(self) -> str:
        if not self.remote_enabled:
            return ""
        if self.remote_user:
            return f"{self.remote_user}@{self.remote_host}"
        return self.remote_host

    def _run_ssh(self, remote_cmd: str, check: bool = True) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["ssh", "-p", str(self.remote_port), self._ssh_target, remote_cmd],
            capture_output=True, text=True, encoding="utf-8", errors="replace", check=check
        )

    def _run_scp_to_remote(self, local_path: str, remote_path: str):
        subprocess.run(
            ["scp", "-P", str(self.remote_port), local_path, f"{self._ssh_target}:{remote_path}"],
            capture_output=True, text=True, encoding="utf-8", errors="replace", check=True
        )

    def _should_use_tunnel(self) -> bool:
        if not self.remote_enabled or not self._auto_ssh_tunnel:
            return False
        airsim_host = (cfg.get("SIM", {}).get("AIRSIM_IP", "") or "").strip().lower()
        return airsim_host in ("", "localhost", "127.0.0.1")

    def _ensure_tunnel(self):
        """Expose remote AirSim RPC as local localhost:port for Windows clients."""
        if not self._should_use_tunnel():
            return
        if self._tunnel_proc is not None and self._tunnel_proc.poll() is None:
            return
        cmd = [
            "ssh",
            "-p", str(self.remote_port),
            "-o", "ExitOnForwardFailure=yes",
            "-N",
            "-L", f"{self.airsim_port}:127.0.0.1:{self.airsim_port}",
            self._ssh_target,
        ]
        self._tunnel_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(0.5)
        if self._tunnel_proc.poll() is not None:
            print(f"  [SceneManager] ⚠️ SSH tunnel failed, local port {self.airsim_port} may be occupied")
            self._tunnel_proc = None
            return
        print(f"  [SceneManager] SSH tunnel: localhost:{self.airsim_port} -> {self._ssh_target}:127.0.0.1:{self.airsim_port}")

    def _stop_tunnel(self):
        if self._tunnel_proc is None:
            return
        if self._tunnel_proc.poll() is None:
            self._tunnel_proc.terminate()
            try:
                self._tunnel_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._tunnel_proc.kill()
        self._tunnel_proc = None

    def _scan_remote_scenes(self, env_root: str) -> Dict[str, str]:
        scene_map: Dict[str, str] = {}
        try:
            result = self._run_ssh(
                f"find {shlex.quote(env_root)} -type f -name '*.sh'"
            )
        except Exception as exc:
            print(f"  [SceneManager] ⚠️ 远程扫描场景失败: {exc}")
            return scene_map
        for line in result.stdout.splitlines():
            path_str = line.strip()
            if not path_str:
                continue
            scene_map[_scene_name_from_path(path_str, remote=True)] = path_str
        return scene_map

    def _tail_remote_log(self, lines: int = 20) -> str:
        if not self.remote_enabled:
            return ""
        try:
            cmd = f"test -f {self._remote_log_path} && tail -n {lines} {self._remote_log_path} || true"
            result = self._run_ssh(
                f"bash -lc {shlex.quote(cmd)}",
                check=False,
            )
            return (result.stdout or "").strip()
        except Exception:
            return ""

    def _remote_debug_snapshot(self) -> str:
        if not self.remote_enabled:
            return ""
        try:
            debug_cmd = f"""
echo "[ssh_target] {self._ssh_target}"
echo "[pid_file]"
if [ -f {shlex.quote(self._remote_pid_path)} ]; then
  cat {shlex.quote(self._remote_pid_path)}
else
  echo "missing"
fi
echo "[ps_pid]"
if [ -f {shlex.quote(self._remote_pid_path)} ]; then
  ps -fp "$(cat {shlex.quote(self._remote_pid_path)})" || true
else
  echo "missing"
fi
echo "[log_file]"
ls -l {shlex.quote(self._remote_log_path)} 2>/dev/null || echo "missing"
echo "[settings_file]"
ls -l {shlex.quote(self._remote_settings_path)} 2>/dev/null || echo "missing"
echo "[scene_processes]"
ps -ef | grep -E 'LinuxNoEditor|Carla|AirSim|UE4' | grep -v grep | head -n 20 || true
"""
            result = self._run_ssh(
                f"bash -lc {shlex.quote(debug_cmd)}",
                check=False,
            )
            return ((result.stdout or "") + (result.stderr or "")).strip()
        except Exception:
            return ""

    def _remote_port_listening(self) -> bool:
        try:
            cmd = f'ss -ltn 2>/dev/null | grep -q ":{self.airsim_port} "'
            result = self._run_ssh(
                f"bash -lc {shlex.quote(cmd)}",
                check=False,
            )
            return result.returncode == 0
        except Exception:
            return False

    def _local_port_open(self, host: str, timeout: float = 2.0) -> bool:
        try:
            with socket.create_connection((host, self.airsim_port), timeout=timeout):
                return True
        except OSError:
            return False

    def _wait_until_ready(self) -> bool:
        deadline = time.time() + self.startup_wait
        sim_cfg = cfg.get("SIM", {})
        airsim_host = (sim_cfg.get("AIRSIM_IP", "") or "").strip()
        if not airsim_host:
            airsim_host = "127.0.0.1"
        saw_remote_listen = False
        saw_local_connect = False
        saw_alive = False

        while time.time() < deadline:
            alive = self._is_alive() if self.remote_enabled else True
            saw_alive = saw_alive or alive

            port_ok = self._remote_port_listening() if self.remote_enabled else self._local_port_open(airsim_host, timeout=1.0)
            saw_remote_listen = saw_remote_listen or port_ok
            if port_ok:
                if self.remote_enabled:
                    self._ensure_tunnel()
                    if self._local_port_open(airsim_host, timeout=1.0):
                        saw_local_connect = True
                        return True
                else:
                    return True
            time.sleep(2.0)
        if self.remote_enabled:
            print(f"  [SceneManager] debug: saw_alive={saw_alive} remote_listen={saw_remote_listen} local_connect={saw_local_connect}")
        return False

    def _write_settings(self) -> str:
        """生成 settings.json 并返回路径。"""
        settings = build_airsim_settings_with_overrides()
        settings["ApiServerPort"] = self.airsim_port
        path = os.path.join(self._settings_dir, "settings.json")
        with open(path, "w") as f:
            import json
            json.dump(settings, f)
        print(f"  [SceneManager] base settings: {LOCAL_AIRSIM_SETTINGS_PATH}")
        return path

    def start(self, scene_name: str) -> bool:
        if scene_name == self._current_scene and self._is_alive():
            print(f"  [SceneManager] 复用当前场景: {scene_name}")
            return True

        sh_path = self._scene_map.get(scene_name)
        if sh_path is None:
            for key, path in self._scene_map.items():
                if scene_name.lower() in key.lower() or key.lower() in scene_name.lower():
                    sh_path = path
                    break
        if sh_path is None:
            print(f"  [SceneManager] ⚠️ 未找到场景 '{scene_name}'")
            return False

        self.stop()

        settings_path = self._write_settings()
        print(f"  [SceneManager] 启动场景: {scene_name}")
        print(f"  [SceneManager] scene script: {sh_path}")
        print(f"  [SceneManager] settings: {settings_path}")

        if self.remote_enabled:
            try:
                self._run_ssh(f"mkdir -p {shlex.quote(self._remote_state_dir)}")
                self._run_scp_to_remote(settings_path, self._remote_settings_path)
                launch_cmd = "\n".join([
                    f"mkdir -p {shlex.quote(self._remote_state_dir)}",
                    f"if [ -f {shlex.quote(self._remote_pid_path)} ]; then",
                    f"  old_pid=$(cat {shlex.quote(self._remote_pid_path)})",
                    "  pkill -TERM -P \"$old_pid\" >/dev/null 2>&1 || true",
                    "  kill -TERM \"$old_pid\" >/dev/null 2>&1 || true",
                    f"  rm -f {shlex.quote(self._remote_pid_path)}",
                    "fi",
                    f"printf '%s\\n' \"[launch] $(date -Is) scene={scene_name} script={sh_path}\" > {shlex.quote(self._remote_log_path)}",
                    (
                        f"nohup setsid bash {shlex.quote(sh_path)} -RenderOffscreen "
                        f"-GraphicsAdapter={self.gpu_id} "
                        f"-settings={shlex.quote(self._remote_settings_path)} "
                        f">> {shlex.quote(self._remote_log_path)} 2>&1 < /dev/null &"
                    ),
                    f"echo $! > {shlex.quote(self._remote_pid_path)}",
                ])
                self._run_ssh(f"bash -lc {shlex.quote(launch_cmd)}")
            except Exception as exc:
                print(f"  [SceneManager] ⚠️ 远程启动失败: {exc}")
                return False
        else:
            cmd = ["bash", sh_path, "-RenderOffscreen",
                   f"-GraphicsAdapter={self.gpu_id}",
                   f"-settings={settings_path}"]
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        self._current_scene = scene_name
        print(f"  [SceneManager] 等待 UE4/AirSim 就绪 (最多 {self.startup_wait}s)...")
        if self._wait_until_ready():
            print(f"  [SceneManager] 就绪, AirSim 端口 {self.airsim_port}")
            return True

        print(f"  [SceneManager] ⚠️ 等待 AirSim 端口 {self.airsim_port} 就绪超时")
        if self.remote_enabled:
            tail = self._tail_remote_log()
            if tail:
                print("  [SceneManager] 远端日志尾部:")
                for line in tail.splitlines():
                    print(f"    {line}")
            else:
                print("  [SceneManager] 远端日志为空或尚未生成")
            snapshot = self._remote_debug_snapshot()
            if snapshot:
                print("  [SceneManager] 远端调试信息:")
                for line in snapshot.splitlines():
                    print(f"    {line}")
        return False

    def stop(self):
        if self.remote_enabled:
            if self._current_scene:
                print(f"  [SceneManager] 关闭场景: {self._current_scene}")
            try:
                stop_cmd = (
                    f"if [ -f {shlex.quote(self._remote_pid_path)} ]; then "
                    f"old_pid=$(cat {shlex.quote(self._remote_pid_path)}); "
                    f"pkill -TERM -P \"$old_pid\" >/dev/null 2>&1 || true; "
                    f"kill -TERM \"$old_pid\" >/dev/null 2>&1 || true; "
                    f"rm -f {shlex.quote(self._remote_pid_path)}; fi"
                )
                self._run_ssh(f"bash -lc {shlex.quote(stop_cmd)}", check=False)
            except Exception:
                pass
            self._current_scene = ""
            time.sleep(2)
            return
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
            self._proc = None
            self._current_scene = ""
            time.sleep(2)

    def _is_alive(self) -> bool:
        if self.remote_enabled:
            if not self._current_scene:
                return False
            try:
                result = self._run_ssh(
                    f"bash -lc {shlex.quote(f'if [ -f {self._remote_pid_path} ] && kill -0 $(cat {self._remote_pid_path}) >/dev/null 2>&1; then echo alive; fi')}",
                    check=False,
                )
            except Exception:
                return False
            return "alive" in (result.stdout or "")
        if self._proc is None:
            return False
        return self._proc.poll() is None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.stop()

    def __del__(self):
        self.stop()
        self._stop_tunnel()
        import shutil
        shutil.rmtree(self._settings_dir, ignore_errors=True)
