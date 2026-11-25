"""Phase 2 在线仪表盘：纯标准库 http.server，轮询磁盘上的 run 目录。

与评测进程解耦：评测把 progress.json / metrics.json / per-episode log 写到 run 目录，
本服务只读这些文件并渲染网页。端口默认 8010。

端点：
  GET /                         单页前端
  GET /static/<file>            前端静态资源
  GET /api/runs                 所有 run 及状态
  GET /api/state?run=NAME       当前 run 的进度/指标/episode 列表
  GET /api/traj?run=&id=        某 episode 的预测轨迹 + 目标 + 成功半径
  GET /api/frame?run=&id=&cam=  某 episode 最新 front/down 缩略图 (png)
"""
from __future__ import annotations

import glob
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from .config import REPO_ROOT
from .dataset import load_episodes

_WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web")
# run.py 所在目录（evalcore 的上一级），spawn 评测子进程时作 cwd。
_EVAL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_CONTENT_TYPES = {".html": "text/html; charset=utf-8", ".js": "application/javascript",
                  ".css": "text/css", ".png": "image/png", ".json": "application/json"}

# 规划器主服务的 HTTP 基址（拉起 dashboard 时由主服务经 AVLN_PLANNER_API 传下来）。
# 评测没有独立规划器：起评测前读主服务 /api/profile 拿当前 profile（世界模型/普通VLM），
# 透传给 run.py，保证与主界面选择一致。
_PLANNER_API = os.environ.get("AVLN_PLANNER_API", "http://127.0.0.1:8000").rstrip("/")

# --- 发跑评测：一次只允许一个子进程 ------------------------------------------
# 与只读看板分离：这些状态属于"当前发起的评测"，看板数据仍来自磁盘 run 目录。
_ACTIVE_LOCK = threading.Lock()
_ACTIVE = {"proc": None, "scene": None, "uuids": None, "profile": None, "log": None}
_EPISODE_CACHE: list | None = None
_EPISODE_LOCK = threading.Lock()


def _planner_profile() -> dict:
    """读主服务当前 profile（只读镜像）；连不上则 online=False，评测回退 default。"""
    planner_port = None
    try:
        planner_port = urlparse(_PLANNER_API).port
        req = urllib.request.Request(_PLANNER_API + "/api/profile", method="GET")
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        return {"online": True,
                "planner_port": planner_port,
                "current": data.get("current"),
                "profiles": data.get("profiles") or []}
    except Exception:
        return {"online": False, "planner_port": planner_port, "current": None, "profiles": []}


def _all_episodes(cfg):
    """读取全部 episode 并按场景分组缓存（dataset 图不大，读一次即可）。"""
    global _EPISODE_CACHE
    with _EPISODE_LOCK:
        if _EPISODE_CACHE is None:
            try:
                _EPISODE_CACHE = load_episodes(cfg)
            except Exception:
                _EPISODE_CACHE = []
        return _EPISODE_CACHE


def _public_episode(ep) -> dict:
    return {
        "uuid": ep.uuid,
        "scene": ep.scene,
        "instruction": ep.instruction,
        "object_position": ep.object_position,
    }


def _dataset_summary(cfg) -> dict:
    episodes = _all_episodes(cfg)
    counts: dict[str, int] = {}
    for ep in episodes:
        counts[ep.scene] = counts.get(ep.scene, 0) + 1
    return {
        "split": cfg.split,
        "scenes": [
            {"id": scene, "num_episodes": n}
            for scene, n in sorted(counts.items())
        ],
        "total": len(episodes),
    }


def _scene_episodes(cfg, scene: str) -> list[dict]:
    return [_public_episode(ep) for ep in _all_episodes(cfg) if ep.scene == scene]


def _run_status() -> dict:
    with _ACTIVE_LOCK:
        proc = _ACTIVE["proc"]
        active = proc is not None and proc.poll() is None
        return {
            "active": active,
            "scene": _ACTIVE["scene"],
            "uuids": _ACTIVE["uuids"],
            "profile": _ACTIVE["profile"],
            "returncode": None if proc is None else proc.poll(),
            "log": _ACTIVE["log"],
        }


def _start_run(cfg, scene: str, uuids: list[str], profile: str | None = None) -> dict:
    """在 eval 目录起 `run.py batch` 跑一个场景（或其中若干 episode）。

    profile 由前端下拉传入（可改，仅影响评测）；未指定时现读主服务当前 profile。
    """
    if not scene:
        raise ValueError("必须指定 scene")
    with _ACTIVE_LOCK:
        proc = _ACTIVE["proc"]
        if proc is not None and proc.poll() is None:
            raise RuntimeError(f"已有评测在跑（scene={_ACTIVE['scene']}），先等它结束或停止")
        profile = (profile or "").strip() or _planner_profile().get("current") or cfg.planner.profile
        argv = [
            sys.executable, "-u", "run.py", "batch", "--scenes", scene,
            "--config", str(cfg.config_path),
            "--root-path", cfg.root_path,
            "--dataset-root", cfg.dataset_root,
            "--output-dir", cfg.output_dir,
        ]
        if profile:
            argv += ["--planner-profile", profile]
        if uuids:
            argv += ["--uuids", ",".join(uuids)]
        log_path = os.path.join(_EVAL_DIR, "served_run_last.log")
        with open(log_path, "ab") as log:
            proc = subprocess.Popen(
                argv, cwd=_EVAL_DIR, stdout=log, stderr=subprocess.STDOUT,
                env=os.environ.copy(), start_new_session=True,
            )
        _ACTIVE.update(proc=proc, scene=scene, uuids=uuids, profile=profile, log=log_path)
        return {"started": True, "pid": proc.pid, "scene": scene,
                "uuids": uuids, "profile": profile, "log": log_path}


def _stop_run() -> dict:
    with _ACTIVE_LOCK:
        proc = _ACTIVE["proc"]
        if proc is None or proc.poll() is not None:
            return {"stopped": False, "reason": "没有正在跑的评测"}
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
        return {"stopped": True}



def _safe_run(cfg, run_name: str | None) -> str | None:
    """返回 output_dir 下一个合法 run 目录绝对路径；非法/不存在则 None。"""
    if not run_name:
        runs = _list_runs(cfg)
        return runs[0]["path"] if runs else None
    candidate = os.path.join(cfg.output_dir, run_name)
    real = os.path.realpath(candidate)
    if not real.startswith(os.path.realpath(cfg.output_dir) + os.sep):
        return None
    return real if os.path.isdir(real) else None


def _list_saved(saved_dir: str) -> list[dict]:
    """读取规划器保存的评测记录。"""
    out = []
    if not os.path.isdir(saved_dir):
        return out
    for path in glob.glob(os.path.join(saved_dir, "*", "record.json")):
        record = _read_json(path, {}) or {}
        out.append({
            "dir": os.path.basename(os.path.dirname(path)),
            "name": record.get("name") or os.path.basename(os.path.dirname(path)),
            "sr": record.get("sr"),
            "success": record.get("success"),
            "final_distance_m": record.get("final_distance_m"),
            "created": record.get("created"),
            "kind": record.get("kind"),
            "map": record.get("map"),
            "planner_profile": record.get("planner_profile"),
        })
    out.sort(key=lambda r: (r.get("created") or "", r.get("dir") or ""), reverse=True)
    return out


def _saved_record(saved_dir: str, name: str):
    base = os.path.realpath(saved_dir)
    real = os.path.realpath(os.path.join(base, name or ""))
    if not real.startswith(base + os.sep):
        return None
    return _read_json(os.path.join(real, "record.json"), None)


def _delete_saved(saved_dir: str, name: str) -> bool:
    """删除保存目录中的一条评测记录。"""
    name = (name or "").strip()
    if not name:
        return False
    base = os.path.realpath(saved_dir)
    real = os.path.realpath(os.path.join(base, name))
    if real == base or not real.startswith(base + os.sep):
        return False
    if not os.path.isdir(real):
        return False
    shutil.rmtree(real, ignore_errors=True)
    return not os.path.exists(real)


def _delete_run(cfg, run_name: str) -> bool:
    """删除一个 run 输出目录（output_dir/run_*），带穿越防护且不删正在跑的 run。"""
    run_name = (run_name or "").strip()
    if not run_name:
        return False
    run_path = _safe_run(cfg, run_name)  # 已校验在 output_dir 下且存在
    if not run_path:
        return False
    prog = _read_json(os.path.join(run_path, "progress.json"), {}) or {}
    if prog.get("state") == "running":
        raise RuntimeError("该 run 正在评测中，不能删除")
    shutil.rmtree(run_path, ignore_errors=True)
    return not os.path.exists(run_path)


def _read_json(path, default=None):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def _list_runs(cfg) -> list[dict]:
    runs = []
    for d in glob.glob(os.path.join(cfg.output_dir, "run_*")):
        if not os.path.isdir(d):
            continue
        prog = _read_json(os.path.join(d, "progress.json"), {}) or {}
        runs.append({
            "name": os.path.basename(d),
            "path": d,
            "mtime": os.path.getmtime(d),
            "state": prog.get("state", "unknown"),
            "done": prog.get("done"), "total": prog.get("total"),
        })
    runs.sort(key=lambda r: r["mtime"], reverse=True)
    return runs


def _run_summary(run_path: str, run: dict | None = None) -> dict:
    """一个 run 的批次汇总（历史列表顶部的「批次卡片」用）。

    SR / 碰撞数 / 成功数 / NE 一律由 episodes[] 的 success/collided/final_ne 现算：
    旧 metrics.json 的 metrics.sr 有解析 bug（写成了 OSR），不可信。
    OSR / SPL 仍取 metrics.metrics 里现成的值（可能为 None）。
    run 还没写 metrics.json（运行中）时，回退扫各集 result.json。
    """
    run = run or {}
    metrics = _read_json(os.path.join(run_path, "metrics.json"), {}) or {}
    eps = metrics.get("episodes")
    planner = metrics.get("planner")
    if eps is None:                          # 运行中：尚无 metrics.json
        eps = []
        for uuid, d in _episode_dirs(run_path).items():
            res = _read_json(os.path.join(d, "result.json"), {}) or {}
            eps.append({"uuid": uuid, "scene": res.get("scene"),
                        "success": res.get("success"), "collided": res.get("collided"),
                        "final_ne": res.get("final_ne"), "steps": res.get("steps"),
                        "planner_profile": res.get("planner_profile")})
        if planner is None:
            planner = next((e.get("planner_profile") for e in eps
                            if e.get("planner_profile")), None)

    n = len(eps)
    num_success = sum(1 for e in eps if e.get("success"))
    num_collided = sum(1 for e in eps if e.get("collided"))
    sr = round(100.0 * num_success / n, 2) if n else 0.0   # 权威：由 success 现算
    nes = []
    for e in eps:
        v = e.get("final_ne")
        if isinstance(v, bool):
            continue
        if isinstance(v, (int, float)) and not math.isnan(v):
            nes.append(float(v))
    ne_mean = round(sum(nes) / len(nes), 2) if nes else None
    mm = metrics.get("metrics") or {}
    scenes = sorted({e.get("scene") for e in eps if e.get("scene")})
    return {
        "name": os.path.basename(run_path),
        "state": run.get("state"),
        "mtime": run.get("mtime") or os.path.getmtime(run_path),
        "done": run.get("done"),
        "total": run.get("total"),
        "scenes": scenes,
        "planner": planner,
        "num_episodes": n,
        "num_success": num_success,
        "num_collided": num_collided,
        "num_failed": n - num_success,
        "sr": sr,
        "osr": mm.get("osr"),
        "ne_mean": ne_mean,
        "spl": mm.get("spl"),
    }


def _episode_dirs(run_path: str) -> dict:
    """uuid -> 实际目录（含 success_ 前缀）。"""
    m = {}
    for d in glob.glob(os.path.join(run_path, "*")):
        if not os.path.isdir(d):
            continue
        name = os.path.basename(d)
        uuid = name[len("success_"):] if name.startswith("success_") else name
        m[uuid] = d
    return m


def _recompute_metrics(eps: list[dict]) -> dict:
    """删了 episode 后按剩余集重算汇总：SR/NE 可重算，OSR/SPL 缺数据置 None(—)。"""
    n = len(eps)
    ok = sum(1 for e in eps if e.get("success"))
    sr = round(100.0 * ok / n, 2) if n else 0.0
    nes = [
        e.get("final_ne") for e in eps
        if isinstance(e.get("final_ne"), (int, float)) and not math.isnan(e["final_ne"])
    ]
    ne = round(sum(nes) / len(nes), 2) if nes else None
    return {"sr": sr, "osr": None, "ne_mean": ne, "spl": None, "num": n}


def _delete_episode(cfg, run_name: str, uuid: str) -> bool:
    """删除某个 run 下单个 episode 的结果目录，并把它从 metrics.json 剔除、重算汇总。"""
    run_name = (run_name or "").strip()
    uuid = (uuid or "").strip()
    if not run_name or not uuid:
        return False
    run_path = _safe_run(cfg, run_name)
    if not run_path:
        return False
    prog = _read_json(os.path.join(run_path, "progress.json"), {}) or {}
    if prog.get("state") == "running":
        raise RuntimeError("该 run 正在评测中，不能删除")
    d = _episode_dirs(run_path).get(uuid)
    if not d:
        return False
    shutil.rmtree(d, ignore_errors=True)
    mpath = os.path.join(run_path, "metrics.json")
    data = _read_json(mpath, None)
    if isinstance(data, dict) and isinstance(data.get("episodes"), list):
        eps = [e for e in data["episodes"] if e.get("uuid") != uuid]
        data["episodes"] = eps
        data["metrics"] = _recompute_metrics(eps)
        try:
            with open(mpath, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except OSError:
            pass
    return not os.path.exists(d)


def _state(cfg, run_path):
    prog = _read_json(os.path.join(run_path, "progress.json"), {}) or {}
    metrics = _read_json(os.path.join(run_path, "metrics.json"), {}) or {}
    eps = metrics.get("episodes")
    if eps is None:
        eps = []
        for uuid, d in _episode_dirs(run_path).items():
            r = _read_json(os.path.join(d, "result.json"), {}) or {}
            eps.append({"uuid": uuid, "scene": r.get("scene"), "success": r.get("success"),
                        "collided": r.get("collided"), "final_ne": r.get("final_ne"),
                        "steps": r.get("steps")})
    return {"run": os.path.basename(run_path), "progress": prog,
            "metrics": metrics.get("metrics"), "planner": metrics.get("planner"),
            "episodes": eps}


def _history(cfg, limit: int = 800) -> list[dict]:
    """把所有 run 的每集结果拍平成一条时间线（新 → 旧）。

    run 按 mtime 新到旧；同一批内已完成集在 metrics.json 里按完成顺序存，倒序让最后完成的在上。
    运行中的 run 还没有 metrics.json，回退扫各集 result.json（含 planner_profile）。
    """
    out = []
    for r in _list_runs(cfg):                # 已按 mtime 新到旧
        run_path = r["path"]
        metrics = _read_json(os.path.join(run_path, "metrics.json"), {}) or {}
        prof = metrics.get("planner")
        eps = metrics.get("episodes")
        if eps is None:                      # 运行中：metrics.json 还没写
            eps = []
            for uuid, d in _episode_dirs(run_path).items():
                res = _read_json(os.path.join(d, "result.json"), {}) or {}
                eps.append({"uuid": uuid, "scene": res.get("scene"),
                            "success": res.get("success"), "collided": res.get("collided"),
                            "final_ne": res.get("final_ne"), "steps": res.get("steps"),
                            "planner_profile": res.get("planner_profile")})
        for e in reversed(eps):              # 批内新→旧
            out.append({
                "run": r["name"],
                "scene": e.get("scene"),
                "uuid": e.get("uuid"),
                "success": e.get("success"),
                "collided": e.get("collided"),
                "final_ne": e.get("final_ne"),
                "steps": e.get("steps"),
                "planner": prof or e.get("planner_profile"),
            })
        if len(out) >= limit:
            break
    return out[:limit]


def _expert_positions(log_dir):
    """按帧序读专家轨迹位置（数据集录制轨迹）。"""
    out = []
    for fn in sorted(glob.glob(os.path.join(log_dir, "*.json"))):
        try:
            with open(fn, "r", encoding="utf-8") as f:
                out.append(json.load(f)["sensors"]["state"]["position"])
        except Exception:
            pass
    return out


def _traj(run_path, uuid):
    dirs = _episode_dirs(run_path)
    d = dirs.get(uuid)
    if not d:
        return {"error": "episode not found"}
    logs = sorted(glob.glob(os.path.join(d, "log", "*.json")))
    positions = []
    for fn in logs:
        try:
            with open(fn, "r", encoding="utf-8") as f:
                positions.append(json.load(f)["sensors"]["state"]["position"])
        except Exception:
            pass
    result = _read_json(os.path.join(d, "result.json"), {}) or {}
    metrics = _read_json(os.path.join(run_path, "metrics.json"), {}) or {}
    radius = (metrics.get("dataset") or {}).get("success_radius_m", 10.0)
    # 专家轨迹：新数据在 result.json；老数据回退读 ori_info.json 指向的原始 log/。
    expert = result.get("expert_positions") or []
    if not expert:
        ori = _read_json(os.path.join(d, "ori_info.json"), {}) or {}
        ori_dir = ori.get("ori_traj_dir")
        if ori_dir and os.path.isdir(os.path.join(ori_dir, "log")):
            expert = _expert_positions(os.path.join(ori_dir, "log"))
    return {"uuid": uuid, "positions": positions,
            "expert": expert,
            "target": result.get("target_position"),
            "start": positions[0] if positions else None,
            "radius": radius}


class _Handler(BaseHTTPRequestHandler):
    cfg = None  # 由 make_server 注入

    def log_message(self, *a):  # 静默
        pass

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode("utf-8")
        elif isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        # 看板/前端改完刷新即生效：禁止任何缓存（含浏览器启发式缓存）。
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        self.wfile.write(body)

    def _file(self, path):
        if not os.path.isfile(path):
            self._send(404, {"error": "not found"})
            return
        ext = os.path.splitext(path)[1]
        with open(path, "rb") as f:
            data = f.read()
        self._send(200, data, _CONTENT_TYPES.get(ext, "application/octet-stream"))

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        cfg = self.cfg
        if u.path in ("/", "/index.html"):
            self._file(os.path.join(_WEB_DIR, "index.html"))
        elif u.path == "/static/theme.css":
            self._file(REPO_ROOT / "console" / "theme.css")
        elif u.path.startswith("/static/"):
            name = os.path.basename(u.path)
            self._file(os.path.join(_WEB_DIR, name))
        elif u.path == "/api/runs":
            # 每个 run 附带批次汇总（SR 按 episodes[] 现算，见 _run_summary）。
            self._send(200, {"runs": [_run_summary(r["path"], r) for r in _list_runs(cfg)]})
        elif u.path == "/api/state":
            run_path = _safe_run(cfg, (q.get("run") or [None])[0])
            if not run_path:
                self._send(404, {"error": "run not found"}); return
            self._send(200, _state(cfg, run_path))
        elif u.path == "/api/history":
            # 平铺时间线：所有 run 的每集结果，按时间新→旧。
            self._send(200, {"episodes": _history(cfg)})
        elif u.path == "/api/traj":
            run_path = _safe_run(cfg, (q.get("run") or [None])[0])
            uuid = (q.get("id") or [""])[0]
            if not run_path:
                self._send(404, {"error": "run not found"}); return
            self._send(200, _traj(run_path, uuid))
        elif u.path == "/api/frame":
            run_path = _safe_run(cfg, (q.get("run") or [None])[0])
            uuid = (q.get("id") or [""])[0]
            cam = (q.get("cam") or ["front"])[0]
            d = _episode_dirs(run_path).get(uuid) if run_path else None
            fname = "frame_front.png" if cam == "front" else "frame_down.png"
            if not d:
                self._send(404, {"error": "episode not found"}); return
            self._file(os.path.join(d, fname))
        elif u.path == "/api/dataset":
            self._send(200, _dataset_summary(cfg))
        elif u.path == "/api/episodes":
            scene = (q.get("scene") or [""])[0]
            self._send(200, {"scene": scene, "episodes": _scene_episodes(cfg, scene)})
        elif u.path == "/api/runstatus":
            self._send(200, _run_status())
        elif u.path == "/api/eval/profile":
            # 只读镜像：主服务当前 profile（世界模型/普通VLM）及其可选项。
            self._send(200, _planner_profile())
        elif u.path == "/api/saved":
            self._send(200, {"records": _list_saved(cfg.saved_dir)})
        elif u.path == "/api/saved/traj":
            rec = _saved_record(cfg.saved_dir, (q.get("dir") or [""])[0])
            if rec is None:
                self._send(404, {"error": "record not found"})
            else:
                self._send(200, rec)
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        u = urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw or b"{}")
        except ValueError:
            body = {}
        if u.path == "/api/start":
            scene = str(body.get("scene") or "").strip()
            uuids = body.get("uuids") or []
            if isinstance(uuids, str):
                uuids = [x.strip() for x in uuids.split(",") if x.strip()]
            uuids = [str(x).strip() for x in uuids if str(x).strip()]
            profile = str(body.get("profile") or "").strip() or None
            try:
                self._send(200, _start_run(self.cfg, scene, uuids, profile))
            except RuntimeError as exc:
                self._send(409, {"error": str(exc)})
            except Exception as exc:
                self._send(500, {"error": str(exc)})
        elif u.path == "/api/stop":
            self._send(200, _stop_run())
        elif u.path == "/api/saved/delete":
            ok = _delete_saved(self.cfg.saved_dir, str(body.get("dir") or ""))
            self._send(200 if ok else 404, {"deleted": ok})
        elif u.path == "/api/runs/delete":
            try:
                ok = _delete_run(self.cfg, str(body.get("run") or ""))
            except RuntimeError as exc:
                self._send(409, {"error": str(exc)})
            else:
                self._send(200 if ok else 404, {"deleted": ok})
        elif u.path == "/api/episode/delete":
            try:
                ok = _delete_episode(
                    self.cfg, str(body.get("run") or ""), str(body.get("id") or "")
                )
            except RuntimeError as exc:
                self._send(409, {"error": str(exc)})
            else:
                self._send(200 if ok else 404, {"deleted": ok})
        else:
            self._send(404, {"error": "not found"})


def serve(cfg, host="127.0.0.1", port=None, *, parent_pipe=None):
    port = port or cfg.dashboard_port
    _Handler.cfg = cfg
    with ThreadingHTTPServer((host, port), _Handler) as httpd:
        if parent_pipe is not None:
            def wait_for_parent_exit():
                parent_pipe.read(1)
                httpd.shutdown()

            threading.Thread(target=wait_for_parent_exit, daemon=True).start()
        print(f"[dashboard] http://{host}:{port}  (output_dir={cfg.output_dir})")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
