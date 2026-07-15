# project-architecture

更新时间：2026-07-15。

> 本文档已按当前 AirSim UAV-VLN 主线刷新。当前在线入口是 `run_airsim_web.py`，配置入口是 `config/default.yaml`，离线评测入口是 `eval/run_eval.py`。

## 项目结构

| 目录 | 作用 |
|---|---|
| `agent/` | 任务解析、检测、规划、完成判定、候选轨迹、世界模型、重定位、碰撞恢复 |
| `sim/` | AirSim 客户端、抓图、后台取帧 |
| `web/` | Flask Dashboard、前端、共享状态 |
| `eval/` | 离线批量评测与远程场景管理 |
| `config/` | 主配置 |
| `tools/` | 数据处理和训练辅助脚本 |

## 主入口

```bash
python run_airsim_web.py
```

启动后会连接 AirSim、预热模型服务、启动 Dashboard，并等待用户输入任务。

## 数据流

```text
Dashboard -> SharedState -> main_loop -> TaskManager -> agent modules -> AirSimClient -> SharedState -> Dashboard
```

## 模块边界

- `run_airsim_web.py` 负责组装和调度。
- `agent/` 负责决策模块。
- `sim/` 负责 AirSim 交互。
- `web/` 负责展示和任务输入。
- 新功能优先放在 `agent/` 子模块，通过 `config/default.yaml` 开关控制。
