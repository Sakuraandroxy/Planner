# PLANNER_README

更新时间：2026-07-15。

> 本文档已按当前 AirSim UAV-VLN 主线刷新。当前在线入口是 `run_airsim_web.py`，配置入口是 `config/default.yaml`，离线评测入口是 `eval/run_eval.py`。

## 当前默认规划器

默认 `AGENT.PLANNER=qwen_planner`。Qwen 服务输入 front/down 两张图和任务指令，输出 5 个累计 body-frame waypoint。

## 输入格式

```text
<image><image>Instruction: Fly to the red car.
Output exactly 5 cumulative body-frame waypoints as a JSON list.
Each waypoint must be [dx, dy, dz].
Do not output any other text.
```

## 输出格式

```json
[[1.0, 0.0, 0.0], [3.0, 0.1, 0.0], [5.0, 0.2, 0.0], [7.0, 0.3, 0.0], [9.0, 0.4, 0.0]]
```

## 坐标约定

- waypoint 是累计位移，不是相邻增量。
- `dx` 为机体前方。
- `dy` 为机体右方。
- `dz` 由 AirSim/NED 执行层解释。

## 候选轨迹

`CANDIDATE.ENABLED=true` 时，系统在原始轨迹基础上生成候选。默认 `smooth_bridge`：固定起终点，对中间点加入平滑噪声，并按进展、方向一致性、安全性和平滑性预打分。

## 世界模型

`WORLD_MODEL.ENABLED=false` 时选择预打分第一名。开启后将 top-k 候选交给外部世界模型服务打分。
