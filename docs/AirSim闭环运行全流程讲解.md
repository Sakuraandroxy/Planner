# AirSim闭环运行全流程讲解

更新时间：2026-07-15。

> 本文档已按当前 AirSim UAV-VLN 主线刷新。当前在线入口是 `run_airsim_web.py`，配置入口是 `config/default.yaml`，离线评测入口是 `eval/run_eval.py`。

## 启动流程

1. 启动 AirSim/UE 场景。
2. 启动模型服务：任务解析、GroundingDINO、Qwen planner。
3. 运行 `python run_airsim_web.py`。
4. 打开 `http://localhost:5000`。
5. 输入自然语言任务。

## 任务阶段

| mode | 行为 |
|---|---|
| `action` | 上升、下降、左转、右转、前进、后退等直接动作 |
| `detect` | 查找/锁定目标，可配合重定位 |
| `target` | 导航到目标关系，如 `to`、`near`、`above` |

## target 阶段一步

```text
[Pipeline] RGB 到手
  -> [DetectRGB] front/down 检测
  -> [Qwen] 输出 5 个 waypoint
  -> [TargetDepth] bbox 对应深度
  -> [Completion] 是否完成
  -> [Trajectory] 选择轨迹
  -> [Execute] AirSim 执行
```

## 日志阅读

| 日志 | 说明 |
|---|---|
| `[TASK PARSER]` | 任务解析耗时和阶段数 |
| `[DetectRGB]` | 两视图检测框、置信度、best view |
| `[TargetDepth]` | 检测框中心/框内深度 |
| `[Completion]` | VLM 是否接受目标、是否完成 |
| `[TopK Trajectory]` | 候选轨迹预打分排序 |
| `[METRICS]` | 在线调试指标 |

## 指标说明

在线 StepTracker 的 NE/SR/SPL 是视觉估计指标，仅用于调试观察。正式论文指标应以离线评测 GT 目标点为准。
