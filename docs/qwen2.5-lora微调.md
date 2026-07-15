# qwen2.5-lora微调

更新时间：2026-07-15。

> 本文档已按当前 AirSim UAV-VLN 主线刷新。当前在线入口是 `run_airsim_web.py`，配置入口是 `config/default.yaml`，离线评测入口是 `eval/run_eval.py`。

## 微调目标

将 Qwen2.5-VL 微调为 UAV-VLN waypoint planner。输入 front/down 双视图和 instruction，输出 5 个累计 body-frame waypoint。

## 训练样本格式

```json
{
  "messages": [
    {"role": "user", "content": "<image><image>Instruction: Fly to the white dog...
Output exactly 5 cumulative body-frame waypoints as a JSON list.
Each waypoint must be [dx, dy, dz].
Do not output any other text."},
    {"role": "assistant", "content": "[[1.01,0.00,0.29],[1.94,-0.07,0.81],[3.49,-0.42,2.13],[4.17,-0.67,2.88],[6.08,-1.59,4.51]]"}
  ],
  "images": ["front.png", "down.png"]
}
```

## 在线调用原则

- prompt 尽量保持训练格式。
- 不额外塞大量规则和解释。
- 模型必须只输出 JSON waypoint list。
- 看不到目标时模型仍可能输出轨迹，因此需要完成判定和可选重定位兜底。

## 默认服务

```yaml
AGENT.PLANNER: qwen_planner
AGENT.PLANNER_URL: http://172.27.143.102:8004/plan
AGENT.PLANNER_MODEL: /data/sakura/models/3DG-VLN-finetuned-1
```
