# vlm_api_context

更新时间：2026-07-15。

> 本文档已按当前 AirSim UAV-VLN 主线刷新。当前在线入口是 `run_airsim_web.py`，配置入口是 `config/default.yaml`，离线评测入口是 `eval/run_eval.py`。

## 默认完成判定

当前默认：`TASK_COMPLETION.NAME=depth_detector`。

流程：

1. GroundingDINO 对 front/down RGB 检测目标。
2. 程序读取 bbox 对应 front/down depth。
3. VLM 结合任务、双视图、bbox、score、depth 输出结构化结果。

输出格式：

```json
{"target_detected": true, "done": false, "accepted_view": "front"}
```

## 配置

```yaml
TASK_COMPLETION:
  NAME: depth_detector
  CALL_MODE: parallel_preplan
  SPLIT_RGB_DEPTH_CAPTURE: true
  STOP_DEPTH_THRESHOLD: 8.0
  VLM_FINAL_JUDGE_ENABLED: true
  VLM_FAIL_FALLBACK_TO_HEURISTIC: false
```

## 注意

- GroundingDINO 只提供候选框，不等于最终完成。
- VLM 最终决定是否接受目标和视图。
- 程序根据 accepted view 对应深度判断距离约束。
- 终端默认不打印长原因，只打印 `detected/accepted/done`。
