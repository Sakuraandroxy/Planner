# MissionMemory 长任务记忆系统及避障

更新时间：2026-08-01

当前版本相比初始 `planner`，主要从“只依赖当前视野和 VLM 轨迹”升级为“长任务记忆 + 目标一致性 + 完成判定约束 + 局部避障”的闭环系统。

## 核心改进

### MissionMemory

新增 `agent/functions/memory/`，在一个完整长任务内部保存轻量目标记忆。例如“先到白车旁边，再到灌木丛旁边的红车旁边”，系统会尽量在开始阶段和飞行过程中提前记住后续红车和灌木丛，而不是完成白车后才重新寻找。

memory 不保存原图、完整深度图或密集点云，只保存目标类别、描述、估计世界坐标、置信度、不确定性、footprint 半径、少量观测历史和外观摘要，因此开销较小。

### 目标一致性

当前任务解析和候选轨迹评分加入了序号、辅助锚点和 stage lock。这样可以更好处理“第 2 辆红车”“灌木丛旁边的红车”等多实例歧义，避免飞行中因为视角变化切换到另一个相似目标。

相关模块主要包括 `agent/functions/task_parser/`、`agent/functions/candidate/` 和 `agent/functions/memory/`。

### 完成判定

完成判定不再只是进入固定半径。当前会结合 memory、目标关系和几何约束判断：

- `near` / `beside` 需要满足水平距离和高度差。
- `above` 需要满足水平对齐和上方高度约束。
- `land` 必须执行真实降落，不能只靠 memory 完成。

同时新增内外两层距离概念，允许无人机进入目标附近区域，而不是机械停在完成半径圆周上。

### 轨迹约束与避障

新增 memory path guard，用于检查 VLM 轨迹是否会飞过当前目标、远离目标，或在队列已经足够接近目标时继续追加过长轨迹。

新增 `agent/functions/obstacle_avoidance/`，基于前视图和下视图深度维护短期局部障碍缓存。它只保存有限数量的 obstacle cells，并带有过期机制，用于在执行前裁剪危险轨迹、提前停在障碍前，或对非目标障碍尝试简单绕行。

AirSim 侧补充了深度采集接口：`sim/airsim_client.py` 中的 `capture_planning_views_depth_with_pose`。

## 配置与测试

主要配置位于 `config/default.yaml`：

- `FUNCTIONS.MEMORY`
- `FUNCTIONS.OBSTACLE_AVOIDANCE`

新增测试文件为 `eval/test_mission_memory.py`，覆盖目标锁定、完成判定、路径防 overshoot、方向词过滤和深度避障等关键逻辑。

## 仍需改进

当前 memory 仍是轻量任务记忆，不是完整 SLAM 或全局地图；避障也偏局部反应式。当前的导航更倾向于平面，后续还需要重点加强建筑飞越时的主动爬升策略，以及复杂场景下的全局绕障能力。

## 总结

这次改进的重点是让 planner 能记住长任务中的目标、保持目标实例一致、减少飞过目标和撞障碍的情况，并让任务完成判定更符合实际飞行关系。
