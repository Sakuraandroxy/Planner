# 共用平移/yaw同步执行基线

服务器Qwen和DeepSeek共用此运动模块，不依赖任务名称。启动命令不变，默认开启。

## 分层与接口

- `planner/services/motion/synchronized.py`：纯数学运动规划。为每个世界坐标航段分配时间，位置和最短yaw增量共用五次曲线；起点和终点参考速度、加速度为零。
- `planner/domain/motion.py`：不可变MotionLimits，单位米、秒、度。MotionSegment沿用原有duration_s/profile字段，未改变模型四维输出协议或Vehicle接口。
- `planner/adapters/airsim/motion_tracker.py`：读取实际位姿，使用前馈速度加位置/yaw误差反馈，通过世界NED速度和yaw角速度执行。使用MaxDegreeOfFreedom，平移方向不决定视角。
- `vehicle.py`按profile选择执行器；`bootstrap.py`统一注入MotionPlanner和相同限制。未来Workflow复用这些服务，不复制控制循环。

纯旋转在同步模式中使用零平移参考和受限yaw角速度，位置反馈抵消漂移。旧linear模式仍保留rotateToYawAsync路径。

## 配置

两个YAML都有motion配置。`enabled: false`可回到原来的逐点位置API。

```yaml
motion:
  enabled: true
  control_hz: 20.0
  max_acceleration_mps2: 1.5
  max_yaw_rate_deg_s: 30.0
  max_yaw_acceleration_deg_s2: 30.0
  position_tolerance_m: 0.3
  yaw_tolerance_deg: 2.0
```

`airsim.speed_mps`是速度上限，不是整段平均速度。默认增益、稳定时间及低速阈值定义在MotionLimits，也支持作为motion字段覆盖。单段执行总时间受`airsim.move_timeout_s`限制：参考曲线过长时先拒绝，执行后未到位则超时报错。

执行器每个周期检查碰撞，位置和yaw均进入容差且运动速率较低并持续稳定后悬停，再复查误差。失败和Ctrl+C由ExecutionService取消、悬停。短时速度RPC串行等待完成，避免共享RPC客户端并发；实际更新率受RPC开销、模拟器速度和负载影响。

## 能力范围

本版本保证参考轨迹的平移/yaw同步和平滑启停；实际飞行需反馈纠偏，不能承诺严格同一瞬间或零误差到达。每个航点会减速停止，多航点连续过弯以及模型请求等待期间的连续飞行尚未实现。没有新增避障规划。

这套曲线用共同时间参数，不是先转头再前进。yaw很大时会延长平移时间。不得把目标yaw提前整段固定下发而声称已实现同步转向。

## 验证

`python -m pytest tests -q`

数学测试检查同步进度、首尾速度、速度/加速度上限、yaw跨界。带虚拟时钟的运动测试检查到位、超时、碰撞以及世界坐标速度接口。模拟替身没有包含AirSim真实动力学。

AirSim验收建议：在空旷处分别测试纯平移、纯旋转、上升、向右前方移动同时右转90度，观察`[Motion]`时长和最终误差日志，并查看录制画面。若任务解析器将同时动作拆成多个阶段，它们仍会按阶段顺序执行；同步针对模型同一航点中的平移和yaw。
