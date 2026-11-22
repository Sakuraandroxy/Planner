# AirSim VLN 规划器

用一句话指挥仿真里的无人机或无人车：它在画面里找目标、规划路线、飞过去或开过去。

浏览器选一张地图，页面上是引擎实时画面和一个指令框。输入「开到前方那辆红色汽车旁」，服务把这句话拆成有先后的阶段，让视觉模型在画面里定位目标，规划器给出五个落点，控制循环一步步执行，途中持续重规划。可以随时停，剩余路线、日志、画面都在同一页。

同时支持两代 AirSim：基于虚幻 5 的 ProjectAirSim，和基于虚幻 4 的经典 AirSim。

## 能做什么

| 你说的话 | 它做的事 |
| --- | --- |
| 向前直行五米，然后左转九十度 | 按距离与角度逐段执行，停稳后报告完成 |
| 向前飞行十米，再上升五米 | 同上，加升降 |
| 开到前方那辆红色汽车旁 | 定位目标，规划五个落点，边接近边修正，到了确认 |
| 沿着这条路一直开，遇障绕开 | 持续巡航，碰障碍后退一点、重新看、继续，叫停才结束 |

## 支持的场景

| 地图 | 引擎 | 载具 | 可跑任务 |
| --- | --- | --- | --- |
| 城市、郊区、交流中心 | ProjectAirSim（UE5） | 无人机、阿克曼无人车 | 数值动作、目标接近、沿路巡航 |
| 郊区 NH、废弃公园、MSBuild 园区、TrapCam、Blocks、上海、校园 | AirSim（UE4） | 无人机 | 数值动作、目标接近、沿路巡航 |

可跑任务由「任务声明的必需端口」与「引擎实际填的槽位」求交得出，`/api/status` 的 `capabilities` 字段是唯一出口。没有白名单开关。

## 架构一句话版

六层加三支撑层，只能向下依赖，跨层只走层入口：

```
adapters  →  assembly  →  workflows  →  runtime  →  algorithms  →  domain / ports
                                                                  ↑
                                              simulators（实现端口）· simulator（进程栈）
```

三条不变量：

1. 任务只决定去哪，碰撞退避由执行层被动触发。
2. 引擎只下发命令与推进周期，采样、限幅、反馈律在执行层与计算层。
3. 能力由端口交集推导，不用开关列表。

## 文档

| 文档 | 读它解决什么 |
| --- | --- |
| [项目架构](docs/ARCHITECTURE.md) | 分层图、一次任务全链路、三条主干路径、两条命令通道、设计取舍 |
| [部署文档](docs/DEPLOYMENT.md) | 环境、安装、四层配置、启动、进程栈、端点、标定、故障排查 |
| [开发文档](docs/DEVELOPMENT.md) | 仓库布局、硬性规矩、扩任务、接引擎、加端点、测试策略、提交 |
| [待办与缺口](docs/TODO.md) | 跨层结构性事项与已确认不改的取舍 |
| [层文档](docs/layers/) | 每层的职责、输入输出、对外名字、流程、失败模式、测试 |

层文档直链：[domain](docs/layers/domain.md) · [ports](docs/layers/ports.md) · [algorithms](docs/layers/algorithms.md) · [runtime](docs/layers/runtime.md) · [workflows](docs/layers/workflows.md) · [assembly](docs/layers/assembly.md) · [adapters](docs/layers/adapters.md) · [simulator](docs/layers/simulator.md) · [simulators](docs/layers/simulators.md)

重构过程与裁决记录留在 [docs/refactor/](docs/refactor/)，与当前代码冲突时以代码与上面这套文档为准。踩过的坑与实测数据在 [docs/archive/](docs/archive/)。

## 跑起来

完整步骤见 [部署文档](docs/DEPLOYMENT.md)。最短路径：

```bash
conda activate vln
export VLN_API_KEY='<你的密钥>'
cd /roxy/kedior/Planner
python scripts/serve_planner.py
```

浏览器打开 `http://127.0.0.1:8000/`，在世界设置里选图、启动场景，状态就绪后画面接上。选载具、输指令、提交。用完点停止世界关引擎。

服务启动时未连接：不加载场景，不开仿真器 socket。选图才拉起进程栈。只清理本项目自己拉起的残留进程。

## 目录

```
planner/     六层加两套引擎实现与进程栈，见架构文档
console/     静态控制台，无构建步骤
config/      四层配置：服务、仿真器、地图、任务与模型端点
scripts/     启动、标定、冒烟、提交检查、流验收
tests/       87 个测试文件，含三条架构守门测试
docs/        本套文档、重构档案、历史记录
```

## 常用命令

```bash
python -m pytest tests/ -q                              # 全量用例
ruff check planner tests scripts                        # 静态检查
bash scripts/verify_stream.sh /roxy/kedior/Planner       # 提交前总验收
python scripts/check_profiles.py                        # 配置自检
python scripts/calibrate_streaming_views.py             # 相机机位标定
```

## 状态

当前分支 `refactor/layered-architecture`：全量 1217 条用例通过，零 xfail；跨层深层 import 为零；层入口导出面与层文档逐名一致，由 `tests/test_public_surface.py` 守着。

已知缺口见 [待办与缺口](docs/TODO.md)，各层内部问题见对应层文档的「已知缺口」小节。
