# AirSim VLN 规划器

一个服务，同时驱动两代 AirSim 仿真器，用自然语言指令完成视觉语言导航。

浏览器里选地图、看画面、下发一句话；后端把指令解析成有顺序的任务阶段，让无人机或无人车
看画面、规划航点、飞过去，再把剩余轨迹、日志和实时画面回送到网页。

- 仿真器：ProjectAirSim（UE5，topic + service 双端口）与经典 AirSim（UE4，RPC）
- 载具：一个会话同时持有无人机与地面车，任务按载具路由
- 感知：RGB + 米制深度，视觉目标由 VLM 给出框与表面点
- 交互画面：xpra 把引擎窗口搬到浏览器，可点击操作场景

## 能力

| 能力 | 说明 |
|---|---|
| 自然语言任务 | 一句话解析成 `mission_plan_v1` 的有序阶段，逐阶段执行，首个失败即停 |
| 数值动作 | 直行/倒车 N 米、左/右转 N 度、上升/下降 N 米，同步五次多项式运动段 |
| 视觉目标导航 | 目标检测 → 5 航点规划 → 滑窗连续执行 → 本地到达条件 + 模型语义确认 |
| 自由导航 | 沿道路持续前进，20 Hz 连续跟随，原生碰撞触发有界倒车与重规划 |
| 双载具 | 同一会话管理无人机与地面车，切换即归位，画面里只显示一台 |
| 地图切换 | 控制台选图后整套重启仿真栈，天气重放，视角重置 |
| 运维控件 | 天气预设与滑块、四个机位、载具外观模型、回起点 |
| 观测 | MJPEG 帧流、内存日志环形缓冲、剩余世界航点与里程 |
| 手动工具 | 机位标定、环境自检、四个实机冒烟脚本 |

## 支持的地图与任务

地图在 `config/maps.yaml` 登记，每条声明由哪个仿真器驱动。任务类型不是开关，由仿真器实现了
哪些端口推导，缺端口就是缺能力。

| 任务 | ProjectAirSim (UE5) | AirSim (UE4) |
|---|---|---|
| `basic_motion` | 可执行 | 可执行 |
| `visual_goal_navigation` | 可执行 | 可执行 |
| `free_navigation` | 可执行 | 未实测，默认不提供 |
| `visual_goal_above` | 缺俯视端口 | 未接线 |

| 运维能力 | ProjectAirSim (UE5) | AirSim (UE4) |
|---|---|---|
| 载具 | rover + drone | drone |
| 机位切换 | 四个固定相机循环 | 无 |
| 载具外观模型 | 四旋翼 / 六旋翼 / 越野车 | 无 |
| 天气 | 8 参数 + 预设 | 8 参数 + 预设 |
| 回起点 | 有 | 无 |
| MJPEG 帧流 | 有 | 无（画面走 xpra） |

## 快速开始

需要一台有显卡的 Linux 机器、Xvfb 与 xpra、以及两个引擎的打包产物。规划服务不编译引擎，
不打开编辑器，只拉起已打包的二进制。

1. **Python 环境。** 需要两个解释器：主服务与 UE5 客户端一个，UE4 客户端另一个。
   两族的客户端库抢同一个模块名，不能在同一个解释器里共存。
   ```bash
   python -m venv .venv && . .venv/bin/activate
   pip install -e ".[test]"
   pip install --no-deps projectairsim==1.0.2
   ```
   `projectairsim` 在本仓库依赖之外，需要单独装。完整清单、安装顺序与踩坑见
   [部署文档](docs/DEPLOYMENT.md)。
2. **登记地图。** 在 `config/maps.yaml` 里填每张图的引擎二进制路径与地图名，
   `binary` 同时是进程归属判据。
3. **配置模型端点。** `config/base.yaml` 的 `task_parser`、`target_detection`、
   `trajectory_planner`、`task_completion` 四段指向一个 OpenAI 兼容端点，
   密钥通过环境变量 `VLN_API_KEY` 提供。四项都是硬依赖：没有它们，任务会被明确拒绝，
   画面仍然可用。
4. **启动服务。**
   ```bash
   python scripts/serve_planner.py --host 0.0.0.0
   ```
   服务启动时不连接仿真器，状态是 `disconnected`，控制台没有画面。
5. **在控制台里起场景。** 打开 `http://<主机>:8000/`，「世界设置」选地图，点「启动场景」，
   状态从 `connecting` 变 `idle`，画面自动接上。
6. **下发任务。** 在指令框输入一句话，例如「向前直行 5 米，然后左转 90 度」，
   或「开到前方的红色汽车」；无人车与无人机按钮决定任务路由到哪台载具。

启动前可以先跑环境自检，它按角色判断你站在哪个解释器里：

```bash
python scripts/probe_ue4_env.py
python -m pytest -q
```

## 目录

```
planner/
  domain/        纯数据类型：位姿、任务、观测、轨迹、天气、机位
  ports/         接口协议：载具、观测源、相机、天气、机位、外观模型、帧流
  services/      执行与控制：运动规划、连续执行、滑窗、帧汇、轨迹汇
  capabilities/  能力算法：目标几何、滑窗调度、完成判定、几何规划器
  workflows/     任务工作流：数值动作、视觉目标、自由导航
  application/   应用编排：解析、校验、路由、会话装配、服务控制器
  adapters/      外部适配：模型端点、HTTP 路由、MJPEG、诊断
  simulator/     仿真器共用：进程驱动、argv 渲染、注册表、worker、地图标记
  simulators/    每个仿真器一个包：airsim (UE4)、projectairsim (UE5)
console/         控制台（单页，无构建步骤）
config/          服务配置、地图目录、场景文件、任务规则
scripts/         启动、自检、机位标定、实机冒烟
tests/           单元测试与架构规则
docs/            文档
```

## 文档

| 文档 | 内容 |
|---|---|
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | 部署：环境、依赖、每个配置字段、换机器要改什么、故障排查 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 架构：分层、端口与能力推导、任务管线、进程模型、接口表 |
| [docs/AGENT.md](docs/AGENT.md) | 开发约定：先读什么、怎么扩展任务类型/地图/仿真器、提交规范 |
| [docs/archive/](docs/archive/) | 历史记录：踩过的坑、实测数据、被取代的设计 |

## 已知限制

- 一个仿真器实例只接一个客户端。topic 链路是单消费者，规划服务与冒烟脚本不能同时连。
- 两个引擎都是一个进程只加载一张 cooked 地图。换图 = 重启仿真栈，画面会断开一次再重连。
- 视角只能在场景里 `streaming-enabled` 的相机之间循环，没有自由观察者相机。
- 「只剩一台载具」是画面效果：另一台被抬到地图上方 1 km 并钉住高度，物理世界里仍然存在。
- `free_navigation` 没有完成条件，只能停止，或等本地失败结束。
- 模型端点是硬依赖。解析、检测、规划、完成判定都没有确定性回退。
- xpra 画面没有鉴权，只应暴露在可信网络。