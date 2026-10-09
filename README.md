# Base Planner

这是一个面向 AirSim 的模块化无人机规划器。本分支只注册明确数值动作 `basic_motion`，以及可选的低空平视静态目标接近任务 `visual_goal_navigation`。旧示例 `navigation` 已删除；`base_planner` 分支不受影响。

`basic_motion` 使用任务解析器提取动作参数，本地代码计算航点并以实际位姿和停稳观测确认完成，不调用轨迹模型。`visual_goal_navigation` 通过 VLM 定位静态目标、深度测距，调用轨迹模型生成安全航段前缀，并确认在目标附近停稳。后者默认关闭。

当前不包含 GroundingDINO、全局地图、完整避障/绕行、持久记忆、动态目标跟随、连续滑动窗口或候选轨迹评分。目标定位、关联及几何完成判断依赖观测质量和人工校准，不能保证所有天气或任意未知环境下成功。

## 当前运行流程

```text
自然语言指令
  → TaskParser 解析为 MissionPlan
  → MissionValidator 校验
  → MissionRunner / WorkflowRouter 选择任务工作流
  ├─ basic_motion：停稳观测 → 明确动作转为一个相对航点 → 安全检查
  └─ visual_goal_navigation：停稳观测 → 目标定位/深度几何 → 模型规划 → 安全检查
       ↓ 共用坐标变换、轨迹校验、平滑运动和执行服务
  MotionPlanner → ExecutionService → AirSim Vehicle / MotionTracker
       ↓ 实测反馈与任务专用完成判定
  StageResult → 下一阶段或停止 → MissionResult
```

每个航点的平移量和偏航量，都相对于前一个预测航点表示。AirSim 执行时将位置与偏航结合控制，因此无人机的移动方向可以与机头/相机朝向不同。相机适配器会读取图像返回的相机位姿，并获取相机 FOV；不会假设相机一定水平朝前。

`basic_motion` 仅在实测位置、yaw、停稳状态及无碰撞条件满足时返回 `completed`；单纯发完 API 不能算完成。`visual_goal_navigation` 只有通过目标关联、深度距离带、安全条件和停稳观测确认，才返回 `reached`；其依据仍是估计证据，不是仿真器目标位置真值。

## 配置和启动

先检查并按本机环境修改 `config/base.yaml`，再安装依赖：

```bash
pip install -r requirements.txt
```

交互式启动：

```bash
python run_airsim_cli.py
```

直接执行单条指令：

```bash
python run_airsim_cli.py --instruction "上升12米"
```

连接 AirSim 但不自动起飞：

```bash
python run_airsim_cli.py --no-takeoff
```

通过 Camera API 连续保存相机实际画面（包括配置了 pitch/yaw/roll 的相机）：

```bash
python run_airsim_cli.py --record-camera-api --camera-record-mode frames
```

录制图像、深度和元数据默认保存到 `output/camera_api/<时间戳>/` 下。录制器使用独立的 AirSim RPC 连接，避免主连接等待飞行指令时停止采集。

### 使用 DeepSeek 轨迹规划

将 `config/deepseek.example.yaml` 复制为本地配置 `config/deepseek.yaml`，填写 API 凭据和需要的模型参数，然后启动：

```powershell
python run_airsim_cli.py --config config/deepseek.yaml --record-camera-api --camera-record-mode frames
```

真实 API key 不要提交到 Git。`config/deepseek.yaml` 是本地配置文件；提交共享配置时只提交不含密钥的示例文件。

## 项目目录和职责

```text
run_airsim_cli.py             命令行入口
config/                       配置 schema、加载器和 YAML 配置
  schema.py
  loader.py
  basic_motion.py             基础动作安全空间配置和校验
  base.yaml                   默认配置
  deepseek.example.yaml       不含密钥的 DeepSeek 配置模板
  visual_goal.py              视觉目标任务的类型配置和校验
  visual_goal_navigation.example.yaml  新任务无密钥模板（默认关闭）
planner/
  domain/                     任务、观测、位姿、轨迹和结果等领域数据类型
  ports/                      可替换组件的类型化接口
  application/                计划校验、阶段运行和工作流路由
  workflows/                  面向具体任务的编排流程
    basic_motion.py           明确数值动作工作流
    visual_goal_navigation.py 静态目标接近工作流
    termination/              通用退出接口及视觉目标任务预算/搜索策略
  capabilities/               可复用任务能力：目标关联、深度估计与完成条件
  services/                   共享的轨迹变换、校验和执行协调逻辑
    motion/                   从世界轨迹生成运动段与运动曲线
  adapters/                    模型、仿真器和外部服务的具体实现
    airsim/                   AirSim 连接、观测、车辆、运动跟踪和录制
    task_parser/              任务解析提示词、HTTP 调用和响应解析
    goal_observer/            静态目标图像定位、提示词和严格协议解析
    diagnostics/              实现 TaskTrace 的本地文件诊断输出
    trajectory_planner/       轨迹规划客户端、提示词和响应解析
      prompts/                针对具体任务的规划提示词构造
  extensions/                  显式白名单注册骨架；当前不是动态插件系统
  bootstrap.py                 创建并组装运行时依赖
  visual_goal_bootstrap.py     新任务依赖装配，不包含执行循环
  errors.py                    规划器异常类型
tests/
  unit/                        单元测试
  contract/                    接口契约测试
  integration/                 多组件集成测试（默认使用 fake）
  fakes/                       测试用替身组件
output/                        本地运行输出，例如相机画面和日志
```

### 主要目录细分

- `run_airsim_cli.py` 负责命令行参数、连接与关闭、用户输入和结果展示。任务决策、模型请求和 AirSim 业务逻辑不应堆进 CLI。
- `config/schema.py` 定义有类型的配置结构；`config/loader.py` 负责读取并校验 YAML。环境路径、凭据等本机参数应留在本地配置中。
- `planner/domain/` 存放 `MissionPlan`、`Observation`、`WorldPose`、轨迹、进度状态和结果等核心数据。领域对象不应直接调用 API 或读取 YAML。
- `planner/ports/` 声明调用方需要的接口，例如 `TaskParser`、`ObservationSource`、`TrajectoryPlanner` 和 `Vehicle`。接口描述“要做什么”，具体方式由适配器实现。
- `planner/application/` 校验解析结果、按顺序运行阶段，并通过 `TaskKind` 路由到工作流；不负责图像处理或 AirSim 细节。
- `planner/workflows/` 编排某类任务的观测、规划、执行、复核和退出流程。不同任务应有各自的工作流，不要不断扩展一个万能工作流。
- `planner/workflows/termination/` 放置退出策略接口及具体任务策略。后续任务可以定义自己的停止/完成判定，不必把规则塞入导航工作流。
- `planner/capabilities/` 放置多个工作流可以共享的任务能力，当前实现静态目标关联、深度估计和 near 完成条件；尚无持久记忆系统。
- `planner/capabilities/basic_motion_conversion.py` 只将动作参数转为相对轨迹；`basic_motion_safety.py` 检查人工确认的空间范围；`completion/basic_motion.py` 进行实测完成判定。它们都不直接调用 AirSim。
- `planner/capabilities/targeting/` 实现目标关联及 RGB 射线到深度的几何映射；`completion/` 实现 near 到达条件。这里不发送模型 HTTP 请求。
- `planner/adapters/goal_observer/` 实现独立的图像定位后端、定位提示词及严格返回解析，不需要启动 GroundingDINO。
- `planner/workflows/visual_goal_navigation.py` 编排新任务；`visual_goal_session.py` 保存一次小任务的局部状态，`termination/visual_goal_navigation.py` 管理搜索、预算及退出策略。
- `planner/services/trajectory_safety.py` 筛选完整安全航段前缀；`planner/adapters/diagnostics/visual_goal_trace.py` 实现 TaskTrace 接口，保存诊断证据。安全检查是单前视深度支持的采样检查，不是全向避障保证。
- `planner/visual_goal_bootstrap.py` 只装配新任务依赖，由共同 bootstrap 显式调用，不处理任务运行循环。
- `planner/services/` 放置跨适配器复用的流程和转换逻辑；运动段生成及运动曲线位于 `services/motion/`。
- `planner/adapters/airsim/` 封装 AirSim RPC、车辆控制、相机观测、平滑运动跟踪和录制等仿真器相关实现。
- `planner/adapters/task_parser/` 实现任务解析模型协议；`planner/adapters/trajectory_planner/` 实现轨迹规划模型协议。对应提示词和响应解析应留在各自功能边界内。
- `planner/adapters/trajectory_planner/prompts/` 管理规划任务提示词构造。提示词策略与网络传输、响应解析分开，便于不同任务选择不同 prompt。
- `planner/extensions/` 当前只是显式白名单注册骨架，不是自动扫描或动态加载机制。禁止根据模型输出导入或执行任意模块。
- `planner/bootstrap.py` 是组合根：创建具体适配器，通过接口注入依赖，并注册 `TaskKind → Workflow` 映射。
- `tests/` 按测试边界组织。普通自动化测试优先使用 `fakes/`，不依赖 API key、在线模型服务或正在运行的 AirSim；需要仿真器的测试应明确标出前置条件。
- `output/` 仅用于本机运行产物，不放源码或共享配置模板。

## 如何添加一种新任务

例如后续增加巡检、独立搜索或道路跟随时，按显式链路扩展：

1. 在 `planner/domain/` 中增加 `TaskKind` 成员和专属的不可变参数类型，并扩展 `MissionStage` 的参数类型，避免用无约束的通用字典承载所有任务参数。
2. 更新任务解析提示词、响应解析/工厂和 `MissionValidator`。对未知任务类型、缺失参数和错误参数明确拒绝。
3. 在 `planner/workflows/<任务名>.py` 实现独立工作流。为循环设置配置化或明确的上限，并返回可解释的完成、停止和失败结果。
4. 只为该任务接入需要的能力。可复用的任务逻辑放在 `capabilities/`；需要替换实现的外部依赖先在 `ports/` 定义窄接口，再放入合适的 `adapters/` 实现。
5. 在 `planner/bootstrap.py` 装配该工作流，并注册 `TaskKind → Workflow`。由 `WorkflowRouter` 做路由，不要把任务判断分支堆入 `MissionRunner`。
6. 补充输入合法性、错误处理、路由、成功/失败传递及与其他阶段交互的单元、契约和集成测试。

不同任务也可以使用不同的提示词与退出策略：提示词放在对应 adapter 的 `prompts/` 中，由任务工作流/规划器选择；退出策略放在 `workflows/termination/`，通过接口注入相应工作流。避免在一个 prompt 或退出逻辑中混入所有任务的特殊情况。

## 如何添加新模块或外部集成

先判断模块的职责，再决定落点：

- 任务步骤编排：`planner/workflows/`
- 多任务共用的任务能力：`planner/capabilities/`
- 共享的数据转换或执行协调：`planner/services/`
- 模型、仿真器、硬件或网络服务实现：`planner/adapters/`
- 核心数据类型：`planner/domain/`
- 可替换依赖的接口：`planner/ports/`

如果需要更换实现，在 `ports/` 定义足够小且有类型的接口，并通过构造函数注入实现。不要要求所有模块继承一个万能父类：只有确实需要可替换性时才定义 `Protocol`，具体实现使用普通类即可。新增配置时更新 `config/schema.py` 和 `config/loader.py`，再在 `planner/bootstrap.py` 装配。不要使用隐藏的全局注册、导入时副作用或模型控制的动态导入。对于会访问外部服务或仿真器的模块，记录输入输出、异常、超时、状态生命周期和资源所有权。

## 如何适配一种新仿真器

例如增加其他仿真器时，仿真器 SDK、RPC 和坐标系差异都应封装在对应的适配器目录中；工作流和领域层不应直接导入仿真器 SDK。

1. 在 `planner/adapters/<仿真器名>/` 新建适配器包，分别实现 `Vehicle` 和 `ObservationSource`：前者提供当前位置/姿态、执行运动段、取消、悬停及碰撞状态；后者采集并返回统一的 `Observation`。仿真器特有的连接、传感器和运动控制辅助类也放在此包内。
2. 在适配器边界把仿真器数据转换为项目统一的领域语义。`WorldPose`、`MotionSegment`、相机位姿、内参、RGB 和以米为单位的深度都必须语义一致；尤其要核对世界坐标方向、单位、yaw 正方向、角度单位、机体/相机坐标变换及深度定义。若仿真器采用不同坐标系，应在适配器中显式转换并增加正反向转换测试，不要把坐标系特例散落到 workflow 或 prompt。
3. 核对 `Vehicle.execute_segment()` 对 `MotionSegment` 的执行语义：从起点到终点、平移和 yaw 同步、速度/时长限制、悬停与取消行为，以及运行中碰撞检查。仿真器 API 不同可以使用不同控制实现，但要满足同一个 `Vehicle` 契约；低层跟踪器放在该仿真器自己的适配器包中。
4. 处理仿真器生命周期。当前 `run_airsim_cli.py` 直接通过 `Runtime.connection` 调用 AirSim 的 `connect()`、`prepare_vehicle()`、`takeoff()` 和 `shutdown()`；`bootstrap.py` 也直接创建 `AirSimConnection`。所以新增仿真器时，不能只替换 `Vehicle`：应先在 `planner/ports/` 定义通用的会话/生命周期接口（例如 `connect()`、`prepare_vehicle()`、`takeoff()`、`shutdown()`），由各仿真器适配器分别实现；再在 `bootstrap.py` 选择并装配对应组件，让 CLI 依赖该接口而非 AirSim 专属方法。不要把生命周期控制代码复制成多个 CLI 分支。
5. 在 `config/schema.py`、`config/loader.py` 和 YAML 配置中加入必要的仿真器选择与连接、相机、速度、超时等参数。若需要在同一代码版本中选择多种仿真器，应显式配置仿真器类型并在 bootstrap 中白名单分派；避免散落的字符串判断和隐式自动发现。凭据和机器专属地址继续留在本地配置。
6. 检查通用逻辑是否真的与仿真器无关。可复用的 `ExecutionService`、轨迹变换、校验、运动规划和 `ObservationGate` 应继续依赖 `Vehicle` 等接口；如果它们出现具体 SDK 类型或仿真器方法名，应将相关细节移回适配器，或先定义必要的窄接口。
7. 为新适配器增加契约测试：统一的姿态/坐标转换、观测字段和 RGB-深度配对、执行运动段、纯 yaw 旋转、取消/悬停、碰撞上报、连接失败和关闭清理。用 fake 仿真器客户端测试常规路径；需要真实仿真器的测试单独标记并说明启动条件。

完成后，至少确认原有 AirSim 路径仍可运行，并运行 `python -m pytest -q`。如果生命周期接口或领域坐标语义需要调整，应先同步更新对应 Port、CLI/bootstrap 和契约测试，再接入新仿真器；不要让某个仿真器的特殊假设悄悄改变所有现有任务的行为。

## 从入口到 AirSim 的调用关系

```text
CLI
 → PlannerApplication
 → TaskParser
 → MissionValidator
 → MissionRunner
 → WorkflowRouter
 → 对应任务 Workflow
 → 观测 / 轨迹规划 / 校验与坐标变换
 → MotionPlanner
 → ExecutionService
 → AirSim Vehicle 适配器
 → AirSim
```

任务工作流负责任务层的观测、重规划和退出决策；运动跟踪器负责单个运动段中的低层反馈控制。轨迹 API 正常返回或飞完轨迹，不等于已经证明语义任务完成。

## 测试

运行全部自动化测试：

```bash
python -m pytest -q
```

根目录 `docs/` 当前被 Git 忽略，其中的本地文档不会随仓库共享。如需共享项目规范，请放入本 README，或在明确评估后调整忽略规则。

## 基础数值动作：basic_motion

本分支使用独立的 `BasicMotionWorkflow`，不再保留示例 `navigation` 工作流。任务解析模型将明确的数值命令转换为动作参数，基础动作流程**不请求轨迹规划模型或目标定位模型**，直接生成相对轨迹，复用世界坐标转换、平滑运动和统一执行服务。

例如“上升12米”的解析结果为：

```json
{
  "protocol_version": "mission_plan_v1",
  "stages": [{
    "stage_id": "stage_1",
    "kind": "basic_motion",
    "parameters": {
      "instruction": "上升12米",
      "forward_m": 0,
      "right_m": 0,
      "up_m": 12,
      "yaw_right_deg": 0
    }
  }]
}
```

参数为相对**该阶段起始实际姿态**的动作：`forward_m` 正向前、负向后；`right_m` 正向右、负向左；`up_m` 正上升、负下降；`yaw_right_deg` 正右转、负左转。转换为运动协议时，上升12米对应 `[0,0,-12,0]`，NED 的正 Z 向下约定不变。

- “左转90度，然后前进5米”拆为两个阶段；第二阶段以前一阶段执行后的实际朝向为基准。
- “向前8米、向右5米，同时右转15度”是一个同步动作阶段。
- 支持单步左右180°掉头，内部保留旋转方向；旧的仅起终点构造仍默认最短转向。外部航点仍为四维数组，不改变训练或服务器输出协议。
- 不猜测缺失距离/角度，不支持绝对高度或世界坐标命令。道路跟随、绕行、穿越、动态目标等未实现任务明确拒绝；若长任务含不支持部分，解析器应拒绝整条任务，不执行一个省略了剩余动作的计划。

### 安全前置条件及配置迁移

模板中的 `basic_motion.free_space.confirmed` 默认为 `false`，此时任务校验会拒绝包含基础动作的整份计划。实验前需人工确认一个**固定世界 NED 坐标的静态无障碍盒**，在本地配置填写三个轴的最小/最大边界，并核实机体和跟踪误差余量后才允许设置 `confirmed: true`：

```yaml
basic_motion:
  free_space:
    confirmed: false
    minimum_world_ned_m: null  # 核实后填 [最小x, 最小y, 最小z]
    maximum_world_ned_m: null  # 核实后填 [最大x, 最大y, 最大z]
    clearance_radius_m: 1.0   # 初值；须包含机体尺寸及位置/跟踪误差余量
```

此空间不会随无人机移动，不是单张前视深度生成的全向避障地图。流程只允许完整直线航段的起终点及球形安全包络位于盒内；纯旋转也要满足净空。场景、障碍或边界有效性变化后，应撤销确认并重建运行时。视觉目标任务自身的起始净空配置不能代替此确认，基础上升动作也不受视觉目标任务的 ±1m 保高约束。

执行前会停稳观测、检查轨迹/空间、复核观测起点是否漂移以及碰撞状态；执行后再次停稳观测，按 `motion.position_tolerance_m`、`motion.yaw_tolerance_deg` 校验实际终点。只有测量确认到达并停稳才为 `completed`。执行失败或未到达时取消/悬停，不自动重复原始相对动作；停止 API 成功本身不算停稳确认。

旧本地配置如含 `navigation:` 块，需手动删除，并加入以上 `basic_motion:` 配置；否则加载器会明确报迁移错误。共享模板已更新，带密钥的 `config/deepseek.yaml` 等本地文件未被改写。基础动作仍需要任务解析服务；关闭视觉目标任务时不会启动或调用轨迹规划、目标定位后端，也不要求未使用的轨迹后端 API key。

**启动注意：**CLI 原有默认自动起飞行为未改变，`free_space.confirmed: false` 仅阻止任务动作，不阻止启动时的起飞。已在空中且准备好安全起点时可用 `--no-takeoff`。先只读检查配置，再在仿真器和安全条件就绪后启动：

```powershell
python -c "from config import load_config; c=load_config('config/base.yaml'); print('basic_motion_space_confirmed:', c.basic_motion.free_space.confirmed)"
python run_airsim_cli.py --config config/base.yaml --no-takeoff --record-camera-api --camera-record-mode frames --camera-record-fps 5
```

动作阶段总预算使用 `airsim.move_timeout_s`，包括观测、执行及完成复核；RPC 阻塞仍可能造成超时滞后，不承诺硬实时。此次验证使用离线替身，尚未验证真实任务解析服务或 AirSim 飞行。

## 本分支新增：低空平视视觉目标导航

### 适用范围与闭环

仅支持静态目标的 `near` 关系，例如“靠近前方红色房子并悬停”。不能把飞到屋顶上方、绕到背后、进入建筑或跟随移动目标偷偷改成 near。长任务仍先拆为 MissionStage，各阶段的目标、历史和预算独立。

```text
停稳观测 → VLM 多候选定位 → 原始深度逐点测距 → 选择/关联接近区域
             ├─ 不可见：有限原地搜索 → 再观测
             ├─ 歧义/深度不足：有限恢复或退出
             ├─ 未到达：航点规划 → 安全前缀 → 平滑执行 → 再观测
             └─ 满足候选条件：第二次新观测确认 → reached
```

实验初值：到选定可见表面 4m，允许带 3～5m；加入 0.3m 工程余量后，估计值需在 3.3～4.7m 内。两次新采集观测至少间隔 0.5s。保持起始世界高度附近（±1m），相机初始 pitch 为 ±5° 内；这不是离地高度控制。

每轮只执行累计不超过 6m/8s、得到空间证据支持的完整航段前缀；不会静默截短航点。较大转向后重新观测。默认搜索每步 30°、左右最多 90°，每次最多 9 步，整个任务最多 2 次/18 步，且必须确认旋转净空。原有每点 `[dx,dy,dz,dyaw_deg]` 协议、NED 符号和 `moveByVelocityAsync` 平滑控制保持不变。

### 配置及启动

先复制无密钥模板：

```powershell
Copy-Item config/visual_goal_navigation.example.yaml config/visual_goal_navigation.local.yaml
```

编辑本地文件，先完成以下准备，不要未经核查就把确认项改为 true：

1. `goal_observer` 填写实测支持图片、`goal_observation_v2` 归一化框/多候选点协议的模型地址和模型名。首轮一张 RGB，后续请求还包含目标整体和选中区域两张参考裁剪图；必须验证该后端支持一次请求三张图片。可以和轨迹模型共用服务，但不能假定航点微调模型就会输出定位框。支持本地 `api_key` 或 `api_key_env`；本地配置已忽略，不提交密钥。
2. 校准 RGB/DepthPerspective 的 FOV、画幅、视角和坐标基底，填写 `depth_horizontal_fov_deg`，确认后设置 `geometry_verified: true`。不同分辨率按内参射线映射，不直接拉伸做几何计算。
3. 按机型实测填写 `vehicle_radius_m`、跟踪/障碍余量；确认**每次任务起点**周围所配置半径内的静态净空后，设置 `initial_clearance_confirmed: true`。如需搜索，也必须确认 `search_clearance_confirmed: true`。初始净空不是由单张前视图自动证明的。
4. 最后设置 `visual_goal_navigation.enabled: true`。相机 FOV 若在运行中变化，需要重建运行时，当前缓存不支持动态 FOV。

安全检查的占用半径叠加机体半径、额外障碍余量、跟踪余量及允许的观测起点漂移，不能用同一份余量同时抵扣后两项。默认合计 2.1m；显式停靠带下界必须同时满足该机型的派生安全半径和 minimum_standoff_m，不能只检查抽象的 2m 下限。

检查配置（只读取文件，不连接仿真器、不起飞）：

```powershell
python -c "from config import load_config; c=load_config('config/visual_goal_navigation.local.yaml'); print('visual_goal_enabled:', c.visual_goal.enabled)"
```

模型和 AirSim 就绪且上述条件满足后启动：

```powershell
python run_airsim_cli.py --config config/visual_goal_navigation.local.yaml --record-camera-api --camera-record-mode frames --camera-record-fps 5
```

默认入口会自动起飞；已在合适空中起点可加 `--no-takeoff`。该选项不会自动验证初始净空。输入“靠近前方红色房子并悬停”，检查解析日志是否为 `visual_goal_navigation`。明确数值动作路由到 `basic_motion`；道路跟随等未实现任务明确拒绝，不再走旧 navigation。

### 感知服务返回协议

GoalObserver 是与航点规划分离的图像请求，返回严格 JSON，不返回航点或完成声明：

```json
{
  "protocol_version": "goal_observation_v2",
  "status": "visible",
  "bbox": [0.38, 0.25, 0.69, 0.82],
  "surface_candidates": [
    {"candidate_id": "a", "point": [0.53, 0.57], "description": "central solid facade", "matches_reference_region": false},
    {"candidate_id": "b", "point": [0.43, 0.57], "description": "left solid facade", "matches_reference_region": false},
    {"candidate_id": "c", "point": [0.63, 0.57], "description": "right solid facade", "matches_reference_region": false}
  ],
  "description": "the red house ahead",
  "evidence": "matching red facade is visible",
  "same_target": true
}
```

坐标相对**当前完整 RGB 图像**归一化至 0～1；visible 时点必须在非空框内。通常请求 3～5 个分散的可见实心表面点，但允许更少，不能为凑数编造点；不可见或歧义时框为 null、候选列表为空。`same_target` 只表示物体身份；有区域参考时最多一个候选可用 `matches_reference_region: true` 标记原选中位置。候选编号仅在当前观测内有效，不能用于跨帧跟踪。无参考时区域匹配必须为 false。未知/重复字段、重复编号或位置、NaN/Inf、越界坐标和数量超限拒绝。不把 VLM 自报置信度当真值。

多点测距和区域选择位于 `capabilities/targeting/`，不是新的工作流。首轮按深度可靠性、保持高度的适用性和转向需求选择，不平均所有点、不盲选最近点；后续结合局部参考和世界坐标门限关联同一区域。近邻重复采样按已有深度窗口大小去重，不新增距离阈值。进展距离使用首轮固定世界锚点，完成还须满足当前深度的到达带；区域无法关联时按原有观测预算退出，不自动换立面。所有可靠候选（包括未选中的）仍参与接近距离约束，原始深度空间安全检查不变。

配置仅新增 `visual_goal_navigation.limits.maximum_surface_candidates`（默认 5，正整数上限，不是数量下限）。区域关联复用 `association_gate_m`（默认 1.5m），深度、退出及安全阈值复用已有配置和默认值，不另列整套算法参数。模型请求次数不随候选数量增加；增加的是单次输出和本地处理。错误协议不会静默退化到 v1。

接口迁移：`GoalObserver.observe()` 返回 `GoalSurfaceObservation`；本地 `SurfaceSelector` 输出选中区域，显式投影为原有 `GoalObservation` 规划上下文。多表面定位改动本身不改变四维航点协议，只有定位模块的提示词与解析器同步 v2；轨迹规划提示词的后续调整见下节。如果更换定位 Adapter 或测试 Fake，也必须实现新接口。每个阶段新建参考，选择能力本身无状态、无独立线程和重试。

## 固定五点规划与预留 pending 接口

- 轨迹模型的 system/user 提示词统一要求恰好5个新增 `[dx,dy,dz,dyaw_deg]`，
  模型客户端也检查返回数量。`trajectory.point_count` 当前必须为5。
- 通用规划规则放在 `planner/adapters/trajectory_planner/prompts/base.py`；
  `visual_goal_navigation` 在其基础上增加目标身份、表面停靠、高度和单轮预算约束。
  任务解析和目标定位有各自独立的提示词，不混入航点输出规则。
- `TrajectoryPlanner.plan()` 和提示词构造接口预留可选参数
  `pending_trajectory: RelativeTrajectory | None = None`，与已执行的
  `previous_trajectory` 明确区分。pending 是当前观测时尚未执行的动作。
- 当前工作流没有 pending 队列，实际用户提示词明确写入空队列 `[]`。
  提示词构造器能够渲染0～5个 pending，供未来接入使用，但模型客户端目前
  拒绝非空 pending，且不会发送请求。这不是已实现的滑动窗口或快慢系统。
- 后续接入非空 pending 时，必须同时设计观测/队列快照、并发所有权、已消费
  航点处理，以及以末尾预测姿态衔接新增点的世界坐标转换和执行安全校验。
  不能只解除客户端限制而继续从当前位姿转换新增点。
- 固定5点约束只针对轨迹模型输出。本地基础动作与搜索纯旋转仍可只有1点；
  安全检查或重新观测可能仅执行模型输出的安全前缀，不保证每批5点全部执行。
- 本次不修改数据转换器。训练侧提示词仍需在正式训练前与最终推理协议核对。

### 同一次请求中的系统与用户消息

轨迹规划客户端一次提交 `messages=[system, user]`：系统消息包含通用与任务专属规则，
用户消息包含当前 RGB/深度、任务、位姿、历史和空 pending。只发送一次请求，模型只生成一次；
不能在服务端把两个角色合并成一条 `user`，也不能把系统提示词单独请求一次。

配套的独立部署脚本位于项目上一级的 `server.py`，需单独同步到模型服务目录并重启服务。
它在 `/v1/chat/completions` 中保留消息角色、文本与图片顺序，然后一起交给模型的聊天模板。
坏图片、未知消息角色或内容类型会返回 HTTP 400，不会静默丢弃或调用模型。
该脚本不依赖 Planner 包；客户端协议、服务地址和原有启动指令保持不变。
`/health` 的 `chat_message_roles_preserved: true` 可确认已部署新版脚本。

旧 `/plan` 接口保留原有单用户提示词行为，仅作为兼容入口；当前 Workflow 不使用它。
本次没有修改数据处理代码，训练提示词与 yaw 边界的同步仍待后续单独处理。

`tests/contract/test_trajectory_server_messages.py` 使用假的模型和处理器验证配套脚本，
不会加载权重或连接仿真器。该测试要求项目上一级存在 `server.py` 且安装 Flask；
仅克隆 Planner、缺少配套脚本或 Flask 时会明确跳过，不代表真实服务已验证。

### 输出与边界

诊断保存在 `output/visual_goal/<时间戳_唯一编号>/`：RGB、深度可视化、原始米制 `.npy`、候选框点叠加图和 `*_surface_selection.png`（绿色选中、黄色可用、橙色测距可靠但未通过筛选、红色深度不可靠）、`events.jsonl`（所有候选估计与筛选原因、固定锚点、模型名、实验配置、规划/执行前缀、预算和最终状态）。背景 Camera API 录制仍只读，不触发悬停；正式任务观测通过 ObservationGate 停稳后依次采集图像与车辆姿态，并不宣称跨 RPC 严格同一时刻。

只有 `reached` 是新任务成功；其他状态包括目标未找到/丢失/歧义/疑似移动、感知不可靠、blocked、stuck、timeout、safety_stop、模型/观测/执行错误及 cancelled。目标移动判断依据同表面关联下多次一致世界位移，仍依赖定位误差；不升级为动态跟随。不能可靠关联的表面切换会停止，不自动换表面继续。

观测门控失败后必须重建运行时，不能在同一实例中恢复运动。失败时取消/悬停分别尝试，API 成功不等于反馈确认停稳；结果记录停止确认状态。任务 deadline 贯穿观测、模型请求和运动控制，拒绝迟到响应；RPC/future 阻塞仍可能造成超时滞后，不承诺硬实时停止。

`path_length_m` 当前是**实测航段端点距离之和**，不是高频采样的完整实飞路径，不能直接用作 SPL。安全检查使用静态场景、人工起始净空和深度采样证据，不能认证未观察空间的全部风险，也不提供绕障路线。当前自动化测试通过不代表已通过 AirSim 实飞或真实 VLM 验收。

多个候选点不是建筑重建或分割掩码，不能推断整个楼体的最近表面、背面或未观察通道；世界锚点依赖首轮深度误差，模型的物体/区域匹配也可能出错。当前仅支持接近选中的可见区域，不保证任意建筑都有可达接近位置。
