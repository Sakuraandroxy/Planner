# Uni-LaViRA AirSim Web Planner

这是一个基于 **AirSim + Web Dashboard + 多模态大模型（VLM）** 的无人机闭环导航原型。系统接收中文自然语言任务，采集 AirSim 第一人称 RGB 图和深度矩阵，通过 OpenAI 兼容接口完成任务解析、目标检测和候选轨迹生成，并把动作转换为 AirSim 航点执行。

当前仓库重点是 **AirSim 空中导航闭环实验系统**，不是原始 Uni-LaViRA 全量论文仓库。

## 当前核心能力

- **Web 闭环控制**：浏览器输入任务，实时查看 RGB、深度预览、候选轨迹、执行状态和推理摘要。
- **VLM 任务解析**：任务开始时先调用一次纯文本 VLM，将自然语言任务拆成 `action / detect / target` 阶段。
- **分阶段任务管理**：固定动作阶段直接执行；检测阶段只锁定目标；目标阶段才进入 bbox + planner 闭环。
- **两阶段 VLM 调用**：第一阶段 VLM 输出目标/障碍物 bbox；第二阶段 VLM 基于 bbox 深度生成候选轨迹。
- **目标深度计算**：VLM 只负责框选目标，Python 从 AirSim `DepthPerspective` 原始深度矩阵中计算目标深度。
- **场景物体深度**：可同时检测目标、障碍物和参考物，打印并注入其 bbox 与深度，辅助规划绕障。
- **目标实例锁定**：首次发现目标后锁定类别和世界坐标，减少同类目标误切换。
- **轻量上下文管理**：只保留目标记忆和状态机，不负责重定位、局部前进或到达判断。
- **四向环视重定位**：目标丢失后独立触发 4 视角环视，重新寻找最可能的锁定目标。
- **本地/远程 VLM 兼容**：支持云端 API，也支持内网 vLLM 部署的 `Qwen3-VL-4B-Instruct`。

## 系统流程

```text
用户任务
  ↓
TaskParser（纯文本 VLM，拆分阶段）
  ↓
TaskManager（按阶段执行）
  ├── action 阶段：直接执行动作，不调 planner
  ├── detect 阶段：bbox API 锁定目标，不调 planner
  └── target 阶段：
        AirSim RGB + Depth
          ↓
        bbox / scene objects API
          ↓
        Python 计算目标和障碍物深度
          ↓
        ContextManager + TargetIdentity
          ↓
        planner API 生成候选轨迹
          ↓
        解析 JSON → AirSim 航点执行
          ↓
        若目标丢失 → Relocalizer 四向环视
```

## 项目结构

| 路径 | 作用 |
|---|---|
| `run_airsim_web.py` | 主入口：连接 AirSim、启动 Web、运行闭环任务 |
| `run_airsim_minimal.py` | 命令行版最小闭环运行脚本 |
| `sim/airsim_client.py` | AirSim RPC 封装：RGB、Depth、位姿、移动、碰撞检测 |
| `sim/frame_capturer.py` | 后台抓帧线程：为前端提供 RGB 和深度预览 |
| `sim/action_executor.py` | 将 `forward/left/right/up/down` 转换为 AirSim 航点并执行 |
| `agent/task_parser.py` | 纯文本 VLM 任务解析：拆分多阶段任务 |
| `agent/task_manager.py` | 阶段管理：`action / detect / target` |
| `agent/planner.py` | VLM 调用编排：bbox、深度计算、prompt、planner、解析 |
| `agent/target_depth.py` | bbox 解析、目标选择、bbox 到深度图映射、深度统计 |
| `agent/target_identity.py` | 目标实例锁定，防止追错同类目标 |
| `agent/context_manager.py` | 轻量上下文：目标记忆 + 状态机 |
| `agent/relocalizer.py` | 目标丢失后的四向环视重定位 |
| `agent/prompt_builder.py` | 图片编码、深度提示格式化、planner 消息构造 |
| `agent/response_parser.py` | 解析 VLM JSON，并裁剪/规范动作 |
| `agent/vlm_client.py` | OpenAI 兼容 Chat Completions API 客户端 |
| `planner/config.py` | Planner 配置和默认参数 |
| `planner/prompt_templates.py` | 中文规划提示词模板 |
| `planner/trajectory.py` | 动作解析、航点计算、噪声模拟 |
| `web/` | Flask 后端与 Web Dashboard |
| `本地部署.md` | vLLM + Qwen3-VL 本地部署记录 |
| `本周周报7-1.md` | 当前阶段汇报材料 |

## 环境准备

建议使用 Python 3.10：

```bash
conda create -n airsim python=3.10
conda activate airsim
pip install -r requirements.txt
```

主要依赖：

- `airsim`：连接 AirSim / Unreal 仿真器
- `openai`：调用 OpenAI 兼容 VLM API
- `Flask`：Web Dashboard
- `Pillow` / `numpy`：图像和深度矩阵处理

## AirSim 配置

AirSim 配置文件通常位于：

```text
C:\Users\<用户名>\Documents\AirSim\settings.json
```

推荐设置：RGB 保持高清，深度矩阵使用较低分辨率，避免 AirSim RPC 传输过慢。

```json
{
  "SettingsVersion": 1.2,
  "ClockSpeed": 1,
  "SimMode": "Multirotor",
  "CameraDefaults": {
    "CaptureSettings": [
      {
        "ImageType": 0,
        "FOV_Degrees": 90,
        "Width": 1920,
        "Height": 1080
      },
      {
        "ImageType": 2,
        "FOV_Degrees": 90,
        "Width": 640,
        "Height": 360
      }
    ]
  },
  "TargetFPS": 60
}
```

说明：

- `ImageType: 0` = RGB 场景图（Scene）
- `ImageType: 2` = 原始深度矩阵（DepthPerspective）
- 修改 `settings.json` 后必须重启 AirSim / Unreal 场景
- 不建议直接把 `DepthPerspective` 开到 `1920x1080`，float 深度矩阵通过 RPC 传输会明显变慢

## API 与配置

项目读取系统环境变量，并自动加载项目根目录 `.env`。首次使用可复制模板：

```bash
copy .env.example .env
```

`.env` 和 `planner/.env` 不应提交真实 API key。

### 常用 VLM 参数

| 环境变量 | 默认值 | 说明 |
|---|---:|---|
| `PLANNER_BASE_URL` | `http://localhost:8000/v1` | OpenAI 兼容 API 地址 |
| `PLANNER_API_KEY` | `no-key` | API key；本地 vLLM 可随意填 |
| `PLANNER_MODEL_NAME` | `gpt-4o` | 模型名或本地模型路径 |
| `PLANNER_THINKING_MODE` | `disabled` | 是否关闭 MiMo 思考模式 |
| `PLANNER_REASONING_EFFORT` | `default` | 支持该参数的模型可设为 `low` |
| `PLANNER_ENABLE_THINKING` | `default` | 支持该参数的模型可设为 `false` |
| `PLANNER_WARMUP_ENABLED` | `1` | 启动后是否发送一次预热请求 |

### 常用规划参数

| 环境变量 | 默认值 | 说明 |
|---|---:|---|
| `PLANNER_TASK_MANAGER_ENABLED` | `1` | 是否启用任务解析和阶段管理 |
| `PLANNER_CANDIDATE_COUNT` | `5` | 每轮候选轨迹数量 |
| `PLANNER_MAX_TRAJECTORY_LENGTH` | `5` | 每条候选轨迹最多动作数 |
| `PLANNER_ACTION_MODE` | `atomic` | `atomic` 或 `value`；当前推荐 `value` |
| `PLANNER_MAX_FORWARD_STEP` | `inf` | 单次 forward 上限；设数值可强制限速 |
| `PLANNER_MAX_TRACKING_YAW_STEP_DEG` | `inf` | 目标可见时 yaw 上限 |
| `PLANNER_APPROACH_STOP_MARGIN` | `1.0` | 接近目标时的停止余量；不要和到达半径混用 |
| `PLANNER_CONTEXT_ARRIVAL_DEPTH` | `5.0` | 到达半径，只用于 prompt 判断 `done=true/false` |
| `PLANNER_VELOCITY` | `0.5` | AirSim 移动速度 |
| `PLANNER_MAX_TOKENS` | `8192` | planner API 最大输出 token |
| `PLANNER_TASK_PARSER_MAX_TOKENS` | `2048` | 任务解析 API 最大输出 token |
| `PLANNER_TEMPERATURE` | `0.8` | 模型采样温度 |
| `WEB_PORT` | `5000` | Web 服务端口 |
| `MAX_STEPS` | `20` | 单个任务最大闭环步数 |

### 感知与记忆参数

| 环境变量 | 默认值 | 说明 |
|---|---:|---|
| `PLANNER_TARGET_DEPTH_ENABLED` | `1` | 是否启用目标 bbox 深度估计 |
| `PLANNER_SCENE_OBSTACLE_PLANNING_ENABLED` | `1` | 是否检测目标、障碍物和参考物并计算深度 |
| `PLANNER_CONTEXT_ENABLED` | `0` | 是否把轻量目标记忆注入 planner prompt |
| `PLANNER_CONTEXT_MAX_STEPS` | `5` | 注入历史执行摘要的步数 |
| `PLANNER_CONTEXT_CAMERA_FOV_DEG` | `90.0` | bbox 中心转方位角时使用的水平 FOV |
| `PLANNER_CONTEXT_WORLD_POS_UPDATE_ALPHA` | `0.35` | 目标世界坐标平滑系数 |
| `PLANNER_TARGET_IDENTITY_ENABLED` | `1` | 是否锁定首次目标实例 |
| `PLANNER_TARGET_IDENTITY_WORLD_TOLERANCE_ABS` | `15.0` | 目标实例世界坐标匹配半径下限 |
| `PLANNER_TARGET_IDENTITY_WORLD_TOLERANCE_RATIO` | `0.35` | 根据深度放大的匹配半径比例 |
| `PLANNER_RELOCALIZER_ENABLED` | `1` | 目标丢失后是否启用四向环视 |
| `PLANNER_RELOCALIZER_VIEW_COUNT` | `4` | 环视视角数量 |
| `PLANNER_RELOCALIZER_YAW_STEP_DEG` | `90.0` | 环视每次旋转角 |

## 运行方式

1. 启动 AirSim / Unreal 场景。
2. 确认 `settings.json` 已配置并重启 AirSim。
3. 启动 Web 闭环：

```bash
python run_airsim_web.py
```

4. 打开浏览器：

```text
http://localhost:5000
```

5. 输入任务，例如：

```text
飞到红色汽车旁
右转，找到第一辆车，飞到第一辆车旁边
飞到前方三岔路口，左转，找到一辆红色汽车，靠近它
```

## 任务阶段类型

TaskParser 会把任务拆成以下三类阶段：

| 类型 | 含义 | 示例 | 是否调用 planner |
|---|---|---|---|
| `action` | 固定动作，不需要视觉判断 | `右转90度`、`前进10m` | 否，直接执行 |
| `detect` | 只要求找到并锁定目标 | `找到红色汽车` | 否，只调用 bbox |
| `target` | 围绕目标或语义区域完成导航 | `飞到汽车旁`、`飞到三岔路口` | 是 |

阶段执行日志示例：

```text
[TASK PARSER] parsed 3 stages in 3.80s
[TASK] 任务阶段：→1:右转 | ·2:找到第一辆车 | ·3:飞到第一辆车旁边
[TASK] stage 1/3 mode=action: 右转
[Step 1/20] DIRECT actions=['right 90']
```

## 上下文、目标锁定与重定位

### ContextManager

当前上下文模块只保留两个必要功能：

- **目标记忆**：记录目标类别、bbox、深度、朝向和世界坐标。
- **状态机**：区分目标刚锁定、稳定跟踪、目标丢失和尚未锁定。

状态如下：

| 状态 | 作用 |
|---|---|
| `INIT_DETECT` | 首次可靠检测到目标，建立目标记忆 |
| `TRACKING` | 当前帧仍能看到锁定目标，正常规划 |
| `LOST_OR_OCCLUDED` | 当前帧看不到目标或检测不像原目标，避免追错同类目标 |
| `REDETECT` | 尚未形成目标记忆，需要继续检测 |

重定位、自动前进、到达判断都不属于 ContextManager。

### TargetIdentity

`TargetIdentityManager` 负责锁定首次目标实例。后续如果 VLM 检测到同类目标，但世界坐标与锁定目标差距过大，会拒绝该目标，防止“红车 A”切换成“红车 B”。

### Relocalizer

目标丢失时，普通 planner 不继续盲目前进，而是触发独立四向环视：

```text
[RELOCALIZE] target missing; scanning 4 views...
[RELOCALIZE] found view=2 yaw=159.3 conf=0.92 depth=7.76 world_pos=[...]
```

Relocalizer 只负责重新找目标候选，不直接决定最终轨迹。

## VLM 输出格式

planner API 应返回合法 JSON：

```json
{
  "selected_index": 0,
  "done": false,
  "scene_analysis": "当前目标在前方，右侧有障碍物",
  "reasoning_summary": "目标深度较远，路径基本可通行，选择单步较长前进",
  "candidates": [
    {
      "actions": ["forward 30"],
      "reason": "目标在前方较远且道路可通行",
      "scale": 1.0
    }
  ]
}
```

注意：

- `actions` 必须是字符串数组，例如 `["forward 30"]`
- 禁止输出对象动作，例如 `{"action": "forward", "value": 30}`
- `done=true` 只表示当前阶段完成
- 到达半径只用于判断 `done`，不能用 `target_depth - arrival_radius` 计算前进距离
- 接近目标时应使用 `PLANNER_APPROACH_STOP_MARGIN` 作为停止余量

## 深度估计说明

当前不是让 VLM 直接读深度图，而是：

1. VLM 根据 RGB 图输出目标和场景物体 bbox。
2. 程序把 RGB bbox 映射到 AirSim `DepthPerspective` 矩阵。
3. 在 bbox 区域内计算深度中位数、均值等统计。
4. 将目标、障碍物、参考物的深度以文本形式注入 planner prompt。

日志示例：

```text
[TARGET DEPTH] image_size=(1920, 1080) depth_shape=(360, 640)
[TARGET DEPTH] target=car rgb_bbox=[979, 529, 1037, 572] depth_bbox=[326, 176, 346, 191] median=38.375
[SCENE OBJECTS] obj1:target:car:画面中央偏右 depth=38.375; obj2:obstacle:tree:左前方 depth=7.60
```

## 本地 VLM 部署

当前支持在同一内网服务器上运行 VLM，在本机运行 AirSim 规划器。推荐流程见：

```text
本地部署.md
```

示例服务端命令：

```bash
CUDA_VISIBLE_DEVICES=0,1 vllm serve /data/sakura/models/Qwen3-VL-4B-Instruct \
  --host 0.0.0.0 --port 8000 \
  --max-model-len 8192 \
  --tensor-parallel-size 2
```

本机 `.env` 示例：

```bash
PLANNER_BASE_URL=http://<服务器IP>:8000/v1
PLANNER_API_KEY=111
PLANNER_MODEL_NAME=/data/sakura/models/Qwen3-VL-4B-Instruct
```

## Web 调试接口

| 地址 | 说明 |
|---|---|
| `/` | Web Dashboard |
| `/frame` | 当前 RGB 帧 |
| `/depth_frame` | 当前深度预览图 |
| `/events` | SSE 状态流 |
| `/debug_state` | 当前后端状态 JSON |

常用诊断字段：

```json
{
  "frame_version": 1,
  "depth_version": 1,
  "depth_bytes": 12345,
  "status": "waiting_task"
}
```

## 常见问题

### RGB 有画面，但深度图一直为空

常见原因：

- `settings.json` 没有被 AirSim 重新加载
- `DepthPerspective` 分辨率过高导致 RPC 卡住
- 相机名或场景配置不匹配

建议把 `ImageType: 2` 设置为 `640x360`，重启 AirSim 后再试。

### 模型报最大上下文长度错误

如果本地 vLLM 报类似：

```text
maximum context length is 8192 tokens
```

通常是 `PLANNER_MAX_TOKENS` 设置过大。降低为 `1024~2048`，或增大 vLLM 启动参数 `--max-model-len`。

### VLM 输出动作格式错误

当前 prompt 已要求：

```json
"actions": ["forward 30"]
```

如果模型输出对象动作，`response_parser.py` 可能无法解析。建议降低温度或更换结构化输出更稳定的模型。

### 目标接近后仍继续小步前进

注意区分两个参数：

- `PLANNER_CONTEXT_ARRIVAL_DEPTH`：到达半径，只用于判断 `done`
- `PLANNER_APPROACH_STOP_MARGIN`：接近停止余量，用于提示 planner 不要过度保守

如果仍然输出过小步长，后续应将距离控制进一步下沉到程序侧，而不是继续依赖 prompt。

### AirSim 取图较慢

当前系统每步通过 Python AirSim RPC 主动获取 RGB 和 Depth，再做图像转换和 base64 编码，因此 `capture` 可能达到数秒。后续可参考连续传感器流/帧缓存方式，只取最新帧，减少同步等待。

## 当前限制与后续方向

- VLM 直接生成轨迹仍不够稳定，后续应让 VLM 更多承担目标检测和语义定位。
- 轨迹规划应逐步转向本地几何规划器，参考 OnFly 的目标定位 + 本地规划思路。
- 当前多候选轨迹是优势，后续可由本地规划器生成多条可行轨迹，再用规则或打分器选择。
- AirSim 当前取图链路仍偏慢，后续考虑持续帧缓存和 RGB/Depth 更紧凑的数据流。

## 许可证

本项目沿用仓库中的许可证，详见 [`LICENSE`](LICENSE)。
