# Uni-LaViRA AirSim Web Planner

这是一个基于 **AirSim + Web Dashboard + 多模态大模型（MLLM/VLM）** 的无人机闭环导航原型。系统接收自然语言任务，实时采集无人机第一人称 RGB 图和深度信息，调用 OpenAI 兼容接口生成候选轨迹，并将动作转换为 AirSim 控制指令执行。

当前项目重点是 **AirSim 空中导航闭环**，不是原始 Uni-LaViRA 全量论文仓库结构。

## 核心能力

- **Web 交互**：在浏览器中输入任务、查看 RGB 画面、深度图、候选轨迹、推理摘要和执行状态。
- **VLM 规划**：通过 OpenAI 兼容 API 调用多模态模型，根据当前图像和任务生成候选动作。
- **目标深度估计**：VLM 负责定位目标 bbox，Python 从 AirSim `DepthPerspective` 原始深度矩阵中计算目标深度。
- **闭环执行**：每轮执行后重新观察、重新规划，避免一次性飞远。
- **安全约束**：限制单次最大前进距离和目标可见时的最大旋转角，降低飞过目标或偏离目标的风险。
- **调试可视化**：前端实时展示状态，后端打印目标 bbox、深度采样、候选动作和航点。

## 项目结构

| 路径 | 作用 |
|---|---|
| `run_airsim_web.py` | 主入口：连接 AirSim、启动 Web、运行闭环规划与执行 |
| `run_airsim_minimal.py` | 命令行版最小闭环运行脚本 |
| `sim/airsim_client.py` | AirSim RPC 封装：图像、深度、位姿、起飞、移动、碰撞检测 |
| `sim/frame_capturer.py` | 后台线程：持续抓取 RGB 和深度预览给前端 |
| `sim/action_executor.py` | 将 VLM 动作序列转换为 AirSim 航点并执行 |
| `agent/planner.py` | VLM 规划编排：目标深度估计、构造 prompt、调用模型、解析结果 |
| `agent/vlm_client.py` | OpenAI 兼容 Chat Completions API 客户端 |
| `agent/prompt_builder.py` | 图片编码、深度提示格式化、消息构造 |
| `agent/response_parser.py` | 解析 VLM JSON，裁剪不安全动作 |
| `agent/target_depth.py` | 目标 bbox 解析、多候选选择、bbox 内深度统计 |
| `planner/config.py` | Planner 配置项和默认参数 |
| `planner/prompt_templates.py` | 中文规划提示词模板 |
| `planner/trajectory.py` | 动作解析、位移计算、噪声模拟 |
| `web/` | Flask 后端、共享状态、Web 前端页面 |
| `本周工作.md` | 当前阶段工作汇报 |
| `深度图转换逻辑.md` | 深度图和目标深度估计方案说明 |

## 环境准备

建议使用 Python 3.10：

```bash
conda create -n airsim python=3.10
conda activate airsim
pip install -r requirements.txt
```

主要依赖：

- `airsim`：连接 AirSim / Unreal 仿真器
- `openai`：调用 OpenAI 兼容 MLLM API
- `Flask`：Web Dashboard
- `Pillow` / `numpy`：图像和深度矩阵处理

## AirSim 配置

AirSim 配置文件通常位于：

```text
C:\Users\<用户名>\Documents\AirSim\settings.json
```

推荐配置：RGB 保持高清，`DepthPerspective` 不要开到 1080p float，否则 AirSim RPC 可能长时间卡住。建议先使用 `640x360` 深度矩阵：

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
- 如果将 `DepthPerspective` 设置为 `1920x1080`，float 深度矩阵可能通过 RPC 传输过慢，导致深度帧一直为空

## API 配置

项目会读取系统环境变量，并自动加载项目根目录的 `.env`。首次使用时复制模板：

```bash
copy .env.example .env
```

然后在 `.env` 中填写自己的 API 地址、Key 和模型名。`.env` 已加入 `.gitignore`，不要提交真实 API key。

常用规划参数：

| 环境变量 | 默认值 | 说明 |
|---|---:|---|
| `PLANNER_CANDIDATE_COUNT` | `5` | 每轮候选轨迹数量 |
| `PLANNER_MAX_TRAJECTORY_LENGTH` | `5` | 每条候选轨迹最多动作数 |
| `PLANNER_ACTION_MODE` | `atomic` | `atomic` 输出离散动作；`value` 输出带数值动作 |
| `PLANNER_MAX_FORWARD_STEP` | `10.0` | 单次最大前进距离，防止一次飞太远 |
| `PLANNER_MAX_TRACKING_YAW_STEP_DEG` | `10.0` | 目标可见时单次最大旋转角 |
| `PLANNER_TARGET_DEPTH_ENABLED` | `1` | 是否启用目标 bbox 深度估计 |
| `PLANNER_VELOCITY` | `0.5` | AirSim 移动速度 |
| `PLANNER_CAPTURE_INTERVAL` | `0.1` | 前端抓帧间隔 |
| `PLANNER_DEPTH_PREVIEW_INTERVAL` | `2.0` | 前端深度图低频刷新间隔 |
| `PLANNER_MAX_TOKENS` | `8192` | 规划模型最大输出 token |
| `PLANNER_TEMPERATURE` | `0.8` | 模型采样温度 |
| `WEB_PORT` | `5000` | Web 服务端口 |
| `MAX_STEPS` | `20` | 单个任务最大闭环步数 |

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

5. 在页面输入任务，例如：

```text
飞到较远汽车旁
前进 10 米
靠近红色汽车
```

## 运行流程

```text
Web 输入任务
  -> AirSim 获取 RGB / DepthPerspective / 位姿
  -> VLM 定位目标 bbox
  -> Python 从原始深度矩阵计算目标深度
  -> VLM 生成候选轨迹 JSON
  -> 解析并裁剪不安全动作
  -> 转换为 AirSim 航点
  -> 执行动作
  -> 下一帧重新观察并规划
```

## VLM 输出格式

规划模型应返回 JSON，核心字段包括：

```json
{
  "selected_index": 0,
  "done": false,
  "scene_analysis": "一句话场景描述",
  "reasoning_summary": "目标、场景、深度、策略、候选轨迹和选择理由",
  "candidates": [
    {
      "actions": ["forward 10", "left 5"],
      "reason": "先向目标方向安全前进，再小角度修正",
      "scale": 1.0
    }
  ]
}
```

前端候选卡片只显示动作和位移，完整候选原因与最终选择原因显示在 `Reasoning` 区域。

## 深度估计说明

当前深度方案不是让模型直接读深度图，而是：

1. VLM 根据 RGB 图输出目标 `bbox_norm`。
2. 程序把 bbox 映射到 AirSim `DepthPerspective` 矩阵。
3. 在 bbox 区域中计算目标深度统计。
4. 将目标深度以文本形式注入规划 prompt。

调试日志示例：

```text
[TARGET DEPTH] image_size=(1920, 1080) depth_shape=(360, 640)
[TARGET DEPTH] candidates: target=car raw_bbox=... rgb_bbox=... depth_bbox=... median=...
[TARGET DEPTH] chosen: target=car rgb_bbox=... depth_bbox=... median=...
```

如果 `depth_shape` 仍是 `(144, 256)`，说明 AirSim 深度配置没有生效或场景未重启。

## 安全约束

系统同时依赖 prompt 和程序硬限制：

- 单次 `forward` 不超过 `PLANNER_MAX_FORWARD_STEP`
- 目标可见时 `left/right` 不超过 `PLANNER_MAX_TRACKING_YAW_STEP_DEG`
- 只有目标不可见时才允许大角度旋转搜索
- VLM 输出超限动作时，`response_parser.py` 会在执行前裁剪

这可以避免深度估计偏大时无人机一次性飞远，也可以避免目标可见时大角度旋转导致偏离目标。

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
  "depth_version": 0,
  "depth_bytes": 0,
  "status": "waiting_task"
}
```

含义：

- `frame_version > 0`：RGB 已正常采集
- `depth_version > 0` 且 `depth_bytes > 0`：深度预览已正常生成
- `depth_version = 0` 且 `depth_bytes = 0`：AirSim 没有返回可用 DepthPerspective

## 常见问题

### 1. RGB 有画面，但深度图一直是 `bytes=0`

先看终端是否有：

```text
[FrameCapturer] empty_depth_response: width=... height=... floats=0
```

常见原因：

- `settings.json` 没有被 AirSim 重新加载
- `DepthPerspective` 分辨率设置过高导致 RPC 卡住
- 相机名或场景配置不匹配

建议将 `ImageType: 2` 设置为 `640x360`，重启 AirSim 后再试。

### 2. 远处目标深度偏大或偏小

远处目标 bbox 映射到低分辨率深度矩阵后可能只有少量像素，容易采到背景。建议：

- 将 `DepthPerspective` 提高到 `640x360` 或 `1280x720`
- 不建议直接使用 `1920x1080` float 深度矩阵
- 查看 `[TARGET DEPTH]` 日志中的 `depth_bbox`、`median/min/mean`

### 3. 模型输出 JSON 不稳定

系统已有基础容错和重试，但仍建议：

- 使用支持图像输入和结构化输出能力较好的模型
- 保持 `PLANNER_CANDIDATE_COUNT=5`
- 查看终端 `[VLM RAW]` 或 `[TARGET DEPTH] raw bbox response`

### 4. 无人机一次飞太远

设置：

```bash
set PLANNER_MAX_FORWARD_STEP=10
```

即使模型输出 `forward 100`，程序也会裁剪到安全上限。

### 5. 目标可见时旋转角过大

设置：

```bash
set PLANNER_MAX_TRACKING_YAW_STEP_DEG=10
```

目标可见时，程序会把 `left/right` 裁剪到该范围内；目标不可见时保留大角度搜索能力。

## 当前限制与后续计划

当前系统仍是研究原型，主要限制包括：

- 目标跟踪还未形成完整状态机，单帧 VLM 可能误切换目标
- 远目标深度受深度图分辨率和 bbox 精度影响
- 任务完成判断仍需结合历史目标深度、累计前进距离和多帧确认
- AirSim 高分辨率 float 深度矩阵通过 RPC 传输较慢，不适合直接开到 1080p

后续建议实现 `TargetTracker`，维护 `SEARCHING / TRACKING / NEAR_TARGET / ARRIVED / LOST_TARGET` 状态，避免到达目标附近后因单帧看不到目标而追逐假目标。

## 许可证

本项目沿用仓库中的许可证，详见 [`LICENSE`](LICENSE)。
