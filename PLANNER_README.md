# 基于 VLM 的 Zero-Shot 规划器——完整开发指南

> 本文档面向**对代码没有任何了解的开发者**，逐阶段说明如何基于现有的 Uni-LaViRA 仓库，实现一个基于 VLM（视觉语言大模型）+ Agent 的零样本规划器。规划器的核心任务：接收当前帧图像 + 文本任务目标，输出由原子动作组成的候选轨迹序列。

---

## 第一章：总体目标与架构

### 1.1 我们要做什么

我们要实现一个规划器模块，它：
- **输入**：无人机当前的前方 RGB 图像（单张图片）+ 文本描述的任务目标（如"飞到东北方向的塔吊上方"）
- **输出**：K 条候选轨迹，每条轨迹是一串原子动作的序列，形如：

```
候选轨迹 1: ["forward", "forward", "left", "forward"]
候选轨迹 2: ["forward", "left", "forward"]
候选轨迹 3: ["left", "forward", "forward", "forward"]
...
```

- **方式**：完全不训练，只靠写 prompt + 调 VLM API，零样本运行

### 1.2 系统中的位置

```
当前帧图像 + 任务文本
        │
        ▼
┌─────────────────────────────────┐
│        规划器 (Planner)          │
│  ┌───────────────────────────┐  │
│  │  Agent LLM (推理决策层)     │  │──────── K 条候选轨迹
│  │  → 理解场景 + 任务         │  │         (动作序列)
│  │  → 生成多个候选路径        │  │
│  │  → 每个路径是原子动作序列   │  │
│  └───────────────────────────┘  │
└─────────────────────────────────┘
        │
        ▼
    世界模型 → 预测每条的下一帧
        │
        ▼
    打分器 → 选出最优轨迹
        │
        ▼
    Agent/仿真器 → 执行动作 → 获得真实下一帧 → 回到规划器循环
```

### 1.3 技术路线（我们的方法）

- **VLM 选型**：使用支持视觉理解的大模型（如 GPT-4V、Qwen-VL、Gemini Vision 等），统一通过 OpenAI 兼容 API 调用
- **Agent 设计**：用一个 VLM 实例同时承担"场景理解 + 规划生成"两个角色，不拆分为多个模型
- **候选生成策略**：通过精心设计的 prompt，让 VLM 直接输出多条候选轨迹（而非仅输出一条最优路径）
- **动作空间**：5 个原子动作——`forward`（前进0.1m）、`left`（左转15°）、`right`（右转15°）、`up`（上升0.1m）、`down`（下降0.1m）

### 1.4 打分器复用 VLM 的可能性

后续打分器可以复用规划器的同一套 VLM 来完成评分：
- 将候选轨迹 + 世界模型预测的下一帧发送给 VLM
- 让 VLM 判断：这个预测结果是否符合任务目标？符合程度如何？
- 这样可以省去训练专门的打分模型，并利用 VLM 的语义理解能力对复杂场景进行评分

---

## 第二章：基础概念速查

### 2.1 原子动作详解

| 动作名 | 中文名 | 效果 |
|--------|--------|------|
| `forward` | 前进 | 沿当前朝向前进 0.1 米 |
| `left` | 左转 | 原地向左转 15° |
| `right` | 右转 | 原地向右转 15° |
| `up` | 上升 | 上升 0.1 米 |
| `down` | 下降 | 下降 0.1 米 |

步长和转角都是默认值，可以在 `PlannerConfig` 中修改。

### 2.2 轨迹的数据结构

一条轨迹包含三个字段：
```
Trajectory:
  actions: ["forward", "left", "forward", ...]    # 动作名列表
  clean_delta: [dx, dy, dz, dphi]                  # 无噪声的终点状态
  noisy_delta: [dx, dy, dz, dphi]                  # 加噪声的终点状态
```

其中 `[dx, dy, dz, dphi]` 的含义：
- `dx`：终点 X 坐标 - 起点 X 坐标（米），使用全局坐标系
- `dy`：终点 Y 坐标 - 起点 Y 坐标（米）
- `dz`：终点 Z 坐标 - 起点 Z 坐标（米），正值=下降，负值=上升
- `dphi`：终点偏航角 - 起点偏航角（度），正值=向右转

例如轨迹 `["left", "forward"]`：
```
起点: (0, 0, 0)，朝向 0°
  left    → 朝向改为 15°
  forward → 沿 15° 方向走 0.1m → 位置 (0.097, 0.026, 0)
最终: 位置 (0.097, 0.026, 0)，朝向 15°
clean_delta: [0.097, 0.026, 0, 15.0]
```

### 2.3 代码目录结构说明

```
uni-lavira-code-main/
├── sim-code/
│   └── airsim/               ← 无人机仿真环境代码
│       ├── unilavira_evaluator.py    ← 评估入口
│       ├── scripts/                 ← 运行脚本
│       │   ├── unilavira_eval.sh
│       │   └── dagger_NYC.sh
│       └── src/
│           ├── model_wrapper/
│           │   ├── unilavira_model.py   ← 已有零样本VLM导航代码（重要参考！）
│           │   ├── travel_llm.py        ← 已有训练好的轨迹模型
│           │   └── base_model.py        ← 模型基类
│           ├── vlnce_src/
│           │   ├── eval.py              ← 评估主循环（重要参考！）
│           │   ├── closeloop_util.py    ← 闭环状态管理
│           │   └── env_uav.py           ← AirSim仿真环境接口
│           └── common/
│               └── param.py             ← 参数配置
├── planner/                   ← ★ 将要创建的新规划器模块
│   ├── __init__.py
│   ├── vlm_planner.py         ← VLM规划器核心类
│   ├── prompt_templates.py    ← prompt模板
│   ├── trajectory.py          ← 轨迹数据结构
│   └── config.py              ← 规划器配置
└── PLANNER_README.md          ← 本文件
```

---

## 第三章：五阶段实施计划

整个实现分为 5 个阶段，每个阶段完成一个可验证的里程碑。

### 阶段零：理解已有代码模板

在你开始编码之前，先花时间理解下面两个关键文件。它们是新规划器最重要的参考：

**文件 A：`sim-code/airsim/src/model_wrapper/unilavira_model.py`**

这是本仓库已有的零样本 VLM 导航代码。关键结构如下：

```python
class ZeroShotVlnEvaluatorMP(BaseModelWrapper):
    def __init__(self, ...):
        # 初始化 VLM 客户端（两个模型）
        self.model = OpenAIVisionClient(
            model_name="qwen3.5-27b",          # VA 模型（用于视觉定位）
            secondary_model_name="gemini-3.5-flash"  # LA 模型（用于文字推理）
        )
    
    def query_llm(self, instruction, position, ...):
        # 【重点参考】构建 prompt，将当前帧+任务发给 VLM
        # 输出：target可见？方向？决策（ascend/descend/move/backtrack）？
        
    def query_image(self, rgb, depth, ...):
        # 在选定方向后，估算具体前进距离
        
    def run(self, inputs, episodes, ...):
        # 执行循环：query_llm → query_image → 生成waypoints
```

**关键要点**：这个已有实现展示了如何将图像编码为 base64、如何构造多轮对话、如何解析 VLM 的 JSON 输出。但它的问题是：规划在"方向"层面，而不是在"原子动作序列"层面。

**文件 B：`sim-code/airsim/src/vlnce_src/eval.py`**

这是评估主循环，展示了如何串联各个模块：

```python
# 核心循环结构
for t in range(maxWaypoints + 1):
    # 1. 检查终止
    batch_state.check_batch_termination(t)
    
    # 2. 调用模型，生成waypoints
    waypoints = model_wrapper.run(inputs, episodes, ...)
    
    # 3. 执行动作
    eval_env.makeActions(waypoints)
    
    # 4. 获取新观测
    outputs = eval_env.get_obs()
    
    # 5. 更新状态
    batch_state.update_from_env_output(outputs)
```

---

### 阶段一：创建规划器的基础数据结构（代码完成即可验证）

**目标**：定义轨迹、动作、配置的数据类型，不涉及 VLM 调用。

**步骤 1.1：新建 `planner/` 目录**

在仓库根目录创建新文件夹：
```
uni-lavira-code-main/planner/
```

**步骤 1.2：创建 `planner/trajectory.py`**

这个文件定义了轨迹最核心的数据结构。直接复制已有 `PLANNER_README.md` 中的数据结构设计：

```python
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Tuple

class AtomicAction(Enum):
    """5种原子动作"""
    FORWARD = "forward"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"

@dataclass
class Delta:
    """轨迹终点相对于起点的位移和偏航"""
    dx: float = 0.0    # X方向位移（米）
    dy: float = 0.0    # Y方向位移（米）
    dz: float = 0.0    # Z方向位移（米），正=下降，负=上升
    dphi: float = 0.0  # 偏航变化（度），正=右转
    
    def as_list(self) -> List[float]:
        return [self.dx, self.dy, self.dz, self.dphi]

@dataclass
class Trajectory:
    """一条候选轨迹"""
    actions: List[str] = field(default_factory=list)  # 动作序列
    clean_delta: Delta = field(default_factory=Delta) # 无噪声终点
    noisy_delta: Delta = field(default_factory=Delta) # 加噪声终点
    name: str = "unnamed"  # 候选轨迹名称
    
    def __len__(self) -> int:
        return len(self.actions)
```

**步骤 1.3：创建 `planner/config.py`**

配置类，用于控制规划器的所有参数：

```python
from dataclasses import dataclass, field
from typing import Optional

@dataclass
class PlannerConfig:
    """规划器配置"""
    # VLM API 配置（对接现有仓库的配置体系）
    la_base_url: str = "http://localhost:8000/v1"
    la_model_name: str = "Qwen3.5-27B-Q4_K_M"
    la_api_key: str = ""
    va_base_url: str = "http://localhost:8000/v1"
    va_model_name: str = "Qwen3.5-27B-Q4_K_M"
    va_api_key: str = ""
    
    # 动作参数
    horizontal_step: float = 0.1        # forward步长（米）
    vertical_step: float = 0.1          # up/down步长（米）
    yaw_step_deg: float = 15.0          # left/right转角（度）
    
    # 候选轨迹参数
    candidate_count: int = 5            # 每次生成多少条候选轨迹
    max_trajectory_length: int = 5      # 单条轨迹的最大动作数
    min_trajectory_length: int = 1      # 单条轨迹的最小动作数
    temperature: float = 0.8            # VLM 采样温度，越高越多样化
    seed: Optional[int] = None          # 随机种子
    
    # Delta计算参数
    noise_sigma_dx: float = 0.005       # dx噪声标准差（米）
    noise_sigma_dy: float = 0.002       # dy噪声标准差（米）
    noise_sigma_dz: float = 0.003       # dz噪声标准差（米）
    noise_sigma_dphi: float = 0.5       # dphi噪声标准差（度）
```

**步骤 1.4：创建 `planner/__init__.py`**

导出所有公共类：

```python
from .trajectory import AtomicAction, Delta, Trajectory
from .config import PlannerConfig
```

**验证方式**：写一个简单测试脚本：

```python
# 在 planner/ 目录下运行
from planner import Trajectory, Delta, AtomicAction, PlannerConfig

# 创建一个轨迹
traj = Trajectory(
    actions=["forward", "left", "forward"],
    name="demo"
)
print(f"轨迹: {traj.actions}")    
print(f"长度: {len(traj)}步")

# 创建配置
config = PlannerConfig(
    candidate_count=5,
    max_trajectory_length=3,
    temperature=0.8
)
print(f"候选数: {config.candidate_count}")
```

---

### 阶段二：Delta 计算器（代码完成即可验证）

**目标**：给定一条原子动作序列，计算出执行后的终点位移和偏航。

**步骤 2.1：在 `planner/trajectory.py` 中添加 `compute_delta` 函数**

```python
import math
import random
import numpy as np

def compute_delta(actions: List[str], config: PlannerConfig) -> Delta:
    """
    根据原子动作序列，计算机器人从(0,0,0)出发后的最终状态。
    
    计算过程：
    - forward: 沿当前朝向走 horizontal_step 米
    - left: 朝向减 yaw_step_deg 度
    - right: 朝向加 yaw_step_deg 度
    - up: z 减 vertical_step（AirSim中Z向上为正？需根据实际坐标系确认）
    - down: z 加 vertical_step
    """
    x, y, z = 0.0, 0.0, 0.0
    yaw = 0.0  # 朝向，度
    
    for action in actions:
        if action == "forward":
            rad = math.radians(yaw)
            x += config.horizontal_step * math.cos(rad)
            y += config.horizontal_step * math.sin(rad)
        elif action == "left":
            yaw -= config.yaw_step_deg
        elif action == "right":
            yaw += config.yaw_step_deg
        elif action == "up":
            z -= config.vertical_step  # 上升，z减小
        elif action == "down":
            z += config.vertical_step  # 下降，z增大
    
    return Delta(dx=x, dy=y, dz=z, dphi=yaw)


def add_noise(delta: Delta, config: PlannerConfig) -> Delta:
    """给delta加高斯噪声，模拟传感器误差"""
    return Delta(
        dx=delta.dx + random.gauss(0, config.noise_sigma_dx),
        dy=delta.dy + random.gauss(0, config.noise_sigma_dy),
        dz=delta.dz + random.gauss(0, config.noise_sigma_dz),
        dphi=delta.dphi + random.gauss(0, config.noise_sigma_dphi),
    )
```

**验证方式**：

```python
from planner import Trajectory, Delta, PlannerConfig, trajectory

config = PlannerConfig(horizontal_step=0.1, yaw_step_deg=15.0)

# 测试纯前进
delta = compute_delta(["forward", "forward"], config)
print(delta.as_list())  # 期望: [0.2, 0, 0, 0]

# 测试左转+前进
delta = compute_delta(["left", "forward"], config)
print(delta.as_list())  # 期望靠近: [0.097, 0.026, 0, 15.0]

# 测试完整轨迹
delta = compute_delta(["left", "left", "forward", "forward", "forward"], config)
print(delta.as_list())  # 期望靠近: [0.260, 0.150, 0, 30.0]
```

---

### 阶段三：实现 VLM Prompt 规划器（核心里程碑）

**目标**：通过调用 VLM API，让大模型根据当前帧图像+任务文本，直接生成候选轨迹。

**步骤 3.1：创建 `planner/prompt_templates.py`**

精心设计的 prompt 是零样本规划器的核心。下面是一个完整的 prompt 模板：

```python
PLANNER_SYSTEM_PROMPT = """你是一个无人机导航规划器。你的任务是：
1. 观察当前无人机的第一人称视角图像
2. 理解文本描述的任务目标
3. 生成 K 条不同的候选动作序列（轨迹）

动作空间（每个动作的坐标变换）：
- forward: 沿当前朝向飞行 0.1 米
- left: 原地向左转 15°
- right: 原地向右转 15°
- up: 上升 0.1 米
- down: 下降 0.1 米

注意事项：
- 每条轨迹应针对当前场景提出一个可能的下一步移动方案
- 轨迹之间应该多样化：一些偏左、一些偏右、一些升高、一些降低
- 每条轨迹的长度可以不同，从 1 到 3 个动作
- 优先考虑向任务目标方向移动的动作
- 如果障碍物看起来很近，可以考虑左右转向绕开，或升降以避开

输出格式：返回严格的 JSON，格式如下：
{
  "scene_analysis": "一句话描述当前场景和需要往哪个方向走",
  "candidates": [
    {
      "actions": ["forward", "left", "forward"],
      "reason": "为什么选这个轨迹"
    },
    {
      "actions": ["right", "forward", "forward"],
      "reason": "备选方向"
    }
  ]
}

不要输出 Markdown 代码块，只输出纯 JSON。""" 

TASK_INSTRUCTION_PROMPT = "任务目标：{task_description}。无人机当前画面如上所示。请生成 {k} 条不同的候选动作轨迹。"
```

提示模板的设计要点：
- 明确告诉 VLM 它的角色、输入、输出格式
- 给出具体的动作空间定义（步长、转角值）
- 要求 VLM 输出 JSON 格式，方便程序解析
- 包含 `scene_analysis` 让 VLM 先理解场景再规划
- 每条候选都附带 `reason` 解释为什么要这么走

**步骤 3.2：创建 `planner/vlm_planner.py`**

这是规划器的核心类，负责：
1. 调用 VLM API
2. 解析返回的 JSON
3. 将 JSON 中的候选动作序列转换为 `Trajectory` 对象
4. 计算每条轨迹的 Delta

```python
import json
import re
import base64
import io
import math
from typing import List, Dict, Optional
from PIL import Image
import numpy as np
from openai import OpenAI

from planner import Trajectory, Delta, PlannerConfig
from planner.trajectory import compute_delta, add_noise
from planner.prompt_templates import PLANNER_SYSTEM_PROMPT, TASK_INSTRUCTION_PROMPT


class VLMPlanner:
    """
    基于 VLM 的零样本轨迹规划器。
    
    核心方法：
    - generate_candidates(current_frame, task): 生成候选轨迹
    """
    
    def __init__(self, config: PlannerConfig):
        self.config = config
        
        # 创建 OpenAI 兼容客户端（可对接各种 VLM）
        self.client = OpenAI(
            api_key=config.la_api_key or "no-key",
            base_url=config.la_base_url
        )
        self.model_name = config.la_model_name
    
    def _encode_image(self, image) -> str:
        """将图像编码为 base64 字符串"""
        if isinstance(image, np.ndarray):
            image = Image.fromarray(image)
        elif isinstance(image, str):
            # 如果传入的是文件路径
            image = Image.open(image)
        
        buffered = io.BytesIO()
        image.save(buffered, format="PNG")
        return base64.b64encode(buffered.getvalue()).decode("utf-8")
    
    def _parse_vlm_response(self, response_text: str) -> List[Dict]:
        """解析 VLM 返回的 JSON"""
        # 清理可能的 markdown 代码块标记
        text = re.sub(r'```json\s*', '', response_text)
        text = re.sub(r'```\s*', '', text)
        
        # 提取 JSON 部分
        json_match = re.search(r'\{.*\}', text, re.DOTALL)
        if not json_match:
            raise ValueError(f"无法从 VLM 响应中提取 JSON: {response_text[:200]}")
        
        data = json.loads(json_match.group())
        return data["candidates"]
    
    def generate_candidates(
        self, 
        current_frame, 
        task_description: str,
        k: int = None
    ) -> List[Trajectory]:
        """
        核心方法：基于当前帧和任务描述，生成候选轨迹。
        
        参数:
            current_frame: numpy 数组或 PIL Image 或文件路径
            task_description: 任务描述文本
            k: 需要生成的候选数量（默认使用配置的 candidate_count）
        
        返回:
            List[Trajectory]: K 条候选轨迹
        """
        if k is None:
            k = self.config.candidate_count
        
        # 1. 编码图像
        base64_image = self._encode_image(current_frame)
        
        # 2. 构建 VLM 调用消息
        messages = [
            {"role": "system", "content": PLANNER_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/png;base64,{base64_image}"
                        }
                    },
                    {
                        "type": "text",
                        "text": TASK_INSTRUCTION_PROMPT.format(
                            task_description=task_description,
                            k=k
                        )
                    }
                ]
            }
        ]
        
        # 3. 调用 VLM
        response = self.client.chat.completions.create(
            model=self.model_name,
            messages=messages,
            max_tokens=4096,
            temperature=self.config.temperature
        )
        
        response_text = response.choices[0].message.content
        print(f"[VLM Planner] 原始响应:\n{response_text}")
        
        # 4. 解析 JSON
        candidate_dicts = self._parse_vlm_response(response_text)
        
        # 5. 转换为 Trajectory 对象并计算 Delta
        trajectories = []
        for i, cand in enumerate(candidate_dicts):
            actions = cand["actions"]
            reason = cand.get("reason", "")
            
            # 计算 clean delta
            clean_delta = compute_delta(actions, self.config)
            
            # 计算 noisy delta（可选）
            noisy_delta = add_noise(clean_delta, self.config)
            
            traj = Trajectory(
                actions=actions,
                clean_delta=clean_delta,
                noisy_delta=noisy_delta,
                name=f"vlm_candidate_{i+1}"
            )
            
            print(f"  候选 {i+1}: actions={actions}, delta={clean_delta.as_list()}, reason={reason}")
            trajectories.append(traj)
        
        return trajectories
```

**步骤 3.3：验证方式**

写一个测试脚本，验证是否能成功调用 VLM 并返回结构化候选轨迹：

```python
# test_vlm_planner.py
from planner import VLMPlanner, PlannerConfig
import cv2

# 1. 配置
config = PlannerConfig(
    la_base_url="http://localhost:8000/v1",
    la_model_name="Qwen3.5-27B-Q4_K_M",
    candidate_count=5,
    temperature=0.8
)

# 2. 初始化规划器
planner = VLMPlanner(config)

# 3. 读取当前帧（从文件或仿真器获取）
frame = cv2.imread("test_frame.png")

# 4. 生成候选轨迹
candidates = planner.generate_candidates(
    current_frame=frame,
    task_description="向东北方向飞，绕过前方的建筑物，然后飞到塔吊上方",
    k=5
)

# 5. 打印结果
for traj in candidates:
    print(f"{traj.name}: {traj.actions} → {traj.clean_delta.as_list()}")
```

---

### 阶段四：集成到仿真闭环（可在 AirSim 中运行验证）

**目标**：将规划器接入现有的 AirSim 仿真评估循环，实现闭环导航。

**步骤 4.1：理解现有评估循环（参考 `eval.py`）**

现有的评估循环结构：
```python
# 在 sim-code/airsim/src/vlnce_src/eval.py 中
for t in range(maxWaypoints + 1):
    batch_state.check_batch_termination(t)
    
    # 调用模型（这里替换为我们的规划器）
    refined_waypoints = model_wrapper.run(inputs, episodes, ...)
    
    # 执行动作
    eval_env.makeActions(refined_waypoints)
    
    # 获取新观测
    outputs = eval_env.get_obs()
    
    # 更新状态
    batch_state.update_from_env_output(outputs)
```

**步骤 4.2：创建新的评估脚本**

新建 `sim-code/airsim/src/vlnce_src/vlm_eval.py`：

```python
"""
基于 VLM 规划器的闭环评估脚本。
替换原有的 TravelModelWrapper / ZeroShotVlnEvaluatorMP，使用 VLMPlanner。
"""
import os
import json
import numpy as np
import torch
from pathlib import Path
import sys
sys.path.append(str(Path(os.getcwd()).resolve()))

from src.common.param import args
from src.vlnce_src.closeloop_util import EvalBatchState, BatchIterator
from src.vlnce_src.env_uav import AirVLNENV
from src.vlnce_src.assist import Assist

# 导入我们新的规划器
from planner import VLMPlanner, PlannerConfig


class VLMPlannerWrapper:
    """
    将 VLMPlanner 包装为 AirSim 评估循环可调用的接口。
    使其兼容现有 eval.py 的调用方式。
    """
    
    def __init__(self, config: PlannerConfig):
        self.planner = VLMPlanner(config)
        self.config = config
        self._ep_state = {}  # 记录每个 episode 的状态
    
    def prepare_inputs(self, episodes, target_positions, assist_notices=None):
        """准备模型输入（兼容现有接口）"""
        bs = len(episodes)
        inputs = []
        for env_idx in range(bs):
            ep = episodes[env_idx]
            last_obs = ep[-1]
            
            # 获取当前帧（RGB前方视图）
            frame = last_obs['rgb'][0]  # front view
            
            # 获取任务指令
            instruction = self._get_instruction(ep)
            
            # 存储到状态
            if env_idx not in self._ep_state:
                self._ep_state[env_idx] = {
                    "frame": frame,
                    "instruction": instruction,
                    "step": 0,
                }
            else:
                self._ep_state[env_idx]["frame"] = frame
                self._ep_state[env_idx]["step"] += 1
            
            inputs.append({
                "frame": frame,
                "instruction": instruction,
            })
        
        return inputs, [None] * bs
    
    def _get_instruction(self, episode):
        """从 episode 中提取指令"""
        if len(episode) > 0:
            obs = episode[0]
            if 'instruction' in obs:
                ins = obs['instruction']
                if isinstance(ins, dict) and 'text' in ins:
                    return ins['text']
                if isinstance(ins, str):
                    return ins
        return "Find the target object described in the task."
    
    def run(self, inputs, episodes, rot_to_targets=None):
        """
        核心运行方法：对每个 episode，调用 VLMPlanner 生成候选轨迹，
        从候选中选择第一条作为当前步的执行轨迹（后续打分器会替换这一步）。
        """
        all_waypoints = []
        
        for env_idx in range(len(episodes)):
            inp = inputs[env_idx]
            frame = inp["frame"]
            instruction = inp["instruction"]
            
            # 调用 VLM 规划器生成候选轨迹
            candidates = self.planner.generate_candidates(
                current_frame=frame,
                task_description=instruction,
                k=self.config.candidate_count
            )
            
            # ★ 临时方案：选择第一条候选（后续会被打分器替换）
            best_traj = candidates[0]
            
            # 将最佳轨迹转换为 waypoints（AirSim 执行格式）
            waypoints = self._trajectory_to_waypoints(
                best_traj, 
                episodes[env_idx][-1]['sensors']['state']['position']
            )
            all_waypoints.append(waypoints)
        
        return all_waypoints
    
    def _trajectory_to_waypoints(self, trajectory, current_position):
        """
        将 Trajectory 对象转换为 AirSim 可执行的 waypoints。
        
        这里有两种策略：
        策略A：将整个轨迹展开为多个中间 waypoint（精细控制）
        策略B：直接将终点作为单个 waypoint（快速执行）
        
        推荐策略A：每个动作生成一个 waypoint。
        """
        x, y, z = current_position[0], current_position[1], current_position[2]
        yaw = 0.0  # 需要从当前状态的 orientation 中解析
        
        waypoints = []
        for action in trajectory.actions:
            if action == "forward":
                rad = math.radians(yaw)
                x += self.config.horizontal_step * math.cos(rad)
                y += self.config.horizontal_step * math.sin(rad)
            elif action == "left":
                yaw -= self.config.yaw_step_deg
            elif action == "right":
                yaw += self.config.yaw_step_deg
            elif action == "up":
                z -= self.config.vertical_step
            elif action == "down":
                z += self.config.vertical_step
            
            waypoints.append([x, y, z])
        
        return waypoints
    
    def eval(self):
        """评估模式（对于 VLM 来说就是推理，不需要特殊处理）"""
        pass
    
    def predict_done(self, episodes, object_infos):
        """判断是否到达目的地（目前使用 DINO 检测）"""
        return [False] * len(episodes)
    
    def pop_llm_replies(self, env_idx):
        """获取并清空 LLM 回复记录"""
        return []
```

**步骤 4.3：创建运行入口脚本**

新建 `sim-code/airsim/scripts/vlm_planner_eval.sh`：

```bash
#!/bin/bash
# VLM 规划器评估脚本

export VA_API_KEY="your_api_key"
export VA_BASE_URL="http://localhost:8000/v1"
export VA_MODEL_NAME="Qwen3.5-27B-Q4_K_M"
export LA_API_KEY="your_api_key"
export LA_BASE_URL="http://localhost:8000/v1"
export LA_MODEL_NAME="Qwen3.5-27B-Q4_K_M"

python -m src.vlnce_src.vlm_eval \
    --dataset_path /path/to/dataset \
    --eval_save_path /path/to/save \
    --batchSize 4
```

**步骤 4.4：完整的评估主循环**

```python
# sim-code/airsim/src/vlnce_src/vlm_eval.py 后半部分

def eval_vlm_planner():
    """VLM 规划器评估入口"""
    from src.common.param import args
    from src.vlnce_src.closeloop_util import initialize_env_eval, setup
    
    # 1. 初始化环境
    setup()
    eval_env = initialize_env_eval(
        dataset_path=args.dataset_path,
        save_path=args.eval_save_path,
        eval_json_path=args.eval_json_path
    )
    
    # 2. 初始化 VLM 规划器
    planner_config = PlannerConfig(
        la_base_url=os.environ.get("LA_BASE_URL", "http://localhost:8000/v1"),
        la_model_name=os.environ.get("LA_MODEL_NAME", "Qwen3.5-27B-Q4_K_M"),
        candidate_count=5,
        temperature=0.8
    )
    model_wrapper = VLMPlannerWrapper(planner_config)
    assist = Assist(always_help=False, use_gt=False)
    
    # 3. 开始评估（循环逻辑复用了已有的 BatchIterator 和 BatchState）
    with torch.no_grad():
        dataset = BatchIterator(eval_env)
        while True:
            env_batchs = eval_env.next_minibatch()
            if env_batchs is None:
                break
            
            batch_state = EvalBatchState(
                batch_size=eval_env.batch_size,
                env_batchs=env_batchs,
                env=eval_env,
                assist=assist,
                model_wrapper=model_wrapper
            )
            
            for t in range(int(args.maxWaypoints) + 1):
                if batch_state.check_batch_termination(t):
                    break
                
                # 准备输入 → 调用规划器 → 执行 → 获取新观测
                inputs, _ = model_wrapper.prepare_inputs(
                    batch_state.episodes, 
                    batch_state.target_positions
                )
                waypoints = model_wrapper.run(
                    inputs=inputs,
                    episodes=batch_state.episodes
                )
                eval_env.makeActions(waypoints)
                outputs = eval_env.get_obs()
                batch_state.update_from_env_output(outputs)
                batch_state.update_metric()


if __name__ == "__main__":
    eval_vlm_planner()
```

---

### 阶段五：候选轨迹的多轮迭代与打分器接口预留

**目标**：完善规划器的迭代能力，为后续打分器接入预留接口。

**步骤 5.1：在 `VLMPlanner` 中添加历史状态传递**

将历史观测和已执行轨迹作为上下文传给 VLM，使其能做出更好的决策：

```python
class VLMPlanner:
    def __init__(self, config: PlannerConfig):
        # ... 现有初始化 ...
        self.history = []  # 记录历史帧和动作
    
    def generate_candidates_with_history(
        self,
        current_frame,
        task_description: str,
        previous_actions: List[str] = None,
        k: int = None
    ):
        """带历史上下文的规划"""
        if previous_actions is None:
            previous_actions = []
        
        # 构建带历史的消息
        messages = [
            {"role": "system", "content": PLANNER_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{self._encode_image(current_frame)}"}},
                    {"type": "text", "text": f"任务目标：{task_description}"},
                    {"type": "text", "text": f"已执行的历史动作：{previous_actions}"},
                    {"type": "text", "text": f"请生成 {k or self.config.candidate_count} 条下一步候选动作，考虑刚刚执行过的动作避免重复。"},
                ]
            }
        ]
        
        # ... 后续同 generate_candidates ...
```

**步骤 5.2：预留打分器接口**

在 `VLMPlanner` 中增加一个方法，供后续打分器调用：

```python
class VLMPlanner:
    def score_trajectory(self, current_frame, task_description, trajectory: Trajectory, predicted_next_frame) -> float:
        """
        为一条候选轨迹打分（后续打分器可用）。
        
        这个函数可以：
        1. 将轨迹的预测下一帧+当前帧+任务发给 VLM
        2. 让 VLM 判断这条轨迹是否符合任务目标
        3. 返回一个分数（0~1）
        
        ★ 这就是"打分器复用 VLM"的接口！
        """
        # 构建打分 prompt
        scoring_prompt = f"""
        任务目标：{task_description}
        
        候选动作轨迹：{trajectory.actions}
        
        请观察"当前帧"和"执行该候选轨迹后的预测帧"两张图片，
        判断这个候选轨迹是否朝着任务目标前进。
        
        输出格式（JSON）：
        {{
            "score": 0.0~1.0,
            "reason": "打分的理由"
        }}
        """
        
        messages = [
            {"role": "system", "content": "你是一个轨迹评分器。"},
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{self._encode_image(current_frame)}"}},
                    {"type": "text", "text": "（当前帧）"},
                    {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{self._encode_image(predicted_next_frame)}"}},
                    {"type": "text", "text": "（预测的下一帧）"},
                    {"type": "text", "text": scoring_prompt},
                ]
            }
        ]
        
        response = self.client.chat.completions.create(...)
        result = json.loads(...)
        return result["score"]
```

---

## 第四章：常见问题 FAQ

### Q1：我没有 VLM API 怎么办？

使用本仓库已有的 llama.cpp 本地部署方式：
```bash
# 下载 Qwen3.5-27B GGUF 模型
llama-server --model path/to/Qwen3.5-27B-Q4_K_M.gguf \
    --mmproj path/to/mmproj.gguf \
    --alias Qwen3.5-27B-Q4_K_M \
    --host 0.0.0.0 --port 8000 \
    --ctx-size 8192 --n-gpu-layers 999
```

然后设置 `PlannerConfig` 的 `la_base_url="http://localhost:8000/v1"`。

### Q2：VLM 返回的 JSON 格式不正确怎么办？

当前代码中已经包含了清理和恢复逻辑（`_parse_vlm_response`）。如果仍遇到解析错误，可以：
1. 降低 `temperature` 参数（0.5~0.7），使输出更稳定
2. 在 prompt 中更严格地强调输出格式
3. 使用支持函数调用的 API 直接约束输出 schema

### Q3：规划器如何与打分器配合？

规划器的输出（候选轨迹列表）会传给世界模型，世界模型的预测结果会被打分器评分。打分器复用规划器的 VLM 来评分，调用 `score_trajectory` 方法。

### Q4：当前帧是单张还是多张？

目前规划器只接收单张前方 RGB 图像。如果后续需要全景信息，可以改为输入多张图像（前/左/右/后/下）拼接成一张全景图，或分别编码后一起发给 VLM。

### Q5：和现有的 ZeroShotVlnEvaluatorMP 有什么区别？

| 维度 | 现有 ZeroShotVlnEvaluatorMP | 我们新的 VLMPlanner |
|------|---------------------------|-------------------|
| 输出粒度 | 单条waypoint方向+距离 | 多条候选原子动作序列 |
| 模型调用 | LA做方向决策→VA做距离估算（两次调用） | 一次调用直接输出动作序列 |
| 是否提供候选 | 否，只输出一条 | 是，输出K条候选供打分 |
| 轨迹格式 | 连续坐标waypoint | 离散原子动作序列 |
| 与打分器关系 | 无 | 预留了 score_trajectory 接口 |

---

## 第五章：开发和测试流程建议

### 推荐开发顺序

1. **阶段零**：阅读 `unilavira_model.py` 和 `eval.py`，理解 VLM 调用方式和循环结构
2. **阶段一**：创建 `planner/` 目录和数据结构，验证能创建/打印轨迹
3. **阶段二**：实现 Delta 计算器，用已知轨迹手动验证计算结果
4. **阶段三**：实现 VLM 调用，先用单帧图像测试（截图放到代码目录），验证能返回候选轨迹
5. **阶段四**：集成到 AirSim，让无人机在仿真中飞行测试
6. **阶段五**：加入历史上下文，完善迭代能力

### 测试验证清单

每个阶段完成后，验证以下内容：

- [ ] 阶段一：能创建 `Trajectory` 对象、能设置 `PlannerConfig` 并正确读取参数
- [ ] 阶段二：`compute_delta(["forward", "forward"])` 返回 `[0.2, 0, 0, 0]`
- [ ] 阶段三：VLM 返回了格式正确的 `candidates` 列表，每个候选有 `actions` 和 `reason`
- [ ] 阶段四：无人机能在 AirSim 中执行连续的规划→动作→观测循环
- [ ] 阶段五：历史上下文正确传递，VLM 能基于已执行动作调整下一步

### 调试技巧

1. **打印 VLM 原始输出**：不要直接解析，先打印出来看原始格式
2. **用静态图片测试**：使用一张截图作为固定输入，调试 prompt 效果
3. **逐步增加难度**：先测试"直飞"（forward），再测试"转弯"，最后测试复杂指令
4. **对比方案**：将 VLM 的候选轨迹和专家轨迹（如果有的话）比较，评估合理性
