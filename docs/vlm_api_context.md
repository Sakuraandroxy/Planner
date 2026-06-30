# VLM API 调用上下文文档

> 本文档说明当前项目中每一次 VLM API 调用的完整上下文结构，包括每条消息的内容来源、缺失字段和已知问题。

---

## 目录

1. [调用 1 — 目标 bbox 检测](#1-调用-1--目标-bbox-检测)
2. [调用 2 — 轨迹规划](#2-调用-2--轨迹规划)
3. [调用 3 — 重定位（四向环视）](#3-调用-3--重定位四向环视)
4. [上下文缺失问题汇总](#上下文缺失问题汇总)
5. [上下文变更历史](#上下文变更历史)

---

## 1. 调用 1 — 目标 bbox 检测

### 1.1 基本信息

| 字段 | 值 |
|---|---|
| 触发函数 | `Planner.estimate_target_depth()` |
| 触发时机 | 首次检测目标；复用 bbox 失败后回退；每一步默认都调 |
| API 调用次数/步 | 1（失败时重试 1 次） |
| VLM 模型 | `config.model_name`（当前 step-3.7-flash） |

### 1.2 完整消息列表

```python
# 正常调用（不重试）
messages = [
    {"role": "system",  "content": SYSTEM_PROMPT},
    {"role": "user",    "content": [image_url_rgb, image_url_depth?, text_instruction]},
]

# 解析失败重试时，在原有消息后追加：
messages += [
    {"role": "assistant", "content": "<上一条模型返回的原始文本（截断 1200 字符）>"},
    {"role": "user",      "content": "上一条没有给出可解析JSON。只输出一行JSON，格式为：..."},
]
```

### 1.3 System prompt

```python
# 场景物体模式（scene_obstacle_planning_enabled=True，默认）
SCENE_BBOX_SYSTEM_PROMPT = (
    "你是无人机场景检测器。只输出一个合法JSON对象，不要解释、不要Markdown、不要思考过程。"
)

# 普通目标检测模式
TARGET_BBOX_SYSTEM_PROMPT = (
    "你是目标检测器。根据图片输出真实目标框JSON。"
    "只输出一行JSON对象；禁止思考过程、解释、Markdown、动作规划、占位符。"
)
```

**System prompt 里包含的信息**：
- 角色身份（目标检测器 / 场景检测器）
- 输出格式约束（一行 JSON）
- **不包含**历史对话、任务进度、深度统计、前帧 bbox

### 1.4 User content — 图片

| 图片 | 来源 | 说明 |
|---|---|---|
| RGB 当前帧 | `AirSimClient.get_scene_and_depth_meters()` 返回的 `frame` | 已 encode 为 base64 PNG |
| 深度图 | `build_scene_bbox_messages()` / `build_target_bbox_messages()` 的 `depth_frame` 参数 | **当前 `run_airsim_web.py` 传 `None`，实际不传** |

### 1.5 User content — 文本指令

```python
# 场景物体模式
SCENE_BBOX_PROMPT.format(
    task_description=task_description,
    image_width=image_size[0],
    image_height=image_size[1],
)

# 普通目标检测模式
TARGET_BBOX_PROMPT.format(
    task_description=task_description,
    image_width=image_size[0],
    image_height=image_size[1],
)
```

文本指令包含：
- 任务描述（`task_description`）
- 图像尺寸（宽×高像素）
- 输出字段约束（`visible / bbox_norm / confidence / target`）
- 特殊指令：如果有"较远/远处/far"，选视觉更远的目标；禁止输出 candidates 字段

### 1.6 本调用的特点与问题

| 特点 | 说明 |
|---|---|
| ✅ 无历史对话 | 每步独立检测，不利用上一帧 bbox 约束搜索 |
| ✅ prompt 极短 | system + 1 图 + 短文本，是本项目最快的 VLM 调用 |
| ❌ 无深度图 | `run_airsim_web.py` 传 `depth_frame=None`，VLM 只能靠视觉猜距离 |
| ❌ 无紧凑上下文 | 不传入 `ContextManager.build_compact_context()` |
| ❌ 无任务进度 | 不知道这是第几步、累计前进了多少米 |
| ⚠️ 默认每步都调 | 虽有 `_try_reuse_target_depth()` 复用逻辑，但默认步步调用 |

---

## 2. 调用 2 — 轨迹规划

### 2.1 基本信息

| 字段 | 值 |
|---|---|
| 触发函数 | `Planner.generate_candidates()` |
| 触发时机 | 目标检测通过后，不满足本地直行条件时 |
| API 调用次数/步 | 1 |
| VLM 模型 | `config.model_name`（当前 step-3.7-flash） |

### 2.2 完整消息列表

```python
messages = [
    {"role": "system", "content": PLANNER_SYSTEM_PROMPT},          # ①
    # ← ② compact_context（如果 context_enabled=True）
    {"role": "user",   "content": [image_rgb, image_depth?, text_instruction]},  # ③
    # ← ④ 历史执行记录（最近 N 步，每步 user+assistant 共 2 条）
]
```

#### ① System prompt

来源：`PromptBuilder.build_system_prompt()`

```python
PLANNER_SYSTEM_PROMPT.format(
    horizontal_step=config.horizontal_step,
    vertical_step=config.vertical_step,
    yaw_step_deg=config.yaw_step_deg,
    max_forward_step=config.max_forward_step,
    max_tracking_yaw_step_deg=config.max_tracking_yaw_step_deg,
    max_trajectory_length=config.max_trajectory_length,
    arrival_depth=config.context_arrival_depth,   # 默认 5.0 m
    task_description=task_description,
    k=k,                                          # 默认 3 条候选
    depth_info=depth_info_str,                    # 关键：深度统计文字
)
```

System prompt 末尾追加的固定限制：

```
补充限制：只输出JSON，不要Markdown；
轨迹必须使用深度提示中的目标/障碍物位置和深度；
任务是旁边/附近/接近时，可用 5.0m 到达半径判断完成；
任务是上方/上面/顶部/侧方/绕行时，必须到对应相对位置后才 done=true；
当前帧有目标bbox和深度时必须基于当前目标规划；
只有深度提示明确写着当前RGB未可靠发现目标时才只允许原地旋转搜索；
相邻动作必须不同类型，连续同类动作必须合并。
```

#### ② Compact context（`ContextManager.build_compact_context()`）

```python
"上下文记忆：\n"
"- 任务进度：instruction=..., state=..., accumulated_forward=...\n"
"- 目标记忆：class=..., bbox=..., depth=..., bearing=..., world_yaw=..., ...\n"
"- 局部障碍：front_safe_depth=..., target_corridor_safe_depth=..., blocked=...\n"
"- 搜索记忆：inactive / active, scan_steps=..., best_seen=...\n"
"要求：不要因为单帧误检轻易切换目标；目标短暂丢失时优先回到历史目标方向；"
"只有前方/目标方向深度通道安全时才允许连续前进。"
```

compact context 包含：

| 类别 | 字段 | 来源 |
|---|---|---|
| 任务进度 | `instruction / state / accumulated_forward_distance` | `TaskMemory` |
| 目标记忆 | `class / bbox / depth / bearing / world_yaw / world_pos / visible_streak / lost_streak` | `TargetMemory` |
| 局部障碍 | `front_safe_depth / target_corridor_safe_depth / blocked` | `ObstacleMemory` |
| 搜索记忆 | `active / scan_steps / best_seen_target` | `SearchMemory` |

#### ③ User content — 图片

| 图片 | 来源 | 说明 |
|---|---|---|
| RGB 当前帧 | AirSim `get_scene_and_depth_meters()` 返回的 `frame` | base64 PNG |
| 深度图 | `depth_frame` 参数 | **当前 `run_airsim_web.py` 传 `None`，实际不传** |

#### ④ 历史执行记录（`ContextManager.get_messages()`）

```python
# 最近 N 步的执行摘要（每步 2 条消息）
[
    {"role": "user",    "content": "[Step 1] executed=['forward 40']; pose=(x,y,z); yaw=xxx; ..."},
    {"role": "assistant","content": "已记录执行结果，下一步继续遵循目标记忆和安全深度约束。"},
    {"role": "user",    "content": "[Step 2] executed=['forward 5', 'right 10']; ..."},
    {"role": "assistant","content": "已记录执行结果，..."},
    # ... 最多 max_steps * 2 条（默认 5 步 x 2 = 10 条）
]
```

历史摘要里包含的信息：
- 执行的动作列表
- 无人机位姿变化（起点 → 终点）
- 偏航角
- 是否碰撞
- 目标状态、深度、世界坐标
- 丢失次数、累计前进距离

### 2.3 深度信息文字（`_format_depth_info()`）

`depth_info_str` 在 system prompt 和 user instruction 里都会出现：

```python
# 目标可见时
"深度提示：程序已从目标bbox内原始深度矩阵计算：目标中位深度=44.47m，"
"目标中心中位深度=41.2m，目标最近深度=38.1m；规划必须优先使用这些目标深度数值。"
"画面中心最近深度=5.4m，全画面最近深度=3.9m。"
"场景物体与障碍物深度列表：obj1:target:car:... depth=44.5；obj2:obstacle:tree:... depth=5.3；..."

# 目标不可见时
"深度提示：目标检测：当前RGB图中未可靠发现目标。"
"画面中心最近深度=11.3m，全画面最近深度=4.0m。"

# 无深度数据时
"深度提示：未获取到有效目标深度统计，请保守行动并优先小步观察。"
```

### 2.4 本调用的特点与问题

| 特点 | 说明 |
|---|---|
| ✅ compact context 结构化 | 目标/任务/障碍/搜索四类记忆以文字块注入 |
| ✅ 有历史执行摘要 | VLM 知道前面几步做了什么 |
| ✅ 深度统计文字详细 | 目标深度、中心深度、最近深度、场景物体列表都在 |
| ❌ 深度图实际不传 | `depth_frame=None`，VLM 看不到可视化深度图 |
| ❌ 无真正 KV-cache | 每次完整 messages 重发，ViT 视觉编码每次都重算 |
| ⚠️ 历史摘要每步追加 | `add_step()` 每步加 2 条消息，`max_steps` 默认 5，超过截断 |

---

## 3. 调用 3 — 重定位（四向环视）

### 3.1 基本信息

| 字段 | 值 |
|---|---|
| 触发函数 | `Relocalizer.run()` |
| 触发时机 | `Planner` 判断 `target_missing=True` 时 |
| API 调用次数/次 | 1（每次重定位） |
| VLM 模型 | `config.model_name`（共享同一个 `planner.vlm` 实例） |

### 3.2 完整消息列表

```python
messages = [
    {"role": "system", "content": RELOCALIZER_SYSTEM_PROMPT},
    {"role": "user",   "content": [
        text_task_and_view_list,         # 纯文本
        "view_index=0, yaw=0.0度",      # 文本标注
        image_url_view0,                  # 图片
        "view_index=1, yaw=90.0度",     # 文本标注
        image_url_view1,                  # 图片
        "view_index=2, yaw=180.0度",    # 文本标注
        image_url_view2,                  # 图片
        "view_index=3, yaw=270.0度",    # 文本标注
        image_url_view3,                  # 图片
    ]},
]
```

### 3.3 System prompt

```python
RELOCALIZER_SYSTEM_PROMPT = """\
你是无人机目标重定位候选检测器。
你只负责在多张环视RGB图中找出"可能是任务目标"的可疑目标框。
不要做路径规划，不要输出动作，不要选择最终方向，只输出合法JSON。\
"""
```

### 3.4 User content — 图片

| 图片 | 来源 | 说明 |
|---|---|---|
| 4 张环视 RGB | `Relocalizer._capture_views()` 每转 90° 拍一张 | base64 PNG，带偏航角标注 |

共 4 张图，覆盖 360°（默认 `relocalizer_view_count=4`，`yaw_step=90°`）。

### 3.5 User content — 文本

```python
f"""任务目标：{task_description}

无人机会原地环视多个方向。图片按顺序给出：
{view_descriptions}

请对每张图分别输出所有"可能是任务目标"的可疑目标，最多每张图3个。
如果某张图没有可疑任务目标，该图 targets 为空数组。
只输出JSON，格式如下：
{{
  "views": [
    {{
      "view_index": 0,
      "targets": [
        {{"name": "car", "bbox_norm": [0.1, 0.2, 0.3, 0.4], "confidence": 0.85, "reason": "可能是任务目标"}}
      ]
    }}
  ]
}}

要求：
1. view_index 必须对应输入图片编号。
2. bbox_norm 必须是[left, top, right, bottom]，每个值在0到1之间。
3. 只输出和任务目标语义相关的可疑目标；障碍物、道路、树等不要放进 targets。
4. 不要输出 forward/left/right/up/down。
5. 不要解释，不要Markdown。"""
```

### 3.6 本调用的特点与问题

| 特点 | 说明 |
|---|---|
| ✅ 4 张环视图保证 360° 覆盖 | 比单帧旋转搜索更鲁棒 |
| ✅ VLM 只负责候选框标注 | 不做规划，职责清晰 |
| ✅ 程序后续算深度/世界坐标 | `_target_to_world_candidate()` 自动算每个候选的 `world_pos` 和 `distance_to_lock` |
| ❌ 无历史对话 | 重定位是独立调用，不传入 `build_compact_context()` |
| ❌ 无深度图 | VLM 只输出 bbox，深度由程序从 depth_meters 矩阵计算 |
| ❌ 无紧凑上下文 | 不传入任务进度、目标记忆、搜索记忆 |
| ⚠️ `locked_world_pos` 不传给 VLM | 只用于程序侧候选排序，VLM 不知道目标历史位置 |
| ⚠️ 置信度阈值固定 | `relocalizer_confidence_threshold=0.5`，对某些场景可能偏高或偏低 |

---

## 4. 上下文缺失问题汇总

| 缺失项 | 影响调用 | 具体影响 |
|---|---|---|
| **bbox API 无历史对话** | 调用 1 | 每一步独立检测，无法利用上一帧 bbox 约束搜索区域，增加误检概率 |
| **深度图实际不传** | 调用 1、调用 2 | VLM 只能靠文字描述猜深度，看不到可视化深度通道，无法判断前方是否有障碍 |
| **无 KV-cache 共享** | 调用 1、2、3 | 每次完整重发 messages，ViT 视觉编码每次都重算，增加每步总耗时 |
| **重定位无紧凑上下文** | 调用 3 | VLM 不知道目标类别、历史位置、丢失次数，只能靠图片猜测 |
| **重定位无任务进度** | 调用 3 | VLM 不知道当前是第几步、累计前进了多远、目标大概在哪 |
| **紧凑上下文固定插入** | 调用 2 | 不管当前状态是否需要，每次都插入完整目标/任务/障碍/搜索记忆块，浪费 token |
| **无碰撞/滑跑结构化记录** | 调用 2 | 历史摘要里只有文字，没有结构化的碰撞位姿、滑跑偏移量 |
| **无环境变量配置** | 全部 | `context_enabled` / `recovery_enabled` 等开关没有统一的环境变量入口 |

---

## 5. 上下文变更历史

| 日期 | 文件 | 变更 |
|---|---|---|
| 2026-06-22 | `agent/planner.py` | 初始化 Planner，第一次实现目标框 API + 规划 API 两阶段调用 |
| 2026-06-23 | `agent/context_manager.py` | 引入 `TargetMemory / TaskMemory / SearchMemory / ObstacleMemory`，`build_compact_context()` 开始注入 planning VLM |
| 2026-06-24 | `agent/target_depth.py` | `TARGET_BBOX_PROMPT` 改写为只输出一行 JSON；禁止输出 candidates 字段 |
| 2026-06-24 | `agent/relocalizer.py` | 重写为独立模块，四向环视 + VLM 候选标注 + 程序算深度/世界坐标 |
| 2026-06-27 | `agent/target_identity.py` | 新增 `TargetIdentityManager`，锁定首个目标实例，按世界坐标容忍度判断是否接受 |
| 2026-06-29 | `agent/context_manager.py` | `ContextManager.evaluate_observation()` 状态机；`NEAR_GOAL_CONFIRM / LOST_OR_OCCLUDED / RELOCALIZE` 等状态 |
| 2026-06-29 | `planner/prompt_templates.py` | `PLANNER_SYSTEM_PROMPT` 加入到达半径规则、相邻动作合并规则；`build_system_prompt()` 末尾追加 5 条限制 |
| 2026-06-29 | `run_airsim_web.py` | `capture_scene_depth()` 合并 RGB + DepthPerspective RPC；加入 `frame_cache` 缓存；`warmup_vlm_api()` 预热 |