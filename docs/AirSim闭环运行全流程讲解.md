# AirSim 闭环运行全流程讲解

这份文档是给“马上要介绍项目，但还没来得及读代码”的人准备的。目标不是讲论文，而是讲清楚这个项目**怎么从前端输入一句任务，最后真的让 AirSim 里的无人机执行动作**。

如果你只记一句话，可以记这个：

> 这个系统本质上是一个“看一眼画面 -> 让 VLM 出几种走法 -> 程序挑中一条 -> 转成 AirSim 控制命令 -> 飞一下 -> 再看下一帧”的闭环。

---

## 1. 先说结论：系统里真正发生了什么

用户在网页里输入一句任务，比如：

```text
飞到较远汽车旁
```

系统随后做 5 件事：

1. 前端把这句任务发给 Flask 后端。
2. 后端主循环从 AirSim 取一帧 RGB、深度图和当前位姿。
3. 后端调用 VLM 先看目标在哪，再让 VLM 规划多条候选轨迹。
4. 程序把 VLM 返回的动作 JSON 转成 AirSim 的移动和转向 API。
5. 无人机执行后，系统重新观察下一帧，再重复这套流程。

这就是“闭环”：**不是一次规划飞到底，而是每飞一步就重新看、重新想、重新飞。**

---

## 2. 这套系统的关键文件

你介绍项目时，只抓下面这些文件就够了：

| 文件 | 作用 |
|---|---|
| `run_airsim_web.py` | 总入口。负责启动后端、连接 AirSim、跑闭环主循环 |
| `web/app.py` | Flask Web 服务，提供前端页面和接口 |
| `web/frontend/index.html` | 前端页面 |
| `web/frontend/app.js` | 前端逻辑，发任务、收状态、刷新画面 |
| `web/shared_state.py` | 前后端共享状态中心 |
| `agent/planner.py` | 规划总控：组织 VLM、提示词、结果解析 |
| `agent/vlm_client.py` | 调 OpenAI 兼容接口 |
| `agent/prompt_builder.py` | 把图像、任务、深度提示打包成 VLM 输入 |
| `agent/response_parser.py` | 解析 VLM 返回的 JSON |
| `agent/target_depth.py` | 先找目标框，再从深度矩阵算目标距离 |
| `sim/airsim_client.py` | 对 AirSim API 做了一层封装 |
| `sim/action_executor.py` | 把动作列表变成 AirSim 可执行的位移和转向 |
| `sim/frame_capturer.py` | 后台持续抓图给前端看 |
| `planner/config.py` | 控制动作步长、候选数、速度等参数 |
| `planner/trajectory.py` | 解析动作字符串、计算位移变化 |

---

## 3. 一张图看完整链路

```mermaid
flowchart LR
    A[前端输入任务] --> B[POST /task]
    B --> C[SharedState.task]
    C --> D[run_airsim_web.py main_loop]
    D --> E[AirSim 获取 RGB 深度 位姿]
    E --> F[Planner.generate_candidates]
    F --> G[目标检测 VLM 调用]
    G --> H[从深度矩阵计算目标距离]
    H --> I[规划 VLM 调用]
    I --> J[解析 JSON 候选动作]
    J --> K[动作转航点]
    K --> L[AirSim 执行动作]
    L --> M[记录结果和状态]
    M --> D
    M --> N[前端通过 /events /frame /depth_frame 实时展示]
```

---

## 4. 从启动开始：程序是怎么跑起来的

入口文件是 `run_airsim_web.py`。

### 4.1 `main()`

这个函数做三件事：

1. 从 `.env` 和环境变量中读取配置。
2. 创建：
   - `Planner(cfg)`：规划器
   - `SharedState()`：共享状态
3. 启动两个并行部分：
   - 一个后台线程跑 `main_loop(...)`
   - 主线程启动 Flask Web：`create_app(state)` 然后 `app.run(...)`

所以系统实际上是：

- **Flask 主线程**：负责网页和接口
- **闭环后台线程**：负责连接 AirSim、抓图、调 VLM、执行动作

这是整个系统的第一层结构。

---

## 5. 前端是怎么把任务发进来的

### 5.1 页面结构：`web/frontend/index.html`

页面上最重要的控件只有一个输入框和一个按钮：

- 输入框：`<input id="taskInput">`
- 按钮：`<button id="taskBtn" onclick="updateTask()">`

页面右侧还会展示：

- 当前场景分析
- 深度图
- VLM 推理摘要
- 候选轨迹

### 5.2 发任务：`web/frontend/app.js -> updateTask()`

当前端点击 Update 时，会执行：

```javascript
fetch("/task", {
  method: "POST",
  headers: {"Content-Type":"application/json"},
  body: JSON.stringify({task: v})
})
```

也就是说，前端发给后端的任务格式是：

```json
{
  "task": "飞到较远汽车旁"
}
```

### 5.3 后端接收：`web/app.py -> update_task()`

Flask 路由：

```python
@app.route("/task", methods=["POST"])
def update_task():
    data = request.get_json(force=True)
    if data and "task" in data:
        state.update(task=data["task"])
        return {"status": "ok", "task": data["task"]}
```

后端做的事非常简单：

1. 收到 JSON
2. 取出 `task`
3. 写进 `SharedState.task`
4. 返回确认

返回给前端的数据格式是：

```json
{
  "status": "ok",
  "task": "飞到较远汽车旁"
}
```

### 5.4 任务进入闭环线程：`run_airsim_web.py -> main_loop()`

后台主循环里一直在看这一句：

```python
cur_task = state.task or initial_task
```

一旦 `state.task` 不为空，闭环就开始工作。

---

## 6. 前端怎么实时看到系统状态

前端不是不停轮询很多 JSON，而是用了 **SSE（Server-Sent Events）**。

### 6.1 前端订阅：`web/frontend/app.js`

```javascript
var es = new EventSource("/events");
es.onmessage = function(e) {
    var s = JSON.parse(e.data);
    state = s;
    updateUI(s);
}
```

### 6.2 后端推送：`web/app.py -> /events`

```python
@app.route("/events")
def events():
    def gen():
        last_ver = -1
        while True:
            st = state.get_state()
            if st["version"] != last_ver:
                last_ver = st["version"]
                yield f"data: {json.dumps(st)}\\n\\n"
            time.sleep(0.15)
```

只要 `SharedState.version` 变化，后端就推一次最新状态。

### 6.3 状态 JSON 长什么样

`web/shared_state.py -> get_state()` 返回的数据结构如下：

```json
{
  "version": 12,
  "frame_version": 5,
  "depth_version": 2,
  "depth_bytes": 18342,
  "status": "thinking",
  "task": "飞到较远汽车旁",
  "action_mode": "atomic",
  "step": 3,
  "max_steps": 20,
  "pose": [12.3, -4.8, -2.1],
  "yaw": 15.0,
  "collided": false,
  "reasoning": "...",
  "scene_analysis": "前方远处有汽车，左侧有建筑遮挡",
  "candidates": [
    {
      "actions": ["forward", "forward", "right"],
      "reason": "先接近，再微调朝向",
      "delta": {"dx": 10.0, "dy": 0.0, "dz": 0.0, "dphi": 15.0}
    }
  ],
  "selected": ["forward", "forward", "right"],
  "reasoning_summary": "...",
  "task_done": false,
  "model_name": "gpt-4o",
  "error": ""
}
```

前端拿到这个状态后：

- 用 `/frame?t=frame_version` 刷 RGB 图
- 用 `/depth_frame?t=depth_version` 刷深度图
- 用 `scene_analysis`、`reasoning_summary` 和 `candidates` 刷右侧面板

---

## 7. 真的开始闭环时，第一轮发生了什么

下面开始讲最关键的一段：**一轮闭环到底怎么跑。**

函数入口是：

```python
run_airsim_web.py -> main_loop(state, planner, initial_task, max_steps, cfg)
```

### 7.1 连接 AirSim

一开始会做这些动作：

1. `client = AirSimClient()`
2. `client.connect()`
3. `client.enable_api_control(True)`
4. `client.arm(True)`
5. 如果当前还没飞起来：`client.takeoff()`

这些实际调用的底层 AirSim API 在 `sim/airsim_client.py`：

| 封装函数 | 底层 AirSim API | 作用 |
|---|---|---|
| `connect()` | `confirmConnection()` | 连接仿真器 |
| `enable_api_control(True)` | `enableApiControl(True)` | 把控制权交给程序 |
| `arm(True)` | `armDisarm(True)` | 解锁无人机 |
| `takeoff()` | `takeoffAsync().join()` | 起飞 |

### 7.2 启动后台画面采集线程

`main_loop()` 会创建：

```python
capturer = FrameCapturer(...)
capturer.start(state)
```

这个线程在 `sim/frame_capturer.py -> _loop()` 里持续做两件事：

1. 低延迟抓 RGB 图，更新给前端
2. 低频抓深度图预览，更新给前端

注意一个容易讲错的点：

> 前端看到的图，不完全等于“规划时用的那一张图”。

因为系统有两路更新：

- `FrameCapturer` 持续给前端刷图
- `main_loop()` 在真正规划前，也会主动再抓一张最新 RGB + 深度，并覆盖到 `SharedState`

所以你可以这样讲：

> 前端是实时预览，真正给 VLM 规划的是主循环在关键时刻主动抓取的最新一帧。

---

## 8. 一轮规划前，系统从 AirSim 拿到了什么

在每一轮循环里，主函数会先执行：

```python
frame, depth_meters = client.get_scene_and_depth_meters()
depth_frame_display = client.depth_meters_to_image(depth_meters)
depth_data = client.depth_meters_to_stats(depth_meters)
pos, yaw = client.get_pose()
collided = client.check_collision()
```

### 8.1 `get_scene_and_depth_meters()`

这个函数在 `sim/airsim_client.py` 中，底层使用一次 AirSim RPC 请求两种图：

```python
airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True)
airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)
```

返回的数据是：

1. `frame`
   - 类型：`PIL.Image`
   - 内容：无人机第一视角 RGB 图
2. `depth_meters`
   - 类型：`numpy.ndarray`
   - 形状：通常是 `[H, W]`
   - 数值：每个像素对应的真实深度，单位米

### 8.2 `depth_meters_to_image(depth_meters)`

这一步不是给 VLM 的核心输入，而是为了前端展示。

它会把真实深度矩阵裁到 0~100 米，再映射成 8-bit 灰度图，变成可以展示的 PNG。

### 8.3 `depth_meters_to_stats(depth_meters)`

这一步会算几个关键数字：

```json
{
  "scene_min": 2.1,
  "scene_max": 86.7,
  "center_min": 18.3,
  "center_avg": 25.9
}
```

这些数字后面会被写成中文提示词塞给 VLM，比如：

```text
深度提示：画面中心最近深度=18.3m，画面中心平均深度=25.9m，全画面最近深度=2.1m...
```

### 8.4 `get_pose()`

`sim/airsim_client.py -> get_pose()` 会：

1. 调 `simGetVehiclePose()`
2. 从四元数里解出偏航角 `yaw`
3. 返回：

```json
{
  "pose": [x, y, z],
  "yaw": 15.0
}
```

其中坐标约定是：

- `x, y`：平面位置
- `z`：高度方向，**更负表示更高**

这就是为什么代码里：

- `up` -> `z -= val`
- `down` -> `z += val`

---

## 9. 真正的规划不是一次 VLM 调用，而是两次

这是介绍时非常值得讲的一点。

`agent/planner.py -> Planner.generate_candidates(...)` 里，真实流程是：

1. **先做目标定位和目标深度估计**
2. **再做导航动作规划**

所以是两个模型请求，不是一个。

---

## 10. 第一次 VLM 调用：先找目标在哪

函数链路：

```text
Planner.generate_candidates
  -> Planner.estimate_target_depth
  -> PromptBuilder.encode_image
  -> build_target_bbox_messages
  -> VLMClient.call
  -> parse_target_candidates
  -> estimate_depth_in_bbox
  -> choose_target_estimate
```

### 10.1 为什么先找目标框

如果任务是“飞到较远汽车旁”，系统不能只看整张深度图，否则不知道哪部分深度对应“汽车”。

所以它先让 VLM 做一件更简单的事：

> 只看 RGB，告诉我“汽车”大概在哪个框里。

### 10.2 发给 VLM 的输入格式

`agent/target_depth.py -> build_target_bbox_messages(...)` 构造的 `messages` 结构是：

```json
[
  {
    "role": "system",
    "content": "你是目标检测器，只输出 JSON。"
  },
  {
    "role": "user",
    "content": [
      {
        "type": "image_url",
        "image_url": {
          "url": "data:image/png;base64,..."
        }
      },
      {
        "type": "text",
        "text": "任务目标：飞到较远汽车旁；请输出 visible、bbox_norm、confidence、target、candidates ..."
      }
    ]
  }
]
```

这里的图像不是文件路径，而是：

- 先把 `PIL.Image` 编码成 PNG
- 再转成 base64
- 再拼成 `data:image/png;base64,...`

这个逻辑在 `agent/prompt_builder.py -> encode_image()`。

### 10.3 期望 VLM 返回什么

典型返回格式：

```json
{
  "visible": true,
  "bbox_norm": [0.62, 0.41, 0.77, 0.58],
  "confidence": 0.88,
  "target": "car",
  "candidates": [
    {
      "bbox_norm": [0.62, 0.41, 0.77, 0.58],
      "confidence": 0.88,
      "target": "car"
    }
  ]
}
```

其中：

- `visible`：目标是否可见
- `bbox_norm`：归一化框 `[x1, y1, x2, y2]`，范围 0~1
- `confidence`：置信度
- `target`：识别到的目标名

### 10.4 程序怎么把“框”变成“米”

接下来是这个项目最有价值的一步：

`agent/target_depth.py -> estimate_depth_in_bbox(depth_meters, estimate, ...)`

它会：

1. 把 RGB 图里的 bbox 映射到深度矩阵分辨率
2. 取出 bbox 区域的所有有效深度像素
3. 计算：
   - `depth_median`
   - `depth_min`
   - `depth_mean`
   - `depth_center_median`

最后输出的数据长这样：

```json
{
  "target_visible": true,
  "target_bbox": [1190, 442, 1478, 626],
  "target_depth_bbox": [397, 147, 493, 209],
  "target_confidence": 0.88,
  "target_name": "car",
  "target_depth_median": 26.4,
  "target_depth_min": 23.8,
  "target_depth_mean": 28.1,
  "target_depth_center_median": 25.7,
  "target_depth_valid_pixels": 412
}
```

这一步的含义很重要：

> VLM 负责“看见目标在哪”，真正的距离数字由程序从 AirSim 深度矩阵里算出来。

也就是说，**不是让大模型凭感觉猜距离**。

---

## 11. 第二次 VLM 调用：让 VLM 规划动作

函数链路：

```text
Planner.generate_candidates
  -> PromptBuilder.build_messages
  -> VLMClient.call
  -> ResponseParser.parse
```

### 11.1 这次喂给 VLM 什么

`PromptBuilder.build_messages(...)` 会把以下内容打包进去：

1. 当前 RGB 图
2. 任务描述
3. 场景深度统计
4. 目标 bbox 深度统计
5. 历史动作上下文（如果有）

最终 `messages` 长这样：

```json
[
  {
    "role": "system",
    "content": "你是无人机导航规划器，输出合法 JSON；给出 k 条候选轨迹..."
  },
  {
    "role": "user",
    "content": [
      {
        "type": "image_url",
        "image_url": {
          "url": "data:image/png;base64,..."
        }
      },
      {
        "type": "text",
        "text": "任务：飞到较远汽车旁。深度提示：目标中位深度=26.4m，目标最近深度=23.8m，画面中心平均深度=25.9m..."
      }
    ]
  }
]
```

### 11.2 真正调用的接口

`agent/vlm_client.py -> VLMClient.call()` 实际调用的是 OpenAI 兼容接口：

```python
response = self.client.chat.completions.create(
    model=self.model_name,
    messages=messages,
    max_tokens=max_tokens,
    temperature=self.temperature,
)
```

也就是说，只要你的模型服务兼容 OpenAI Chat Completions 协议，这里都能接。

相关配置来自 `planner/config.py` 和环境变量：

- `PLANNER_BASE_URL`
- `PLANNER_API_KEY`
- `PLANNER_MODEL_NAME`
- `PLANNER_TEMPERATURE`

### 11.3 VLM 期望返回的动作 JSON

项目当前要求的核心格式是：

```json
{
  "selected_index": 1,
  "done": false,
  "scene_analysis": "前方远处有一辆汽车，前进方向基本正确，右侧更开阔。",
  "reasoning_summary": "目标在前方偏右，距离较远，适合先连续前进再小角度修正。",
  "candidates": [
    {
      "actions": ["forward", "forward", "left"],
      "reason": "先接近后微调",
      "scale": 1.0
    },
    {
      "actions": ["forward", "forward", "right"],
      "reason": "右侧路径更开阔",
      "scale": 1.0
    },
    {
      "actions": ["right", "forward", "forward"],
      "reason": "先对准再前进",
      "scale": 1.0
    }
  ]
}
```

当前默认 `action_mode=atomic`，所以动作一般像：

- `forward`
- `left`
- `right`
- `up`
- `down`

如果切换成 `value` 模式，也可能返回：

- `forward 4.8`
- `right 15`

---

## 12. VLM 返回后，程序怎么“收口”为结构化动作

函数在：

```text
agent/response_parser.py -> ResponseParser.parse()
```

### 12.1 先做 JSON 容错

因为大模型有时会输出：

- 带 ```json 代码块
- 少一个括号
- 单引号

所以解析器会先：

1. 去掉 Markdown 包裹
2. 自动补缺失括号
3. 尝试修复常见 JSON 格式错误

### 12.2 再做动作安全裁剪

解析出来后，`_limit_actions()` 会做二次约束。

例如：

- 如果模型输出 `forward 100`
  - 会裁成不超过 `config.max_forward_step`
- 如果目标可见，模型输出 `right 45`
  - 会裁成不超过 `config.max_tracking_yaw_step_deg`

这一步非常适合介绍时强调：

> 系统不是盲信大模型，而是把模型当“建议器”，最后由程序做安全兜底。

### 12.3 转成 `Trajectory`

每个候选动作会变成一个 `Trajectory` 对象，内部包含：

- `actions`
- `clean_delta`
- `noisy_delta`
- `scale`

其中 `clean_delta` 由 `planner/trajectory.py -> compute_delta()` 计算，表示这串动作理论上会带来多少位移和偏航变化。

比如在默认原子模式下，如果：

- `horizontal_step = 5m`
- `yaw_step_deg = 15`

那么：

```json
["forward", "forward", "right"]
```

理论位移大致会被算成：

```json
{
  "dx": 10.0,
  "dy": 0.0,
  "dz": 0.0,
  "dphi": 15.0
}
```

这个结果会被送到前端显示为候选轨迹信息。

---

## 13. 主循环怎么决定执行哪条轨迹

在 `run_airsim_web.py -> main_loop()` 里，核心逻辑是：

1. 从 `Planner.generate_candidates()` 得到：
   - `trajectories`
   - `scene_analysis`
   - `reasoning_summary`
   - `task_done`
   - `selected_idx`
2. 把候选列表写进 `SharedState`
3. 选择：

```python
best_idx = selected_idx if 0 <= selected_idx < len(trajectories) else 0
best = trajectories[best_idx]
actions = best.actions
```

也就是说：

- 如果 VLM 明确给了 `selected_index`，优先用它
- 否则默认取第 1 条候选

---

## 14. 动作列表是怎么变成 AirSim 指令的

这是“从智能到执行”的最后一步。

函数链路：

```text
main_loop
  -> actions_to_waypoints(actions, pos, yaw, cfg, scale)
  -> ActionExecutor.execute(actions, scale)
  -> AirSimClient.rotate_to_yaw / move_to_position
```

### 14.1 第一步：先算理论航点

`sim/action_executor.py -> actions_to_waypoints(...)`

输入：

- `actions`：例如 `["forward", "forward", "right"]`
- `start_pos`：例如 `[12.0, -4.0, -2.0]`
- `start_yaw_deg`：例如 `0`

它会按动作逐个推演出绝对坐标航点。

举例：

#### 初始状态

```json
{
  "pos": [12.0, -4.0, -2.0],
  "yaw": 0
}
```

#### 动作 1：`forward`

默认前进步长 5m：

```json
[17.0, -4.0, -2.0]
```

#### 动作 2：`forward`

```json
[22.0, -4.0, -2.0]
```

#### 动作 3：`right`

右转只改朝向，不立刻改位置，所以航点还是当前位置：

```json
[22.0, -4.0, -2.0]
```

最终这串动作会展开成一个航点列表，例如：

```json
[
  [17.0, -4.0, -2.0],
  [22.0, -4.0, -2.0],
  [22.0, -4.0, -2.0]
]
```

### 14.2 第二步：合并相邻同类动作

`group_consecutive_actions(actions)` 会把：

```json
["forward", "forward", "right"]
```

变成：

```json
[
  ["forward", 2],
  ["right", 1]
]
```

这样做的好处是：

- 两次前进可以合并成一次更长的移动
- 不用一小步一小步发很多 RPC

### 14.3 第三步：真正调用 AirSim

`ActionExecutor.execute()` 里分两类：

#### A. 转向动作

如果是 `left/right`，就调：

```python
self.client.rotate_to_yaw(yaw_cur)
```

底层是：

```python
rotateToYawAsync(yaw_deg, timeout_sec=timeout).join()
```

#### B. 位移动作

如果是 `forward/backward/up/down`，就调：

```python
self.client.move_to_position(
    target_wp[0], target_wp[1], target_wp[2],
    velocity=self.config.velocity,
    timeout=timeout
)
```

底层是：

```python
moveToPositionAsync(x, y, z, velocity, timeout_sec=timeout).join()
```

所以你可以很直接地描述这一步：

> VLM 只负责给动作建议；真正让 AirSim 飞起来的是 `moveToPositionAsync` 和 `rotateToYawAsync`。

---

## 15. 执行完之后，系统怎么进入下一轮

动作执行完后，`main_loop()` 会继续做几件事：

### 15.1 更新最新位姿和碰撞状态

```python
state.update(pose=pos_final, yaw=yaw_final, collided=col)
```

### 15.2 把这一步记进上下文

```python
planner.record_step(step, actions, pos_before, pos_final, yaw_final, col)
```

底层是 `agent/context_manager.py`，它会把：

- 这一步执行了什么动作
- 从哪飞到哪
- 是否撞了

整理成历史消息，在下一轮规划时插回 `messages`，让 VLM 知道“我们刚刚干了什么”。

这就是闭环里“记忆”的部分。

### 15.3 碰撞恢复

如果撞了，执行：

```python
executor.back_up(pos_final, yaw_final)
```

恢复策略是：

1. 先升到更安全的高度
2. 再往后退一段

这是一个简单但很实用的容错逻辑。

### 15.4 进入下一帧

休眠一个很短的采样间隔后，循环继续：

```python
time.sleep(cfg.capture_interval)
```

然后再次：

1. 抓 RGB
2. 抓深度
3. 看位姿
4. 调 VLM
5. 执行动作

这就形成真正的闭环。

---

## 16. 用一个完整例子把全流程串起来

下面这个例子不是拍脑袋编的流程，而是**完全按照代码里的真实数据结构和函数调用方式写出来的一个可能发生的实例**。

---

### 16.1 用户在网页输入任务

前端输入：

```text
飞到较远汽车旁
```

前端请求：

```http
POST /task
Content-Type: application/json
```

请求体：

```json
{
  "task": "飞到较远汽车旁"
}
```

后端返回：

```json
{
  "status": "ok",
  "task": "飞到较远汽车旁"
}
```

---

### 16.2 后端主循环读到任务，开始第 1 步

当前状态假设为：

```json
{
  "pose": [12.0, -4.0, -2.0],
  "yaw": 0.0
}
```

调用：

```python
frame, depth_meters = client.get_scene_and_depth_meters()
depth_data = client.depth_meters_to_stats(depth_meters)
```

得到：

```json
{
  "scene_min": 2.1,
  "scene_max": 86.7,
  "center_min": 18.3,
  "center_avg": 25.9
}
```

---

### 16.3 第一次 VLM：先找“汽车”在哪

程序发图和文本给目标检测 VLM，期望返回：

```json
{
  "visible": true,
  "bbox_norm": [0.62, 0.41, 0.77, 0.58],
  "confidence": 0.88,
  "target": "car",
  "candidates": [
    {
      "bbox_norm": [0.62, 0.41, 0.77, 0.58],
      "confidence": 0.88,
      "target": "car"
    }
  ]
}
```

程序再从深度矩阵里算目标距离，得到：

```json
{
  "target_visible": true,
  "target_name": "car",
  "target_depth_median": 26.4,
  "target_depth_min": 23.8,
  "target_depth_mean": 28.1,
  "target_depth_center_median": 25.7
}
```

---

### 16.4 第二次 VLM：规划候选轨迹

程序把“RGB + 任务 + 深度提示”发给规划 VLM。

模型返回：

```json
{
  "selected_index": 1,
  "done": false,
  "scene_analysis": "汽车位于前方偏右的远处，右侧路径更开阔。",
  "reasoning_summary": "目标可见且距离较远，适合先连续前进再小角度右转修正。",
  "candidates": [
    {
      "actions": ["forward", "forward", "left"],
      "reason": "先接近后微调",
      "scale": 1.0
    },
    {
      "actions": ["forward", "forward", "right"],
      "reason": "右侧更开阔，接近后更容易贴近汽车",
      "scale": 1.0
    },
    {
      "actions": ["right", "forward", "forward"],
      "reason": "先对准再前进",
      "scale": 1.0
    }
  ]
}
```

因为 `selected_index = 1`，系统选择第二条：

```json
["forward", "forward", "right"]
```

---

### 16.5 程序把动作变成 AirSim 可执行命令

先算理论航点：

```json
[
  [17.0, -4.0, -2.0],
  [22.0, -4.0, -2.0],
  [22.0, -4.0, -2.0]
]
```

再合并动作：

```json
[
  ["forward", 2],
  ["right", 1]
]
```

然后执行：

1. `moveToPositionAsync(22.0, -4.0, -2.0, velocity=0.5, timeout_sec=...)`
2. `rotateToYawAsync(15.0, timeout_sec=...)`

动作完成后，假设新位姿是：

```json
{
  "pose": [21.8, -4.1, -2.0],
  "yaw": 14.7,
  "collided": false
}
```

---

### 16.6 进入第 2 步闭环

现在系统不会说“我已经规划完了”，而是马上重新看下一帧：

1. 再抓一帧 RGB
2. 再算一次深度
3. 再问一次 VLM
4. 再执行下一段动作

直到：

- `done=true`
- 或者达到 `MAX_STEPS`

这就是“闭环运行”最核心的含义。

---

## 17. 你介绍项目时最值得强调的 8 个点

### 17.1 这是闭环，不是一次性规划

它不是“给一句命令，直接飞完整条路径”，而是每一步都重新观察。

### 17.2 前端只是控制台，不是决策核心

真正的智能逻辑都在后端线程里。

### 17.3 决策核心是 VLM，但不是全交给 VLM

程序仍然负责：

- 目标深度计算
- 动作安全裁剪
- AirSim 控制执行

### 17.4 实际用了两次 VLM

1. 找目标框
2. 规划动作

### 17.5 深度不是模型猜的，是程序从 AirSim 原始矩阵算的

这一点很关键，也很有说服力。

### 17.6 候选轨迹不是摆设

VLM 会输出多条候选，并给出 `selected_index`。

### 17.7 执行层非常朴素但可靠

最终执行层本质上就是：

- `moveToPositionAsync`
- `rotateToYawAsync`

### 17.8 整个系统最像一个“感知-规划-执行”的最小闭环原型

前端、VLM、深度、AirSim 都已经串起来了。

---

## 18. 如果你只剩 1 分钟，照着这段讲

可以直接这么说：

> 我们这个 AirSim 原型是一个网页驱动的无人机闭环系统。用户先在前端输入任务，比如“飞到较远汽车旁”。后端主循环接到任务后，会从 AirSim 取无人机第一视角 RGB、深度图和当前位姿。然后系统先调用一次 VLM 找目标在图里的位置，再结合 AirSim 的原始深度矩阵计算目标的真实距离。接着再调用一次 VLM，让它根据当前画面、任务和深度提示输出多条候选动作轨迹以及最终选择。程序把这份 JSON 解析后，转成 AirSim 的 `moveToPositionAsync` 和 `rotateToYawAsync` 指令去执行。执行完后不是结束，而是重新观察下一帧，再规划、再执行，所以它是一个真正的闭环。前端则通过 `/events`、`/frame` 和 `/depth_frame` 实时显示当前状态、画面、深度图和候选轨迹。

---

## 19. 运行时你可以盯的接口和状态

### 页面

```text
http://localhost:5000
```

### 关键接口

| 接口 | 方法 | 作用 |
|---|---|---|
| `/task` | `POST` | 下发任务 |
| `/events` | `GET` | 实时状态流 |
| `/frame` | `GET` | 当前 RGB 图 |
| `/depth_frame` | `GET` | 当前深度预览 |
| `/debug_state` | `GET` | 查看完整状态 JSON |

### 你讲解时最值得看的状态字段

| 字段 | 含义 |
|---|---|
| `status` | 当前阶段，比如 `planning`、`thinking`、`executing` |
| `task` | 当前任务 |
| `pose` | 当前无人机位置 |
| `yaw` | 当前朝向 |
| `scene_analysis` | 模型对画面的简述 |
| `candidates` | 候选轨迹 |
| `selected` | 当前真正执行的动作 |
| `task_done` | 是否完成任务 |
| `collided` | 是否发生碰撞 |

---

## 20. 最后一句话总结

这个项目可以被理解成一条非常清楚的链：

**前端输入任务 -> 后端拿 AirSim 观测 -> VLM 理解场景并给候选动作 -> 程序把动作转成 AirSim 控制命令 -> 无人机执行 -> 再看下一帧，继续闭环。**

如果你能把这条链顺着讲清楚，基本就已经把这个原型项目讲明白了。
