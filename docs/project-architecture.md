
# Uni-LaViRA AirSim 规划器 --- 架构说明

一句话：sim/ 管仿真器，agent/ 管大模型，web/ 管前端，planner/ 管配置和向后兼容。

---

## 一、项目结构

\E:\uni-lavira-code-main\
  run_airsim_web.py       # 入口：初始化 + 启动主循环 + Flask
  run_airsim_minimal.py   # 备用：简易命令行版
  requirements.txt        # pip install -r requirements.txt

  sim/                      # 仿真器层：所有 AirSim 交互
    AirSimClient           # 连接、起飞、拍照、移动、碰撞检测
    FrameCapturer          # 后台线程：持续抓取无人机画面
    ActionExecutor         # 执行轨迹：分组 -> 算航点 -> 调 API -> 碰撞恢复

  agent/                    # 智能体层：所有 VLM 规划逻辑
    Planner                # 编排器：组装消息 -> 调 API -> 解析结果
    VLMClient              # OpenAI API 调用（重试、超时）
    PromptBuilder          # 构建 prompt（图片编码 + 格式化）
    ResponseParser         # JSON 提取 + 容错修复
    ContextManager         # 多轮对话历史管理

  web/                      # Web 层：Flask 仪表盘
    SharedState            # 线程安全共享状态
    app.py                 # Flask 路由
    frontend/              # 前端静态文件

  planner/                  # 兼容层：旧代码照常 import
    config.py             # 核心配置（步长、速度、候选数等）
    trajectory.py         # 动作定义 + 坐标计算
    prompt_templates.py   # VLM 提示词模板
\
---

## 二、启动方式

\\powershell
conda activate airsim
cd E:\uni-lavira-code-main
python run_airsim_web.py
\
浏览器打开 http://localhost:5000

---

## 三、数据流向

\'
用户输入任务
  -> POST /task
  -> web/app.py 写入 SharedState.task
  -> main_loop() 检测到非空任务
    -> 1. 抓取当前帧 + 深度图 -> 写入 SharedState -> 前端展示
    -> 2. 碰撞检测 + 位姿获取 -> sim/airsim_client.py
    -> 3. VLM 规划 -> agent/planner.py
         -> prompt_builder 编码图片、格式化提示词
         -> vlm_client 调 OpenAI 兼容 API
         -> response_parser 解析 JSON
    -> 4. 选最优轨迹
    -> 5. 执行 -> sim/action_executor.py
         分组相同动作 -> 算航点 -> 调 AirSim API
    -> 6. 碰撞恢复（如有）
    -> 7. 循环回到 1
\'

---

## 四、常见需求改哪里

| 你想做的事 | 改对应文件 |
|---|---|
| 调步长/速度/候选数 | planner/config.py 或 .env |
| 改 VLM 提示词（中文） | planner/prompt_templates.py |
| 换 VLM 模型/API 地址 | agent/vlm_client.py |
| 改轨迹执行逻辑（分组、速度） | sim/action_executor.py |
| 加新动作（如 hover、急停） | planner/trajectory.py + sim/action_executor.py |
| 改前端布局/配色 | web/frontend/index.html / style.css |
| 改前端 JS 逻辑 | web/frontend/app.js |
| 加 Web 路由 | web/app.py |
| 加共享状态字段 | web/shared_state.py |
| 改碰撞恢复策略 | sim/action_executor.py |
| 换仿真器（如 Habitat） | 只改 sim/airsim_client.py |
| 加世界模型打分 | 新建 agent/trajectory_scorer.py |

---

## 五、模块职责速查

### sim/ 仿真器层

| 类/函数 | 核心方法 | 说明 |
|---|---|---|
| AirSimClient | connect(), get_pose(), get_image(), move_to_position(), rotate_to_yaw(), check_collision() | 纯 AirSim API 封装，唯一 import airsim 的地方 |
| FrameCapturer | start(state) | 后台 daemon 线程，持续抓帧写入 SharedState |
| ActionExecutor | execute(actions, scale) | 分组 -> 算航点 -> 调 AirSim API -> 返回碰撞状态 |
| actions_to_waypoints / group_consecutive_actions | 工具函数 | 动作转绝对坐标。相同动作合并 |

### agent/ 智能体层

| 类 | 核心方法 | 说明 |
|---|---|---|
| Planner | generate_candidates(frame, task, ...) | 编排器：调下面三个模块，療回结构化结果 |
| VLMClient | call(messages) | OpenAI API 调用，提取 reasoning_content |
| PromptBuilder | build_messages(frame, task, k, history, depth) | 图片 base64 编码 + 提示词格式化 |
| ResponseParser | parse(text, config, k) | JSON 提取、括号补全、容错重试 |
| ContextManager | add_step(), get_messages() | 多轮对话历史自动裁剪 |

### web/ Web 层

| 类/函数 | 方法/路由 | 说明 |
|---|---|---|
| SharedState | update(), get_state(), set_frame() | 线程安全数据总线 |
| create_app(state) | /, /frame, /events, /task | Flask 路由工厂 |
| frontend/ | index.html + app.js + style.css | 深色主题仪表盘，SSE 实时推送 |

---

## 六、运行原理

### 帧抓取
- sim/frame_capturer.py 启动独立的 AirSim 客户端连接
- 持续以 capture_interval（默认 0.1s）间隔抓取 RGB + 深度图
- 写入 SharedState，前端通过 /frame 和 /depth_frame 轮询
- 主循环执行动作期间，前端图像不中断

### 轨迹执行
- VLM 输出原子动作序列：[forward, forward, left, forward]
- action_executor.py 先分组：[(forward,2), (left,1), (forward,1)]
- 按组执行：旋转动作 rotateToYawAsync，位移动作 moveToPositionAsync
- 相同动作一次性到位，减少中间停顿

### 任务完成逻辑
- done=true + 空轨迹 -> 立刻停止（目标已在眼前 / 停止指令）
- done=true + 有轨迹 -> 先执行动作，再清空任务
- done=false -> 继续下一步规划

---

## 七、加功能示例

### 例1：增加急停动作
1. planner/trajectory.py -> AtomicAction 加 STOP
2. sim/action_executor.py -> execute() 中 if action == stop 跳过移动
3. planner/prompt_templates.py -> 提示词加上 stop

### 例2：加世界模型打分排序
1. 新建 agent/trajectory_scorer.py
2. 在 agent/planner.py 的 generate_candidates() 中调 scorer 排序

