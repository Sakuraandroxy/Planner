# Mock Demo 架构详解与真机部署指南

> 本文档系统地解释 `mock_demo.py` 的完整原理架构，以及如何一步步将演示转化为真实机器人/无人机的控制系统。

---

## 第一部分：Mock Demo 架构详解

### 1.1 总体架构

```
┌─────────── 浏览器（手机/电脑）───────────┐
│  http://127.0.0.1:5000                   │
│                                          │
│  ┌──────────────────────────────────────┐│
│  │  HTML 页面（index.html）              ││
│  │  ┌──────────────┐  ┌──────────────┐  ││
│  │  │ 视频窗口       │  │ 状态框        │  ││
│  │  │ <img> MJPEG流  │  │ LLM思考过程   │  ││
│  │  └──────────────┘  │ 实时推送       │  ││
│  │                     └──────────────┘  ││
│  │  ┌──────────────────────────────────┐ ││
│  │  │ 输入框 + 发送按钮 + 语音按钮      │ ││
│  │  └──────────────────────────────────┘ ││
│  └──────────────────────────────────────┘│
│          ↕  WebSocket (SocketIO)         │
│          ↕  HTTP (MJPEG)                  │
└──────────────────────────────────────────┘
                     │
                     ▼
┌───────────── Flask 服务器（mock_demo.py）────┐
│                                                │
│  ┌────────────────────────────────────────┐   │
│  │  路由：                                 │   │
│  │  GET  /          → 返回 HTML 页面       │   │
│  │  GET  /video_feed → MJPEG 视频流        │   │
│  │  SocketIO 事件：                         │   │
│  │  text_command  → 接收指令               │   │
│  │  status_update → 推送状态到前端          │   │
│  │  response      → 推送结果到前端          │   │
│  └────────────────────────────────────────┘   │
│                     │                          │
│                     ▼                          │
│  ┌────────────────────────────────────────┐   │
│  │  MockController 模拟层                  │   │
│  │                                         │   │
│  │  start_new_task(指令)                   │   │
│  │    → 启动后台线程 _run_mock_task()       │   │
│  │    → 按顺序发射模拟事件：                 │   │
│  │      1. "拍摄全景图..."                  │   │
│  │      2. "LA模型推理：分析场景..."         │   │
│  │      3. "VA模型检测：寻找目标bbox..."     │   │
│  │      4. "执行轨迹..."                     │   │
│  │      5. ...                              │   │
│  │                                         │   │
│  │  get_front_image_jpeg()                  │   │
│  │    → 读取本地图片 → JPEG编码 → 返回      │   │
│  └────────────────────────────────────────┘   │
└────────────────────────────────────────────────┘
```

### 1.2 核心通信链路详解

#### 1.2.1 SocketIO WebSocket 事件流

```python
# ========== 方向：前端 → 后端 ==========

# 事件1：text_command（用户打字）
前端发送: { "instruction": "飞到红色大树上方" }
后端处理: controller.start_new_task(instruction)

# 事件2：audio_command（用户语音——Mock省略，直接文字替代）
前端发送: audio_blob_bytes
后端处理: whisper三级别转写 → start_new_task()


# ========== 方向：后端 → 前端 ==========

# 事件1：status_update（状态更新，展示LLM思考过程）
后端发射: socketio.emit("status_update", {
    "message": "🧠 LA模型推理中：分析场景和任务目标..."
})
前端收到: socket.on("status_update", (d) => addStatus("📋 " + d.message))

# 事件2：response（最终响应）
后端发射: socketio.emit("response", {
    "message": "✅ 导航完成！已到达目标位置。"
})
前端收到: socket.on("response", (d) => addStatus("🤖 " + d.message))
```

#### 1.2.2 MJPEG 视频流

```python
# HTTP 长连接，永不关闭
@app.route("/video_feed")
def video_feed():
    def generate():
        while True:
            jpeg_bytes = controller.get_front_image_jpeg()
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg_bytes + b"\r\n"
            time.sleep(0.05)  # ~20 FPS
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
```

前端通过 `<img src="/video_feed">` 引用，浏览器自动解析 `multipart/x-mixed-replace` 格式，每收到一帧自动替换显示。

#### 1.2.3 后台线程模型

这是整个系统的关键设计——**导航循环在后台线程运行，不阻塞 Web 服务器**：

```python
def start_new_task(self, instruction):
    # 后台线程：daemon=True，主进程退出时自动结束
    self.task_thread = threading.Thread(
        target=self._run_mock_task,
        args=(instruction,),
        daemon=True
    )
    self.task_thread.start()
    # 主线程立即返回，继续处理 Web 请求

def _run_mock_task(self, instruction):
    # 在这个线程中，按顺序发射 SocketIO 事件
    socketio.emit("status_update", {"message": "第1步..."})
    time.sleep(1.5)
    socketio.emit("status_update", {"message": "第2步..."})
    # ... 所有 sleep 只是模拟 LLM 的耗时
```

### 1.3 Mock 层的三个替换点

Mock 演示用模拟数据替换了真实系统的三个部分：

| 替换点 | Mock 实现 | 真实实现 |
|--------|----------|----------|
| **视频帧** | `assets/teaser.png` 静态图 | 相机实时画面 |
| **LLM 响应** | 预写的字符串 + time.sleep | OpenAI API 调用 |
| **动作执行** | 只发状态文字 | iPlanner + 机器人控制 |

**这三个替换点就是"从 Mock 到真机"需要改造的三个地方。**

---

## 第二部分：从 Mock 到真实机器人的转换路径

### 2.1 总体路线图

```
阶段一：Mock Demo（已完成）
  └── 验证 Web UI + SocketIO 通信链路正常工作

阶段二：替换视频流（接入真实/仿真相机）
  └── 用 AirSim 仿真画面或相机画面替换静态图片

阶段三：替换 LLM 推理（接入真实 VLM）
  └── 配置本地 llama.cpp 或远程 API，调用 LA + VA 模型

阶段四：替换动作执行（接入仿真器）
  └── 接入 AirSim，执行真实动作并获得反馈

阶段五：接入真实无人机
  └── 用真实硬件替换仿真器
```

---

### 2.2 阶段二：替换视频流

**目标**：让视频窗口显示真实的仿真器或相机画面。

#### 方案 A：接入 AirSim 仿真画面

```python
# 在 MockController 中替换 get_front_image_jpeg()
class RealController(MockController):
    def __init__(self):
        import airsim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
    
    def get_front_image_jpeg(self):
        # 从 AirSim 获取前相机帧
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene)
        ])
        response = responses[0]
        if response.width > 0 and response.height > 0:
            img = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img = img.reshape(response.height, response.width, 3)
            _, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return jpeg.tobytes()
        return super().get_front_image_jpeg()  # fallback to static
```

#### 方案 B：接入 USB 相机（真实硬件）

```python
import cv2

class RealController(MockController):
    def __init__(self):
        self.cap = cv2.VideoCapture(0)  # 第一个 USB 相机
    
    def get_front_image_jpeg(self):
        ret, frame = self.cap.read()
        if ret:
            _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return jpeg.tobytes()
        return super().get_front_image_jpeg()
```

---

### 2.3 阶段三：替换 LLM 推理

**目标**：用真实的 VLM API 调用替换模拟的字符串输出。

#### 第一步：设置本地/远程 VLM

本地部署（llama.cpp）：
```bash
# 下载 Qwen3.5-27B 模型
llama-server --model path/to/Qwen3.5-27B-Q4_K_M.gguf \
    --mmproj path/to/mmproj.gguf \
    --alias Qwen3.5-27B-Q4_K_M \
    --host 0.0.0.0 --port 8000 \
    --ctx-size 8192 --n-gpu-layers 999
```

或使用远程 API（无需本地 GPU）：
```bash
# 在 .env 中配置
export LA_API_KEY=your_api_key
export LA_BASE_URL=https://yunwu.ai/v1
export LA_MODEL_NAME=gemini-3.5-flash
```

#### 第二步：替换 _run_mock_task 中的模拟调用

对照仓库中真实的 LLM 调用代码：

```python
# 参考：real-world-code/unitree_g1/robot/navigation_api.py
class LaViRANavigationAPI:
    def __init__(self, client):
        self.client = client  # LaViRAVisionClient
    
    def language_action(self, instruction, panorama_frames, ...):
        """LA 模型：战略决策——决定往哪个方向走"""
        messages = self._build_strategic_prompt(instruction, panorama_frames)
        response = self.client.la_client.chat.completions.create(
            model=self.client.la_model_name,
            messages=messages,
            max_tokens=4096,
            temperature=0.7
        )
        return json.loads(response.choices[0].message.content)
        # 返回: {"turn_direction": "front", "stop": false, "reasoning": "..."}
    
    def vision_action(self, img_np, instruction, ...):
        """VA 模型：战术检测——找到目标位置"""
        messages = self._build_tactical_prompt(img_np, instruction)
        response = self.client.va_client.chat.completions.create(
            model=self.client.va_model_name,
            messages=messages,
            max_tokens=2048,
            temperature=0.1
        )
        return json.loads(response.choices[0].message.content)
        # 返回: {"action": "NAVIGATE", "bbox_2d": [x1,y1,x2,y2]}
```

在你的 DemoController 中替换：

```python
class LLMController(MockController):
    def __init__(self):
        super().__init__()
        self.client = LaViRAVisionClient()  # 复用仓库中的 VLM 客户端
        self.nav_api = LaViRANavigationAPI(self.client)
    
    def _run_task(self, instruction):
        """真实的 LLM 导航循环"""
        while not self.is_stopped:
            # 1. 获取当前帧
            frame = self._get_current_frame()
            
            # 2. LA 模型调用（包含推理过程推送）
            socketio.emit("status_update", {"message": "🧠 LA模型推理中..."})
            decision = self.nav_api.language_action(
                instruction=instruction,
                panorama_frames=[frame],
                current_step=self.step
            )
            socketio.emit("status_update", {
                "message": f"✅ LA决策：{decision['turn_direction']}，理由：{decision['reasoning']}"
            })
            
            # 3. 执行动作
            # ...（接入阶段四的仿真器/真机）
```

#### 第三步：复用仓库现有的 prompt 模板

```python
# 参考：real-world-code/unitree_g1/robot/navigation_api.py 中的 prompt 构建
# 或参考：sim-code/airsim/src/model_wrapper/unilavira_model.py 中的 query_llm()

# LA 模型的 prompt 结构（参考已有代码）：
STRATEGIC_PROMPT = """
你是一个无人机导航规划器。
任务目标：{instruction}
当前视角图像如上所示。

分析任务并输出 JSON：
{{
    "turn_direction": "front/left/right/behind",
    "stop": true/false,
    "reasoning": "你的推理过程"
}}
"""

# VA 模型的 prompt 结构（参考已有代码）：
TACTICAL_PROMPT = """
你面前是无人机的前方视角。
任务目标：{instruction}

请判断目标物体是否在画面中，如果在，输出它的边界框。
输出 JSON：
{{
    "action": "NAVIGATE/STOP",
    "bbox_2d": [x1,y1,x2,y2]
}}
"""
```

---

### 2.4 阶段四：接入 AirSim 仿真器

**目标**：实现完整的"感知→规划→执行→观测→再规划"闭环。

#### 替换后的完整控制器

```python
class AirSimController:
    """完整的 AirSim 仿真控制器"""
    
    def __init__(self, ip="127.0.0.1"):
        import airsim
        self.client = airsim.MultirotorClient(ip)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
        # VLM 客户端
        self.client = LaViRAVisionClient()
        self.nav_api = LaViRANavigationAPI(self.client)
        
        self.is_task_running = False
    
    def get_front_image_jpeg(self):
        """获取 AirSim 前相机帧 → JPEG"""
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
        ])
        if responses and responses[0].width > 0:
            img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            img = img.reshape(responses[0].height, responses[0].width, 3)
            _, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return jpeg.tobytes()
        return None
    
    def start_new_task(self, instruction):
        """启动导航任务"""
        self.is_task_running = True
        socketio.emit("response", {"message": f"收到指令：{instruction}"})
        
        thread = threading.Thread(
            target=self._navigation_loop,
            args=(instruction,),
            daemon=True
        )
        thread.start()
    
    def _navigation_loop(self, instruction):
        """主导航循环"""
        try:
            step = 0
            while self.is_task_running:
                step += 1
                
                # =========================================================
                # 1. 获取当前全景图
                # =========================================================
                socketio.emit("status_update", {
                    "message": f"📷 第{step}步：拍摄全景图..."
                })
                rgb_list, depth_list = self._get_panorama()
                
                # =========================================================
                # 2. LA 模型：战略决策
                # =========================================================
                socketio.emit("status_update", {
                    "message": "🧠 LA模型推理中：分析场景和任务目标..."
                })
                decision = self.nav_api.language_action(
                    instruction=instruction,
                    panorama_frames=rgb_list,
                    current_step=step
                )
                turn_direction = decision.get("turn_direction", "front")
                socketio.emit("status_update", {
                    "message": f"✅ LA决策：向{turn_direction}，"
                               f"理由：{decision.get('reasoning','')}"
                })
                
                if decision.get("stop", False):
                    socketio.emit("status_update", {"message": "🎯 目标已到达！"})
                    socketio.emit("response", {"message": "✅ 导航完成！"})
                    break
                
                # =========================================================
                # 3. 旋转无人机
                # =========================================================
                socketio.emit("status_update", {"message": f"🔄 转向{turn_direction}..."})
                if turn_direction == "left":
                    self._rotate_by(90)
                elif turn_direction == "right":
                    self._rotate_by(-90)
                elif turn_direction == "behind":
                    self._rotate_by(180)
                
                # =========================================================
                # 4. VA 模型：目标检测
                # =========================================================
                socketio.emit("status_update", {
                    "message": "👁️ VA模型检测中：寻找目标..."
                })
                front_img = self._get_front_rgb()
                bbox_result = self.nav_api.vision_action(
                    img_np=front_img,
                    instruction=instruction
                )
                
                # =========================================================
                # 5. 执行轨迹
                # =========================================================
                socketio.emit("status_update", {"message": "🚁 执行轨迹..."})
                self._execute_trajectory(bbox_result)
                
        except Exception as e:
            socketio.emit("status_update", {"message": f"❌ 导航出错: {e}"})
        finally:
            self.is_task_running = False
    
    def _get_panorama(self):
        """获取五方向全景图（前/左/右/后/下）"""
        # 参考：sim-code/airsim/src/vlnce_src/env_uav.py 中的图像获取
        # 和 unilavira_model.py 中的 get_panorama()
        pass
    
    def _execute_trajectory(self, bbox_result):
        """执行轨迹：将 bbox → 3D 坐标 → 飞行控制"""
        # 参考：real-world-code/unitree_g1/tasks/vln.py 中第6步之后的代码
        # 将 bbox 底部中心像素 + 深度图 → 3D 世界坐标
        # → iPlanner 规划路径 → 执行
        pass
    
    def _rotate_by(self, degrees):
        """旋转无人机指定角度"""
        pass
    
    def _get_front_rgb(self):
        """获取前相机 RGB 图像"""
        pass
```

#### 将控制器接入 Flask 服务器

```python
# 在 mock_demo.py 末尾，只需替换一行：
# controller = MockController()       ← 旧
controller = AirSimController()   # ← 新
# 其余代码完全不变！
```

---

### 2.5 阶段五：接入真实无人机

**目标**：用真实无人机替换 AirSim 仿真器。

#### 可选的无人机平台

**选项 1：自研无人机 + PX4（参考 `self_built_uav/`）**

```bash
# 硬件：NxtPX4v2 飞控 + Livox LiDAR + RealSense 相机
# 启动顺序：
roslaunch realsense2_camera rs_camera_vins.launch
# → FAST-LIO2 → ROS 节点提供 /Odometry
roslaunch vln_node indoor_eval.launch
```

替换控制器的飞行控制部分：

```python
class PX4Controller(AirSimController):
    """PX4 真实无人机控制器"""
    
    def __init__(self):
        import rospy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import PositionTarget
        
        rospy.init_node("vlm_planner")
        self.pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # VLM 客户端不变
        self.vlm_client = LaViRAVisionClient()
    
    def _execute_trajectory(self, target_xyz):
        """发送位置指令到 PX4"""
        pose = PoseStamped()
        pose.pose.position.x = target_xyz[0]
        pose.pose.position.y = target_xyz[1]
        pose.pose.position.z = target_xyz[2]
        self.pose_pub.publish(pose)
```

**选项 2：Unitree 机器人（Go1/G1）**

```bash
# 参考：real-world-code/unitree_g1/main.py
python main.py --task interact --inference_mode local
# 这已经包含了完整的 Web UI + LLM + 机器人控制
# 只需确保硬件已连接：Unitree SDK + Orbbec 相机
```

---

## 第三部分：代码对照清单

### 3.1 文件对应关系

| Mock Demo | 真实代码 | 功能 |
|-----------|---------|------|
| `mock_demo.py` | `unitree_g1/main.py` + `unitree_g1/web/app.py` | 主入口 + Web 服务器 |
| `MockController._run_mock_task()` | `unitree_g1/tasks/vln.py` VLNTask.run() | 导航循环 |
| `MockController.start_new_task()` | `unitree_g1/robot/nav_controller.py` IntegratedVisionNavController.start_new_task() | 任务管理 |
| 模拟的 LA 字符串 | `unitree_g1/robot/navigation_api.py` LaViRANavigationAPI.language_action() | LA 模型调用 |
| 模拟的 VA 字符串 | `unitree_g1/robot/navigation_api.py` LaViRANavigationAPI.vision_action() | VA 模型调用 |
| 模拟的 JPEG | `unitree_g1/robot/robot_controller.py` RobotController.get_front_image_jpeg() | 实时视频流 |
| HTML + JS 前端 | `unitree_g1/web/templates/index.html` | Web UI |

### 3.2 最小改动方案

如果你已经有了 AirSim 仿真环境，从 Mock 转换到真实系统只需要修改 `mock_demo.py` 中的三个函数：

```python
# 修改 1：替换 get_front_image_jpeg()
# 旧：读取本地静态图片
# 新：从 AirSim/相机获取帧

# 修改 2：替换 _run_mock_task() 中的模拟 LLM 调用
# 旧：预写字符串 + sleep
# 新：调用 OpenAI API → 解析 JSON → 发射真实结果

# 修改 3：替换 _run_mock_task() 中的动作模拟
# 旧：只发状态文字
# 新：调用 AirSim API / ROS 发送控制指令
```

**以上三个替换，每个都可以独立进行，互不依赖。** 你可以先替换视频流（立即看到仿真画面），再替换 LLM（看到真实的推理过程），最后替换动作执行（无人机真正飞起来）。

---

## 第四部分：常见问题

### Q1：我没有 GPU，能跑 VLM 吗？

可以。仓库支持两种模式：

**远程 API 模式**（不需要任何 GPU）：
```bash
export LA_API_KEY=sk-your-key
export LA_BASE_URL=https://yunwu.ai/v1
export LA_MODEL_NAME=gemini-3.5-flash
python mock_demo.py
```

**本地 CPU 模式**（慢但可用）：
```bash
# 使用 Qwen3.5-27B 的 4-bit 量化版
llama-server --model Qwen3.5-27B-Q4_K_M.gguf --mmproj mmproj.gguf \
    --host 0.0.0.0 --port 8000 --n-gpu-layers 0  # 0 = CPU only
```

### Q2：没有 AirSim，能测试完整闭环吗？

可以。用下面的步骤建立最小闭环测试：

```bash
# 1. 启动 Mock Demo（已有视频+UI）
python mock_demo.py

# 2. 在另一个终端，替换为真实 VLM
# 修改 mock_demo.py，将 _run_mock_task 中的模拟文本
# 改为调用 OpenAI API

# 3. 如果要模拟飞行效果但没 AirSim：
# 让无人机"在状态文本中"前进——在状态框里更新位置信息
# {"message": "当前位置 (12.3, 5.6, 2.1)，继续向目标前进..."}
```

### Q3：怎么把 Mock Demo 部署到手机可以访问？

默认 `host="0.0.0.0"` 已经允许局域网访问。在同一 WiFi 下的手机：

```bash
# 查看电脑的局域网 IP
ipconfig
# 记下类似 192.168.1.100 的地址

# 手机浏览器打开
# http://192.168.1.100:5000
```

> ⚠️ 语音功能需要 HTTPS。生产环境请配置 SSL 证书，或使用 ngrok 等工具建立加密隧道。

### Q4：Mock Demo 和真机可以共享同一个前端页面吗？

完全可以。`mock_demo.py` 中的 HTML 页面与 `unitree_g1/web/templates/index.html` 的结构完全相同。唯一的区别是后端的数据来源。**前端代码完全不需要修改。**"

---

## 总结：一句话版本

**Mock Demo 就是真机系统的骨架——它实现了完全一致的前端 UI、WebSocket 通信链路、后台线程模型，只差了三个"插件"：视频源、LLM 模型、动作执行器。一个组件一个组件地替换这三样东西，就能从演示平滑过渡到真机。**
