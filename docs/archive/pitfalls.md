# 踩过的坑

按主题记录现象、根因与现在的处理。每条都来自实机或测试，不是推测。条目保留当时的数字与路径，方便对照日志。

## 进程与显示

### 引擎启动超时，且一个字节的日志都没有

现象：切图卡到 `launch_timeout_s`，`output/xpra/logs/` 空。

根因：上一次 Xvfb 被强杀，`/tmp/.X<n>-lock` 没删。同显示号再启动直接失败，引擎子进程根本没起来。

处理：启动前走 `reclaim_stale_display`——锁里的 PID 已死才删，还活着就报错并给可执行建议。不删别人的显示号。

### `xpra stop` 之后 Xvfb 还活着占着显示锁

现象：引擎与会话都退了，端口也放了，`Xvfb-for-Xpra-104` 仍在，下次启动报 `display :104 is still held by PID`。
只在接管别人拉起的栈时出现。

处理：停止收尾先验三件事再动手——同 uid、argv 里是这个显示号、进程名是 Xvfb。

### RPC 应答正常，`simGetImages` 永不返回

根因：seamless xpra 在客户端接上之前不 map 窗口，UE4 对未 map 的窗口暂停渲染。

处理：`window` 探针找 `xpra info` 里的 `windows.<id>.pid`，校验归属，再 `xpra control map` + `focus`。
不要用 `-RenderOffscreen` 绕：那个参数只是把症状抹掉，它连桌面画面一起去掉。

### 跨族切换卡住，UE5 的栈停不掉

根因：归属判据要求 argv 里有本族的 `-settings=` 标记，而 UE5 引擎没有这个标记，
UE4 驱动因此把还在跑的 UE5 栈判成不可管理的进程。

处理：拆成两个问题——`catalogued_map()` 回答"能不能停"（二进制在自家 `maps.yaml` 里就够），
`owned_map()` 回答"是哪一族的哪张图"。

### 换族失败，卡在显示锁上

根因：`FamilyRouter.restart` 把整次切换交给目标那一族的驱动，UE4 驱动只读自己的端口，看不到 UE5 栈，
于是认为机器空闲、跳过 stop 直接 launch。

处理：路由先按 marker 找到在跑的那一族 stop，再交给目标族 launch。测试是 `tests/test_family_router.py`。

### `Ctrl-C` 停服务后仿真器还在跑

根因：`ServerController.close()` 不停仿真栈。

处理：退出走与换图相同的停止阶梯；`SimulatorNotOwned` 时只告警，不动别人的栈。

### 每次关闭都报一条 worker 超时

根因：子进程 `shutdown()` 写完命令就返回，而控制器只翻队列里已有的事件。

处理：等到 CLOSED，上限 `land_timeout_s + 10`。

### 任务中 SIGKILL 子进程

现象：服务自己重启 worker 并回到 `idle`，之后切图与再飞正常。

结论：旧分支的 `ConnectionMonitor` 不必搬回，这条崩溃路径已被现有重启逻辑覆盖。未覆盖的是长时间 RPC 卡住。

### UE5 的构建不写日志

CityEnviron 与 AirSimNH 两个打包产物不写日志文件，`-log` 也不往 stdout 打，退出时不刷。
所以 UE5 的 `child_argv` 里不能放 `-abslog` / `-Stdout`，就绪判断只能靠 `window` 与 `ports`。
驱动的超时提示按实际 argv 生成，不会指一个不存在的路径。

## UE4 适配

### 载具名为空，所有控制调用静默失效

现象：`listVehicles()` 返回 `['Drone_1']`，`getMultirotorState()` 读得到数据，
但 `moveToPositionAsync(...)` 返回 False，
`landAsync("")` 直接抛 `RPCError: function 'land' ... threw an exception`。

根因：这份构建在 settings 里把载具注册成 `Drone_1`，而适配层所有调用都传空串。

处理：`vehicle_name` 从 `AirSimConnection` 构造函数注入三条 RPC 通道。
能不能收这个关键字由 `inspect.signature` 判定并缓存，不维护名单。

### 异步动作的返回值不能当成败依据

现象：`moveToPositionAsync(...).get()` 返回 False，而无人机确实从 z=-0.60 爬到 -1.59；
`takeoffAsync` 在高度增益 0.5 m 以上时也可能报 False。

处理：ack 只记日志，完成判定读回位置与朝向（0.5 m / 15°）；起飞看高度增益 ≥ 0.5 m。

### RGB 与深度必须同尺寸时，单帧要 25.97 秒

根因：`pixels_as_float=True` 时 AirSim 把 921 600 个 float 当 msgpack 数组发回，
而这份环境里的 `msgpack-python` 0.5.6 只有纯 Python 实现。

`compress=True` 不是出路：`pixels_as_float=False` 下发回的是**可视化深度**（整图只有 4 个不同值，中位 0 m），
不是米制；`pixels_as_float=True` 时 `compress` 被忽略。

处理：放宽共享契约，允许深度与自己的 `depth_intrinsics` 一致。RGB 保持 1280×720，深度 256×144，单帧 0.40 秒。

顺带发现一个被"同尺寸"掩盖的 bug：`depth_foreground` 用 RGB 内参给深度网格圈窗口。

### 深度分辨率改在 `CameraDefaults` 静默无效

生效位置是 `Vehicles.<载具>.Cameras.<相机>.CaptureSettings`；`getSettingsString()` 回报的是 `CameraDefaults`，
相机级覆盖它不报。

### PNG 不是米制

与同帧 float 参考逐像素比对：中位误差 22.99 m，误差小于 1 cm 的像素 0.00%，每通道只有 5 到 11 个唯一值。
它发回的是可视化图，不是可解的编码。

### `landed_state` 冷启动正确，落地之后就不对了

现象：冷启动稳定报 `Landed`（Blocks z=0.67，Campus z=3.56）；
落地之后永远停在 `Flying`（实测在 Blocks 上一个任务落地后再没变回 0）。

后果：第二个任务以为还在飞、不起飞，
贴地前飞 60 秒后报 `synchronized motion timed out before position/yaw settled`。症状像运动问题，真因是一个过期枚举。

处理：`prepare` 接受地面高度参考，z 回到参考高度 0.25 m 内即算在地面；
参考值取会话见过的最高 z（UE 的 z 轴向下，地面就是最大值）。

### 三处 `AttributeError` 来自照抄旧分支

| 现象 | 真因 |
|---|---|
| `'AirSimVehicle' object has no attribute 'capture'` | 装配把载具当观测源传给视觉工作流；UE5 载具自己会取图，所以这个约定从没露馅 |
| `'PositionHold' object has no attribute 'wait_for_observation'` | 共享服务里叫 `wait`，语义相同 |
| `'PositionHold' object has no attribute 'observation_ready'` | 共享服务里叫 `observable` |

处理：`build_vehicle_arm` 新增 `observation_source` 参数；后两条只改调用名。
新增 `tests/test_airsim_port_names.py`，用 AST 扫属性访问并比对真实成员，0.19 秒跑完。
这类错不该靠整机启动才发现。

### 路面与自身接触被当成碰撞

现象：郊区图输入"沿着道路行驶，不要发生碰撞"，车连续退 3/6/9 米，报 `collision recovery budget exhausted`。
服务端日志锁存 `object=Road_89`、`penetration=0.000`。

根因：碰撞忽略前缀只有 `Landscape`。城市图的路面属于地形，郊区图的路面是独立的 `Road_82/84/88/89` 模型。
出生与传送还会产生自身接触。

处理：默认忽略前缀扩展为 `("Landscape", "Road_")`，并新增 `ignored_names` 精确忽略自身名字，只忽略自己、
不影响 `Rover10` 这类其他对象。

### 上海图是坏的打包

载具出生即卡进几何体（`FindTeleportSpot ... intersecting geometry`），掉到 z=3000 米仍在自由落体，
`_wait_ready` 因速度不静止而超时。属打包问题，不是代码。

## UE5 适配

### topic socket 是单消费者

同一进程里连第二张图前必须先 `connection.close()`；只调 `client.disconnect()` 会留下活动连接，
下一张图报 `another ProjectAirSim connection is active`。

### 场景时钟暂停，不推进就没有新帧

双载具场景 `pause-on-start: true`，`World` 是可步进的。`switch_streaming_view` 只在渲染帧上生效，
任何"等画面"的循环都要继续推进时钟，否则画面停在切换前。

### 隐藏载具的横向传送把无人机嵌进墙里

现象：郊区自由导航连续退避，
服务端日志从启动隐藏那刻起持续输出 `Collision detected between 'Frame' and 'Outer_Wall_Quart_Win_10' at z=276.99`，
4 小时累计 93970 条。

根因：隐藏位姿是水平偏移 `(-300, -300)` 米，正好落在那面墙里；两台机器人的碰撞 link 都叫 `Frame`，
服务端日志无法区分来源。

处理：只沿垂直方向抬高 1000 米，不再横向移动；被隐藏的载具仍在受重力的世界里，每 2 秒仿真时间重新钉住一次。
没有隐藏载具时钩子零开销。

### 归位后车悬在半空

根因：场景文件里的作者坐标比路面高约 1 米（`z=-2.0`），物理车在时钟推进后落到 `z=-1.001`；
暖机 `capture()` 只推进了 0.05 秒，读到的还是下落前的 `-1.99`。

处理：暖机取帧后按住刹车推进 2 秒仿真时间，再取一帧，用实测姿态作为归位目标。
日志有 `Rover home pose measured ... z=...`。

### `SetObjectPose` 拒绝对同一位置传送

现象：切回无人车时整次激活失败，
报 `SetObjectPose failed. Unable to move object Rover1, check if object state is movable!`。

根因：激活时先把目标恢复回停放点，紧接着 `return_home` 又传送到同一个出生点。

处理：目标与当前距离 ≤ 0.01 m、偏航差 ≤ 0.5° 时只发 `SetPose`，跳过 actor 传送。快速物理状态是权威的，
渲染 actor 会在下一次时钟推进时同步。

### 关闭时会话读取落地状态抛 `Object closed`

后果：异常跳过了 `connection.close()`，进程级连接占用没释放，
之后所有重连都报 `another ProjectAirSim connection is active`，服务永久卡在 `connecting`。

处理：`close()` 整段 `try/finally`；落地状态读取失败只记 WARNING 并当作已落地，连接释放放在 `finally`。

### 机位循环：服务端不告诉你现在在第几格

`World.switch_streaming_view()` 只把相机往后挪一格。能做的三件事都做过：

1. 写死索引——实测顺序在每次加载后都可能不同，四个按钮整体错位。
2. 运行时标定——截图与各相机参考帧做相关匹配，正确机位 0.92 左右、其余 ≤0.28。第一次约 13.8 秒，
   其中 8 秒是无人车追尾相机的取帧超时；整个起图因此从 13.6 秒拖到 60 到 90 秒。
3. 现在：起点写进配置 `view_cycle_start`，按"起点 + 已切换次数"记账。
   标定工具 `scripts/calibrate_streaming_views.py` 留作手动确认。

标定过程中踩到的两个坑：

- 取参考帧不能占用主连接：`GetImages` 会阻塞到服务端拍下新帧，而推进时钟必须用主连接，同一个连接会死锁。
  改用服务专用客户端。
- xpra 影子帧会滞后：切机位后第一次截图可能还是旧画面。取帧前等连续两张截图相关度≥ 0.9995、且与上一机位不同。
- 逐位置挑最高分会误判：无人车追尾相机不可取帧，它的位置与车前视角相关度可达 0.75，
  而真正的车前视角得分 0.998。改成整圈排列求解，不可取帧的位置由排除法定位。

### 两台车同点出生时画面分不开

同点出生配上暂停时钟是对的，但标定前必须先把非活动载具抬走，否则两个前视相机拍的是同一条街。

## 前端

### 卡片被整块藏起来

现象：UE4 下图，世界/地图卡片加载后一秒消失。

根因：`applyCapabilities` 用 `closest("section")` 按 DOM 嵌套找卡片，而"载具模型"块写在世界卡内部；
UE4 没有 `models` 能力，整张世界卡被藏。

处理：世界、模型、天气拆成三张独立卡片，门控按 id 点名。

### 卡片被裁掉

世界设置卡片内容变长后被容器裁掉，既看不到也滚不动。改成自适应高度加滚动容器。

### 机位切换时画面来回跳

标定、切载具、切机位都靠循环相机，控制台会把中间机位全部显示出来。控制台用不透明纯黑遮罩盖住 iframe：
`connecting`、`unknown` 或 `view_pending=true` 时显示，机位确认后隐藏。遮罩上不放文字。

## 模型端点

### 任何指令都报 `ProtocolError: task parser request failed (HTTPStatusError)`

真因是密钥缺失导致 401，旧代码只打印异常类名。

处理：`adapters/http_errors.py` 统一上报状态码、URL、该改哪个字段与 provider 原话；
装配期对无凭据的端点直接告警。

### 报文格式与 provider 不匹配

`request_format` 取 `openai` / `deepseek` / `anthropic`。`anthropic` 用 Messages API，
只接受 `reasoning_effort: none`。`openai` 的思考开关字段与另外两家不同。

### 端点能收图

`api.deepseek.com/anthropic` 可以收图像输入。曾经担心这台机器没有可用视觉模型，事实不是。

## 测试与工具

### `TestClient(app)` 不进上下文管理器会假卡死

现象：脚本化验 HTTP 服务时服务一直停在 `connecting`，而底下的仿真器是好的。

根因：控制器的监控线程在 lifespan 里启动；不进上下文管理器就不跑 lifespan，事件没人排空。

处理：

```python
with TestClient(build_server(load_config("config/base.yaml"))) as client:
    ...
```

### 共存检查给假绿灯

`import msgpackrpc, pynng, msgpack` 不碰打包 API，混装下照样打印 `coexist ok`。判共存要跑到真正发包那一步：
`airsim.MultirotorClient().ping()`。`TransportError` 是环境正确，`TypeError` 是被覆盖了。

### 环境自检要求一个没法修的 FAIL

原来把两族依赖压在同一份检查里，在 UE4 运行时环境里因缺 `fastapi` 报 FAIL。

处理：`probe_ue4_env.py` 加 `--role auto|service|runtime`，按能导入的包判断站在哪个环境。
不要为了让它变绿把 `airsim` 装进主环境。

### 装 `airsim` 报构建失败

`airsim` 只有 sdist，`setup.py` 开头 `import numpy`，构建隔离环境里没有。先装 numpy，
再 `pip install --no-deps --no-build-isolation airsim==1.8.1`。

### 装 `projectairsim` 会把 `cv2` 装两份

它声明依赖 `opencv-python`（GUI 版），与 `opencv-python-headless` 抢同一个目录。用 `--no-deps` 装本体，
基础依赖手工补齐。

### `pip check` 报噪音

`projectairsim requires opencv-python`、`airsim requires opencv-contrib-python`。满足它们的是 headless 变体，属预期。

### 在 macOS 上无法验证任何仿真器行为

历史记录：所有仿真器侧行为都是在目标 Linux 机器上验证的；文档里标"未验"的结论必须重跑。