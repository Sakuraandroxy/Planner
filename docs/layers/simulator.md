> 上级：[项目架构](../ARCHITECTURE.md)
> 同层兄弟：[domain](domain.md)、[ports](ports.md)、[algorithms](algorithms.md)、[runtime](runtime.md)、[workflows](workflows.md)、[assembly](assembly.md)、[adapters](adapters.md)、[simulators](simulators.md)

# simulator：进程栈与 worker 协议

## 1 一句话职责

本层是与引擎无关的进程栈：用 `xpra start --start-child` 起停一套仿真器软件、把任务会话放在线程或子进程里跑、把"A 用什么办法跑任务"收敛成装配层能读的一组接口与一份端口 bundle。

## 2 边界

管这些：

- 一套仿真器的起停：argv 渲染、就绪探针、归属校验、停止时的降级 kill、X display 回收（`planner/simulator/xpra_start.py:59`）。
- 进程级原语：端口存活、监听者 PID、`/proc` 身份快照、进程树终止、display 锁判定（`planner/simulator/processes.py`）。
- 任务会话的两种运行形态：线程 `ThreadedMissionWorker`（`planner/simulator/worker.py:18`）与子进程 `SubprocessMissionWorker` + `MissionChild`（`planner/simulator/subprocess_worker.py:113`、`planner/simulator/mission_child.py:141`）。
- worker 协议类型与事件：`BackendWorker`、`TaskSubmission`、`WorkerEvent`、`WorkerEventKind`（`planner/simulator/worker_protocol.py:14-69`）。
- 仿真器 id → 模块 → 会话的解析（`planner/simulator/registry.py:18-84`）。
- 端口 bundle 的数据结构与占位实现：`SimulatorPorts`（`planner/simulator/ports.py:42-119`）。
- 地图标记文件读写（`planner/simulator/marker.py:17-35`）。
- 子进程里可用的轨迹转发 sink（`planner/simulator/telemetry_relay.py:27`）。

不管这些：

- 引擎客户端调用、相机、天气、载具动作：见 [simulators](simulators.md)。
- 任务怎么跑、运动律怎么算：见 [workflows](workflows.md)、[runtime](runtime.md)。
- 能力位与任务需求表：`planner/assembly/requirements.py:97`，见 [assembly](assembly.md)。
- HTTP 路由与模型客户端：见 [adapters](adapters.md)。

## 3 输入与输出

### 入口

| 入口 | 位置 | 说明 |
| --- | --- | --- |
| `registry.ports_for(simulator)` | `registry.py:28` | 仿真器 id → 声明的 `SimulatorPorts` |
| `registry.capabilities_for(simulator)` | `registry.py:38` | 转调装配层推导，返回能力位 |
| `registry.build_session(simulator, *, config, map_entry, family, resources, log)` | `registry.py:45` | 建会话并接上任务臂 |
| `registry.labels()` | `registry.py:87` | 控制台显示名 |
| `FamilyRouter.running/preflight/stop/start/restart/launch_command` | `router.py:83-113` | 按地图所属 family 路由 |
| `XpraStartSupervisor.start/stop/restart/launch/child_argv/launch_argv/holders/owned_map/catalogued_map/window_ready` | `xpra_start.py` | 单 family 的驱动 |
| `ThreadedMissionWorker` / `SubprocessMissionWorker` | `worker.py:18`、`subprocess_worker.py:113` | 实现 `BackendWorker` 的全部方法 |
| `MissionChild.main(argv)` | `mission_child.py:375` | 子进程入口，`--config`、`--map` |

### 契约类型

`SimulatorPorts`（`ports.py:42-119`，字段 `vehicle`、`observation`、`streaming`、`controlled_capture`、`moving_capture`、`weather`、`views`、`models`、`frames`、`home`、`vehicles`）、`BackendWorker`（`worker_protocol.py:50-69`）、`TaskSubmission`（`:31-35`）、`WorkerEvent`（`:38-47`）、`WorkerEventKind`（`:14-27`，九个取值）。

### 返回与异常

| 异常 | 抛出点 | 语义 |
| --- | --- | --- |
| `ValueError` | `registry.py:31`、`registry.py:55`、`registry.py:62`、`argv.py:31`、`argv.py:36`、`xpra_start.py:102-112` | 未知仿真器、argv 占位符无法解析、family 声明了驱动不支持的探针 |
| `SimulatorRestartError` | `xpra_start.py:272`、`:306`、`:376`、`:463`、`processes.py:190-241` | 起不来、另一个地图在跑、display 被占 |
| `SimulatorNotOwned` | `xpra_start.py:231-249` | 端口属于无法证明归属的进程，一个都不杀 |
| `UnattributedListener` | `processes.py:55-56`、`:72-75` | 端口在监听但 `ss` 读不到属主 |
| `ChildProcessError` | `subprocess_worker.py:52`、`:66-79`、`:83-90`、`:231-241` | 子进程非协议输出、事件不可解析、管道已断 |

## 4 对外名字表

| 名字 | 类型 | 一句话用途 | 层外调用点 |
| --- | --- | --- | --- |
| `SimulatorPorts` | dataclass | 引擎交回的端口 bundle | `planner/assembly/requirements.py:97` 起的推导链、`planner/simulators/*/runtime.py:45`/`:75` |
| `SimulatorPorts.from_slots` | 类方法 | 不建客户端先报能力 | `planner/simulators/airsim/runtime.py:75`、`planner/simulators/projectairsim/runtime.py:75` |
| `SimulatorPorts.implemented` | 方法 | 列出真正填了的槽位 | `requirements.capabilities_for`、`tests/test_simulator_port_slots.py` |
| `SimulatorPorts.verify_slots` | 方法 | 构造期比对声明槽位与实物，缺一个就报错并点名 | 两个引擎的 `build_ports`（`airsim/runtime.py:30-58`、`projectairsim/runtime.py:34-68`） |
| `BackendWorker` | Protocol | 驱动进程栈的五个 + 四个可选方法 | `planner/assembly/controller/commands.py:194` 等控制器路径 |
| `TaskSubmission` | dataclass | 一次任务指令 | `planner/assembly/…`、`planner/simulators/*/session.py:32-33` |
| `WorkerEvent`/`WorkerEventKind` | dataclass/Enum | worker → 控制器的事件 | 控制器事件泵、`planner/assembly/reporting.py:48` 的配合 |
| `registry.build_session` | 函数 | 建会话 + `attach_arms` | `planner/server_bootstrap.py:98` |
| `registry.capabilities_for` | 函数 | 能力位 | `planner/assembly/controller/__init__.py:219`、`planner/config/vehicles.py:181` |
| `registry.labels` | 函数 | 显示名 | `planner/assembly/controller/__init__.py:229` |
| `FamilyRouter` | 类 | 多 family 路由 | `planner/server_bootstrap.py:111` |
| `ThreadedMissionWorker` | 类 | 同进程跑会话 | `planner/server_bootstrap.py:107` |
| `SubprocessMissionWorker` | 类 | 子进程跑会话 | `planner/server_bootstrap.py:84` |
| `read_map_marker` | 函数 | 读"当前在跑哪张图" | `planner/server_bootstrap.py:115`、`router.py:79` |
| `write_map_marker` | 函数 | 启动成功后写标记 | `xpra_start.py:313` |
| `RelayedTelemetry` | 类 | 子进程侧的轨迹 sink | `mission_child.py:242` |
| `EVENT_FD_ENV` | 常量 | 协议 fd 的环境变量名 | `mission_child.py:41` 读取 |

## 5 关键流程

### 5.1 服务启动时选栈

1. `server_bootstrap` 调 `read_map_marker`，标记文件指向的地图不在目录里就当没有（`marker.py:24-35`、`server_bootstrap.py:115`）。
2. 起 `FamilyRouter(config.families, config.simulator, PROJECT_DIR, config.maps)`（`server_bootstrap.py:111`）。
3. 控制器调 `running()`/`preflight()`/`stop()` 时，路由目标由标记文件推 family：`_active_family_id` 读标记 → 地图 → `entry.simulator`，失败回落 `default_family`（`router.py:74-81`）。
4. 带地图的调用按 `entry.simulator` 直接选 driver（`router.py:93-113`）。

### 5.2 起一套仿真器

1. `XpraStartSupervisor.start(entry)`：若 `running()` 且所有监听者都属于这张图，直接返回；属于别的图则抛 `SimulatorRestartError`（`xpra_start.py:266-276`）。
2. `launch` 检查 binary 可执行（`xpra_start.py:316-321`）→ `preflight` 校验归属（`:277-280`）→ `reclaim_stale_display` 清掉死掉的 X 锁（`:281`）。
3. `launch_argv` 渲染 argv 模板，`{start_child}` 由 `shlex.join(child_argv(entry))` 拼成（`argv.py:20-46`、`xpra_start.py:132-140`）。
4. 建 `runtime_dir/{logs,sockets,sessions}`，把 launcher 输出追加进 `logs/xpra-start.<map>.log`，然后 `spawn`（`xpra_start.py:282-306`）。
5. 记录 launcher 进程的 `identity` 快照（`xpra_start.py:304`）。
6. `_wait_ready` 按 family 声明的探针顺序等待：`ports` → `rpc` → `window`（`xpra_start.py:323-350`）。
7. 每轮先看 launcher 是否已退出，退出就立刻失败并指日志（`xpra_start.py:363-375`）；超时上限 `settings.launch_timeout_s`，默认 180 s（`planner/config/sections.py:464`）。
8. `rpc` 探针要求端口存活且**所有**监听者都是这张图（`xpra_start.py:381-388`）。
9. `window` 探针用 `xpra info`/`xpra control map|focus` 把窗口映射并聚焦（`xpra_start.py:402-424`）。
10. 全绿后写标记文件并记日志（`xpra_start.py:313-314`）。
11. 任一步失败都走 `_cleanup_launch`，不留半启动的栈（`xpra_start.py:308-312`）。

### 5.3 停一套仿真器

1. `holders()` 先给所有端口找属主：`owned_map`（本 family 指纹：可执行文件 realpath + `-settings=`）或 `catalogued_map`（任意 family，只要 binary 在 maps.yaml 里）（`xpra_start.py:227-250`）。
2. 任一端口属主不可证明 → 抛 `SimulatorNotOwned`，一个信号都不发（`xpra_start.py:239-249`）。
3. 先发 `xpra stop`（`xpra_start.py:426-437`），然后 `_wait_ports_down(port_release_timeout_s)`，默认 30 s（`xpra_start.py:468-474`）。
4. 端口没下来就**重新**读属主与身份快照，再 `terminate_verified_tree`：SIGTERM 整棵树 → 等 `KILL_WAIT_S=5.0` → 只对身份未变的 PID 发 SIGKILL（`processes.py:152-189`）。
5. 仍不放过就再 `holders()` 一次，抛 `SimulatorRestartError`（`xpra_start.py:463-466`）。
6. `_cleanup_launch`：回收自己 spawn 的 launcher → 等 display 释放 → `_release_orphan_display` 收掉 xpra 留下的 `Xvfb-for-Xpra-<n>`，但只在"属主是我 + 命令行里含 `Xvfb` 与 display 号"时杀（`xpra_start.py:476-554`）。

### 5.4 任务会话的线程形态

1. `ThreadedMissionWorker.start` 起 `vln-worker` 线程，`daemon=True`（`worker.py:39-45`）。
2. `_run` 用 `session_factory(log)` 建会话；失败发 `ERROR` 并直接返回（`worker.py:101-120`）。
3. 成功后发 `READY`，若会话有 `weather_catalog` 再发 `WEATHER`（`worker.py:121-140`）。
4. `_loop` 每 50 ms 取一条命令；空档调 `session.idle_tick()`（`worker.py:142-163`）。
5. `task` → `_run_task`：重置 stop event、调 `session.run(task, stop_event=…, log=…)`、把结果交给 `task_finish_events` 发 `LOG` 与 `FINISHED`（`worker.py:246-276`）。
6. `home`/`activate`/`view`/`weather` 用 `getattr(session, 名字, None)` 鸭子调用，会话没有就静默跳过（`worker.py:164-240`）。
7. `shutdown` 置 stop event、投 `shutdown` 命令、`join(join_timeout_s=10.0)`；`_run` 的 `finally` 调 `session.close(skip_landing=…)` 并发 `CLOSED`（`worker.py:91-97`、`:149-175`）。
8. `terminate` 走 `shutdown` 之后仍存活的线程：置放弃标记、打一条 WARNING、在 `vln-worker-release` 守护线程上 `session.close(skip_landing=True)` 拆掉底层连接，之后 `submit` 只记一条 ERROR 不再入队（`worker.py:52-60`、`:99-135`）。线程不可强杀，泄漏以日志与拒收显式化（见 5.5 第 6 步的子进程对照）。

### 5.5 任务会话的子进程形态

1. `SubprocessMissionWorker.start`：`os.pipe()` 自建协议管道，把写端 fd 号塞进 `PLANNER_MISSION_EVENT_FD` 并设为可继承（`subprocess_worker.py:159-175`）。
2. argv 是 `<runtime_python> -m planner.simulator.mission_child --config … --map …`，`cwd` 是仓库根，`pass_fds=(write_fd,)`（`subprocess_worker.py:176-198`）。
3. `_pump` 线程逐行读协议 fd；`TRAJECTORY` 事件不进事件队列，直接 `trajectory_sink.adopt(payload)`（`subprocess_worker.py:199-231`）。
4. `stdout` 与 `stderr` 各起一个 `mission-child-stdout`/`mission-child-stderr` 守护读线程，与协议 fd 同时排空，每行转发成 `LOG`（`subprocess_worker.py:232-259`）。协议行不排在日志之后：解析在本线程随到随发。
5. 协议 fd 到 EOF 后，以 `join_timeout_s` 为界等两个日志线程；仍在跑的按名字打 WARNING，再 `process.wait()` 发 `CLOSED`（`subprocess_worker.py:260-274`）。
5. `shutdown` 发命令后同步等 `CLOSED`，超时 `shutdown_timeout_s=60.0`，等到的队列内容原样放回（`subprocess_worker.py:300-337`）。
6. `terminate` 发 SIGTERM，`join` 超时就 SIGKILL（`subprocess_worker.py:360-378`）。
7. 子进程侧 `MissionChild.run`：装 `_LogBridge` 把库日志塞进协议 → 起 stdin 读线程 → 建会话 → 发 `READY` 与 `WEATHER` → 50 ms 循环取命令（`mission_child.py:327-373`）。
8. 子进程的 `stop` 在 stdin 线程里**立即**设 event，不进命令队列（`mission_child.py:175-215`）；任务号在入队前就占住，`stop` 能取消"已排队未开跑"的任务（`:215-221`）。
9. 子进程的 `home`/`activate`/`view` 一律记 ERROR：这条路径上不支持（`mission_child.py:315-322`）。

### 5.6 换栈

`FamilyRouter.restart(entry)`：先按标记找到**实际在跑**的 family 并 `stop()`，再让目标 family `restart(entry)`（`router.py:96-109`）。两个 family 共用 `xpra: 10010`（`config/simulators.yaml:20`、`:123`），所以同一时刻只能有一套栈。

## 6 失败模式

| 报错 | 触发条件 | 调用方怎么处理 |
| --- | --- | --- |
| `SimulatorRestartError("the launch process exited with … while waiting for …")` | launcher 在探针期间退出（`xpra_start.py:363-375`） | 读 `logs/xpra-start.<map>.log`；UE5 没有引擎日志，`_engine_log_hint` 会明说（`:349-356`） |
| `SimulatorRestartError("timed out after 180s waiting for …")` | 探针超时（`xpra_start.py:376-380`） | 同上；消息里带探针描述与日志路径 |
| `SimulatorRestartError("another map is live (…); switch … instead of starting over")` | 端口在监听且属于另一张图（`xpra_start.py:272-275`） | 走 `POST /api/world` 的换图路径，不要重发 start |
| `SimulatorNotOwned("port N belongs to unmanaged PID …")` | 端口属主无法归属（`xpra_start.py:239-248`） | 人工确认；代码故意不杀 |
| `SimulatorRestartError("display :104 is still held by PID …")` | X 锁活着且 PID 还活着（`processes.py:232-236`） | 停掉占 display 的进程，或改 `simulator.display` |
| `SimulatorRestartError("the <family> stack still holds ports …")` | `xpra stop` + 整树 kill 后端口仍在（`xpra_start.py:463-466`） | 视为环境损坏；`holders()` 会再报一次属主 |
| `WorkerEventKind.ERROR` + 线程结束 | `idle_tick()` 抛异常（`worker.py:145-163`） | 控制器只能从事件队列里看到一条 ERROR 与一条 CLOSED；见 Qsim5 |
| `ChildProcessError("child sent a non-JSON line …")` | 子进程往协议 fd 写了非 JSON（`subprocess_worker.py:66-79`） | `_pump` 把它变成一条 ERROR 事件并继续读，不中断 |
| `ChildProcessError("the mission child is not running")` | `_stdin` 为 `None`（已 terminate 或从未 start） | 调用方必须当作"栈已死"，不要重试同一对象 |
| `ChildProcessError("… stopped accepting commands …")` | 写 stdin 时 `BrokenPipeError`（`subprocess_worker.py:231-241`） | 同上；日志里应已有子进程退出记录 |
| `SystemExit("this process is not a mission child: …")` | 手工启动 `mission_child`，环境变量缺失（`mission_child.py:79-85`） | 只能由父进程起，不要手跑 |

### 6.1 已知缺口

同样的"超时即放走"还有两处：`_release_orphan_display` 之前的 `_wait_display_released` 到点返回布尔值而不抛（`planner/simulator/xpra_start.py:555-561`）；`SubprocessMissionWorker._wait_for_closed` 到点后把已取到的事件放回队列就返回，不区分"等到 CLOSED"与"只是超时"（`planner/simulator/subprocess_worker.py:317-337`）。

`registry.build_session` 只在会话构造完成后调 `attach_arms`，自己不比对槽位（`planner/simulator/registry.py:67-84`）：比对发生在两个引擎的 `build_ports` 末尾，走 `SimulatorPorts.verify_slots`（`planner/simulator/ports.py:68-84`）。绕过 `build_ports` 手工拼 bundle 的会话不会被检查，今天没有这样的调用点。

## 7 测试位置

覆盖到的：

- `tests/test_xpra_start_driver.py`（argv 渲染、探针顺序、归属、停止降级）、`tests/test_family_router.py`（按标记与按地图两种路由、跨 family 换栈）。
- `tests/test_argv.py`、`tests/test_map_marker.py`。
- `tests/test_threaded_worker.py`、`tests/test_subprocess_worker.py`、`tests/test_mission_child.py`（重放测试，覆盖协议编解码、停止、关闭确认）。两个可靠性用例：`test_a_child_that_fills_stdout_still_gets_its_events_through`（真子进程先写 256 KiB stdout 再发协议）、`test_terminate_releases_a_worker_stuck_inside_a_simulator_call`（线程卡在 `idle_tick` 里被放弃）。
- `tests/test_architecture.py:120-135`（两个仿真器都声明了 `PORT_SLOTS` 且槽位名在 `SimulatorPorts` 里存在）、`tests/test_simulator_port_slots.py`（声明与实物逐项比对、缺槽位报错并点名、填了未声明槽位报错）、`tests/test_requirements.py`、`tests/test_layering.py`、`tests/test_layer_entries.py`。

没覆盖的空白：

- `processes.py` 的 `reclaim_stale_display`、`display_released`、`display_holder`、`terminate_verified_tree` 没有直接单测，全部经由 `test_xpra_start_driver.py` 的注入替身间接验证；`/proc` 与 `ss` 的真实分支没有一条测试。
- `registry.ports_for` 没有生产调用者，只有测试在调（见 Qsim7）。
- UE5 的 `build_ports` 只在构造期跑一次，没有测试起过真实 `ProjectAirSimMissionSession`：`tests/test_projectairsim_session.py` 用 `__new__` 与替身类，验的是它前面的构造顺序与 sink 转发。

## 8 与其他层的联动

| 方向 | 对象 | 形态 |
| --- | --- | --- |
| 上游 | `planner.assembly`（`ServerController`、`MissionResources`、`assemble_missions`、`task_finish_events`） | `mission_child.py:31-33`、`worker.py:14`、`registry.py:74` 直接 import；装配层通过 `BackendWorker` 协议驱动本层 |
| 上游 | `planner.config` | `MapCatalog`、`MapEntry`、`SimulatorSettings`、`family` 段（`argv.py:49-78`、`xpra_start.py:66-90`） |
| 下游 | `planner.simulators.<name>.runtime` | 只按 `RUNTIME_MODULES` 表用 `import_module` 取 `describe()`/`build_session()`（`registry.py:18-21`、`:59-69`） |
| 下游 | `planner.runtime` / `planner.domain` | `snapshot_from_payload`、`snapshot_to_payload`、`TrajectorySnapshot`（runtime）；`weather_catalog_from_payload`（domain） |
| 下游 | `planner.runtime` | `TrajectoryHub`（`telemetry_relay.py:40`） |
| 数据形态 | 子进程序列化 | 事件是每行一个 JSON 对象；`WEATHER` 回灌成 `WeatherCatalog`、`TRAJECTORY` 回灌成 `TrajectorySnapshot`，其余保持裸 JSON（`subprocess_worker.py:92-110`） |

## 9 疑问与争议

- Qsim1 层：simulator 结论：`SimulatorPorts.vehicle` 与 `.observation` 声明为非可选，但 `from_slots` 在槽位缺失时传 `None`，类型与运行时不符。依据：`planner/simulator/ports.py:45-46` 与 `:106-107`。影响：否。
- Qsim5 层：simulator 结论：`idle_tick()` 抛异常时 worker 线程直接 `return`，之后 `submit()` 仍会往队列里塞任务，永远不会被消费，也不会再报错。依据：`planner/simulator/worker.py:174-191`、`:49-50`。影响：是（assembly 的控制器需要看 CLOSED 判活）。
- Qsim6 层：simulator 结论：子进程侧的 `_handle` 里 `elif cmd == "stop"` 分支不可达，`stop` 在 stdin 线程已被 `continue` 掉。依据：`planner/simulator/mission_child.py:190-217` 与 `:270-273`。影响：否。
- Qsim7 层：simulator 结论：`registry.ports_for` 只有 `capabilities_for` 与测试在调，是单点抽象；同时 `capabilities_for` 反向 import 装配层，本层的"独立可测"要靠这条反向依赖成立。依据：`planner/simulator/registry.py:38-42`。影响：是（assembly 文档）。
- Qsim8 层：simulator 结论：子进程 family 收到 `home`/`activate`/`view` 只记一条 ERROR 就丢弃，控制器侧等不到 `ACTIVE`/`VIEW` 事件；这条路径只对 UE4 生效，而 UE4 的端口槽位本来就没有 `home`/`views`，两边靠能力位隐式对齐，没有任何校验。依据：`planner/simulator/mission_child.py:288-296`、`planner/simulators/airsim/runtime.py:17-23`。影响：是（assembly 与 simulators 文档）。
- Qsim9 层：simulator 结论：两个 family 共用 `xpra: 10010`，`running()` 只检查自己声明的端口集合，所以同时起两套栈不会被 `running()` 拦住，只会在 xpra 端口冲突时失败。依据：`config/simulators.yaml:20`、`:123`、`planner/simulator/xpra_start.py:276-277`。影响：否。
- Qsim10 层：simulator 结论：`xpra_start` 里有多处带引用温度的注释但缺少可执行证据链（例如 `_HIDE`、display 复用事故都写在注释里），这些常量没有配置文件入口也没有量测脚本。依据：`planner/simulator/processes.py:41-43`（`POLL_INTERVAL_S`、`KILL_WAIT_S` 注释写明"reference console 的调参默认值"）、`planner/simulator/subprocess_worker.py:44`。影响：否。
- Qsim11 层：simulator 结论：`RelayedTelemetry._forward` 用 0.4 s 合并窗口，但 `_sent_at` 只在真正发出时更新；`clear()`/`start()` 强制发送，所以一次任务的最后一个 `publish_*` 可能永远不发出去。依据：`planner/simulator/telemetry_relay.py:59-76`、`:43-50`。影响：是（runtime 的 TrajectoryHub 语义）。
- Qsim12 层：simulator 结论：`SubprocessMissionWorker.stop()` 在 `self._process is None` 时静默返回，但在"进程存在、stdin 已断"时抛 `ChildProcessError`；同一个"栈不可用"两种表现。依据：`planner/simulator/subprocess_worker.py:274-277`、`:263-272`。影响：否。
