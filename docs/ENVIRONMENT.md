# 环境配置（本机实测）

分支 `feat/ue4-dual-runtime`，机器：无图形界面的 Ubuntu，conda 26.9.4。

本文只写实测通过的命令与踩到的坑。与 `docs/UE4_BRINGUP.md` 第 1 节冲突时以本文为准，
那里有两条命令照抄会失败。所有版本号都是这台机器上的真实值，不是抄来的期望值。

## 1 两个环境，硬约束要求分开

| 环境 | Python | 谁用 | 装了什么客户端 |
|---|---|---|---|
| `vln` | 3.11.18 | HTTP 主进程 + UE5 运行时 | `projectairsim` 1.0.2 |
| `vln-ue4` | 3.11.18 | UE4 运行时子进程 | `airsim` 1.8.1 |

分开的理由不是"依赖可能冲突"这种猜测，是一个确定的模块名撞车：

- `airsim` → `msgpack-rpc-python` 0.4.1 → `msgpack-python` 0.5.6，提供模块 `msgpack`（版本 0.5.6）
- `projectairsim` 1.0.2 → `msgpack>=1.0.5`，同一个模块名 `msgpack`（版本 1.2.4）

两个发行版往 `site-packages/msgpack/` 写同一批文件。装进同一个解释器，后装的覆盖前一个，
另一边在 import 成功之后才炸。`airsim` 还额外拖进 `tornado` 4.5.3。

主进程永远不 import 两族客户端，这条由 `tests/test_architecture.py` 守着，所以 Web 侧
只需要 `vln`。`planner/simulators/airsim/` 里的 `import airsim` 全部写在函数体内，是惰性的。

### 1.1 撞车的具体后果（已实测）

`msgpackrpc/transport/tcp.py:12-13` 这样构造打包器：

```python
self._packer = msgpack.Packer(encoding=encodings[0], default=lambda x: x.to_msgpack())
self._unpacker = msgpack.Unpacker(encoding=encodings[1])
```

`encoding` 参数在 msgpack 1.0 里被删掉了。同一个调用在两个环境里的结果：

| 环境 | `import airsim` | `MultirotorClient()` | `client.ping()` |
|---|---|---|---|
| `vln-ue4`（msgpack 0.5.6） | 成功 | 成功 | `TransportError: Retry connection over the limit`（ECONNREFUSED） |
| 混装（msgpack 1.2.4 抢占模块名） | 成功 | 成功 | `TypeError: __init__() got an unexpected keyword argument 'encoding'` |

两边前两步一模一样，只在真正发包时才分岔。`vln-ue4` 的失败是对的：序列化链路通了，
只是没起 UE4 服务。混装的失败是死的：一行 RPC 都发不出去。

### 1.2 由此得出：`AGENT_PROMPT.md` 第 1 步那条共存检查给出假绿灯

原文用
`python -c "import msgpackrpc, pynng, msgpack; print('coexist ok')"` 判共存，
实测打印 `coexist ok`，但这三个 import 全都不碰 msgpack 的打包 API，所以它们成功不代表能用。
同一环境里 `pip list` 显示 `msgpack 1.2.4` 与 `msgpack-python 0.5.6` 并存，
`msgpack.version` 解析到 `(1, 2, 4)`。判共存要用上表里那条 `client.ping()`，
看它是 `TransportError`（环境正确）还是 `TypeError`（已被覆盖）。

## 2 `vln`：主环境 + UE5

```bash
conda create -n vln python=3.11 -y
conda activate vln
pip install numpy pyyaml httpx fastapi "uvicorn[standard]" pydantic \
  opencv-python-headless pytest Pillow
pip install --no-deps projectairsim==1.0.2
pip install pynng "msgpack>=1.0.5" matplotlib commentjson "jsonschema>=4.4.0" \
  inputs pqdict cryptography pykml junit-xml jsonlines Shapely azure-storage-blob
```

四个坑：

**2.1 `projectairsim` 在 PyPI 上。** `pyproject.toml:15` 的注释写着它 "not published on PyPI"，
过时了。PyPI 上有 `0.2.0 / 0.3.0 / 1.0.1 / 1.0.2 / 1.1.0`，`requires_python >=3.9,<4`。
本仓库按注释的口径装 **1.0.2**。1.1.0 是最新稳定版，依赖集合与 1.0.2 完全相同，
差别的只是 API，升级前要先过一遍 UE5 侧的测试。

**2.2 `Pillow` 是主环境的硬需求，不只是 UE4 的。** `planner/simulators/airsim/observation_source.py:13`
在模块顶层 `from PIL import Image`。不装它会看到 2 个测试失败，失败点看起来和 UE4 无关。

**2.3 装 `projectairsim` 必须 `--no-deps`。** 它声明依赖 `opencv-python`（GUI 版），
会把第二份 `cv2/` 写进 `site-packages`，和仓库声明的 `opencv-python-headless` 抢同一个包目录。
所以本体用 `--no-deps` 装，基础依赖手工补齐（上面第三条命令就是它的 14 个基础依赖，
逐个从 wheel 元数据的 `requires_dist` 抄出来，只去掉 `opencv-python`）。

**2.4 副作用：`pip check` 有噪音。**

```
projectairsim 1.0.2 requires opencv-python, which is not installed.
```

预期如此，因为满足它的是 headless 变体。不要为此补装 `opencv-python`。

## 3 `vln-ue4`：UE4 客户端

```bash
conda create -n vln-ue4 python=3.11 -y
conda activate vln-ue4
pip install numpy requests Pillow pyyaml pydantic \
  opencv-contrib-python-headless "msgpack-rpc-python>=0.4.1"
pip install --no-deps --no-build-isolation airsim==1.8.1
```

验证：

```bash
python -c "import airsim, msgpackrpc, cv2; print(airsim.__version__, cv2.__version__)"
# 1.8.1 5.0.0
PYTHONPATH=. python -c "
import importlib, pkgutil, planner.simulators.airsim as p
[importlib.import_module('planner.simulators.airsim.'+m.name) for m in pkgutil.iter_modules(p.__path__)]
print('14 个适配层模块全部 import 成功')"
```

四个坑：

**3.1 BRINGUP 第 1 节那条命令照抄会失败。** 原文是
`pip install "airsim>=1.8.1" "msgpack-rpc-python>=0.4.1"`，报错
`Getting requirements to build wheel did not run successfully`。
原因：`airsim` 只有 sdist，它的 `setup.py` 开头 `from airsim import __version__`，
这会 import `client.py` → `import numpy`，而 PEP 517 的构建隔离环境里没有 numpy。
**装 `airsim` 之前先把 numpy 装好**，再加 `--no-deps --no-build-isolation`。

**3.2 不需要为老依赖降级 Python。** 我原先预计 `msgpack-python` 0.5.6 和 `tornado` 4.5.3
在 3.11 上编不过，实测**三个 sdist 全部构建成功**，3.11 可用，不必回到 3.9/3.10。

**3.3 但 `msgpack-python` 只有纯 Python 实现在跑。** 它的 wheel 只有 10 KB，
`msgpack._cmsgpack` 不存在，`packb/unpackb` 走 fallback。功能正确，性能比 C 扩展差，
而 AirSim 每次取图都要过一层 msgpack 解包。要压延迟时这是一个已知开销点，
换 `msgpack` 1.x 又会破坏 `msgpackrpc`，别在这上面动依赖。

**3.4 `airsim` 声明的是 `opencv-contrib-python`（GUI 版）。** 用 `-headless` 变体替代，
配合 `--no-deps`。副作用同样是 `pip check` 噪音：

```
airsim 1.8.1 requires opencv-contrib-python, which is not installed.
```

## 4 实测版本号：不要预防性降级

这套组合下全量测试通过，没有一处需要退回旧版本：

| 包 | 版本 | |
|---|---|---|
| numpy | 2.4.6 | 2.x 大版本，没踩 |
| opencv | 5.0.0.93 | cv2 5.0 大版本，没踩 |
| pydantic | 2.14.3 | |
| Pillow | 13.0.0 | |
| pytest | 9.2.0 | |
| PyYAML | 6.0.3 | |

`commentjson` 会拖进 `lark-parser` 0.7.8，跑测试时打印
`DeprecationWarning: module 'sre_parse' is deprecated`。无害，忽略。

## 5 自检：命令与**真实**数字

```bash
conda activate vln
cd /roxy/kedior/Planner
python -m compileall -q planner scripts tests   # 期望：退出码 0
python -m pytest -q                             # 期望：851 passed
python scripts/probe_ue4_env.py
```

**`851 passed` 才是这个分支的全量结果**，与 `docs/AGENT_PROMPT.md` 第 0 步的基线一致。
`docs/UE4_BRINGUP.md:102` 还写着 `847 passed`，那个数字过时了。

缺包时的失败长这样，可以用来判断自己漏了哪一步：

| 缺 | 现象 |
|---|---|
| `projectairsim` | 7 个测试文件 collection 直接 error：`test_projectairsim_*`（5 个）、`test_runtime_catalog.py`、`test_streaming_views.py` |
| `Pillow` | `test_architecture` 2 条失败，报 `No module named 'PIL'` |
| `opencv-python-headless` | `test_architecture::test_main_process_never_loads_a_simulator_client` 失败 |

**`probe_ue4_env.py` 在 `vln` 里必然报 1 个 FAIL**，内容是 `airsim packages: missing ['airsim', 'msgpackrpc']`。
这是设计使然——UE4 客户端按第 1 节装在另一个环境。实测其余为 `25 ok, 1 warn`。
**不要为了让它变绿把 airsim 装进 `vln`**，那会触发 `msgpack` 撞车。
把它跑到 `0 failed` 的唯一正确做法不在这台机器的文档里，需要改脚本让它按环境分工来判。

那条 warn 是 `ApiServerPort: absent, AirSim will default to 41451`。
`/data/airsim/settings.json` 没有这个字段，AirSim 用默认 41451，与
`config/simulators.yaml` 的 `airsim.ports.rpc` 一致，不用改。

## 6 仿真器：只用打包好的二进制

**规则：不碰 UE 引擎源码，不编译，不开编辑器。** 只用已经打包好的 shipping / NoEditor 产物。
`/roxy/kedior/ue4/`（261 GB，内含 `UnrealEngine-4.27.2-release` 与 `AirSim` 源码）只是它们的来源，
不要在那里执行任何构建命令。

早期交接说法是"UE4 应用还没下载下来、上一个会话没记下路径"。这个前提在本机不成立，
两边都不缺：

| 族 | 打包产物 |
|---|---|
| AirSim (UE4) | `/data/airsim/<地图>/LinuxNoEditor/`，7 张：`Blocks`、`AirSimNH`、`AbandonedPark`、`MSBuild2018`、`TrapCam`、`Shanghai`、`Campus`（25 GB） |
| AirSim 配置 | `/data/airsim/settings.json`，`SimMode: Multirotor`，载具 `Drone_1`，相机 `front_center` + `down_center` |
| ProjectAirSim (UE5) | `/roxy/sakura/XV3DGS-airsim-vln/sim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping`，另有 `Neighborhood` |

**坑 6.1：`config/maps.yaml` 少了 `campus`。** 6 张图配好了路径，`/data/airsim/Campus` 的二进制存在但没进目录。
`Campus` 正是本机自己从 `urbanScene` 打包出来的那张。

**坑 6.2：`scripts/launch_projectairsim_xpra.sh` 把 UE5 路径写死成 `$PROJECT_DIR/sim/`。**
`PROJECT_DIR` 由脚本自身位置推导，即 `/roxy/kedior/Planner`，而本仓库没有 `sim/` 目录。
直接跑会报 `simulator binary not found`，且没有任何环境变量能覆盖它。
两个选择：建软链 `Planner/sim -> /roxy/sakura/XV3DGS-airsim-vln/sim`，或改脚本支持环境变量。
**未处理**，需要先定方向。

## 7 图形与 GPU

无桌面会话，`DISPLAY` 为空。已装：`/usr/bin/xpra`、`/usr/bin/Xvfb`、`xdpyinfo`。
`libGL.so.1` 存在，所以链接了 GUI 库的 wheel 也能 import——但依赖仍保持 headless，见 2.3 与 3.4。

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
# driver 595.99.02，可见 3× RTX 3080 Ti (12 GB)
```

**有一张卡是坏的**：`Unable to determine the device handle for GPU3: 0000:68:00.0: Unknown Error`。
UE4 渲染和 torch 都要显式绑定到健康的卡上（`CUDA_VISIBLE_DEVICES` 或 UE4 侧的 `-GraphicsAdapter`），
不要依赖默认选卡。

**torch 目前两个环境都没装**，`pyproject.toml` 也不要求它。真要装本地 VLN 模型时，
别用 `pip install torch` 加 conda 的默认解析——那样装到的是 CPU 版，测试照样全过，
只有现场推理才会慢。装之前先 `nvidia-smi` 确认驱动（本机 595.99.02），再从 PyTorch
对应的 CUDA 索引装，装完用 `torch.cuda.is_available()` 验证。

## 8 环境装好后当场能答的一个问题

`projectairsim` 一装上，`docs/UE4_BRINGUP.md` 第 5 节那个"必须上机才能确认"的问题就有了答案：

```
Drone.set_camera_pose(camera_id: str, pose: Transform, wait_for_pose_update: bool = True) -> bool
Drone.reset_camera_pose(camera_id: str, wait_for_pose_update: bool = True) -> bool
Drone.camera_look_at_object(camera_id: str, object_name: str, ...) -> bool
World: 另有 spawn_object / set_object_pose / get_object_pose 等
```

`set_camera_pose` 存在，写的是载具相机的相对位姿。`AGENT_PROMPT.md` 第 1 步预设的判读规则是
"drone 里出现相机位姿写入 → UE5 可以做俯视，会把 `PORT_SLOTS` 加一项"，条件已满足。
这与 `docs/UE4_REMAINING_WORK.md` 第 6 节"UE5 不做俯视"那条边界的前提相反（那条假设 UE5 没有
这个 API）。改代码属于实现问题，此处只记录环境侧测得的事实；结论已同步进
`UE4_REMAINING_WORK.md` 第 7 节。
