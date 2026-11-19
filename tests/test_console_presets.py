"""预设指令下拉框的数据源是 console/presets.json，后端不需要新路由。

``/console`` 整个目录挂在静态文件伺服上，加一条预设指令只改这个文件、刷新页面。这
里锁住三件事：文件的形状是字符串数组、静态路由真的把它伺服出来、选中之后文本落进
指令输入框。前两条错一条，下拉框就永远只有占位项。
"""

import json
import re
from pathlib import Path

from test_server_api import FakeController, _client

CONSOLE = Path(__file__).resolve().parents[1] / "console"
SRC = CONSOLE / "src"
#: classic scripts in load order, app.js binds events last.
SCRIPTS = ("core.js", "world.js", "settings.js", "flight.js", "app.js")


def _presets():
    return json.loads((CONSOLE / "presets.json").read_text(encoding="utf-8"))


def _sources():
    return "\n".join(
        (SRC / name).read_text(encoding="utf-8") for name in SCRIPTS
    )


def test_the_preset_file_is_a_list_of_instructions():
    items = _presets()
    assert items, "预设文件为空，下拉框没有可选项"
    assert all(
        isinstance(item, str) and item.strip() for item in items
    ), "每一项都得是一条能直接提交的指令文本"


def test_the_static_console_route_serves_the_preset_file():
    with _client(FakeController()) as client:
        response = client.get("/console/presets.json")
    assert response.status_code == 200
    assert response.json() == _presets()


def test_a_choice_fills_the_instruction_box():
    html = (CONSOLE / "index.html").read_text(encoding="utf-8")
    assert 'id="instruction-preset"' in html
    app = _sources()
    assert 'request("/console/presets.json")' in app
    assert '$("instruction-preset").addEventListener' in app
    body = re.search(
        r"function applyPresetCommand\(\).*?\n}", app, flags=re.S
    ).group(0)
    assert '$("instruction").value = text' in body
