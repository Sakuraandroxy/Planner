"""提示词模板的守门检查：文件形态、引用完整性、禁用句式、语义契约。

四条规则对应 `docs/PROMPT_STYLE.md` 的硬规则与术语表。改模板时这四条会先红，
比让模型在生产里跑歪便宜得多。
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
ADAPTERS = ROOT / "planner/adapters"
TEMPLATES = ADAPTERS / "prompts/templates"

#: 解释性否定：整句在解释证据强弱或状态，不产生新动作，一律不许出现在模板里。
BANNED_PHRASES = (
    "仅属于",
    "不等于",
    "不能直接",
    "不必重新",
    "仅为",
    "直接证明",
    "does not mean",
    "is not the same as",
)


def _template_files() -> list[Path]:
    return sorted(TEMPLATES.rglob("*.j2"))


def _template_texts() -> dict[str, str]:
    return {p.name: p.read_text(encoding="utf-8") for p in _template_files()}


def _python_sources() -> str:
    parts = []
    for path in sorted(ADAPTERS.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        parts.append(path.read_text(encoding="utf-8"))
    return "\n".join(parts)


def _included_names() -> set[str]:
    """模板之间通过 ``{% include %}`` 引用的片段名，算作「被引用」。"""
    names: set[str] = set()
    for path in _template_files():
        for line in path.read_text(encoding="utf-8").splitlines():
            match = re.search(r'\{%\s*include\s+"([^"]+)"\s*%\}', line)
            if match:
                names.add(Path(match.group(1)).stem)
    return names


def test_no_fragment_markdown_left():
    """片段标题式 md 已全部并入 Jinja2 模板。"""
    leftovers = [p for p in ADAPTERS.rglob("*.md") if "__pycache__" not in p.parts]
    assert not leftovers, f"仍有多余的 md 片段：{leftovers}"


def test_every_template_is_rendered_somewhere():
    """模板目录里不留孤儿文件。"""
    sources = _python_sources()
    included = _included_names() | {"selftest"}
    orphans = [
        path.name
        for path in _template_files()
        # 片段型模板由宿主 {% include %} 引用，顶层模板由 render_prompt 的名字引用
        if path.stem not in sources and path.stem not in included and path.name not in sources
    ]
    assert not orphans, f"这些模板没有任何代码引用：{orphans}"


@pytest.mark.parametrize("phrase", BANNED_PHRASES)
def test_no_explanatory_negation(phrase):
    """解释性否定不产生新动作，模板里不允许出现。"""
    hits = [
        f"{path.name}: {line.strip()}"
        for path in _template_files()
        for line in path.read_text(encoding="utf-8").splitlines()
        if phrase in line
    ]
    assert not hits, f"模板里有解释性否定「{phrase}」：{hits}"


def test_templates_are_not_single_liners():
    """一句一个文件是上一版的问题。只允许 include 片段单行，顶层模板必须有结构。"""
    included = _included_names()
    thin = [
        p.name
        for p in _template_files()
        if len(p.read_text(encoding="utf-8").splitlines()) <= 1 and p.stem not in included
    ]
    assert not thin, f"这些模板只有一行，应当并入宿主模板：{thin}"


def test_target_detection_keeps_its_output_contract():
    """检测提示词必须保留输出格式与坐标口径，这两条是契约。"""
    text = _template_texts()
    joined = "\n".join(text[name] for name in text if "detection" in name)
    assert '"bbox"' in joined and '"surface_points"' in joined
    assert re.search(r"normalized to \[0,1\]|归一化", joined), "缺少坐标归一化说明"
    assert re.search(r"up to 5|最多\s*5|up to five", joined, re.IGNORECASE), "缺少表面点上限"


def test_trajectory_keeps_point_count_and_units():
    """轨迹提示词必须保留每轮点数与单位。"""
    text = _template_texts()
    joined = "\n".join(
        text[name] for name in text if "trajectory" in name or name.startswith("trajectory_output")
    )
    assert re.search(r"exactly 5 NEW actions|five (new )?(actions|points)|五个", joined), "缺少每轮五个点的约束"
    assert re.search(r"metres|meters|米", joined), "缺少长度单位"


def test_ground_and_aerial_templates_differ_only_in_motion():
    """地面与无人机模板共享结构，差异只应落在运动语义上。"""
    text = _template_texts()
    ground = text["trajectory_ground.j2"]
    aerial = text["trajectory_aerial.j2"]
    assert ground != aerial
    assert "dz = 0" in ground, "地面模板缺少 dz = 0 的竖直约束"
    assert "dz = 0" not in aerial, "无人机模板不应禁用竖直运动"
