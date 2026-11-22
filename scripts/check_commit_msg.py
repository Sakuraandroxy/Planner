"""Check one commit message against this repository's commit conventions.

Four rules, each reported separately:

1. The title reads ``<type>(<scope>): <描述>`` or ``<type>: <描述>``，type 属于
   ``TYPES``。
2. 描述以动词开头，不超过 30 字。
3. 正文不含「和」「以及」「同时」。
4. 没有 ``Co-Authored-By`` 尾注。

标题与正文分开判：标题只看格式、type、动词、长度，正文只看禁用词与尾注。

    python scripts/check_commit_msg.py [path]

不传路径时读 ``.git/COMMIT_EDITMSG``。有失败项时打到 stderr 并退出 1，
全部通过时不输出，退出 0。以 ``#`` 开头的行是 git 写的注释，不参与检查。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

DEFAULT_MESSAGE_PATH = Path(".git/COMMIT_EDITMSG")

# 顺序与项目 git skill 一致，报错时原样列出，方便对照。
TYPES = (
    "feat",
    "fix",
    "docs",
    "style",
    "refactor",
    "perf",
    "test",
    "build",
    "ci",
    "chore",
    "revert",
)

# 「动词开头」用白名单判定。黑名单列不全同义动词，会把不合规的标题放过去；
# 白名单的失败模式是挡住一个新动词，报错里带着允许列表，加一行就能修。
VERBS = (
    "新增",
    "添加",
    "增加",
    "实现",
    "修改",
    "修复",
    "修正",
    "删除",
    "移除",
    "去掉",
    "重构",
    "重写",
    "拆分",
    "合并",
    "统一",
    "提取",
    "迁移",
    "替换",
    "调整",
    "补充",
    "补齐",
    "更新",
    "优化",
    "精简",
    "清理",
    "升级",
    "降低",
    "提升",
    "记录",
    "改名",
    "重命名",
    "引入",
    "接入",
    "对齐",
    "解耦",
    "废弃",
    "回退",
    "恢复",
    "支持",
    "核对",
    "对账",
    "重审",
    "复核",
    "审阅",
    "评审",
    "搬迁",
    "移动",
    "移出",
    "外置",
    "下沉",
    "上移",
    "收敛",
    "扩展",
    "扩充",
    "收口",
    "收紧",
    "放宽",
    "限定",
    "登记",
    "标注",
    "转换",
    "跟随",
    "归位",
    "划分",
    "重算",
    "重挂",
    "重排",
)

BANNED_WORDS = ("和", "以及", "同时")

MAX_DESCRIPTION = 30

TITLE_RE = re.compile(
    r"^(?P<type>[a-z]+)(?:\((?P<scope>[a-z][a-z0-9-]*)\))?: (?P<description>\S.*)$"
)
TRAILER_RE = re.compile(r"^co-authored-by\s*:", re.IGNORECASE)


def _check_title(title: str) -> list[str]:
    match = TITLE_RE.match(title)
    if match is None:
        return [f"标题：不符合 <type>(<scope>): <描述> 格式：{title}"]

    errors = []
    kind = match.group("type")
    if kind not in TYPES:
        errors.append(f"标题：type「{kind}」不在允许列表 {' '.join(TYPES)}")

    description = match.group("description")
    if len(description) > MAX_DESCRIPTION:
        errors.append(
            f"标题：描述 {len(description)} 字，超过 {MAX_DESCRIPTION} 字上限"
        )
    if not description.startswith(VERBS):
        errors.append(f"标题：描述未以动词开头：{description}")
    if "，" in description or "," in description:
        errors.append("标题：描述里只能写一件事，不要用逗号并列")
    return errors


def check_message(text: str) -> list[str]:
    """Return every violation in ``text``; an empty list means it passes."""
    title = None
    body = []
    for number, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if title is None:
            title = line
        else:
            body.append((number, line))

    if title is None:
        return ["标题：提交信息为空"]

    errors = _check_title(title)
    for number, line in body:
        for word in BANNED_WORDS:
            if word in line:
                errors.append(f"正文第 {number} 行：出现禁用词「{word}」")
        if TRAILER_RE.match(line):
            errors.append(f"正文第 {number} 行：禁止 Co-Authored-By 尾注")
    return errors


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    path = Path(args[0]) if args else DEFAULT_MESSAGE_PATH
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        print(f"读不到提交信息文件 {path}：{exc}", file=sys.stderr)
        return 1

    errors = check_message(text)
    if not errors:
        return 0

    print(f"提交信息检查失败（{len(errors)} 项）：", file=sys.stderr)
    for error in errors:
        print(f"  {error}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
