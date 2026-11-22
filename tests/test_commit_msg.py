import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "check_commit_msg.py"


def _check(tmp_path, message):
    path = tmp_path / "COMMIT_EDITMSG"
    path.write_text(message, encoding="utf-8")
    return subprocess.run(
        [sys.executable, str(SCRIPT), str(path)],
        capture_output=True,
        text=True,
    )


def test_accepts_valid_title(tmp_path):
    result = _check(tmp_path, "test(lint): 新增提交信息检查脚本\n")
    assert result.returncode == 0
    assert result.stdout == ""
    assert result.stderr == ""


def test_rejects_description_without_verb(tmp_path):
    result = _check(tmp_path, "fix(auth): 登录跳转以 session 状态为准\n")
    assert result.returncode == 1
    assert "描述未以动词开头" in result.stderr


def test_rejects_long_description(tmp_path):
    description = "新增超时时间与重试次数的默认取值配置项并在部署脚本中同步校验上限"
    assert len(description) > 30
    result = _check(tmp_path, f"fix(config): {description}\n")
    assert result.returncode == 1
    assert "超过 30 字上限" in result.stderr


def test_rejects_unknown_type(tmp_path):
    result = _check(tmp_path, "feature: 新增超时配置项\n")
    assert result.returncode == 1
    assert "不在允许列表" in result.stderr


def test_rejects_banned_word_in_body(tmp_path):
    result = _check(tmp_path, "docs: 更新部署说明\n\n补充了端口字段和启动顺序。\n")
    assert result.returncode == 1
    assert "禁用词「和」" in result.stderr


def test_rejects_co_author_trailer(tmp_path):
    message = "docs: 更新部署说明\n\nCo-Authored-By: someone <someone@example.com>\n"
    result = _check(tmp_path, message)
    assert result.returncode == 1
    assert "Co-Authored-By" in result.stderr
