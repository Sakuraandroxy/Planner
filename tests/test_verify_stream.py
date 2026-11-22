import os
import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "verify_stream.sh"

VALID_MESSAGE = "test(lint): 新增验收脚本用例"
PASSING_TEST = "def test_ok():\n    assert True\n"
FAILING_TEST = "def test_broken():\n    assert False\n"


def _git(repo, *args):
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        capture_output=True,
        text=True,
        check=True,
    )


def _init_repo(tmp_path):
    repo = tmp_path / "ws"
    repo.mkdir()
    _git(repo, "init", "-q", "-b", "main")
    _git(repo, "config", "user.email", "verify@example.com")
    _git(repo, "config", "user.name", "verify")
    return repo


def _commit(repo, files, message):
    for name, text in files.items():
        path = repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        _git(repo, "add", name)
    _git(repo, "commit", "-q", "-m", message)


def _verify(workspace):
    return subprocess.run(
        ["bash", str(SCRIPT), str(workspace)],
        capture_output=True,
        text=True,
        env={**os.environ, "PYTHON": sys.executable},
    )


def _repo_with_stream(tmp_path, stream_files, stream_message):
    repo = _init_repo(tmp_path)
    _commit(repo, {"tests/test_ok.py": PASSING_TEST}, "test(lint): 新增验收脚本用例")
    _git(repo, "checkout", "-q", "-b", "impl/stream")
    _commit(repo, stream_files, stream_message)
    return repo


def test_passes_when_all_checks_succeed(tmp_path):
    repo = _repo_with_stream(
        tmp_path, {"tests/test_more.py": PASSING_TEST}, VALID_MESSAGE
    )

    result = _verify(repo)

    assert result.returncode == 0, result.stderr
    assert "全部通过" in result.stdout


def test_fails_when_tests_fail(tmp_path):
    repo = _repo_with_stream(
        tmp_path, {"tests/test_more.py": FAILING_TEST}, VALID_MESSAGE
    )

    result = _verify(repo)

    assert result.returncode == 1
    assert "全量测试" in result.stderr
    assert "提交信息" not in result.stderr


def test_fails_when_commit_message_violates_convention(tmp_path):
    repo = _repo_with_stream(
        tmp_path,
        {"tests/test_more.py": PASSING_TEST},
        "test(lint): 登录跳转以 session 状态为准",
    )

    result = _verify(repo)

    assert result.returncode == 1
    assert "提交信息" in result.stderr
    assert "描述未以动词开头" in result.stderr
    assert "全量测试" not in result.stderr


def test_fails_when_conflict_marker_present(tmp_path):
    repo = _repo_with_stream(
        tmp_path,
        {"notes.md": "头部\n<<<<<<< HEAD\n我方\n=======\n对方\n>>>>>>> impl/other\n"},
        VALID_MESSAGE,
    )

    result = _verify(repo)

    assert result.returncode == 1
    assert "冲突标记" in result.stderr
    assert "notes.md" in result.stderr
