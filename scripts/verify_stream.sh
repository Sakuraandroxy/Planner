#!/usr/bin/env bash
# 验收一条重构流的工作区：全量测试、提交信息、冲突标记。
#
# 用法：scripts/verify_stream.sh <工作区路径>
#
# 三步依次执行，任一步失败继续跑后面的步骤，最后列出全部失败项并退出 1。
# 主干取 origin/HEAD，没有远端时取 main；检查范围是 主干..HEAD。
# 解释器默认 ~/miniconda3/envs/vln/bin/python，可用 PYTHON 覆盖。

set -uo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CHECK_COMMIT_MSG="$SCRIPT_DIR/check_commit_msg.py"
PYTHON=${PYTHON:-$HOME/miniconda3/envs/vln/bin/python}

if [ $# -ne 1 ]; then
    echo "用法：$0 <工作区路径>" >&2
    exit 2
fi

if ! WORKSPACE=$(cd "$1" 2>/dev/null && pwd); then
    echo "工作区不存在：$1" >&2
    exit 2
fi

git_w() {
    git -C "$WORKSPACE" "$@"
}

if ! git_w rev-parse --git-dir >/dev/null 2>&1; then
    echo "不是 git 工作区：$WORKSPACE" >&2
    exit 2
fi

TRUNK=""
for candidate in origin/HEAD main; do
    if git_w rev-parse --verify --quiet "$candidate" >/dev/null 2>&1; then
        TRUNK=$(git_w rev-parse --abbrev-ref "$candidate")
        break
    fi
done

BRANCH=$(git_w rev-parse --abbrev-ref HEAD)
echo "工作区：$WORKSPACE"
echo "分支：$BRANCH"

FAILED=()

# 第一步：全量测试。
echo "[1/3] 全量测试"
if (cd "$WORKSPACE" && "$PYTHON" -m pytest -q); then
    echo "      通过"
else
    echo "      失败" >&2
    FAILED+=("全量测试")
fi

# 第二步：主干以来每条提交的信息。
echo "[2/3] 提交信息"
if [ -z "$TRUNK" ]; then
    echo "      失败：找不到主干（origin/HEAD 与 main 都不存在）" >&2
    FAILED+=("提交信息")
else
    echo "      范围：$TRUNK..HEAD"
    checked=0
    bad=0
    for commit in $(git_w rev-list --no-merges "$TRUNK..HEAD"); do
        message_file=$(mktemp)
        git_w log -1 --format=%B "$commit" >"$message_file"
        checked=$((checked + 1))
        if ! output=$("$PYTHON" "$CHECK_COMMIT_MSG" "$message_file" 2>&1); then
            bad=$((bad + 1))
            echo "      失败：$(git_w log -1 --format='%h %s' "$commit")" >&2
            printf '%s\n' "$output" | sed 's/^/        /' >&2
        fi
        rm -f "$message_file"
    done
    if [ "$bad" -eq 0 ]; then
        echo "      通过：$checked 条"
    else
        FAILED+=("提交信息")
    fi
fi

# 第三步：冲突标记。只看 <<<<<<< 与 >>>>>>> 行首标记；======= 会和标题下划线混淆。
echo "[3/3] 冲突标记"
markers=$(git_w grep -n -E '^(<<<<<<< |>>>>>>> )' -- . 2>&1)
grep_status=$?
if [ "$grep_status" -eq 0 ]; then
    echo "      失败：" >&2
    printf '%s\n' "$markers" | sed 's/^/        /' >&2
    FAILED+=("冲突标记")
elif [ "$grep_status" -eq 1 ]; then
    echo "      通过"
else
    echo "      失败：git grep 出错" >&2
    printf '%s\n' "$markers" | sed 's/^/        /' >&2
    FAILED+=("冲突标记")
fi

if [ ${#FAILED[@]} -gt 0 ]; then
    printf '失败项：%s\n' "${FAILED[*]}" >&2
    exit 1
fi

echo "全部通过"
exit 0
