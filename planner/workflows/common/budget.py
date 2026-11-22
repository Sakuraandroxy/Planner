"""任务层的执行预算与模型调用计数辅助，任务错误类型由调用方注入。"""
import time
from collections.abc import Callable

from planner.domain import ExecutionBudget
from planner.errors import BudgetExceeded, ModelCallTimeout


def require_time(budget: ExecutionBudget, clock=time.monotonic) -> None:
    """Raise ``BudgetExceeded`` once the stage budget has run out."""
    if budget.remaining(clock()) <= 0:
        raise BudgetExceeded("task deadline exhausted")


def model_call(
    operation,
    limits,
    *,
    budget: ExecutionBudget,
    model_calls=None,
    checkpoint: Callable[[], None] | None = None,
    on_planning_call: Callable[[], None] | None = None,
    on_error: Callable[[Exception], Exception],
    is_planning: bool = False,
    validate: Callable[[object], None] | None = None,
    clock=time.monotonic,
):
    """Run one model operation under the budget, retrying up to the limit.

    ``on_error`` maps a final failure to the task's own exception type; the
    helper itself knows only the shared budget and timeout errors.
    """
    for attempt in range(limits.model_retry_count + 1):
        require_time(budget, clock)
        if is_planning:
            on_planning_call()
        try:
            if model_calls is None:
                value = operation()
            else:
                timeout = min(limits.model_request_timeout_s, budget.remaining(clock()))
                value = model_calls.call(operation, timeout, checkpoint)
            require_time(budget, clock)
            if validate is not None:
                validate(value)
            return value
        except (BudgetExceeded, ModelCallTimeout):
            raise
        except Exception as exc:
            if attempt >= limits.model_retry_count:
                raise on_error(exc) from exc
    raise AssertionError("unreachable retry state")
