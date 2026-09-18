class PlannerError(RuntimeError):
    """Base error for planner failures."""


class ConfigurationError(PlannerError):
    pass


class ProtocolError(PlannerError):
    pass


class ExecutionError(PlannerError):
    pass

