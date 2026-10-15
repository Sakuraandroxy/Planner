"""Strict YAML value checks; never coerce booleans into safety limits."""
import math

from planner.errors import ConfigurationError


def number(value, name: str, *, positive: bool = True) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{name} must be a number")
    try:
        valid = math.isfinite(value) and (value > 0 if positive else value >= 0)
    except OverflowError:
        valid = False
    if not valid:
        raise ConfigurationError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return float(value)


def integer(value, name: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ConfigurationError(f"{name} must be an integer between {minimum} and {maximum}")
    return value


def mapping(value, name: str, allowed: set[str] | None = None) -> dict:
    if not isinstance(value, dict):
        raise ConfigurationError(f"{name} must be a mapping")
    if allowed is not None and set(value) - allowed:
        raise ConfigurationError(f"unknown {name} setting")
    return value


def string(value, name: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str) or (not allow_empty and not value.strip()):
        raise ConfigurationError(f"{name} must be a {'string' if allow_empty else 'non-empty string'}")
    return value
