"""Request bodies for the console routes. Every model forbids unknown keys."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, field_validator

#: One process driver for every family: planner/simulator/xpra_start.py renders the
#: argv the family declares in config/simulators.yaml. ``SimulatorFamily`` rejects
#: any other driver when the config is parsed, so the HTTP layer does not guess
#: again and nothing here branches on a simulator name.

ViewMode = Literal["rover_front", "rover_chase", "drone_front", "drone_chase"]


class TaskRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    instruction: str
    # Route the task to one of the session's vehicles ("rover" or "drone").
    vehicle: str | None = None

    @field_validator("instruction")
    @classmethod
    def nonempty(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("instruction must be non-empty text")
        return value.strip()

    @field_validator("vehicle")
    @classmethod
    def optional_vehicle(cls, value: str | None) -> str | None:
        if value is None:
            return None
        text = value.strip()
        return text or None


class WorldRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    map: str

    @field_validator("map")
    @classmethod
    def nonempty(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("map must be non-empty text")
        return value.strip()


class ProfileRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    profile: str

    @field_validator("profile")
    @classmethod
    def nonempty(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("profile must be non-empty text")
        return value.strip()


class ViewRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mode: ViewMode


class HomeRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    vehicle: str | None = None

    @field_validator("vehicle")
    @classmethod
    def optional_vehicle(cls, value: str | None) -> str | None:
        if value is None:
            return None
        text = value.strip()
        return text or None


class VehicleRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    vehicle: str

    @field_validator("vehicle")
    @classmethod
    def nonempty(cls, value: str) -> str:
        text = value.strip()
        if not text:
            raise ValueError("vehicle must be non-empty text")
        return text


class ModelsRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    # Partial updates are merged over the current selection.
    models: dict[str, str]

    @field_validator("models")
    @classmethod
    def nonempty(cls, value: dict[str, str]) -> dict[str, str]:
        cleaned = {}
        for vehicle, model_id in value.items():
            key = str(vehicle).strip()
            chosen = str(model_id).strip()
            if not key or not chosen:
                raise ValueError("models must name non-empty vehicles and ids")
            cleaned[key] = chosen
        if not cleaned:
            raise ValueError("models must name at least one vehicle")
        return cleaned
