"""Render a simulator's launch argv from its template.

Both families come through here, which is what "unified xpra startup" means in
practice: one code path builds the command, and the difference between AirSim
and ProjectAirSim is the template text in ``config/simulators.yaml``.

Placeholders are ``{name}`` or ``{map.name}``. Anything unresolved is an error
rather than a literal left in the argv, because a stray ``{binary}`` reaches
``xpra`` as an argument and shows up much later as a window that never appears.
"""

from __future__ import annotations

import re
from typing import Mapping

TOKEN = re.compile(r"\{([a-z_][a-z0-9_]*(?:\.[a-z_][a-z0-9_]*)?)\}")


def render(part: str, values: Mapping[str, object]) -> str:
    """Substitute one argv entry. Unknown or empty placeholders raise."""
    missing: list[str] = []

    def replace(match: re.Match[str]) -> str:
        key = match.group(1)
        if key not in values:
            missing.append(key)
            return match.group(0)
        value = values[key]
        if value is None or (isinstance(value, str) and not value.strip()):
            raise ValueError(f"argv placeholder '{key}' resolved to nothing")
        return str(value)

    text = TOKEN.sub(replace, part)
    if missing:
        raise ValueError(
            f"argv entry '{part}' needs values this build does not provide: "
            f"{sorted(set(missing))}"
        )
    return text


def render_argv(
    template: tuple[str, ...] | list[str], values: Mapping[str, object]
) -> list[str]:
    return [render(part, values) for part in template]


def argv_values(
    *, entry, simulator, settings, runtime_dir: str, extra: Mapping[str, object] | None = None
) -> dict[str, object]:
    """Build the placeholder map for one launch.

    ``map.*`` comes from the catalog entry, so a per-map binary or scene name is
    always visible in the template. Ports come from the simulator's own block,
    which is why adding a simulator cannot make another one resolve a wrong port.
    """
    values: dict[str, object] = {
        "display": settings.display,
        "display_number": str(settings.display).lstrip(":"),
        "runtime_dir": runtime_dir,
        "launch_script": simulator.launch_script,
        "stop_script": simulator.stop_script,
        "settings_file": simulator.settings_file,
        "res_x": simulator.res_x,
        "res_y": simulator.res_y,
        "abslog": f"{runtime_dir}/logs/{entry.id}.log",
        "map.map_name": entry.map_name,
        "map.launch_map": entry.launch_map,
        "map.id": entry.id,
        "map.binary": entry.binary,
        "map.scene_file": entry.scene_file,
        "map.sim_config_path": entry.sim_config_path,
    }
    for name, port in simulator.ports.items():
        values[f"ports.{name}"] = port
        values[name] = port
    if extra:
        values.update(extra)
    return values
