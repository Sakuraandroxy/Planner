"""Which map the live simulator stack actually loaded.

One process writes the marker, others read it: the driver writes it when a launch
passes its probes, the router reads it to tell which family is running, and the HTTP
bootstrap reads it before anything starts so it does not pick a scene JSONC that
does not match the window on screen. Both halves live here so the file format has
exactly one home.
"""

from __future__ import annotations

from pathlib import Path

from ..config import MapCatalog


def write_map_marker(path: str | Path, map_id: str) -> None:
    """Record the map a stack just loaded, creating the runtime directory."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(map_id + "\n", encoding="utf-8")


def read_map_marker(path: str | Path, catalog: MapCatalog) -> str | None:
    """Return the marker's map id when it names a catalog entry.

    Anything else -- no file, a half-written file, an id dropped from
    ``config/maps.yaml`` -- reads as "unknown" rather than raising: a stale marker
    must not stop the server from booting.
    """
    try:
        value = Path(path).read_text(encoding="utf-8").strip()
    except OSError:
        return None
    return value if catalog.entry(value) is not None else None
