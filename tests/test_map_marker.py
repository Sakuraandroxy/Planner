"""The current-map marker: who writes it, who is allowed to read it back."""

from planner.config import MapCatalog, MapEntry, MapScene
from planner.simulator.marker import read_map_marker, write_map_marker

CATALOG = MapCatalog(
    default_map="city",
    maps=(
        MapEntry(id="city", label="City", simulator="projectairsim",
                 scene=MapScene(file="s.jsonc", config_path="config/scenes/city")),
    ),
)


def test_round_trip(tmp_path):
    marker = tmp_path / "output" / "xpra" / "current_map"
    write_map_marker(marker, "city")
    assert marker.read_text(encoding="utf-8") == "city\n"
    assert read_map_marker(marker, CATALOG) == "city"


def test_only_catalog_ids_are_believed(tmp_path):
    """A marker naming a map that left the catalog reads as unknown, not as a map.

    The boot sequence uses this to pick the scene JSONC that matches the window on
    screen; trusting a stale id would load spawn coordinates the binary does not
    have.
    """
    assert read_map_marker(tmp_path / "absent", CATALOG) is None
    write_map_marker(tmp_path / "marker", "suburb")
    assert read_map_marker(tmp_path / "marker", CATALOG) is None
    write_map_marker(tmp_path / "marker", "city")
    assert read_map_marker(tmp_path / "marker", CATALOG) == "city"


def test_a_directory_instead_of_a_file_is_unknown(tmp_path):
    """A wrong ``map_marker_file`` must not take the boot sequence down."""
    (tmp_path / "current_map").mkdir()
    assert read_map_marker(tmp_path / "current_map", CATALOG) is None
